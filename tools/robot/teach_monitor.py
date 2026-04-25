import woanarm_api_py as woan
import time, sys, tty, termios, asyncio, threading, json, logging
import numpy as np
from scipy.spatial.transform import Rotation
import websockets

# 浏览器误访问 http://host:8765/ 会让 websockets 抛一堆 InvalidUpgrade
# 的 traceback，看着吓人其实没坏。把这条日志降到 INFO 以下就干净了。
logging.getLogger("websockets.server").setLevel(logging.CRITICAL)
logging.getLogger("websockets.protocol").setLevel(logging.CRITICAL)
logging.getLogger("websockets").setLevel(logging.WARNING)

# ── 机械臂初始化 ──────────────────────────────────────────────
config = woan.WoanConfig()
config.device = "/dev/ttyACM0"
config.robot_model = "a1_s"
config.version = "A1"
config.model_description_path = "/home/huan/ArmApi/woan_description"

arm = woan.WoanArm(config)
arm.enable_motors()
# arm.movej([0.0]*7, speed_scale=0.2)
time.sleep(5)

POS_STEP = 0.02
ROT_STEP = 0.1

# 状态统计（print_status 里显示用）
phone_frame_count = 0
phone_rot_count   = 0

# ── 手机 AR 联动参数 ──────────────────────────────────────────
# 看门狗：没收到帧超过这个时间就停机。MediaPipe Pose 在手机端
# 稳态约 15 FPS（~66ms/帧），1.0s 留 ~15 帧余量 —— 之前 0.5s 太紧，
# 用户刚转个身脱离镜头就触发停机，用摇杆控制完全被切断。
PHONE_MAX_GAP_S        = 1.0
# 单帧手腕跳变超过这个距离（米，user 世界坐标）就停机，防止检测
# 在多人进镜/丢失重捕时把 TCP 瞬间打到别处。
PHONE_MAX_WRIST_JUMP_M = 0.20
# user 空间 → robot 空间的增益。手机端发的是 poseWorldLandmarks（米，
# 以髋中心为原点），du = 右腕 − 右肩，范围大致 ±0.6m。1:1 基本把机器
# 人的可达工作空间用满，想小范围试运动先把这里调到 0.5。
PHONE_GAIN             = 1.0
# 右臂 a1_s 安装方向 → 映射用户右臂到机器人 TCP。Rough convention（用户
# 面对镜头，MediaPipe world：+x 向用户右、+y 向下、+z 向远离相机）：
#   user 右  (du_x>0)  → 机器人 Y 负（靠身体一侧）
#   user 上  (du_y<0)  → 机器人 Z 正
#   user 前伸 (du_z<0) → 机器人 X 正
# 真实轴向肯定要实机标定，先给方向符号，部署后调 PHONE_AXIS_* 和 GAIN。
PHONE_AXIS_X_SRC, PHONE_AXIS_X_SIGN = 'z', -1.0   # robot Δx ← −user Δz
PHONE_AXIS_Y_SRC, PHONE_AXIS_Y_SIGN = 'x', -1.0   # robot Δy ← −user Δx
PHONE_AXIS_Z_SRC, PHONE_AXIS_Z_SIGN = 'y', -1.0   # robot Δz ← −user Δy
# 目标位姿相对 home 的三轴限幅（米），以防标定错了把机器人往外甩。
PHONE_CLAMP = (0.25, 0.25, 0.25)
# 执行器消费目标的最大频率（movel 不能比它快，否则前一个还没到就被覆盖）。
PHONE_EXEC_HZ   = 20
PHONE_MOVEL_SPEED = 0.4
# 7-DOF "ready" 关节序列 —— 按 i 键 / phone_ready 命令 / phone_start
# 都会把机械臂送到这里。每个关节都略弯，确保不在奇点，姿态 / 手臂
# 模式开启后 IK 才有解。值是经验性安全选择，速度 0.2 慢速过渡。
READY_JOINTS = [0.0, 0.4, 0.0, -0.8, 0.0, 0.4, 0.0]

# 共享状态
tx, ty, tz = -0.3060, 0.0800, 0.3163
rot = Rotation.from_quat([0.510, 0.511, -0.490, -0.489])
state_lock = threading.Lock()
ws_clients = set()

# ── 工具函数 ──────────────────────────────────────────────────
def get_state_dict():
    with state_lock:
        rpy = rot.as_euler('xyz', degrees=True)
        q = rot.as_quat()
        return {
            "x": round(tx, 4), "y": round(ty, 4), "z": round(tz, 4),
            "roll": round(rpy[0], 2), "pitch": round(rpy[1], 2), "yaw": round(rpy[2], 2),
            "qw": round(q[3], 4), "qx": round(q[0], 4), "qy": round(q[1], 4), "qz": round(q[2], 4),
        }

def _ws_summary():
    """
    一行总结 WebSocket / 手机联动状态：
      [WS=N] phone=ON/off frames=M rot=K
    颜色：活跃（至少一个 client + phone_enabled）用绿，有 client 未授权用黄，无 client 用灰。
    """
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    GREY = "\033[90m"
    RED = "\033[91m"
    RESET = "\033[0m"
    BOLD = "\033[1m"
    n = len(ws_clients)
    if n == 0:
        tag = f"{GREY}WS=0 (无手机){RESET}"
    elif phone_enabled:
        tag = f"{GREEN}{BOLD}WS={n} phone=ON{RESET}{GREEN} frames={phone_frame_count} rot={phone_rot_count}{RESET}"
    else:
        tag = f"{YELLOW}WS={n} phone=off{RESET}"
    return tag

def print_status():
    s = get_state_dict()
    # 清掉整行再重写，防止前次 tag 变短时残留字符（带颜色码长度不可见）
    print("\r\033[2K" + _ws_summary() + "  │  "
          f"X={s['x']:7.4f}  Y={s['y']:7.4f}  Z={s['z']:7.4f}  "
          f"R={s['roll']:6.1f}°  P={s['pitch']:6.1f}°  Y={s['yaw']:6.1f}°  ",
          end="", flush=True)

def _event_log(msg):
    """
    在状态行"之上"打一行事件日志 —— 连接/断开/急停等，这些需要留痕。
    做法：先换行把当前状态行"顶上去"，再重新画状态行。
    """
    print(f"\r\033[2K{msg}")
    print_status()

def broadcast_state():
    msg = json.dumps(get_state_dict())
    for ws in list(ws_clients):
        asyncio.run_coroutine_threadsafe(ws.send(msg), ws_loop)

def do_move(dx=0, dy=0, dz=0, drot=None):
    global tx, ty, tz, rot
    with state_lock:
        nx, ny, nz = tx+dx, ty+dy, tz+dz
        nrot = drot * rot if drot else rot
    p = woan.Pose()
    p.x, p.y, p.z = nx, ny, nz
    q = nrot.as_quat()
    p.qw, p.qx, p.qy, p.qz = q[3], q[0], q[1], q[2]
    ret = arm.movel(p, speed_scale=0.3)
    if ret == 0:
        with state_lock:
            tx, ty, tz = nx, ny, nz
            rot = nrot
        broadcast_state()
    else:
        print(f"\n  IK失败({ret})", end="")
    print_status()

# ── 手机 AR 联动：状态 + 执行器 + 看门狗 ──────────────────────
phone_enabled       = False
phone_home          = None                 # (x,y,z) 机器人本地原点
phone_home_rot      = None                 # Rotation，启动时的 TCP 姿态（roll/pitch/yaw 的增量都叠在它上面）
phone_rot_delta     = Rotation.identity()  # 手机端摇杆积累的姿态增量
phone_last_ts       = 0.0                  # 最近一帧的墙钟时间
phone_last_wrist    = None                 # 上一帧用户右腕 (wx, wy, wz)
phone_last_user_d   = (0.0, 0.0, 0.0)      # 上一帧 user Δ = wrist − shoulder（摇杆可单独推送位姿更新时复用）
phone_target_pose   = None                 # 待执行 Pose（由执行器线程消费）
phone_lock          = threading.Lock()
# 用户显式急停后锁死，只有 phone_start 能解锁 —— 区别于看门狗 / 跳变
# 这种瞬态 auto-stop（那种下次推摇杆会自动恢复）。
phone_estop_latched = False
# 摇杆单次推送最大角度（弧度）。手机端发 droll/dpitch/dyaw 时预期都在 ±0.1 内。
PHONE_ROT_CLAMP = 0.3
# 急停处理方式：'soft' = 只停 phone 跟随; 'hard' = 停跟随 + 失能电机
# 硬急停后机械臂会瘫（如果有负载或处于非原点姿态会掉），要重启脚本才能恢复。
PHONE_ESTOP_MODE = 'soft'

def _pick_axis(vec, src, sign):
    """vec = (x,y,z)，按 src ∈ {'x','y','z'} 取出一个分量并乘 sign。"""
    return sign * vec[{'x':0, 'y':1, 'z':2}[src]]

def phone_stop(reason=""):
    """统一停机入口。不关机器人电机，只是停止消费手机帧。"""
    global phone_enabled, phone_last_wrist, phone_target_pose
    with phone_lock:
        was_enabled = phone_enabled
        phone_enabled = False
        phone_last_wrist = None
        phone_target_pose = None
    if was_enabled:
        _event_log(f"\033[91m[phone] ■ STOP\033[0m  reason: {reason}")

def phone_executor():
    """
    单线程消费最新目标位姿 —— 手机端 15 FPS 发帧，movel 单次 >50ms，
    如果像 do_move 那样来一帧起一个线程，执行会叠起来，机器人会追
    旧命令。这里只保留"最新一个"目标，保证实时跟随。
    """
    global phone_target_pose, tx, ty, tz, rot
    period = 1.0 / PHONE_EXEC_HZ
    consec_failures = 0
    last_logged_ret = None
    exec_count = 0
    while True:
        time.sleep(period)
        if not phone_enabled:
            continue
        with phone_lock:
            target = phone_target_pose
            phone_target_pose = None
        if target is None:
            continue
        exec_count += 1
        ret = arm.movel(target, speed_scale=PHONE_MOVEL_SPEED)
        if ret == 0:
            if consec_failures > 0:
                _event_log(f"\033[92m[phone] ✓ movel OK\033[0m  "
                           f"之前连续失败 {consec_failures} 次，恢复")
            consec_failures = 0
            with state_lock:
                tx, ty, tz = target.x, target.y, target.z
                rot = Rotation.from_quat([target.qx, target.qy, target.qz, target.qw])
            broadcast_state()
        else:
            consec_failures += 1
            # 前 3 次 + 每 30 次打一条：说明 IK 在失败，target 可能越界 /
            # 奇点 / 和当前姿态差太远。
            if consec_failures <= 3 or consec_failures % 30 == 0:
                if ret != last_logged_ret or consec_failures <= 3:
                    _event_log(
                        f"\033[91m[phone] ✗ movel 失败\033[0m  ret={ret}  "
                        f"target=({target.x:+.3f}, {target.y:+.3f}, {target.z:+.3f})  "
                        f"第 {consec_failures} 次"
                    )
                    last_logged_ret = ret
            # 持续失败 30 次 → 几乎确定卡在奇点 / 工作空间边缘。给个
            # 明确指引，避免用户对着没反应的机器人发呆。
            if consec_failures == 30:
                _event_log(
                    f"\033[93m[phone] ⚠ IK 持续失败\033[0m  机械臂可能在奇点 / 工作空间边缘 — "
                    f"用键盘 ↑↓←→/wsd 把它移到一个舒展的位姿，再重新开 手臂/姿态"
                )
        # 第一次执行时打印一条信息帮助调试
        if exec_count == 1:
            _event_log(
                f"\033[96m[phone] ► first movel\033[0m  "
                f"target=({target.x:+.3f}, {target.y:+.3f}, {target.z:+.3f})  ret={ret}"
            )

def phone_watchdog():
    """丢帧超时看门狗。"""
    while True:
        time.sleep(0.1)
        if phone_enabled and (time.time() - phone_last_ts) > PHONE_MAX_GAP_S:
            phone_stop(f"no frame for {PHONE_MAX_GAP_S}s (landmarks lost)")

def _build_target_from_user_delta(du):
    """
    把 user Δ (wx−sx, wy−sy, wz−sz) 组合成 Pose：
      平移 = home + 映射后的 robot Δ（限幅）
      姿态 = home_rot ∘ phone_rot_delta（摇杆累积的增量）
    返回 woan.Pose 或 None（如果 home 没准备好）。
    """
    if phone_home is None or phone_home_rot is None:
        return None
    dx = _pick_axis(du, PHONE_AXIS_X_SRC, PHONE_AXIS_X_SIGN) * PHONE_GAIN
    dy = _pick_axis(du, PHONE_AXIS_Y_SRC, PHONE_AXIS_Y_SIGN) * PHONE_GAIN
    dz = _pick_axis(du, PHONE_AXIS_Z_SRC, PHONE_AXIS_Z_SIGN) * PHONE_GAIN
    cx, cy, cz = PHONE_CLAMP
    dx = max(-cx, min(cx, dx))
    dy = max(-cy, min(cy, dy))
    dz = max(-cz, min(cz, dz))
    hx, hy, hz = phone_home
    target = woan.Pose()
    target.x, target.y, target.z = hx + dx, hy + dy, hz + dz
    # 姿态 = 启动时姿态 ∘ 摇杆累积增量（都在 base/world frame 下左乘）
    combined = phone_rot_delta * phone_home_rot
    q = combined.as_quat()
    target.qw, target.qx, target.qy, target.qz = q[3], q[0], q[1], q[2]
    return target

def phone_handle_frame(data):
    """
    手机 AR 帧的入口。载荷：
      sx, sy, sz  右肩 world-landmark（米，以髋中心为原点）
      wx, wy, wz  右腕 world-landmark
      vis         右腕 visibility（0..1）
      ts          手机端时间戳（ms，仅 log 用）

    处理：
      1) 跳变（突然"很多点位"/多人干扰）→ 停机
      2) user Δ = wrist − shoulder，映射到 robot Δ
      3) 姿态叠加摇杆累积增量
      4) 写入 phone_target_pose，由 phone_executor 消费
    """
    global phone_last_ts, phone_last_wrist, phone_target_pose, phone_last_user_d
    global phone_frame_count, phone_enabled
    if phone_home is None or phone_estop_latched:
        return
    if not phone_enabled:
        # 看门狗 / 跳变导致的瞬态 stop，人重新出现在画面里就恢复跟随。
        phone_enabled = True
        _event_log(f"\033[93m[phone] ▶ RESUME\033[0m  (arm frame 触发)")

    try:
        sx, sy, sz = float(data['sx']), float(data['sy']), float(data['sz'])
        wx, wy, wz = float(data['wx']), float(data['wy']), float(data['wz'])
    except (KeyError, TypeError, ValueError):
        return
    phone_frame_count += 1

    # 1) 跳变检测
    if phone_last_wrist is not None:
        lwx, lwy, lwz = phone_last_wrist
        d = ((wx-lwx)**2 + (wy-lwy)**2 + (wz-lwz)**2) ** 0.5
        if d > PHONE_MAX_WRIST_JUMP_M:
            phone_stop(f"wrist jumped {d*100:.1f}cm in 1 frame")
            return

    du = (wx - sx, wy - sy, wz - sz)
    target = _build_target_from_user_delta(du)
    if target is None:
        return
    with phone_lock:
        phone_target_pose = target
    phone_last_ts = time.time()
    phone_last_wrist = (wx, wy, wz)
    phone_last_user_d = du

def phone_set_orient(droll=0.0, dpitch=0.0, dyaw=0.0):
    """
    传感器姿态模式 —— 直接覆盖 phone_rot_delta（绝对值，相对手机
    传感器开启时的初始姿态）。

    注意：clamp 到 ±0.5 rad (~28°)。机械臂在很多位姿下可达姿态范围
    都很窄（特别是伸到极限位置时），手机倾 90° 时让目标姿态够不到，
    IK 全部失败。手机倾角 -π/2..π/2 会被映射到 robot ±28°，UI 上看
    起来像是"收敛比"，但能保证 IK 一直找得到解。
    """
    global phone_rot_delta, phone_target_pose, phone_last_ts, phone_enabled
    if phone_estop_latched:
        return
    if not phone_enabled:
        if phone_home is None:
            return
        phone_enabled = True
        _event_log(f"\033[93m[phone] ▶ RESUME\033[0m  (sensor 触发)")
    SENSOR_CLAMP = 0.5  # ~28° 任一轴上限
    droll  = max(-SENSOR_CLAMP, min(SENSOR_CLAMP, float(droll)))
    dpitch = max(-SENSOR_CLAMP, min(SENSOR_CLAMP, float(dpitch)))
    dyaw   = max(-SENSOR_CLAMP, min(SENSOR_CLAMP, float(dyaw)))
    phone_rot_delta = Rotation.from_euler('xyz', [droll, dpitch, dyaw])
    target = _build_target_from_user_delta(phone_last_user_d)
    if target is not None:
        with phone_lock:
            phone_target_pose = target
        phone_last_ts = time.time()

def phone_apply_rot(droll=0.0, dpitch=0.0, dyaw=0.0):
    """
    摇杆每 tick 调一次，单次 droll/dpitch/dyaw 预期在 ±0.05 rad。把增量
    累加到 phone_rot_delta 上，然后用最近一次的 user Δ 重算 target，这
    样即使手机没在推新 pose 帧（例如人静止），摇杆动也能让机器人转头。

    姿态组合顺序：new = Rotation.from_euler('xyz', [dR, dP, dY]) ∘ old，
    意味着增量是在 base frame 里做的（和 teach_monitor 原本的键盘旋转
    一致 —— r/f/t/g/y/h 就是 base-frame 左乘）。

    自动复活：如果 session 被看门狗 / 跳变之类的瞬态条件 stop 过，但
    用户没按急停（phone_estop_latched=False），摇杆推一下就重新开起来。
    这样 MediaPipe 间歇检不到人时摇杆依然能用。
    """
    global phone_rot_delta, phone_target_pose, phone_last_ts, phone_rot_count
    global phone_enabled
    if phone_estop_latched:
        return
    if not phone_enabled:
        if phone_home is None:
            return  # 还没 phone_start 过，无法复活
        phone_enabled = True
        _event_log(f"\033[93m[phone] ▶ RESUME\033[0m  (joystick 触发)")
    phone_rot_count += 1
    # 限幅：单次摇杆 tick 别超过 0.3 rad（约 17°），防止 packet 大小异常
    # 或前端误发时机器人瞬间大幅翻转。
    droll  = max(-PHONE_ROT_CLAMP, min(PHONE_ROT_CLAMP, float(droll)))
    dpitch = max(-PHONE_ROT_CLAMP, min(PHONE_ROT_CLAMP, float(dpitch)))
    dyaw   = max(-PHONE_ROT_CLAMP, min(PHONE_ROT_CLAMP, float(dyaw)))
    if droll == 0 and dpitch == 0 and dyaw == 0:
        return
    inc = Rotation.from_euler('xyz', [droll, dpitch, dyaw])
    phone_rot_delta = inc * phone_rot_delta
    # 重算 target —— 用最近的 user Δ；刚 start 还没帧就用全 0。
    target = _build_target_from_user_delta(phone_last_user_d)
    if target is not None:
        with phone_lock:
            phone_target_pose = target
        phone_last_ts = time.time()

def phone_estop(hard=False):
    """
    急停。默认 soft：停跟随 + 清队列，并且**锁死会话**（latch）——
    只有 phone_start 才能再次启用。和看门狗/跳变那种瞬态 stop 不同
    （瞬态的摇杆推一下就自动复活）。
    hard=True 时额外 disable_motors —— 紧急情况下切电，机械臂会失去
    支撑力，注意防掉落。
    """
    global phone_estop_latched
    phone_estop_latched = True
    phone_stop(f"ESTOP ({'hard' if hard else 'soft'})")
    if hard:
        try:
            arm.disable_motors()
            print("[phone] motors disabled — 需要重启脚本恢复")
        except Exception as e:
            print(f"[phone] disable_motors err: {e}")

def move_to_ready():
    """
    把机械臂 movej 到 READY_JOINTS（一个略微弯曲的非奇点位姿）。
    完成后刷新 tx/ty/tz/rot 缓存，使后续 phone_start 读到的 home
    就是这个舒展位姿，而不是开机时的零位收缩态。
    """
    global tx, ty, tz, rot
    _event_log(f"\033[96m[ready] ► movej → {READY_JOINTS}\033[0m  (慢速 0.2)")
    ret = arm.movej(READY_JOINTS, speed_scale=0.2)
    if ret != 0:
        _event_log(f"\033[91m[ready] movej 返回 {ret} —— 失败\033[0m")
        return False
    time.sleep(2)  # 等动作完成
    try:
        p = arm.get_end_effector_pose()
        with state_lock:
            tx, ty, tz = p.x, p.y, p.z
            rot = Rotation.from_quat([p.qx, p.qy, p.qz, p.qw])
        _event_log(
            f"\033[92m[ready] ✓ 抵达 ready 位姿\033[0m  "
            f"TCP=({tx:+.3f}, {ty:+.3f}, {tz:+.3f})"
        )
        return True
    except Exception as e:
        _event_log(f"\033[91m[ready] get_end_effector_pose 失败: {e}\033[0m")
        return False

def phone_start(origin_from_current=True):
    """激活手机跟随，把当前 TCP 位置 + 姿态都记为 home。姿态增量重置。
    也会清除急停 latch，所以按急停后通过重连 / 手动再开手臂就能解锁。"""
    global phone_enabled, phone_home, phone_home_rot, phone_rot_delta
    global phone_last_ts, phone_last_wrist, phone_target_pose, phone_last_user_d
    global phone_frame_count, phone_rot_count, phone_estop_latched
    global tx, ty, tz, rot
    phone_estop_latched = False
    # 关键：直接从机器人读真实 TCP 位姿，不信任脚本里的 tx/ty/tz
    # 缓存（那个值只有在显式 movel/movej 成功后才会更新）。启动时
    # 如果机器人处于任意姿态（比如之前被手推过），缓存值就是错的，
    # 第一个 movel 就把机器人"扯"到脚本初始化的那个点 → 多半 IK 失败。
    try:
        real_pose = arm.get_end_effector_pose()
        with state_lock:
            tx, ty, tz = real_pose.x, real_pose.y, real_pose.z
            rot = Rotation.from_quat([real_pose.qx, real_pose.qy, real_pose.qz, real_pose.qw])
            phone_home = (tx, ty, tz)
            phone_home_rot = Rotation.from_quat(rot.as_quat())
        _event_log(
            f"\033[96m[phone] sync'd real TCP pose\033[0m  "
            f"({tx:+.3f}, {ty:+.3f}, {tz:+.3f})"
        )
    except Exception as e:
        # 没读成功就回退到缓存值，至少有个起点
        print(f"\n[phone] get_end_effector_pose err: {e}, 回退到缓存 TCP pose")
        with state_lock:
            phone_home = (tx, ty, tz)
            phone_home_rot = Rotation.from_quat(rot.as_quat())
    phone_rot_delta = Rotation.identity()
    phone_last_wrist = None
    phone_target_pose = None
    phone_last_user_d = (0.0, 0.0, 0.0)
    phone_last_ts = time.time()
    phone_enabled = True
    phone_frame_count = 0
    phone_rot_count = 0
    _event_log(
        f"\033[92m[phone] ▶ START\033[0m  home=({phone_home[0]:+.3f}, "
        f"{phone_home[1]:+.3f}, {phone_home[2]:+.3f})"
    )

threading.Thread(target=phone_executor, daemon=True).start()
threading.Thread(target=phone_watchdog, daemon=True).start()

# ── WebSocket 服务 ────────────────────────────────────────────
# 接收命令格式（JSON）：
#   {"cmd": "move", "dx": 0.02, "dy": 0, "dz": 0}
#   {"cmd": "rotate", "axis": "x", "angle": 0.1}
#   {"cmd": "get_state"}
#   {"cmd": "phone_start"}                         激活手机 AR 跟随
#   {"cmd": "phone_stop"}                          停止跟随
#   {"cmd": "phone_frame", "sx":..,"sy":..,"sz":.., 手机 AR 每帧推送的
#       "wx":..,"wy":..,"wz":.., "vis":.., "ts":..}  右肩/右腕 world-landmark
async def ws_handler(websocket):
    # 客户端连上/断开都打一行到事件区，方便从终端直接看到"有手机来了"。
    peer = ""
    try:
        remote = websocket.remote_address
        peer = f"{remote[0]}:{remote[1]}" if remote else "?"
    except Exception:
        peer = "?"
    ws_clients.add(websocket)
    _event_log(f"\033[96m[ws]  + CONNECT\033[0m  {peer}  (总 {len(ws_clients)} 客户端)")
    await websocket.send(json.dumps(get_state_dict()))
    try:
        async for msg in websocket:
            try:
                data = json.loads(msg)
                cmd = data.get("cmd")
                if cmd == "get_state":
                    await websocket.send(json.dumps(get_state_dict()))
                elif cmd == "move":
                    threading.Thread(target=do_move, kwargs={
                        "dx": data.get("dx", 0),
                        "dy": data.get("dy", 0),
                        "dz": data.get("dz", 0),
                    }, daemon=True).start()
                elif cmd == "rotate":
                    axis = data.get("axis", "z")
                    angle = data.get("angle", ROT_STEP)
                    threading.Thread(target=do_move, kwargs={
                        "drot": Rotation.from_euler(axis, angle)
                    }, daemon=True).start()
                elif cmd == "phone_start":
                    phone_start()
                    await websocket.send(json.dumps({"phone": "enabled",
                        "home": list(phone_home) if phone_home else None}))
                elif cmd == "phone_stop":
                    phone_stop("requested by client")
                    await websocket.send(json.dumps({"phone": "disabled"}))
                elif cmd == "phone_frame":
                    # Hot path — 15 FPS 从手机过来，不要在这里起线程，
                    # 直接在 event loop 里写共享变量由 phone_executor 消费。
                    phone_handle_frame(data)
                elif cmd == "phone_rot":
                    # 摇杆每 tick 发一次 droll/dpitch/dyaw（base frame，rad）
                    phone_apply_rot(
                        droll  = data.get("droll", 0.0),
                        dpitch = data.get("dpitch", 0.0),
                        dyaw   = data.get("dyaw", 0.0),
                    )
                elif cmd == "phone_orient":
                    # 传感器模式：手机倾角的绝对 delta（相对开启时的零位）
                    phone_set_orient(
                        droll  = data.get("droll", 0.0),
                        dpitch = data.get("dpitch", 0.0),
                        dyaw   = data.get("dyaw", 0.0),
                    )
                elif cmd == "estop":
                    phone_estop(hard = bool(data.get("hard", False))
                                       or PHONE_ESTOP_MODE == 'hard')
                    await websocket.send(json.dumps({"phone": "estop"}))
                elif cmd == "phone_ready":
                    # 同步阻塞执行 movej —— 完成后回 ack。客户端可以用
                    # 这个在打开 姿态/手臂 之前先把机械臂"展开"。
                    threading.Thread(target=move_to_ready, daemon=True).start()
                    await websocket.send(json.dumps({"phone": "ready_started"}))
                elif cmd == "moveto":
                    # 直接指定目标位姿
                    global tx, ty, tz, rot
                    p = woan.Pose()
                    p.x = data.get("x", tx)
                    p.y = data.get("y", ty)
                    p.z = data.get("z", tz)
                    p.qw = data.get("qw", rot.as_quat()[3])
                    p.qx = data.get("qx", rot.as_quat()[0])
                    p.qy = data.get("qy", rot.as_quat()[1])
                    p.qz = data.get("qz", rot.as_quat()[2])
                    def _moveto():
                        global tx, ty, tz, rot
                        ret = arm.movel(p, speed_scale=data.get("speed", 0.3))
                        if ret == 0:
                            with state_lock:
                                tx, ty, tz = p.x, p.y, p.z
                                rot = Rotation.from_quat([p.qx, p.qy, p.qz, p.qw])
                            broadcast_state()
                    threading.Thread(target=_moveto, daemon=True).start()
            except Exception as e:
                await websocket.send(json.dumps({"error": str(e)}))
    finally:
        ws_clients.discard(websocket)
        # 如果断开的是当前在控制的那个 phone session，顺手停一下跟随
        # 避免断线后机器人还在消耗上一帧。
        if phone_enabled and len(ws_clients) == 0:
            phone_stop("client disconnected, no active ws")
        _event_log(f"\033[96m[ws]  - DISCONN\033[0m  {peer}  (剩 {len(ws_clients)} 客户端)")

ws_loop = asyncio.new_event_loop()

def start_ws_server():
    asyncio.set_event_loop(ws_loop)
    async def _run():
        async with websockets.serve(ws_handler, "0.0.0.0", 8765):
            await asyncio.Future()
    ws_loop.run_until_complete(_run())

ws_thread = threading.Thread(target=start_ws_server, daemon=True)
ws_thread.start()

# ── 键盘控制 ──────────────────────────────────────────────────
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            return ch + ch2
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# 移动到初始位姿
p0 = woan.Pose()
p0.x, p0.y, p0.z = tx, ty, tz
q0 = rot.as_quat()
p0.qw, p0.qx, p0.qy, p0.qz = q0[3], q0[0], q0[1], q0[2]
# arm.movel(p0, speed_scale=0.2)
time.sleep(5)

print("\033[1m\033[96m╔══════════════════════════════════════════════════════════════╗\033[0m")
print("\033[1m\033[96m║  WoanArm teach monitor — WebSocket on ws://0.0.0.0:8765     ║\033[0m")
print("\033[1m\033[96m╚══════════════════════════════════════════════════════════════╝\033[0m")
print("键盘控制：")
print("  位置（2cm）：↑/↓=X  ←/→=Y  w/s=Z")
print("  姿态（~6°）：r/f=绕X  t/g=绕Y  y/h=绕Z")
print("  \033[1m\033[93mi  : Init — movej 到 ready 位姿（开手机模式之前先按一次！）\033[0m")
print("  q : 退出")
print("\n\033[90m底部状态行格式: [WS=客户端数 phone=ON/off frames=M rot=K]  X/Y/Z/R/P/Y\033[0m\n")
print_status()

try:
    while True:
        key = get_key()
        if   key == '\x1b[A': do_move(dx=+POS_STEP)
        elif key == '\x1b[B': do_move(dx=-POS_STEP)
        elif key == '\x1b[C': do_move(dy=+POS_STEP)
        elif key == '\x1b[D': do_move(dy=-POS_STEP)
        elif key in ('w','W'): do_move(dz=+POS_STEP)
        elif key in ('s','S'): do_move(dz=-POS_STEP)
        elif key in ('r','R'): do_move(drot=Rotation.from_euler('x', +ROT_STEP))
        elif key in ('f','F'): do_move(drot=Rotation.from_euler('x', -ROT_STEP))
        elif key in ('t','T'): do_move(drot=Rotation.from_euler('y', +ROT_STEP))
        elif key in ('g','G'): do_move(drot=Rotation.from_euler('y', -ROT_STEP))
        elif key in ('y','Y'): do_move(drot=Rotation.from_euler('z', +ROT_STEP))
        elif key in ('h','H'): do_move(drot=Rotation.from_euler('z', -ROT_STEP))
        elif key in ('i','I'): move_to_ready()
        elif key in ('q','Q'): break
except KeyboardInterrupt:
    pass
finally:
    print("\n退出，回零位...")
    arm.movej([0.0]*7, speed_scale=0.2)
    time.sleep(4)
    arm.disable_motors()
    print("电机已失能")
