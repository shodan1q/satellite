# 喂！星 · Nebula Core

> AR 卫星工坊 + 全身姿态识别 + 实物机械臂联动 —— HarmonyOS 原生 app
>
> AttraX Spring Hackathon 提交

[![tag](https://img.shields.io/badge/AttraX-Spring_Hackathon-FF4D33)](#)
[![harmonyos](https://img.shields.io/badge/HarmonyOS-NEXT-blue)](#)
[![license](https://img.shields.io/badge/license-MIT-green)](#)

---

## 一句话说明

手机做"卫星总装工坊"，可以**用右臂 / 倾斜手机 / 双摇杆**实时控制一台 7 自由度真实机械臂，配套 AR 太空场景做交互教学。

## 主要特性

| 模块 | 描述 | 关键文件 |
|---|---|---|
| **AR 卫星工坊** | 摄像头透视 + three.js 渲染卫星模型，支持 12 类组件实时拼装、爆炸图、星轨背景（120MB Gaussian Splat 国际空间站场景） | `entry/src/main/resources/rawfile/three/ar.html` |
| **手势模式 (手势)** | MediaPipe Hands · 摄像头识别拇指+食指距离驱动卫星爆炸度，握拳 = 抓取拖动 | `entry/src/main/resources/rawfile/hand.html` |
| **手臂姿态模式 (手臂)** | MediaPipe Pose · 右臂世界坐标 → 远程机械臂 TCP 平移跟随 | `entry/src/main/resources/rawfile/arm.html` |
| **传感器姿态模式 (姿态)** | DeviceMotion 原生陀螺仪 → 机械臂末端 6-DOF 姿态绝对值映射 | `entry/src/main/ets/pages/Workshop.ets` `toggleSensor()` |
| **体感模式 (体感)** | 加速度计快速挥动手机 → 卫星爆炸度的瞬时控制 | `entry/src/main/ets/services/GesturePipeline.ets` |
| **机械臂控制后端** | 原生 `@ohos.net.webSocket` → Python `teach_monitor.py` → 7-DOF a1_r 机械臂 | `entry/src/main/ets/services/RobotClient.ets` `tools/robot/teach_monitor.py` |
| **多端协同** | DeviceManager 跨手机 / 平板 / 手表同步火箭设计与姿态状态 | `entry/src/main/ets/services/NebulaSync.ets` |
| **教学（科普 / 发现 / 周边 / 我的）** | 详尽的卫星科普内容、轨道演示、设备配对、用户档案 | `entry/src/main/ets/pages/{Academy,Discover,Device,Profile}.ets` |

## 真实机械臂联动 — 核心创新

整个项目的最大亮点是**用手机驱动一台实物 7-DOF 机械臂**。三种独立控制模式可同时启用：

```
                 ┌─────────────────────────────────────┐
                 │     HarmonyOS 手机 satellite app    │
                 ├─────────────────────────────────────┤
   📷 前摄像头 ──► MediaPipe Pose (arm.html)            │
                 │       │                             │
                 │       ▼  右肩+右腕 world landmarks   │
                 │  ArmJsBridge.onRobotCmd  ─┐         │
                 │                            ▼        │
   📐 陀螺仪    ──► DeviceMotion              │        │
                 │       │  pitch/roll/yaw    │        │
                 │       ▼                    ▼        │
                 │   sensor mode ──► forwardRobotCmd   │
                 │                       │             │
   🕹️ 双摇杆    ───────────────────────► │             │
                 │                       ▼             │
                 │           RobotClient (原生 WebSocket)
                 └────────────────────────┬────────────┘
                                          │  ws://<robot>:8765
                                          ▼
              ┌─────────────────────────────────────┐
              │    teach_monitor.py (机器人本机)     │
              ├─────────────────────────────────────┤
              │  WebSocket 服务（~15-20Hz)           │
              │     ▼                                │
              │  phone_executor → arm.movel(target) │
              │     • 看门狗 1.0s                    │
              │     • 跳变检测 20cm/帧               │
              │     • 急停 latch                     │
              └─────────────────────────────────────┘
                                          │
                                          ▼
                            🦾 woan a1_r 7-DOF 机械臂
```

### 数据流细节

1. **手臂模式**：手机摄像头里的右臂关键点 → world-meter 坐标 → `phone_frame` 命令 → 机械臂 TCP 跟随 (wrist − shoulder) 偏移
2. **姿态模式**：手机倾角相对开启时零位的差值 → `phone_orient` 命令 → 机械臂末端旋转
3. **摇杆**：左摇杆 dx→roll dy→pitch、右摇杆 dx→yaw → `phone_rot` 增量
4. **三层安全闸**：
   - 手机端：右臂检测丢失自动停发帧
   - 服务端：1s 看门狗
   - 服务端：手腕跳变 >20cm 立即停机
   - 顶部 **急停** 按钮：硬停 + 锁死，需重连解锁

## 技术栈

- **HarmonyOS NEXT** · ArkTS · ArkUI · ArkWeb
- **three.js** (rawfile/three/) — 程序化卫星几何 + Gaussian Splat 场景渲染
- **MediaPipe** Hands / Pose (lite/full) — 全部 client-side 推理
- **@ohos.net.webSocket** + **@ohos.net.connection** — 原生 WiFi 路由 + WS 通信
- **Python 3.12 + websockets + scipy** — 机器人侧控制服务
- **WoanArm SDK (`woanarm_api_py`)** — 机械臂运动学 + IK

## 项目结构

```
satellite/
├── AppScope/                              # 全局 app 资源（图标 / 名称）
├── entry/src/main/
│   ├── ets/
│   │   ├── components/                    # 14 个 UI 组件
│   │   │   ├── ArViewer.ets              # AR three.js 桥接
│   │   │   ├── JoystickPad.ets           # 双摇杆（已美化）
│   │   │   ├── Nebula{Button,Card,Icon,Pill}.ets  # 设计系统
│   │   │   └── …
│   │   ├── pages/                         # 5 大主页面 + 启动屏
│   │   │   ├── Workshop.ets              # 1900+ 行 — 整个项目核心
│   │   │   ├── Academy.ets               # 卫星科普
│   │   │   ├── Discover.ets              # 太空发现
│   │   │   ├── Device.ets                # 配对外设
│   │   │   └── Profile.ets               # 用户档案
│   │   ├── services/                      # 15 个底层服务
│   │   │   ├── RobotClient.ets           # ⭐ 原生 WS 到机械臂
│   │   │   ├── HandGesturePipeline.ets   # 手势事件分发
│   │   │   ├── DeviceMotion.ets          # 陀螺仪
│   │   │   ├── NebulaSync.ets            # 多端同步
│   │   │   └── …
│   │   └── data/Catalog.ets              # 12 个卫星 / 火箭 蓝图
│   └── resources/
│       ├── base/profile/network_config.json   # ⭐ 原生 ws:// LAN 白名单
│       └── rawfile/
│           ├── arm.html                   # ⭐ 右臂姿态识别 WebView
│           ├── hand.html                  # 手势识别 WebView
│           ├── three/
│           │   ├── ar.html               # ⭐ AR 渲染主入口（球形卫星本体在这）
│           │   ├── workshop.html         # 工作室预览模式
│           │   ├── nebula-three-bundle.js
│           │   └── scenes/iss.splat      # 120MB 国际空间站 GS 场景（Git LFS）
│           └── icons/                     # 33 个 Lucide SVG
└── tools/                                 # （建议入库）机器人后端 + Mac 视觉
    ├── robot/teach_monitor.py            # WS 服务端 + 键盘控制
    └── vision/                            # ArUco / ROI 追踪 / 运动检测
```

## 跑起来

### 手机 app

1. 用 DevEco Studio 打开 `satellite/`，签名后部署到 HarmonyOS NEXT 设备
2. 首次启动会请求摄像头 / 蓝牙 / 网络 / 加速度计权限 — 全部允许
3. 进入 **AR 工坊** 开始

### 机械臂后端（可选）

如果有 woan a1_r 实机：

```bash
# 在机器人本机
~/.local/bin/micromamba run -n woanarm python teach_monitor.py
# 会监听 ws://0.0.0.0:8765
```

**网络要求**：手机和机器人需在同一 LAN 下并能互通 TCP 8765。我们在
`network_config.json` 已为常见热点 IP（`172.20.10.x`、`192.168.43.x`）
开放 cleartext。注意 iPhone 个人热点 client-to-client TCP 默认隔离，
建议**用 HarmonyOS 手机做热点**，让机器人作为客户端连过来；如果机器
人在 WSL2 里跑，需 Windows 防火墙放行 TCP 8765。

### 修改机器人 IP

App 里长按 topBar 的 `ROBOT` 小圆点 → 弹出对话框输入新地址，立即重连，
新 URL 持久化到 WebView localStorage，下次启动免重输。

## ArUco 视觉追踪（实验性）

`tools/vision/` 提供 Mac 端使用 Insta360 X4 实时追踪机械臂位姿的工具：

- `aruco_generate.py` — 生成 ArUco 标记 PNG，打印贴机械臂
- `arm_pose_tracker.py` — 实时输出 6-DOF 末端相对底座的位姿
- `arm_motion_detector.py` — MOG2 背景减除运动检测
- `arm_roi_tracker.py` — CSRT 框选追踪（无需打印标记）

## 安全 · 注意事项

- **任何机器人控制改动都先小幅度测试** —— `tools/robot/teach_monitor.py`
  里 `PHONE_GAIN`、`PHONE_CLAMP`、`PHONE_MOVEL_SPEED` 三个常数控制运动量
- **机械臂启动时常处于"零位奇点"**：在 teach_monitor 终端按 `i` 让它先
  movej 到舒展 ready 位姿（`READY_JOINTS = [0, 0.4, 0, -0.8, 0, 0.4, 0]`）
- **急停优先级最高**：app 顶部红色 急停 按钮 → 锁死会话，硬停可硬切电机

## 已知限制

- iPhone 个人热点会拦客户端间 TCP（用 HarmonyOS 手机做热点解决）
- WSL2 + Windows 主机时 Windows Firewall 默认拦 8765（一行 PowerShell 放行）
- ArkWeb 不支持 cleartext WebSocket，所以机械臂 WS 走原生 `@ohos.net.webSocket`
- MediaPipe Pose lite 模型 visibility 在弱光下偏低（已放宽阈值）

## License

MIT

## 致谢

- **AttraX Spring Hackathon** 主办方
- **MediaPipe** by Google
- **three.js** community
- **WoanArm** SDK
- **OpenWonderLabs/ArmApi**
- Insta360 / 影石
- Lucide icons

---

#AttraX_Spring_Hackathon
