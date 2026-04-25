#!/usr/bin/env python3
"""
Arm Pose Tracker — 用 ArUco 标记追踪机械臂的 6-DOF 位姿。

工作流：
  1. 打印 aruco/ 里的标记，量准黑色方块的边长（米），贴到机械臂上
     默认约定：ID 0 = 底座（参考系），ID 1 = 末端
  2. 运行本脚本，看到 marker 时画面会叠 XYZ 坐标轴 + 实时位姿
  3. 终端持续打印末端相对底座的 (x, y, z, roll, pitch, yaw)

操作：
  c   切换"是否相对底座"显示（绝对相机坐标 vs 相对底座）
  l   开启/关闭终端日志（默认开）
  +/- 调整 marker 实际边长 ±5%（如果距离测得不准，临时校正）
  s   截图
  q   退出

依赖：cv2, numpy（已装在 .venv-cv）
跑：./.venv-cv/bin/python arm_pose_tracker.py --camera 1 --size 0.05
"""
import argparse
import math
import sys
import time
from datetime import datetime

import cv2
import cv2.aruco as aruco
import numpy as np


# ── 相机内参（粗估，X4 webcam 模式典型值）─────────────────────
# 实际想精确测姿态需要做 chessboard 校准。这里给一组"够用"的默认，
# 适合 1920×1080 的鱼眼输出（单镜头线性化模式）。X4 在 Insta360
# 控制中心切到 "Single Lens · Linear" 后输出接近这组参数。
def default_camera_matrix(width: int, height: int):
    # 视角约 100° 水平 → fx ≈ width / (2 tan(50°)) ≈ width * 0.42
    fx = width * 0.85
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    # 鱼眼输出已经线性化的话畸变近似为 0；仍留少量畸变项缓冲
    dist = np.array([0.05, -0.05, 0.0, 0.0, 0.0], dtype=np.float32)
    return K, dist


def rotmat_to_eulerXYZ(R):
    """3x3 旋转矩阵 → (roll, pitch, yaw) 度。XYZ 内禀。"""
    sy = -R[2, 0]
    cy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if cy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(sy, cy)
        yaw = 0
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def make_marker_object_points(size: float):
    """ArUco solvePnP 用的 3D 物体点（marker 4 个角，z=0 平面）。"""
    s = size / 2.0
    return np.array([
        [-s,  s, 0],
        [ s,  s, 0],
        [ s, -s, 0],
        [-s, -s, 0],
    ], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=None,
                    help="相机索引（不指定就交互式选）")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--size", type=float, default=0.05,
                    help="ArUco marker 实际打印边长（米），默认 5cm")
    ap.add_argument("--dict", type=str, default="DICT_4X4_50",
                    help="必须和 aruco_generate.py 用的字典一致")
    ap.add_argument("--base-id", type=int, default=0,
                    help="底座 marker ID，默认 0")
    ap.add_argument("--ee-id", type=int, default=1,
                    help="末端 marker ID，默认 1")
    args = ap.parse_args()

    if args.camera is None:
        # 重用 motion detector 的交互式选相机
        from arm_motion_detector import probe_cameras, pick_camera_interactive
        cams = probe_cameras()
        if not cams:
            print("[error] 没找到摄像头", file=sys.stderr)
            sys.exit(1)
        for idx, w, h in cams:
            print(f"  cam {idx}  {w}x{h}")
        chosen = pick_camera_interactive(cams)
        if chosen is None:
            sys.exit(0)
        args.camera = chosen

    cap = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"[error] 打不开 cam {args.camera}", file=sys.stderr)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[cap] {actual_w}x{actual_h}")

    K, dist = default_camera_matrix(actual_w, actual_h)
    print(f"[K] fx={K[0,0]:.0f} fy={K[1,1]:.0f} cx={K[0,2]:.0f} cy={K[1,2]:.0f}")
    print(f"[size] marker={args.size*100:.1f}cm")

    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    detector_params = aruco.DetectorParameters()
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 23
    detector_params.adaptiveThreshWinSizeStep = 4
    detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector = aruco.ArucoDetector(aruco_dict, detector_params)

    obj_pts = make_marker_object_points(args.size)
    relative_mode = True
    log_enabled = True
    last_log_t = 0.0
    LOG_INTERVAL = 0.2  # 5 Hz 控制台输出

    print(f"\n[ready] base ID={args.base_id}  end-effector ID={args.ee_id}")
    print(f"[mode] 相对底座 = ON  (按 c 切换到绝对相机坐标)\n")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        # 解算每个检出 marker 的位姿
        marker_poses = {}  # id -> (rvec, tvec)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                img_pts = corners[i].reshape(-1, 2).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if ok:
                    marker_poses[int(mid)] = (rvec, tvec)

        # 画检测结果
        out = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(out, corners, ids)
            for mid, (rvec, tvec) in marker_poses.items():
                cv2.drawFrameAxes(out, K, dist, rvec, tvec, args.size * 0.6)

        # 计算末端相对底座的位姿
        ee_pose_text = []
        if args.ee_id in marker_poses:
            rvec_ee, tvec_ee = marker_poses[args.ee_id]
            R_ee, _ = cv2.Rodrigues(rvec_ee)

            if relative_mode and args.base_id in marker_poses:
                rvec_base, tvec_base = marker_poses[args.base_id]
                R_base, _ = cv2.Rodrigues(rvec_base)
                # T_base_ee = T_base^-1 * T_ee
                R_rel = R_base.T @ R_ee
                t_rel = R_base.T @ (tvec_ee - tvec_base)
                pos = t_rel.flatten()
                roll, pitch, yaw = rotmat_to_eulerXYZ(R_rel)
                src = "相对底座"
            else:
                pos = tvec_ee.flatten()
                roll, pitch, yaw = rotmat_to_eulerXYZ(R_ee)
                src = "相对相机"

            ee_pose_text = [
                f"EE pose ({src}):",
                f"  X={pos[0]*100:+7.2f} cm",
                f"  Y={pos[1]*100:+7.2f} cm",
                f"  Z={pos[2]*100:+7.2f} cm",
                f"  R={roll:+6.1f}°",
                f"  P={pitch:+6.1f}°",
                f"  Y={yaw:+6.1f}°",
            ]

            # 5 Hz 终端输出
            now = time.time()
            if log_enabled and now - last_log_t >= LOG_INTERVAL:
                last_log_t = now
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{stamp}] {src}: "
                      f"X={pos[0]*100:+6.1f}cm Y={pos[1]*100:+6.1f}cm "
                      f"Z={pos[2]*100:+6.1f}cm R={roll:+5.1f}° "
                      f"P={pitch:+5.1f}° Y={yaw:+5.1f}°")

        # HUD
        h, w = out.shape[:2]
        bar_h = 32
        cv2.rectangle(out, (0, 0), (w, bar_h), (20, 20, 24), -1)
        n_detect = 0 if ids is None else len(ids)
        info = f"detected={n_detect}  base={args.base_id}@{'OK' if args.base_id in marker_poses else '--'}"
        info += f"  EE={args.ee_id}@{'OK' if args.ee_id in marker_poses else '--'}"
        info += f"  size={args.size*100:.1f}cm  log={'ON' if log_enabled else 'OFF'}"
        cv2.putText(out, info, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)

        # 左下浮窗显示末端 6-DOF
        if ee_pose_text:
            ty = h - 14 - 24 * len(ee_pose_text)
            cv2.rectangle(out, (8, ty - 8),
                          (260, h - 8), (15, 15, 18), -1)
            for line in ee_pose_text:
                cv2.putText(out, line, (16, ty + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (140, 230, 255) if "EE" in line else (220, 220, 220), 1)
                ty += 24

        cv2.putText(out, "[c] mode  [l] log  [+/-] size  [s] snap  [q] quit",
                    (10, h - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        cv2.imshow("Arm Pose Tracker", out)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('c'):
            relative_mode = not relative_mode
            print(f"[mode] 相对底座 = {'ON' if relative_mode else 'OFF'}")
        elif k == ord('l'):
            log_enabled = not log_enabled
            print(f"[log] {'ON' if log_enabled else 'OFF'}")
        elif k == ord('+') or k == ord('='):
            args.size *= 1.05
            obj_pts = make_marker_object_points(args.size)
            print(f"[size] {args.size*100:.2f}cm")
        elif k == ord('-') or k == ord('_'):
            args.size *= 0.95
            obj_pts = make_marker_object_points(args.size)
            print(f"[size] {args.size*100:.2f}cm")
        elif k == ord('s'):
            fname = f"pose_snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(fname, out)
            print(f"[snap] {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
