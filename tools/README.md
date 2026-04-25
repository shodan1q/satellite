# tools/

辅助脚本，不打包进 app，但是项目完整工作流的一部分。

## `robot/teach_monitor.py`

跑在机械臂主机上（woan a1_s）。提供：
- 终端键盘控制（`↑↓←→`/`wsr/f/t/g/y/h`）
- WebSocket 服务（`ws://0.0.0.0:8765`）接受手机发来的 `phone_frame` /
  `phone_rot` / `phone_orient` / `estop` 等命令
- 看门狗（1.0s）+ 跳变检测（20cm/帧）+ 急停 latch 三层安全
- 实时彩色状态行 + 事件日志

依赖：`woanarm_api_py`（厂商 SDK） + `scipy` + `numpy` + `websockets`。

环境通常用 micromamba：
```bash
~/.local/bin/micromamba run -n woanarm python teach_monitor.py
```

启动后按 `i` 把机械臂送到非奇点 ready 位姿（关节
`[0, 0.4, 0, -0.8, 0, 0.4, 0]`），然后才能跑姿态/手臂模式。

## `vision/*.py`

Mac 端用 Insta360 X4（USB webcam 模式）做机械臂视觉感知：

- `aruco_generate.py` — 生成 ArUco 二维码 PNG（默认 4 张 ID 0~3）
- `arm_pose_tracker.py` — ArUco 标记 6-DOF 位姿追踪
- `arm_roi_tracker.py` — CSRT/KCF/MOSSE 框选区域追踪（无需打印标记）
- `arm_motion_detector.py` — MOG2 背景减除运动事件检测

依赖：`opencv-contrib-python`（含 trackers + ArUco）。建议 venv：
```bash
python3 -m venv .venv-cv
.venv-cv/bin/pip install opencv-contrib-python numpy
.venv-cv/bin/python arm_roi_tracker.py
```
