#!/usr/bin/env python3
"""
Arm ROI Tracker — 用户框选一个区域，OpenCV 追踪器持续跟随。

工作流：
  1. 启动后选相机（自动列出所有，按 y 选用）
  2. 主窗口出来后按 r 拖一个矩形框住要追踪的部位（末端 / 整条臂 / 工件…）
  3. 之后画面里这块区域被实时追踪，叠绿框 + 中心十字
  4. 终端持续打印 (cx, cy, w, h, confidence)

操作：
  r       重新选 ROI
  c       清除 ROI（停止追踪）
  t       切换追踪算法（CSRT 准 / KCF 快 / MOSSE 极快但糙）
  l       切换终端日志开关
  +/-     调整目标尺寸过滤（用于检测严重失败时的提示）
  s       截图
  space   暂停 / 继续
  q       退出

依赖：cv2（已装）
跑：./.venv-cv/bin/python arm_roi_tracker.py --camera N
"""
import argparse
import csv
import sys
import time
from datetime import datetime

import cv2
import numpy as np


TRACKER_FACTORIES = {
    "CSRT":  lambda: cv2.TrackerCSRT.create(),
    "KCF":   lambda: cv2.TrackerKCF.create(),
}
# MOSSE 在某些版本要 cv2.legacy；只有可用才注册。
try:
    cv2.legacy.TrackerMOSSE_create()
    TRACKER_FACTORIES["MOSSE"] = lambda: cv2.legacy.TrackerMOSSE_create()
except Exception:
    pass


def make_tracker(name: str):
    return TRACKER_FACTORIES[name]()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--algo", type=str, default="CSRT",
                    choices=list(TRACKER_FACTORIES.keys()),
                    help="初始追踪算法，运行中按 t 切换")
    ap.add_argument("--log-csv", type=str, default=None,
                    help="把每帧追踪结果写入 CSV")
    args = ap.parse_args()

    if args.camera is None:
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

    print(f"[ready] 算法={args.algo}  按 r 选追踪区域\n")

    csv_writer = None
    csv_file = None
    if args.log_csv:
        csv_file = open(args.log_csv, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(["timestamp", "cx", "cy", "w", "h", "ok"])

    tracker = None
    algo_name = args.algo
    last_box = None
    init_box = None    # 初始框，用于"丢了"时提示
    init_size = 0
    paused = False
    log_enabled = True
    last_log_t = 0.0
    LOG_INTERVAL = 0.1   # 10Hz

    fps_buf = []
    t_prev = time.time()

    while True:
        if not paused:
            ok_read, frame = cap.read()
            if not ok_read or frame is None:
                time.sleep(0.05)
                continue

            # 跑追踪器
            track_ok = False
            if tracker is not None:
                track_ok, box = tracker.update(frame)
                if track_ok:
                    last_box = tuple(int(v) for v in box)

        out = frame.copy()
        h, w = out.shape[:2]

        # 画追踪框
        if tracker is not None and last_box is not None:
            x, y, bw, bh = last_box
            color = (80, 220, 80) if track_ok else (0, 80, 240)
            cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
            # 中心十字
            cx, cy = x + bw // 2, y + bh // 2
            cv2.drawMarker(out, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)

            # 大小变化检测：远了 / 近了
            cur_size = bw * bh
            if init_size > 0:
                ratio = cur_size / init_size
                size_hint = (
                    "FAR" if ratio < 0.5
                    else "CLOSE" if ratio > 2.0
                    else "OK"
                )
            else:
                size_hint = "OK"

            label = f"({cx},{cy}) {bw}x{bh} {size_hint}"
            cv2.putText(out, label, (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            if not track_ok:
                cv2.putText(out, "TRACKING LOST  press r to re-select",
                            (10, h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0, 80, 240), 2)

            now = time.time()
            if log_enabled and now - last_log_t >= LOG_INTERVAL:
                last_log_t = now
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                state = "OK" if track_ok else "LOST"
                print(f"[{stamp}] {state} center=({cx:4d},{cy:4d}) size={bw}x{bh}")
                if csv_writer:
                    csv_writer.writerow([
                        datetime.now().isoformat(timespec="milliseconds"),
                        cx, cy, bw, bh, int(track_ok)
                    ])
                    csv_file.flush()

        # FPS 测量
        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt > 0:
            fps_buf.append(1.0 / dt)
            if len(fps_buf) > 30:
                fps_buf.pop(0)
        fps = sum(fps_buf) / len(fps_buf) if fps_buf else 0.0

        # 顶部状态条
        cv2.rectangle(out, (0, 0), (w, 32), (20, 20, 24), -1)
        state = ("PAUSED" if paused
                 else "TRACKING" if (tracker is not None and last_box is not None and track_ok)
                 else "LOST" if (tracker is not None and not track_ok)
                 else "no ROI · press r")
        col = ((220, 220, 100) if paused
               else (80, 230, 80) if state == "TRACKING"
               else (0, 80, 240) if state == "LOST"
               else (180, 180, 180))
        cv2.putText(out, state, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        cv2.putText(out, f"algo={algo_name}  fps={fps:4.1f}  log={'ON' if log_enabled else 'OFF'}",
                    (140, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
        cv2.putText(out, "[r]select  [c]clear  [t]algo  [l]log  [s]snap  [space]pause  [q]quit",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        cv2.imshow("Arm ROI Tracker", out)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('q'):
            break
        elif k == ord(' '):
            paused = not paused
            print(f"[{'paused' if paused else 'resumed'}]")
        elif k == ord('r'):
            print("[roi] 在弹窗里拖一个矩形框住目标，回车确认 / c 取消")
            r = cv2.selectROI("Arm ROI Tracker", frame, showCrosshair=True)
            if r != (0, 0, 0, 0):
                init_box = r
                init_size = r[2] * r[3]
                tracker = make_tracker(algo_name)
                tracker.init(frame, r)
                last_box = tuple(int(v) for v in r)
                print(f"[track] init {algo_name} box={init_box}")
            else:
                print("[roi] 取消")
        elif k == ord('c'):
            tracker = None
            last_box = None
            init_box = None
            init_size = 0
            print("[track] 清除")
        elif k == ord('t'):
            names = list(TRACKER_FACTORIES.keys())
            algo_name = names[(names.index(algo_name) + 1) % len(names)]
            print(f"[algo] {algo_name}")
            if tracker is not None and last_box is not None:
                tracker = make_tracker(algo_name)
                tracker.init(frame, last_box)
                print(f"[track] re-init {algo_name}")
        elif k == ord('l'):
            log_enabled = not log_enabled
            print(f"[log] {'ON' if log_enabled else 'OFF'}")
        elif k == ord('s'):
            fname = f"track_snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(fname, out)
            print(f"[snap] {fname}")

    cap.release()
    cv2.destroyAllWindows()
    if csv_file:
        csv_file.close()


if __name__ == "__main__":
    main()
