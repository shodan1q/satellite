#!/usr/bin/env python3
"""
Arm Motion Detector — 实时检测画面中的运动区域，专门用于看机械臂是否在动。

输入：USB 接的 Insta360 X4（或任何 UVC 摄像头）
处理：MOG2 背景建模 → 形态学清噪 → 轮廓 → 包围框
输出：屏幕实时叠框 + stdout 事件日志（带时间戳和 bbox）

操作（运行后按键）：
  r   重新标定工作区 ROI（拖一个矩形，只看这块里的运动）
  c   清掉 ROI（看整个画面）
  b   重学背景（拍照式，适合环境变化后重置）
  s   截图保存到当前目录
  + / -   提高 / 降低运动灵敏度
  space   暂停 / 继续
  q   退出

依赖：cv2, numpy（已装在 .venv-cv）
跑：./.venv-cv/bin/python arm_motion_detector.py
   ./.venv-cv/bin/python arm_motion_detector.py --camera 1   # 指定相机
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def probe_cameras(max_idx: int = 6):
    """逐个试相机索引，回 (idx, w, h) 列表。"""
    found = []
    for i in range(max_idx):
        cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                found.append((i, w, h))
            cap.release()
    return found


def pick_camera_interactive(cams):
    """
    依次显示每个相机的画面，让用户按 y 选用 / n 下一个。
    分辨率自动猜不靠谱（MacBook 内置可能比 X4 还宽），所以让眼睛判断最稳。
    返回选中的 idx，或者 None（用户取消）。
    """
    print("\n[picker] 依次显示每个相机的画面，看到你想用的就按 y 选用，按 n 看下一个，q 退出")
    for idx, w, h in cams:
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            continue
        title = f"Camera {idx}  {w}x{h}  — press y=use  n=next  q=quit"
        for _ in range(60):  # 显示约 2 秒，期间反复读帧让画面活的
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            label = frame.copy()
            cv2.rectangle(label, (0, 0), (label.shape[1], 40), (20, 20, 24), -1)
            cv2.putText(label, f"cam {idx}  {w}x{h}  [y]use  [n]next  [q]quit",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 230, 255), 2)
            cv2.imshow("Camera picker", label)
            k = cv2.waitKey(30) & 0xFF
            if k == ord('y'):
                cap.release()
                cv2.destroyWindow("Camera picker")
                print(f"[picker] 选 cam {idx}")
                return idx
            if k == ord('n'):
                break
            if k == ord('q'):
                cap.release()
                cv2.destroyWindow("Camera picker")
                return None
        cap.release()
    cv2.destroyWindow("Camera picker")
    print("[picker] 全过完了，没选 —— 退出")
    return None


class MotionDetector:
    """MOG2 背景建模 + 形态学 + 轮廓。"""

    def __init__(self, var_threshold: float = 30.0, history: int = 250):
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=history,
            varThreshold=var_threshold,
            detectShadows=False,
        )
        # 形态学核 —— 闭运算填小洞，开运算去散点
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        # 灵敏度调节：min_area 越大 = 越不灵敏
        self.min_area = 600
        self.max_area_ratio = 0.85  # 超过画面 85% 当干扰丢弃

    def reset_background(self, frame):
        """重学背景 —— 直接重建一个 MOG2，喂一帧当起点。"""
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=250, varThreshold=30, detectShadows=False
        )
        for _ in range(5):
            self.bg.apply(frame)

    def detect(self, frame, roi=None):
        """
        返回 (mask, motion_boxes)：
          mask           二值图（255=运动）
          motion_boxes   [(x, y, w, h, area), ...]
        roi: (x, y, w, h) 只在这个矩形里检测；None = 全图
        """
        if roi is not None:
            x0, y0, w0, h0 = roi
            sub = frame[y0:y0 + h0, x0:x0 + w0]
            local_mask = self._raw_mask(sub)
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            mask[y0:y0 + h0, x0:x0 + w0] = local_mask
        else:
            mask = self._raw_mask(frame)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        frame_area = frame.shape[0] * frame.shape[1]
        boxes = []
        for c in contours:
            a = cv2.contourArea(c)
            if a < self.min_area:
                continue
            if a > frame_area * self.max_area_ratio:
                continue  # 整画面闪一下（曝光跳变）— 丢
            x, y, w, h = cv2.boundingRect(c)
            boxes.append((x, y, w, h, int(a)))
        # 大块在前
        boxes.sort(key=lambda b: -b[4])
        return mask, boxes

    def _raw_mask(self, frame):
        m = self.bg.apply(frame)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        return m


def draw_overlay(frame, boxes, roi, paused, sensitivity, fps, has_motion):
    """在画面上叠 bbox + ROI 框 + 状态条。返回新图（不改 in-place）。"""
    out = frame.copy()
    h, w = out.shape[:2]

    # ROI 边框（青色）
    if roi is not None:
        x, y, rw, rh = roi
        cv2.rectangle(out, (x, y), (x + rw, y + rh), (255, 200, 0), 2)
        cv2.putText(out, "ROI", (x + 6, y + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

    # 运动框（红色，最大那个亮一点）
    for i, (x, y, bw, bh, a) in enumerate(boxes):
        color = (50, 50, 255) if i == 0 else (80, 80, 200)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        cv2.putText(out, f"{a}px", (x + 4, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # 顶部状态条
    bar_h = 32
    cv2.rectangle(out, (0, 0), (w, bar_h), (20, 20, 24), -1)
    state = "PAUSED" if paused else ("MOTION" if has_motion else "still")
    state_color = ((220, 220, 100) if paused
                   else (80, 230, 80) if has_motion
                   else (180, 180, 180))
    cv2.putText(out, f"{state}", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    cv2.putText(out, f"min_area={sensitivity}  fps={fps:4.1f}  boxes={len(boxes)}",
                (110, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
    cv2.putText(out, "[r] ROI  [c] clear  [b] reset bg  [+/-] sens  [s] snap  [space] pause  [q] quit",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=None,
                    help="camera index, default = 自动选第一个 1280+ 宽度的")
    ap.add_argument("--width", type=int, default=1920,
                    help="请求分辨率宽度（X4 webcam 一般 1920×960 或 3840×1920）")
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--log-csv", type=str, default=None,
                    help="把每次运动事件追加写到此 CSV")
    ap.add_argument("--mask", action="store_true",
                    help="边窗口显示二值 mask")
    args = ap.parse_args()

    # 选相机
    if args.camera is None:
        print("[probe] 扫描可用相机…")
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
    print(f"[cap] 实际分辨率 {actual_w}×{actual_h}")

    detector = MotionDetector()
    roi = None
    paused = False
    last_motion_t = 0.0
    motion_event_cooldown = 0.5  # 0.5s 内的连续运动算一次事件
    in_event = False
    event_start_t = 0.0

    csv_writer = None
    csv_file = None
    if args.log_csv:
        csv_file = open(args.log_csv, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(["timestamp", "duration_s", "max_box_x",
                                 "max_box_y", "max_box_w", "max_box_h",
                                 "max_area_px"])

    print("\n[ready] 按 r 选 ROI · b 重学背景 · q 退出\n")

    fps_buf = []
    t_prev = time.time()

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[warn] 读帧失败")
                time.sleep(0.05)
                continue
            mask, boxes = detector.detect(frame, roi)
            has_motion = len(boxes) > 0
        # else 帧不更新

        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt > 0:
            fps_buf.append(1.0 / dt)
            if len(fps_buf) > 30:
                fps_buf.pop(0)
        fps = sum(fps_buf) / len(fps_buf) if fps_buf else 0.0

        # 事件聚合 + 日志
        if not paused and has_motion:
            if not in_event:
                in_event = True
                event_start_t = now
                top = boxes[0]
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{stamp}] motion start  bbox=({top[0]},{top[1]},{top[2]}×{top[3]})  area={top[4]}px")
            last_motion_t = now
        elif in_event and now - last_motion_t > motion_event_cooldown:
            in_event = False
            duration = last_motion_t - event_start_t
            top = boxes[0] if boxes else (0, 0, 0, 0, 0)
            stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{stamp}] motion end    duration={duration:.2f}s")
            if csv_writer:
                csv_writer.writerow([
                    datetime.now().isoformat(timespec="milliseconds"),
                    f"{duration:.3f}", top[0], top[1], top[2], top[3], top[4]
                ])
                csv_file.flush()

        overlay = draw_overlay(frame, boxes, roi, paused,
                               detector.min_area, fps, has_motion)
        cv2.imshow("Arm Motion Detector", overlay)
        if args.mask:
            cv2.imshow("Mask", mask)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord(' '):
            paused = not paused
            print(f"[{'paused' if paused else 'resumed'}]")
        elif k == ord('b'):
            detector.reset_background(frame)
            print("[bg] 已重学背景")
        elif k == ord('r'):
            print("[roi] 在弹窗里拖一个矩形，回车确认 / c 取消")
            r = cv2.selectROI("Arm Motion Detector", frame, showCrosshair=True)
            if r != (0, 0, 0, 0):
                roi = r
                print(f"[roi] 设置为 {roi}")
                detector.reset_background(frame)
            else:
                print("[roi] 取消")
        elif k == ord('c'):
            roi = None
            print("[roi] 已清除（全图检测）")
        elif k == ord('+') or k == ord('='):
            detector.min_area = max(50, int(detector.min_area * 0.7))
            print(f"[sens] min_area={detector.min_area}")
        elif k == ord('-') or k == ord('_'):
            detector.min_area = int(detector.min_area * 1.4)
            print(f"[sens] min_area={detector.min_area}")
        elif k == ord('s'):
            fname = f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(fname, overlay)
            print(f"[snap] 保存 {fname}")

    cap.release()
    cv2.destroyAllWindows()
    if csv_file:
        csv_file.close()


if __name__ == "__main__":
    main()
