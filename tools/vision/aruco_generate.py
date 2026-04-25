#!/usr/bin/env python3
"""
生成 ArUco 标记（PNG，含黑边白底，方便打印）。

用法：./.venv-cv/bin/python aruco_generate.py
默认生成 ID 0/1/2/3 四张图，每张 800px，可以打印成 ~5cm 的方块。
打印时纸尺 = 你之后告诉脚本 --size 0.05 (单位米)，要量准。

贴到机械臂建议方案：
  ID 0  → 底座顶部（不动，做参考系）
  ID 1  → 末端法兰（随机械臂动）
  ID 2/3 → 备用，可以贴中段链接做附加追踪
"""
import argparse
import cv2
import cv2.aruco as aruco
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3],
                    help="要生成的 marker ID 列表（默认 0 1 2 3）")
    ap.add_argument("--size-px", type=int, default=800,
                    help="单张 PNG 边长像素（默认 800）")
    ap.add_argument("--out-dir", type=str, default="aruco",
                    help="输出目录（默认 ./aruco/）")
    ap.add_argument("--dict", type=str, default="DICT_4X4_50",
                    help="字典：DICT_4X4_50（少量大块）/ DICT_5X5_100 / DICT_6X6_250")
    args = ap.parse_args()

    import os
    os.makedirs(args.out_dir, exist_ok=True)

    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))

    # 标记本体大小（去掉白边）
    inner_px = int(args.size_px * 0.8)
    border_px = (args.size_px - inner_px) // 2

    for mid in args.ids:
        marker = aruco.generateImageMarker(aruco_dict, mid, inner_px)
        # 加白边（打印时方便剪、也是 ArUco 检测要求的 quiet zone）
        full = np.full((args.size_px, args.size_px), 255, dtype=np.uint8)
        full[border_px:border_px + inner_px,
             border_px:border_px + inner_px] = marker
        # 角落写 ID 文字
        cv2.putText(full, f"ID {mid} ({args.dict})",
                    (border_px, args.size_px - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,), 2)
        path = os.path.join(args.out_dir, f"aruco_{args.dict}_id{mid:03d}.png")
        cv2.imwrite(path, full)
        print(f"[gen] {path}")

    print(f"\n打印建议：")
    print(f"  - 用 100% 大小打印（不要 fit-to-page），保证黑块尺寸准确")
    print(f"  - 用厚一点的纸或者贴泡沫板上，避免弯曲影响识别")
    print(f"  - ID {args.ids[0]} 贴底座 / ID {args.ids[1]} 贴末端")
    print(f"  - 实际打印出来的黑色 marker 边长（量准），运行追踪脚本时填到 --size")


if __name__ == "__main__":
    main()
