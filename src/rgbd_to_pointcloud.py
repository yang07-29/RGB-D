"""将一组已对齐的 RGB-D PNG 转换为 PLY 点云。"""

from __future__ import annotations

import argparse

from .geometry import statistical_outlier_filter, voxel_downsample
from .rgbd import depth_to_points, load_png, write_colored_ply


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth", required=True, help="16-bit 深度 PNG")
    parser.add_argument("--rgb", help="与深度图像素对齐的 RGB PNG")
    parser.add_argument("--output", required=True, help="输出 PLY 路径")
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--voxel", type=float, default=0.02, help="米")
    args = parser.parse_args()

    depth = load_png(args.depth)
    color = load_png(args.rgb) if args.rgb else None
    points, colors = depth_to_points(depth, color=color, stride=args.stride)
    raw_count = len(points)

    # 颜色和点云需要保留同一索引；教学第一阶段先在无颜色点云上滤波/下采样。
    if colors is not None:
        print("检测到颜色：本阶段为保持索引一致性，仅输出原始采样点云。")
    else:
        points = statistical_outlier_filter(points, neighbors=min(20, len(points) - 1))
        points = voxel_downsample(points, args.voxel)

    write_colored_ply(args.output, points, colors)
    print(f"raw sampled points: {raw_count}")
    print(f"written points: {len(points)}")
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
