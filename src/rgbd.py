"""TUM 风格 RGB-D 图像到点云的最小实现。

不依赖 Open3D，目的是让每个坐标转换步骤都可检查。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .geometry import as_points


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float = 525.0
    fy: float = 525.0
    cx: float = 319.5
    cy: float = 239.5
    depth_scale: float = 5000.0


TUM_KINECT = CameraIntrinsics()


def load_png(path: str | Path) -> np.ndarray:
    """读取 PNG，同时保留 16-bit 深度值。"""
    return np.asarray(Image.open(path))


def depth_to_points(
    depth: np.ndarray,
    *,
    intrinsics: CameraIntrinsics = TUM_KINECT,
    color: np.ndarray | None = None,
    stride: int = 4,
    min_depth_m: float = 0.2,
    max_depth_m: float = 5.0,
) -> tuple[np.ndarray, np.ndarray | None]:
    """由已对齐深度图反投影为相机坐标系点云。

    对像素 (u, v) 和深度 Z，有 X=(u-cx)Z/fx, Y=(v-cy)Z/fy。
    """
    depth = np.asarray(depth)
    if depth.ndim != 2:
        raise ValueError("depth 必须是单通道二维图像")
    if stride <= 0:
        raise ValueError("stride 必须大于 0")
    if color is not None:
        color = np.asarray(color)
        if color.shape[:2] != depth.shape:
            raise ValueError("彩色图与深度图必须像素对齐且尺寸相同")

    v, u = np.mgrid[0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride]
    raw_depth = depth[::stride, ::stride].astype(np.float64)
    z = raw_depth / intrinsics.depth_scale
    valid = (z >= min_depth_m) & (z <= max_depth_m) & np.isfinite(z)

    z = z[valid]
    x = (u[valid] - intrinsics.cx) * z / intrinsics.fx
    y = (v[valid] - intrinsics.cy) * z / intrinsics.fy
    points = np.column_stack((x, y, z))

    colors = None
    if color is not None:
        sampled = color[::stride, ::stride]
        colors = sampled[valid]
        if colors.ndim == 1:
            colors = np.repeat(colors[:, None], 3, axis=1)
        colors = colors[:, :3].astype(np.float64) / 255.0
    return points, colors


def write_colored_ply(path: str | Path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    """写出最简单的 ASCII PLY，方便用 MeshLab/CloudCompare 打开。"""
    points = as_points(points)
    if colors is not None:
        colors = np.asarray(colors)
        if colors.shape != points.shape:
            raise ValueError("colors 应为与 points 相同的 (N, 3) 形状")
        colors = np.clip(np.round(colors * 255), 0, 255).astype(np.uint8)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = ["x", "y", "z"]
    if colors is not None:
        columns += ["red", "green", "blue"]
    header = ["ply", "format ascii 1.0", f"element vertex {len(points)}"]
    header += [f"property float {name}" for name in columns[:3]]
    if colors is not None:
        header += [f"property uchar {name}" for name in columns[3:]]
    header.append("end_header")

    with path.open("w", encoding="ascii") as file:
        file.write("\n".join(header) + "\n")
        if colors is None:
            for x, y, z in points:
                file.write(f"{x:.7f} {y:.7f} {z:.7f}\n")
        else:
            for (x, y, z), (r, g, b) in zip(points, colors):
                file.write(f"{x:.7f} {y:.7f} {z:.7f} {r} {g} {b}\n")
