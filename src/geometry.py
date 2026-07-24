"""不依赖 Open3D 的最小点云几何与 ICP 实现。

这里的目标是学习算法和验证结果，不追求生产级性能。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


Array = np.ndarray


@dataclass
class ICPResult:
    rotation: Array
    translation: Array
    aligned_source: Array
    rmse: float
    iterations: int
    correspondences: int


def as_points(points: Array) -> Array:
    """校验并返回形状为 (N, 3) 的点云。"""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"点云应为 (N, 3)，实际为 {points.shape}")
    if len(points) < 3:
        raise ValueError("至少需要三个点")
    return points


def transform_points(points: Array, rotation: Array, translation: Array) -> Array:
    """对行向量点云执行 p' = R p + t。"""
    points = as_points(points)
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(3)
    if rotation.shape != (3, 3):
        raise ValueError("rotation 应为 (3, 3)")
    return (rotation @ points.T).T + translation


def kabsch(source: Array, target: Array) -> tuple[Array, Array]:
    """已知一一对应点时，最小二乘估计 source 到 target 的刚体变换。"""
    source = as_points(source)
    target = as_points(target)
    if source.shape != target.shape:
        raise ValueError("source 与 target 必须点数相同且一一对应")

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center

    covariance = source_zero.T @ target_zero
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T

    # SVD 可能产生镜像反射；刚体变换必须保持右手坐标系。
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T

    translation = target_center - rotation @ source_center
    return rotation, translation


def voxel_downsample(points: Array, voxel_size: float) -> Array:
    """按体素内点的平均值进行最小化下采样。"""
    points = as_points(points)
    if voxel_size <= 0:
        raise ValueError("voxel_size 必须大于 0")

    voxel_id = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(voxel_id, axis=0, return_inverse=True)
    sums = np.zeros((inverse.max() + 1, 3), dtype=float)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def statistical_outlier_filter(points: Array, neighbors: int = 20, std_ratio: float = 2.0) -> Array:
    """用邻域平均距离过滤孤立点。

    这是一个教学版实现；大点云的工程版本应优先使用 Open3D/PCL。
    """
    points = as_points(points)
    if not 1 <= neighbors < len(points):
        raise ValueError("neighbors 必须介于 1 与点数减一之间")
    if std_ratio < 0:
        raise ValueError("std_ratio 不能为负")

    distances, _ = cKDTree(points).query(points, k=neighbors + 1)
    mean_distance = distances[:, 1:].mean(axis=1)
    threshold = mean_distance.mean() + std_ratio * mean_distance.std()
    return points[mean_distance <= threshold]


def icp_point_to_point(
    source: Array,
    target: Array,
    *,
    max_iterations: int = 80,
    tolerance: float = 1e-8,
    max_correspondence_distance: float | None = None,
) -> ICPResult:
    """使用最近邻匹配和 Kabsch 更新的点到点 ICP。

    ICP 只在初始位姿足够接近时可靠。真实场景中通常需要特征匹配或 RANSAC 提供初值。
    """
    source = as_points(source)
    target = as_points(target)
    tree = cKDTree(target)

    aligned = source.copy()
    total_rotation = np.eye(3)
    total_translation = np.zeros(3)
    previous_rmse = np.inf

    for iteration in range(1, max_iterations + 1):
        distances, nearest_index = tree.query(aligned, k=1)
        valid = np.ones(len(aligned), dtype=bool)
        if max_correspondence_distance is not None:
            valid = distances <= max_correspondence_distance
        if valid.sum() < 3:
            raise RuntimeError("有效对应点少于 3 个；请放宽阈值或改善初始位姿")

        matched_source = aligned[valid]
        matched_target = target[nearest_index[valid]]
        delta_rotation, delta_translation = kabsch(matched_source, matched_target)
        aligned = transform_points(aligned, delta_rotation, delta_translation)

        total_rotation = delta_rotation @ total_rotation
        total_translation = delta_rotation @ total_translation + delta_translation

        new_distances, _ = tree.query(aligned, k=1)
        rmse = float(np.sqrt(np.mean(new_distances**2)))
        if abs(previous_rmse - rmse) < tolerance:
            return ICPResult(
                total_rotation,
                total_translation,
                aligned,
                rmse,
                iteration,
                int(valid.sum()),
            )
        previous_rmse = rmse

    return ICPResult(
        total_rotation,
        total_translation,
        aligned,
        previous_rmse,
        max_iterations,
        int(valid.sum()),
    )
