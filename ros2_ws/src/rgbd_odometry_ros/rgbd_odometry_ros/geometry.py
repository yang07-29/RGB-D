"""NumPy/SciPy geometry used by the portable ROS 2 Python backend."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class IcpResult:
    transform: np.ndarray
    correspondences: int
    rmse_m: float
    iterations: int


def depth_to_points(
    depth: np.ndarray,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float,
    stride: int,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    rows = np.arange(0, depth.shape[0], stride)
    columns = np.arange(0, depth.shape[1], stride)
    u, v = np.meshgrid(columns, rows)
    z = depth[np.ix_(rows, columns)].astype(np.float64) / depth_scale
    valid = (z >= min_depth_m) & (z <= max_depth_m)
    z = z[valid]
    return np.column_stack(((u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z))


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if len(points) == 0:
        return points.copy()
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((int(inverse.max()) + 1, 3), dtype=np.float64)
    counts = np.zeros(len(sums), dtype=np.int64)
    np.add.at(sums, inverse, points)
    np.add.at(counts, inverse, 1)
    return sums / counts[:, None]


def kabsch(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0:
        right_t[-1] *= -1
        rotation = right_t.T @ left.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def icp_point_to_point(
    source: np.ndarray,
    target: np.ndarray,
    *,
    max_iterations: int,
    max_correspondence_m: float,
    tolerance: float = 1e-8,
) -> IcpResult:
    if len(source) < 3 or len(target) < 3:
        return IcpResult(np.eye(4), 0, float("inf"), 0)
    tree = cKDTree(target)
    transformed = source.copy()
    total = np.eye(4)
    previous_rmse = float("inf")
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        distances, indices = tree.query(transformed, k=1, workers=1)
        mask = distances <= max_correspondence_m
        if np.count_nonzero(mask) < 3:
            break
        increment = kabsch(transformed[mask], target[indices[mask]])
        transformed = transform_points(transformed, increment)
        total = increment @ total
        current_rmse = float(np.sqrt(np.mean(distances[mask] ** 2)))
        if abs(previous_rmse - current_rmse) < tolerance:
            break
        previous_rmse = current_rmse
    distances, _ = tree.query(transformed, k=1, workers=1)
    correspondences = int(np.count_nonzero(distances <= max_correspondence_m))
    rmse = float(np.sqrt(np.mean(distances**2)))
    return IcpResult(total, correspondences, rmse, iterations)
