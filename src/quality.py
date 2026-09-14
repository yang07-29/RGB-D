"""Explicit registration acceptance rules shared by all odometry baselines."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class RegistrationQuality:
    """Backend-independent nearest-neighbour quality after registration."""

    correspondences: int
    correspondence_ratio: float
    inlier_rmse_m: float
    all_point_rmse_m: float


def evaluate_registration_quality(
    source: np.ndarray,
    target: np.ndarray,
    transformation: np.ndarray,
    *,
    max_correspondence_m: float,
) -> RegistrationQuality:
    """Measure a final source-to-target transform with one shared protocol."""
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    transformation = np.asarray(transformation, dtype=float)
    if source.ndim != 2 or source.shape[1:] != (3,) or len(source) < 1:
        raise ValueError("source must contain finite 3D points")
    if target.ndim != 2 or target.shape[1:] != (3,) or len(target) < 1:
        raise ValueError("target must contain finite 3D points")
    if transformation.shape != (4, 4):
        raise ValueError("transformation must be a 4x4 matrix")
    if not np.isfinite(source).all() or not np.isfinite(target).all() or not np.isfinite(transformation).all():
        raise ValueError("registration inputs and transformation must be finite")
    if not np.isfinite(max_correspondence_m) or max_correspondence_m <= 0.0:
        raise ValueError("max_correspondence_m must be finite and positive")
    if not np.allclose(transformation[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-8):
        raise ValueError("transformation must have a homogeneous bottom row")
    rotation = transformation[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-5
    ):
        raise ValueError("transformation rotation must be a rigid SE(3) rotation")

    transformed = (transformation[:3, :3] @ source.T).T + transformation[:3, 3]
    distances, _ = cKDTree(target).query(transformed, k=1)
    inliers = distances <= max_correspondence_m
    correspondences = int(np.count_nonzero(inliers))
    inlier_rmse = float(np.sqrt(np.mean(np.square(distances[inliers])))) if correspondences else float("inf")
    return RegistrationQuality(
        correspondences=correspondences,
        correspondence_ratio=float(correspondences / len(source)),
        inlier_rmse_m=inlier_rmse,
        all_point_rmse_m=float(np.sqrt(np.mean(np.square(distances)))),
    )


def registration_failure_reason(
    *,
    correspondence_ratio: float,
    residual_rmse_m: float,
    min_correspondence_ratio: float,
    max_residual_rmse_m: float,
) -> str | None:
    """Return an auditable failure reason, or ``None`` for an accepted result."""
    values = (correspondence_ratio, residual_rmse_m, min_correspondence_ratio, max_residual_rmse_m)
    if not all(np.isfinite(value) for value in values):
        return "non_finite_registration_metric"
    if not 0.0 <= correspondence_ratio <= 1.0:
        return f"correspondence_ratio {correspondence_ratio:.3f} outside [0, 1]"
    if not 0.0 <= min_correspondence_ratio <= 1.0 or max_residual_rmse_m < 0.0:
        raise ValueError("registration quality thresholds are invalid")
    if correspondence_ratio < min_correspondence_ratio:
        return f"correspondence_ratio {correspondence_ratio:.3f} < {min_correspondence_ratio:.3f}"
    if residual_rmse_m > max_residual_rmse_m:
        return f"residual_rmse_m {residual_rmse_m:.4f} > {max_residual_rmse_m:.4f}"
    return None
