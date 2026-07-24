"""Trajectory metrics used by the RGB-D odometry experiment."""

from __future__ import annotations

import numpy as np

from .geometry import kabsch


def invert_pose(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    inverse = np.eye(4)
    inverse[:3, :3] = pose[:3, :3].T
    inverse[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return inverse


def compose_camera_to_world(previous_camera_to_world: np.ndarray, current_to_previous: np.ndarray) -> np.ndarray:
    """Accumulate an ICP transform that maps current-camera points into the previous camera frame.

    For a static world point, ``p_prev = T_prev_curr @ p_curr`` and
    ``p_world = T_world_prev @ p_prev``. Therefore the current camera pose is
    ``T_world_curr = T_world_prev @ T_prev_curr``; no inverse is applied.
    """
    return np.asarray(previous_camera_to_world, dtype=float) @ np.asarray(current_to_previous, dtype=float)


def rotation_angle_degrees(rotation: np.ndarray) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def align_estimated_poses(estimated: list[np.ndarray], reference: list[np.ndarray]) -> tuple[list[np.ndarray], np.ndarray, np.ndarray]:
    """SE(3)-align an estimated trajectory's positions to the reference positions."""
    if len(estimated) != len(reference) or len(estimated) < 3:
        raise ValueError("Need equal trajectory lengths of at least three poses")
    estimated_positions = np.array([pose[:3, 3] for pose in estimated])
    reference_positions = np.array([pose[:3, 3] for pose in reference])
    rotation, translation = kabsch(estimated_positions, reference_positions)
    alignment = np.eye(4)
    alignment[:3, :3] = rotation
    alignment[:3, 3] = translation
    aligned = [alignment @ pose for pose in estimated]
    return aligned, rotation, translation


def absolute_trajectory_error(aligned_estimated: list[np.ndarray], reference: list[np.ndarray]) -> dict[str, float]:
    errors = np.array([np.linalg.norm(estimate[:3, 3] - truth[:3, 3]) for estimate, truth in zip(aligned_estimated, reference)])
    return {
        "rmse_m": float(np.sqrt(np.mean(errors**2))),
        "mean_m": float(np.mean(errors)),
        "median_m": float(np.median(errors)),
        "max_m": float(np.max(errors)),
    }


def relative_pose_error(estimated: list[np.ndarray], reference: list[np.ndarray], *, frame_delta: int = 1) -> dict[str, float | int]:
    """Compute RPE between poses separated by a fixed number of frame samples."""
    if frame_delta < 1 or len(estimated) <= frame_delta:
        raise ValueError("frame_delta must be positive and smaller than trajectory length")
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for index in range(len(estimated) - frame_delta):
        estimate_relative = invert_pose(estimated[index]) @ estimated[index + frame_delta]
        truth_relative = invert_pose(reference[index]) @ reference[index + frame_delta]
        error = invert_pose(truth_relative) @ estimate_relative
        translation_errors.append(float(np.linalg.norm(error[:3, 3])))
        rotation_errors.append(rotation_angle_degrees(error[:3, :3]))
    translations = np.asarray(translation_errors)
    rotations = np.asarray(rotation_errors)
    return {
        "frame_delta": frame_delta,
        "pairs": int(len(translations)),
        "translation_rmse_m": float(np.sqrt(np.mean(translations**2))),
        "translation_mean_m": float(np.mean(translations)),
        "rotation_rmse_deg": float(np.sqrt(np.mean(rotations**2))),
        "rotation_mean_deg": float(np.mean(rotations)),
    }
