"""Keyframe and loop-candidate logic that is independent of Open3D."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .metrics import invert_pose, rotation_angle_degrees


@dataclass(frozen=True)
class LoopCandidate:
    source_keyframe_id: int
    target_keyframe_id: int
    estimated_distance_m: float
    estimated_rotation_deg: float


@dataclass(frozen=True)
class DescriptorLoopCandidate:
    source_keyframe_id: int
    target_keyframe_id: int
    descriptor_similarity: float


def select_keyframe_indices(
    poses: list[np.ndarray],
    *,
    min_translation_m: float,
    min_rotation_deg: float,
    max_interval_frames: int,
) -> list[int]:
    """Select a keyframe when motion or the maximum frame interval is reached."""
    if len(poses) < 2:
        raise ValueError("At least two poses are required")
    if min_translation_m <= 0 or min_rotation_deg <= 0 or max_interval_frames < 1:
        raise ValueError("Keyframe thresholds must be positive")
    indices = [0]
    last = 0
    for index in range(1, len(poses)):
        relative = invert_pose(poses[last]) @ poses[index]
        translation = float(np.linalg.norm(relative[:3, 3]))
        rotation = rotation_angle_degrees(relative[:3, :3])
        if translation >= min_translation_m or rotation >= min_rotation_deg or index - last >= max_interval_frames:
            indices.append(index)
            last = index
    if indices[-1] != len(poses) - 1:
        indices.append(len(poses) - 1)
    return indices


def propose_loop_candidates(
    keyframe_poses: list[np.ndarray],
    *,
    min_keyframe_separation: int,
    max_estimated_distance_m: float,
    max_estimated_rotation_deg: float,
    max_candidates: int,
    nonmax_radius_keyframes: int = 2,
) -> list[LoopCandidate]:
    """Propose non-local loops from odometry proximity without using ground truth.

    Each target keyframe contributes only its best older candidate. A simple
    temporal non-maximum suppression then avoids evaluating many near-duplicate
    pairs around the same physical revisit.
    """
    if len(keyframe_poses) < 2:
        raise ValueError("At least two keyframe poses are required")
    if min_keyframe_separation < 2 or max_estimated_distance_m <= 0 or max_estimated_rotation_deg <= 0 or max_candidates < 1:
        raise ValueError("Loop-candidate thresholds are invalid")

    proposed: list[LoopCandidate] = []
    for target_id in range(min_keyframe_separation, len(keyframe_poses)):
        best: LoopCandidate | None = None
        best_score = float("inf")
        for source_id in range(0, target_id - min_keyframe_separation + 1):
            relative = invert_pose(keyframe_poses[source_id]) @ keyframe_poses[target_id]
            distance = float(np.linalg.norm(relative[:3, 3]))
            rotation = rotation_angle_degrees(relative[:3, :3])
            if distance > max_estimated_distance_m or rotation > max_estimated_rotation_deg:
                continue
            score = distance / max_estimated_distance_m + rotation / max_estimated_rotation_deg
            if score < best_score:
                best_score = score
                best = LoopCandidate(source_id, target_id, distance, rotation)
        if best is not None:
            proposed.append(best)

    proposed.sort(key=lambda item: (item.estimated_distance_m, item.estimated_rotation_deg, item.source_keyframe_id, item.target_keyframe_id))
    selected: list[LoopCandidate] = []
    for candidate in proposed:
        duplicate = any(
            abs(candidate.source_keyframe_id - other.source_keyframe_id) <= nonmax_radius_keyframes
            and abs(candidate.target_keyframe_id - other.target_keyframe_id) <= nonmax_radius_keyframes
            for other in selected
        )
        if not duplicate:
            selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return selected


def propose_descriptor_loop_candidates(
    descriptors: np.ndarray,
    *,
    min_keyframe_separation: int,
    min_similarity: float,
    max_candidates: int,
    top_k_per_target: int = 1,
    nonmax_radius_keyframes: int = 2,
) -> list[DescriptorLoopCandidate]:
    """Propose non-local loops using descriptor similarity only.

    The function deliberately accepts no pose input: estimated or ground-truth
    trajectory cannot leak into learned candidate discovery. Geometric
    registration remains responsible for accepting or rejecting proposals.
    """
    descriptors = np.asarray(descriptors, dtype=np.float32)
    if descriptors.ndim != 2 or len(descriptors) < 2:
        raise ValueError("Descriptors must be a two-dimensional non-empty matrix")
    if min_keyframe_separation < 2 or max_candidates < 1 or top_k_per_target < 1:
        raise ValueError("Descriptor candidate thresholds are invalid")
    norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
    if np.any(norms <= 0):
        raise ValueError("Descriptors must have non-zero norm")
    descriptors = descriptors / norms
    proposed: list[DescriptorLoopCandidate] = []
    for target_id in range(min_keyframe_separation, len(descriptors)):
        last_source = target_id - min_keyframe_separation
        scores = descriptors[: last_source + 1] @ descriptors[target_id]
        order = np.argsort(-scores, kind="stable")[:top_k_per_target]
        for source_id in order:
            score = float(scores[source_id])
            if score >= min_similarity:
                proposed.append(DescriptorLoopCandidate(int(source_id), target_id, score))
    proposed.sort(key=lambda item: (-item.descriptor_similarity, item.source_keyframe_id, item.target_keyframe_id))
    selected: list[DescriptorLoopCandidate] = []
    for candidate in proposed:
        duplicate = any(
            abs(candidate.source_keyframe_id - other.source_keyframe_id) <= nonmax_radius_keyframes
            and abs(candidate.target_keyframe_id - other.target_keyframe_id) <= nonmax_radius_keyframes
            for other in selected
        )
        if not duplicate:
            selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return selected
