"""Backend-independent acceptance rules for sequential and loop pose-graph edges."""

from __future__ import annotations

import numpy as np

from .quality import registration_failure_reason


def sequential_edge_failure_reason(
    *,
    correspondence_ratio: float,
    all_point_rmse_m: float,
    min_correspondence_ratio: float,
    max_all_point_rmse_m: float,
) -> str | None:
    return registration_failure_reason(
        correspondence_ratio=correspondence_ratio,
        residual_rmse_m=all_point_rmse_m,
        min_correspondence_ratio=min_correspondence_ratio,
        max_residual_rmse_m=max_all_point_rmse_m,
    )


def loop_edge_rejection_reasons(
    *,
    global_fitness: float,
    refined_fitness: float,
    refined_inlier_rmse_m: float,
    shared_correspondence_ratio: float,
    shared_all_point_rmse_m: float,
    consistency_translation_m: float,
    consistency_rotation_deg: float,
    min_global_fitness: float,
    min_refined_fitness: float,
    max_refined_inlier_rmse_m: float,
    min_shared_correspondence_ratio: float,
    max_shared_all_point_rmse_m: float,
    max_consistency_translation_m: float,
    max_consistency_rotation_deg: float,
) -> list[str]:
    values = (
        global_fitness,
        refined_fitness,
        refined_inlier_rmse_m,
        shared_correspondence_ratio,
        shared_all_point_rmse_m,
        consistency_translation_m,
        consistency_rotation_deg,
    )
    if not all(np.isfinite(value) for value in values):
        return ["non_finite_edge_metric"]
    reasons = []
    if global_fitness < min_global_fitness:
        reasons.append(f"global_fitness<{min_global_fitness}")
    if refined_fitness < min_refined_fitness:
        reasons.append(f"refined_fitness<{min_refined_fitness}")
    if refined_inlier_rmse_m > max_refined_inlier_rmse_m:
        reasons.append(f"refined_inlier_rmse>{max_refined_inlier_rmse_m}")
    if shared_correspondence_ratio < min_shared_correspondence_ratio:
        reasons.append(f"shared_correspondence_ratio<{min_shared_correspondence_ratio}")
    if shared_all_point_rmse_m > max_shared_all_point_rmse_m:
        reasons.append(f"shared_all_point_rmse>{max_shared_all_point_rmse_m}")
    if consistency_translation_m > max_consistency_translation_m:
        reasons.append(f"consistency_translation>{max_consistency_translation_m}")
    if consistency_rotation_deg > max_consistency_rotation_deg:
        reasons.append(f"consistency_rotation>{max_consistency_rotation_deg}")
    return reasons
