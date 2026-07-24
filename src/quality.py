"""Explicit registration acceptance rules shared by all odometry baselines."""

from __future__ import annotations


def registration_failure_reason(
    *,
    correspondence_ratio: float,
    residual_rmse_m: float,
    min_correspondence_ratio: float,
    max_residual_rmse_m: float,
) -> str | None:
    """Return an auditable failure reason, or ``None`` for an accepted result."""
    if correspondence_ratio < min_correspondence_ratio:
        return f"correspondence_ratio {correspondence_ratio:.3f} < {min_correspondence_ratio:.3f}"
    if residual_rmse_m > max_residual_rmse_m:
        return f"residual_rmse_m {residual_rmse_m:.4f} > {max_residual_rmse_m:.4f}"
    return None
