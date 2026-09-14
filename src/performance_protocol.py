"""Shared timing and resident-memory reporting for offline odometry runs."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from .runtime import latency_stats
from .tum import RgbdPair


INPUT_TO_POSE_SCOPE = (
    "RGB/depth decode + point-cloud preprocessing + registration/pose decision; "
    "excludes association, evaluation, plotting, artifact serialization, and RSS sampling"
)


def repeat_rgbd_pairs_for_performance(frames: list[RgbdPair], *, repeats: int) -> list[RgbdPair]:
    """Repeat file inputs with monotonic timestamps for a performance-only soak run."""
    if repeats < 1:
        raise ValueError("repeats must be at least one")
    if repeats == 1:
        return list(frames)
    if len(frames) < 2:
        raise ValueError("at least two frames are required for repeated replay")
    intervals = np.diff([frame.timestamp for frame in frames])
    positive_intervals = intervals[intervals > 0]
    if len(positive_intervals) == 0:
        raise ValueError("frame timestamps must contain a positive interval")
    period = frames[-1].timestamp - frames[0].timestamp + float(np.median(positive_intervals))
    return [
        replace(frame, timestamp=frame.timestamp + repetition * period)
        for repetition in range(repeats)
        for frame in frames
    ]


def _registration_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    return [row for row in rows if int(row["frame_index"]) > 0]


def _latency(rows: list[dict[str, object]], key: str) -> dict[str, float]:
    return latency_stats([float(row[key]) for row in rows])


def summarize_frame_performance(
    rows: list[dict[str, object]], *, warmup_frames: int
) -> dict[str, object]:
    if warmup_frames < 0:
        raise ValueError("warmup_frames must be non-negative")
    registration_rows = _registration_rows(rows)
    steady_rows = [row for row in registration_rows if int(row["frame_index"]) >= warmup_frames]
    if not steady_rows:
        raise ValueError("warmup must leave at least one measured registration frame")
    return {
        "measurement_scope": {
            "rgbd_read": "decode one RGB image and one depth image from local storage",
            "preprocessing": "depth back-projection, filtering/downsampling, and backend-required normal estimation",
            "registration": "ICP, final-transform quality calculation, acceptance decision, and pose accumulation",
            "compute": "preprocessing + registration; excludes RGB-D decode",
            "input_to_pose": INPUT_TO_POSE_SCOPE,
            "instrumentation": "per-frame RSS sampling and CSV/JSON/image writes are outside input-to-pose timing",
        },
        "warmup_frames": warmup_frames,
        "all_registration_frames": {
            "samples": len(registration_rows),
            "input_to_pose_latency": _latency(registration_rows, "input_to_pose_runtime_s"),
        },
        "steady_state_after_warmup": {
            "frame_index_at_or_after": warmup_frames,
            "rgbd_read_latency": _latency(steady_rows, "rgbd_read_runtime_s"),
            "preprocessing_latency": _latency(steady_rows, "preprocessing_runtime_s"),
            "registration_latency": _latency(steady_rows, "registration_runtime_s"),
            "compute_latency": _latency(steady_rows, "compute_runtime_s"),
            "input_to_pose_latency": _latency(steady_rows, "input_to_pose_runtime_s"),
        },
    }


def summarize_rss_trend(
    rows: list[dict[str, object]], *, warmup_frames: int
) -> dict[str, int | float | str | None]:
    samples = [
        (int(row["frame_index"]), int(row["process_rss_bytes_after_frame"]))
        for row in rows
        if row.get("process_rss_bytes_after_frame") is not None
    ]
    if not samples:
        return {
            "sampling": "after every processed frame, outside input-to-pose timing",
            "samples": 0,
            "start_bytes": None,
            "end_bytes": None,
            "peak_bytes": None,
            "steady_state_start_bytes": None,
            "steady_state_end_bytes": None,
            "steady_state_growth_bytes": None,
            "steady_state_linear_slope_bytes_per_frame": None,
        }
    steady = [(index, rss) for index, rss in samples if index >= warmup_frames]
    if not steady:
        raise ValueError("warmup must leave at least one RSS sample")
    x = np.asarray([index for index, _ in steady], dtype=float)
    y = np.asarray([rss for _, rss in steady], dtype=float)
    slope = float(np.polyfit(x, y, 1)[0]) if len(steady) > 1 else 0.0
    return {
        "sampling": "after every processed frame, outside input-to-pose timing",
        "samples": len(samples),
        "start_bytes": samples[0][1],
        "end_bytes": samples[-1][1],
        "peak_bytes": max(rss for _, rss in samples),
        "steady_state_start_bytes": steady[0][1],
        "steady_state_end_bytes": steady[-1][1],
        "steady_state_growth_bytes": steady[-1][1] - steady[0][1],
        "steady_state_linear_slope_bytes_per_frame": slope,
    }
