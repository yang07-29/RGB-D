import pytest
from pathlib import Path

from src.performance_protocol import repeat_rgbd_pairs_for_performance, summarize_frame_performance, summarize_rss_trend
from src.tum import RgbdPair


def _row(index: int, rss: int) -> dict[str, float | int]:
    return {
        "frame_index": index,
        "rgbd_read_runtime_s": 0.002,
        "preprocessing_runtime_s": 0.003,
        "registration_runtime_s": 0.005 if index else 0.0,
        "compute_runtime_s": 0.008 if index else 0.003,
        "input_to_pose_runtime_s": 0.010 if index else 0.005,
        "process_rss_bytes_after_frame": rss,
    }


def test_frame_performance_excludes_first_frame_and_warmup_from_steady_state():
    rows = [_row(index, 1000 + index * 10) for index in range(5)]
    summary = summarize_frame_performance(rows, warmup_frames=2)

    assert summary["measurement_scope"]["input_to_pose"] == (
        "RGB/depth decode + point-cloud preprocessing + registration/pose decision; "
        "excludes association, evaluation, plotting, artifact serialization, and RSS sampling"
    )
    assert summary["all_registration_frames"]["samples"] == 4
    assert summary["steady_state_after_warmup"]["input_to_pose_latency"]["samples"] == 3
    assert summary["steady_state_after_warmup"]["input_to_pose_latency"]["mean_s"] == pytest.approx(0.010)


def test_rss_trend_reports_growth_after_warmup():
    rows = [_row(index, rss) for index, rss in enumerate((1000, 1100, 1200, 1300, 1400))]
    trend = summarize_rss_trend(rows, warmup_frames=2)

    assert trend["samples"] == 5
    assert trend["start_bytes"] == 1000
    assert trend["peak_bytes"] == 1400
    assert trend["steady_state_start_bytes"] == 1200
    assert trend["steady_state_end_bytes"] == 1400
    assert trend["steady_state_growth_bytes"] == 200
    assert trend["steady_state_linear_slope_bytes_per_frame"] == pytest.approx(100.0)


def test_warmup_must_leave_a_measured_registration_frame():
    rows = [_row(index, 1000) for index in range(3)]
    with pytest.raises(ValueError, match="warmup"):
        summarize_frame_performance(rows, warmup_frames=3)


def test_performance_replay_repeats_inputs_with_monotonic_timestamps():
    pairs = [
        RgbdPair(1.0, Path("rgb0.png"), Path("depth0.png"), 0.0),
        RgbdPair(1.1, Path("rgb1.png"), Path("depth1.png"), 0.0),
        RgbdPair(1.2, Path("rgb2.png"), Path("depth2.png"), 0.0),
    ]
    repeated = repeat_rgbd_pairs_for_performance(pairs, repeats=2)
    assert len(repeated) == 6
    assert [pair.rgb_path for pair in repeated] == [pair.rgb_path for pair in pairs] * 2
    assert all(right.timestamp > left.timestamp for left, right in zip(repeated, repeated[1:]))
