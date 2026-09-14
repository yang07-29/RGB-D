import numpy as np

from src.run_odometry import run_icp_with_recovery, summarize_tracking


def test_tracking_summary_counts_loss_and_recovery_duration():
    rows = [
        {"pair_index": 1, "accepted": False, "recovery_attempted": True, "recovery_succeeded": False},
        {"pair_index": 2, "accepted": False, "recovery_attempted": True, "recovery_succeeded": False},
        {"pair_index": 3, "accepted": True, "recovery_attempted": True, "recovery_succeeded": True},
    ]
    summary = summarize_tracking(rows, lost_after=2)
    assert summary["lost_events"] == 1
    assert summary["recovered_lost_events"] == 1
    assert summary["unrecovered_lost_events"] == 0
    assert summary["recovery_attempt_success_rate"] == 1 / 3
    assert summary["mean_lost_event_recovery_frames"] == 3.0


def test_multiscale_retry_recovers_motion_outside_the_fine_gate():
    indices = np.arange(20, dtype=float)
    base = np.column_stack((indices, np.sin(indices * 0.7), np.cos(indices * 0.31)))
    clouds = [
        base,
        base - np.array([0.06, 0.0, 0.0]),
        base - np.array([0.31, 0.0, 0.0]),
    ]
    poses, rows = run_icp_with_recovery(
        clouds,
        max_iterations=30,
        correspondence_distance_m=0.08,
        min_correspondence_ratio=0.5,
        max_acceptable_rmse_m=0.08,
        coarse_voxel_m=0.05,
        coarse_distance_multiplier=4.0,
        lost_after=1,
        rss_samples=[],
    )
    assert rows[0]["status"] == "ok_identity"
    assert rows[1]["status"] == "recovered_multiscale"
    assert rows[1]["recovery_succeeded"] is True
    np.testing.assert_allclose(poses[2][:3, 3], [0.31, 0.0, 0.0], atol=1e-7)
