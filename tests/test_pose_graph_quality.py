import math

from src.pose_graph_quality import loop_edge_rejection_reasons, sequential_edge_failure_reason


def test_bad_sequential_edge_uses_shared_quality_failure_reason():
    assert sequential_edge_failure_reason(
        correspondence_ratio=0.2,
        all_point_rmse_m=0.03,
        min_correspondence_ratio=0.5,
        max_all_point_rmse_m=0.08,
    ).startswith("correspondence_ratio")
    assert sequential_edge_failure_reason(
        correspondence_ratio=0.9,
        all_point_rmse_m=0.03,
        min_correspondence_ratio=0.5,
        max_all_point_rmse_m=0.08,
    ) is None


def test_loop_edge_requires_shared_quality_and_finite_metrics():
    common = dict(
        global_fitness=0.8,
        refined_fitness=0.8,
        refined_inlier_rmse_m=0.02,
        shared_correspondence_ratio=0.8,
        shared_all_point_rmse_m=0.04,
        consistency_translation_m=0.02,
        consistency_rotation_deg=2.0,
        min_global_fitness=0.15,
        min_refined_fitness=0.45,
        max_refined_inlier_rmse_m=0.04,
        min_shared_correspondence_ratio=0.45,
        max_shared_all_point_rmse_m=0.12,
        max_consistency_translation_m=0.08,
        max_consistency_rotation_deg=15.0,
    )
    assert loop_edge_rejection_reasons(**common) == []
    assert "shared_correspondence_ratio" in loop_edge_rejection_reasons(
        **(common | {"shared_correspondence_ratio": 0.2})
    )[0]
    assert loop_edge_rejection_reasons(**(common | {"shared_all_point_rmse_m": math.nan})) == [
        "non_finite_edge_metric"
    ]
