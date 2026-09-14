import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def test_parameter_sweep_publishes_all_protocol_v2_runs():
    result = ROOT / "results" / "parameter_sweep_one_to_one_quality_v2"
    rows = _rows(result / "benchmark_table.csv")
    assert len(rows) == 18
    assert {row["frames"] for row in rows} == {"790"}
    assert {row["association_protocol"] for row in rows} == {"one_to_one_minimum_offset_greedy_v2"}
    assert {row["method"] for row in rows} == {
        "frame_to_frame_point_to_point_icp",
        "open3d_point_to_plane_icp",
    }
    assert all((ROOT / Path(row["summary_path"])).is_file() for row in rows)


def test_multi_sequence_report_has_three_methods_per_sequence():
    result = ROOT / "results" / "multi_sequence_one_to_one_quality_v2"
    rows = _rows(result / "benchmark_table.csv")
    assert len(rows) == 9
    expected_frames = {"fr1_xyz": "790", "fr1_desk": "573", "fr1_desk2": "612"}
    for sequence, frame_count in expected_frames.items():
        sequence_rows = [row for row in rows if row["sequence"] == sequence]
        assert len(sequence_rows) == 3
        assert {row["frames"] for row in sequence_rows} == {frame_count}
        assert sum(row["method"] == "identity_no_motion_baseline" for row in sequence_rows) == 1
    assert all((ROOT / Path(row["summary_path"])).is_file() for row in rows)


def test_cpp_full_result_matches_python_protocol_and_metrics():
    cpp_result = ROOT / "results" / "cpp_one_to_one_quality_v2"
    cpp = json.loads((cpp_result / "summary.json").read_text(encoding="utf-8"))
    trajectory_rows = [
        line for line in (cpp_result / "trajectory_cpp_local.txt").read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    assert cpp["association_protocol"] == "one_to_one_minimum_offset_greedy_v2"
    assert cpp["frames"]["count"] == len(trajectory_rows) == 790
    assert cpp["quality"]["accepted_pairs"] + cpp["quality"]["rejected_or_exception_pairs"] == 789

    python_path = (
        ROOT / "results" / "parameter_sweep_one_to_one_quality_v2" / "summaries"
        / "numpy_point_to_point" / "voxel_0.05_corr_0.12" / "summary.json"
    )
    python = json.loads(python_path.read_text(encoding="utf-8"))
    python_method = python["methods"][1]
    assert abs(cpp["ate"]["rmse_m"] - python_method["ate"]["rmse_m"]) < 1e-4
    assert abs(cpp["rpe_frame_delta_1"]["translation_rmse_m"] - python_method["rpe_frame_delta_1"]["translation_rmse_m"]) < 1e-6
    assert abs(cpp["rpe_frame_delta_1"]["rotation_rmse_deg"] - python_method["rpe_frame_delta_1"]["rotation_rmse_deg"]) < 2e-5


def test_evo_crosscheck_is_within_frozen_tolerance():
    path = ROOT / "results" / "evo_crosscheck_one_to_one_v2" / "crosscheck.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    assert report["tool"] == {"name": "evo", "version": "1.37.1"}
    assert report["ape_alignment"] == "SE(3) Umeyama, scale correction disabled"
    assert report["passed"] is True
    assert report["maximum_absolute_difference"] <= report["tolerance"] == 1e-8
    assert report["comparisons"]["ate_translation"]["samples"] == 790
    assert report["comparisons"]["rpe_delta1_translation"]["samples"] == 789
    assert report["comparisons"]["rpe_delta30_translation"]["samples"] == 760


def test_tracking_stress_report_preserves_baseline_and_records_tradeoff():
    result = ROOT / "results" / "tracking_stress_one_to_one_v2"
    rows = _rows(result / "benchmark_table.csv")
    assert len(rows) == 6
    assert {(row["frame_step"], row["tracking_mode"]) for row in rows} == {
        (step, mode) for step in ("1", "2", "3") for mode in ("identity", "predictive_recovery")
    }
    assert all((ROOT / Path(row["summary_path"])).is_file() for row in rows)
    by_key = {(row["frame_step"], row["tracking_mode"]): row for row in rows}
    assert by_key[("1", "identity")]["ate_rmse_m"] == by_key[("1", "predictive_recovery")]["ate_rmse_m"]
    assert float(by_key[("3", "predictive_recovery")]["ate_rmse_m"]) < float(by_key[("3", "identity")]["ate_rmse_m"])
    assert int(by_key[("3", "predictive_recovery")]["successful_recovery_attempts"]) > 0
    assert int(by_key[("3", "predictive_recovery")]["unrecovered_lost_events"]) == 0


def test_pose_graph_edge_quality_report_records_clean_and_adversarial_runs():
    result = ROOT / "results" / "pose_graph_one_to_one_edge_quality_v2"
    report = json.loads((result / "summary.json").read_text(encoding="utf-8"))

    assert report["association_protocol"] == "one_to_one_minimum_offset_greedy_v2"
    assert report["frames"] == 790
    assert report["keyframes"] == 125
    sequential = report["sequential_edge_quality"]
    assert sequential["accepted_keyframe_icp"] + sequential["odometry_prediction_fallback"] == report["odometry_edges"]

    labels = report["posthoc_loop_labels"]
    assert labels["false_accepts"] == labels["accepted_incorrect"]
    assert labels["false_rejects"] == labels["rejected_correct"]
    assert sum(labels[key] for key in (
        "accepted_correct", "accepted_incorrect", "rejected_correct", "rejected_incorrect"
    )) == report["loop_candidates"]

    hard = report["hard_negative_stress"]
    assert hard["candidates"] == hard["rejected_by_same_gates"] == 10
    forced = report["forced_bad_edge_stress"]
    assert forced["performed"] is True
    assert forced["posthoc_gt_label"] == "incorrect"
    clean_ate = report["keyframe_after"]["ate"]["rmse_m"]
    bad_ate = forced["metrics_after_forced_insertion"]["ate"]["rmse_m"]
    assert bad_ate > clean_ate * 10

    for name in (
        "sequential_edges.csv",
        "loop_candidates.csv",
        "hard_negative_candidates.csv",
        "trajectory_keyframes_before.txt",
        "trajectory_keyframes_after.txt",
        "trajectory_keyframes_forced_bad_edge.txt",
        "trajectory_keyframes_before_after.png",
        "loop_correct_and_rejected_cases.png",
    ):
        assert (result / name).is_file()


def test_streaming_performance_report_has_repeats_and_soak_evidence():
    result = ROOT / "results" / "performance_streaming_v2"
    rows = _rows(result / "benchmark_table.csv")
    regular = [row for row in rows if row["run"].startswith("run_")]
    soak = next(row for row in rows if row["run"] == "soak_3x")
    assert len(regular) == 6
    assert {(row["method"], row["run"]) for row in regular} == {
        (method, f"run_{index}")
        for method in ("numpy", "open3d")
        for index in range(1, 4)
    }
    assert {row["frames"] for row in regular} == {"790"}
    assert all(row["evaluation_available"] == "True" for row in regular)
    for method in ("numpy", "open3d"):
        ate_values = [float(row["ate_rmse_m"]) for row in regular if row["method"] == method]
        assert max(ate_values) - min(ate_values) < 1e-12
    assert soak["method"] == "open3d"
    assert soak["frames"] == "2376"
    assert soak["evaluation_available"] == "False"

    report = json.loads((result / "summary.json").read_text(encoding="utf-8"))
    assert report["independent_full_sequence_runs"] == 3
    assert set(report["methods"]) == {"numpy", "open3d"}
    assert report["soak_input_frames_per_cycle"] == 792
    assert len(report["soak_cycle_rss"]) == 3
    assert (result / "latency_curve.png").is_file()
    assert (result / "rss_curve.png").is_file()
