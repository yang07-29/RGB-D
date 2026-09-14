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
