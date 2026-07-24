"""Aggregate NumPy and Open3D sweep summaries into one auditable CSV/Markdown table."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def row_from_summary(path: Path) -> dict[str, object]:
    summary = json.loads(path.read_text(encoding="utf-8"))
    method = summary["methods"][1]
    performance_key = "icp_performance" if "icp_performance" in summary else "performance"
    performance = summary[performance_key]
    e2e = performance["end_to_end_latency_excluding_first_frame"]
    registration = performance["registration_latency"]
    return {
        "method": method["method"], "voxel_m": summary["parameters"]["voxel"], "max_correspondence_m": summary["parameters"]["max_correspondence"],
        "ate_rmse_m": method["ate"]["rmse_m"], "rpe_translation_rmse_m": method["rpe_frame_delta_1"]["translation_rmse_m"],
        "rpe_rotation_rmse_deg": method["rpe_frame_delta_1"]["rotation_rmse_deg"],
        "end_to_end_mean_ms": 1000 * e2e["mean_s"], "end_to_end_p95_ms": 1000 * e2e["p95_s"], "fps_from_mean": e2e["fps_from_mean"],
        "registration_mean_ms": 1000 * registration["mean_s"], "rss_peak_bytes": performance["process_rss"]["peak_bytes"],
        "accepted_pairs": performance["accepted_pairs"], "rejected_or_exception_pairs": performance["rejected_or_exception_pairs"],
        "python": summary["environment"]["python"].split()[0], "open3d": summary["environment"].get("open3d", "N/A"), "summary_path": str(path),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Sweep root containing method/voxel_*/corr_* directories.")
    parser.add_argument("--output", type=Path, default=Path("results/parameter_sweep"))
    parser.add_argument("--extra-summary", type=Path, nargs="*", default=[], help="Additional equivalent summary.json paths to include without copying them.")
    args = parser.parse_args()
    paths = sorted(args.input.glob("**/summary.json")) + args.extra_summary
    rows = [row_from_summary(path) for path in paths]
    if not rows:
        raise RuntimeError(f"No summary.json files found under {args.input}")
    rows.sort(key=lambda row: (row["method"], float(row["voxel_m"]), float(row["max_correspondence_m"])))
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "benchmark_table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    headers = ["方法", "体素(m)", "对应阈值(m)", "ATE RMSE(m)", "RPE 平移(m)", "RPE 旋转(°)", "端到端均值(ms)", "p95(ms)", "FPS", "RSS(MB)", "拒绝/异常帧对"]
    lines = ["# 参数实验：真实结果", "", "所有行使用同一 796 帧关联、同一真值和同一 ATE/RPE 实现；全部性能行在同一 Python 3.12/Open3D 环境中顺序运行。", "", "| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    for row in rows:
        lines.append("| {method} | {voxel_m:.2f} | {max_correspondence_m:.2f} | {ate_rmse_m:.6f} | {rpe_translation_rmse_m:.6f} | {rpe_rotation_rmse_deg:.6f} | {end_to_end_mean_ms:.3f} | {end_to_end_p95_ms:.3f} | {fps_from_mean:.2f} | {rss_mb:.2f} | {rejected_or_exception_pairs} |".format(**row, rss_mb=row["rss_peak_bytes"] / 1024**2))
    lines += ["", "失败判据：内点/对应比例 < 0.50，或残差 RMSE 大于当前对应点阈值；被拒绝帧对保持上一帧位姿，并计入轨迹误差。", "", "原始汇总 JSON 的路径逐行保存在 `benchmark_table.csv` 中。"]
    (args.output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
