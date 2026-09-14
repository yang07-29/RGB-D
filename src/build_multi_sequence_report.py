"""Build a reproducible fixed-configuration report across multiple TUM sequences."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


def _sequence_name(dataset: str) -> str:
    name = Path(dataset).name
    return name.removeprefix("rgbd_dataset_").replace("freiburg", "fr")


def rows_from_summary(path: Path, include_identity: bool) -> list[dict[str, object]]:
    summary = json.loads(path.read_text(encoding="utf-8"))
    methods = summary["methods"] if include_identity else summary["methods"][1:]
    performance = summary.get("icp_performance", summary.get("performance", {}))
    e2e = performance.get("end_to_end_latency_excluding_first_frame", {})
    rows: list[dict[str, object]] = []
    for method in methods:
        identity = method["method"] == "identity_no_motion_baseline"
        rpe30 = method.get("rpe_frame_delta_30", {})
        rows.append({
            "sequence": _sequence_name(summary["dataset"]),
            "method": method["method"],
            "frames": summary["frames"]["count"],
            "voxel_m": "N/A" if identity else summary["parameters"]["voxel"],
            "max_correspondence_m": "N/A" if identity else summary["parameters"]["max_correspondence"],
            "ate_rmse_m": method["ate"]["rmse_m"],
            "rpe_delta_1_translation_rmse_m": method["rpe_frame_delta_1"]["translation_rmse_m"],
            "rpe_delta_1_rotation_rmse_deg": method["rpe_frame_delta_1"]["rotation_rmse_deg"],
            "rpe_delta_30_translation_rmse_m": rpe30.get("translation_rmse_m", "N/A"),
            "rpe_delta_30_rotation_rmse_deg": rpe30.get("rotation_rmse_deg", "N/A"),
            "end_to_end_mean_ms": "N/A" if identity else 1000 * e2e["mean_s"],
            "end_to_end_p95_ms": "N/A" if identity else 1000 * e2e["p95_s"],
            "fps_from_mean": "N/A" if identity else e2e["fps_from_mean"],
            "rss_peak_bytes": "N/A" if identity else performance["process_rss"]["peak_bytes"],
            "accepted_pairs": "N/A" if identity else performance["accepted_pairs"],
            "rejected_pairs": "N/A" if identity else performance["rejected_or_exception_pairs"],
            "association_protocol": summary["association_protocol"],
            "quality_protocol": "N/A" if identity else summary["quality_protocol"],
            "summary_path": str(path),
        })
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summaries = sorted(args.input.glob("**/summary.json"))
    if not summaries:
        raise RuntimeError(f"No summary.json files found under {args.input}")

    rows: list[dict[str, object]] = []
    identity_seen: set[str] = set()
    for path in summaries:
        summary = json.loads(path.read_text(encoding="utf-8"))
        sequence = _sequence_name(summary["dataset"])
        include_identity = sequence not in identity_seen
        path_rows = rows_from_summary(path, include_identity=include_identity)
        snapshot = args.output / "summaries" / path.relative_to(args.input)
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, snapshot)
        for row in path_rows:
            row["summary_path"] = snapshot.as_posix()
        rows.extend(path_rows)
        identity_seen.add(sequence)

    protocols = {str(row["association_protocol"]) for row in rows}
    if len(protocols) != 1:
        raise RuntimeError(f"Mixed timestamp association protocols: {sorted(protocols)}")
    rows.sort(key=lambda row: (str(row["sequence"]), str(row["method"])))
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "benchmark_table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    headers = ["序列", "方法", "帧数", "ATE(m)", "RPE Δ1(m/°)", "RPE Δ30(m/°)", "均值/p95(ms)", "FPS", "RSS(MB)", "拒绝帧对"]
    lines = [
        "# TUM 多序列固定参数评测",
        "",
        "`fr1/xyz` 用于选择固定配置；`fr1/desk` 与 `fr1/desk2` 只做配置迁移评测。两种 ICP 都固定使用 voxel=0.05 m、correspondence=0.08 m，真值只用于运行后的指标计算。",
        "",
        f"时间戳协议：`{next(iter(protocols))}`。",
        "",
        "| " + " | ".join(headers) + " |",
        "|" + "---|" * len(headers),
    ]
    for row in rows:
        identity = row["method"] == "identity_no_motion_baseline"
        latency = "N/A" if identity else f"{float(row['end_to_end_mean_ms']):.2f}/{float(row['end_to_end_p95_ms']):.2f}"
        fps = "N/A" if identity else f"{float(row['fps_from_mean']):.2f}"
        rss = "N/A" if identity else f"{int(row['rss_peak_bytes']) / 1024**2:.1f}"
        rejected = "N/A" if identity else str(row["rejected_pairs"])
        lines.append(
            f"| {row['sequence']} | {row['method']} | {row['frames']} | {float(row['ate_rmse_m']):.6f} | "
            f"{float(row['rpe_delta_1_translation_rmse_m']):.6f}/{float(row['rpe_delta_1_rotation_rmse_deg']):.3f} | "
            f"{float(row['rpe_delta_30_translation_rmse_m']):.6f}/{float(row['rpe_delta_30_rotation_rmse_deg']):.3f} | "
            f"{latency} | {fps} | {rss} | {rejected} |"
        )
    lines += [
        "",
        "ATE 使用 SE(3) 对齐，不做尺度缩放。RPE 同时报告 Δ=1 和 Δ=30 帧。性能只对实际 ICP 行有意义；静止基线记为 N/A。",
        "",
        "逐次运行的原始 `summary.json` 路径保存在 CSV 中。",
    ]
    (args.output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
