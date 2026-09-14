"""Aggregate repeated streaming odometry runs and publish latency/RSS evidence."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_run(path: Path) -> tuple[dict[str, object], list[dict[str, str]]]:
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    with (path / "frame_timings.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return summary, rows


def table_row(name: str, summary: dict[str, object]) -> dict[str, object]:
    performance = summary.get("performance", summary.get("icp_performance"))
    steady = performance["timing_protocol_v2"]["steady_state_after_warmup"]
    rss = performance["rss_trend_v2"]
    methods = summary["methods"]
    estimate = next((method for method in methods if method["method"] != "identity_no_motion_baseline"), None)
    return {
        "run": name,
        "frames": summary["frames"]["count"],
        "evaluation_available": summary["evaluation"]["available"],
        "ate_rmse_m": estimate["ate"]["rmse_m"] if estimate else None,
        "input_to_pose_mean_ms": steady["input_to_pose_latency"]["mean_s"] * 1000.0,
        "input_to_pose_median_ms": steady["input_to_pose_latency"]["median_s"] * 1000.0,
        "input_to_pose_p95_ms": steady["input_to_pose_latency"]["p95_s"] * 1000.0,
        "input_to_pose_fps": steady["input_to_pose_latency"]["fps_from_mean"],
        "compute_mean_ms": steady["compute_latency"]["mean_s"] * 1000.0,
        "compute_p95_ms": steady["compute_latency"]["p95_s"] * 1000.0,
        "peak_rss_bytes": rss["peak_bytes"],
        "steady_rss_growth_bytes": rss["steady_state_growth_bytes"],
        "rss_slope_bytes_per_frame": rss["steady_state_linear_slope_bytes_per_frame"],
    }


def plot_curves(output: Path, runs: list[tuple[str, list[dict[str, str]]]]) -> None:
    latency_figure, latency_axis = plt.subplots(figsize=(10, 4.8))
    rss_figure, rss_axis = plt.subplots(figsize=(10, 4.8))
    for name, rows in runs:
        frame = np.asarray([int(row["frame_index"]) for row in rows])
        latency_ms = np.asarray([float(row["input_to_pose_runtime_s"]) * 1000.0 for row in rows])
        rss_mib = np.asarray([float(row["process_rss_bytes_after_frame"]) / 2**20 for row in rows])
        window = min(50, len(rows))
        kernel = np.ones(window) / window
        rolling = np.convolve(latency_ms, kernel, mode="valid")
        latency_axis.plot(frame[window - 1:], rolling, label=f"{name} ({window}-frame moving mean)")
        rss_axis.plot(frame, rss_mib, label=name, alpha=0.85)
    latency_axis.set_xlabel("Processed frame index")
    latency_axis.set_ylabel("Input-to-pose latency (ms)")
    latency_axis.grid(True, alpha=0.3)
    latency_axis.legend()
    latency_figure.tight_layout()
    latency_figure.savefig(output / "latency_curve.png", dpi=170)
    plt.close(latency_figure)

    rss_axis.set_xlabel("Processed frame index")
    rss_axis.set_ylabel("Process RSS (MiB)")
    rss_axis.grid(True, alpha=0.3)
    rss_axis.legend()
    rss_figure.tight_layout()
    rss_figure.savefig(output / "rss_curve.png", dpi=170)
    plt.close(rss_figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    loaded = []
    table = []
    method_rows: dict[str, list[dict[str, object]]] = {}
    for method in ("numpy", "open3d"):
        method_rows[method] = []
        for index in range(1, args.repeats + 1):
            run_name = f"run_{index}"
            label = f"{method}_{run_name}"
            summary, rows = load_run(args.input / method / run_name)
            loaded.append((label, rows))
            row = table_row(run_name, summary)
            row = {"method": method, **row}
            table.append(row)
            method_rows[method].append(row)
            target = args.output / "runs" / method / run_name
            target.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.input / method / run_name / "summary.json", target / "summary.json")
            shutil.copy2(args.input / method / run_name / "frame_timings.csv", target / "frame_timings.csv")

    soak_summary, soak_rows = load_run(args.input / "open3d" / "soak_3x")
    table.append({"method": "open3d", **table_row("soak_3x", soak_summary)})
    loaded.append(("open3d_soak_3x", soak_rows))
    soak_target = args.output / "runs" / "open3d" / "soak_3x"
    soak_target.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.input / "soak_3x" / "summary.json", soak_target / "summary.json")
    shutil.copy2(args.input / "soak_3x" / "frame_timings.csv", soak_target / "frame_timings.csv")

    with (args.output / "benchmark_table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    plot_curves(args.output, loaded)

    repeat_aggregates = {}
    for method, repeats in method_rows.items():
        mean_latency = np.asarray([float(row["input_to_pose_mean_ms"]) for row in repeats])
        p95_latency = np.asarray([float(row["input_to_pose_p95_ms"]) for row in repeats])
        peak_rss = np.asarray([int(row["peak_rss_bytes"]) for row in repeats])
        repeat_aggregates[method] = {
            "input_to_pose_mean_ms": {
                "mean": float(np.mean(mean_latency)),
                "std": float(np.std(mean_latency)),
                "min": float(np.min(mean_latency)),
                "max": float(np.max(mean_latency)),
            },
            "input_to_pose_p95_ms": {
                "mean": float(np.mean(p95_latency)),
                "std": float(np.std(p95_latency)),
            },
            "peak_rss_mib": {
                "mean": float(np.mean(peak_rss) / 2**20),
                "min": float(np.min(peak_rss) / 2**20),
                "max": float(np.max(peak_rss) / 2**20),
            },
        }
    soak_repeat_count = int(soak_summary["parameters"]["input_repeats"])
    soak_base_frames = int(soak_summary["frames"]["count"]) // soak_repeat_count
    soak_cycle_rss = []
    for cycle in range(soak_repeat_count):
        cycle_rows = soak_rows[cycle * soak_base_frames:(cycle + 1) * soak_base_frames]
        start_rss = int(cycle_rows[0]["process_rss_bytes_after_frame"])
        end_rss = int(cycle_rows[-1]["process_rss_bytes_after_frame"])
        soak_cycle_rss.append({
            "cycle": cycle + 1,
            "frames": len(cycle_rows),
            "start_bytes": start_rss,
            "end_bytes": end_rss,
            "growth_bytes": end_rss - start_rss,
        })
    aggregate = {
        "experiment": "Open3D streaming performance protocol v2",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "independent_full_sequence_runs": args.repeats,
        "frames_per_run": int(method_rows["open3d"][0]["frames"]),
        "methods": repeat_aggregates,
        "soak_run": table[-1],
        "soak_input_frames_per_cycle": soak_base_frames,
        "soak_cycle_rss": soak_cycle_rss,
        "scope": soak_summary["performance"]["timing_protocol_v2"]["measurement_scope"],
        "notes": [
            "Each repeat is a fresh process; both methods use the same frozen 790-frame association and parameters.",
            f"The soak run replays all {soak_base_frames} RGB-D-only pairs three times with monotonic synthetic timestamps; it is performance-only and has no ATE/RPE.",
            "RSS is sampled after each frame and outside the input-to-pose timer.",
        ],
    }
    (args.output / "summary.json").write_text(json.dumps(aggregate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    soak = table[-1]
    readme = f"""# 流式性能与长时间回放

NumPy point-to-point 与 Open3D point-to-plane 都使用固定参数 `voxel=0.05 m`、`correspondence=0.08 m`。两个程序都只保留前一帧点云；每次 790 帧正式运行都是独立进程，前 30 帧保留在轨迹中但不计入稳态延迟。

| 方法 / 运行 | 帧数 | ATE（m） | 输入到位姿 mean / p95（ms） | FPS | 峰值 RSS（MiB） |
| --- | ---: | ---: | ---: | ---: | ---: |
"""
    for row in table:
        ate = f"{float(row['ate_rmse_m']):.6f}" if row["ate_rmse_m"] is not None else "N/A"
        readme += f"| {row['method']} / {row['run']} | {row['frames']} | {ate} | {float(row['input_to_pose_mean_ms']):.2f} / {float(row['input_to_pose_p95_ms']):.2f} | {float(row['input_to_pose_fps']):.2f} | {int(row['peak_rss_bytes']) / 2**20:.1f} |\n"
    readme += f"""

三次正式运行的 mean 延迟为 NumPy `{aggregate['methods']['numpy']['input_to_pose_mean_ms']['mean']:.2f}±{aggregate['methods']['numpy']['input_to_pose_mean_ms']['std']:.2f} ms`、Open3D `{aggregate['methods']['open3d']['input_to_pose_mean_ms']['mean']:.2f}±{aggregate['methods']['open3d']['input_to_pose_mean_ms']['std']:.2f} ms`。Open3D 长时间压力运行使用无真值条件下的全部 {soak_base_frames} 对 RGB-D，循环读取三遍，共 {soak['frames']} 帧；它只用于性能和内存检查，不计算 ATE/RPE。稳态 RSS 首尾增加 `{int(soak['steady_rss_growth_bytes']) / 2**20:.2f} MiB`，线性拟合约 `{float(soak['rss_slope_bytes_per_frame']) / 1024:.2f} KiB/帧`，因此当前结果不能声称 RSS 已完全稳定。流式代码没有保存历史点云，但 Open3D/系统分配器的常驻内存仍需后续继续定位。

“输入到位姿”包含 RGB/depth 解码、反投影/下采样/法线和 ICP/质量判定/位姿累积；不包含时间戳关联、真值评测、绘图、结果写盘和 RSS 采样。原始逐帧 CSV、各次 summary 和两张曲线图都保存在本目录。
"""
    (args.output / "README.md").write_text(readme, encoding="utf-8")


if __name__ == "__main__":
    main()
