"""Aggregate tracking-loss stress runs and preserve their summaries."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    paths = sorted(args.input.glob("**/summary.json"))
    if len(paths) != 6:
        raise RuntimeError(f"Expected 6 stress summaries, found {len(paths)}")
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in paths:
        summary = json.loads(path.read_text(encoding="utf-8"))
        method = summary["methods"][1]
        performance = summary["icp_performance"]
        tracking = summary["tracking"]
        snapshot = args.output / "summaries" / path.relative_to(args.input)
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, snapshot)
        rows.append({
            "frame_step": summary["parameters"]["frame_step"],
            "skipped_between_inputs": summary["parameters"]["frame_step"] - 1,
            "tracking_mode": summary["parameters"]["tracking_mode"],
            "frames": summary["frames"]["count"],
            "ate_rmse_m": method["ate"]["rmse_m"],
            "rpe_translation_rmse_m": method["rpe_frame_delta_1"]["translation_rmse_m"],
            "rpe_rotation_rmse_deg": method["rpe_frame_delta_1"]["rotation_rmse_deg"],
            "accepted_pairs": performance["accepted_pairs"],
            "rejected_pairs": performance["rejected_or_exception_pairs"],
            "lost_events": tracking["lost_events"],
            "recovered_lost_events": tracking["recovered_lost_events"],
            "unrecovered_lost_events": tracking["unrecovered_lost_events"],
            "recovery_attempts": tracking["recovery_attempts"],
            "successful_recovery_attempts": tracking["successful_recovery_attempts"],
            "recovery_attempt_success_rate": tracking["recovery_attempt_success_rate"],
            "mean_lost_event_recovery_frames": tracking["mean_lost_event_recovery_frames"],
            "end_to_end_mean_ms": 1000 * performance["end_to_end_latency_excluding_first_frame"]["mean_s"],
            "end_to_end_p95_ms": 1000 * performance["end_to_end_latency_excluding_first_frame"]["p95_s"],
            "summary_path": snapshot.as_posix(),
        })
    rows.sort(key=lambda row: (int(row["frame_step"]), str(row["tracking_mode"])))
    with (args.output / "benchmark_table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# 跟踪丢失与恢复压力实验",
        "",
        "数据为 TUM fr1/xyz 一对一关联轨迹。frame step=2/3 分别人为跳过 1/2 帧，用来增大相邻输入运动和降低重叠。两种模式使用相同点云、门限和最终质量判据。",
        "",
        "| step | 模式 | 帧数 | ATE(m) | RPE(m/°) | 接受/拒绝 | 丢失/恢复/未恢复 | 平均恢复帧数 | 恢复尝试成功率 | mean/p95(ms) |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        success_rate = row["recovery_attempt_success_rate"]
        rate_text = "N/A" if success_rate is None else f"{100 * float(success_rate):.1f}%"
        recovery_frames = row["mean_lost_event_recovery_frames"]
        recovery_frames_text = "N/A" if recovery_frames is None else f"{float(recovery_frames):.2f}"
        lines.append(
            f"| {row['frame_step']} | {row['tracking_mode']} | {row['frames']} | {float(row['ate_rmse_m']):.6f} | "
            f"{float(row['rpe_translation_rmse_m']):.6f}/{float(row['rpe_rotation_rmse_deg']):.3f} | "
            f"{row['accepted_pairs']}/{row['rejected_pairs']} | {row['lost_events']}/{row['recovered_lost_events']}/{row['unrecovered_lost_events']} | "
            f"{recovery_frames_text} | "
            f"{row['successful_recovery_attempts']}/{row['recovery_attempts']} ({rate_text}) | "
            f"{float(row['end_to_end_mean_ms']):.2f}/{float(row['end_to_end_p95_ms']):.2f} |"
        )
    lines += [
        "",
        "丢失事件定义为连续 2 个最终拒绝帧。恢复模式优先保留单位初值基线；失败后才使用恒速初值、粗到细 ICP、最近有效关键帧和内部暂定位姿链。被拒绝帧在输出轨迹中仍保持上一有效位姿。",
        "",
        "结果并非所有压力等级都改善：正常 step=1 与基线轨迹完全一致但更慢；step=2 ATE/RPE 略差；step=3 ATE 下降约 18%，但短间隔 RPE 变差。该策略作为可审计的实验分支保留，不替换默认 identity 基线。",
    ]
    (args.output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
