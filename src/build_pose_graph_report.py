"""Publish compact, auditable pose-graph evidence from a completed run."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


FILES = (
    "summary.json",
    "keyframes.csv",
    "sequential_edges.csv",
    "loop_candidates.csv",
    "hard_negative_candidates.csv",
    "pose_graph_optimized.json",
    "trajectory_keyframes_before.txt",
    "trajectory_keyframes_after.txt",
    "trajectory_keyframes_forced_bad_edge.txt",
    "trajectory_keyframes_before_after.png",
    "loop_correct_and_rejected_cases.png",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads((args.input / "summary.json").read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        source = args.input / name
        if source.is_file():
            shutil.copy2(source, args.output / name)

    before = summary["keyframe_before"]
    after = summary["keyframe_after"]
    bad = summary["forced_bad_edge_stress"]["metrics_after_forced_insertion"]
    labels = summary["posthoc_loop_labels"]
    sequential = summary["sequential_edge_quality"]
    hard = summary["hard_negative_stress"]
    lines = [
        "# 一对一协议下的位姿图边质量实验",
        "",
        f"输入为 TUM fr1/xyz 的 {summary['frames']} 帧一对一关联，选出 {summary['keyframes']} 个关键帧。顺序边和回环边都记录共享最终变换质量；真值只在优化结束后计算轨迹指标和候选标签。",
        "",
        "| 轨迹 | ATE(m) | RPE Δ1(m/°) |",
        "| --- | ---: | ---: |",
        f"| 优化前 | {before['ate']['rmse_m']:.6f} | {before['rpe_frame_delta_1']['translation_rmse_m']:.6f}/{before['rpe_frame_delta_1']['rotation_rmse_deg']:.3f} |",
        f"| 通过门限的位姿图 | {after['ate']['rmse_m']:.6f} | {after['rpe_frame_delta_1']['translation_rmse_m']:.6f}/{after['rpe_frame_delta_1']['rotation_rmse_deg']:.3f} |",
    ]
    if bad is not None:
        lines.append(
            f"| 强制加入 1 条错误边 | {bad['ate']['rmse_m']:.6f} | {bad['rpe_frame_delta_1']['translation_rmse_m']:.6f}/{bad['rpe_frame_delta_1']['rotation_rmse_deg']:.3f} |"
        )
    lines += [
        "",
        f"顺序边共 {summary['odometry_edges']} 条：{sequential['accepted_keyframe_icp']} 条通过共享质量门限；{sequential['odometry_prediction_fallback']} 条没有把低质量 ICP 当成确定边，而是回退到已评测的逐帧里程计相对位姿。",
        "",
        f"正式回环候选 {summary['loop_candidates']} 条：接受正确/接受错误/拒绝正确/拒绝错误 = {labels['accepted_correct']}/{labels['accepted_incorrect']}/{labels['rejected_correct']}/{labels['rejected_incorrect']}。误接受 {labels['false_accepts']}，误拒绝 {labels['false_rejects']}。",
        "",
        f"另测试 {hard['candidates']} 个不入图的 hard negative，其中 {hard['rejected_by_same_gates']} 个被相同门限拒绝，事后真值标为错误的有 {hard['posthoc_incorrect']} 个。强制错误边实验明确绕过生产门限，仅用于量化错误约束对优化的影响。",
        "",
        "逐边指标、拒绝原因、轨迹和 Open3D 位姿图 JSON 均保存在本目录；大点云地图留在本地 artifacts，不提交 Git。",
    ]
    (args.output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
