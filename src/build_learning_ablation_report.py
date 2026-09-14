"""Aggregate multi-seed loop-retrieval and downstream pose-graph evidence."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


METHODS = (
    "hsv_histogram",
    "mobilenetv3_imagenet_frozen",
    "mobilenetv3_no_se",
    "mobilenetv3_se",
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def mean_std(rows: list[dict[str, object]], key: str) -> dict[str, float]:
    values = np.asarray([float(row[key]) for row in rows])
    return {"mean": float(np.mean(values)), "std": float(np.std(values))}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    retrieval_rows: list[dict[str, object]] = []
    training_records = []
    for seed in args.seeds:
        source = args.input / f"seed_{seed}"
        summary = json.loads((source / "summary.json").read_text(encoding="utf-8"))
        training_records.append(summary)
        for row in read_csv(source / "metrics.csv"):
            retrieval_rows.append({"seed": seed, **row})
        target = args.output / "training_runs" / f"seed_{seed}"
        target.mkdir(parents=True, exist_ok=True)
        for name in ("summary.json", "protocol.json", "metrics.csv", "training_history.csv"):
            shutil.copy2(source / name, target / name)

    pose_rows: list[dict[str, object]] = []
    jobs = [("hsv_histogram", args.seeds[0]), ("mobilenetv3_imagenet_frozen", args.seeds[0])]
    jobs += [(method, seed) for seed in args.seeds for method in ("mobilenetv3_no_se", "mobilenetv3_se")]
    for method, seed in jobs:
        source = args.input / "pose_graph" / f"{method}_seed_{seed}"
        summary = json.loads((source / "summary.json").read_text(encoding="utf-8"))
        labels = summary["posthoc_loop_labels"]
        pose_rows.append({
            "method": method,
            "seed": seed,
            "keyframes": summary["keyframes"],
            "loop_candidates": summary["loop_candidates"],
            "accepted_loop_edges": summary["accepted_loop_edges_before_optimization"],
            "accepted_correct": labels["accepted_correct"],
            "false_accepts": labels["false_accepts"],
            "false_rejects": labels["false_rejects"],
            "ate_before_m": summary["keyframe_before"]["ate"]["rmse_m"],
            "ate_after_m": summary["keyframe_after"]["ate"]["rmse_m"],
            "rpe_delta1_translation_m": summary["keyframe_after"]["rpe_frame_delta_1"]["translation_rmse_m"],
            "rpe_delta1_rotation_deg": summary["keyframe_after"]["rpe_frame_delta_1"]["rotation_rmse_deg"],
        })
        target = args.output / "pose_graph_runs" / f"{method}_seed_{seed}"
        target.mkdir(parents=True, exist_ok=True)
        for name in ("summary.json", "loop_candidates.csv", "hard_negative_candidates.csv"):
            shutil.copy2(source / name, target / name)

    write_csv(args.output / "retrieval_runs.csv", retrieval_rows)
    write_csv(args.output / "pose_graph_runs.csv", pose_rows)

    retrieval_aggregate = {}
    for method in METHODS:
        rows = [row for row in retrieval_rows if row["method"] == method]
        retrieval_aggregate[method] = {
            "runs": len(rows),
            **{key: mean_std(rows, key) for key in (
                "test_recall_at_1", "test_recall_at_5", "test_precision", "test_recall", "test_f1",
            )},
        }
    pose_aggregate = {}
    for method in METHODS:
        rows = [row for row in pose_rows if row["method"] == method]
        pose_aggregate[method] = {
            "runs": len(rows),
            "ate_before_m": mean_std(rows, "ate_before_m"),
            "ate_after_m": mean_std(rows, "ate_after_m"),
            "false_accepts": mean_std(rows, "false_accepts"),
            "false_rejects": mean_std(rows, "false_rejects"),
        }
    summary = {
        "experiment": "multi-seed loop descriptor ablation with downstream pose graphs",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "association_protocol": "one_to_one_minimum_offset_greedy_v2",
        "seeds": args.seeds,
        "retrieval": retrieval_aggregate,
        "pose_graph": pose_aggregate,
        "environment": training_records[0]["environment"],
        "protocol": training_records[0]["protocol"],
        "notes": [
            "HSV and frozen ImageNet features are deterministic baselines; they are evaluated for all seeds but their downstream pose graph is run once.",
            "The frozen ImageNet baseline has no triplet training or random projection.",
            "No-SE and SE use the same data, geometric-hard-negative policy, hyperparameters, and three seeds.",
            "Some task-trained runs reached near-zero triplet training loss while validation retrieval remained non-monotonic, so the current mining policy is not treated as solved.",
            "Ground truth is used for supervision, validation threshold selection, and post-hoc metrics only.",
        ],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    x = np.arange(len(METHODS))
    short = ("HSV", "ImageNet frozen", "MobileNet no SE", "MobileNet + SE")
    recall_mean = [retrieval_aggregate[method]["test_recall_at_1"]["mean"] for method in METHODS]
    recall_std = [retrieval_aggregate[method]["test_recall_at_1"]["std"] for method in METHODS]
    axes[0].bar(x, recall_mean, yerr=recall_std, capsize=4)
    axes[0].set_xticks(x, short, rotation=15)
    axes[0].set_ylabel("Test Recall@1")
    axes[0].set_ylim(0, 1.05)
    axes[0].grid(axis="y", alpha=0.3)
    ate_mean = [pose_aggregate[method]["ate_after_m"]["mean"] for method in METHODS]
    ate_std = [pose_aggregate[method]["ate_after_m"]["std"] for method in METHODS]
    axes[1].bar(x, ate_mean, yerr=ate_std, capsize=4)
    axes[1].set_xticks(x, short, rotation=15)
    axes[1].set_ylabel("Pose-graph ATE RMSE (m)")
    axes[1].grid(axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(args.output / "retrieval_and_ate_ablation.png", dpi=180)
    plt.close(figure)

    readme = """# 三随机种子回环描述子消融

训练、验证、测试分别使用 fr1/desk、fr1/desk2、fr1/xyz，一对一关联且按序列隔离。每种方法只在验证序列选阈值；测试真值不参与候选生成或几何接收。

| 方法 | Recall@1 | Recall@5 | pair F1 | 位姿图 ATE（m） |
| --- | ---: | ---: | ---: | ---: |
"""
    labels = {
        "hsv_histogram": "HSV 直方图",
        "mobilenetv3_imagenet_frozen": "ImageNet 冻结特征",
        "mobilenetv3_no_se": "MobileNetV3 无 SE",
        "mobilenetv3_se": "MobileNetV3 + SE",
    }
    for method in METHODS:
        retrieval = retrieval_aggregate[method]
        pose = pose_aggregate[method]
        readme += (
            f"| {labels[method]} | {retrieval['test_recall_at_1']['mean']:.4f} ± {retrieval['test_recall_at_1']['std']:.4f} "
            f"| {retrieval['test_recall_at_5']['mean']:.4f} ± {retrieval['test_recall_at_5']['std']:.4f} "
            f"| {retrieval['test_f1']['mean']:.4f} ± {retrieval['test_f1']['std']:.4f} "
            f"| {pose['ate_after_m']['mean']:.6f} ± {pose['ate_after_m']['std']:.6f} |\n"
        )
    readme += """

HSV 和冻结 ImageNet 特征不训练；冻结特征直接对 MobileNetV3-Small 的 576 维全局池化输出做 L2 归一化，没有随机投影。两种任务训练模型使用完全相同的数据、三元组数量、几何困难负例策略、超参数和三个随机种子，差别只在是否保留 SE。

表中的“±”是三次种子的总体标准差；HSV/冻结特征本身是确定性的，因此其检索标准差应为 0，位姿图只运行一次。`retrieval_runs.csv` 和 `pose_graph_runs.csv` 保留每次原始结果，checkpoint 只保存在本地 artifacts，summary 中记录哈希。

部分训练在后期出现接近 0 的 triplet loss，但验证检索指标并未单调改善，说明当前几何困难负例仍可能过易。checkpoint 严格按验证 Recall@5、Recall@1、F1 的顺序选择，测试集没有用于选 epoch。
"""
    (args.output / "README.md").write_text(readme, encoding="utf-8")


if __name__ == "__main__":
    main()
