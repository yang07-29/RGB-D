"""Compare project ATE/RPE values with independently generated evo result archives."""

from __future__ import annotations

import argparse
import io
import json
import shutil
import zipfile
from pathlib import Path

import numpy as np


ARCHIVES = {
    "ate_translation": "ape_translation.zip",
    "rpe_delta1_translation": "rpe_delta1_translation.zip",
    "rpe_delta1_rotation": "rpe_delta1_rotation.zip",
    "rpe_delta30_translation": "rpe_delta30_translation.zip",
    "rpe_delta30_rotation": "rpe_delta30_rotation.zip",
}


def _read_evo_archive(path: Path) -> dict[str, object]:
    with zipfile.ZipFile(path) as archive:
        stats = json.loads(archive.read("stats.json"))
        info = json.loads(archive.read("info.json"))
        errors = np.load(io.BytesIO(archive.read("error_array.npy")), allow_pickle=False)
    return {"stats": stats, "info": info, "samples": int(len(errors))}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evo-results", type=Path, required=True)
    parser.add_argument("--project-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    summary = json.loads(args.project_summary.read_text(encoding="utf-8"))
    method = summary["methods"][1]
    evo = {name: _read_evo_archive(args.evo_results / filename) for name, filename in ARCHIVES.items()}
    project_values = {
        "ate_translation": method["ate"]["rmse_m"],
        "rpe_delta1_translation": method["rpe_frame_delta_1"]["translation_rmse_m"],
        "rpe_delta1_rotation": method["rpe_frame_delta_1"]["rotation_rmse_deg"],
        "rpe_delta30_translation": method["rpe_frame_delta_30"]["translation_rmse_m"],
        "rpe_delta30_rotation": method["rpe_frame_delta_30"]["rotation_rmse_deg"],
    }
    comparisons = {}
    for name, project_value in project_values.items():
        evo_value = float(evo[name]["stats"]["rmse"])
        comparisons[name] = {
            "project_rmse": project_value,
            "evo_rmse": evo_value,
            "absolute_difference": abs(project_value - evo_value),
            "samples": evo[name]["samples"],
        }
    maximum_difference = max(item["absolute_difference"] for item in comparisons.values())
    tolerance = 1e-8
    if maximum_difference > tolerance:
        raise RuntimeError(f"evo cross-check exceeded tolerance: {maximum_difference} > {tolerance}")

    args.output.mkdir(parents=True, exist_ok=True)
    for filename in ARCHIVES.values():
        shutil.copy2(args.evo_results / filename, args.output / filename)
    report = {
        "tool": {"name": "evo", "version": "1.37.1"},
        "estimated_trajectory": evo["ate_translation"]["info"]["est_name"],
        "reference_trajectory": evo["ate_translation"]["info"]["ref_name"],
        "association_max_difference_s": 0.02,
        "ape_alignment": "SE(3) Umeyama, scale correction disabled",
        "rpe_pairing": "frame delta; all overlapping pairs for delta=30",
        "tolerance": tolerance,
        "maximum_absolute_difference": maximum_difference,
        "passed": True,
        "comparisons": comparisons,
    }
    (args.output / "crosscheck.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    labels = {
        "ate_translation": "ATE translation (m)",
        "rpe_delta1_translation": "RPE Δ1 translation (m)",
        "rpe_delta1_rotation": "RPE Δ1 rotation (deg)",
        "rpe_delta30_translation": "RPE Δ30 translation (m)",
        "rpe_delta30_rotation": "RPE Δ30 rotation (deg)",
    }
    lines = [
        "# evo 独立指标核对",
        "",
        "使用 evo 1.37.1 重新读取保存的 TUM 轨迹。ATE 采用 SE(3) Umeyama 对齐，未启用 scale correction；RPE Δ=30 使用全部 760 个重叠帧对。",
        "",
        "| 指标 | 项目实现 | evo | 绝对差 | 样本数 |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, comparison in comparisons.items():
        lines.append(
            f"| {labels[name]} | {comparison['project_rmse']:.12f} | {comparison['evo_rmse']:.12f} | "
            f"{comparison['absolute_difference']:.3e} | {comparison['samples']} |"
        )
    lines += [
        "",
        f"最大绝对差为 `{maximum_difference:.3e}`，低于验收容差 `{tolerance:.1e}`。五个 evo 原始结果包与机器可读核对文件一并保存在本目录。",
    ]
    (args.output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
