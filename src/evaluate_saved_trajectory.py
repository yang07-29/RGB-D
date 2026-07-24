"""Evaluate a saved TUM-format trajectory with the project's SE(3) ATE/RPE protocol."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .run_odometry import evaluate, plot_trajectories, write_trajectory
from .tum import load_tum_rgbd_frames, read_tum_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", default="saved_rgbd_odometry")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--timestamp-tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    frames = load_tum_rgbd_frames(args.dataset)
    if args.max_frames is not None:
        if args.max_frames < 3:
            parser.error("--max-frames must be at least 3")
        frames = frames[: args.max_frames]
    timestamps, estimated = read_tum_trajectory(args.trajectory)
    if len(estimated) != len(frames):
        parser.error(f"Trajectory has {len(estimated)} poses but dataset association has {len(frames)} frames")
    offsets = np.abs(np.asarray(timestamps) - np.asarray([frame.timestamp for frame in frames]))
    if float(np.max(offsets)) > args.timestamp_tolerance:
        parser.error(
            f"Trajectory timestamps do not match associated RGB frames: max offset={float(np.max(offsets)):.9g}s"
        )

    ground_truth = [frame.ground_truth for frame in frames]
    metrics, aligned = evaluate(args.method, estimated, ground_truth)
    args.output.mkdir(parents=True, exist_ok=True)
    write_trajectory(args.output / "trajectory_ate_aligned.txt", frames, aligned)
    plot_trajectories(args.output / "trajectory_xy_xz.png", ground_truth, {args.method: aligned})
    result = {
        "trajectory": str(args.trajectory),
        "dataset": str(args.dataset),
        "frames": len(frames),
        "timestamp_max_offset_s": float(np.max(offsets)),
        "alignment": "SE(3), no scale",
        "metrics": metrics,
        "artifacts": ["trajectory_ate_aligned.txt", "trajectory_xy_xz.png"],
    }
    (args.output / "evaluation.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
