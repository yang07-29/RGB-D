"""Reproducible multi-frame RGB-D ICP odometry experiment for TUM RGB-D data."""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import PIL
import scipy

from .geometry import icp_point_to_point, voxel_downsample
from .metrics import absolute_trajectory_error, align_estimated_poses, compose_camera_to_world, relative_pose_error
from .rgbd import depth_to_points, load_png
from .quality import registration_failure_reason
from .runtime import latency_stats, process_rss_bytes
from .tum import RgbdFrame, load_tum_rgbd_frames, pose_to_tum_row


def make_point_cloud(frame: RgbdFrame, *, stride: int, voxel_size_m: float, min_depth_m: float, max_depth_m: float) -> tuple[np.ndarray, float, float]:
    """Read the RGB-D inputs and produce an odometry point cloud.

    The RGB image is intentionally read but not used by this depth-only baseline;
    doing so makes the recorded input timing genuinely RGB-D end-to-end.
    """
    read_start = time.perf_counter()
    _rgb = load_png(frame.rgb_path)
    depth = load_png(frame.depth_path)
    read_runtime = time.perf_counter() - read_start
    preprocess_start = time.perf_counter()
    points, _ = depth_to_points(depth, stride=stride, min_depth_m=min_depth_m, max_depth_m=max_depth_m)
    cloud = voxel_downsample(points, voxel_size_m)
    return cloud, read_runtime, time.perf_counter() - preprocess_start


def run_identity(frame_count: int) -> list[np.ndarray]:
    return [np.eye(4) for _ in range(frame_count)]


def run_icp(
    clouds: list[np.ndarray],
    *,
    max_iterations: int,
    correspondence_distance_m: float,
    min_correspondence_ratio: float,
    max_acceptable_rmse_m: float,
    rss_samples: list[int],
) -> tuple[list[np.ndarray], list[dict[str, object]]]:
    poses = [np.eye(4)]
    rows: list[dict[str, object]] = []
    for index in range(1, len(clouds)):
        start = time.perf_counter()
        try:
            # Current -> previous.  Identity is a fair frame-to-frame initial guess;
            # no ground-truth pose is used by the odometry solver.
            result = icp_point_to_point(
                clouds[index], clouds[index - 1], max_iterations=max_iterations,
                max_correspondence_distance=correspondence_distance_m,
            )
            reason = registration_failure_reason(
                correspondence_ratio=result.correspondences / len(clouds[index]), residual_rmse_m=result.rmse,
                min_correspondence_ratio=min_correspondence_ratio, max_residual_rmse_m=max_acceptable_rmse_m,
            )
            if reason is not None:
                poses.append(poses[-1].copy())
                rows.append({
                    "pair_index": index, "source_points": len(clouds[index]), "target_points": len(clouds[index - 1]),
                    "status": f"rejected: {reason}", "iterations": result.iterations, "correspondences": result.correspondences,
                    "nearest_neighbor_rmse_m": result.rmse, "registration_runtime_s": time.perf_counter() - start,
                })
                rss = process_rss_bytes()
                if rss is not None:
                    rss_samples.append(rss)
                rows[-1]["process_rss_bytes_after_registration"] = rss
                continue
            current_to_previous = np.eye(4)
            current_to_previous[:3, :3] = result.rotation
            current_to_previous[:3, 3] = result.translation
            poses.append(compose_camera_to_world(poses[-1], current_to_previous))
            rows.append({
                "pair_index": index, "source_points": len(clouds[index]), "target_points": len(clouds[index - 1]),
                "status": "ok", "iterations": result.iterations, "correspondences": result.correspondences,
                "nearest_neighbor_rmse_m": result.rmse, "registration_runtime_s": time.perf_counter() - start,
            })
        except RuntimeError as error:
            # Keep the previous pose so a single registration failure does not abort
            # the experiment; record it explicitly in the CSV and summary.
            poses.append(poses[-1].copy())
            rows.append({
                "pair_index": index, "source_points": len(clouds[index]), "target_points": len(clouds[index - 1]),
                "status": f"failed: {error}", "iterations": 0, "correspondences": 0,
                "nearest_neighbor_rmse_m": None, "registration_runtime_s": time.perf_counter() - start,
            })
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        rows[-1]["process_rss_bytes_after_registration"] = rss
    return poses, rows


def evaluate(name: str, poses: list[np.ndarray], ground_truth: list[np.ndarray]) -> tuple[dict[str, object], list[np.ndarray]]:
    aligned, rotation, translation = align_estimated_poses(poses, ground_truth)
    return {
        "method": name,
        "ate": absolute_trajectory_error(aligned, ground_truth),
        "rpe_frame_delta_1": relative_pose_error(poses, ground_truth, frame_delta=1),
        "alignment_estimated_to_ground_truth": {"rotation": rotation.tolist(), "translation_m": translation.tolist()},
    }, aligned


def write_trajectory(path: Path, frames: list[RgbdFrame], poses: list[np.ndarray]) -> None:
    path.write_text("\n".join(pose_to_tum_row(frame.timestamp, pose) for frame, pose in zip(frames, poses)) + "\n", encoding="utf-8")


def plot_trajectories(path: Path, ground_truth: list[np.ndarray], trajectories: dict[str, list[np.ndarray]]) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    gt = np.array([pose[:3, 3] for pose in ground_truth])
    for axis, dimensions, label in ((axes[0], (0, 1), "x / y"), (axes[1], (0, 2), "x / z")):
        axis.plot(gt[:, dimensions[0]], gt[:, dimensions[1]], label="ground truth", linewidth=2)
        for name, poses in trajectories.items():
            positions = np.array([pose[:3, 3] for pose in poses])
            axis.plot(positions[:, dimensions[0]], positions[:, dimensions[1]], label=name, alpha=0.85)
        axis.set_xlabel(label.split(" /")[0] + " (m)")
        axis.set_ylabel(label.split(" /")[1] + " (m)")
        axis.axis("equal")
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.suptitle("TUM fr1/xyz trajectory (ATE trajectories SE(3)-aligned)")
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("artifacts/tum_fr1_xyz"))
    parser.add_argument("--max-frames", type=int, default=None, help="Use the first N associated frames; default uses all.")
    parser.add_argument("--frame-step", type=int, default=1, help="Use every Nth associated frame.")
    parser.add_argument("--stride", type=int, default=8, help="Depth-image sampling stride in pixels.")
    parser.add_argument("--voxel", type=float, default=0.05, help="Voxel size in metres.")
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--max-correspondence", type=float, default=0.12, help="ICP correspondence gate in metres.")
    parser.add_argument("--min-correspondence-ratio", type=float, default=0.5, help="Reject a result below this inlier/correspondence ratio.")
    parser.add_argument("--max-acceptable-rmse", type=float, default=None, help="Reject a result above this RMSE in metres; default equals max-correspondence.")
    parser.add_argument("--quiet", action="store_true", help="Write all artifacts but do not print the summary JSON.")
    args = parser.parse_args()
    if args.frame_step < 1 or args.stride < 1 or args.voxel <= 0:
        parser.error("frame-step and stride must be positive; voxel must be positive")
    if not 0 < args.min_correspondence_ratio <= 1:
        parser.error("min-correspondence-ratio must be in (0, 1]")
    if args.max_acceptable_rmse is None:
        args.max_acceptable_rmse = args.max_correspondence

    frames = load_tum_rgbd_frames(args.dataset)[::args.frame_step]
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if len(frames) < 3:
        parser.error("At least three associated frames are required")
    args.output.mkdir(parents=True, exist_ok=True)

    total_start = time.perf_counter()
    rss_samples = [rss for rss in [process_rss_bytes()] if rss is not None]
    clouds: list[np.ndarray] = []
    frame_rows: list[dict[str, object]] = []
    for index, frame in enumerate(frames):
        cloud, rgbd_read_runtime, preprocessing_runtime = make_point_cloud(
            frame, stride=args.stride, voxel_size_m=args.voxel, min_depth_m=args.min_depth, max_depth_m=args.max_depth,
        )
        clouds.append(cloud)
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        frame_rows.append({
            "frame_index": index, "rgb_timestamp": frame.timestamp, "rgb_path": str(frame.rgb_path), "depth_path": str(frame.depth_path),
            "rgb_depth_offset_s": frame.depth_time_offset_s, "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s,
            "point_count": len(cloud), "rgbd_read_runtime_s": rgbd_read_runtime, "preprocessing_runtime_s": preprocessing_runtime,
            "process_rss_bytes_after_preprocessing": rss,
        })
    cloud_runtime = sum(float(row["rgbd_read_runtime_s"]) + float(row["preprocessing_runtime_s"]) for row in frame_rows)
    ground_truth = [frame.ground_truth for frame in frames]

    identity_poses = run_identity(len(frames))
    icp_poses, step_rows = run_icp(clouds, max_iterations=args.max_iterations, correspondence_distance_m=args.max_correspondence, min_correspondence_ratio=args.min_correspondence_ratio, max_acceptable_rmse_m=args.max_acceptable_rmse, rss_samples=rss_samples)
    for row in step_rows:
        index = int(row["pair_index"])
        row["rgbd_read_runtime_s"] = frame_rows[index]["rgbd_read_runtime_s"]
        row["preprocessing_runtime_s"] = frame_rows[index]["preprocessing_runtime_s"]
        row["end_to_end_runtime_s"] = float(row["registration_runtime_s"]) + float(row["rgbd_read_runtime_s"]) + float(row["preprocessing_runtime_s"])
    identity_metrics, identity_aligned = evaluate("identity_no_motion_baseline", identity_poses, ground_truth)
    icp_metrics, icp_aligned = evaluate("frame_to_frame_point_to_point_icp", icp_poses, ground_truth)

    write_trajectory(args.output / "trajectory_icp_local.txt", frames, icp_poses)
    write_trajectory(args.output / "trajectory_icp_ate_aligned.txt", frames, icp_aligned)
    write_trajectory(args.output / "trajectory_identity_ate_aligned.txt", frames, identity_aligned)
    with (args.output / "icp_steps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(step_rows[0]))
        writer.writeheader()
        writer.writerows(step_rows)
    with (args.output / "frame_timings.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(frame_rows[0]))
        writer.writeheader()
        writer.writerows(frame_rows)
    plot_trajectories(args.output / "trajectory_xy_xz.png", ground_truth, {"ICP": icp_aligned, "identity baseline": identity_aligned})

    successful = [row for row in step_rows if row["status"] == "ok"]
    final_rss = process_rss_bytes()
    if final_rss is not None:
        rss_samples.append(final_rss)
    summary = {
        "experiment": "TUM RGB-D fr1/xyz multi-frame odometry",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "dataset": str(args.dataset),
        "parameters": vars(args) | {"dataset": str(args.dataset), "output": str(args.output)},
        "frames": {
            "count": len(frames), "first_timestamp": frames[0].timestamp, "last_timestamp": frames[-1].timestamp,
            "mean_rgb_depth_offset_s": float(np.mean([frame.depth_time_offset_s for frame in frames])),
            "mean_rgb_ground_truth_offset_s": float(np.mean([frame.ground_truth_time_offset_s for frame in frames])),
            "point_counts": {"mean": float(np.mean([len(cloud) for cloud in clouds])), "min": int(min(map(len, clouds))), "max": int(max(map(len, clouds)))},
        },
        "methods": [identity_metrics, icp_metrics],
        "icp_performance": {
            "accepted_pairs": len(successful), "rejected_or_exception_pairs": len(step_rows) - len(successful),
            "registration_latency": latency_stats([float(row["registration_runtime_s"]) for row in successful]),
            "rgbd_read_latency": latency_stats([float(row["rgbd_read_runtime_s"]) for row in frame_rows]),
            "preprocessing_latency": latency_stats([float(row["preprocessing_runtime_s"]) for row in frame_rows]),
            "end_to_end_latency_excluding_first_frame": latency_stats([float(row["end_to_end_runtime_s"]) for row in successful]),
            "mean_iterations": float(np.mean([row["iterations"] for row in successful])) if successful else None,
            "total_runtime_s": time.perf_counter() - total_start,
            "cloud_construction_runtime_s": cloud_runtime,
            "process_rss": {"sampling": "before the run, after every frame preprocessing and registration, and at experiment end", "peak_bytes": max(rss_samples) if rss_samples else None, "final_bytes": final_rss},
        },
        "environment": {"python": sys.version, "numpy": np.__version__, "scipy": scipy.__version__, "pillow": PIL.__version__, "platform": platform.platform(), "processor": platform.processor(), "logical_cpu_count": os.cpu_count(), "gpu": "not used (CPU-only NumPy/SciPy baseline)"},
        "artifacts": ["trajectory_icp_local.txt", "trajectory_icp_ate_aligned.txt", "trajectory_identity_ate_aligned.txt", "icp_steps.csv", "frame_timings.csv", "trajectory_xy_xz.png"],
        "notes": ["ICP uses only consecutive depth point clouds and an identity relative-pose initialization.", "A pair is rejected when correspondence ratio is below min-correspondence-ratio or residual RMSE exceeds max-acceptable-rmse; rejected pairs retain the previous pose.", "RGB images are read for end-to-end RGB-D input timing but are not used by this depth-only ICP solver.", "Ground truth is used only after odometry for evaluation and ATE alignment.", "Identity is a no-motion baseline on exactly the same associated frames."],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
