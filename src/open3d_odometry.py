"""Open3D point-to-plane ICP odometry using the project's TUM evaluation protocol."""

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

import matplotlib
import numpy as np
import open3d as o3d
import PIL
import scipy

from .metrics import align_estimated_poses, absolute_trajectory_error, compose_camera_to_world, relative_pose_error
from .quality import registration_failure_reason
from .run_odometry import evaluate, make_point_cloud, plot_trajectories, run_identity, write_trajectory
from .runtime import latency_stats, process_rss_bytes
from .tum import load_tum_rgbd_frames


def build_clouds_with_normals(frames, args, rss_samples):
    clouds = []
    rows = []
    for index, frame in enumerate(frames):
        points, read_runtime, base_preprocess_runtime = make_point_cloud(
            frame, stride=args.stride, voxel_size_m=args.voxel, min_depth_m=args.min_depth, max_depth_m=args.max_depth,
        )
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        normal_start = time.perf_counter()
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.normal_radius, max_nn=args.normal_max_nn))
        normal_runtime = time.perf_counter() - normal_start
        clouds.append(cloud)
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        rows.append({
            "frame_index": index, "rgb_timestamp": frame.timestamp, "rgb_path": str(frame.rgb_path), "depth_path": str(frame.depth_path),
            "rgb_depth_offset_s": frame.depth_time_offset_s, "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s,
            "point_count": len(points), "rgbd_read_runtime_s": read_runtime, "base_preprocessing_runtime_s": base_preprocess_runtime,
            "normal_estimation_runtime_s": normal_runtime, "preprocessing_runtime_s": base_preprocess_runtime + normal_runtime,
            "process_rss_bytes_after_preprocessing": rss,
        })
    return clouds, rows


def run_point_to_plane(clouds, args, rss_samples):
    poses = [np.eye(4)]
    rows = []
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=args.max_iterations)
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    for index in range(1, len(clouds)):
        start = time.perf_counter()
        try:
            result = o3d.pipelines.registration.registration_icp(
                clouds[index], clouds[index - 1], args.max_correspondence, np.eye(4), estimation, criteria,
            )
            source_points = len(clouds[index].points)
            reason = registration_failure_reason(
                correspondence_ratio=float(result.fitness), residual_rmse_m=float(result.inlier_rmse),
                min_correspondence_ratio=args.min_correspondence_ratio, max_residual_rmse_m=args.max_acceptable_rmse,
            )
            runtime = time.perf_counter() - start
            if reason is None:
                poses.append(compose_camera_to_world(poses[-1], np.asarray(result.transformation)))
                status = "ok"
            else:
                poses.append(poses[-1].copy())
                status = f"rejected: {reason}"
            rss = process_rss_bytes()
            if rss is not None:
                rss_samples.append(rss)
            rows.append({
                "pair_index": index, "source_points": source_points, "target_points": len(clouds[index - 1].points),
                "status": status, "fitness": float(result.fitness), "inlier_rmse_m": float(result.inlier_rmse),
                "registration_runtime_s": runtime, "process_rss_bytes_after_registration": rss,
            })
        except RuntimeError as error:
            poses.append(poses[-1].copy())
            rss = process_rss_bytes()
            if rss is not None:
                rss_samples.append(rss)
            rows.append({
                "pair_index": index, "source_points": len(clouds[index].points), "target_points": len(clouds[index - 1].points),
                "status": f"exception: {error}", "fitness": None, "inlier_rmse_m": None,
                "registration_runtime_s": time.perf_counter() - start, "process_rss_bytes_after_registration": rss,
            })
    return poses, rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("artifacts/tum_fr1_xyz_open3d_point_to_plane"))
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--normal-radius", type=float, default=0.10)
    parser.add_argument("--normal-max-nn", type=int, default=30)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--max-correspondence", type=float, default=0.12)
    parser.add_argument("--min-correspondence-ratio", type=float, default=0.5)
    parser.add_argument("--max-acceptable-rmse", type=float, default=None)
    parser.add_argument("--quiet", action="store_true", help="Write all artifacts but do not print the summary JSON.")
    args = parser.parse_args()
    if args.max_acceptable_rmse is None:
        args.max_acceptable_rmse = args.max_correspondence
    if args.frame_step < 1 or args.stride < 1 or args.voxel <= 0 or args.normal_radius <= 0:
        parser.error("frame-step, stride, voxel, and normal-radius must be positive")

    frames = load_tum_rgbd_frames(args.dataset)[::args.frame_step]
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if len(frames) < 3:
        parser.error("At least three associated frames are required")
    args.output.mkdir(parents=True, exist_ok=True)
    total_start = time.perf_counter()
    rss_samples = [rss for rss in [process_rss_bytes()] if rss is not None]
    clouds, frame_rows = build_clouds_with_normals(frames, args, rss_samples)
    ground_truth = [frame.ground_truth for frame in frames]
    poses, pair_rows = run_point_to_plane(clouds, args, rss_samples)
    for row in pair_rows:
        frame_row = frame_rows[int(row["pair_index"])]
        row["rgbd_read_runtime_s"] = frame_row["rgbd_read_runtime_s"]
        row["preprocessing_runtime_s"] = frame_row["preprocessing_runtime_s"]
        row["end_to_end_runtime_s"] = float(row["registration_runtime_s"]) + float(row["rgbd_read_runtime_s"]) + float(row["preprocessing_runtime_s"])
    identity_metrics, identity_aligned = evaluate("identity_no_motion_baseline", run_identity(len(frames)), ground_truth)
    metrics, aligned = evaluate("open3d_point_to_plane_icp", poses, ground_truth)
    write_trajectory(args.output / "trajectory_open3d_local.txt", frames, poses)
    write_trajectory(args.output / "trajectory_open3d_ate_aligned.txt", frames, aligned)
    write_trajectory(args.output / "trajectory_identity_ate_aligned.txt", frames, identity_aligned)
    for name, rows in (("frame_timings.csv", frame_rows), ("open3d_steps.csv", pair_rows)):
        with (args.output / name).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    plot_trajectories(args.output / "trajectory_xy_xz.png", ground_truth, {"Open3D point-to-plane ICP": aligned, "identity baseline": identity_aligned})
    accepted = [row for row in pair_rows if row["status"] == "ok"]
    final_rss = process_rss_bytes()
    if final_rss is not None:
        rss_samples.append(final_rss)
    summary = {
        "experiment": "TUM RGB-D fr1/xyz Open3D point-to-plane ICP odometry",
        "created_utc": datetime.now(timezone.utc).isoformat(), "dataset": str(args.dataset),
        "parameters": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "frames": {"count": len(frames), "first_timestamp": frames[0].timestamp, "last_timestamp": frames[-1].timestamp,
                   "mean_rgb_depth_offset_s": float(np.mean([frame.depth_time_offset_s for frame in frames])),
                   "mean_rgb_ground_truth_offset_s": float(np.mean([frame.ground_truth_time_offset_s for frame in frames])),
                   "point_counts": {"mean": float(np.mean([len(cloud.points) for cloud in clouds])), "min": min(map(lambda cloud: len(cloud.points), clouds)), "max": max(map(lambda cloud: len(cloud.points), clouds))}},
        "methods": [identity_metrics, metrics],
        "performance": {"accepted_pairs": len(accepted), "rejected_or_exception_pairs": len(pair_rows) - len(accepted),
                        "rgbd_read_latency": latency_stats([float(row["rgbd_read_runtime_s"]) for row in frame_rows]),
                        "preprocessing_including_normals_latency": latency_stats([float(row["preprocessing_runtime_s"]) for row in frame_rows]),
                        "registration_latency": latency_stats([float(row["registration_runtime_s"]) for row in accepted]),
                        "end_to_end_latency_excluding_first_frame": latency_stats([float(row["end_to_end_runtime_s"]) for row in accepted]),
                        "process_rss": {"sampling": "before run, after every preprocessing/registration, and at experiment end", "peak_bytes": max(rss_samples) if rss_samples else None, "final_bytes": final_rss},
                        "total_runtime_s": time.perf_counter() - total_start},
        "environment": {"python": sys.version, "open3d": o3d.__version__, "numpy": np.__version__, "scipy": scipy.__version__, "pillow": PIL.__version__, "matplotlib": matplotlib.__version__, "platform": platform.platform(), "processor": platform.processor(), "logical_cpu_count": os.cpu_count(), "gpu": "not used"},
        "notes": ["Uses exactly the same deterministic association and ATE/RPE functions as the NumPy baseline.", "Ground truth is evaluation-only.", "A pair is rejected for low Open3D fitness or high Open3D inlier RMSE; rejected pairs retain the previous pose."],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
