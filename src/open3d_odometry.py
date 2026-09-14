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
from .performance_protocol import repeat_rgbd_pairs_for_performance, summarize_frame_performance, summarize_rss_trend
from .quality import evaluate_registration_quality, registration_failure_reason
from .run_odometry import evaluate, make_point_cloud, plot_trajectories, run_identity, write_trajectory
from .runtime import latency_stats, process_rss_bytes
from .tum import RgbdFrame, associate_ground_truth, load_tum_rgbd_pairs


def make_open3d_cloud(frame, args):
    points, read_runtime, base_preprocess_runtime = make_point_cloud(
        frame, stride=args.stride, voxel_size_m=args.voxel, min_depth_m=args.min_depth, max_depth_m=args.max_depth,
    )
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    normal_start = time.perf_counter()
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.normal_radius, max_nn=args.normal_max_nn))
    normal_runtime = time.perf_counter() - normal_start
    return cloud, read_runtime, base_preprocess_runtime, normal_runtime


def build_clouds_with_normals(frames, args, rss_samples):
    clouds = []
    rows = []
    for index, frame in enumerate(frames):
        cloud, read_runtime, base_preprocess_runtime, normal_runtime = make_open3d_cloud(frame, args)
        clouds.append(cloud)
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        rows.append({
            "frame_index": index, "rgb_timestamp": frame.timestamp, "rgb_path": str(frame.rgb_path), "depth_path": str(frame.depth_path),
            "rgb_depth_offset_s": frame.depth_time_offset_s,
            "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s if isinstance(frame, RgbdFrame) else None,
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
            quality = evaluate_registration_quality(
                np.asarray(clouds[index].points), np.asarray(clouds[index - 1].points), np.asarray(result.transformation),
                max_correspondence_m=args.max_correspondence,
            )
            reason = registration_failure_reason(
                correspondence_ratio=quality.correspondence_ratio, residual_rmse_m=quality.all_point_rmse_m,
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
                "shared_correspondences": quality.correspondences,
                "shared_correspondence_ratio": quality.correspondence_ratio,
                "shared_inlier_rmse_m": quality.inlier_rmse_m,
                "shared_all_point_rmse_m": quality.all_point_rmse_m,
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
                "shared_correspondences": 0, "shared_correspondence_ratio": 0.0,
                "shared_inlier_rmse_m": None, "shared_all_point_rmse_m": None,
                "registration_runtime_s": time.perf_counter() - start, "process_rss_bytes_after_registration": rss,
            })
    return poses, rows


def run_streaming_point_to_plane(frames, args, rss_samples):
    """Run point-to-plane ICP while retaining only the previous point cloud."""
    poses = [np.eye(4)]
    frame_rows = []
    pair_rows = []
    point_counts = []
    previous_cloud = None
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=args.max_iterations)
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    for index, frame in enumerate(frames):
        cloud, read_runtime, base_preprocess_runtime, normal_runtime = make_open3d_cloud(frame, args)
        preprocessing_runtime = base_preprocess_runtime + normal_runtime
        registration_runtime = 0.0
        pair_row = None
        if previous_cloud is not None:
            registration_start = time.perf_counter()
            try:
                result = o3d.pipelines.registration.registration_icp(
                    cloud, previous_cloud, args.max_correspondence, np.eye(4), estimation, criteria,
                )
                quality = evaluate_registration_quality(
                    np.asarray(cloud.points), np.asarray(previous_cloud.points), np.asarray(result.transformation),
                    max_correspondence_m=args.max_correspondence,
                )
                reason = registration_failure_reason(
                    correspondence_ratio=quality.correspondence_ratio,
                    residual_rmse_m=quality.all_point_rmse_m,
                    min_correspondence_ratio=args.min_correspondence_ratio,
                    max_residual_rmse_m=args.max_acceptable_rmse,
                )
                accepted = reason is None
                poses.append(
                    compose_camera_to_world(poses[-1], np.asarray(result.transformation))
                    if accepted else poses[-1].copy()
                )
                pair_row = {
                    "pair_index": index,
                    "source_points": len(cloud.points),
                    "target_points": len(previous_cloud.points),
                    "status": "ok" if accepted else f"rejected: {reason}",
                    "fitness": float(result.fitness),
                    "inlier_rmse_m": float(result.inlier_rmse),
                    "shared_correspondences": quality.correspondences,
                    "shared_correspondence_ratio": quality.correspondence_ratio,
                    "shared_inlier_rmse_m": quality.inlier_rmse_m,
                    "shared_all_point_rmse_m": quality.all_point_rmse_m,
                }
            except (RuntimeError, ValueError) as error:
                poses.append(poses[-1].copy())
                pair_row = {
                    "pair_index": index,
                    "source_points": len(cloud.points),
                    "target_points": len(previous_cloud.points),
                    "status": f"exception: {error}",
                    "fitness": None,
                    "inlier_rmse_m": None,
                    "shared_correspondences": 0,
                    "shared_correspondence_ratio": 0.0,
                    "shared_inlier_rmse_m": None,
                    "shared_all_point_rmse_m": None,
                }
            registration_runtime = time.perf_counter() - registration_start

        compute_runtime = preprocessing_runtime + registration_runtime
        input_to_pose_runtime = read_runtime + compute_runtime
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        frame_rows.append({
            "frame_index": index,
            "rgb_timestamp": frame.timestamp,
            "rgb_path": str(frame.rgb_path),
            "depth_path": str(frame.depth_path),
            "rgb_depth_offset_s": frame.depth_time_offset_s,
            "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s if isinstance(frame, RgbdFrame) else None,
            "point_count": len(cloud.points),
            "rgbd_read_runtime_s": read_runtime,
            "base_preprocessing_runtime_s": base_preprocess_runtime,
            "normal_estimation_runtime_s": normal_runtime,
            "preprocessing_runtime_s": preprocessing_runtime,
            "registration_runtime_s": registration_runtime,
            "compute_runtime_s": compute_runtime,
            "input_to_pose_runtime_s": input_to_pose_runtime,
            "process_rss_bytes_after_frame": rss,
        })
        if pair_row is not None:
            pair_row.update({
                "registration_runtime_s": registration_runtime,
                "rgbd_read_runtime_s": read_runtime,
                "preprocessing_runtime_s": preprocessing_runtime,
                "compute_runtime_s": compute_runtime,
                "end_to_end_runtime_s": input_to_pose_runtime,
                "input_to_pose_runtime_s": input_to_pose_runtime,
                "process_rss_bytes_after_registration": rss,
            })
            pair_rows.append(pair_row)
        point_counts.append(len(cloud.points))
        previous_cloud = cloud
    return poses, pair_rows, frame_rows, point_counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, default=None, help="Optional TUM ground-truth file; defaults to DATASET/groundtruth.txt when present.")
    parser.add_argument("--no-evaluation", action="store_true", help="Run odometry on all RGB-D pairs without reading ground truth.")
    parser.add_argument("--output", type=Path, default=Path("artifacts/tum_fr1_xyz_open3d_point_to_plane"))
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--input-repeats", type=int, default=1, help="Repeat the selected RGB-D files for a performance-only soak run; requires --no-evaluation.")
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
    parser.add_argument("--warmup-frames", type=int, default=30, help="Exclude these initial frame indices from steady-state latency statistics.")
    parser.add_argument("--quiet", action="store_true", help="Write all artifacts but do not print the summary JSON.")
    args = parser.parse_args()
    if args.max_acceptable_rmse is None:
        args.max_acceptable_rmse = args.max_correspondence
    if args.frame_step < 1 or args.input_repeats < 1 or args.stride < 1 or args.voxel <= 0 or args.normal_radius <= 0:
        parser.error("frame-step, input-repeats, stride, voxel, and normal-radius must be positive")
    if args.warmup_frames < 0:
        parser.error("warmup-frames must be non-negative")

    pairs = load_tum_rgbd_pairs(args.dataset)
    default_ground_truth = args.dataset / "groundtruth.txt"
    ground_truth_path = args.ground_truth if args.ground_truth is not None else default_ground_truth
    if args.ground_truth is not None and not args.ground_truth.is_file():
        parser.error(f"Ground-truth file does not exist: {args.ground_truth}")
    evaluation_available = not args.no_evaluation and ground_truth_path.is_file()
    if args.input_repeats > 1 and evaluation_available:
        parser.error("input-repeats is a performance-only replay and requires --no-evaluation")
    frames = (associate_ground_truth(pairs, ground_truth_path) if evaluation_available else pairs)[::args.frame_step]
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    frames = repeat_rgbd_pairs_for_performance(frames, repeats=args.input_repeats)
    if len(frames) < 3:
        parser.error("At least three associated frames are required")
    effective_warmup_frames = min(args.warmup_frames, len(frames) - 1)
    args.output.mkdir(parents=True, exist_ok=True)
    total_start = time.perf_counter()
    rss_samples = [rss for rss in [process_rss_bytes()] if rss is not None]
    poses, pair_rows, frame_rows, point_counts = run_streaming_point_to_plane(frames, args, rss_samples)
    write_trajectory(args.output / "trajectory_open3d_local.txt", frames, poses)
    methods: list[dict[str, object]] = []
    generated_artifacts = ["trajectory_open3d_local.txt", "frame_timings.csv", "open3d_steps.csv"]
    if evaluation_available:
        ground_truth = [frame.ground_truth for frame in frames if isinstance(frame, RgbdFrame)]
        identity_metrics, identity_aligned = evaluate("identity_no_motion_baseline", run_identity(len(frames)), ground_truth)
        metrics, aligned = evaluate("open3d_point_to_plane_icp", poses, ground_truth)
        methods = [identity_metrics, metrics]
        write_trajectory(args.output / "trajectory_open3d_ate_aligned.txt", frames, aligned)
        write_trajectory(args.output / "trajectory_identity_ate_aligned.txt", frames, identity_aligned)
        plot_trajectories(args.output / "trajectory_xy_xz.png", ground_truth, {"Open3D point-to-plane ICP": aligned, "identity baseline": identity_aligned})
        generated_artifacts += ["trajectory_open3d_ate_aligned.txt", "trajectory_identity_ate_aligned.txt", "trajectory_xy_xz.png"]
    for name, rows in (("frame_timings.csv", frame_rows), ("open3d_steps.csv", pair_rows)):
        with (args.output / name).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    accepted = [row for row in pair_rows if row["status"] == "ok"]
    final_rss = process_rss_bytes()
    if final_rss is not None:
        rss_samples.append(final_rss)
    summary = {
        "experiment": "TUM RGB-D fr1/xyz Open3D point-to-plane ICP odometry",
        "created_utc": datetime.now(timezone.utc).isoformat(), "dataset": str(args.dataset),
        "association_protocol": "one_to_one_minimum_offset_greedy_v2",
        "parameters": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "frames": {"count": len(frames), "first_timestamp": frames[0].timestamp, "last_timestamp": frames[-1].timestamp,
                   "mean_rgb_depth_offset_s": float(np.mean([frame.depth_time_offset_s for frame in frames])),
                   "mean_rgb_ground_truth_offset_s": float(np.mean([frame.ground_truth_time_offset_s for frame in frames if isinstance(frame, RgbdFrame)])) if evaluation_available else None,
                   "point_counts": {"mean": float(np.mean(point_counts)), "min": min(point_counts), "max": max(point_counts)}},
        "evaluation": {
            "available": evaluation_available,
            "ground_truth_path": str(ground_truth_path) if evaluation_available else None,
            "reason": None if evaluation_available else ("disabled_by_user" if args.no_evaluation else "groundtruth_file_not_found"),
        },
        "methods": methods,
        "performance": {"processing_mode": "streaming_previous_cloud_only",
                        "accepted_pairs": len(accepted), "rejected_or_exception_pairs": len(pair_rows) - len(accepted),
                        "rgbd_read_latency": latency_stats([float(row["rgbd_read_runtime_s"]) for row in frame_rows]),
                        "preprocessing_including_normals_latency": latency_stats([float(row["preprocessing_runtime_s"]) for row in frame_rows]),
                        "registration_latency": latency_stats([float(row["registration_runtime_s"]) for row in accepted]),
                        "end_to_end_latency_excluding_first_frame": latency_stats([float(row["end_to_end_runtime_s"]) for row in accepted]),
                        "process_rss": {"sampling": "before run, after every preprocessing/registration, and at experiment end", "peak_bytes": max(rss_samples) if rss_samples else None, "final_bytes": final_rss},
                        "timing_protocol_v2": summarize_frame_performance(frame_rows, warmup_frames=effective_warmup_frames),
                        "rss_trend_v2": summarize_rss_trend(frame_rows, warmup_frames=effective_warmup_frames),
                        "total_runtime_s": time.perf_counter() - total_start},
        "environment": {"python": sys.version, "open3d": o3d.__version__, "numpy": np.__version__, "scipy": scipy.__version__, "pillow": PIL.__version__, "matplotlib": matplotlib.__version__, "platform": platform.platform(), "processor": platform.processor(), "logical_cpu_count": os.cpu_count(), "gpu": "not used"},
        "quality_protocol": "shared_nearest_neighbor_v2: final-transform source-to-target; gated correspondence ratio and inlier RMSE; all-source-point RMSE for residual rejection",
        "artifacts": generated_artifacts,
        "notes": ["Uses exactly the same deterministic association, final-transform registration quality, and ATE/RPE functions as the NumPy baseline.", "Ground truth is optional and evaluation-only.", "Open3D fitness/inlier RMSE are retained for diagnostics; acceptance uses the shared correspondence ratio and all-source-point RMSE. Rejected pairs retain the previous pose."],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
