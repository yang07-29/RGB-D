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
from .quality import evaluate_registration_quality, registration_failure_reason
from .runtime import latency_stats, process_rss_bytes
from .tum import RgbdFrame, RgbdPair, associate_ground_truth, load_tum_rgbd_pairs, pose_to_tum_row


def make_point_cloud(frame: RgbdPair, *, stride: int, voxel_size_m: float, min_depth_m: float, max_depth_m: float) -> tuple[np.ndarray, float, float]:
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
            current_to_previous = np.eye(4)
            current_to_previous[:3, :3] = result.rotation
            current_to_previous[:3, 3] = result.translation
            quality = evaluate_registration_quality(
                clouds[index], clouds[index - 1], current_to_previous,
                max_correspondence_m=correspondence_distance_m,
            )
            reason = registration_failure_reason(
                correspondence_ratio=quality.correspondence_ratio, residual_rmse_m=quality.all_point_rmse_m,
                min_correspondence_ratio=min_correspondence_ratio, max_residual_rmse_m=max_acceptable_rmse_m,
            )
            common = {
                "pair_index": index, "source_points": len(clouds[index]), "target_points": len(clouds[index - 1]),
                "iterations": result.iterations, "correspondences": quality.correspondences,
                "correspondence_ratio": quality.correspondence_ratio,
                "inlier_rmse_m": quality.inlier_rmse_m,
                "all_point_rmse_m": quality.all_point_rmse_m,
                "nearest_neighbor_rmse_m": quality.all_point_rmse_m,
            }
            if reason is not None:
                poses.append(poses[-1].copy())
                rows.append(common | {"status": f"rejected: {reason}", "accepted": False, "recovery_attempted": False, "recovery_succeeded": False, "registration_runtime_s": time.perf_counter() - start})
                rss = process_rss_bytes()
                if rss is not None:
                    rss_samples.append(rss)
                rows[-1]["process_rss_bytes_after_registration"] = rss
                continue
            poses.append(compose_camera_to_world(poses[-1], current_to_previous))
            rows.append(common | {"status": "ok", "accepted": True, "recovery_attempted": False, "recovery_succeeded": False, "registration_runtime_s": time.perf_counter() - start})
        except RuntimeError as error:
            # Keep the previous pose so a single registration failure does not abort
            # the experiment; record it explicitly in the CSV and summary.
            poses.append(poses[-1].copy())
            rows.append({
                "pair_index": index, "source_points": len(clouds[index]), "target_points": len(clouds[index - 1]),
                "status": f"failed: {error}", "iterations": 0, "correspondences": 0,
                "correspondence_ratio": 0.0, "inlier_rmse_m": None, "all_point_rmse_m": None,
                "nearest_neighbor_rmse_m": None, "accepted": False, "recovery_attempted": False,
                "recovery_succeeded": False, "registration_runtime_s": time.perf_counter() - start,
            })
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        rows[-1]["process_rss_bytes_after_registration"] = rss
    return poses, rows


def _estimate_icp_transform(
    source: np.ndarray,
    target: np.ndarray,
    *,
    initial_transform: np.ndarray,
    max_iterations: int,
    correspondence_distance_m: float,
    min_correspondence_ratio: float,
    max_acceptable_rmse_m: float,
) -> tuple[np.ndarray, object, object, str | None]:
    result = icp_point_to_point(
        source,
        target,
        max_iterations=max_iterations,
        max_correspondence_distance=correspondence_distance_m,
        initial_transform=initial_transform,
    )
    transform = np.eye(4)
    transform[:3, :3] = result.rotation
    transform[:3, 3] = result.translation
    quality = evaluate_registration_quality(
        source, target, transform, max_correspondence_m=correspondence_distance_m
    )
    reason = registration_failure_reason(
        correspondence_ratio=quality.correspondence_ratio,
        residual_rmse_m=quality.all_point_rmse_m,
        min_correspondence_ratio=min_correspondence_ratio,
        max_residual_rmse_m=max_acceptable_rmse_m,
    )
    return transform, result, quality, reason


def _estimate_multiscale_transform(
    source: np.ndarray,
    target: np.ndarray,
    *,
    initial_transform: np.ndarray,
    max_iterations: int,
    correspondence_distance_m: float,
    min_correspondence_ratio: float,
    max_acceptable_rmse_m: float,
    coarse_voxel_m: float,
    coarse_distance_multiplier: float,
) -> tuple[np.ndarray, object, object, str | None]:
    coarse_source = voxel_downsample(source, coarse_voxel_m)
    coarse_target = voxel_downsample(target, coarse_voxel_m)
    coarse = icp_point_to_point(
        coarse_source,
        coarse_target,
        max_iterations=max_iterations,
        max_correspondence_distance=correspondence_distance_m * coarse_distance_multiplier,
        initial_transform=initial_transform,
    )
    coarse_transform = np.eye(4)
    coarse_transform[:3, :3] = coarse.rotation
    coarse_transform[:3, 3] = coarse.translation
    return _estimate_icp_transform(
        source,
        target,
        initial_transform=coarse_transform,
        max_iterations=max_iterations,
        correspondence_distance_m=correspondence_distance_m,
        min_correspondence_ratio=min_correspondence_ratio,
        max_acceptable_rmse_m=max_acceptable_rmse_m,
    )


def summarize_tracking(rows: list[dict[str, object]], *, lost_after: int) -> dict[str, object]:
    """Summarize loss/recovery episodes from final per-frame acceptance decisions."""
    if lost_after < 1:
        raise ValueError("lost_after must be at least one")
    consecutive_failures = 0
    lost_start: int | None = None
    lost_events = 0
    recovered_lost_events = 0
    recovery_lengths: list[int] = []
    for row in rows:
        if bool(row["accepted"]):
            if lost_start is not None:
                recovered_lost_events += 1
                recovery_lengths.append(int(row["pair_index"]) - lost_start + 1)
                lost_start = None
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures == lost_after:
                lost_events += 1
                lost_start = int(row["pair_index"]) - lost_after + 1
    attempts = sum(bool(row.get("recovery_attempted")) for row in rows)
    successes = sum(bool(row.get("recovery_succeeded")) for row in rows)
    return {
        "lost_after_consecutive_failures": lost_after,
        "lost_events": lost_events,
        "recovered_lost_events": recovered_lost_events,
        "unrecovered_lost_events": lost_events - recovered_lost_events,
        "recovery_attempts": attempts,
        "successful_recovery_attempts": successes,
        "recovery_attempt_success_rate": float(successes / attempts) if attempts else None,
        "mean_lost_event_recovery_frames": float(np.mean(recovery_lengths)) if recovery_lengths else None,
        "max_lost_event_recovery_frames": max(recovery_lengths) if recovery_lengths else None,
    }


def run_icp_with_recovery(
    clouds: list[np.ndarray],
    *,
    max_iterations: int,
    correspondence_distance_m: float,
    min_correspondence_ratio: float,
    max_acceptable_rmse_m: float,
    coarse_voxel_m: float,
    coarse_distance_multiplier: float,
    lost_after: int,
    rss_samples: list[int],
) -> tuple[list[np.ndarray], list[dict[str, object]]]:
    """Track with constant-velocity initialization and multiscale/keyframe recovery."""
    poses = [np.eye(4)]
    internal_poses = [np.eye(4)]
    tracked = [True]
    rows: list[dict[str, object]] = []
    last_relative = np.eye(4)
    last_valid_index = 0
    consecutive_failures = 0
    for index in range(1, len(clouds)):
        start = time.perf_counter()
        primary_reason = None
        recovery_attempted = False
        recovery_succeeded = False
        target_index = index - 1
        gap = 1
        was_lost = consecutive_failures >= lost_after
        initial = np.linalg.matrix_power(last_relative, gap)
        transform = None
        tentative_transform = None
        tentative_quality_rmse = float("inf")
        result = None
        quality = None
        reason = None
        selected_initialization = None
        candidates = [("identity", np.eye(4))]
        if not np.allclose(initial, np.eye(4), atol=1e-10):
            candidates.append(("constant_velocity", initial))
        accepted_candidates = []
        candidate_failures = []
        for initialization_name, candidate_initial in candidates:
            try:
                candidate = _estimate_icp_transform(
                    clouds[index], clouds[target_index], initial_transform=candidate_initial,
                    max_iterations=max_iterations, correspondence_distance_m=correspondence_distance_m,
                    min_correspondence_ratio=min_correspondence_ratio,
                    max_acceptable_rmse_m=max_acceptable_rmse_m,
                )
                candidate_rmse = candidate[2].all_point_rmse_m
                if candidate_rmse < tentative_quality_rmse:
                    tentative_transform = candidate[0]
                    tentative_quality_rmse = candidate_rmse
                if candidate[3] is None:
                    accepted_candidates.append((initialization_name, *candidate))
                else:
                    candidate_failures.append(f"{initialization_name}: {candidate[3]}")
            except (RuntimeError, ValueError) as error:
                candidate_failures.append(f"{initialization_name}: exception: {error}")
        if accepted_candidates:
            # Identity is the frozen baseline. Motion prediction is only a
            # fallback when that baseline cannot produce an accepted pose.
            selected_initialization, transform, result, quality, reason = accepted_candidates[0]
            primary_reason = "; ".join(candidate_failures) or None
        else:
            reason = "; ".join(candidate_failures)
            primary_reason = reason
        tentative_chain_recovery = was_lost
        if tentative_chain_recovery:
            recovery_attempted = True

        if reason is not None and consecutive_failures + 1 >= lost_after:
            recovery_attempted = True
            if was_lost:
                target_index = last_valid_index
                gap = index - target_index
                initial = np.linalg.matrix_power(last_relative, gap)
            try:
                transform, result, quality, reason = _estimate_multiscale_transform(
                    clouds[index], clouds[target_index], initial_transform=initial,
                    max_iterations=max_iterations, correspondence_distance_m=correspondence_distance_m,
                    min_correspondence_ratio=min_correspondence_ratio,
                    max_acceptable_rmse_m=max_acceptable_rmse_m,
                    coarse_voxel_m=coarse_voxel_m,
                    coarse_distance_multiplier=coarse_distance_multiplier,
                )
                if reason is None:
                    selected_initialization = "multiscale_constant_velocity"
            except (RuntimeError, ValueError) as error:
                reason = f"exception: {error}"

        accepted = reason is None and transform is not None and result is not None and quality is not None
        if accepted:
            bridge_tentative_chain = was_lost or (recovery_attempted and consecutive_failures > 0)
            pose_base = internal_poses[target_index] if bridge_tentative_chain or target_index != index - 1 else poses[target_index]
            recovered_pose = pose_base @ transform
            poses.append(recovered_pose)
            internal_poses.append(recovered_pose)
            tracked.append(True)
            if selected_initialization == "constant_velocity" and primary_reason is not None:
                recovery_attempted = True
                recovery_succeeded = True
            recovery_succeeded = recovery_attempted
            if gap == 1:
                last_relative = transform
            last_valid_index = index
            consecutive_failures = 0
            if tentative_chain_recovery and target_index == index - 1:
                status = "recovered_tentative_chain"
            elif recovery_attempted and gap > 1:
                status = "recovered_keyframe"
            elif recovery_attempted:
                status = "recovered_multiscale"
            else:
                status = f"ok_{selected_initialization}"
        else:
            poses.append(poses[-1].copy())
            coast_transform = tentative_transform if tentative_transform is not None else last_relative
            internal_poses.append(internal_poses[-1] @ coast_transform)
            consecutive_failures += 1
            tracked.append(consecutive_failures < lost_after)
            status = f"lost: {reason}"

        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)
        rows.append({
            "pair_index": index,
            "target_index": target_index,
            "target_gap_frames": gap,
            "source_points": len(clouds[index]),
            "target_points": len(clouds[target_index]),
            "status": status,
            "accepted": accepted,
            "initialization": selected_initialization,
            "primary_failure_reason": primary_reason,
            "recovery_attempted": recovery_attempted,
            "recovery_succeeded": recovery_succeeded,
            "iterations": result.iterations if result is not None else 0,
            "correspondences": quality.correspondences if quality is not None else 0,
            "correspondence_ratio": quality.correspondence_ratio if quality is not None else 0.0,
            "inlier_rmse_m": quality.inlier_rmse_m if quality is not None else None,
            "all_point_rmse_m": quality.all_point_rmse_m if quality is not None else None,
            "nearest_neighbor_rmse_m": quality.all_point_rmse_m if quality is not None else None,
            "registration_runtime_s": time.perf_counter() - start,
            "process_rss_bytes_after_registration": rss,
        })
    return poses, rows


def evaluate(name: str, poses: list[np.ndarray], ground_truth: list[np.ndarray]) -> tuple[dict[str, object], list[np.ndarray]]:
    aligned, rotation, translation = align_estimated_poses(poses, ground_truth)
    metrics = {
        "method": name,
        "ate": absolute_trajectory_error(aligned, ground_truth),
        "rpe_frame_delta_1": relative_pose_error(poses, ground_truth, frame_delta=1),
        "alignment_estimated_to_ground_truth": {"rotation": rotation.tolist(), "translation_m": translation.tolist()},
    }
    if len(poses) > 30:
        metrics["rpe_frame_delta_30"] = relative_pose_error(poses, ground_truth, frame_delta=30)
    return metrics, aligned


def write_trajectory(path: Path, frames: list[RgbdPair], poses: list[np.ndarray]) -> None:
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
    parser.add_argument("--ground-truth", type=Path, default=None, help="Optional TUM ground-truth file; defaults to DATASET/groundtruth.txt when present.")
    parser.add_argument("--no-evaluation", action="store_true", help="Run odometry on all RGB-D pairs without reading ground truth.")
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
    parser.add_argument("--tracking-mode", choices=("identity", "predictive_recovery"), default="identity", help="Relative-pose initialization and recovery policy.")
    parser.add_argument("--recovery-coarse-voxel", type=float, default=0.12, help="Coarse voxel size used by recovery ICP.")
    parser.add_argument("--recovery-distance-multiplier", type=float, default=2.5, help="Coarse recovery correspondence gate multiplier.")
    parser.add_argument("--lost-after", type=int, default=2, help="Declare a tracking-loss event after this many consecutive rejected frames.")
    parser.add_argument("--quiet", action="store_true", help="Write all artifacts but do not print the summary JSON.")
    args = parser.parse_args()
    if args.frame_step < 1 or args.stride < 1 or args.voxel <= 0 or args.recovery_coarse_voxel <= 0:
        parser.error("frame-step, stride, voxel, and recovery-coarse-voxel must be positive")
    if args.recovery_distance_multiplier < 1 or args.lost_after < 1:
        parser.error("recovery-distance-multiplier and lost-after must be at least one")
    if not 0 < args.min_correspondence_ratio <= 1:
        parser.error("min-correspondence-ratio must be in (0, 1]")
    if args.max_acceptable_rmse is None:
        args.max_acceptable_rmse = args.max_correspondence

    pairs = load_tum_rgbd_pairs(args.dataset)
    default_ground_truth = args.dataset / "groundtruth.txt"
    ground_truth_path = args.ground_truth if args.ground_truth is not None else default_ground_truth
    if args.ground_truth is not None and not args.ground_truth.is_file():
        parser.error(f"Ground-truth file does not exist: {args.ground_truth}")
    evaluation_available = not args.no_evaluation and ground_truth_path.is_file()
    frames: list[RgbdPair] = (
        associate_ground_truth(pairs, ground_truth_path) if evaluation_available else pairs
    )[::args.frame_step]
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
            "rgb_depth_offset_s": frame.depth_time_offset_s,
            "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s if isinstance(frame, RgbdFrame) else None,
            "point_count": len(cloud), "rgbd_read_runtime_s": rgbd_read_runtime, "preprocessing_runtime_s": preprocessing_runtime,
            "process_rss_bytes_after_preprocessing": rss,
        })
    cloud_runtime = sum(float(row["rgbd_read_runtime_s"]) + float(row["preprocessing_runtime_s"]) for row in frame_rows)
    identity_poses = run_identity(len(frames))
    if args.tracking_mode == "predictive_recovery":
        icp_poses, step_rows = run_icp_with_recovery(
            clouds,
            max_iterations=args.max_iterations,
            correspondence_distance_m=args.max_correspondence,
            min_correspondence_ratio=args.min_correspondence_ratio,
            max_acceptable_rmse_m=args.max_acceptable_rmse,
            coarse_voxel_m=args.recovery_coarse_voxel,
            coarse_distance_multiplier=args.recovery_distance_multiplier,
            lost_after=args.lost_after,
            rss_samples=rss_samples,
        )
        method_name = "point_to_point_icp_predictive_recovery"
    else:
        icp_poses, step_rows = run_icp(
            clouds,
            max_iterations=args.max_iterations,
            correspondence_distance_m=args.max_correspondence,
            min_correspondence_ratio=args.min_correspondence_ratio,
            max_acceptable_rmse_m=args.max_acceptable_rmse,
            rss_samples=rss_samples,
        )
        method_name = "frame_to_frame_point_to_point_icp"
    for row in step_rows:
        index = int(row["pair_index"])
        row["rgbd_read_runtime_s"] = frame_rows[index]["rgbd_read_runtime_s"]
        row["preprocessing_runtime_s"] = frame_rows[index]["preprocessing_runtime_s"]
        row["end_to_end_runtime_s"] = float(row["registration_runtime_s"]) + float(row["rgbd_read_runtime_s"]) + float(row["preprocessing_runtime_s"])
    write_trajectory(args.output / "trajectory_icp_local.txt", frames, icp_poses)
    methods: list[dict[str, object]] = []
    generated_artifacts = ["trajectory_icp_local.txt", "icp_steps.csv", "frame_timings.csv"]
    if evaluation_available:
        evaluated_frames = [frame for frame in frames if isinstance(frame, RgbdFrame)]
        ground_truth = [frame.ground_truth for frame in evaluated_frames]
        identity_metrics, identity_aligned = evaluate("identity_no_motion_baseline", identity_poses, ground_truth)
        icp_metrics, icp_aligned = evaluate(method_name, icp_poses, ground_truth)
        methods = [identity_metrics, icp_metrics]
        write_trajectory(args.output / "trajectory_icp_ate_aligned.txt", frames, icp_aligned)
        write_trajectory(args.output / "trajectory_identity_ate_aligned.txt", frames, identity_aligned)
        plot_trajectories(args.output / "trajectory_xy_xz.png", ground_truth, {"ICP": icp_aligned, "identity baseline": identity_aligned})
        generated_artifacts += ["trajectory_icp_ate_aligned.txt", "trajectory_identity_ate_aligned.txt", "trajectory_xy_xz.png"]
    with (args.output / "icp_steps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(step_rows[0]))
        writer.writeheader()
        writer.writerows(step_rows)
    with (args.output / "frame_timings.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(frame_rows[0]))
        writer.writeheader()
        writer.writerows(frame_rows)
    successful = [row for row in step_rows if bool(row["accepted"])]
    final_rss = process_rss_bytes()
    if final_rss is not None:
        rss_samples.append(final_rss)
    summary = {
        "experiment": "TUM RGB-D fr1/xyz multi-frame odometry",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "dataset": str(args.dataset),
        "association_protocol": "one_to_one_minimum_offset_greedy_v2",
        "parameters": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "frames": {
            "count": len(frames), "first_timestamp": frames[0].timestamp, "last_timestamp": frames[-1].timestamp,
            "mean_rgb_depth_offset_s": float(np.mean([frame.depth_time_offset_s for frame in frames])),
            "mean_rgb_ground_truth_offset_s": float(np.mean([frame.ground_truth_time_offset_s for frame in frames if isinstance(frame, RgbdFrame)])) if evaluation_available else None,
            "point_counts": {"mean": float(np.mean([len(cloud) for cloud in clouds])), "min": int(min(map(len, clouds))), "max": int(max(map(len, clouds)))},
        },
        "evaluation": {
            "available": evaluation_available,
            "ground_truth_path": str(ground_truth_path) if evaluation_available else None,
            "reason": None if evaluation_available else ("disabled_by_user" if args.no_evaluation else "groundtruth_file_not_found"),
        },
        "methods": methods,
        "tracking": summarize_tracking(step_rows, lost_after=args.lost_after),
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
        "artifacts": generated_artifacts,
        "quality_protocol": "shared_nearest_neighbor_v2: final-transform source-to-target; gated correspondence ratio and inlier RMSE; all-source-point RMSE for residual rejection",
        "notes": ["ICP uses only consecutive depth point clouds and an identity relative-pose initialization.", "A pair is rejected when the shared final-transform correspondence ratio is too low or all-source-point nearest-neighbor RMSE is too high; rejected pairs retain the previous pose.", "RGB images are read for end-to-end RGB-D input timing but are not used by this depth-only ICP solver.", "Ground truth is optional and, when present, is attached only after RGB-D association for evaluation.", "Identity is reported only when evaluation ground truth is available."],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
