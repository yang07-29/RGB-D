"""Keyframe, FPFH/RANSAC loop closure, and Open3D pose-graph experiment."""

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
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from .metrics import invert_pose, rotation_angle_degrees
from .open3d_odometry import build_clouds_with_normals, run_point_to_plane
from .pose_graph_quality import loop_edge_rejection_reasons, sequential_edge_failure_reason
from .quality import evaluate_registration_quality
from .run_odometry import evaluate, plot_trajectories, write_trajectory
from .runtime import process_rss_bytes
from .slam import propose_descriptor_loop_candidates, propose_loop_candidates, select_keyframe_indices
from .tum import load_tum_rgbd_frames


def relative_error(estimate: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
    error = invert_pose(reference) @ estimate
    return float(np.linalg.norm(error[:3, 3])), rotation_angle_degrees(error[:3, :3])


def prepare_fpfh(cloud, *, normal_radius: float, feature_radius: float, max_nn: int):
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=max_nn))
    return o3d.pipelines.registration.compute_fpfh_feature(
        cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=max_nn * 3),
    )


def global_then_local_registration(source, target, source_fpfh, target_fpfh, args):
    global_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source,
        target,
        source_fpfh,
        target_fpfh,
        True,
        args.ransac_correspondence,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(args.ransac_correspondence),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(args.ransac_iterations, args.ransac_confidence),
    )
    refined = o3d.pipelines.registration.registration_icp(
        source,
        target,
        args.loop_refine_correspondence,
        global_result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=args.loop_refine_iterations),
    )
    return global_result, refined


def clone_pose_graph(graph):
    clone = o3d.pipelines.registration.PoseGraph()
    for node in graph.nodes:
        clone.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.asarray(node.pose).copy()))
    for edge in graph.edges:
        clone.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                edge.source_node_id,
                edge.target_node_id,
                np.asarray(edge.transformation).copy(),
                np.asarray(edge.information).copy(),
                edge.uncertain,
                edge.confidence,
            )
        )
    return clone


def build_sequential_pose_graph(
    key_clouds,
    initial_poses,
    max_correspondence: float,
    iterations: int,
    *,
    min_correspondence_ratio: float,
    max_all_point_rmse_m: float,
):
    graph = o3d.pipelines.registration.PoseGraph()
    graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))
    poses = [np.eye(4)]
    rows = []
    for source_id in range(len(key_clouds) - 1):
        target_id = source_id + 1
        predicted = invert_pose(initial_poses[target_id]) @ initial_poses[source_id]
        result = o3d.pipelines.registration.registration_icp(
            key_clouds[source_id],
            key_clouds[target_id],
            max_correspondence,
            predicted,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations),
        )
        icp_transformation = np.asarray(result.transformation)
        quality = evaluate_registration_quality(
            np.asarray(key_clouds[source_id].points),
            np.asarray(key_clouds[target_id].points),
            icp_transformation,
            max_correspondence_m=max_correspondence,
        )
        failure_reason = sequential_edge_failure_reason(
            correspondence_ratio=quality.correspondence_ratio,
            all_point_rmse_m=quality.all_point_rmse_m,
            min_correspondence_ratio=min_correspondence_ratio,
            max_all_point_rmse_m=max_all_point_rmse_m,
        )
        transformation = predicted if failure_reason is not None else icp_transformation
        edge_source = "odometry_prediction_fallback" if failure_reason is not None else "keyframe_icp"
        information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            key_clouds[source_id], key_clouds[target_id], max_correspondence, transformation,
        )
        poses.append(poses[-1] @ invert_pose(transformation))
        graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(poses[-1]))
        graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                source_id, target_id, transformation, information, uncertain=False,
            )
        )
        rows.append({
            "source_keyframe_id": source_id,
            "target_keyframe_id": target_id,
            "fitness": float(result.fitness),
            "inlier_rmse_m": float(result.inlier_rmse),
            "shared_correspondences": quality.correspondences,
            "shared_correspondence_ratio": quality.correspondence_ratio,
            "shared_inlier_rmse_m": quality.inlier_rmse_m,
            "shared_all_point_rmse_m": quality.all_point_rmse_m,
            "edge_source": edge_source,
            "quality_accepted": failure_reason is None,
            "failure_reason": failure_reason,
        })
    return graph, poses, rows


def plot_loop_cases(path: Path, key_clouds, case_records: list[dict[str, object]]) -> None:
    if not case_records:
        return
    figure, axes = plt.subplots(len(case_records), 2, figsize=(11, 4.5 * len(case_records)))
    axes = np.atleast_2d(axes)
    for row_index, record in enumerate(case_records):
        source_id = int(record["source_keyframe_id"])
        target_id = int(record["target_keyframe_id"])
        transform = np.asarray(record["refined_transformation"])
        source = np.asarray(key_clouds[source_id].points)
        target = np.asarray(key_clouds[target_id].points)
        aligned = (transform[:3, :3] @ source.T).T + transform[:3, 3]
        for axis, dims, labels in ((axes[row_index, 0], (0, 1), ("x", "y")), (axes[row_index, 1], (0, 2), ("x", "z"))):
            axis.scatter(target[:, dims[0]], target[:, dims[1]], s=2, alpha=0.45, label="target")
            axis.scatter(aligned[:, dims[0]], aligned[:, dims[1]], s=2, alpha=0.45, label="aligned source")
            axis.set_xlabel(f"{labels[0]} (m)")
            axis.set_ylabel(f"{labels[1]} (m)")
            axis.axis("equal")
            axis.grid(True, alpha=0.2)
            axis.legend()
        label = record["posthoc_gt_label"]
        decision = "accepted" if record["accepted"] else "rejected"
        axes[row_index, 0].set_title(f"{decision} {label}: keyframes {source_id}→{target_id}")
        axes[row_index, 1].set_title(
            f"GT error {record['gt_translation_error_m']:.3f} m / {record['gt_rotation_error_deg']:.2f}°"
        )
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("artifacts/pose_graph_fr1_xyz"))
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--normal-radius", type=float, default=0.10)
    parser.add_argument("--normal-max-nn", type=int, default=30)
    parser.add_argument("--odometry-correspondence", type=float, default=0.08)
    parser.add_argument("--odometry-iterations", type=int, default=30)
    parser.add_argument("--min-sequential-correspondence-ratio", type=float, default=0.5)
    parser.add_argument("--max-sequential-all-point-rmse", type=float, default=0.08)
    parser.add_argument("--keyframe-translation", type=float, default=0.05)
    parser.add_argument("--keyframe-rotation-deg", type=float, default=10.0)
    parser.add_argument("--keyframe-max-gap", type=int, default=40)
    parser.add_argument("--loop-min-separation", type=int, default=10)
    parser.add_argument("--candidate-mode", choices=("pose", "descriptor"), default="pose")
    parser.add_argument("--descriptor-file", type=Path)
    parser.add_argument("--descriptor-threshold", type=float, default=0.7909525632858276)
    parser.add_argument("--descriptor-top-k-per-target", type=int, default=1)
    parser.add_argument("--loop-candidate-distance", type=float, default=0.10)
    parser.add_argument("--loop-candidate-rotation-deg", type=float, default=25.0)
    parser.add_argument("--max-loop-candidates", type=int, default=30)
    parser.add_argument("--hard-negative-candidates", type=int, default=10)
    parser.add_argument("--hard-negative-min-distance", type=float, default=0.40)
    parser.add_argument("--feature-radius", type=float, default=0.25)
    parser.add_argument("--ransac-correspondence", type=float, default=0.075)
    parser.add_argument("--ransac-iterations", type=int, default=50000)
    parser.add_argument("--ransac-confidence", type=float, default=0.999)
    parser.add_argument("--loop-refine-correspondence", type=float, default=0.08)
    parser.add_argument("--loop-refine-iterations", type=int, default=50)
    parser.add_argument("--min-global-fitness", type=float, default=0.15)
    parser.add_argument("--min-refined-fitness", type=float, default=0.45)
    parser.add_argument("--max-refined-rmse", type=float, default=0.04)
    parser.add_argument("--min-loop-correspondence-ratio", type=float, default=0.45)
    parser.add_argument("--max-loop-all-point-rmse", type=float, default=0.12)
    parser.add_argument("--max-consistency-translation", type=float, default=0.08)
    parser.add_argument("--max-consistency-rotation-deg", type=float, default=15.0)
    parser.add_argument("--edge-prune-threshold", type=float, default=0.25)
    parser.add_argument("--loop-closure-preference", type=float, default=0.1)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    if args.voxel <= 0 or args.stride < 1 or args.max_loop_candidates < 1 or args.hard_negative_candidates < 1:
        parser.error("voxel, stride, and max-loop-candidates must be positive")
    if not 0 <= args.min_sequential_correspondence_ratio <= 1 or not 0 <= args.min_loop_correspondence_ratio <= 1:
        parser.error("sequential and loop correspondence ratios must be in [0, 1]")
    if args.max_sequential_all_point_rmse <= 0 or args.max_loop_all_point_rmse <= 0:
        parser.error("sequential and loop all-point RMSE gates must be positive")

    args.output.mkdir(parents=True, exist_ok=True)
    total_start = time.perf_counter()
    o3d.utility.random.seed(0)
    frames = load_tum_rgbd_frames(args.dataset)
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if len(frames) < 3:
        parser.error("At least three associated frames are required")
    odometry_args = SimpleNamespace(
        stride=args.stride,
        voxel=args.voxel,
        min_depth=args.min_depth,
        max_depth=args.max_depth,
        normal_radius=args.normal_radius,
        normal_max_nn=args.normal_max_nn,
        max_iterations=args.odometry_iterations,
        max_correspondence=args.odometry_correspondence,
        min_correspondence_ratio=0.5,
        max_acceptable_rmse=args.odometry_correspondence,
    )
    rss_samples = [value for value in [process_rss_bytes()] if value is not None]
    clouds, _frame_rows = build_clouds_with_normals(frames, odometry_args, rss_samples)
    full_odometry_poses, full_pair_rows = run_point_to_plane(clouds, odometry_args, rss_samples)
    full_ground_truth = [frame.ground_truth for frame in frames]
    full_odometry_metrics, _ = evaluate("open3d_point_to_plane_odometry", full_odometry_poses, full_ground_truth)

    keyframe_indices = select_keyframe_indices(
        full_odometry_poses,
        min_translation_m=args.keyframe_translation,
        min_rotation_deg=args.keyframe_rotation_deg,
        max_interval_frames=args.keyframe_max_gap,
    )
    key_frames = [frames[index] for index in keyframe_indices]
    key_clouds = [clouds[index] for index in keyframe_indices]
    selected_odometry_poses = [full_odometry_poses[index] for index in keyframe_indices]
    key_ground_truth = [frame.ground_truth for frame in key_frames]

    pose_graph, initial_key_poses, odometry_edges = build_sequential_pose_graph(
        key_clouds, selected_odometry_poses, args.odometry_correspondence, args.odometry_iterations,
        min_correspondence_ratio=args.min_sequential_correspondence_ratio,
        max_all_point_rmse_m=args.max_sequential_all_point_rmse,
    )
    if args.candidate_mode == "descriptor":
        if args.descriptor_file is None:
            parser.error("--descriptor-file is required when --candidate-mode=descriptor")
        full_descriptors = np.load(args.descriptor_file)
        if full_descriptors.ndim != 2 or full_descriptors.shape[0] != len(frames):
            raise ValueError(
                f"Descriptor rows ({full_descriptors.shape}) must match associated frames ({len(frames)})"
            )
        candidates = propose_descriptor_loop_candidates(
            full_descriptors[keyframe_indices],
            min_keyframe_separation=args.loop_min_separation,
            min_similarity=args.descriptor_threshold,
            max_candidates=args.max_loop_candidates,
            top_k_per_target=args.descriptor_top_k_per_target,
        )
    else:
        candidates = propose_loop_candidates(
            initial_key_poses,
            min_keyframe_separation=args.loop_min_separation,
            max_estimated_distance_m=args.loop_candidate_distance,
            max_estimated_rotation_deg=args.loop_candidate_rotation_deg,
            max_candidates=args.max_loop_candidates,
        )
    features = [
        prepare_fpfh(
            cloud,
            normal_radius=args.normal_radius,
            feature_radius=args.feature_radius,
            max_nn=args.normal_max_nn,
        )
        for cloud in key_clouds
    ]

    loop_rows: list[dict[str, object]] = []
    case_records: list[dict[str, object]] = []
    accepted_loop_edges = 0
    for candidate in candidates:
        source_id = candidate.source_keyframe_id
        target_id = candidate.target_keyframe_id
        estimated_candidate_relative = invert_pose(initial_key_poses[target_id]) @ initial_key_poses[source_id]
        estimated_candidate_distance = float(np.linalg.norm(estimated_candidate_relative[:3, 3]))
        estimated_candidate_rotation = rotation_angle_degrees(estimated_candidate_relative[:3, :3])
        start = time.perf_counter()
        global_result, refined = global_then_local_registration(
            key_clouds[source_id], key_clouds[target_id], features[source_id], features[target_id], args,
        )
        predicted = invert_pose(initial_key_poses[target_id]) @ initial_key_poses[source_id]
        consistency_translation, consistency_rotation = relative_error(np.asarray(refined.transformation), predicted)
        truth_relative = invert_pose(key_ground_truth[target_id]) @ key_ground_truth[source_id]
        gt_translation_error, gt_rotation_error = relative_error(np.asarray(refined.transformation), truth_relative)
        loop_quality = evaluate_registration_quality(
            np.asarray(key_clouds[source_id].points),
            np.asarray(key_clouds[target_id].points),
            np.asarray(refined.transformation),
            max_correspondence_m=args.loop_refine_correspondence,
        )
        reasons = loop_edge_rejection_reasons(
            global_fitness=float(global_result.fitness),
            refined_fitness=float(refined.fitness),
            refined_inlier_rmse_m=float(refined.inlier_rmse),
            shared_correspondence_ratio=loop_quality.correspondence_ratio,
            shared_all_point_rmse_m=loop_quality.all_point_rmse_m,
            consistency_translation_m=consistency_translation,
            consistency_rotation_deg=consistency_rotation,
            min_global_fitness=args.min_global_fitness,
            min_refined_fitness=args.min_refined_fitness,
            max_refined_inlier_rmse_m=args.max_refined_rmse,
            min_shared_correspondence_ratio=args.min_loop_correspondence_ratio,
            max_shared_all_point_rmse_m=args.max_loop_all_point_rmse,
            max_consistency_translation_m=args.max_consistency_translation,
            max_consistency_rotation_deg=args.max_consistency_rotation_deg,
        )
        accepted = not reasons
        if accepted:
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                key_clouds[source_id], key_clouds[target_id], args.loop_refine_correspondence, refined.transformation,
            )
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    source_id, target_id, refined.transformation, information, uncertain=True,
                )
            )
            accepted_loop_edges += 1
        row = {
            "source_keyframe_id": source_id,
            "target_keyframe_id": target_id,
            "source_frame_index": keyframe_indices[source_id],
            "target_frame_index": keyframe_indices[target_id],
            "candidate_mode": args.candidate_mode,
            "descriptor_similarity": getattr(candidate, "descriptor_similarity", "N/A"),
            "estimated_candidate_distance_m": estimated_candidate_distance,
            "estimated_candidate_rotation_deg": estimated_candidate_rotation,
            "global_fitness": float(global_result.fitness),
            "global_inlier_rmse_m": float(global_result.inlier_rmse),
            "refined_fitness": float(refined.fitness),
            "refined_inlier_rmse_m": float(refined.inlier_rmse),
            "shared_correspondences": loop_quality.correspondences,
            "shared_correspondence_ratio": loop_quality.correspondence_ratio,
            "shared_inlier_rmse_m": loop_quality.inlier_rmse_m,
            "shared_all_point_rmse_m": loop_quality.all_point_rmse_m,
            "odometry_consistency_translation_m": consistency_translation,
            "odometry_consistency_rotation_deg": consistency_rotation,
            "accepted": accepted,
            "decision_reason": "accepted" if accepted else "; ".join(reasons),
            "gt_translation_error_m": gt_translation_error,
            "gt_rotation_error_deg": gt_rotation_error,
            "posthoc_gt_label": "correct" if gt_translation_error <= 0.05 and gt_rotation_error <= 5.0 else "incorrect",
            "registration_runtime_s": time.perf_counter() - start,
        }
        loop_rows.append(row)
        case_records.append(row | {"refined_transformation": np.asarray(refined.transformation)})
        rss = process_rss_bytes()
        if rss is not None:
            rss_samples.append(rss)

    # Deliberately stress the loop verifier with odometry-distant pairs. These
    # probes are never inserted into the pose graph, even if they pass. Ground
    # truth is used only after registration to label the resulting transform.
    hard_pair_pool = []
    for source_id in range(len(initial_key_poses)):
        for target_id in range(source_id + args.loop_min_separation, len(initial_key_poses)):
            distance = float(np.linalg.norm(initial_key_poses[source_id][:3, 3] - initial_key_poses[target_id][:3, 3]))
            if distance >= args.hard_negative_min_distance:
                hard_pair_pool.append((distance, source_id, target_id))
    hard_pair_pool.sort(reverse=True)
    hard_pairs = []
    for distance, source_id, target_id in hard_pair_pool:
        if any(abs(source_id - other_source) <= 2 and abs(target_id - other_target) <= 2 for _, other_source, other_target in hard_pairs):
            continue
        hard_pairs.append((distance, source_id, target_id))
        if len(hard_pairs) >= args.hard_negative_candidates:
            break

    hard_negative_rows: list[dict[str, object]] = []
    hard_case_records: list[dict[str, object]] = []
    for estimated_distance, source_id, target_id in hard_pairs:
        start = time.perf_counter()
        global_result, refined = global_then_local_registration(
            key_clouds[source_id], key_clouds[target_id], features[source_id], features[target_id], args,
        )
        predicted = invert_pose(initial_key_poses[target_id]) @ initial_key_poses[source_id]
        consistency_translation, consistency_rotation = relative_error(np.asarray(refined.transformation), predicted)
        truth_relative = invert_pose(key_ground_truth[target_id]) @ key_ground_truth[source_id]
        gt_translation_error, gt_rotation_error = relative_error(np.asarray(refined.transformation), truth_relative)
        loop_quality = evaluate_registration_quality(
            np.asarray(key_clouds[source_id].points),
            np.asarray(key_clouds[target_id].points),
            np.asarray(refined.transformation),
            max_correspondence_m=args.loop_refine_correspondence,
        )
        reasons = loop_edge_rejection_reasons(
            global_fitness=float(global_result.fitness),
            refined_fitness=float(refined.fitness),
            refined_inlier_rmse_m=float(refined.inlier_rmse),
            shared_correspondence_ratio=loop_quality.correspondence_ratio,
            shared_all_point_rmse_m=loop_quality.all_point_rmse_m,
            consistency_translation_m=consistency_translation,
            consistency_rotation_deg=consistency_rotation,
            min_global_fitness=args.min_global_fitness,
            min_refined_fitness=args.min_refined_fitness,
            max_refined_inlier_rmse_m=args.max_refined_rmse,
            min_shared_correspondence_ratio=args.min_loop_correspondence_ratio,
            max_shared_all_point_rmse_m=args.max_loop_all_point_rmse,
            max_consistency_translation_m=args.max_consistency_translation,
            max_consistency_rotation_deg=args.max_consistency_rotation_deg,
        )
        accepted_under_gates = not reasons
        row = {
            "source_keyframe_id": source_id,
            "target_keyframe_id": target_id,
            "source_frame_index": keyframe_indices[source_id],
            "target_frame_index": keyframe_indices[target_id],
            "estimated_pose_distance_m": estimated_distance,
            "global_fitness": float(global_result.fitness),
            "global_inlier_rmse_m": float(global_result.inlier_rmse),
            "refined_fitness": float(refined.fitness),
            "refined_inlier_rmse_m": float(refined.inlier_rmse),
            "shared_correspondences": loop_quality.correspondences,
            "shared_correspondence_ratio": loop_quality.correspondence_ratio,
            "shared_inlier_rmse_m": loop_quality.inlier_rmse_m,
            "shared_all_point_rmse_m": loop_quality.all_point_rmse_m,
            "odometry_consistency_translation_m": consistency_translation,
            "odometry_consistency_rotation_deg": consistency_rotation,
            "accepted_under_same_gates": accepted_under_gates,
            "decision_reason": "would pass gates but excluded by candidate policy" if accepted_under_gates else "; ".join(reasons),
            "gt_translation_error_m": gt_translation_error,
            "gt_rotation_error_deg": gt_rotation_error,
            "posthoc_gt_label": "correct" if gt_translation_error <= 0.05 and gt_rotation_error <= 5.0 else "incorrect",
            "registration_runtime_s": time.perf_counter() - start,
            "note": "Hard-negative stress probe; never inserted into pose graph.",
        }
        hard_negative_rows.append(row)
        hard_case_records.append(row | {"accepted": False, "refined_transformation": np.asarray(refined.transformation)})

    initial_metrics, initial_aligned = evaluate("keyframe_odometry_before_optimization", initial_key_poses, key_ground_truth)
    criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=args.loop_refine_correspondence,
        edge_prune_threshold=args.edge_prune_threshold,
        preference_loop_closure=args.loop_closure_preference,
        reference_node=0,
    )
    forced_bad_graph = None
    forced_bad_record = next(
        (record for record in hard_case_records if record["posthoc_gt_label"] == "incorrect"),
        None,
    )
    if forced_bad_record is not None:
        forced_bad_graph = clone_pose_graph(pose_graph)
        source_id = int(forced_bad_record["source_keyframe_id"])
        target_id = int(forced_bad_record["target_keyframe_id"])
        transformation = np.asarray(forced_bad_record["refined_transformation"])
        information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            key_clouds[source_id], key_clouds[target_id], args.loop_refine_correspondence, transformation,
        )
        # Evaluation-only adversarial diagnostic: deliberately bypass every
        # production gate and force one post-hoc wrong edge into the graph.
        forced_bad_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                source_id, target_id, transformation, information, uncertain=False,
            )
        )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        criteria,
        option,
    )
    optimized_key_poses = [np.asarray(node.pose) for node in pose_graph.nodes]
    optimized_metrics, optimized_aligned = evaluate("keyframe_pose_graph_after_optimization", optimized_key_poses, key_ground_truth)
    forced_bad_metrics = None
    if forced_bad_graph is not None:
        o3d.pipelines.registration.global_optimization(
            forced_bad_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            criteria,
            option,
        )
        forced_bad_poses = [np.asarray(node.pose) for node in forced_bad_graph.nodes]
        forced_bad_metrics, _ = evaluate("pose_graph_with_one_forced_bad_edge", forced_bad_poses, key_ground_truth)
        write_trajectory(args.output / "trajectory_keyframes_forced_bad_edge.txt", key_frames, forced_bad_poses)

    write_trajectory(args.output / "trajectory_keyframes_before.txt", key_frames, initial_key_poses)
    write_trajectory(args.output / "trajectory_keyframes_after.txt", key_frames, optimized_key_poses)
    write_trajectory(args.output / "trajectory_keyframes_before_ate_aligned.txt", key_frames, initial_aligned)
    write_trajectory(args.output / "trajectory_keyframes_after_ate_aligned.txt", key_frames, optimized_aligned)
    plot_trajectories(
        args.output / "trajectory_keyframes_before_after.png",
        key_ground_truth,
        {"before pose graph": initial_aligned, "after pose graph": optimized_aligned},
    )
    o3d.io.write_pose_graph(str(args.output / "pose_graph_optimized.json"), pose_graph)

    combined = o3d.geometry.PointCloud()
    for cloud, pose in zip(key_clouds, optimized_key_poses):
        transformed = o3d.geometry.PointCloud(cloud)
        transformed.transform(pose)
        combined += transformed
    combined = combined.voxel_down_sample(args.voxel)
    o3d.io.write_point_cloud(str(args.output / "optimized_keyframe_map.ply"), combined, write_ascii=False)

    if loop_rows:
        with (args.output / "loop_candidates.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(loop_rows[0]))
            writer.writeheader()
            writer.writerows(loop_rows)
    if hard_negative_rows:
        with (args.output / "hard_negative_candidates.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(hard_negative_rows[0]))
            writer.writeheader()
            writer.writerows(hard_negative_rows)
    with (args.output / "sequential_edges.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(odometry_edges[0]))
        writer.writeheader()
        writer.writerows(odometry_edges)
    with (args.output / "keyframes.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["keyframe_id", "frame_index", "timestamp"])
        writer.writeheader()
        writer.writerows(
            {"keyframe_id": key_id, "frame_index": frame_index, "timestamp": frames[frame_index].timestamp}
            for key_id, frame_index in enumerate(keyframe_indices)
        )

    correct_accepted = [row for row in case_records if row["accepted"] and row["posthoc_gt_label"] == "correct"]
    incorrect_accepted = [row for row in case_records if row["accepted"] and row["posthoc_gt_label"] == "incorrect"]
    rejected_incorrect = [row for row in case_records if not row["accepted"] and row["posthoc_gt_label"] == "incorrect"] + hard_case_records
    chosen_cases = []
    if correct_accepted:
        chosen_cases.append(min(correct_accepted, key=lambda row: row["gt_translation_error_m"] + row["gt_rotation_error_deg"] / 20.0))
    if incorrect_accepted:
        chosen_cases.append(max(incorrect_accepted, key=lambda row: row["gt_translation_error_m"] + row["gt_rotation_error_deg"] / 20.0))
    if rejected_incorrect:
        chosen_cases.append(max(rejected_incorrect, key=lambda row: row["gt_translation_error_m"] + row["gt_rotation_error_deg"] / 20.0))
    plot_loop_cases(args.output / "loop_correct_and_rejected_cases.png", key_clouds, chosen_cases)

    active_loop_edges = sum(1 for edge in pose_graph.edges if edge.uncertain and edge.confidence > args.edge_prune_threshold)
    accepted_correct = sum(1 for row in loop_rows if row["accepted"] and row["posthoc_gt_label"] == "correct")
    accepted_incorrect = sum(1 for row in loop_rows if row["accepted"] and row["posthoc_gt_label"] == "incorrect")
    rejected_correct = sum(1 for row in loop_rows if not row["accepted"] and row["posthoc_gt_label"] == "correct")
    rejected_incorrect_count = sum(1 for row in loop_rows if not row["accepted"] and row["posthoc_gt_label"] == "incorrect")
    summary = {
        "experiment": f"TUM fr1/xyz keyframes, {args.candidate_mode} loop candidates, FPFH/RANSAC, and Open3D pose graph",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "dataset": str(args.dataset),
        "association_protocol": "one_to_one_minimum_offset_greedy_v2",
        "edge_quality_protocol": "shared_nearest_neighbor_v2 plus backend fitness/inlier RMSE and odometry consistency",
        "parameters": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "frames": len(frames),
        "keyframes": len(keyframe_indices),
        "odometry_edges": len(odometry_edges),
        "sequential_edge_quality": {
            "accepted_keyframe_icp": sum(bool(row["quality_accepted"]) for row in odometry_edges),
            "odometry_prediction_fallback": sum(not bool(row["quality_accepted"]) for row in odometry_edges),
        },
        "loop_candidates": len(loop_rows),
        "accepted_loop_edges_before_optimization": accepted_loop_edges,
        "active_loop_edges_after_optimization": active_loop_edges,
        "posthoc_loop_labels": {
            "accepted_correct": accepted_correct,
            "accepted_incorrect": accepted_incorrect,
            "rejected_correct": rejected_correct,
            "rejected_incorrect": rejected_incorrect_count,
            "false_accepts": accepted_incorrect,
            "false_rejects": rejected_correct,
            "note": "Ground truth labels are evaluation-only and never enter candidate generation or acceptance.",
        },
        "hard_negative_stress": {
            "candidates": len(hard_negative_rows),
            "rejected_by_same_gates": sum(1 for row in hard_negative_rows if not row["accepted_under_same_gates"]),
            "posthoc_incorrect": sum(1 for row in hard_negative_rows if row["posthoc_gt_label"] == "incorrect"),
            "note": "Selected from odometry-distant pairs without ground truth and never inserted into the graph.",
        },
        "full_frame_odometry": full_odometry_metrics,
        "keyframe_before": initial_metrics,
        "keyframe_after": optimized_metrics,
        "forced_bad_edge_stress": {
            "performed": forced_bad_record is not None,
            "source_keyframe_id": forced_bad_record["source_keyframe_id"] if forced_bad_record is not None else None,
            "target_keyframe_id": forced_bad_record["target_keyframe_id"] if forced_bad_record is not None else None,
            "posthoc_gt_label": forced_bad_record["posthoc_gt_label"] if forced_bad_record is not None else None,
            "gt_translation_error_m": forced_bad_record["gt_translation_error_m"] if forced_bad_record is not None else None,
            "gt_rotation_error_deg": forced_bad_record["gt_rotation_error_deg"] if forced_bad_record is not None else None,
            "metrics_after_forced_insertion": forced_bad_metrics,
            "note": "Evaluation-only adversarial test: one post-hoc incorrect hard negative is inserted as a certain edge, bypassing all production gates.",
        },
        "performance": {
            "total_runtime_s": time.perf_counter() - total_start,
            "peak_rss_bytes": max(rss_samples) if rss_samples else process_rss_bytes(),
        },
        "environment": {
            "python": sys.version,
            "open3d": o3d.__version__,
            "platform": platform.platform(),
            "processor": platform.processor(),
            "logical_cpu_count": os.cpu_count(),
            "gpu": "not used",
        },
        "artifacts": [
            "summary.json", "keyframes.csv", "sequential_edges.csv", "loop_candidates.csv", "hard_negative_candidates.csv", "pose_graph_optimized.json",
            "trajectory_keyframes_before.txt", "trajectory_keyframes_after.txt",
            "trajectory_keyframes_before_after.png", "loop_correct_and_rejected_cases.png", "optimized_keyframe_map.ply",
            "trajectory_keyframes_forced_bad_edge.txt",
        ],
        "notes": [
            "Ground truth is used only for final ATE/RPE and post-hoc loop labels.",
            (
                "Candidate discovery uses frozen RGB descriptors only; FPFH/RANSAC supplies global initialization."
                if args.candidate_mode == "descriptor"
                else "Candidate discovery uses estimated pose proximity; FPFH/RANSAC supplies global initialization."
            ),
            "Sequential keyframe ICP uses shared final-transform quality; failed ICP edges fall back to the already estimated odometry transform instead of entering the graph as confident ICP.",
            "Loop acceptance uses backend metrics, shared final-transform quality, and odometry-cycle consistency, not ground truth.",
            "Reported before/after pose-graph metrics are on the identical selected keyframe timestamps.",
        ],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
