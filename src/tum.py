"""TUM RGB-D text-file parsing and timestamp association utilities."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class RgbdFrame:
    """One RGB/depth pair with the closest ground-truth camera pose."""

    timestamp: float
    rgb_path: Path
    depth_path: Path
    depth_time_offset_s: float
    ground_truth: np.ndarray  # 4x4 camera-to-world matrix
    ground_truth_time_offset_s: float


def _read_index(path: Path) -> list[tuple[float, Path]]:
    rows: list[tuple[float, Path]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        stamp, name = line.split(maxsplit=1)
        rows.append((float(stamp), path.parent / name))
    return rows


def _associate_one_to_one(first_times: list[float], second_times: list[float], max_offset_s: float) -> list[tuple[int, int]]:
    """Return deterministic minimum-offset greedy matches without sample reuse."""
    if not np.isfinite(max_offset_s) or max_offset_s < 0.0:
        raise ValueError("max_offset_s must be finite and non-negative")
    candidates: list[tuple[float, int, int]] = []
    for first_index, first_time in enumerate(first_times):
        left = bisect_left(second_times, first_time - max_offset_s)
        right = bisect_right(second_times, first_time + max_offset_s)
        candidates.extend(
            (abs(first_time - second_times[second_index]), first_index, second_index)
            for second_index in range(left, right)
        )
    used_first: set[int] = set()
    used_second: set[int] = set()
    matches: list[tuple[int, int]] = []
    for _, first_index, second_index in sorted(candidates):
        if first_index not in used_first and second_index not in used_second:
            used_first.add(first_index)
            used_second.add(second_index)
            matches.append((first_index, second_index))
    return sorted(matches)


def _read_ground_truth(path: Path) -> list[tuple[float, np.ndarray]]:
    poses: list[tuple[float, np.ndarray]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8:
            raise ValueError(f"Expected 8 columns in {path}: {line}")
        timestamp, tx, ty, tz, qx, qy, qz, qw = values
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        pose[:3, 3] = [tx, ty, tz]
        poses.append((timestamp, pose))
    return poses


def load_tum_rgbd_frames(
    dataset_dir: str | Path,
    *,
    max_depth_time_offset_s: float = 0.02,
    max_ground_truth_time_offset_s: float = 0.02,
) -> list[RgbdFrame]:
    """Associate RGB, depth, and ground truth one-to-one by minimum time offset.

    Candidate pairs inside each gate are sorted by offset and greedily consumed,
    matching the TUM association script's no-reuse rule. Poses follow TUM
    convention: camera to world.
    """
    root = Path(dataset_dir)
    rgb = _read_index(root / "rgb.txt")
    depth = _read_index(root / "depth.txt")
    ground_truth = _read_ground_truth(root / "groundtruth.txt")
    if not rgb or not depth or not ground_truth:
        raise ValueError("RGB, depth, and ground-truth files must all contain samples")

    rgb_depth_matches = _associate_one_to_one(
        [item[0] for item in rgb], [item[0] for item in depth], max_depth_time_offset_s
    )
    matched_rgb_times = [rgb[rgb_index][0] for rgb_index, _ in rgb_depth_matches]
    rgbd_ground_truth_matches = _associate_one_to_one(
        matched_rgb_times, [item[0] for item in ground_truth], max_ground_truth_time_offset_s
    )
    frames: list[RgbdFrame] = []
    for rgb_depth_index, gt_index in rgbd_ground_truth_matches:
        rgb_index, depth_index = rgb_depth_matches[rgb_depth_index]
        rgb_time, rgb_path = rgb[rgb_index]
        depth_time, depth_path = depth[depth_index]
        gt_time, gt_pose = ground_truth[gt_index]
        depth_offset = abs(depth_time - rgb_time)
        gt_offset = abs(gt_time - rgb_time)
        frames.append(RgbdFrame(rgb_time, rgb_path, depth_path, depth_offset, gt_pose, gt_offset))
    if len(frames) < 2:
        raise RuntimeError("Fewer than two timestamp-associated RGB-D frames were found")
    return frames


def pose_to_tum_row(timestamp: float, pose: np.ndarray) -> str:
    """Serialize a camera-to-world matrix in the standard TUM trajectory format."""
    pose = np.asarray(pose, dtype=float)
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    translation = pose[:3, 3]
    values = [timestamp, *translation, *quaternion]
    return " ".join(f"{value:.9f}" for value in values)


def read_tum_trajectory(path: str | Path) -> tuple[list[float], list[np.ndarray]]:
    """Read ``timestamp tx ty tz qx qy qz qw`` trajectory rows."""
    timestamps: list[float] = []
    poses: list[np.ndarray] = []
    for line_number, raw_line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8:
            raise ValueError(f"Expected 8 TUM trajectory columns at line {line_number}: {raw_line}")
        timestamp, tx, ty, tz, qx, qy, qz, qw = values
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        pose[:3, 3] = [tx, ty, tz]
        timestamps.append(timestamp)
        poses.append(pose)
    if not poses:
        raise ValueError(f"No poses found in {path}")
    return timestamps, poses
