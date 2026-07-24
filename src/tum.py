"""TUM RGB-D text-file parsing and timestamp association utilities."""

from __future__ import annotations

from bisect import bisect_left
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


def _nearest_index(times: list[float], value: float) -> int:
    position = bisect_left(times, value)
    candidates = [index for index in (position - 1, position) if 0 <= index < len(times)]
    return min(candidates, key=lambda index: abs(times[index] - value))


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
    """Associate every RGB image with nearest depth and ground-truth samples.

    The association is deterministic and only retains triples that meet both
    supplied timestamp tolerances.  Poses follow TUM convention: camera to world.
    """
    root = Path(dataset_dir)
    rgb = _read_index(root / "rgb.txt")
    depth = _read_index(root / "depth.txt")
    ground_truth = _read_ground_truth(root / "groundtruth.txt")
    if not rgb or not depth or not ground_truth:
        raise ValueError("RGB, depth, and ground-truth files must all contain samples")

    depth_times = [item[0] for item in depth]
    gt_times = [item[0] for item in ground_truth]
    frames: list[RgbdFrame] = []
    for rgb_time, rgb_path in rgb:
        depth_index = _nearest_index(depth_times, rgb_time)
        gt_index = _nearest_index(gt_times, rgb_time)
        depth_time, depth_path = depth[depth_index]
        gt_time, gt_pose = ground_truth[gt_index]
        depth_offset = abs(depth_time - rgb_time)
        gt_offset = abs(gt_time - rgb_time)
        if depth_offset <= max_depth_time_offset_s and gt_offset <= max_ground_truth_time_offset_s:
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
