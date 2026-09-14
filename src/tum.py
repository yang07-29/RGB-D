"""TUM RGB-D text-file parsing and timestamp association utilities."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class RgbdPair:
    """One timestamp-associated RGB/depth pair, independent of evaluation data."""

    timestamp: float
    rgb_path: Path
    depth_path: Path
    depth_time_offset_s: float


@dataclass(frozen=True)
class RgbdFrame(RgbdPair):
    """An RGB-D pair with an evaluation-only ground-truth camera pose."""

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
    return sorted(rows, key=lambda row: row[0])


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
    return sorted(poses, key=lambda row: row[0])


def load_tum_rgbd_pairs(
    dataset_dir: str | Path,
    *,
    max_depth_time_offset_s: float = 0.02,
) -> list[RgbdPair]:
    """Load deterministic one-to-one RGB/depth pairs without requiring poses."""
    root = Path(dataset_dir)
    rgb = _read_index(root / "rgb.txt")
    depth = _read_index(root / "depth.txt")
    if not rgb or not depth:
        raise ValueError("RGB and depth index files must both contain samples")
    matches = _associate_one_to_one(
        [item[0] for item in rgb], [item[0] for item in depth], max_depth_time_offset_s
    )
    pairs = [
        RgbdPair(
            rgb[rgb_index][0],
            rgb[rgb_index][1],
            depth[depth_index][1],
            abs(depth[depth_index][0] - rgb[rgb_index][0]),
        )
        for rgb_index, depth_index in matches
    ]
    if len(pairs) < 2:
        raise RuntimeError("Fewer than two timestamp-associated RGB-D pairs were found")
    return pairs


def associate_ground_truth(
    pairs: list[RgbdPair],
    ground_truth_path: str | Path,
    *,
    max_ground_truth_time_offset_s: float = 0.02,
) -> list[RgbdFrame]:
    """Attach evaluation poses one-to-one without changing the RGB-D loader."""
    ground_truth = _read_ground_truth(Path(ground_truth_path))
    if not ground_truth:
        raise ValueError("Ground-truth file must contain poses")
    matches = _associate_one_to_one(
        [pair.timestamp for pair in pairs],
        [item[0] for item in ground_truth],
        max_ground_truth_time_offset_s,
    )
    frames = [
        RgbdFrame(
            pairs[pair_index].timestamp,
            pairs[pair_index].rgb_path,
            pairs[pair_index].depth_path,
            pairs[pair_index].depth_time_offset_s,
            ground_truth[truth_index][1],
            abs(ground_truth[truth_index][0] - pairs[pair_index].timestamp),
        )
        for pair_index, truth_index in matches
    ]
    if len(frames) < 2:
        raise RuntimeError("Fewer than two timestamp-associated evaluation frames were found")
    return frames


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
    pairs = load_tum_rgbd_pairs(root, max_depth_time_offset_s=max_depth_time_offset_s)
    return associate_ground_truth(
        pairs,
        root / "groundtruth.txt",
        max_ground_truth_time_offset_s=max_ground_truth_time_offset_s,
    )


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
