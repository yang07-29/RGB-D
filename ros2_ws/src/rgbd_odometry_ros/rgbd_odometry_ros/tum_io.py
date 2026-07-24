"""ROS-independent TUM RGB/depth index parsing and association."""

from __future__ import annotations

from bisect import bisect_left
from pathlib import Path


def read_index(path: Path) -> list[tuple[float, Path]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        stamp, filename = line.split(maxsplit=1)
        rows.append((float(stamp), path.parent / filename))
    return rows


def read_timestamps(path: Path) -> list[float]:
    timestamps = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            timestamps.append(float(line.split(maxsplit=1)[0]))
    return timestamps


def nearest_index(times: list[float], value: float) -> int:
    position = bisect_left(times, value)
    candidates = [index for index in (position - 1, position) if 0 <= index < len(times)]
    return min(candidates, key=lambda index: abs(times[index] - value))


def associate_rgb_depth(
    dataset: Path,
    max_offset_s: float = 0.02,
    max_ground_truth_offset_s: float = 0.02,
) -> list[tuple[float, float, Path, Path]]:
    """Return the exact RGB/depth subset used by the offline evaluation protocol.

    Ground-truth timestamps are used only by this dataset replay harness to
    choose the evaluated frame subset. Ground-truth poses are neither parsed
    nor published and never enter the odometry node.
    """
    rgb = read_index(dataset / "rgb.txt")
    depth = read_index(dataset / "depth.txt")
    ground_truth_times = read_timestamps(dataset / "groundtruth.txt")
    if not rgb or not depth or not ground_truth_times:
        raise RuntimeError("RGB, depth, and ground-truth indices must be non-empty")
    depth_times = [row[0] for row in depth]
    associated = []
    for rgb_time, rgb_path in rgb:
        index = nearest_index(depth_times, rgb_time)
        ground_truth_index = nearest_index(ground_truth_times, rgb_time)
        if (
            abs(depth[index][0] - rgb_time) <= max_offset_s
            and abs(ground_truth_times[ground_truth_index] - rgb_time) <= max_ground_truth_offset_s
        ):
            associated.append((rgb_time, depth[index][0], rgb_path, depth[index][1]))
    if len(associated) < 2:
        raise RuntimeError("Fewer than two RGB-depth pairs were associated")
    return associated
