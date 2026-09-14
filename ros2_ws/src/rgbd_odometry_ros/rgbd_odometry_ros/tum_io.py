"""ROS-independent TUM RGB/depth index parsing and association."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
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


def associate_one_to_one(first_times: list[float], second_times: list[float], max_offset_s: float) -> list[tuple[int, int]]:
    candidates = []
    for first_index, first_time in enumerate(first_times):
        left = bisect_left(second_times, first_time - max_offset_s)
        right = bisect_right(second_times, first_time + max_offset_s)
        candidates.extend(
            (abs(first_time - second_times[second_index]), first_index, second_index)
            for second_index in range(left, right)
        )
    used_first = set()
    used_second = set()
    matches = []
    for _, first_index, second_index in sorted(candidates):
        if first_index not in used_first and second_index not in used_second:
            used_first.add(first_index)
            used_second.add(second_index)
            matches.append((first_index, second_index))
    return sorted(matches)


def associate_rgb_depth(
    dataset: Path,
    max_offset_s: float = 0.02,
    max_ground_truth_offset_s: float = 0.02,
) -> list[tuple[float, float, Path, Path]]:
    """Return the one-to-one RGB/depth subset used by offline evaluation.

    Ground-truth timestamps are used only by this dataset replay harness to
    choose the evaluated frame subset. Ground-truth poses are neither parsed
    nor published and never enter the odometry node.
    """
    rgb = read_index(dataset / "rgb.txt")
    depth = read_index(dataset / "depth.txt")
    ground_truth_times = read_timestamps(dataset / "groundtruth.txt")
    if not rgb or not depth or not ground_truth_times:
        raise RuntimeError("RGB, depth, and ground-truth indices must be non-empty")
    rgb_depth_matches = associate_one_to_one(
        [row[0] for row in rgb], [row[0] for row in depth], max_offset_s
    )
    rgbd_ground_truth_matches = associate_one_to_one(
        [rgb[rgb_index][0] for rgb_index, _ in rgb_depth_matches],
        ground_truth_times,
        max_ground_truth_offset_s,
    )
    associated = []
    for rgb_depth_index, _ in rgbd_ground_truth_matches:
        rgb_index, depth_index = rgb_depth_matches[rgb_depth_index]
        associated.append((rgb[rgb_index][0], depth[depth_index][0], rgb[rgb_index][1], depth[depth_index][1]))
    if len(associated) < 2:
        raise RuntimeError("Fewer than two RGB-depth pairs were associated")
    return associated
