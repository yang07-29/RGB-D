"""合成点云上的 ICP 验证实验。

运行：python -m src.experiment --plot
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .geometry import icp_point_to_point, transform_points, voxel_downsample


def rotation_from_euler_xyz(degrees: tuple[float, float, float]) -> np.ndarray:
    x, y, z = np.deg2rad(degrees)
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_asymmetric_cloud(seed: int, points_per_cluster: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    centers = np.array([[-1.2, -0.3, 0.2], [0.7, 0.6, -0.5], [0.3, -1.0, 1.1]])
    scales = np.array([[0.25, 0.10, 0.35], [0.15, 0.35, 0.10], [0.35, 0.18, 0.12]])
    clusters = [rng.normal(center, scale, size=(points_per_cluster, 3)) for center, scale in zip(centers, scales)]
    return np.vstack(clusters)


def rotation_error_degrees(estimated: np.ndarray, ground_truth: np.ndarray) -> float:
    delta = estimated @ ground_truth.T
    cosine = np.clip((np.trace(delta) - 1) / 2, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def save_plot(source: np.ndarray, target: np.ndarray, aligned: np.ndarray, output: Path) -> None:
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(11, 5))
    before = figure.add_subplot(121, projection="3d")
    after = figure.add_subplot(122, projection="3d")
    before.scatter(source[:, 0], source[:, 1], source[:, 2], s=3, label="source")
    before.scatter(target[:, 0], target[:, 1], target[:, 2], s=3, label="target")
    before.set_title("before ICP")
    before.legend()
    after.scatter(aligned[:, 0], aligned[:, 1], aligned[:, 2], s=3, label="aligned source")
    after.scatter(target[:, 0], target[:, 1], target[:, 2], s=3, label="target")
    after.set_title("after ICP")
    after.legend()
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true", help="保存配准前后可视化图片")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    source = make_asymmetric_cloud(args.seed, points_per_cluster=500)
    source = voxel_downsample(source, voxel_size=0.035)
    ground_truth_rotation = rotation_from_euler_xyz((3.0, -4.0, 6.0))
    ground_truth_translation = np.array([0.12, -0.08, 0.06])
    target = transform_points(source, ground_truth_rotation, ground_truth_translation)

    result = icp_point_to_point(
        source,
        target,
        max_correspondence_distance=0.35,
    )
    print("这是合成数据上的代码正确性验证，不是公开数据集成绩。")
    print(f"points: {len(source)}")
    print(f"iterations: {result.iterations}")
    print(f"correspondences: {result.correspondences}")
    print(f"nearest-neighbor RMSE: {result.rmse:.8f}")
    print(f"rotation error (deg): {rotation_error_degrees(result.rotation, ground_truth_rotation):.8f}")
    print(f"translation error: {np.linalg.norm(result.translation - ground_truth_translation):.8f}")

    if args.plot:
        output = Path("artifacts/synthetic/icp_before_after.png")
        save_plot(source, target, result.aligned_source, output)
        print(f"plot saved: {output}")


if __name__ == "__main__":
    main()
