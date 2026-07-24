"""Export and inspect one timestamp-associated TUM RGB-D frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .rgbd import depth_to_points, load_png, write_colored_ply
from .tum import load_tum_rgbd_frames


def save_preview(path: Path, rgb: np.ndarray, depth: np.ndarray, points: np.ndarray, colors: np.ndarray | None) -> None:
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(14, 4.5))
    rgb_axis = figure.add_subplot(131)
    rgb_axis.imshow(rgb)
    rgb_axis.set_title("Associated RGB")
    rgb_axis.axis("off")
    depth_axis = figure.add_subplot(132)
    image = depth_axis.imshow(depth / 5000.0, cmap="turbo", vmin=0.2, vmax=4.0)
    depth_axis.set_title("Depth (m; raw / 5000)")
    depth_axis.axis("off")
    figure.colorbar(image, ax=depth_axis, fraction=0.046)
    cloud_axis = figure.add_subplot(133, projection="3d")
    sample = slice(None, None, max(1, len(points) // 12000))
    kwargs = {"c": colors[sample]} if colors is not None else {}
    cloud_axis.scatter(points[sample, 0], points[sample, 1], points[sample, 2], s=0.4, **kwargs)
    cloud_axis.set_title("Back-projected camera cloud")
    cloud_axis.set_xlabel("x (m)")
    cloud_axis.set_ylabel("y (m)")
    cloud_axis.set_zlabel("z (m)")
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("artifacts/tum_fr1_xyz_frame_000"))
    parser.add_argument("--frame-index", type=int, default=0, help="Index into valid timestamp associations.")
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.0)
    args = parser.parse_args()
    frames = load_tum_rgbd_frames(args.dataset)
    if not 0 <= args.frame_index < len(frames):
        parser.error(f"frame-index must be in [0, {len(frames) - 1}]")
    frame = frames[args.frame_index]
    rgb = load_png(frame.rgb_path)
    depth = load_png(frame.depth_path)
    points, colors = depth_to_points(depth, color=rgb, stride=args.stride, min_depth_m=args.min_depth, max_depth_m=args.max_depth)
    args.output.mkdir(parents=True, exist_ok=True)
    write_colored_ply(args.output / "cloud_colored.ply", points, colors)
    save_preview(args.output / "rgb_depth_cloud_preview.png", rgb, depth, points, colors)
    record = {
        "association_index": args.frame_index,
        "rgb_timestamp": frame.timestamp,
        "rgb_path": str(frame.rgb_path),
        "depth_path": str(frame.depth_path),
        "rgb_depth_offset_s": frame.depth_time_offset_s,
        "rgb_ground_truth_offset_s": frame.ground_truth_time_offset_s,
        "depth_dtype": str(depth.dtype),
        "depth_shape": list(depth.shape),
        "depth_scale": 5000.0,
        "valid_points": len(points),
        "camera_coordinate_bounds_m": {"min": points.min(axis=0).tolist(), "max": points.max(axis=0).tolist()},
        "artifacts": ["cloud_colored.ply", "rgb_depth_cloud_preview.png"],
    }
    (args.output / "frame_record.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
