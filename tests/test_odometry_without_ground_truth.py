import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]


def test_numpy_odometry_runs_without_ground_truth_file():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory) / "dataset"
        output = Path(directory) / "output"
        (root / "rgb").mkdir(parents=True)
        (root / "depth").mkdir()
        rgb_rows = []
        depth_rows = []
        for index in range(3):
            timestamp = 1.0 + index * 0.03
            rgb_name = f"rgb/{index}.png"
            depth_name = f"depth/{index}.png"
            Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(root / rgb_name)
            Image.fromarray(np.full((8, 8), 5000, dtype=np.uint16)).save(root / depth_name)
            rgb_rows.append(f"{timestamp:.3f} {rgb_name}")
            depth_rows.append(f"{timestamp + 0.005:.3f} {depth_name}")
        (root / "rgb.txt").write_text("\n".join(rgb_rows) + "\n", encoding="utf-8")
        (root / "depth.txt").write_text("\n".join(depth_rows) + "\n", encoding="utf-8")

        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "src.run_odometry",
                "--dataset",
                str(root),
                "--output",
                str(output),
                "--stride",
                "1",
                "--voxel",
                "0.01",
                "--max-frames",
                "3",
                "--quiet",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        trajectory = (output / "trajectory_icp_local.txt").read_text(encoding="utf-8").splitlines()

    assert summary["frames"]["count"] == 3
    assert summary["evaluation"] == {
        "available": False,
        "ground_truth_path": None,
        "reason": "groundtruth_file_not_found",
    }
    assert summary["methods"] == []
    assert len(trajectory) == 3
