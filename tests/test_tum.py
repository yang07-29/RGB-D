import tempfile
import unittest
from pathlib import Path

import numpy as np

from scipy.spatial.transform import Rotation

from src.tum import load_tum_rgbd_frames, pose_to_tum_row, read_tum_trajectory


class TumAssociationTests(unittest.TestCase):
    def test_nearest_timestamp_association_respects_both_thresholds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "rgb.txt").write_text("# timestamp filename\n1.000 rgb/a.png\n2.000 rgb/b.png\n3.000 rgb/c.png\n", encoding="utf-8")
            (root / "depth.txt").write_text("# timestamp filename\n1.008 depth/a.png\n2.015 depth/b.png\n3.050 depth/c.png\n", encoding="utf-8")
            (root / "groundtruth.txt").write_text(
                "# timestamp tx ty tz qx qy qz qw\n"
                "1.004 1 2 3 0 0 0 1\n2.003 4 5 6 0 0 0 1\n3.001 7 8 9 0 0 0 1\n",
                encoding="utf-8",
            )
            frames = load_tum_rgbd_frames(root, max_depth_time_offset_s=0.02, max_ground_truth_time_offset_s=0.01)
        self.assertEqual([frame.timestamp for frame in frames], [1.0, 2.0])
        self.assertAlmostEqual(frames[1].depth_time_offset_s, 0.015)
        np.testing.assert_allclose(frames[1].ground_truth[:3, 3], [4.0, 5.0, 6.0])

    def test_tum_trajectory_round_trip(self):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("xyz", [10.0, -5.0, 20.0], degrees=True).as_matrix()
        pose[:3, 3] = [0.1, -0.2, 1.3]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.txt"
            path.write_text(pose_to_tum_row(12.345, pose) + "\n", encoding="utf-8")
            timestamps, poses = read_tum_trajectory(path)
        self.assertEqual(timestamps, [12.345])
        np.testing.assert_allclose(poses[0], pose, atol=1e-8)

    def test_association_does_not_reuse_depth_or_ground_truth_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "rgb.txt").write_text(
                "1.000 rgb/a.png\n1.006 rgb/b.png\n2.000 rgb/c.png\n3.000 rgb/d.png\n", encoding="utf-8"
            )
            (root / "depth.txt").write_text(
                "1.004 depth/a.png\n2.001 depth/c.png\n3.001 depth/d.png\n", encoding="utf-8"
            )
            (root / "groundtruth.txt").write_text(
                "1.005 1 0 0 0 0 0 1\n2.002 2 0 0 0 0 0 1\n3.002 3 0 0 0 0 0 1\n", encoding="utf-8"
            )
            frames = load_tum_rgbd_frames(root)
        self.assertEqual([frame.timestamp for frame in frames], [1.006, 2.0, 3.0])
        self.assertEqual(len({frame.depth_path for frame in frames}), len(frames))
        self.assertEqual([frame.ground_truth[0, 3] for frame in frames], [1.0, 2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
