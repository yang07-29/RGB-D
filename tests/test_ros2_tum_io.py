import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "ros2_ws"
    / "src"
    / "rgbd_odometry_ros"
    / "rgbd_odometry_ros"
    / "tum_io.py"
)
SPEC = importlib.util.spec_from_file_location("rgbd_odometry_ros_tum_io", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
TUM_IO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TUM_IO)


class Ros2TumIoTests(unittest.TestCase):
    def test_association_uses_nearest_depth_with_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "rgb.txt").write_text(
                "1.000 rgb/a.png\n1.100 rgb/b.png\n1.300 rgb/c.png\n1.400 rgb/d.png\n", encoding="utf-8"
            )
            (root / "depth.txt").write_text(
                "0.990 depth/a.png\n1.115 depth/b.png\n1.350 depth/c.png\n1.399 depth/d.png\n", encoding="utf-8"
            )
            # The fourth RGB/depth pair is geometrically available but has no
            # ground-truth sample within the evaluation gate, so it is excluded.
            (root / "groundtruth.txt").write_text(
                "1.005 0 0 0 0 0 0 1\n1.105 0 0 0 0 0 0 1\n1.301 0 0 0 0 0 0 1\n",
                encoding="utf-8",
            )
            rows = TUM_IO.associate_rgb_depth(root, max_offset_s=0.02)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0][0], 1.000)
            self.assertEqual(rows[0][1], 0.990)
            self.assertEqual(rows[0][2], root / "rgb/a.png")
            self.assertEqual(rows[0][3], root / "depth/a.png")
            self.assertEqual(rows[1][3], root / "depth/b.png")


if __name__ == "__main__":
    unittest.main()
