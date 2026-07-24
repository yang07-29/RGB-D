from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "rgbd_odometry_ros"
sys.path.insert(0, str(PACKAGE_ROOT))
from rgbd_odometry_ros.geometry import depth_to_points, icp_point_to_point  # noqa: E402


class Ros2GeometryTests(unittest.TestCase):
    def test_depth_backprojection_and_icp(self) -> None:
        depth = np.array([[0, 1000, 0], [1000, 1000, 1000], [0, 1000, 0]], dtype=np.uint16)
        points = depth_to_points(
            depth,
            fx=1.0,
            fy=1.0,
            cx=1.0,
            cy=1.0,
            depth_scale=1000.0,
            stride=1,
            min_depth_m=0.1,
            max_depth_m=2.0,
        )
        self.assertEqual(points.shape, (5, 3))
        self.assertTrue(any(np.allclose(point, [0.0, 0.0, 1.0]) for point in points))

        rng = np.random.default_rng(8)
        target = rng.normal(size=(100, 3))
        source = target + np.array([0.04, -0.02, 0.01])
        result = icp_point_to_point(
            source, target, max_iterations=40, max_correspondence_m=0.2,
        )
        self.assertGreaterEqual(result.correspondences, 95)
        self.assertLess(result.rmse_m, 1e-6)
        self.assertTrue(np.allclose(result.transform[:3, 3], [-0.04, 0.02, -0.01], atol=1e-6))


if __name__ == "__main__":
    unittest.main()
