import unittest

import numpy as np

from src.rgbd import CameraIntrinsics, depth_to_points


class RgbdTests(unittest.TestCase):
    def test_depth_to_points_backprojects_center_pixel(self):
        depth = np.array([[1000, 1000], [1000, 0]], dtype=np.uint16)
        intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=0.0, cy=0.0, depth_scale=1000.0)
        points, colors = depth_to_points(depth, intrinsics=intrinsics, stride=1, min_depth_m=0.1, max_depth_m=2.0)
        self.assertIsNone(colors)
        self.assertEqual(len(points), 3)
        np.testing.assert_allclose(points[0], [0.0, 0.0, 1.0])
        np.testing.assert_allclose(points[1], [0.01, 0.0, 1.0])
        np.testing.assert_allclose(points[2], [0.0, 0.01, 1.0])

    def test_depth_to_points_keeps_aligned_colors(self):
        depth = np.array([[1000]], dtype=np.uint16)
        color = np.array([[[128, 64, 32]]], dtype=np.uint8)
        intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=0.0, cy=0.0, depth_scale=1000.0)
        _, colors = depth_to_points(depth, color=color, intrinsics=intrinsics, stride=1)
        np.testing.assert_allclose(colors, [[128 / 255, 64 / 255, 32 / 255]])


if __name__ == "__main__":
    unittest.main()
