import unittest

import numpy as np

from src.experiment import rotation_from_euler_xyz
from src.geometry import icp_point_to_point, kabsch, statistical_outlier_filter, transform_points, voxel_downsample


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.source = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.5, 0.2, 1.0], [-0.4, 1.2, 0.8]]
        )
        self.rotation = rotation_from_euler_xyz((2.0, -3.0, 5.0))
        self.translation = np.array([0.05, -0.03, 0.08])
        self.target = transform_points(self.source, self.rotation, self.translation)

    def test_kabsch_recovers_known_transform(self):
        rotation, translation = kabsch(self.source, self.target)
        np.testing.assert_allclose(rotation, self.rotation, atol=1e-10)
        np.testing.assert_allclose(translation, self.translation, atol=1e-10)

    def test_icp_aligns_nearby_clouds(self):
        result = icp_point_to_point(self.source, self.target, max_correspondence_distance=0.3)
        self.assertLess(result.rmse, 1e-7)
        np.testing.assert_allclose(result.aligned_source, self.target, atol=1e-7)

    def test_voxel_downsample_reduces_points(self):
        points = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 1.0, 1.0]])
        downsampled = voxel_downsample(points, voxel_size=0.1)
        self.assertEqual(len(downsampled), 2)

    def test_statistical_outlier_filter_removes_isolated_point(self):
        cluster = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.01, 0.01, 0.0]])
        filtered = statistical_outlier_filter(np.vstack((cluster, [[10.0, 10.0, 10.0]])), neighbors=2, std_ratio=1.0)
        self.assertEqual(len(filtered), 4)


if __name__ == "__main__":
    unittest.main()
