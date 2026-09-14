import unittest

import numpy as np

from src.metrics import absolute_trajectory_error, align_estimated_poses, compose_camera_to_world, relative_pose_error
from src.run_odometry import evaluate


class MetricsTests(unittest.TestCase):
    def test_current_to_previous_transform_accumulates_without_inverse(self):
        previous_camera_to_world = np.eye(4)
        previous_camera_to_world[:3, 3] = [1.0, 2.0, 3.0]
        current_to_previous = np.eye(4)
        current_to_previous[:3, 3] = [0.1, -0.2, 0.3]
        current_camera_to_world = compose_camera_to_world(previous_camera_to_world, current_to_previous)
        np.testing.assert_allclose(current_camera_to_world[:3, 3], [1.1, 1.8, 3.3])

    def test_alignment_and_metrics_are_zero_for_rigidly_transformed_trajectory(self):
        reference = []
        estimated = []
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        translation = np.array([2.0, -3.0, 1.0])
        for position in ([0, 0, 0], [1, 0, 0], [1, 2, 0], [2, 2, 1]):
            truth = np.eye(4)
            truth[:3, 3] = position
            estimate = np.eye(4)
            estimate[:3, :3] = rotation.T
            estimate[:3, 3] = rotation.T @ (truth[:3, 3] - translation)
            reference.append(truth)
            estimated.append(estimate)
        aligned, _, _ = align_estimated_poses(estimated, reference)
        self.assertLess(absolute_trajectory_error(aligned, reference)["rmse_m"], 1e-12)
        rpe = relative_pose_error(estimated, reference)
        self.assertLess(rpe["translation_rmse_m"], 1e-12)
        self.assertLess(rpe["rotation_rmse_deg"], 1e-12)

    def test_evaluate_reports_short_and_long_interval_rpe(self):
        poses = []
        for index in range(35):
            pose = np.eye(4)
            pose[0, 3] = index * 0.01
            poses.append(pose)
        metrics, _ = evaluate("controlled", poses, poses)
        self.assertEqual(metrics["rpe_frame_delta_1"]["pairs"], 34)
        self.assertEqual(metrics["rpe_frame_delta_30"]["pairs"], 5)


if __name__ == "__main__":
    unittest.main()
