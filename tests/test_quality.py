import unittest

import numpy as np

from src.quality import evaluate_registration_quality, registration_failure_reason


class RegistrationQualityTests(unittest.TestCase):
    def test_accepts_good_registration(self):
        self.assertIsNone(registration_failure_reason(correspondence_ratio=0.9, residual_rmse_m=0.03, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))

    def test_rejects_low_correspondence_ratio(self):
        self.assertIn("correspondence_ratio", registration_failure_reason(correspondence_ratio=0.2, residual_rmse_m=0.01, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))

    def test_rejects_high_residual(self):
        self.assertIn("residual_rmse_m", registration_failure_reason(correspondence_ratio=0.9, residual_rmse_m=0.2, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))

    def test_rejects_non_finite_metrics(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            self.assertIn(
                "non_finite",
                registration_failure_reason(
                    correspondence_ratio=value,
                    residual_rmse_m=0.01,
                    min_correspondence_ratio=0.5,
                    max_residual_rmse_m=0.12,
                ),
            )
            self.assertIn(
                "non_finite",
                registration_failure_reason(
                    correspondence_ratio=0.9,
                    residual_rmse_m=value,
                    min_correspondence_ratio=0.5,
                    max_residual_rmse_m=0.12,
                ),
            )

    def test_shared_quality_reports_inlier_and_all_point_rmse(self):
        source = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
        target = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        quality = evaluate_registration_quality(source, target, np.eye(4), max_correspondence_m=0.1)
        self.assertEqual(quality.correspondences, 2)
        self.assertAlmostEqual(quality.correspondence_ratio, 2.0 / 3.0)
        self.assertAlmostEqual(quality.inlier_rmse_m, 0.0)
        self.assertAlmostEqual(quality.all_point_rmse_m, np.sqrt(64.0 / 3.0))

    def test_shared_quality_rejects_non_finite_transform(self):
        transform = np.eye(4)
        transform[0, 3] = np.nan
        points = np.eye(3)
        with self.assertRaisesRegex(ValueError, "finite"):
            evaluate_registration_quality(points, points, transform, max_correspondence_m=0.1)

    def test_shared_quality_rejects_non_rigid_transform(self):
        points = np.eye(3)
        scaled = np.eye(4)
        scaled[0, 0] = 2.0
        with self.assertRaisesRegex(ValueError, r"rigid SE\(3\)"):
            evaluate_registration_quality(points, points, scaled, max_correspondence_m=0.1)

        invalid_bottom_row = np.eye(4)
        invalid_bottom_row[3, 0] = 1.0
        with self.assertRaisesRegex(ValueError, "homogeneous bottom row"):
            evaluate_registration_quality(points, points, invalid_bottom_row, max_correspondence_m=0.1)

    def test_shared_quality_rejects_empty_and_non_finite_clouds(self):
        with self.assertRaisesRegex(ValueError, "source"):
            evaluate_registration_quality(np.empty((0, 3)), np.eye(3), np.eye(4), max_correspondence_m=0.1)
        target = np.eye(3)
        target[0, 0] = np.inf
        with self.assertRaisesRegex(ValueError, "finite"):
            evaluate_registration_quality(np.eye(3), target, np.eye(4), max_correspondence_m=0.1)


if __name__ == "__main__":
    unittest.main()
