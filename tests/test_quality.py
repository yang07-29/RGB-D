import unittest

from src.quality import registration_failure_reason


class RegistrationQualityTests(unittest.TestCase):
    def test_accepts_good_registration(self):
        self.assertIsNone(registration_failure_reason(correspondence_ratio=0.9, residual_rmse_m=0.03, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))

    def test_rejects_low_correspondence_ratio(self):
        self.assertIn("correspondence_ratio", registration_failure_reason(correspondence_ratio=0.2, residual_rmse_m=0.01, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))

    def test_rejects_high_residual(self):
        self.assertIn("residual_rmse_m", registration_failure_reason(correspondence_ratio=0.9, residual_rmse_m=0.2, min_correspondence_ratio=0.5, max_residual_rmse_m=0.12))


if __name__ == "__main__":
    unittest.main()
