import csv
from pathlib import Path
import tempfile
import unittest

from src.summarize_ros2_metrics import summarize


class Ros2MetricsTests(unittest.TestCase):
    def test_summary_reports_latency_rate_status_and_memory(self) -> None:
        fields = [
            "processed_frame", "rgb_stamp_s", "depth_stamp_s", "rgb_depth_offset_s",
            "rgb_received", "depth_received", "unsynchronized_or_pending", "status",
            "point_count", "correspondences", "rmse_m", "callback_latency_ms", "process_peak_rss_bytes",
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(dict(zip(fields, [1, 1.0, 1.001, 0.001, 1, 1, 0, "initialized", 100, 0, "", 10.0, 1000])))
                writer.writerow(dict(zip(fields, [2, 1.1, 1.101, 0.001, 2, 2, 0, "ok", 100, 80, 0.01, 20.0, 2000])))
                writer.writerow(dict(zip(fields, [3, 1.2, 1.201, 0.001, 3, 3, 0, "ok", 100, 82, 0.01, 30.0, 1800])))
            result = summarize(path)
        self.assertEqual(result["samples"], 3)
        self.assertAlmostEqual(result["callback_latency_ms"]["mean"], 20.0)
        self.assertAlmostEqual(result["input_message_rate_hz_from_timestamps"], 10.0)
        self.assertEqual(result["status_counts"]["ok"], 2)
        self.assertEqual(result["process_peak_rss_bytes"], 2000)
        self.assertIn("before metrics CSV", result["measurement_scope"]["callback_latency_ms"])


if __name__ == "__main__":
    unittest.main()
