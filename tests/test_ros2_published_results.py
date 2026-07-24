import csv
import json
from pathlib import Path
import re
import unittest

import numpy as np
from PIL import Image


ROOT = Path(__file__).parents[1]
RESULTS = ROOT / "results" / "ros2"


class PublishedRos2ResultsTests(unittest.TestCase):
    def test_full_run_bag_and_rviz_evidence_are_internally_consistent(self):
        performance = json.loads((RESULTS / "full_796_performance.json").read_text(encoding="utf-8"))
        evaluation = json.loads((RESULTS / "full_796_evaluation.json").read_text(encoding="utf-8"))
        with (RESULTS / "full_796_metrics.csv").open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        trajectory_rows = [
            line for line in (RESULTS / "full_796_trajectory_local.txt").read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]

        self.assertEqual(len(rows), 796)
        self.assertEqual(len(trajectory_rows), 796)
        self.assertEqual([int(row["processed_frame"]) for row in rows], list(range(1, 797)))
        self.assertEqual(performance["samples"], len(rows))
        self.assertEqual(evaluation["frames"], len(rows))
        self.assertEqual(performance["rgb_received_final"], 796)
        self.assertEqual(performance["depth_received_final"], 796)
        self.assertEqual(performance["unsynchronized_or_pending_final"], 0)
        self.assertEqual(sum(performance["status_counts"].values()), 796)

        latencies = np.asarray([float(row["callback_latency_ms"]) for row in rows])
        self.assertAlmostEqual(performance["callback_latency_ms"]["mean"], float(np.mean(latencies)), places=10)
        self.assertAlmostEqual(performance["callback_latency_ms"]["p95"], float(np.percentile(latencies, 95)), places=10)
        self.assertAlmostEqual(evaluation["metrics"]["ate"]["rmse_m"], 0.1754023160158547, places=12)
        self.assertEqual(evaluation["alignment"], "SE(3), no scale")

        bag_info = (RESULTS / "bag_info.txt").read_text(encoding="utf-8")
        bag_summary = json.loads((RESULTS / "bag_roundtrip_summary.json").read_text(encoding="utf-8"))
        self.assertIn("ROS Distro:        jazzy", bag_info)
        self.assertIn("Messages:          180", bag_info)
        for topic in ("/camera/color/image_raw", "/camera/depth/image_raw", "/camera/camera_info"):
            self.assertRegex(bag_info, rf"Topic: {re.escape(topic)}[^\n]+Count: 60")
            self.assertEqual(bag_summary["recorded_topic_message_counts"][topic], 60)
        self.assertEqual(bag_summary["frames_processed_after_replay"], 60)

        rviz = json.loads((RESULTS / "rviz_summary.json").read_text(encoding="utf-8"))
        image_path = ROOT / "docs" / "images" / "ros2_rviz_bag_demo.png"
        with Image.open(image_path) as image:
            self.assertEqual(list(image.size), rviz["screenshot_size_px"])
        self.assertEqual(rviz["frames_processed"], 60)
        self.assertIn("/cloud", rviz["visible_displays"])


if __name__ == "__main__":
    unittest.main()
