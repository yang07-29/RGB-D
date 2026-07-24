"""Summarize the per-frame CSV emitted by the ROS 2 odometry node."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


def summarize(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("ROS 2 metrics CSV contains no rows")
    latencies = np.asarray([float(row["callback_latency_ms"]) for row in rows], dtype=float)
    timestamps = np.asarray([float(row["rgb_stamp_s"]) for row in rows], dtype=float)
    offsets = np.asarray([float(row["rgb_depth_offset_s"]) for row in rows], dtype=float)
    valid_duration = float(timestamps[-1] - timestamps[0])
    message_hz = float((len(timestamps) - 1) / valid_duration) if valid_duration > 0 and len(timestamps) > 1 else None
    rss_values = [int(row["process_peak_rss_bytes"]) for row in rows if row.get("process_peak_rss_bytes")]
    final_pending = int(rows[-1]["unsynchronized_or_pending"])
    return {
        "samples": len(rows),
        "callback_latency_ms": {
            "mean": float(np.mean(latencies)),
            "median": float(np.median(latencies)),
            "p95": float(np.percentile(latencies, 95)),
            "fps_from_mean": float(1000.0 / np.mean(latencies)),
        },
        "input_message_rate_hz_from_timestamps": message_hz,
        "rgb_depth_offset_s": {
            "mean": float(np.mean(offsets)),
            "max": float(np.max(offsets)),
        },
        "status_counts": dict(Counter(row["status"] for row in rows)),
        "rgb_received_final": int(rows[-1]["rgb_received"]),
        "depth_received_final": int(rows[-1]["depth_received"]),
        "unsynchronized_or_pending_final": final_pending,
        "process_peak_rss_bytes": max(rss_values) if rss_values else None,
        "gpu": "not used",
        "note": "Pending count is sampled during callbacks; inspect node shutdown log for the final post-playback count.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = summarize(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
