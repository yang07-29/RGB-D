"""Record TUM camera topics to rosbag2, replay them, and verify odometry output.

This Windows runner uses the repository-local RoboStack environment.  It sends
CTRL_BREAK to the recorder so rosbag2 can finalize metadata and SQLite cleanly.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


def clean_ros_environment(root: Path, domain_id: int) -> tuple[dict[str, str], Path, Path, Path]:
    prefix = (root / ".conda-ros2").resolve()
    install = (root / "ros2_ws" / "install").resolve()
    python = prefix / "python.exe"
    ros2_script = prefix / "Library" / "bin" / "ros2-script.py"
    required = [prefix, install, python, ros2_script]
    if not all(path.exists() for path in required):
        raise FileNotFoundError("Build .conda-ros2 and ros2_ws/install before running this script")
    windir = Path(os.environ["WINDIR"])
    env = os.environ.copy()
    env["PATH"] = os.pathsep.join(
        str(path)
        for path in [
            install / "Lib" / "rgbd_odometry_py",
            prefix / "Library" / "opt" / "rviz_ogre_vendor" / "bin",
            prefix / "Library" / "opt" / "rviz_ogre_vendor" / "lib",
            prefix / "Library" / "opt" / "rviz_ogre_vendor" / "lib64",
            prefix,
            prefix / "Library" / "bin",
            prefix / "Scripts",
            prefix / "bin",
            windir / "System32",
            windir,
        ]
    )
    env["PYTHONPATH"] = os.pathsep.join(
        [str(install / "Lib" / "site-packages"), str(prefix / "Lib" / "site-packages")]
    )
    env["AMENT_PREFIX_PATH"] = os.pathsep.join([str(install), str(prefix / "Library")])
    env["CMAKE_PREFIX_PATH"] = env["AMENT_PREFIX_PATH"]
    env["COLCON_PREFIX_PATH"] = str(install)
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["ROS_DISTRO"] = "jazzy"
    env["PYTHONNOUSERSITE"] = "1"
    env["OMP_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    return env, python, ros2_script, install


def start_process(arguments: list[str], env: dict[str, str], log_base: Path) -> tuple[subprocess.Popen, object, object]:
    stdout = (log_base.with_suffix(".stdout.log")).open("w", encoding="utf-8")
    stderr = (log_base.with_suffix(".stderr.log")).open("w", encoding="utf-8")
    process = subprocess.Popen(
        arguments,
        env=env,
        stdout=stdout,
        stderr=stderr,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    )
    return process, stdout, stderr


def stop_gracefully(process: subprocess.Popen | None, *, timeout: float = 15.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        if os.name == "nt":
            process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            process.send_signal(signal.SIGINT)
        process.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        process.kill()
        process.wait(timeout=5)


def wait_for_text(path: Path, needle: str, deadline: float) -> None:
    while time.monotonic() < deadline:
        if path.exists() and needle in path.read_text(encoding="utf-8", errors="replace"):
            return
        time.sleep(0.25)
    raise TimeoutError(f"Did not observe {needle!r} in {path}")


def metric_row_count(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open(newline="", encoding="utf-8") as handle:
        return sum(1 for _ in csv.DictReader(handle))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--domain-id", type=int, default=48)
    parser.add_argument("--output", type=Path, default=Path("artifacts/ros2_bag_roundtrip"))
    args = parser.parse_args()
    if args.frames < 3 or args.publish_hz <= 0:
        parser.error("--frames must be at least 3 and --publish-hz must be positive")

    root = Path(__file__).resolve().parents[1]
    output = (root / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    bag = (output / "tum_rgbd_input_bag").resolve()
    if output not in bag.parents:
        raise RuntimeError("Refusing to manage a rosbag outside the selected artifact directory")
    if bag.exists():
        shutil.rmtree(bag)
    for name in ["replay_metrics.csv", "replay_trajectory.txt", "bag_info.txt", "summary.json"]:
        (output / name).unlink(missing_ok=True)

    env, python, ros2_script, install = clean_ros_environment(root, args.domain_id)
    dataset = (root / "data" / "rgbd_dataset_freiburg1_xyz").resolve()
    params = install / "share" / "rgbd_odometry_ros" / "config" / "odometry.yaml"
    recorder = publisher = player = node = None
    metadata_reindexed = False
    handles: list[object] = []
    start = time.monotonic()
    try:
        recorder, out, err = start_process(
            [
                str(python), str(ros2_script), "bag", "record", "-o", str(bag),
                "--storage", "sqlite3", "--max-cache-size", "0", "--disable-keyboard-controls",
                "--topics", "/camera/color/image_raw", "/camera/depth/image_raw", "/camera/camera_info",
            ],
            env,
            output / "record",
        )
        handles += [out, err]
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise RuntimeError("rosbag2 recorder exited before input publishing")

        hz = f"{args.publish_hz:.9g}"
        if "." not in hz:
            hz += ".0"
        publisher, out, err = start_process(
            [
                str(python), "-m", "rgbd_odometry_ros.tum_rgbd_publisher", "--ros-args",
                "-p", f"dataset:={dataset}", "-p", f"publish_hz:={hz}",
                "-p", f"max_frames:={args.frames}",
            ],
            env,
            output / "record_publisher",
        )
        handles += [out, err]
        wait_for_text(
            output / "record_publisher.stderr.log",
            "Finished publishing TUM sequence",
            time.monotonic() + max(30.0, args.frames / args.publish_hz + 15.0),
        )
        stop_gracefully(publisher)
        publisher = None
        time.sleep(1.0)
        stop_gracefully(recorder, timeout=20.0)
        recorder = None
        if not (bag / "metadata.yaml").exists():
            # RoboStack's Windows CLI does not currently turn CTRL_BREAK into
            # the same shutdown path as Ctrl-C on a POSIX terminal. The SQLite
            # writer is configured with no cache, so use rosbag2's official
            # recovery command and then verify exact per-topic counts via info.
            subprocess.run(
                [str(python), str(ros2_script), "bag", "reindex", "-s", "sqlite3", str(bag)],
                env=env,
                text=True,
                capture_output=True,
                timeout=30,
                check=True,
            )
            metadata_reindexed = True
        if not (bag / "metadata.yaml").exists():
            raise RuntimeError("rosbag2 metadata is still missing after official reindex")

        info = subprocess.run(
            [str(python), str(ros2_script), "bag", "info", str(bag)],
            env=env,
            text=True,
            capture_output=True,
            timeout=30,
            check=True,
        )
        (output / "bag_info.txt").write_text(info.stdout + info.stderr, encoding="utf-8")
        recorded_topics = [
            "/camera/color/image_raw", "/camera/depth/image_raw", "/camera/camera_info"
        ]
        topic_counts: dict[str, int] = {}
        for topic in recorded_topics:
            match = re.search(
                rf"Topic:\s*{re.escape(topic)}\s*\|[^\n]*Count:\s*(\d+)", info.stdout
            )
            if match is None:
                raise RuntimeError(f"ros2 bag info did not report {topic}")
            topic_counts[topic] = int(match.group(1))
        if any(count != args.frames for count in topic_counts.values()):
            raise RuntimeError(f"Recorded topic counts are incomplete: {topic_counts}")

        metrics = output / "replay_metrics.csv"
        trajectory = output / "replay_trajectory.txt"
        node, out, err = start_process(
            [
                str(python), "-m", "rgbd_odometry_ros.rgbd_odometry_py_node", "--ros-args",
                "--params-file", str(params), "-p", f"metrics_csv:={metrics}",
                "-p", f"trajectory_tum:={trajectory}",
            ],
            env,
            output / "replay_odometry",
        )
        handles += [out, err]
        time.sleep(2.0)
        if node.poll() is not None:
            raise RuntimeError("Odometry node exited before rosbag replay")
        player, out, err = start_process(
            [str(python), str(ros2_script), "bag", "play", str(bag)],
            env,
            output / "play",
        )
        handles += [out, err]
        player.wait(timeout=max(45.0, args.frames / args.publish_hz + 30.0))
        if player.returncode != 0:
            raise RuntimeError(f"rosbag2 play failed with exit code {player.returncode}")
        player = None
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and metric_row_count(metrics) < args.frames:
            time.sleep(0.25)
        processed = metric_row_count(metrics)
        if processed != args.frames:
            raise RuntimeError(f"Replay odometry processed {processed}/{args.frames} recorded frames")
        if len(trajectory.read_text(encoding="utf-8").splitlines()) != args.frames:
            raise RuntimeError("Replay trajectory row count does not match the recorded frame count")

        summary = {
            "status": "success",
            "frames_requested": args.frames,
            "frames_processed_after_replay": processed,
            "publish_hz": args.publish_hz,
            "ros_domain_id": args.domain_id,
            "rmw_implementation": env["RMW_IMPLEMENTATION"],
            "bag_storage": "sqlite3",
            "metadata_reindexed_after_windows_ctrl_break": metadata_reindexed,
            "recorded_topic_message_counts": topic_counts,
            "elapsed_wall_s": time.monotonic() - start,
            "bag": str(bag),
            "metrics": str(metrics),
            "trajectory": str(trajectory),
        }
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    finally:
        for process in [player, node, publisher, recorder]:
            stop_gracefully(process)
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    if os.name != "nt":
        print("This runner currently targets the verified Windows RoboStack environment.", file=sys.stderr)
    main()
