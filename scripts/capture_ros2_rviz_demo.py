"""Replay the verified rosbag through odometry and capture the real RViz window."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import subprocess
import time

from PIL import ImageGrab

from run_ros2_bag_roundtrip import (
    clean_ros_environment,
    metric_row_count,
    start_process,
    stop_gracefully,
)


def find_rviz_window(process_id: int, timeout: float = 15.0) -> tuple[int, str]:
    if os.name != "nt":
        raise RuntimeError("The verified RViz capture path currently targets Windows")
    user32 = ctypes.windll.user32
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    user32.IsWindowVisible.restype = wintypes.BOOL
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowThreadProcessId.restype = wintypes.DWORD
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextLengthW.restype = ctypes.c_int
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.GetWindowTextW.restype = ctypes.c_int
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process_matches: list[tuple[int, str]] = []
        title_matches: list[tuple[int, str]] = []

        @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
        def callback(hwnd, _lparam):
            if not user32.IsWindowVisible(hwnd):
                return True
            pid = wintypes.DWORD()
            user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
            length = user32.GetWindowTextLengthW(hwnd)
            title_buffer = ctypes.create_unicode_buffer(length + 1)
            user32.GetWindowTextW(hwnd, title_buffer, length + 1)
            title = title_buffer.value
            if int(pid.value) == process_id:
                process_matches.append((int(hwnd), title))
            elif "RViz" in title:
                title_matches.append((int(hwnd), title))
            return True

        user32.EnumWindows.argtypes = [type(callback), wintypes.LPARAM]
        user32.EnumWindows.restype = wintypes.BOOL
        user32.EnumWindows(callback, 0)
        if process_matches:
            return max(process_matches, key=lambda item: len(item[1]))
        if title_matches:
            return max(title_matches, key=lambda item: len(item[1]))
        time.sleep(0.25)
    raise TimeoutError("No visible RViz window appeared")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag", type=Path, default=Path("artifacts/ros2_bag_roundtrip/tum_rgbd_input_bag")
    )
    parser.add_argument("--domain-id", type=int, default=50)
    parser.add_argument("--expected-frames", type=int, default=60)
    parser.add_argument("--output", type=Path, default=Path("artifacts/ros2_rviz_demo"))
    parser.add_argument("--image", type=Path, default=Path("docs/images/ros2_rviz_bag_demo.png"))
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    output = (root / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    image_path = (root / args.image).resolve()
    image_path.parent.mkdir(parents=True, exist_ok=True)
    bag = (root / args.bag).resolve()
    if not (bag / "metadata.yaml").exists():
        raise FileNotFoundError(f"A finalized rosbag is required: {bag}")

    env, python, ros2_script, install = clean_ros_environment(root, args.domain_id)
    params = install / "share" / "rgbd_odometry_ros" / "config" / "odometry.yaml"
    rviz_config = install / "share" / "rgbd_odometry_ros" / "rviz" / "rgbd_odometry.rviz"
    rviz_executable = root / ".conda-ros2" / "Library" / "lib" / "rviz2" / "rviz2.exe"
    metrics = output / "rviz_metrics.csv"
    trajectory = output / "rviz_trajectory.txt"
    metrics.unlink(missing_ok=True)
    trajectory.unlink(missing_ok=True)

    node = rviz = player = None
    handles: list[object] = []
    start = time.monotonic()
    try:
        node, out, err = start_process(
            [
                str(python), "-m", "rgbd_odometry_ros.rgbd_odometry_py_node", "--ros-args",
                "--params-file", str(params), "-p", f"metrics_csv:={metrics}",
                "-p", f"trajectory_tum:={trajectory}",
            ],
            env,
            output / "odometry",
        )
        handles += [out, err]
        rviz, out, err = start_process(
            [str(rviz_executable), "-d", str(rviz_config)], env, output / "rviz"
        )
        handles += [out, err]
        # Let the splash screen close before selecting the long-lived main
        # window. The splash and main window share the same process ID.
        time.sleep(3.0)
        if node.poll() is not None or rviz.poll() is not None:
            raise RuntimeError("Odometry or RViz exited before rosbag playback")

        player, out, err = start_process(
            [str(python), str(ros2_script), "bag", "play", str(bag)], env, output / "play"
        )
        handles += [out, err]
        player.wait(timeout=45)
        if player.returncode != 0:
            raise RuntimeError(f"rosbag play failed with exit code {player.returncode}")
        player = None
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and metric_row_count(metrics) < args.expected_frames:
            time.sleep(0.25)
        processed = metric_row_count(metrics)
        if processed != args.expected_frames:
            raise RuntimeError(f"RViz demo odometry processed {processed}/{args.expected_frames} frames")

        hwnd, window_title = find_rviz_window(rviz.pid)
        user32 = ctypes.windll.user32
        window_handle = wintypes.HWND(hwnd)
        user32.ShowWindow.argtypes = [wintypes.HWND, ctypes.c_int]
        user32.ShowWindow.restype = wintypes.BOOL
        user32.SetForegroundWindow.argtypes = [wintypes.HWND]
        user32.SetForegroundWindow.restype = wintypes.BOOL
        user32.ShowWindow(window_handle, 9)  # SW_RESTORE
        user32.SetForegroundWindow(window_handle)
        time.sleep(1.0)
        class Rect(ctypes.Structure):
            _fields_ = [
                ("left", wintypes.LONG), ("top", wintypes.LONG),
                ("right", wintypes.LONG), ("bottom", wintypes.LONG),
            ]

        rectangle = Rect()
        user32.GetWindowRect.argtypes = [wintypes.HWND, ctypes.POINTER(Rect)]
        user32.GetWindowRect.restype = wintypes.BOOL
        if not user32.GetWindowRect(window_handle, ctypes.byref(rectangle)):
            raise OSError("GetWindowRect failed for the RViz window")
        # Capture the RViz HWND itself so unrelated foreground windows (for
        # example a Windows Firewall prompt) cannot contaminate the artifact.
        screenshot = ImageGrab.grab(window=hwnd)
        screenshot.save(image_path)
        if screenshot.width < 640 or screenshot.height < 400:
            raise RuntimeError(f"Unexpected RViz capture size: {screenshot.size}")

        summary = {
            "status": "success",
            "bag": str(bag),
            "frames_processed": processed,
            "window_title": window_title,
            "screenshot": str(image_path),
            "screenshot_size_px": [screenshot.width, screenshot.height],
            "elapsed_wall_s": time.monotonic() - start,
            "visible_displays": ["/path", "/cloud", "/tf", "Grid"],
        }
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    finally:
        for process in [player, rviz, node]:
            stop_gracefully(process)
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    main()
