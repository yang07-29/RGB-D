"""Publish a local TUM RGB-D sequence as standard ROS 2 camera topics."""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as ImageMessage

from .tum_io import associate_rgb_depth


def to_stamp(seconds: float):
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1e9))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    from builtin_interfaces.msg import Time

    return Time(sec=whole, nanosec=nanoseconds)


class TumRgbdPublisher(Node):
    def __init__(self) -> None:
        super().__init__("tum_rgbd_publisher")
        dataset = Path(self.declare_parameter("dataset", "data/rgbd_dataset_freiburg1_xyz").value)
        publish_hz = float(self.declare_parameter("publish_hz", 30.0).value)
        startup_delay_s = float(self.declare_parameter("startup_delay_s", 0.0).value)
        self.loop = bool(self.declare_parameter("loop", False).value)
        self.camera_frame = str(self.declare_parameter("camera_frame", "camera_link").value)
        max_frames = int(self.declare_parameter("max_frames", 0).value)
        if publish_hz <= 0 or startup_delay_s < 0:
            raise ValueError("publish_hz must be positive and startup_delay_s must be non-negative")
        self.frames = associate_rgb_depth(dataset)
        if max_frames > 0:
            self.frames = self.frames[:max_frames]
        self.index = 0
        self.startup_deadline = time.monotonic() + startup_delay_s
        self.rgb_pub = self.create_publisher(ImageMessage, "/camera/color/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(ImageMessage, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_frame)
        self.get_logger().info(
            f"Loaded {len(self.frames)} associated RGB-D pairs from {dataset}; "
            f"startup delay={startup_delay_s:.1f}s"
        )

    def publish_frame(self) -> None:
        if time.monotonic() < self.startup_deadline:
            return
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
            else:
                self.timer.cancel()
                self.get_logger().info("Finished publishing TUM sequence")
                return
        rgb_timestamp, depth_timestamp, rgb_path, depth_path = self.frames[self.index]
        rgb = np.asarray(Image.open(rgb_path).convert("RGB"), dtype=np.uint8)
        depth = np.asarray(Image.open(depth_path), dtype=np.uint16)
        rgb_stamp = to_stamp(rgb_timestamp)
        depth_stamp = to_stamp(depth_timestamp)

        rgb_message = ImageMessage()
        rgb_message.header.stamp = rgb_stamp
        rgb_message.header.frame_id = self.camera_frame
        rgb_message.height, rgb_message.width = rgb.shape[:2]
        rgb_message.encoding = "rgb8"
        rgb_message.is_bigendian = 0
        rgb_message.step = rgb_message.width * 3
        rgb_message.data = rgb.tobytes()

        depth_message = ImageMessage()
        depth_message.header.stamp = depth_stamp
        depth_message.header.frame_id = self.camera_frame
        depth_message.height, depth_message.width = depth.shape
        depth_message.encoding = "16UC1"
        depth_message.is_bigendian = 0
        depth_message.step = depth_message.width * 2
        depth_message.data = depth.astype("<u2", copy=False).tobytes()

        info = CameraInfo()
        info.header = rgb_message.header
        info.height = rgb_message.height
        info.width = rgb_message.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info)
        self.rgb_pub.publish(rgb_message)
        self.depth_pub.publish(depth_message)
        self.index += 1


def main() -> None:
    rclpy.init()
    node = TumRgbdPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
