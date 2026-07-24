"""Portable ROS 2 RGB-D ICP odometry node used when the C++ toolchain is absent."""

from __future__ import annotations

import csv
from pathlib import Path
import threading
import time

import message_filters
import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .geometry import depth_to_points, icp_point_to_point, transform_points, voxel_downsample


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RgbdOdometryPyNode(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_odometry")
        self.rgb_topic = self.declare_parameter("rgb_topic", "/camera/color/image_raw").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_raw").value
        camera_info_topic = self.declare_parameter("camera_info_topic", "/camera/camera_info").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_link").value
        self.stride = int(self.declare_parameter("stride", 8).value)
        self.voxel_m = float(self.declare_parameter("voxel_m", 0.05).value)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.2).value)
        self.max_depth_m = float(self.declare_parameter("max_depth_m", 4.0).value)
        self.max_iterations = int(self.declare_parameter("max_iterations", 30).value)
        self.max_correspondence_m = float(self.declare_parameter("max_correspondence_m", 0.12).value)
        self.min_correspondence_ratio = float(self.declare_parameter("min_correspondence_ratio", 0.5).value)
        self.max_acceptable_rmse_m = float(self.declare_parameter("max_acceptable_rmse_m", 0.12).value)
        sync_slop_s = float(self.declare_parameter("sync_slop_s", 0.02).value)
        metrics_csv = Path(self.declare_parameter("metrics_csv", "ros2_metrics.csv").value)
        trajectory_tum = Path(self.declare_parameter("trajectory_tum", "ros2_trajectory.txt").value)
        metrics_csv.parent.mkdir(parents=True, exist_ok=True)
        trajectory_tum.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_handle = metrics_csv.open("w", newline="", encoding="utf-8")
        self.trajectory_handle = trajectory_tum.open("w", encoding="utf-8")
        self.metrics = csv.writer(self.metrics_handle)
        self.metrics.writerow(
            [
                "processed_frame", "rgb_stamp_s", "depth_stamp_s", "rgb_depth_offset_s", "rgb_received",
                "depth_received", "unsynchronized_or_pending", "status", "point_count", "correspondences",
                "rmse_m", "callback_latency_ms", "process_peak_rss_bytes",
            ]
        )
        self.process = psutil.Process()
        self.peak_rss = self.process.memory_info().rss
        self.camera_info = None
        self.previous_cloud = None
        self.pose = np.eye(4)
        self.path = PathMessage()
        self.path.header.frame_id = self.odom_frame
        self.processed = 0
        self.rgb_received = 0
        self.depth_received = 0
        # Some Windows RMW/executor combinations may deliver synchronized
        # callbacks concurrently. ICP pose accumulation and csv.writer are
        # stateful, so serialize the complete frame transaction explicitly.
        self.callback_lock = threading.Lock()

        self.rgb_sub = message_filters.Subscriber(
            self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data,
        )
        self.rgb_sub.registerCallback(self._count_rgb)
        self.depth_sub.registerCallback(self._count_depth)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=20, slop=sync_slop_s,
        )
        self.synchronizer.registerCallback(self.rgbd_callback)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.path_pub = self.create_publisher(PathMessage, "path", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "cloud", qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"Python backend subscribing RGB={self.rgb_topic} depth={self.depth_topic}")

    def _count_rgb(self, _message) -> None:
        self.rgb_received += 1

    def _count_depth(self, _message) -> None:
        self.depth_received += 1

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def pending(self) -> int:
        return max(0, min(self.rgb_received, self.depth_received) - self.processed)

    def rgbd_callback(self, rgb_message: Image, depth_message: Image) -> None:
        with self.callback_lock:
            self._process_rgbd(rgb_message, depth_message)

    def _process_rgbd(self, rgb_message: Image, depth_message: Image) -> None:
        start = time.perf_counter()
        status = "ok"
        cloud = np.empty((0, 3), dtype=np.float64)
        correspondences = 0
        rmse_m = float("nan")
        try:
            if self.camera_info is None:
                raise RuntimeError("waiting for CameraInfo")
            if depth_message.encoding not in ("16UC1", "mono16"):
                raise RuntimeError(f"unsupported depth encoding {depth_message.encoding}")
            row_elements = depth_message.step // 2
            depth = np.frombuffer(depth_message.data, dtype="<u2").reshape(depth_message.height, row_elements)
            depth = depth[:, : depth_message.width]
            cloud = depth_to_points(
                depth,
                fx=float(self.camera_info.k[0]), fy=float(self.camera_info.k[4]),
                cx=float(self.camera_info.k[2]), cy=float(self.camera_info.k[5]), depth_scale=5000.0,
                stride=self.stride, min_depth_m=self.min_depth_m, max_depth_m=self.max_depth_m,
            )
            cloud = voxel_downsample(cloud, self.voxel_m)
            if self.previous_cloud is None:
                status = "initialized"
            else:
                result = icp_point_to_point(
                    cloud, self.previous_cloud, max_iterations=self.max_iterations,
                    max_correspondence_m=self.max_correspondence_m,
                )
                correspondences = result.correspondences
                rmse_m = result.rmse_m
                ratio = correspondences / len(cloud) if len(cloud) else 0.0
                if ratio < self.min_correspondence_ratio:
                    status = f"rejected_low_ratio_{ratio:.3f}"
                elif rmse_m > self.max_acceptable_rmse_m:
                    status = f"rejected_high_rmse_{rmse_m:.4f}"
                else:
                    self.pose = self.pose @ result.transform
            self.previous_cloud = cloud
            self.publish_outputs(rgb_message.header.stamp, cloud)
            self.write_trajectory(rgb_message.header.stamp)
        except Exception as error:  # keep the stream alive and record the exact failure
            status = f"failed_{error}"
            self.get_logger().warning(status)
        self.processed += 1
        latency_ms = (time.perf_counter() - start) * 1000.0
        self.peak_rss = max(self.peak_rss, self.process.memory_info().rss)
        self.metrics.writerow(
            [
                self.processed, stamp_seconds(rgb_message.header.stamp), stamp_seconds(depth_message.header.stamp),
                abs(stamp_seconds(rgb_message.header.stamp) - stamp_seconds(depth_message.header.stamp)),
                self.rgb_received, self.depth_received, self.pending(), status, len(cloud), correspondences,
                "" if not np.isfinite(rmse_m) else rmse_m, latency_ms, self.peak_rss,
            ]
        )
        self.metrics_handle.flush()
        if self.processed % 30 == 0:
            self.get_logger().info(
                f"processed={self.processed} latency={latency_ms:.2f} ms pending_or_dropped={self.pending()}"
            )

    def write_trajectory(self, stamp) -> None:
        quaternion = Rotation.from_matrix(self.pose[:3, :3]).as_quat()
        translation = self.pose[:3, 3]
        values = [stamp_seconds(stamp), *translation, *quaternion]
        self.trajectory_handle.write(" ".join(f"{float(value):.9f}" for value in values) + "\n")
        self.trajectory_handle.flush()

    def publish_outputs(self, stamp, camera_cloud: np.ndarray) -> None:
        quaternion = Rotation.from_matrix(self.pose[:3, :3]).as_quat()
        translation = self.pose[:3, 3]
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.camera_frame
        odometry.pose.pose.position.x = float(translation[0])
        odometry.pose.pose.position.y = float(translation[1])
        odometry.pose.pose.position.z = float(translation[2])
        odometry.pose.pose.orientation.x = float(quaternion[0])
        odometry.pose.pose.orientation.y = float(quaternion[1])
        odometry.pose.pose.orientation.z = float(quaternion[2])
        odometry.pose.pose.orientation.w = float(quaternion[3])
        self.odom_pub.publish(odometry)

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        world_cloud = transform_points(camera_cloud, self.pose).astype(np.float32)
        header = Header(stamp=stamp, frame_id=self.odom_frame)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, world_cloud.tolist()))

    def destroy_node(self):
        self.metrics_handle.close()
        self.trajectory_handle.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RgbdOdometryPyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
