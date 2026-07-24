from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("rgbd_odometry_ros"))
    dataset = LaunchConfiguration("dataset")
    use_tum_publisher = LaunchConfiguration("use_tum_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    odometry_executable = LaunchConfiguration("odometry_executable")
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset", default_value="data/rgbd_dataset_freiburg1_xyz"),
            DeclareLaunchArgument("use_tum_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("odometry_executable", default_value="rgbd_odometry_node"),
            Node(
                package="rgbd_odometry_ros",
                executable="tum_rgbd_publisher",
                name="tum_rgbd_publisher",
                output="screen",
                parameters=[{"dataset": dataset}],
                condition=IfCondition(use_tum_publisher),
            ),
            Node(
                package="rgbd_odometry_ros",
                executable=odometry_executable,
                name="rgbd_odometry",
                output="screen",
                parameters=[str(share / "config" / "odometry.yaml")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "rviz" / "rgbd_odometry.rviz")],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
