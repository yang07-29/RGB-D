from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    core_share = Path(get_package_share_directory("rgbd_odometry_ros"))
    dataset = LaunchConfiguration("dataset")
    use_tum_publisher = LaunchConfiguration("use_tum_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset", default_value="data/rgbd_dataset_freiburg1_xyz"),
            DeclareLaunchArgument("use_tum_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="rgbd_odometry_py",
                executable="tum_rgbd_publisher",
                output="screen",
                parameters=[{"dataset": dataset}],
                condition=IfCondition(use_tum_publisher),
            ),
            Node(
                package="rgbd_odometry_py",
                executable="rgbd_odometry_py_node",
                name="rgbd_odometry",
                output="screen",
                parameters=[str(core_share / "config" / "odometry.yaml")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(core_share / "rviz" / "rgbd_odometry.rviz")],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
