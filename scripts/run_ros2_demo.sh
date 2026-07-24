#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$repo_root/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd "$repo_root"
ros2 launch rgbd_odometry_ros tum_rgbd_demo.launch.py \
  dataset:="$repo_root/data/rgbd_dataset_freiburg1_xyz" \
  use_tum_publisher:=true \
  use_rviz:="${USE_RVIZ:-true}"
