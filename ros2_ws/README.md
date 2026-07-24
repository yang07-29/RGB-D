# ROS 2 Jazzy 离线 RGB-D 里程计工作区

该工作区面向 Ubuntu 24.04 + ROS 2 Jazzy。它不需要机器人：可把本地 TUM `fr1/xyz` 发布为标准相机话题，或者先录成 rosbag 再回放。

当前仓库已实现但**尚未在 ROS 2 运行时编译/回放**，原因是本机 WSL 未启用且安装需要管理员 PowerShell。不能把以下模块写成已实测，直到 `results/ros2/` 有真实命令输出、CSV、截图和 bag 元数据。

## 接口

输入：

- `/camera/color/image_raw` — `sensor_msgs/msg/Image`，`rgb8`/`bgr8`
- `/camera/depth/image_raw` — `sensor_msgs/msg/Image`，当前要求 `16UC1`
- `/camera/camera_info` — `sensor_msgs/msg/CameraInfo`

输出：

- `/odom` — `nav_msgs/msg/Odometry`
- `/path` — `nav_msgs/msg/Path`
- `/cloud` — `sensor_msgs/msg/PointCloud2`，已变换到 `odom`
- TF：`odom -> camera_link`
- `ros2_metrics.csv`：同步偏差、状态、点数、ICP 对应/RMSE、回调延迟、消息计数与进程峰值 RSS

## Ubuntu 24.04 构建

ROS 2 Jazzy 官方支持 Ubuntu Noble 24.04。安装 `ros-jazzy-desktop`、`ros-dev-tools`、`libeigen3-dev` 与 `libopencv-dev` 后：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 直接发布 TUM 数据并显示 RViz

从仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch rgbd_odometry_ros tum_rgbd_demo.launch.py \
  dataset:=$PWD/data/rgbd_dataset_freiburg1_xyz \
  use_tum_publisher:=true \
  use_rviz:=true
```

## 录制与回放 rosbag

终端 A：

```bash
ros2 bag record -o artifacts/tum_fr1_xyz_bag \
  /camera/color/image_raw /camera/depth/image_raw /camera/camera_info
```

终端 B：

```bash
ros2 run rgbd_odometry_ros tum_rgbd_publisher \
  --ros-args -p dataset:=$PWD/data/rgbd_dataset_freiburg1_xyz
```

停止录制后，回放验证：

```bash
ros2 launch rgbd_odometry_ros tum_rgbd_demo.launch.py use_tum_publisher:=false use_rviz:=true
ros2 bag play artifacts/tum_fr1_xyz_bag
```

汇总节点性能：

```bash
python3 -m src.summarize_ros2_metrics \
  --input ros2_metrics.csv \
  --output artifacts/ros2_metrics_summary.json
```

## 必须保存的真实证据

- `colcon build` 与 CTest/单元测试输出；
- `ros2 bag info`、`ros2 topic hz`、`ros2 topic echo --once /odom`；
- `ros2_metrics.csv` 与汇总 JSON；
- RViz 截图和一分钟录屏；
- Ubuntu/ROS2/RMW/CPU/内存版本；
- 输入消息数、处理帧数、同步未匹配数、mean/median/p95 延迟。

官方参考：[ROS 2 Jazzy Ubuntu 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)、[ROS 2 包开发](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)。
