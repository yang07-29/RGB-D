# ROS2 Jazzy：Windows RoboStack 真实运行结果

本目录保存可公开的小型证据。原始 TUM 图像、隔离环境、构建目录及 88.1 MiB rosbag 位于被 Git 忽略的 `data/`、`.conda-ros2/`、`ros2_ws/install/` 和 `artifacts/`，不会上传 GitHub。

精确的直接依赖版本保存在 `configs/environment-ros2.yml`，实测平台版本快照保存在本目录的 `environment.json`。

## 796 帧在线数据链路

2026-07-24 在 Windows、RoboStack ROS2 Jazzy、CycloneDDS、CPU-only Python 后端上运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_ros2_tum_demo.ps1 `
  -Frames 796 -PublishHz 10.0 -TimeoutSeconds 180 `
  -OutputDirectory artifacts/ros2_full_fr1_xyz

.\.venv\Scripts\python.exe -m src.evaluate_saved_trajectory `
  --dataset data\rgbd_dataset_freiburg1_xyz `
  --trajectory artifacts\ros2_full_fr1_xyz\trajectory_local.txt `
  --output artifacts\ros2_full_fr1_xyz\evaluation `
  --method ros2_python_point_to_point_icp
```

ROS 发布器与离线评测使用逐项相同的 796 个 RGB 时间戳：RGB-depth 和 RGB-groundtruth 最近时间差均不超过 20 ms。groundtruth 只在数据集回放工具中筛选评测帧，并在运行后计算指标；真值位姿不发布、不输入里程计。

| 指标 | 真实结果 |
| --- | ---: |
| 处理帧 | 796 / 796 |
| 接受 / 拒绝相邻帧对 | 786 / 9 |
| ATE RMSE，SE(3) 对齐、无尺度（m） | 0.175402 |
| RPE 平移 RMSE，Δ=1（m） | 0.011151 |
| RPE 旋转 RMSE，Δ=1（度） | 0.574628 |
| ROS 回调均值 / 中位数 / p95（ms） | 11.298 / 10.010 / 20.841 |
| 由回调均值换算吞吐（FPS） | 88.51 |
| 进程 RSS 峰值 | 98,025,472 B / 93.48 MiB |
| RGB/depth 接收末值 | 796 / 796 |
| 未同步或待处理末值 | 0 |

性能范围包含深度消息解析、反投影、体素下采样、point-to-point ICP、轨迹/TF/点云消息构造与发布，以及 CSV/RSS 记录；不包含 TUM 发布器读 PNG 的时间。回放节流为 10 Hz，所以 88.51 FPS 是回调计算吞吐，不是端到端摄像头帧率。

## rosbag2 录制与独立回放

```powershell
.\.venv\Scripts\python.exe scripts\run_ros2_bag_roundtrip.py `
  --frames 60 --publish-hz 10 --output artifacts/ros2_bag_roundtrip
```

- SQLite bag 共 180 条消息：RGB、depth、CameraInfo 各 60 条。
- 停止实时 TUM 发布器后启动新里程计节点，从 bag 回放并处理 60/60 帧。
- 回放轨迹 60 行、时间戳与同一评测子集完全一致；ATE RMSE 为 0.024906 m（仅前 60 帧，不能与 796 帧 ATE 横向比较）。
- RoboStack Windows 下 CTRL_BREAK 未生成 `metadata.yaml`；脚本设置 `--max-cache-size 0` 后使用 ROS2 官方 `ros2 bag reindex` 收尾，并通过 `ros2 bag info` 强制检查三个话题各 60 条。该平台差异在结果中保留，不包装成无异常的原生 Ubuntu 行为。

## RViz

```powershell
.\.venv\Scripts\python.exe scripts\capture_ros2_rviz_demo.py
```

脚本回放上述 bag，确认新节点处理 60/60 帧后截取真实 RViz 主窗口。RViz 使用 `odom` 固定坐标系，显示 `/path`、Best Effort `/cloud`、`/tf` 和 Grid；日志确认 OpenGL 4.6，且修复后无 QoS 不兼容警告。

自动截图结束时脚本用信号关闭 RViz，RoboStack 的 `class_loader` 会输出一条“对象仍存在、库不卸载”的退出警告；它发生在截图和 60/60 处理完成之后，不是运行中点云/轨迹错误，原始日志仍保留在本地 artifact。

![ROS2 rosbag RViz demo](../../docs/images/ros2_rviz_bag_demo.png)

## 证据索引

- `full_796_performance.json`：796 帧性能、状态和消息计数。
- `environment.json`：CPU、Windows、Python、ROS/RMW 与依赖版本、分辨率和 GPU 未使用边界。
- `full_796_evaluation.json`：统一 ATE/RPE 与 SE(3) 对齐参数。
- `full_796_metrics.csv`：逐回调延迟、RMSE、状态、点数与 RSS。
- `full_796_trajectory_local.txt`：节点直接输出的未对齐 TUM 轨迹。
- `full_topic_list.txt`、`odom_once.yaml`：ROS graph 和独立 `/odom` 接收证据。
- `bag_info.txt`、`bag_roundtrip_summary.json`：rosbag 消息数与回放闭环。
- `rviz_summary.json`：真实窗口标题、截图尺寸和处理帧数。

本目录的运行指标只证明 Windows RoboStack 下的 ROS2 Python 数据链路。本机没有 Visual Studio 2022 C++ 工具链，因此未声称本地 ROS2 C++ 运行。Ubuntu C++ 的独立证据来自 GitHub Actions 运行 [30074936189](https://github.com/yang07-29/rgbd-pointcloud-registration/actions/runs/30074936189)，详见 [`../ros2_cpp_linux_full/`](../ros2_cpp_linux_full/)；两个平台的性能范围不能混用，也都没有机器人、Jetson、功耗或实机传感器结果。
