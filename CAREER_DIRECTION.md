# 求职第二路线：3D 视觉 / SLAM / 机器人感知算法工程师

## 定位

本项目服务于从“水声信号处理/AI”扩展到 **3D 视觉、SLAM、机器人感知与 C++ 视觉算法**岗位的作品集。核心卖点不是专利名称，而是可运行、可复现、可解释的几何算法和工程能力。

当前已真实证明的能力：Python/NumPy/SciPy/Open3D 几何基线、Windows 与 Ubuntu 的 C++17/Eigen/OpenCV/CMake 全序列热路径、RGB-D 点云预处理、Kabsch/ICP、多帧里程计、ATE/RPE、关键帧/回环/位姿图、MobileNetV3/SE 回环描述子，以及 Windows RoboStack 下的 ROS2 Python/rosbag/RViz 数据链路。ROS2 C++ 已在 Ubuntu CI 编译测试；Jetson 和机器人仍无硬件证据。

## 两周执行顺序

### 第 1 周：把 Python 几何基线做成优质项目（已完成）

1. 在独立的 Python 3.12/Open3D 环境中安装 Open3D；不改动已验证的 Python 3.13 `.venv`。
2. 使用完全相同的 796 帧关联、相同内参和现有 ATE/RPE 代码，新增 Open3D point-to-plane ICP 或 RGB-D Odometry。
3. 统一比较：静止基线、自实现 point-to-point ICP、Open3D/point-to-plane 方法。
4. 做参数网格：体素 `0.03/0.05/0.08 m` × 对应点阈值 `0.08/0.12/0.20 m`。
5. 对每次实验保存 ATE、RPE、FPS、p95、RSS、失败帧数和失败原因，形成一张表。

### 第 2 周：C++17 + Linux 核心链路（已完成）

1. 在 Ubuntu 或 WSL 中建立 CMake/C++17 工程。
2. 用 Eigen 实现刚体变换和 SVD；用 OpenCV 读 RGB-D；用 PCL 或 Open3D C++ 处理点云。
3. 优先迁移热路径：反投影 → 预处理 → ICP → 位姿累积。
4. 保持与 Python 版本相同的数据、帧关联、指标与失败判据；补单元测试和性能对比。

## 后续顺序与状态

1. 已完成：关键帧、传统/学习回环候选、局部精配准与位姿图优化，并报告优化前后 ATE 和错误边。
2. 已完成（限定平台）：Windows RoboStack 的 rosbag 回放、图像/相机内参订阅、`/odom`/`/path`/TF/点云发布、RViz 与延迟/RSS；ROS2 C++ 已在 Ubuntu CI 编译测试，但 C++ ROS graph 全序列运行待做。
3. 已完成（限定数据）：MobileNetV3+SE 服务于回环检索，包含序列隔离、HSV/无 SE 基线、Recall/F1 和回环后 ATE；跨数据集泛化待做。
4. 未开始：Jetson；仅在真实硬件上记录 JetPack、ROS2、ONNX/TensorRT、端到端延迟、RAM/显存、功耗和演示视频。

## 当前项目的诚实命名

当前可以诚实称为“RGB-D 里程计与离线回环/位姿图重建工程”，但不能称为机器人实机 SLAM 产品。可以展示 Python、Windows/Ubuntu C++17+CMake、Windows ROS2 Python 数据链路和 Ubuntu ROS2 C++ 编译测试能力；不能把 CI 编译表述成 ROS2 C++ 全序列运行或机器人部署。
