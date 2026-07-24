# 求职作品表述：RGB-D 几何里程计

## 目标岗位

**3D 视觉 / SLAM / 机器人感知算法工程师**。当前作品以 RGB-D 几何、点云配准和多帧里程计为基础，已补充 C++17、Ubuntu 与 ROS2 C++ 全序列工程证据，后续继续强化多序列验证与实机部署能力。

## 简历表述：现在可以使用

```text
RGB-D 点云配准与多帧里程计 | 公开专利申请 CN121095441A（第二发明人）
- 重建公开技术路线中的可复现几何基线：TUM RGB-D 时间戳关联、深度反投影、体素下采样、Kabsch、point-to-point ICP 与轨迹累积。
- 在 TUM fr1/xyz 的 796 个有效关联帧上完成 18 组公平实验；Open3D point-to-plane 最佳 ATE RMSE 为 0.054045 m、RPE 平移 RMSE 为 0.005549 m，保存逐帧延迟、RSS、失败判据和原始轨迹/CSV/JSON。
- 用 C++17、Eigen、OpenCV 与 CMake 迁移 RGB-D 读取、反投影、体素下采样、精确 KD-tree、Kabsch/ICP 和轨迹评测；全序列与 Python 都接受 786/795 帧对，ATE 绝对差 0.000076 m。
- 在 Ubuntu 24.04 GitHub Actions 中从 TUM 官方源下载数据并复现全部 796 帧：独立 C++ 核心 ATE 0.175783 m，端到端均值/p95 为 12.218/18.964 ms；ROS2 Jazzy C++ 节点通过真实 topic graph 处理 796/796 帧，回调均值/p95 为 3.074/7.161 ms。
- 构建 129 关键帧位姿图，以 FPFH/RANSAC+ICP 验证 30 个非局部回环，关键帧 ATE RMSE 从 0.040787 m 降至 0.023802 m；额外错误候选压力测试 2/2 被门限拒绝。
- 使用 fr1/desk、desk2、xyz 做训练/验证/测试序列隔离，训练 MobileNetV3-Small 128D 回环描述子并完成无 SE/有 SE 消融；测试 Recall@1/5 为 0.9574/0.9787，SE 模型 pair-F1 为 0.6768。
- 将冻结学习描述子接入几何验证与位姿图，关键帧 ATE 从 0.040787 m 降至 0.024579 m；保留并分析 2 个略超 5 cm 标准的错误接受边。
- 在 Windows RoboStack ROS2 Jazzy 上跑通 796 帧 Python RGB-D 节点，发布 odom/path/cloud/TF，记录回调延迟与 RSS；另完成 60 帧 rosbag2 录制/回放和 RViz 轨迹/点云展示。
- 如实分析局限：ROS2 C++ 证据来自 GitHub runner 的公开数据回放，不是相机端到端、Jetson 或机器人部署。
```

## 当前仍不能加入简历的内容

Jetson/TensorRT、机器人实机数据和真实相机端到端性能仍需真实运行证据，不能提前占位。Ubuntu C++ 核心与 ROS2 C++ 全序列、Windows RoboStack ROS2 Python/rosbag/RViz 和学习描述子已有证据，可以使用上面的限定表述，但不能扩展为硬件部署、跨数据集泛化或 SOTA。

## 面试 60 秒回答

```text
这项专利最初是轻量化 ROS 三维重建的技术方案，其中很多模块没有在硬件上被我完整验证，所以我不把计划性能当作结果。我从几何链路重建，完成 RGB-D 关联、反投影、Kabsch/ICP、多帧位姿和 ATE/RPE，并在 TUM fr1/xyz 的 796 帧上比较 point-to-point 与 point-to-plane。核心热路径已迁移到 C++17/Eigen/OpenCV/CMake，并在 Windows 与 Ubuntu 全序列交叉复现；随后加入关键帧、FPFH/RANSAC 回环和位姿图，把关键帧 ATE 从 0.040787 米降到 0.023802 米。ROS2 方面在 Windows 跑通 Python 节点、rosbag、话题和 RViz，也在 Ubuntu 用 C++ 节点通过真实 ROS graph 处理 796 帧；我会明确这些仍是公开数据回放，不是机器人、Jetson 或真实相机部署。
```

## 你必须能回答的问题

1. Why does a depth pixel become a 3D point, and what do `fx`, `fy`, `cx`, `cy`, and depth scale mean?
2. Why do outliers and density hurt nearest-neighbour ICP?
3. What Kabsch solves, and why ICP can converge to the wrong local optimum?
4. What the before/after metric means and which frames were used?
5. Which parts are verified code, which are planned system components, and what you would implement next?
