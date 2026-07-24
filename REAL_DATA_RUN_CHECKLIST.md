# 真实数据运行清单

该清单已在 TUM RGB-D `freiburg1/xyz` 上完成。具体命令、参数和结果以 `README.md`、`EVIDENCE_LOG.md` 与 `results/` 为准。

- [x] 记录数据集名称、解压目录、640×480 图像尺寸、`uint16` 深度类型和 5000 深度尺度。
- [x] 以 20 ms 阈值关联 RGB、depth 与 ground truth，得到 796 个三元组。
- [x] 导出并目视检查真实彩色 PLY，核对点数、空间范围和深度方向。
- [x] 在不读取真值初始化的前提下运行相邻帧 ICP 和连续多帧位姿累积。
- [x] 保存逐帧 RMSE、接受/拒绝状态、轨迹、ATE/RPE、延迟和 RSS。
- [x] 完成 NumPy/SciPy、Open3D point-to-plane 与 C++17 公平基线。
- [x] 完成关键帧、回环、位姿图、学习描述子与限定平台的 ROS2/RViz 验证。
- [x] 在 `EVIDENCE_LOG.md` 和结果文档中记录成功结果、失败案例与真实性边界。
- [x] 在 GitHub Actions 中完成 Ubuntu C++/ROS2 C++ 编译测试和 Linux 796 帧核心全序列验证。
- [x] 完成 ROS2 C++ 节点的 ROS graph 796 帧运行与 Python 后端同协议比较（GitHub Actions `30074936189`）。
- [ ] 真实 Jetson/机器人部署；没有硬件证据前保持未完成状态。
