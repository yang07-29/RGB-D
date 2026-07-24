# Python 与 C++17 公平基线

本表来自 2026-07-24 在同一台 Windows 11 主机上的真实全序列运行。两种实现使用同一 TUM `fr1/xyz` 796 帧关联、相同内参/深度尺度、stride=8、voxel=0.05 m、对应阈值 0.12 m、最多 30 次 ICP 迭代、单位相对位姿初值、相同失败判据和同一 SE(3) ATE / Δ=1 帧 RPE 定义。真值只用于最终评测。

| 实现 | ATE RMSE (m) | RPE 平移 (m) | RPE 旋转 (°) | 均值 (ms) | p95 (ms) | FPS | RSS 峰值 (MiB) | 接受/拒绝帧对 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Python 3.12 NumPy/SciPy | 0.175859 | 0.011151 | 0.574578 | 15.585 | 27.398 | 64.16 | 160.87 | 786 / 9 |
| Windows C++17 Eigen/OpenCV | 0.175783 | 0.011151 | 0.574568 | 11.395 | 16.801 | 87.76 | 33.72 | 786 / 9 |
| Ubuntu 24.04 C++17 Eigen/OpenCV | 0.175783 | 0.011151 | 0.574568 | 12.218 | 18.964 | 81.84 | 71.12 | 786 / 9 |

ATE 的绝对差为 0.0000757 m；两者接受/拒绝完全一致，RPE 近似逐位一致。微小 ATE 差异来自不同浮点运算顺序与最近邻平局处理。性能数字只代表此次顺序运行和当前机器，不外推到 Linux、机器人或 Jetson。

端到端耗时包括 RGB/深度 PNG 读取、uint16 深度反投影、体素下采样和相邻帧 ICP；不包含结果绘图。两者均为 CPU-only，GPU 显存和模型大小不适用。C++ 的 ICP 残差与 Python 一致：对应点数量使用距离门限，RMSE 使用变换后所有源点的最近邻距离。

Windows 复现命令：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_and_run_cpp.ps1
```

Ubuntu 依赖与命令：

```bash
sudo apt-get install -y build-essential cmake ninja-build libeigen3-dev libopencv-dev
./scripts/build_and_run_cpp.sh data/rgbd_dataset_freiburg1_xyz
```

Windows 原始证据在本地 `artifacts/cpp_full_voxel_0.05_corr_0.12/`；Ubuntu 结果来自 GitHub Actions 成功运行 [30070922101](https://github.com/yang07-29/rgbd-pointcloud-registration/actions/runs/30070922101)，持久证据见 [`../linux_cpp_full/`](../linux_cpp_full/)。两平台几何指标一致，但延迟和 RSS 必须分别表述。
