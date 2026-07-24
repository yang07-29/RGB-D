# Ubuntu 24.04 C++17 全序列证据

该目录来自 GitHub Actions 手动工作流 [`linux-full-sequence.yml`](../../.github/workflows/linux-full-sequence.yml) 的真实成功运行：

- 工作流运行：[30070922101](https://github.com/yang07-29/rgbd-pointcloud-registration/actions/runs/30070922101)
- 提交：`2b17831f14b94d031f0bc30abb946974d4f67c8f`
- 平台：GitHub-hosted Ubuntu 24.04，x86-64
- 数据：TUM RGB-D `freiburg1/xyz` 官方压缩包
- 压缩包 SHA256：`a0236d97b8c30cd93b653656d2b6c293ff7c982a4130ef2a1a8beecdb124ef98`
- 帧数：796；相邻帧对：795；接受/拒绝：786 / 9

| 指标 | Linux C++17 结果 |
| --- | ---: |
| ATE RMSE，SE(3) 对齐 | 0.175783 m |
| RPE 平移 RMSE，Δ=1 | 0.011151 m |
| RPE 旋转 RMSE，Δ=1 | 0.574568° |
| 端到端均值 / 中位数 / p95 | 12.218 / 11.227 / 18.964 ms |
| FPS（由均值换算） | 81.84 |
| RSS 峰值 | 71.12 MiB |
| 总运行时间（不含依赖安装和下载） | 9.766 s |

端到端耗时包括 OpenCV RGB/深度读取、反投影、体素下采样和配准，不包括依赖安装、数据下载、构建或绘图。经典 ICP 为 CPU-only，没有模型权重或 GPU 显存。

目录中的 `summary.json` 是指标依据；`icp_steps.csv`、两条 TUM 轨迹、数据集哈希和 runner 环境用于独立核查。文件数量和行数已在下载 artifact 后验证：796 个轨迹位姿、795 条相邻帧记录，接受数与拒绝数之和为 795。

Linux 和 Windows C++ 的 ATE/RPE 数字一致，但延迟和 RSS 属于不同机器，不能把二者混成同一平台结果，也不能外推到 Jetson 或机器人。
