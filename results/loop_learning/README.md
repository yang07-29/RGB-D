# 轻量回环描述子与位姿图结果

本目录只汇总已经在本机真实运行的结果。大体积 TUM 数据、训练 checkpoint、逐帧描述子和点云地图位于被 Git 忽略的 `data/`、`artifacts/`，可由脚本重新生成。

## 不泄漏测试集的协议

- 训练：TUM RGB-D `fr1/desk`，596 个有效关联帧。
- 验证：`fr1/desk2`，631 帧；每种方法只在此序列选择余弦相似度阈值。
- 测试：`fr1/xyz`，796 帧；阈值冻结后只评测一次。
- 查询只检索至少早 30 帧的历史图像；正样本要求真值平移不超过 0.20 m 且旋转不超过 25°。
- 真值只生成训练标签、选择验证阈值和进行最终评测；模型推理只读取 RGB。
- 训练采用 1272 个三元组、MobileNetV3-Small ImageNet 初始化、128 维 L2 描述子、TripletMarginLoss、5 epochs、batch 32、固定随机种子 `20260724`。

完整机器可读协议见 [`protocol.json`](protocol.json)，逐轮记录见 [`training_history.csv`](training_history.csv)。

## 检索结果

测试集共有 141 个具备至少一个真值正样本的查询。Recall@K 只在这些查询上统计；pair precision/recall 对全部 59059 个有效历史候选对统计。

| 方法 | 测试 Recall@1 | Recall@5 | pair precision | pair recall | pair F1 | 参数量 | checkpoint | 推理 mean / p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 全局 HSV 直方图 | 0.9291 | 0.9433 | 0.7071 | 0.5595 | 0.6247 | N/A | N/A | 6.963 / 7.385 ms（CPU，含读图与直方图） |
| MobileNetV3，无 SE | 0.9574 | 0.9787 | 0.4512 | 0.9063 | 0.6025 | 539224 | 2.164 MiB | 3.291 / 7.075 ms（GPU，仅模型前向） |
| MobileNetV3 + SE | 0.9574 | 0.9787 | 0.6334 | 0.7267 | 0.6768 | 1000864 | 3.935 MiB | 5.057 / 8.695 ms（GPU，仅模型前向） |

GPU 为 NVIDIA GeForce RTX 5060 Laptop GPU；PyTorch 2.11.0+cu128、torchvision 0.26.0+cu128。GPU 延迟是同步后的 batch=1 模型前向，输入张量已在 GPU；HSV 行包含 CPU 读图和预处理，因此两种延迟作用域不能直接当作同一端到端基准。原始数值见 [`metrics.csv`](metrics.csv) 和 [`summary.json`](summary.json)。

结论应谨慎表述：两个学习模型在该测试协议上的 Recall@1/5 均高于 HSV；带 SE 模型没有继续提高 Recall@K，但相对无 SE 模型提高了测试 pair precision 和 F1。这里只验证了三个 TUM 室内序列，不能据此声称跨数据集泛化或 SOTA。

## 接入位姿图

冻结的 MobileNetV3+SE 描述子为 `fr1/xyz` 的 129 个关键帧提出回环候选，候选发现函数不接收位姿。候选随后使用与传统位姿候选相同的 FPFH/RANSAC、point-to-plane ICP 和几何门限验证，再进入 Open3D 位姿图。

| 项目 | 数值 |
| --- | ---: |
| 描述子候选 / 加入图的边 | 30 / 30 |
| 事后真值标签：正确 / 错误 | 28 / 2 |
| 优化前关键帧 ATE RMSE | 0.040787 m |
| 优化后关键帧 ATE RMSE | 0.024579 m |
| ATE 降幅 | 39.74% |
| 优化后 RPE 平移 / 旋转 RMSE（关键帧 Δ=1） | 0.012145 m / 1.198668° |
| 单线程位姿图总耗时（两次复跑） | 15.742 s / 15.523 s |
| 峰值进程 RSS（第二次） | 230.80 MiB |

两次单线程复跑的 ATE/RPE 和 28/2 标签完全一致，总耗时正常波动。两个“错误边”的事后平移误差分别约 0.0551 m 和 0.0536 m，略高于预先规定的 0.05 m 正确门槛；现有无真值门限没有区分它们。这是保留的真实失败案例，不能使用测试真值反向调整门限。尽管存在这两个边，本次完整图优化的 ATE 仍真实下降。

本次本地 checkpoint 的 SHA-256 为：无 SE `97e6df52754bbb3e3a6f7dc0ef05b0b31f78cd709cb522dac931ce1749292fd3`；带 SE `36685b506887ff4098966c932f1d6f8bada79a7d13af46609e6d4284a69d60f1`。checkpoint 本身不进入仓库；哈希用于区分本次证据与后来重新训练得到的权重。

机器可读证据见 [`learned_pose_graph_summary.json`](learned_pose_graph_summary.json)、[`learned_pose_graph_loop_candidates.csv`](learned_pose_graph_loop_candidates.csv) 和 [`learned_pose_graph_hard_negatives.csv`](learned_pose_graph_hard_negatives.csv)。

## 复现命令

```powershell
conda create --prefix .conda-learning python=3.13 pip --yes
.\.conda-learning\python.exe -m pip install -r requirements-learning.lock.txt
powershell -ExecutionPolicy Bypass -File scripts\run_loop_learning.ps1
powershell -ExecutionPolicy Bypass -File scripts\run_learned_pose_graph.ps1
```

第一次训练会由 torchvision 从 PyTorch 官方地址下载约 9.83 MiB 的 MobileNetV3-Small ImageNet 权重。完整训练在记录机器上用时 600.775 s，进程峰值 RSS 为 2464.59 MiB；checkpoint 保存到 `artifacts/loop_learning/`，不会自动提交到 Git。
