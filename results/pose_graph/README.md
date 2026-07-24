# 关键帧、回环与位姿图：TUM fr1/xyz

这是 2026-07-24 在本机真实运行的结果。命令：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_pose_graph.ps1
```

协议：先在全部 796 个关联帧上运行 Open3D point-to-plane 里程计，再按 0.05 m 平移、10° 旋转或最多 40 帧间隔选择 129 个关键帧。相邻关键帧构成 128 条确定性 ICP 边；非局部候选完全由估计轨迹接近性产生，FPFH+RANSAC 给出全局初值，point-to-plane ICP 精配准。回环是否入图只检查 global/refined fitness、RMSE 和相对里程计一致性，不读取真值。

Open3D 官方定义中，位姿图边的变换应把 source 点云对齐到 target 点云；本实现按此方向构造边，并将回环边标为 `uncertain=True`。参考：[Open3D 0.19 Multiway Registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)。

## 正式结果

| 指标（相同 129 个关键帧） | 优化前 | 优化后 | 变化 |
| --- | ---: | ---: | ---: |
| ATE RMSE (m)，一次 SE(3) 对齐 | 0.040787 | 0.023802 | 降低 41.64% |
| RPE 平移 RMSE (m)，Δ=1 keyframe | 0.012491 | 0.012352 | 降低 1.11% |
| RPE 旋转 RMSE (°)，Δ=1 keyframe | 1.208288 | 1.203777 | 降低 0.37% |

- 30 个正式回环候选全部通过门限并进入图优化。
- 真值事后检查：30 个接受边均为正确，0 个为错误；此标签未参与生成或接受。
- 完整实验耗时 19.725 s，进程 RSS 峰值 239.79 MiB，CPU-only。
- 指标只代表 `fr1/xyz` 的关键帧轨迹，不冒充完整 SLAM 排行或其他 TUM 序列结果。

精确数值见 [`summary.csv`](summary.csv)。

## 失败案例

为验证门限不是只会接受，实验额外从**估计轨迹中距离最远**的非局部关键帧选择 2 个 hard-negative；这一步同样不使用真值。两次 FPFH/RANSAC 都给出错误变换，但被 refined fitness、RMSE 和里程计一致性门限拒绝，且从未加入位姿图：

- 帧 201→368：后验真值误差 3.752 m / 156.30°。
- 帧 310→368：后验真值误差 0.522 m / 76.79°。

正确回环与错误候选的精确记录见 [`loop_cases.csv`](loop_cases.csv)。原始逐候选记录保存在本地 `artifacts/pose_graph_fr1_xyz/`。

![优化前后关键帧轨迹](../../docs/images/pose_graph_trajectory.png)

![正确回环与被拒绝的错误候选](../../docs/images/loop_correct_and_rejected_cases.png)

## 真实性边界

- 真值只用于最终 ATE/RPE 和候选的事后正确/错误标签。
- 这一步证明的是离线关键帧位姿图，不是实时 ROS2 SLAM，也没有机器人或 Jetson 运行证据。
- hard-negative 是验证器压力测试，不计入正式回环候选数量，也不进入优化图。
