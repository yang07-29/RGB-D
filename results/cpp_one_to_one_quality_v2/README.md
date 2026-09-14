# Windows C++17 一对一关联结果

运行日期：2026-09-14。数据：TUM RGB-D `fr1/xyz`。

- 时间戳协议：`one_to_one_minimum_offset_greedy_v2`
- 帧数：790；帧对：789
- 参数：stride=8、voxel=0.05 m、correspondence=0.12 m、ICP 最多 30 次
- 接受 / 拒绝：780 / 9
- ATE RMSE：0.175657 m（SE(3) 对齐）
- RPE Δ=1：0.011160 m / 0.575273°
- 端到端计算延迟：mean 13.962 ms、p95 21.651 ms
- 峰值 RSS：35,123,200 bytes（33.5 MiB）
- 环境：Windows 11、GCC 16.1.0、OpenCV 4.13.0、Eigen header-only；未使用 GPU

`summary.json` 是汇总依据；`icp_steps.csv` 保留逐帧对状态、点数、残差和时间；两条 TUM 轨迹分别是未对齐输出和仅用于 ATE 可视化的 SE(3) 对齐轨迹。

复现：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_and_run_cpp.ps1 `
  -Dataset data\rgbd_dataset_freiburg1_xyz `
  -Output artifacts\cpp_full_one_to_one_quality_v2
```
