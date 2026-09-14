# TUM 多序列固定参数评测

`fr1/xyz` 用于选择固定配置；`fr1/desk` 与 `fr1/desk2` 只做配置迁移评测。两种 ICP 都固定使用 voxel=0.05 m、correspondence=0.08 m，真值只用于运行后的指标计算。

时间戳协议：`one_to_one_minimum_offset_greedy_v2`。

| 序列 | 方法 | 帧数 | ATE(m) | RPE Δ1(m/°) | RPE Δ30(m/°) | 均值/p95(ms) | FPS | RSS(MB) | 拒绝帧对 |
|---|---|---|---|---|---|---|---|---|---|
| fr1_desk | frame_to_frame_point_to_point_icp | 573 | 0.428433 | 0.014485/1.037 | 0.274035/13.872 | 25.16/53.36 | 39.75 | 135.3 | 6 |
| fr1_desk | identity_no_motion_baseline | 573 | 0.871404 | 0.017882/1.472 | 0.473937/26.533 | N/A | N/A | N/A | N/A |
| fr1_desk | open3d_point_to_plane_icp | 573 | 0.225788 | 0.009722/1.007 | 0.112465/6.866 | 26.75/31.38 | 37.38 | 212.3 | 6 |
| fr1_desk2 | frame_to_frame_point_to_point_icp | 612 | 0.551096 | 0.019083/1.323 | 0.339317/17.856 | 30.07/58.05 | 33.26 | 139.2 | 19 |
| fr1_desk2 | identity_no_motion_baseline | 612 | 0.970802 | 0.018937/1.904 | 0.505939/34.464 | N/A | N/A | N/A | N/A |
| fr1_desk2 | open3d_point_to_plane_icp | 612 | 0.391915 | 0.010539/1.125 | 0.121493/8.757 | 27.64/32.14 | 36.18 | 220.0 | 18 |
| fr1_xyz | frame_to_frame_point_to_point_icp | 790 | 0.180483 | 0.011009/0.579 | 0.213658/5.275 | 17.77/30.69 | 56.28 | 146.3 | 21 |
| fr1_xyz | identity_no_motion_baseline | 790 | 0.186227 | 0.011286/0.679 | 0.274607/10.627 | N/A | N/A | N/A | N/A |
| fr1_xyz | open3d_point_to_plane_icp | 790 | 0.058947 | 0.005687/0.551 | 0.046375/2.705 | 27.44/30.41 | 36.44 | 229.4 | 22 |

ATE 使用 SE(3) 对齐，不做尺度缩放。RPE 同时报告 Δ=1 和 Δ=30 帧。性能只对实际 ICP 行有意义；静止基线记为 N/A。

逐次运行的原始 `summary.json` 路径保存在 CSV 中。
