# 历史基线：TUM fr1/xyz 完整序列

> **无效历史结果，不可引用。** 该运行生成于相邻帧位姿累积方向错误修复之前，并且尚未加入显式失败判据，仅保留用于审计开发过程。当前正式结果以 [`parameter_sweep/README.md`](parameter_sweep/README.md) 和 [`parameter_sweep/benchmark_table.csv`](parameter_sweep/benchmark_table.csv) 为准。

This is a compact, versionable record of the locally executed run on 2026-07-14. Its source of truth is the locally generated `artifacts/tum_fr1_xyz_full/summary.json`.

Command:

```powershell
.\.venv\Scripts\python.exe -m src.run_odometry `
  --dataset data\rgbd_dataset_freiburg1_xyz `
  --output artifacts\tum_fr1_xyz_full
```

The detailed metrics were removed from the public-facing record because the pose-composition direction bug invalidated them. The only valid full-sequence comparison is the corrected 18-run parameter sweep linked above. This file remains solely to document why an earlier artifact directory must not be cited.
