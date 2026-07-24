# Evidence Log

This file separates verified work from patent-plan statements. Do not copy a claim into a resume, presentation, or interview answer until its evidence column is complete.

| Item | Current status | Evidence to keep | Safe wording now |
| --- | --- | --- | --- |
| Published patent application | Verified | Public patent PDF: `CN121095441A`; inventor list and publication date | "Second inventor of a published patent application on lightweight ROS 3D reconstruction." |
| Synthetic point-cloud registration baseline | Verified after tests pass | Test command output, commit hash, `artifacts/synthetic/icp_before_after.png` | "Implemented and tested a NumPy/SciPy point-to-point ICP baseline on controlled synthetic point clouds." |
| RGB-D back-projection | Verified on real data | `artifacts/tum_fr1_xyz_full/summary.json`: 796 timestamp-associated frames and 413--2111 post-voxel points/frame | "Implemented calibrated RGB-D depth back-projection and voxelized point clouds on TUM fr1/xyz." |
| Real TUM RGB-D registration | Verified baselines; not competitive | `results/parameter_sweep/benchmark_table.csv`, 18 source `summary.json` files, trajectories and step CSVs | "Implemented and evaluated point-to-point and Open3D point-to-plane ICP RGB-D odometry baselines over 9 parameter settings on all 796 valid TUM fr1/xyz associations. Best ATE RMSE was 0.054045 m; drift remains and the method is not presented as SOTA or full SLAM." |
| C++17/CMake hot path | Verified on Windows and Ubuntu 24.04 | `cpp/`, CTest output, `results/cpp_baseline/`, `results/linux_cpp_full/`, GitHub run `30070922101` | "Migrated RGB-D loading/back-projection, voxel downsampling, exact KD-tree ICP, pose accumulation and ATE/RPE to C++17 with Eigen/OpenCV/CMake; Windows and Ubuntu runs reproduced all 796 frames with identical trajectory metrics." |
| Keyframes, loop closure, pose graph | Verified offline on TUM fr1/xyz | `src/pose_graph_slam.py`, `results/pose_graph/`, local optimized pose graph/PLY/CSV/plots | "Selected 129 keyframes, verified FPFH/RANSAC loop candidates with ICP and optimized an Open3D pose graph; keyframe ATE RMSE decreased from 0.040787 m to 0.023802 m, while two odometry-distant hard negatives were rejected." |
| Lightweight learned loop descriptor | Verified offline on three sequence-isolated TUM splits | `src/train_loop_descriptor.py`, `results/loop_learning/`, local checkpoints/descriptors, 10-epoch history across two variants | "Trained and ablated MobileNetV3-Small loop descriptors with/without SE using fr1/desk train, fr1/desk2 validation and held-out fr1/xyz test; the SE model reached 0.9574/0.9787 Recall@1/5 and 0.6768 pair F1 under the recorded protocol." |
| Learned candidates in pose graph | Verified offline on TUM fr1/xyz | `results/loop_learning/learned_pose_graph_*`, plots and local pose graph/map | "Fed frozen RGB descriptors into geometry-verified loop discovery; keyframe ATE decreased from 0.040787 m to 0.024579 m. Post-hoc GT labelled 28 accepted edges correct and 2 slightly beyond the predeclared 5 cm criterion." |
| ROS2 integration | Python runtime verified on Windows RoboStack; C++ build/test verified on Ubuntu CI | `results/ros2/`, `ros2_ws/`, GitHub run `30070818522`, local full-run/rosbag/RViz artifacts | "Ran a ROS2 Jazzy Python RGB-D odometry node on all 796 TUM associations, published odom/path/cloud/TF, recorded/replayed a rosbag and verified RViz; separately compiled and tested the ROS2 C++ node on Ubuntu 24.04." Do not relabel the CI result as C++ ROS graph runtime, robot, or Jetson evidence. |
| Jetson/robot deployment | Not implemented/verified in this rebuild | Hardware/model/version, commands, runtime and memory logs | Do not claim 5 Hz, memory savings, or on-device deployment. |

## Result record template

Completed for the full-sequence run below. The raw `summary.json` remains the source of truth.

```text
Date: 2026-07-24
Dataset and sequence: local TUM RGB-D freiburg1/xyz, 796 valid timestamp associations
Camera intrinsics / depth scale: fx=fy=525, cx=319.5, cy=239.5; scale=5000
Point-cloud preprocessing: stride 8, 0.2--4.0 m, 0.05 m voxel grid
Registration initialization: identity for each consecutive pair; no ground truth in solver
Experiment grid: point-to-point and Open3D point-to-plane; 30 iterations maximum; voxel 0.03/0.05/0.08 m; correspondence gate 0.08/0.12/0.20 m
Identity baseline ATE RMSE: 0.18581362410504695 m
Best ATE: 0.054045 m (Open3D point-to-plane, voxel=0.05 m, correspondence=0.08 m)
Best-ATE RPE (one frame): 0.005549 m translation; 0.546981 deg rotation
Saved artifacts: artifacts/parameter_sweep/*/summary.json and results/parameter_sweep/benchmark_table.csv
Known limitation: frame-to-frame odometry still drifts; point-to-plane consumes more RSS, and the sequence does not validate loop closure or full SLAM.
```

```text
Date: 2026-07-24
C++ build: GCC 16.1.0, C++17, Eigen 5.0.1, OpenCV 4.13.0, CMake 4.3.3, Ninja 1.13.2
Protocol: same 796 frames and 0.05 m / 0.12 m point-to-point configuration as the Python comparison row
Accepted/rejected pairs: 786 / 9 in both implementations
C++ ATE / RPE translation / RPE rotation: 0.175783 m / 0.011151 m / 0.574568 deg
C++ end-to-end mean / p95 / RSS peak: 11.395 ms / 16.801 ms / 33.72 MiB
Ubuntu evidence: GitHub run 30070922101; same 796 frames; ATE/RPE 0.175783 m / 0.011151 m / 0.574568 deg; mean/p95 12.218/18.964 ms; RSS 71.12 MiB
Evidence: results/cpp_baseline/, results/linux_cpp_full/, and artifacts/cpp_full_voxel_0.05_corr_0.12/
Boundary: Windows and GitHub-hosted Ubuntu measurements are separate machines; performance differences cannot be attributed only to the OS.
```

```text
Date: 2026-07-24
Pose-graph protocol: 796 frames -> 129 keyframes; 128 odometry edges; 30 proximity candidates; FPFH/RANSAC -> point-to-plane ICP -> geometric gates
Loop evidence: 30 accepted; 30 post-hoc correct; 0 post-hoc incorrect
Hard-negative evidence: 2/2 rejected by the same gates; both post-hoc incorrect; never inserted into graph
Keyframe ATE RMSE: 0.040787 m before -> 0.023802 m after (41.64% reduction)
Evidence: results/pose_graph/ and artifacts/pose_graph_fr1_xyz/
Boundary: offline TUM keyframe pose graph, not a real-time ROS2 node or hardware SLAM deployment.
```

```text
Date: 2026-07-24
Learning split: train=fr1/desk (596), validation=fr1/desk2 (631), held-out test=fr1/xyz (796)
Labels: minimum 30-frame separation; positive <=0.20 m and <=25 deg; validation-only similarity thresholds
Training: ImageNet-initialized MobileNetV3-Small, 128-D L2 descriptor, triplet margin loss, 5 epochs per variant, seed 20260724
Test Recall@1 / Recall@5: HSV 0.9291 / 0.9433; no-SE 0.9574 / 0.9787; SE 0.9574 / 0.9787
Test pair F1: HSV 0.6247; no-SE 0.6025; SE 0.6768
SE checkpoint / GPU forward mean / p95: 3.935 MiB / 5.057 ms / 8.695 ms on RTX 5060 Laptop GPU
Learned-loop keyframe ATE: 0.040787 m before -> 0.024579 m after (39.74% reduction)
Accepted learned edges post-hoc: 28 correct / 2 incorrect under the predeclared 0.05 m and 5 deg rule
Evidence: results/loop_learning/ and artifacts/loop_learning/ + artifacts/pose_graph_learned_fr1_xyz/
Boundary: three TUM indoor sequences only; no cross-dataset or learned-descriptor-in-ROS2 claim, and no Jetson or robot claim.
```

```text
Date: 2026-07-24
ROS2 environment: Windows RoboStack Jazzy, CycloneDDS, Python backend, CPU only
Protocol: same 796 RGB/depth/groundtruth timestamp associations and 0.05 m / 0.12 m point-to-point gates
Processed / accepted / rejected: 796 / 786 / 9; pending at end: 0
ATE / RPE translation / RPE rotation: 0.175402 m / 0.011151 m / 0.574628 deg
Callback mean / median / p95 / throughput: 11.298 ms / 10.010 ms / 20.841 ms / 88.51 FPS
Peak process RSS: 98,025,472 bytes (93.48 MiB)
rosbag evidence: 60 RGB + 60 depth + 60 CameraInfo messages; fresh-node replay processed 60/60
RViz evidence: 1280x800 real window capture, Global Status OK, path/cloud/TF enabled and cloud visible
Evidence: results/ros2/ and local artifacts/ros2_full_fr1_xyz, ros2_bag_roundtrip, ros2_rviz_demo
Boundary: the ROS graph runtime evidence is Windows/Python. Ubuntu run 30070818522 proves ROS2 C++ compilation/tests only, not C++ full-sequence topic performance, robot or Jetson deployment.
```

## Non-negotiable rule

An AI-generated draft, a patent plan, or code that has not been executed is not experimental evidence. It is okay to call it a technical proposal or a reconstruction-in-progress; it is not okay to state its planned performance as a measured result.
