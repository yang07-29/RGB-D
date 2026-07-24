# rgbd_odometry_ros

ROS 2 Jazzy package wrapping the repository's tested C++17 `rgbd_core`.

- Approximate-time RGB/depth synchronization with a 20 ms default slop.
- Camera intrinsics from `CameraInfo`; TUM publisher emits `fx=fy=525`, `cx=319.5`, `cy=239.5`.
- CPU point-to-point ICP with explicit correspondence-ratio and RMSE rejection.
- Publishes odometry, path, transformed point cloud, and `odom -> camera_link` TF.
- Writes per-callback latency, synchronization counters, ICP quality, and peak RSS to CSV.

See [`../../README.md`](../../README.md) for build, bag, and RViz commands. The package is source-complete but must not be claimed as runtime-verified until it has been built and replayed in a real ROS 2 environment.
