# rgbd_odometry_ros

ROS 2 Jazzy package wrapping the repository's tested C++17 `rgbd_core`.

- Approximate-time RGB/depth synchronization with a 20 ms default slop.
- Camera intrinsics from `CameraInfo`; TUM publisher emits `fx=fy=525`, `cx=319.5`, `cy=239.5`.
- CPU point-to-point ICP with explicit correspondence-ratio and RMSE rejection.
- Publishes odometry, path, transformed point cloud, and `odom -> camera_link` TF.
- Writes per-callback latency, synchronization counters, ICP quality, and peak RSS to CSV.

On Windows, `scripts/run_ros2_tum_demo.ps1` keeps `ROS_LOG_DIR` inside the selected artifacts directory so the run does not depend on write access to `%USERPROFILE%\.ros`. The script defaults to CycloneDDS and accepts `-RmwImplementation rmw_fastrtps_cpp` for discovery troubleshooting. A successful run must contain the requested metrics/trajectory rows and a captured `/odom` message; process startup alone is not considered success.

See [`../../README.md`](../../README.md) for build, bag, and RViz commands. The package is source-complete but must not be claimed as runtime-verified until it has been built and replayed in a real ROS 2 environment.
