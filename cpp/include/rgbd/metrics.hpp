#pragma once

#include "rgbd/geometry.hpp"

#include <vector>

namespace rgbd {

struct AteMetrics {
    double rmse_m = 0.0;
    double mean_m = 0.0;
    double median_m = 0.0;
    double max_m = 0.0;
};

struct RpeMetrics {
    int frame_delta = 1;
    std::size_t pairs = 0;
    double translation_rmse_m = 0.0;
    double translation_mean_m = 0.0;
    double rotation_rmse_deg = 0.0;
    double rotation_mean_deg = 0.0;
};

Mat4 invert_pose(const Mat4& pose);
Mat4 compose_camera_to_world(const Mat4& previous_camera_to_world, const Mat4& current_to_previous);
std::vector<Mat4> align_estimated_poses(
    const std::vector<Mat4>& estimated,
    const std::vector<Mat4>& reference,
    RigidTransform* alignment = nullptr);
AteMetrics absolute_trajectory_error(const std::vector<Mat4>& aligned, const std::vector<Mat4>& reference);
RpeMetrics relative_pose_error(const std::vector<Mat4>& estimated, const std::vector<Mat4>& reference, int frame_delta = 1);

}  // namespace rgbd
