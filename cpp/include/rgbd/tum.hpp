#pragma once

#include "rgbd/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace rgbd {

struct RgbdFrame {
    double timestamp = 0.0;
    std::filesystem::path rgb_path;
    std::filesystem::path depth_path;
    double depth_time_offset_s = 0.0;
    Mat4 ground_truth = Mat4::Identity();
    double ground_truth_time_offset_s = 0.0;
};

std::vector<RgbdFrame> load_tum_rgbd_frames(
    const std::filesystem::path& dataset_dir,
    double max_depth_time_offset_s = 0.02,
    double max_ground_truth_time_offset_s = 0.02);

std::string pose_to_tum_row(double timestamp, const Mat4& pose);

}  // namespace rgbd
