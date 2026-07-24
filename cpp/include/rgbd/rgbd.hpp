#pragma once

#include "rgbd/types.hpp"

#include <filesystem>

namespace rgbd {

struct CameraIntrinsics {
    double fx = 525.0;
    double fy = 525.0;
    double cx = 319.5;
    double cy = 239.5;
    double depth_scale = 5000.0;
};

PointCloud load_depth_point_cloud(
    const std::filesystem::path& rgb_path,
    const std::filesystem::path& depth_path,
    const CameraIntrinsics& intrinsics,
    int stride,
    double min_depth_m,
    double max_depth_m);

PointCloud depth_to_points_u16(
    const unsigned short* depth,
    int width,
    int height,
    int row_stride_elements,
    const CameraIntrinsics& intrinsics,
    int stride,
    double min_depth_m,
    double max_depth_m);

}  // namespace rgbd
