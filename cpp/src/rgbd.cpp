#include "rgbd/rgbd.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <stdexcept>

namespace rgbd {

PointCloud depth_to_points_u16(
    const unsigned short* depth,
    int width,
    int height,
    int row_stride_elements,
    const CameraIntrinsics& intrinsics,
    int stride,
    double min_depth_m,
    double max_depth_m) {
    if (depth == nullptr || width <= 0 || height <= 0 || row_stride_elements < width || stride <= 0 ||
        intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || intrinsics.depth_scale <= 0.0 ||
        min_depth_m < 0.0 || max_depth_m <= min_depth_m) {
        throw std::invalid_argument("invalid depth image, intrinsics, or depth range");
    }
    PointCloud points;
    points.reserve(static_cast<std::size_t>((width / stride + 1) * (height / stride + 1)));
    for (int v = 0; v < height; v += stride) {
        const unsigned short* row = depth + static_cast<std::ptrdiff_t>(v) * row_stride_elements;
        for (int u = 0; u < width; u += stride) {
            const double z = static_cast<double>(row[u]) / intrinsics.depth_scale;
            if (!std::isfinite(z) || z < min_depth_m || z > max_depth_m) {
                continue;
            }
            points.emplace_back(
                (static_cast<double>(u) - intrinsics.cx) * z / intrinsics.fx,
                (static_cast<double>(v) - intrinsics.cy) * z / intrinsics.fy,
                z);
        }
    }
    if (points.size() < 3) {
        throw std::runtime_error("depth image produced fewer than three valid points");
    }
    return points;
}

PointCloud load_depth_point_cloud(
    const std::filesystem::path& rgb_path,
    const std::filesystem::path& depth_path,
    const CameraIntrinsics& intrinsics,
    int stride,
    double min_depth_m,
    double max_depth_m) {
    const cv::Mat rgb = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
    const cv::Mat depth = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);
    if (rgb.empty()) {
        throw std::runtime_error("failed to read RGB image: " + rgb_path.string());
    }
    if (depth.empty()) {
        throw std::runtime_error("failed to read depth image: " + depth_path.string());
    }
    if (depth.type() != CV_16UC1) {
        throw std::runtime_error("depth image must be 16-bit unsigned single-channel PNG");
    }
    if (rgb.rows != depth.rows || rgb.cols != depth.cols) {
        throw std::runtime_error("RGB and depth image sizes do not match");
    }
    return depth_to_points_u16(
        depth.ptr<unsigned short>(), depth.cols, depth.rows, static_cast<int>(depth.step1()),
        intrinsics, stride, min_depth_m, max_depth_m);
}

}  // namespace rgbd
