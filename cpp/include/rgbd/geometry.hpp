#pragma once

#include "rgbd/types.hpp"

#include <cstddef>

namespace rgbd {

struct RigidTransform {
    Mat3 rotation = Mat3::Identity();
    Vec3 translation = Vec3::Zero();
};

struct IcpResult {
    RigidTransform transform;
    double rmse_m = 0.0;
    int iterations = 0;
    std::size_t correspondences = 0;
};

PointCloud transform_points(const PointCloud& points, const RigidTransform& transform);
RigidTransform kabsch(const PointCloud& source, const PointCloud& target);
PointCloud voxel_downsample(const PointCloud& points, double voxel_size_m);
IcpResult icp_point_to_point(
    const PointCloud& source,
    const PointCloud& target,
    int max_iterations,
    double tolerance,
    double max_correspondence_m);

}  // namespace rgbd
