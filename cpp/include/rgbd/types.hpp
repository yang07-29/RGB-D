#pragma once

#include <Eigen/Core>
#include <vector>

namespace rgbd {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using PointCloud = std::vector<Vec3>;

inline Mat4 make_pose(const Mat3& rotation, const Vec3& translation) {
    Mat4 pose = Mat4::Identity();
    pose.block<3, 3>(0, 0) = rotation;
    pose.block<3, 1>(0, 3) = translation;
    return pose;
}

}  // namespace rgbd
