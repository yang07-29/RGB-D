#include "rgbd/geometry.hpp"
#include "rgbd/metrics.hpp"
#include "rgbd/rgbd.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_kabsch() {
    const rgbd::PointCloud source{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 3.0}};
    const rgbd::Mat3 rotation = Eigen::AngleAxisd(0.2, rgbd::Vec3::UnitZ()).toRotationMatrix();
    const rgbd::Vec3 translation(0.1, -0.05, 0.02);
    const rgbd::PointCloud target = rgbd::transform_points(source, {rotation, translation});
    const rgbd::RigidTransform recovered = rgbd::kabsch(source, target);
    require((recovered.rotation - rotation).norm() < 1e-10, "Kabsch rotation mismatch");
    require((recovered.translation - translation).norm() < 1e-10, "Kabsch translation mismatch");
    require(recovered.rotation.determinant() > 0.999999, "Kabsch returned a reflection");
}

void test_backprojection() {
    const unsigned short depth[9] = {0, 0, 0, 0, 2000, 0, 0, 0, 0};
    rgbd::CameraIntrinsics intrinsics;
    intrinsics.fx = 1.0;
    intrinsics.fy = 1.0;
    intrinsics.cx = 1.0;
    intrinsics.cy = 1.0;
    intrinsics.depth_scale = 1000.0;
    bool threw = false;
    try {
        static_cast<void>(rgbd::depth_to_points_u16(depth, 3, 3, 3, intrinsics, 1, 0.2, 4.0));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "backprojection must reject fewer than three valid points");

    const unsigned short valid_depth[9] = {1000, 0, 1000, 0, 2000, 0, 0, 0, 0};
    const rgbd::PointCloud points = rgbd::depth_to_points_u16(valid_depth, 3, 3, 3, intrinsics, 1, 0.2, 4.0);
    require(points.size() == 3, "backprojection valid-point count mismatch");
    require((points[2] - rgbd::Vec3(0.0, 0.0, 2.0)).norm() < 1e-12, "center pixel backprojection mismatch");
}

void test_voxel() {
    const rgbd::PointCloud points{{0.00, 0.00, 1.0}, {0.01, 0.01, 1.0}, {0.20, 0.0, 1.0}, {0.0, 0.20, 1.0}};
    const rgbd::PointCloud output = rgbd::voxel_downsample(points, 0.05);
    require(output.size() == 3, "voxel downsampling did not merge expected points");
}

void test_icp() {
    const rgbd::PointCloud source{
        {0.0, 0.0, 0.0}, {0.3, 0.0, 0.0}, {0.0, 0.4, 0.0}, {0.0, 0.0, 0.5},
        {0.2, 0.3, 0.1}, {-0.2, 0.1, 0.3}, {0.1, -0.3, 0.2}, {-0.1, -0.2, -0.3}};
    const rgbd::Mat3 rotation = Eigen::AngleAxisd(0.05, rgbd::Vec3::UnitY()).toRotationMatrix();
    const rgbd::Vec3 translation(0.025, -0.015, 0.01);
    const rgbd::PointCloud target = rgbd::transform_points(source, {rotation, translation});
    const rgbd::IcpResult result = rgbd::icp_point_to_point(source, target, 50, 1e-10, 0.15);
    require((result.transform.rotation - rotation).norm() < 1e-6, "ICP rotation mismatch");
    require((result.transform.translation - translation).norm() < 1e-6, "ICP translation mismatch");
    require(result.rmse_m < 1e-8, "ICP residual is not near zero");
}

void test_pose_direction_and_metrics() {
    rgbd::Mat4 previous = rgbd::Mat4::Identity();
    previous(0, 3) = 1.0;
    rgbd::Mat4 current_to_previous = rgbd::Mat4::Identity();
    current_to_previous(0, 3) = 0.2;
    const rgbd::Mat4 current = rgbd::compose_camera_to_world(previous, current_to_previous);
    require(std::abs(current(0, 3) - 1.2) < 1e-12, "pose accumulation inverted current-to-previous transform");

    std::vector<rgbd::Mat4> estimated(4, rgbd::Mat4::Identity());
    for (std::size_t i = 0; i < estimated.size(); ++i) {
        estimated[i](0, 3) = static_cast<double>(i) * 0.1;
        estimated[i].block<3, 3>(0, 0) = Eigen::AngleAxisd(0.01 * static_cast<double>(i), rgbd::Vec3::UnitZ()).toRotationMatrix();
    }
    const rgbd::Mat4 global = rgbd::make_pose(
        Eigen::AngleAxisd(0.4, rgbd::Vec3::UnitY()).toRotationMatrix(), rgbd::Vec3(1.0, 2.0, 3.0));
    std::vector<rgbd::Mat4> reference;
    for (const rgbd::Mat4& pose : estimated) {
        reference.push_back(global * pose);
    }
    const auto aligned = rgbd::align_estimated_poses(estimated, reference);
    require(rgbd::absolute_trajectory_error(aligned, reference).rmse_m < 1e-12, "ATE alignment should remove one global SE(3)");
    require(rgbd::relative_pose_error(estimated, reference, 1).translation_rmse_m < 1e-12, "RPE should be invariant to a global left transform");
}

}  // namespace

int main() {
    try {
        test_kabsch();
        test_backprojection();
        test_voxel();
        test_icp();
        test_pose_direction_and_metrics();
        std::cout << "5 C++ core tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test failure: " << error.what() << '\n';
        return 1;
    }
}
