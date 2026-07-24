#include "rgbd/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace rgbd {
namespace {

double mean(const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double rmse(const std::vector<double>& values) {
    double sum = 0.0;
    for (const double value : values) {
        sum += value * value;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return 0.5 * (values[middle - 1] + values[middle]);
    }
    return values[middle];
}

double rotation_angle_degrees(const Mat3& rotation) {
    const double cosine = std::clamp((rotation.trace() - 1.0) / 2.0, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / std::acos(-1.0);
}

void validate_trajectories(const std::vector<Mat4>& estimated, const std::vector<Mat4>& reference, std::size_t minimum) {
    if (estimated.size() != reference.size() || estimated.size() < minimum) {
        throw std::invalid_argument("trajectory lengths must match and contain enough poses");
    }
}

}  // namespace

Mat4 invert_pose(const Mat4& pose) {
    Mat4 inverse = Mat4::Identity();
    const Mat3 rotation = pose.block<3, 3>(0, 0);
    const Vec3 translation = pose.block<3, 1>(0, 3);
    inverse.block<3, 3>(0, 0) = rotation.transpose();
    inverse.block<3, 1>(0, 3) = -rotation.transpose() * translation;
    return inverse;
}

Mat4 compose_camera_to_world(const Mat4& previous_camera_to_world, const Mat4& current_to_previous) {
    return previous_camera_to_world * current_to_previous;
}

std::vector<Mat4> align_estimated_poses(
    const std::vector<Mat4>& estimated,
    const std::vector<Mat4>& reference,
    RigidTransform* alignment) {
    validate_trajectories(estimated, reference, 3);
    PointCloud estimated_positions;
    PointCloud reference_positions;
    estimated_positions.reserve(estimated.size());
    reference_positions.reserve(reference.size());
    for (std::size_t i = 0; i < estimated.size(); ++i) {
        estimated_positions.push_back(estimated[i].block<3, 1>(0, 3));
        reference_positions.push_back(reference[i].block<3, 1>(0, 3));
    }
    const RigidTransform transform = kabsch(estimated_positions, reference_positions);
    if (alignment != nullptr) {
        *alignment = transform;
    }
    const Mat4 alignment_pose = make_pose(transform.rotation, transform.translation);
    std::vector<Mat4> aligned;
    aligned.reserve(estimated.size());
    for (const Mat4& pose : estimated) {
        aligned.push_back(alignment_pose * pose);
    }
    return aligned;
}

AteMetrics absolute_trajectory_error(const std::vector<Mat4>& aligned, const std::vector<Mat4>& reference) {
    validate_trajectories(aligned, reference, 1);
    std::vector<double> errors;
    errors.reserve(aligned.size());
    for (std::size_t i = 0; i < aligned.size(); ++i) {
        errors.push_back((aligned[i].block<3, 1>(0, 3) - reference[i].block<3, 1>(0, 3)).norm());
    }
    return {rmse(errors), mean(errors), median(errors), *std::max_element(errors.begin(), errors.end())};
}

RpeMetrics relative_pose_error(const std::vector<Mat4>& estimated, const std::vector<Mat4>& reference, int frame_delta) {
    validate_trajectories(estimated, reference, 2);
    if (frame_delta < 1 || static_cast<std::size_t>(frame_delta) >= estimated.size()) {
        throw std::invalid_argument("frame_delta must be positive and shorter than trajectory");
    }
    std::vector<double> translation_errors;
    std::vector<double> rotation_errors;
    const std::size_t delta = static_cast<std::size_t>(frame_delta);
    translation_errors.reserve(estimated.size() - delta);
    rotation_errors.reserve(estimated.size() - delta);
    for (std::size_t i = 0; i + delta < estimated.size(); ++i) {
        const Mat4 estimated_relative = invert_pose(estimated[i]) * estimated[i + delta];
        const Mat4 truth_relative = invert_pose(reference[i]) * reference[i + delta];
        const Mat4 error = invert_pose(truth_relative) * estimated_relative;
        translation_errors.push_back(error.block<3, 1>(0, 3).norm());
        rotation_errors.push_back(rotation_angle_degrees(error.block<3, 3>(0, 0)));
    }
    return {
        frame_delta,
        translation_errors.size(),
        rmse(translation_errors),
        mean(translation_errors),
        rmse(rotation_errors),
        mean(rotation_errors),
    };
}

}  // namespace rgbd
