#include "rgbd/geometry.hpp"

#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace rgbd {
namespace {

class KdTree3d {
public:
    explicit KdTree3d(const PointCloud& points) : points_(points), indices_(points.size()) {
        std::iota(indices_.begin(), indices_.end(), 0U);
        nodes_.reserve(points.size());
        root_ = build(0, indices_.size(), 0);
    }

    std::pair<std::size_t, double> nearest(const Vec3& query) const {
        std::size_t best_index = 0;
        double best_squared = std::numeric_limits<double>::infinity();
        search(root_, query, best_index, best_squared);
        return {best_index, best_squared};
    }

private:
    struct Node {
        std::size_t point_index = 0;
        int left = -1;
        int right = -1;
        int axis = 0;
    };

    int build(std::size_t begin, std::size_t end, int depth) {
        if (begin >= end) {
            return -1;
        }
        const int axis = depth % 3;
        const std::size_t middle = begin + (end - begin) / 2;
        std::nth_element(
            indices_.begin() + static_cast<std::ptrdiff_t>(begin),
            indices_.begin() + static_cast<std::ptrdiff_t>(middle),
            indices_.begin() + static_cast<std::ptrdiff_t>(end),
            [&](std::size_t lhs, std::size_t rhs) {
                if (points_[lhs][axis] == points_[rhs][axis]) {
                    return lhs < rhs;
                }
                return points_[lhs][axis] < points_[rhs][axis];
            });
        const int node_index = static_cast<int>(nodes_.size());
        nodes_.push_back({indices_[middle], -1, -1, axis});
        const int left = build(begin, middle, depth + 1);
        const int right = build(middle + 1, end, depth + 1);
        nodes_[static_cast<std::size_t>(node_index)].left = left;
        nodes_[static_cast<std::size_t>(node_index)].right = right;
        return node_index;
    }

    void search(int node_index, const Vec3& query, std::size_t& best_index, double& best_squared) const {
        if (node_index < 0) {
            return;
        }
        const Node& node = nodes_[static_cast<std::size_t>(node_index)];
        const Vec3& point = points_[node.point_index];
        const double squared = (query - point).squaredNorm();
        if (squared < best_squared || (squared == best_squared && node.point_index < best_index)) {
            best_squared = squared;
            best_index = node.point_index;
        }
        const double difference = query[node.axis] - point[node.axis];
        const int near_child = difference <= 0.0 ? node.left : node.right;
        const int far_child = difference <= 0.0 ? node.right : node.left;
        search(near_child, query, best_index, best_squared);
        if (difference * difference <= best_squared) {
            search(far_child, query, best_index, best_squared);
        }
    }

    const PointCloud& points_;
    std::vector<std::size_t> indices_;
    std::vector<Node> nodes_;
    int root_ = -1;
};

struct Matches {
    PointCloud source;
    PointCloud target;
    double all_point_rmse_m = 0.0;
};

Matches find_matches(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree3d& tree,
    double max_distance) {
    Matches matches;
    matches.source.reserve(source.size());
    matches.target.reserve(source.size());
    const double max_squared = max_distance * max_distance;
    double all_squared_sum = 0.0;
    for (const Vec3& point : source) {
        const auto [index, squared] = tree.nearest(point);
        all_squared_sum += squared;
        if (squared <= max_squared) {
            matches.source.push_back(point);
            matches.target.push_back(target[index]);
        }
    }
    matches.all_point_rmse_m = std::sqrt(all_squared_sum / static_cast<double>(source.size()));
    return matches;
}

}  // namespace

PointCloud transform_points(const PointCloud& points, const RigidTransform& transform) {
    PointCloud transformed;
    transformed.reserve(points.size());
    for (const Vec3& point : points) {
        transformed.push_back(transform.rotation * point + transform.translation);
    }
    return transformed;
}

RigidTransform kabsch(const PointCloud& source, const PointCloud& target) {
    if (source.size() != target.size() || source.size() < 3) {
        throw std::invalid_argument("Kabsch requires equal point clouds with at least three points");
    }
    Vec3 source_center = Vec3::Zero();
    Vec3 target_center = Vec3::Zero();
    for (std::size_t i = 0; i < source.size(); ++i) {
        source_center += source[i];
        target_center += target[i];
    }
    source_center /= static_cast<double>(source.size());
    target_center /= static_cast<double>(target.size());

    Mat3 covariance = Mat3::Zero();
    for (std::size_t i = 0; i < source.size(); ++i) {
        covariance += (source[i] - source_center) * (target[i] - target_center).transpose();
    }
    const Eigen::JacobiSVD<Mat3> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 correction = Mat3::Identity();
    if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
        correction(2, 2) = -1.0;
    }
    RigidTransform result;
    result.rotation = svd.matrixV() * correction * svd.matrixU().transpose();
    result.translation = target_center - result.rotation * source_center;
    return result;
}

PointCloud voxel_downsample(const PointCloud& points, double voxel_size_m) {
    if (points.size() < 3 || voxel_size_m <= 0.0) {
        throw std::invalid_argument("voxel_downsample requires at least three points and a positive voxel size");
    }
    struct Accumulator {
        Vec3 sum = Vec3::Zero();
        std::size_t count = 0;
    };
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, Accumulator> voxels;
    for (const Vec3& point : points) {
        const auto key = std::make_tuple(
            static_cast<std::int64_t>(std::floor(point.x() / voxel_size_m)),
            static_cast<std::int64_t>(std::floor(point.y() / voxel_size_m)),
            static_cast<std::int64_t>(std::floor(point.z() / voxel_size_m)));
        auto& accumulator = voxels[key];
        accumulator.sum += point;
        ++accumulator.count;
    }
    PointCloud output;
    output.reserve(voxels.size());
    for (const auto& [key, accumulator] : voxels) {
        static_cast<void>(key);
        output.push_back(accumulator.sum / static_cast<double>(accumulator.count));
    }
    return output;
}

IcpResult icp_point_to_point(
    const PointCloud& source,
    const PointCloud& target,
    int max_iterations,
    double tolerance,
    double max_correspondence_m) {
    if (source.size() < 3 || target.size() < 3 || max_iterations < 1 || tolerance < 0.0 || max_correspondence_m <= 0.0) {
        throw std::invalid_argument("invalid ICP inputs or parameters");
    }
    const KdTree3d tree(target);
    PointCloud aligned = source;
    RigidTransform total;
    double previous_rmse = std::numeric_limits<double>::infinity();
    IcpResult result;

    for (int iteration = 1; iteration <= max_iterations; ++iteration) {
        const Matches matches = find_matches(aligned, target, tree, max_correspondence_m);
        if (matches.source.size() < 3) {
            throw std::runtime_error("fewer than three valid ICP correspondences");
        }
        const RigidTransform delta = kabsch(matches.source, matches.target);
        aligned = transform_points(aligned, delta);
        total.translation = delta.rotation * total.translation + delta.translation;
        total.rotation = delta.rotation * total.rotation;

        const Matches updated = find_matches(aligned, target, tree, max_correspondence_m);
        result = {total, updated.all_point_rmse_m, iteration, matches.source.size()};
        if (std::abs(previous_rmse - updated.all_point_rmse_m) < tolerance) {
            return result;
        }
        previous_rmse = updated.all_point_rmse_m;
    }
    return result;
}

}  // namespace rgbd
