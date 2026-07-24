#include "rgbd/tum.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rgbd {
namespace {

using FileRow = std::pair<double, std::filesystem::path>;
using PoseRow = std::pair<double, Mat4>;

bool skip_line(const std::string& line) {
    const auto first = line.find_first_not_of(" \t\r\n");
    return first == std::string::npos || line[first] == '#';
}

std::vector<FileRow> read_index(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot open TUM index: " + path.string());
    }
    std::vector<FileRow> rows;
    std::string line;
    while (std::getline(stream, line)) {
        if (skip_line(line)) {
            continue;
        }
        std::istringstream values(line);
        double timestamp = 0.0;
        std::string filename;
        if (!(values >> timestamp >> filename)) {
            throw std::runtime_error("invalid TUM index row in " + path.string());
        }
        rows.emplace_back(timestamp, path.parent_path() / filename);
    }
    return rows;
}

std::vector<PoseRow> read_ground_truth(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot open TUM ground truth: " + path.string());
    }
    std::vector<PoseRow> rows;
    std::string line;
    while (std::getline(stream, line)) {
        if (skip_line(line)) {
            continue;
        }
        std::istringstream values(line);
        double timestamp = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 0.0;
        if (!(values >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            throw std::runtime_error("invalid TUM ground-truth row in " + path.string());
        }
        const Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        Mat4 pose = Mat4::Identity();
        pose.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
        pose.block<3, 1>(0, 3) = Vec3(tx, ty, tz);
        rows.emplace_back(timestamp, pose);
    }
    return rows;
}

template <typename Row>
std::size_t nearest_index(const std::vector<Row>& rows, double timestamp) {
    const auto position = std::lower_bound(
        rows.begin(), rows.end(), timestamp,
        [](const Row& row, double value) { return row.first < value; });
    if (position == rows.begin()) {
        return 0;
    }
    if (position == rows.end()) {
        return rows.size() - 1;
    }
    const std::size_t right = static_cast<std::size_t>(position - rows.begin());
    const std::size_t left = right - 1;
    return std::abs(rows[left].first - timestamp) <= std::abs(rows[right].first - timestamp) ? left : right;
}

}  // namespace

std::vector<RgbdFrame> load_tum_rgbd_frames(
    const std::filesystem::path& dataset_dir,
    double max_depth_time_offset_s,
    double max_ground_truth_time_offset_s) {
    const std::vector<FileRow> rgb = read_index(dataset_dir / "rgb.txt");
    const std::vector<FileRow> depth = read_index(dataset_dir / "depth.txt");
    const std::vector<PoseRow> ground_truth = read_ground_truth(dataset_dir / "groundtruth.txt");
    if (rgb.empty() || depth.empty() || ground_truth.empty()) {
        throw std::runtime_error("RGB, depth, and ground-truth indices must be non-empty");
    }
    std::vector<RgbdFrame> frames;
    frames.reserve(rgb.size());
    for (const auto& [rgb_time, rgb_path] : rgb) {
        const std::size_t depth_index = nearest_index(depth, rgb_time);
        const std::size_t gt_index = nearest_index(ground_truth, rgb_time);
        const double depth_offset = std::abs(depth[depth_index].first - rgb_time);
        const double gt_offset = std::abs(ground_truth[gt_index].first - rgb_time);
        if (depth_offset <= max_depth_time_offset_s && gt_offset <= max_ground_truth_time_offset_s) {
            frames.push_back({
                rgb_time,
                rgb_path,
                depth[depth_index].second,
                depth_offset,
                ground_truth[gt_index].second,
                gt_offset,
            });
        }
    }
    if (frames.size() < 2) {
        throw std::runtime_error("fewer than two associated TUM RGB-D frames");
    }
    return frames;
}

std::string pose_to_tum_row(double timestamp, const Mat4& pose) {
    const Eigen::Quaterniond quaternion(pose.block<3, 3>(0, 0));
    const Vec3 translation = pose.block<3, 1>(0, 3);
    std::ostringstream row;
    row << std::fixed << std::setprecision(9)
        << timestamp << ' ' << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' '
        << quaternion.x() << ' ' << quaternion.y() << ' ' << quaternion.z() << ' ' << quaternion.w();
    return row.str();
}

}  // namespace rgbd
