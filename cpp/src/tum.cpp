#include "rgbd/tum.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <tuple>
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

std::vector<std::pair<std::size_t, std::size_t>> associate_one_to_one(
    const std::vector<double>& first_times,
    const std::vector<double>& second_times,
    double max_offset_s) {
    using Candidate = std::tuple<double, std::size_t, std::size_t>;
    std::vector<Candidate> candidates;
    for (std::size_t first = 0; first < first_times.size(); ++first) {
        const auto left = std::lower_bound(second_times.begin(), second_times.end(), first_times[first] - max_offset_s);
        const auto right = std::upper_bound(second_times.begin(), second_times.end(), first_times[first] + max_offset_s);
        for (auto position = left; position != right; ++position) {
            const std::size_t second = static_cast<std::size_t>(position - second_times.begin());
            candidates.emplace_back(std::abs(first_times[first] - *position), first, second);
        }
    }
    std::sort(candidates.begin(), candidates.end());
    std::vector<bool> used_first(first_times.size(), false);
    std::vector<bool> used_second(second_times.size(), false);
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    for (const auto& [offset, first, second] : candidates) {
        static_cast<void>(offset);
        if (!used_first[first] && !used_second[second]) {
            used_first[first] = true;
            used_second[second] = true;
            matches.emplace_back(first, second);
        }
    }
    std::sort(matches.begin(), matches.end());
    return matches;
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
    std::vector<double> rgb_times;
    std::vector<double> depth_times;
    std::vector<double> ground_truth_times;
    for (const auto& row : rgb) rgb_times.push_back(row.first);
    for (const auto& row : depth) depth_times.push_back(row.first);
    for (const auto& row : ground_truth) ground_truth_times.push_back(row.first);
    const auto rgb_depth_matches = associate_one_to_one(rgb_times, depth_times, max_depth_time_offset_s);
    std::vector<double> matched_rgb_times;
    for (const auto& [rgb_index, depth_index] : rgb_depth_matches) {
        static_cast<void>(depth_index);
        matched_rgb_times.push_back(rgb[rgb_index].first);
    }
    const auto rgbd_gt_matches = associate_one_to_one(
        matched_rgb_times, ground_truth_times, max_ground_truth_time_offset_s);

    std::vector<RgbdFrame> frames;
    frames.reserve(rgbd_gt_matches.size());
    for (const auto& [rgb_depth_index, gt_index] : rgbd_gt_matches) {
        const auto [rgb_index, depth_index] = rgb_depth_matches[rgb_depth_index];
        const auto& [rgb_time, rgb_path] = rgb[rgb_index];
        const double depth_offset = std::abs(depth[depth_index].first - rgb_time);
        const double gt_offset = std::abs(ground_truth[gt_index].first - rgb_time);
        frames.push_back({
            rgb_time,
            rgb_path,
            depth[depth_index].second,
            depth_offset,
            ground_truth[gt_index].second,
            gt_offset,
        });
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
