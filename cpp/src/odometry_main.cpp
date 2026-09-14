#include "rgbd/geometry.hpp"
#include "rgbd/metrics.hpp"
#include "rgbd/rgbd.hpp"
#include "rgbd/tum.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    std::filesystem::path dataset;
    std::filesystem::path output = "artifacts/cpp_odometry";
    std::optional<std::size_t> max_frames;
    int frame_step = 1;
    int stride = 8;
    double voxel_m = 0.05;
    double min_depth_m = 0.2;
    double max_depth_m = 4.0;
    int max_iterations = 30;
    double max_correspondence_m = 0.12;
    double min_correspondence_ratio = 0.5;
    std::optional<double> max_acceptable_rmse_m;
    bool quiet = false;
};

struct LatencyStats {
    std::size_t samples = 0;
    double mean_ms = 0.0;
    double median_ms = 0.0;
    double p95_ms = 0.0;
    double fps_from_mean = 0.0;
};

struct StepRow {
    std::size_t pair_index = 0;
    std::size_t source_points = 0;
    std::size_t target_points = 0;
    std::string status;
    int iterations = 0;
    std::size_t correspondences = 0;
    double rmse_m = std::numeric_limits<double>::quiet_NaN();
    double registration_s = 0.0;
    double input_preprocessing_s = 0.0;
    double end_to_end_s = 0.0;
    std::size_t peak_rss_bytes = 0;
};

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

std::size_t peak_rss_bytes() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS counters{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) == 0) {
        return 0;
    }
    return static_cast<std::size_t>(counters.PeakWorkingSetSize);
#else
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0;
    }
#if defined(__APPLE__)
    return static_cast<std::size_t>(usage.ru_maxrss);
#else
    return static_cast<std::size_t>(usage.ru_maxrss) * 1024U;
#endif
#endif
}

double percentile(std::vector<double> values, double probability) {
    std::sort(values.begin(), values.end());
    const double position = probability * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

LatencyStats latency_stats_seconds(const std::vector<double>& seconds) {
    if (seconds.empty()) {
        return {};
    }
    const double mean_s = std::accumulate(seconds.begin(), seconds.end(), 0.0) / static_cast<double>(seconds.size());
    return {
        seconds.size(),
        mean_s * 1000.0,
        percentile(seconds, 0.5) * 1000.0,
        percentile(seconds, 0.95) * 1000.0,
        1.0 / mean_s,
    };
}

void print_help() {
    std::cout
        << "C++17 TUM RGB-D point-to-point ICP odometry\n"
        << "Required: --dataset PATH\n"
        << "Options: --output PATH --max-frames N --frame-step N --stride N --voxel M\n"
        << "         --min-depth M --max-depth M --max-iterations N --max-correspondence M\n"
        << "         --min-correspondence-ratio R --max-acceptable-rmse M --quiet\n";
}

Options parse_options(int argc, char** argv) {
    Options options;
    auto require_value = [&](int& index, const std::string& flag) -> std::string {
        if (++index >= argc) {
            throw std::invalid_argument("missing value for " + flag);
        }
        return argv[index];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--help" || flag == "-h") {
            print_help();
            std::exit(0);
        } else if (flag == "--dataset") {
            options.dataset = require_value(i, flag);
        } else if (flag == "--output") {
            options.output = require_value(i, flag);
        } else if (flag == "--max-frames") {
            options.max_frames = static_cast<std::size_t>(std::stoull(require_value(i, flag)));
        } else if (flag == "--frame-step") {
            options.frame_step = std::stoi(require_value(i, flag));
        } else if (flag == "--stride") {
            options.stride = std::stoi(require_value(i, flag));
        } else if (flag == "--voxel") {
            options.voxel_m = std::stod(require_value(i, flag));
        } else if (flag == "--min-depth") {
            options.min_depth_m = std::stod(require_value(i, flag));
        } else if (flag == "--max-depth") {
            options.max_depth_m = std::stod(require_value(i, flag));
        } else if (flag == "--max-iterations") {
            options.max_iterations = std::stoi(require_value(i, flag));
        } else if (flag == "--max-correspondence") {
            options.max_correspondence_m = std::stod(require_value(i, flag));
        } else if (flag == "--min-correspondence-ratio") {
            options.min_correspondence_ratio = std::stod(require_value(i, flag));
        } else if (flag == "--max-acceptable-rmse") {
            options.max_acceptable_rmse_m = std::stod(require_value(i, flag));
        } else if (flag == "--quiet") {
            options.quiet = true;
        } else {
            throw std::invalid_argument("unknown argument: " + flag);
        }
    }
    if (options.dataset.empty()) {
        throw std::invalid_argument("--dataset is required");
    }
    if (options.frame_step < 1 || options.stride < 1 || options.voxel_m <= 0.0 || options.max_iterations < 1 ||
        options.max_correspondence_m <= 0.0 || options.min_correspondence_ratio <= 0.0 || options.min_correspondence_ratio > 1.0) {
        throw std::invalid_argument("invalid odometry parameter range");
    }
    if (!options.max_acceptable_rmse_m.has_value()) {
        options.max_acceptable_rmse_m = options.max_correspondence_m;
    }
    return options;
}

void write_trajectory(
    const std::filesystem::path& path,
    const std::vector<rgbd::RgbdFrame>& frames,
    const std::vector<rgbd::Mat4>& poses) {
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("cannot write trajectory: " + path.string());
    }
    for (std::size_t i = 0; i < poses.size(); ++i) {
        stream << rgbd::pose_to_tum_row(frames[i].timestamp, poses[i]) << '\n';
    }
}

void write_steps(const std::filesystem::path& path, const std::vector<StepRow>& rows) {
    std::ofstream stream(path);
    stream << "pair_index,source_points,target_points,status,iterations,correspondences,inlier_rmse_m,registration_runtime_s,input_preprocessing_runtime_s,end_to_end_runtime_s,process_peak_rss_bytes\n";
    stream << std::setprecision(12);
    for (const StepRow& row : rows) {
        stream << row.pair_index << ',' << row.source_points << ',' << row.target_points << ','
               << std::quoted(row.status) << ',' << row.iterations << ',' << row.correspondences << ',';
        if (std::isfinite(row.rmse_m)) {
            stream << row.rmse_m;
        }
        stream << ',' << row.registration_s << ',' << row.input_preprocessing_s << ',' << row.end_to_end_s << ',' << row.peak_rss_bytes << '\n';
    }
}

std::string utc_now() {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm value{};
#ifdef _WIN32
    gmtime_s(&value, &now);
#else
    gmtime_r(&now, &value);
#endif
    std::ostringstream output;
    output << std::put_time(&value, "%Y-%m-%dT%H:%M:%SZ");
    return output.str();
}

void write_latency_json(std::ostream& stream, const LatencyStats& stats) {
    stream << "{\"samples\":" << stats.samples
           << ",\"mean_ms\":" << stats.mean_ms
           << ",\"median_ms\":" << stats.median_ms
           << ",\"p95_ms\":" << stats.p95_ms
           << ",\"fps_from_mean\":" << stats.fps_from_mean << '}';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        const auto total_start = Clock::now();
        std::vector<rgbd::RgbdFrame> all_frames = rgbd::load_tum_rgbd_frames(options.dataset);
        std::vector<rgbd::RgbdFrame> frames;
        for (std::size_t i = 0; i < all_frames.size(); i += static_cast<std::size_t>(options.frame_step)) {
            frames.push_back(all_frames[i]);
            if (options.max_frames.has_value() && frames.size() >= *options.max_frames) {
                break;
            }
        }
        if (frames.size() < 3) {
            throw std::runtime_error("at least three associated frames are required");
        }
        std::filesystem::create_directories(options.output);

        std::vector<rgbd::PointCloud> clouds;
        std::vector<double> input_preprocessing_seconds;
        clouds.reserve(frames.size());
        input_preprocessing_seconds.reserve(frames.size());
        std::size_t minimum_points = std::numeric_limits<std::size_t>::max();
        std::size_t maximum_points = 0;
        std::size_t total_points = 0;
        for (const rgbd::RgbdFrame& frame : frames) {
            const auto start = Clock::now();
            rgbd::PointCloud cloud = rgbd::load_depth_point_cloud(
                frame.rgb_path, frame.depth_path, {}, options.stride, options.min_depth_m, options.max_depth_m);
            cloud = rgbd::voxel_downsample(cloud, options.voxel_m);
            input_preprocessing_seconds.push_back(seconds_since(start));
            minimum_points = std::min(minimum_points, cloud.size());
            maximum_points = std::max(maximum_points, cloud.size());
            total_points += cloud.size();
            clouds.push_back(std::move(cloud));
        }

        std::vector<rgbd::Mat4> poses{rgbd::Mat4::Identity()};
        std::vector<StepRow> steps;
        steps.reserve(frames.size() - 1);
        std::size_t accepted_pairs = 0;
        for (std::size_t index = 1; index < clouds.size(); ++index) {
            StepRow row;
            row.pair_index = index;
            row.source_points = clouds[index].size();
            row.target_points = clouds[index - 1].size();
            row.input_preprocessing_s = input_preprocessing_seconds[index];
            const auto registration_start = Clock::now();
            try {
                const rgbd::IcpResult result = rgbd::icp_point_to_point(
                    clouds[index], clouds[index - 1], options.max_iterations, 1e-8, options.max_correspondence_m);
                row.iterations = result.iterations;
                row.correspondences = result.correspondences;
                row.rmse_m = result.rmse_m;
                const double ratio = static_cast<double>(result.correspondences) / static_cast<double>(clouds[index].size());
                if (ratio < options.min_correspondence_ratio) {
                    std::ostringstream reason;
                    reason << "rejected: correspondence_ratio " << std::fixed << std::setprecision(3) << ratio
                           << " < " << options.min_correspondence_ratio;
                    row.status = reason.str();
                    poses.push_back(poses.back());
                } else if (result.rmse_m > *options.max_acceptable_rmse_m) {
                    std::ostringstream reason;
                    reason << "rejected: residual_rmse_m " << std::fixed << std::setprecision(4) << result.rmse_m
                           << " > " << *options.max_acceptable_rmse_m;
                    row.status = reason.str();
                    poses.push_back(poses.back());
                } else {
                    row.status = "ok";
                    const rgbd::Mat4 current_to_previous = rgbd::make_pose(result.transform.rotation, result.transform.translation);
                    poses.push_back(rgbd::compose_camera_to_world(poses.back(), current_to_previous));
                    ++accepted_pairs;
                }
            } catch (const std::exception& error) {
                row.status = std::string("failed: ") + error.what();
                poses.push_back(poses.back());
            }
            row.registration_s = seconds_since(registration_start);
            row.end_to_end_s = row.registration_s + row.input_preprocessing_s;
            row.peak_rss_bytes = peak_rss_bytes();
            steps.push_back(std::move(row));
        }

        std::vector<rgbd::Mat4> ground_truth;
        ground_truth.reserve(frames.size());
        for (const rgbd::RgbdFrame& frame : frames) {
            ground_truth.push_back(frame.ground_truth);
        }
        rgbd::RigidTransform alignment;
        const std::vector<rgbd::Mat4> aligned = rgbd::align_estimated_poses(poses, ground_truth, &alignment);
        const rgbd::AteMetrics ate = rgbd::absolute_trajectory_error(aligned, ground_truth);
        const rgbd::RpeMetrics rpe = rgbd::relative_pose_error(poses, ground_truth, 1);

        write_trajectory(options.output / "trajectory_cpp_local.txt", frames, poses);
        write_trajectory(options.output / "trajectory_cpp_ate_aligned.txt", frames, aligned);
        write_steps(options.output / "icp_steps.csv", steps);

        std::vector<double> registration_seconds;
        std::vector<double> end_to_end_seconds;
        for (const StepRow& row : steps) {
            if (row.status == "ok") {
                registration_seconds.push_back(row.registration_s);
                end_to_end_seconds.push_back(row.end_to_end_s);
            }
        }
        const LatencyStats input_stats = latency_stats_seconds(input_preprocessing_seconds);
        const LatencyStats registration_stats = latency_stats_seconds(registration_seconds);
        const LatencyStats end_to_end_stats = latency_stats_seconds(end_to_end_seconds);
        const std::size_t rss = peak_rss_bytes();
        const double total_runtime_s = seconds_since(total_start);

        std::ofstream summary(options.output / "summary.json");
        summary << std::setprecision(12);
        summary << "{\n"
                << "  \"experiment\": \"C++17 TUM RGB-D frame-to-frame point-to-point ICP odometry\",\n"
                << "  \"created_utc\": " << std::quoted(utc_now()) << ",\n"
                << "  \"dataset\": " << std::quoted(options.dataset.string()) << ",\n"
                << "  \"association_protocol\": \"one_to_one_minimum_offset_greedy_v2\",\n"
                << "  \"quality_protocol\": \"shared_nearest_neighbor_v2: final-transform source-to-target; gated correspondence ratio and inlier RMSE; all-source-point RMSE for residual rejection\",\n"
                << "  \"parameters\": {\"stride\":" << options.stride << ",\"voxel_m\":" << options.voxel_m
                << ",\"min_depth_m\":" << options.min_depth_m << ",\"max_depth_m\":" << options.max_depth_m
                << ",\"max_iterations\":" << options.max_iterations << ",\"max_correspondence_m\":" << options.max_correspondence_m
                << ",\"min_correspondence_ratio\":" << options.min_correspondence_ratio
                << ",\"max_acceptable_rmse_m\":" << *options.max_acceptable_rmse_m << "},\n"
                << "  \"frames\": {\"count\":" << frames.size() << ",\"associated_total\":" << all_frames.size()
                << ",\"point_count_mean\":" << static_cast<double>(total_points) / static_cast<double>(frames.size())
                << ",\"point_count_min\":" << minimum_points << ",\"point_count_max\":" << maximum_points << "},\n"
                << "  \"quality\": {\"accepted_pairs\":" << accepted_pairs
                << ",\"rejected_or_exception_pairs\":" << steps.size() - accepted_pairs << "},\n"
                << "  \"ate\": {\"alignment\":\"SE(3)\",\"rmse_m\":" << ate.rmse_m << ",\"mean_m\":" << ate.mean_m
                << ",\"median_m\":" << ate.median_m << ",\"max_m\":" << ate.max_m << "},\n"
                << "  \"rpe_frame_delta_1\": {\"pairs\":" << rpe.pairs << ",\"translation_rmse_m\":" << rpe.translation_rmse_m
                << ",\"translation_mean_m\":" << rpe.translation_mean_m << ",\"rotation_rmse_deg\":" << rpe.rotation_rmse_deg
                << ",\"rotation_mean_deg\":" << rpe.rotation_mean_deg << "},\n"
                << "  \"performance\": {\"input_and_preprocessing_latency\":";
        write_latency_json(summary, input_stats);
        summary << ",\"registration_latency\":";
        write_latency_json(summary, registration_stats);
        summary << ",\"end_to_end_latency_excluding_first_frame\":";
        write_latency_json(summary, end_to_end_stats);
        summary << ",\"process_peak_rss_bytes\":" << rss << ",\"total_runtime_s\":" << total_runtime_s << "},\n"
                << "  \"environment\": {\"language\":\"C++17\",\"compiler\":" << std::quoted(__VERSION__)
                << ",\"opencv\":" << std::quoted(RGBD_OPENCV_VERSION) << ",\"eigen\":\"header-only\",\"gpu\":\"not used\"},\n"
                << "  \"notes\": [\"Ground truth is evaluation-only.\",\"ICP maps current-camera points into the previous camera frame.\",\"End-to-end latency includes OpenCV RGB/depth reads, back-projection, voxel downsampling, and registration.\",\"ICP correspondence count uses the distance gate; residual RMSE uses all source points, matching the Python baseline.\"]\n"
                << "}\n";
        summary.close();

        if (!options.quiet) {
            std::cout << "frames=" << frames.size() << " accepted=" << accepted_pairs << " rejected=" << steps.size() - accepted_pairs
                      << " ATE_RMSE_m=" << ate.rmse_m << " RPE_t_m=" << rpe.translation_rmse_m
                      << " RPE_r_deg=" << rpe.rotation_rmse_deg << " mean_ms=" << end_to_end_stats.mean_ms
                      << " p95_ms=" << end_to_end_stats.p95_ms << " RSS_bytes=" << rss << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
