#include "rgbd/geometry.hpp"
#include "rgbd/metrics.hpp"
#include "rgbd/rgbd.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace rgbd_odometry_ros {

class RgbdOdometryNode final : public rclcpp::Node {
public:
    using Image = sensor_msgs::msg::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

    RgbdOdometryNode() : Node("rgbd_odometry") {
        rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
        stride_ = declare_parameter<int>("stride", 8);
        voxel_m_ = declare_parameter<double>("voxel_m", 0.05);
        min_depth_m_ = declare_parameter<double>("min_depth_m", 0.2);
        max_depth_m_ = declare_parameter<double>("max_depth_m", 4.0);
        max_iterations_ = declare_parameter<int>("max_iterations", 30);
        max_correspondence_m_ = declare_parameter<double>("max_correspondence_m", 0.12);
        min_correspondence_ratio_ = declare_parameter<double>("min_correspondence_ratio", 0.5);
        max_acceptable_rmse_m_ = declare_parameter<double>("max_acceptable_rmse_m", 0.12);
        sync_slop_s_ = declare_parameter<double>("sync_slop_s", 0.02);
        metrics_csv_ = declare_parameter<std::string>("metrics_csv", "ros2_metrics.csv");
        trajectory_tum_ = declare_parameter<std::string>("trajectory_tum", "ros2_trajectory.txt");
        if (stride_ < 1 || voxel_m_ <= 0.0 || min_depth_m_ < 0.0 || max_depth_m_ <= min_depth_m_ ||
            max_iterations_ < 1 || max_correspondence_m_ <= 0.0 || min_correspondence_ratio_ <= 0.0 ||
            min_correspondence_ratio_ > 1.0 || sync_slop_s_ <= 0.0) {
            throw std::invalid_argument("invalid ROS 2 odometry parameters");
        }

        const auto metrics_path = std::filesystem::path(metrics_csv_);
        const auto trajectory_path = std::filesystem::path(trajectory_tum_);
        if (metrics_path.has_parent_path()) {
            std::filesystem::create_directories(metrics_path.parent_path());
        }
        if (trajectory_path.has_parent_path()) {
            std::filesystem::create_directories(trajectory_path.parent_path());
        }
        metrics_.open(metrics_path, std::ios::out | std::ios::trunc);
        if (!metrics_) {
            throw std::runtime_error("cannot open metrics CSV: " + metrics_csv_);
        }
        trajectory_.open(trajectory_path, std::ios::out | std::ios::trunc);
        if (!trajectory_) {
            throw std::runtime_error("cannot open TUM trajectory: " + trajectory_tum_);
        }
        metrics_ << "processed_frame,rgb_stamp_s,depth_stamp_s,rgb_depth_offset_s,rgb_received,depth_received,"
                    "unsynchronized_or_pending,status,point_count,correspondences,rmse_m,callback_latency_ms,process_peak_rss_bytes\n";

        const auto sensor_qos = rclcpp::SensorDataQoS();
        rgb_sub_.subscribe(this, rgb_topic_, sensor_qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, depth_topic_, sensor_qos.get_rmw_qos_profile());
        rgb_sub_.registerCallback([this](const Image::ConstSharedPtr&) { ++rgb_received_; });
        depth_sub_.registerCallback([this](const Image::ConstSharedPtr&) { ++depth_received_; });
        synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), rgb_sub_, depth_sub_);
        synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
        synchronizer_->registerCallback(
            std::bind(&RgbdOdometryNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, sensor_qos,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) { camera_info_ = std::move(message); });
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", sensor_qos);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        path_.header.frame_id = odom_frame_;

        RCLCPP_INFO(get_logger(), "Subscribing RGB=%s depth=%s camera_info=%s", rgb_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Metrics CSV: %s", metrics_csv_.c_str());
    }

    ~RgbdOdometryNode() override {
        RCLCPP_INFO(
            get_logger(), "processed=%zu rgb_received=%zu depth_received=%zu pending_or_dropped=%zu",
            processed_, rgb_received_.load(), depth_received_.load(), pending_or_dropped());
    }

private:
    static double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    std::size_t pending_or_dropped() const {
        return std::min(rgb_received_.load(), depth_received_.load()) > processed_
            ? std::min(rgb_received_.load(), depth_received_.load()) - processed_
            : 0U;
    }

    static std::size_t process_peak_rss_bytes() {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS counters{};
        counters.cb = sizeof(counters);
        if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) == 0) {
            return 0U;
        }
        return static_cast<std::size_t>(counters.PeakWorkingSetSize);
#else
        rusage usage{};
        if (getrusage(RUSAGE_SELF, &usage) != 0) {
            return 0U;
        }
#if defined(__APPLE__)
        return static_cast<std::size_t>(usage.ru_maxrss);
#else
        return static_cast<std::size_t>(usage.ru_maxrss) * 1024U;
#endif
#endif
    }

    rgbd::CameraIntrinsics intrinsics_from_message(const sensor_msgs::msg::CameraInfo& info) const {
        rgbd::CameraIntrinsics intrinsics;
        intrinsics.fx = info.k[0];
        intrinsics.fy = info.k[4];
        intrinsics.cx = info.k[2];
        intrinsics.cy = info.k[5];
        intrinsics.depth_scale = 5000.0;
        if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
            throw std::runtime_error("CameraInfo contains invalid focal lengths");
        }
        return intrinsics;
    }

    void rgbd_callback(const Image::ConstSharedPtr& rgb_message, const Image::ConstSharedPtr& depth_message) {
        const auto callback_start = std::chrono::steady_clock::now();
        std::string status = "ok";
        std::size_t point_count = 0;
        std::size_t correspondences = 0;
        double rmse_m = std::numeric_limits<double>::quiet_NaN();
        try {
            if (!camera_info_) {
                throw std::runtime_error("waiting for CameraInfo");
            }
            static_cast<void>(cv_bridge::toCvShare(rgb_message, sensor_msgs::image_encodings::BGR8));
            const auto depth = cv_bridge::toCvShare(depth_message, sensor_msgs::image_encodings::TYPE_16UC1);
            const rgbd::CameraIntrinsics intrinsics = intrinsics_from_message(*camera_info_);
            rgbd::PointCloud cloud = rgbd::depth_to_points_u16(
                depth->image.ptr<unsigned short>(), depth->image.cols, depth->image.rows,
                static_cast<int>(depth->image.step1()), intrinsics, stride_, min_depth_m_, max_depth_m_);
            cloud = rgbd::voxel_downsample(cloud, voxel_m_);
            point_count = cloud.size();

            if (previous_cloud_) {
                const rgbd::IcpResult result = rgbd::icp_point_to_point(
                    cloud, *previous_cloud_, max_iterations_, 1e-8, max_correspondence_m_);
                correspondences = result.correspondences;
                rmse_m = result.rmse_m;
                const double ratio = static_cast<double>(result.correspondences) / static_cast<double>(cloud.size());
                if (ratio < min_correspondence_ratio_) {
                    std::ostringstream reason;
                    reason << "rejected_low_ratio_" << std::fixed << std::setprecision(3) << ratio;
                    status = reason.str();
                } else if (result.rmse_m > max_acceptable_rmse_m_) {
                    std::ostringstream reason;
                    reason << "rejected_high_rmse_" << std::fixed << std::setprecision(4) << result.rmse_m;
                    status = reason.str();
                } else {
                    pose_ = rgbd::compose_camera_to_world(
                        pose_, rgbd::make_pose(result.transform.rotation, result.transform.translation));
                }
            } else {
                status = "initialized";
            }
            previous_cloud_ = cloud;
            publish_outputs(rgb_message->header.stamp, cloud);
            write_trajectory(rgb_message->header.stamp);
        } catch (const std::exception& error) {
            status = std::string("failed_") + error.what();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        }

        ++processed_;
        const double latency_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - callback_start).count();
        metrics_ << std::setprecision(12) << processed_ << ',' << stamp_seconds(rgb_message->header.stamp) << ','
                 << stamp_seconds(depth_message->header.stamp) << ','
                 << std::abs(stamp_seconds(rgb_message->header.stamp) - stamp_seconds(depth_message->header.stamp)) << ','
                 << rgb_received_.load() << ',' << depth_received_.load() << ',' << pending_or_dropped() << ','
                 << std::quoted(status) << ',' << point_count << ',' << correspondences << ',';
        if (std::isfinite(rmse_m)) {
            metrics_ << rmse_m;
        }
        metrics_ << ',' << latency_ms << ',' << process_peak_rss_bytes() << '\n';
        metrics_.flush();
        if (processed_ % 30 == 0) {
            RCLCPP_INFO(get_logger(), "processed=%zu latency=%.2f ms pending_or_dropped=%zu", processed_, latency_ms, pending_or_dropped());
        }
    }

    void publish_outputs(const builtin_interfaces::msg::Time& stamp, const rgbd::PointCloud& camera_cloud) {
        const Eigen::Quaterniond quaternion(pose_.block<3, 3>(0, 0));
        const rgbd::Vec3 translation = pose_.block<3, 1>(0, 3);

        geometry_msgs::msg::Pose pose_message;
        pose_message.position.x = translation.x();
        pose_message.position.y = translation.y();
        pose_message.position.z = translation.z();
        pose_message.orientation.x = quaternion.x();
        pose_message.orientation.y = quaternion.y();
        pose_message.orientation.z = quaternion.z();
        pose_message.orientation.w = quaternion.w();

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = odom_frame_;
        odometry.child_frame_id = camera_frame_;
        odometry.pose.pose = pose_message;
        odometry_pub_->publish(odometry);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odometry.header;
        pose_stamped.pose = pose_message;
        path_.header.stamp = stamp;
        path_.poses.push_back(pose_stamped);
        path_pub_->publish(path_);

        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = camera_frame_;
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        transform.transform.rotation = pose_message.orientation;
        tf_broadcaster_->sendTransform(transform);

        sensor_msgs::msg::PointCloud2 cloud_message;
        cloud_message.header = odometry.header;
        cloud_message.height = 1;
        sensor_msgs::PointCloud2Modifier modifier(cloud_message);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(camera_cloud.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud_message, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud_message, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud_message, "z");
        const rgbd::Mat3 rotation = pose_.block<3, 3>(0, 0);
        for (const rgbd::Vec3& camera_point : camera_cloud) {
            const rgbd::Vec3 world_point = rotation * camera_point + translation;
            *x = static_cast<float>(world_point.x());
            *y = static_cast<float>(world_point.y());
            *z = static_cast<float>(world_point.z());
            ++x;
            ++y;
            ++z;
        }
        cloud_pub_->publish(cloud_message);
    }

    void write_trajectory(const builtin_interfaces::msg::Time& stamp) {
        const Eigen::Quaterniond quaternion(pose_.block<3, 3>(0, 0));
        const rgbd::Vec3 translation = pose_.block<3, 1>(0, 3);
        trajectory_ << std::fixed << std::setprecision(9)
                    << stamp_seconds(stamp) << ' '
                    << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' '
                    << quaternion.x() << ' ' << quaternion.y() << ' ' << quaternion.z() << ' '
                    << quaternion.w() << '\n';
        trajectory_.flush();
    }

    std::string rgb_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string odom_frame_;
    std::string camera_frame_;
    std::string metrics_csv_;
    std::string trajectory_tum_;
    int stride_ = 8;
    double voxel_m_ = 0.05;
    double min_depth_m_ = 0.2;
    double max_depth_m_ = 4.0;
    int max_iterations_ = 30;
    double max_correspondence_m_ = 0.12;
    double min_correspondence_ratio_ = 0.5;
    double max_acceptable_rmse_m_ = 0.12;
    double sync_slop_s_ = 0.02;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path path_;
    std::optional<rgbd::PointCloud> previous_cloud_;
    rgbd::Mat4 pose_ = rgbd::Mat4::Identity();
    std::atomic<std::size_t> rgb_received_{0};
    std::atomic<std::size_t> depth_received_{0};
    std::size_t processed_ = 0;
    std::ofstream metrics_;
    std::ofstream trajectory_;
};

}  // namespace rgbd_odometry_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<rgbd_odometry_ros::RgbdOdometryNode>());
    } catch (const std::exception& error) {
        std::cerr << "rgbd_odometry_node error: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
