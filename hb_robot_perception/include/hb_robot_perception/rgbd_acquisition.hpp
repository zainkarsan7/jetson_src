
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>

#include "hb_robot_perception/perception_types.hpp"

namespace hb_perception {

class RGBDAcquisition {
public:
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            Image, Image>;

    enum class Mode {
        Continuous,
        CaptureWindow
    };

    RGBDAcquisition(
        rclcpp::Node* node,
        tf2_ros::Buffer* tf_buffer,
        const std::string& target_frame,
        const std::string& rgb_topic =
            "/k4a/rgb/image_raw",
        const std::string& depth_topic =
            "/k4a/depth_to_rgb/image_raw",
        const std::string& camera_info_topic =
            "/k4a/depth_to_rgb/camera_info");

    RGBDAcquisition(
        rclcpp::Node* node,
        tf2_ros::Buffer* tf_buffer,
        const std::string& target_frame,
        Mode mode,
        const std::string& rgb_topic =
            "/k4a/rgb/image_raw",
        const std::string& depth_topic =
            "/k4a/depth_to_rgb/image_raw",
        const std::string& camera_info_topic =
            "/k4a/depth_to_rgb/camera_info");

    ~RGBDAcquisition();

    RGBDAcquisition(const RGBDAcquisition&) = delete;
    RGBDAcquisition& operator=(const RGBDAcquisition&) = delete;

    std::optional<Observation> acquireAfter(
        const rclcpp::Time& min_stamp,
        std::chrono::milliseconds timeout);

    std::optional<Observation> latest() const;

    void logCaptureFailure(const rclcpp::Time& boundary);

private:
    struct PendingPair {
        Image::ConstSharedPtr rgb;
        Image::ConstSharedPtr depth;
        std::chrono::steady_clock::time_point arrival;
    };

    

    void synchronizedCallback(
        const Image::ConstSharedPtr& rgb,
        const Image::ConstSharedPtr& depth);

    void resolvePending();
    void logDiagnostics();

    rclcpp::Node* node_;
    tf2_ros::Buffer* tf_buffer_;
    std::string target_frame_;
    Mode mode_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    std::unique_ptr<
        rclcpp::executors::SingleThreadedExecutor>
        window_executor_;

    std::thread window_thread_;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;

    std::shared_ptr<
        message_filters::Synchronizer<SyncPolicy>>
        synchronizer_;

    rclcpp::Subscription<CameraInfo>::SharedPtr
        camera_info_sub_;

    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // Accessed by the acquisition callback group.
    CameraInfo::ConstSharedPtr camera_info_;
    std::optional<PendingPair> pending_;

    std::size_t rgb_received_{0};
    std::size_t depth_received_{0};
    std::size_t synchronized_count_{0};
    std::size_t resolved_count_{0};
    std::size_t tf_failure_count_{0};
    std::size_t expired_count_{0};

    mutable std::mutex observation_mutex_;
    std::condition_variable observation_cv_;

    std::optional<Observation> latest_observation_;
};

}  // namespace hb_perception
