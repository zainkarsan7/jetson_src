
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
#include <tf2_ros/buffer.h>

#include <azure_kinect_ros2_driver_codex/msg/rgbd_frame.hpp>

#include "hb_robot_perception/perception_types.hpp"

namespace hb_perception {

class CroppedRGBDAcquisition {
public:
    using RGBDFrame =
        azure_kinect_ros2_driver_codex::msg::RGBDFrame;

    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    enum class Mode {
        Continuous,
        CaptureWindow
    };

    CroppedRGBDAcquisition(
        rclcpp::Node* node,
        tf2_ros::Buffer* tf_buffer,
        const std::string& target_frame,
        const std::string& rgbd_topic = "/k4a/rgbd");

    CroppedRGBDAcquisition(
        rclcpp::Node* node,
        tf2_ros::Buffer* tf_buffer,
        const std::string& target_frame,
        Mode mode,
        const std::string& rgbd_topic = "/k4a/rgbd");

    ~CroppedRGBDAcquisition();

    CroppedRGBDAcquisition(
        const CroppedRGBDAcquisition&) = delete;

    CroppedRGBDAcquisition& operator=(
        const CroppedRGBDAcquisition&) = delete;

    std::optional<Observation> acquireAfter(
        const rclcpp::Time& min_stamp,
        std::chrono::milliseconds timeout);

    std::optional<Observation> latest() const;

    void logCaptureFailure(
        const rclcpp::Time& boundary);

private:
    struct PendingFrame {
        RGBDFrame::ConstSharedPtr frame;

        std::chrono::steady_clock::time_point arrival;
    };

    void frameCallback(
        RGBDFrame::ConstSharedPtr frame);

    bool validFrame(const RGBDFrame& frame) const;

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

    rclcpp::Subscription<RGBDFrame>::SharedPtr
        rgbd_sub_;

    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // Accessed only by the acquisition callback group.
    std::optional<PendingFrame> pending_;

    std::size_t received_count_{0};
    std::size_t invalid_count_{0};
    std::size_t resolved_count_{0};
    std::size_t tf_failure_count_{0};
    std::size_t expired_count_{0};
    std::size_t superseded_count_{0};

    mutable std::mutex observation_mutex_;
    std::condition_variable observation_cv_;

    std::optional<Observation> latest_observation_;
};

}  // namespace hb_perception
