#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <hb_robot_perception/perception_types.hpp>

namespace hb_perception {

class RGBDAcquisition {
public:
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

    RGBDAcquisition(rclcpp::Node* node, tf2_ros::Buffer* tf_buffer,
        const std::string& target_frame,
        const std::string& rgb_topic = "/k4a/rgb/image_raw",
        const std::string& depth_topic = "/k4a/depth_to_rgb/image_raw",
        const std::string& camera_info_topic = "/k4a/depth_to_rgb/camera_info");

    // Both exposures must be strictly newer than min_stamp.
    std::optional<Observation> acquireAfter(const rclcpp::Time& min_stamp,
        std::chrono::milliseconds timeout);
    std::optional<Observation> latest() const;

private:
    struct PendingPair {
        Image::ConstSharedPtr rgb;
        Image::ConstSharedPtr depth;
        std::chrono::steady_clock::time_point arrival;
    };

    void synchronizedCallback(const Image::ConstSharedPtr& rgb, const Image::ConstSharedPtr& depth);
    void resolvePending();

    rclcpp::Node* node_;
    tf2_ros::Buffer* tf_buffer_;
    const std::string target_frame_;

    // All subscriptions and the retry timer use the node's mutually exclusive default group.
    std::deque<PendingPair> pending_;
    CameraInfo::ConstSharedPtr camera_info_;
    message_filters::Subscriber<Image> rgb_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
    rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    std::size_t synchronized_count_ = 0;
    std::size_t resolved_count_ = 0;
    std::size_t expired_count_ = 0;
    mutable std::mutex observation_mutex_;
    std::condition_variable observation_cv_;
    std::optional<Observation> latest_observation_;
};

}  // namespace hb_perception
