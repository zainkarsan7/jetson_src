#pragma once
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <hb_robot_perception/perception_types.hpp>


namespace hb_perception{

class RGBDAcquisition{
    public:

        using Image = sensor_msgs::msg::Image;

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image,Image>;


        RGBDAcquisition(rclcpp::Node * node, tf2_ros::Buffer* tf_buffer, 
        const std::string& rgb_topic, const std::string& depth_topic, const std::string& target_frame);

        std::optional<Observation> acquireAfter(const rclcpp::Time& min_stamp,
        std::chrono::milliseconds timeout);
    private:
            void synchronizedCallback(const Image::ConstSharedPtr& rgb,
            const Image::ConstSharedPtr& depth);

            rclcpp::Node* node_;
            tf2_ros::Buffer* tf_buffer_;
            message_filters::Subscriber<Image> rgb_sub_;
            message_filters::Subscriber<Image> depth_sub_;
            const std::string target_frame_;

            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
            std::optional<Observation> latest_observation_;
            std::mutex observation_mutex_;
            std::condition_variable observation_cv_;
};
}
