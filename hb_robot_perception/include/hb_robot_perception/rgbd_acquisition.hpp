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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <hb_robot_perception/perception_types.hpp>


namespace hb_perception{

class RGBDAcquisition{
    public:

        using Image = sensor_msgs::msg::Image;
        using CameraInfo = sensor_msgs::msg::CameraInfo;

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image,Image>;


        RGBDAcquisition(rclcpp::Node * node, tf2_ros::Buffer* tf_buffer, const std::string& target_frame, 
        const std::string& rgb_topic = "/k4a/rgb/image_raw", 
        const std::string& depth_topic = "/k4a/depth_to_rgb/image_raw" ,
        const std::string& camera_info_topic = "k4a/depth_to_rgb/camera_info");

        /**
         * wait for RGBD observation captuered after min stamp and before timout may return nullopt
         */
        std::optional<Observation> acquireAfter(const rclcpp::Time& min_stamp,
        std::chrono::milliseconds timeout);
    private:
            // void pointCloudCallback(
            //     const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
            
            void synchronizedCallback(const Image::ConstSharedPtr& rgb,
            const Image::ConstSharedPtr& depth);
            void cameraInfoCallback(const CameraInfo::ConstSharedPtr& camera_info);

            rclcpp::Node* node_;
            tf2_ros::Buffer* tf_buffer_;
            message_filters::Subscriber<Image> rgb_sub_;
            message_filters::Subscriber<Image> depth_sub_;
            // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
            // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
            
            const std::string target_frame_;

            rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
            std::optional<Observation> latest_observation_;
            std::mutex observation_mutex_;
            std::condition_variable observation_cv_;
            CameraInfo::ConstSharedPtr camera_info_;
};

}
