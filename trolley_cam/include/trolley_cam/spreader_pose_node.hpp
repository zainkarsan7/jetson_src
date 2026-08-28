#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "trolley_cam/aruco_detector.hpp"

namespace trolley_cam{

    class SpreaderPoseNode: public rclcpp::Node{
        public:
        explicit SpreaderPoseNode(const rclcpp::NodeOptions & options);
        private:
        
        
        void get_cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
        void get_detections_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        ArucoDetector detector_;
        image_transport::Subscriber trolley_image_sub_;
        image_transport::Publisher anno_trolley_image_pub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr trolley_cam_info_sub_;
        sensor_msgs::msg::CameraInfo trolley_cam_info;

        std::string cam_frame_id;
        bool cam_info_received_{false};

        std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_left_pose_broadcaster_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_right_pose_broadcaster_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr left_t_pub;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr right_t_pub;


    };


}
