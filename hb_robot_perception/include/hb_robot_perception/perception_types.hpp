#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <cstddef>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hb_perception{
struct Observation{
    // put image header, camera pose?
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth_to_rgb;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr rgb_camera_info;
    geometry_msgs::msg::TransformStamped camera_pose;
        
    rclcpp::Time stamp;

};

}