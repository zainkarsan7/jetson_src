#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <cstddef>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hb_perception{
struct Observation{
    // put image header, camera pose?
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    geometry_msgs::msg::TransformStamped camera_pose;
    rclcpp::Time stamp;

};

}