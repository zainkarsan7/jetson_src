//
// ROS2 Node for azure_kinect_ros2 driver
// 

//
#include <string>

// Library headers
//
#include <k4a/k4a.h>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>

// Project headers
//
#include "azure_kinect_ros2_driver_codex/k4a_ros_device.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("azure_kinect_node");
  int exit_code = 0;
  try
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<K4AROS2Device>();
    executor.add_node(node);
    if (node->startCameras() != K4A_RESULT_SUCCEEDED || node->startImu() != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR(logger, "Failed to start Kinect streams");
      exit_code = 1;
    }
    else
    {
      RCLCPP_INFO(logger, "K4A started");
      executor.spin();
    }
    executor.remove_node(node);
    node.reset();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Kinect startup/executor failed: %s", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
