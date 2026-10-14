#include "hb_robot_skills/inspect_profile_server.hpp"
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>


namespace hb_robot_skills{ 

    InspectProfileServer::InspectProfileServer(const rclcpp::NodeOptions & options):Node("inspect_profile_server",options)
, scene_model_(std::make_shared<hb_perception::ProfileObservationModel>())
{
    // pass stuff to the private variables using some parameters
    planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
    camera_link_ = declare_parameter<std::string>("camera_link","camera_visor");
    planning_time_ = declare_parameter<double>("planning_time",5.0);
    stability_timeout_ = declare_parameter<double>("stability_timeout",5.0);
    skip_motion_ = declare_parameter<bool>("skip_inspection_motion", true);
    rgb_topic_ = declare_parameter<std::string>("perception_rgb_topic","/k4a/rgb/image_raw");
    depth_topic_=declare_parameter<std::string>("perception_depth_topic","/k4a/depth_to_rgb/image_raw");
    camera_info_topic_=declare_parameter<std::string>("perception_camera_info_topic","/k4a/depth_to_rgb/camera_info");
    scene_frame_= declare_parameter<std::string>("scene_base_frame","world");
    acquisition_timeout_ = declare_parameter<double>("acquisition timeout",2.0);
    
}




}