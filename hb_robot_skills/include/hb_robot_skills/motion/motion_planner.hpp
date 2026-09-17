#pragma once
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

namespace hb_robot_skills::motion{

class MotionPlanner{
    public:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using Plan = MoveGroupInterface::Plan;

    MotionPlanner(const rclcpp::Node::SharedPtr& node,
        const std::string & planning_group
    );


    bool planToPose(const geometry_msgs::msg::Pose& pose, const std::string& link_name, 
    moveit::planning_interface::MoveGroupInterface::Plan& plan);

    bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
    std::unique_ptr<MoveGroupInterface> move_group_;
    private:
    rclcpp::Node::SharedPtr node_;
   
};

}

