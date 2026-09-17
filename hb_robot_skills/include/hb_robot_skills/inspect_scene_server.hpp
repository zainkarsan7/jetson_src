#pragma once
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <hb_robot_interfaces/action/inspect_scene.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "hb_robot_skills/motion/motion_planner.hpp"
using InspectScene = hb_robot_interfaces::action::InspectScene;
using GoalHandleInspectScene = rclcpp_action::ServerGoalHandle<InspectScene>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class InspectSceneServer : public rclcpp::Node {
    public:
    explicit InspectSceneServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    void initializeMoveit();
    
    private :

        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const InspectScene::Goal> goal);

        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleInspectScene> goal_handle);

        void handleAccepted(const std::shared_ptr<GoalHandleInspectScene> goal_handle);
        void execute(const std::shared_ptr<GoalHandleInspectScene> goal_handle);
        void publishViewpointMarker(const std::vector<geometry_msgs::msg::Pose> &viewpoints);
        rclcpp_action::Server<InspectScene>::SharedPtr action_server_;
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoint_marker_pub_;
        bool execute_motion_;
        std::unique_ptr<hb_robot_skills::motion::MotionPlanner> motion_planner_;
};