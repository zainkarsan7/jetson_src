#pragma once
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <hb_robot_interfaces/action/inspect_scene.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
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

        rclcpp_action::Server<InspectScene>::SharedPtr action_server_;
        std::shared_ptr<MoveGroupInterface> move_group_;
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
        bool execute_motion_;
};