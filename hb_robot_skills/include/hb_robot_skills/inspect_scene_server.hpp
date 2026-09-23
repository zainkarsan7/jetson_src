#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/exploration_types.hpp"
#include <hb_robot_interfaces/action/inspect_scene.hpp>

namespace hb_robot_skills{

    class InspectSceneServer : public rclcpp::Node {
    public:
        using InspectScene = hb_robot_interfaces::action::InspectScene;
        using GoalHandleInspectScene = rclcpp_action::ServerGoalHandle<InspectScene>;

        explicit InspectSceneServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
        void initialize();
    
    private :

        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const InspectScene::Goal> goal);

        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleInspectScene> goal_handle);

        void handleAccepted(const std::shared_ptr<GoalHandleInspectScene> goal_handle);
        void execute(const std::shared_ptr<GoalHandleInspectScene> goal_handle);
        
        void publishViewpointMarker(const std::vector<geometry_msgs::msg::Pose> &viewpoints);
        motion::ExplorationRequest makeExplorationRequest(
            const InspectScene::Goal& goal) const;
        
        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planToView(
            const moveit::core::RobotState& start_state,
            const motion::ViewSolution &view 
        );
        
        bool moveToView(const moveit::core::RobotState& target_state);
        bool waitForStability();
        bool acquireSamples(uint32_t sample_count);
        bool registerView();

        void publishFeedback(const std::shared_ptr<GoalHandleInspectScene>& goal_handle,
            uint32_t current_viewpoint, uint32_t total_viewpoints, uint8_t phase);
        

        void abortGoal(const std::shared_ptr<GoalHandleInspectScene>& goal_handle,
            uint8_t result_code,
            uint32_t viewpoints_captured,
            const std::string& message);
        
        void cancelGoal(const std::shared_ptr<GoalHandleInspectScene>& goal_handle,
                        uint32_t viewpoints_captured
        );
        
        rclcpp_action::Server<InspectScene>::SharedPtr action_server_;

        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoint_marker_pub_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        std::unique_ptr<motion::ExplorationPlanner> exploration_planner_;
        
        std::string planning_group_;
        std::string camera_link_;
        double planning_time_;
        double stability_timeout_;
        std::mutex execution_mutex_;
        bool skip_motion_{true};
        bool require_plan_approval_{false};
    
};


}

