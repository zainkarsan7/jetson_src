#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "hb_robot_perception/profile_types.hpp"
#include "hb_robot_skills/motion/cut_planner.hpp"
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_interfaces/srv/approve_motion.hpp"
#include "hb_robot_interfaces/action/cut_profile.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <mutex>
#include <optional>

namespace hb_robot_skills{

    class CutProfileServer: public rclcpp::Node{
        public:
            using CutProfile = hb_robot_interfaces::action::CutProfile;
            using GoalHandleCutProfile = rclcpp_action::ServerGoalHandle<CutProfile>;

            explicit CutProfileServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            void initialize();
            
            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CutProfile::Goal> goal);

            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleCutProfile>goal_handle);

            void handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle);
            void execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle);

            

            void abortGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            void cancelGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            void profileEstimateCallback(const hb_robot_interfaces::msg::ProfileEstimate::SharedPtr msg);

            std::optional<hb_robot_interfaces::msg::ProfileEstimate> latestProfileEstimate() const;

            visualization_msgs::msg::MarkerArray makeVisualization(const motion::CutPlan& plan, const std::string& frame_id)const;
            
            void publishVisualization(const motion::CutPlan& plan, const std::string& frame_id);

            void publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& stage);

            

            void handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
            std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response);

            
            
        private:


            std::shared_ptr<robot_trajectory::RobotTrajectory> planToTrajectory(
            const moveit::planning_interface::MoveGroupInterface::Plan& plan
            );
            std::unordered_map<std::string, motion::CutSegment> debug_segs_;
            std::mutex debug_segments_mutex;

            void publishCandidateVisualization(const motion::CutSegment segment);
            
            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planConstrainedCut(
                const motion::CutSegment& segment,
                double pos_tol,
                double ang_tol
            );

            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> makeLinPlan(
                const moveit::core::RobotState& start_state,
                const moveit::core::RobotState& goal_state);

            std::shared_ptr<robot_trajectory::RobotTrajectory> collateSegmentTrajectory(
                const motion::SegmentMotionPlan& smp
            );

            moveit::core::RobotState getFinalState(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
            std::optional<motion::SegmentMotionPlan> planSegmentPilzLinear(
            const motion::CutSegment& segment, const moveit::core::RobotState& actual_approach
        );

        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planLinear(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& goal_state
        );


        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planToState(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& target_state
        );
        
        std::string planning_group_;
        std::string plasma_link_;
        double planning_time_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr approval_service_;
        mutable std::mutex approval_mutex_;
        std::condition_variable approval_cv_;
        bool motion_approved_{false};
        hb_robot_interfaces::msg::ProfileEstimate latest_estimate_;
        std::unique_ptr<motion::CutPlanner> cut_planner_;
        rclcpp_action::Server<CutProfile>::SharedPtr action_server_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
        mutable std::mutex estimate_mutex_;
        rclcpp::Subscription<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr profile_estimate_sub_;
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_ ;

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    };



}