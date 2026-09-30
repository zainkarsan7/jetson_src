#pragma once

#include "hb_robot_perception/profile_types.hpp"
#include "hb_robot_skills/motion/cut_planner.hpp"
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
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

            CutProfileServer();
        private:
            
            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CutProfile::Goal> goal);

            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleCutProfile>goal_handle);

            void handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle);
            void execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle);

            void publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            std::string& stage);

            void abortGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            void cancelGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            rclcpp_action::Server<CutProfile>::SharedPtr action_server_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cut_plan_pub_;
            std::mutex cutting_mutex_;


    };



}