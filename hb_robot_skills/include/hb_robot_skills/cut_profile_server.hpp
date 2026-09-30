#pragma once

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

            CutProfileServer();
        
            
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

        rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr approval_service_;
        mutable std::mutex approval_mutex_;
        std::condition_variable approval_cv_;
        bool motion_approved_{false};
        hb_robot_interfaces::msg::ProfileEstimate latest_estimate_;
        motion::CutPlanner cut_planner_;
        rclcpp_action::Server<CutProfile>::SharedPtr action_server_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
        mutable std::mutex estimate_mutex_;
        rclcpp::Subscription<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr profile_estimate_sub_;
        



    };



}