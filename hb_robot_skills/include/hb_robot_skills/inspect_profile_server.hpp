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
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "hb_robot_perception/rgbd_acquisition.hpp"
#include "hb_robot_perception/scene_model.hpp"
#include "hb_robot_perception/workpiece_extractor.hpp"
#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "hb_robot_perception/profile_observation_model.hpp"

#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/exploration_types.hpp"
#include <hb_robot_interfaces/action/inspect_profile.hpp>
#include "hb_robot_interfaces/srv/approve_motion.hpp"
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include <condition_variable>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/float64.hpp>

namespace hb_robot_skills{




class InspectProfileServer: public rclcpp::Node{
    public:

        using InspectProfile = hb_robot_interfaces::action::InspectProfile;
        using GoalHandleInspectProfile = rclcpp_action::ServerGoalHandle<InspectProfile>;

        explicit InspectProfileServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
        void initialize();

    private:

        enum class MotionOutcome{
                SUCCESS,
                REJECTED,
                CANCELED,
                FAILED
            };

        MotionOutcome executeApprovedPlan(
            const moveit::core::RobotState& start_state,
            const moveit::planning_interface::MoveGroupInterface::Plan& plan,
            const std::shared_ptr<GoalHandleInspectProfile>& goal_handle,
            const std::string& description
        );


    ///APPROVAL STUFF
        rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr approval_service_;
        std::mutex approval_mutex_;
        std::condition_variable approval_cv_;
        bool motion_approved_{false};

        void handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
        std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response);

        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const InspectProfile::Goal> goal);

        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleInspectProfile> goal_handle);

        void handleAccepted(const std::shared_ptr<GoalHandleInspectProfile> goal_handle);
        void execute(const std::shared_ptr<GoalHandleInspectProfile> goal_handle);
        motion::ProfileInspectionRequest makeProfileInspectionRequest(
            const InspectProfile::Goal& goal) const;

        
        
        bool moveToView(const moveit::core::RobotState& target_state);
        bool waitForStability();
        bool acquireSamples(uint32_t sample_count, double depth_range);
        bool registerView();
        bool moveHome();

        void publishFeedback(const std::shared_ptr<GoalHandleInspectProfile>& goal_handle,
            uint32_t views_captured, uint32_t total_viewpoints, uint8_t phase);
        

        void abortGoal(const std::shared_ptr<GoalHandleInspectProfile>& goal_handle,
            uint8_t result_code,
            uint32_t viewpoints_captured,
            const std::string& message);
        
        void cancelGoal(const std::shared_ptr<GoalHandleInspectProfile>& goal_handle,
                        uint32_t viewpoints_captured
        );

        bool updateWorkpieceCollision(
            const hb_perception::WorkpieceModel& workpiece
        );


        void publishCameraPose(
            const Eigen::Isometry3d& pose,
            const std::string& ns,
            int id_base,
            std::optional<double> predicted_gain = std::nullopt
        );


        struct ProfileEstimate{
            hb_perception::WorkpieceModel workpiece;
            hb_perception::SectionModel section;    
            hb_perception::ProfileMatch match;
            Eigen::Isometry3d world_from_member =  Eigen::Isometry3d::Identity();
            float coverage = 0.0f;
            bool sufficient = false;
        };

        hb_perception::WorkpieceExtractor extractor_;
        hb_perception::ProfileMatcher matcher_;
        hb_perception::ProfileObservationModel observation_model_;
        std::vector<hb_perception::ProfileModel> profiles_;

        std::optional<ProfileEstimate> estimateProfile();
        void publishProfile(const ProfileEstimate& estimate);
        void publishCoverage(const hb_perception::SectionModel& section);
        void publishSceneCloud();
        std::optional<moveit::planning_interface::MoveGroupInterface::Plan>planToState(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& target_state
        );

        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        
        

        rclcpp_action::Server<InspectProfile>::SharedPtr action_server_;
        std::unique_ptr<motion::NBVPlanner> nbv_planner_;

        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scene_cloud_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr profile_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_marker_pub_;
        rclcpp::Publisher<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr profile_estimate_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_pose_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rms_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr inlier_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr coverage_pub_;
    
        std::string planning_group_;
        std::string camera_link_;
        double planning_time_;
        double stability_timeout_;
        std::mutex execution_mutex_;
        bool skip_motion_{true};
        bool require_plan_approval_{false};
        bool motion_rejected_{false};

        std::shared_ptr<hb_perception::SceneModel> scene_model_;
        std::unique_ptr<hb_perception::RGBDAcquisition> rgbd_acquisition_;
        // configure acquisition stuff
        std::string rgb_topic_;
        std::string depth_topic_;
        std::string camera_info_topic_;
        std::string scene_frame_;
        double acquisition_timeout_;

        // tf buffer stuff
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        
};


}
