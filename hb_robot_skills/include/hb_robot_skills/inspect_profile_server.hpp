
#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <optional>
#include <functional>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Geometry>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <hb_robot_interfaces/action/inspect_profile.hpp>
#include <hb_robot_interfaces/srv/approve_motion.hpp>
#include <hb_robot_interfaces/srv/commit_workpiece.hpp>
#include <hb_robot_interfaces/msg/profile_estimate.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>

#include "hb_robot_perception/rgbd_acquisition.hpp"
#include "hb_robot_perception/scene_model.hpp"
#include "hb_robot_perception/workpiece_extractor.hpp"
#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "hb_robot_perception/profile_observation_model.hpp"
#include "hb_robot_perception/profile_mesh_builder.hpp"

#include "hb_robot_skills/platform_supervision_client.hpp"
#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/exploration_types.hpp"

namespace hb_robot_skills {

class InspectProfileServer : public rclcpp::Node {
public:
    using InspectProfile =
        hb_robot_interfaces::action::InspectProfile;

    using GoalHandle =
        rclcpp_action::ServerGoalHandle<InspectProfile>;

    using CommitWorkpiece =
        hb_robot_interfaces::srv::CommitWorkpiece;

    explicit InspectProfileServer(
        const rclcpp::NodeOptions& options =
            rclcpp::NodeOptions());

    ~InspectProfileServer() override;

    void initialize();

private:
    enum class MotionOutcome {
        SUCCESS,
        REJECTED,
        CANCELED,
        FAILED
    };

    struct ProfileEstimate {
        hb_perception::WorkpieceModel workpiece;
        hb_perception::SectionModel section;
        hb_perception::ProfileMatch match;

        Eigen::Isometry3d world_from_member =
            Eigen::Isometry3d::Identity();

        float coverage = 0.0f;
        bool sufficient = false;
    };

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const InspectProfile::Goal>);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle>);

    void handleAccepted(
        const std::shared_ptr<GoalHandle>);

    void execute(
        const std::shared_ptr<GoalHandle>);

    MotionOutcome executeApprovedPlan(
        const moveit::core::RobotState& start,
        const moveit::planning_interface::
            MoveGroupInterface::Plan& plan,
        const std::shared_ptr<GoalHandle>& handle,
        const std::string& description);

    void handleApproval(
        const std::shared_ptr<
            hb_robot_interfaces::srv::ApproveMotion::Request>,
        std::shared_ptr<
            hb_robot_interfaces::srv::ApproveMotion::Response>);

    std::optional<
        moveit::planning_interface::MoveGroupInterface::Plan>
    planToState(
        const moveit::core::RobotState& start,
        const moveit::core::RobotState& target,
        const bool return_home = false
    );

    motion::ProfileInspectionRequest
    makeProfileInspectionRequest(
        const InspectProfile::Goal&) const;

    bool waitForStability();

    bool acquireSamples(
        uint32_t count,
        double depth_range,
        const std::function<bool()>& canceled);

    std::optional<ProfileEstimate> estimateProfile();

    void publishProfile(
        const ProfileEstimate&);

    void publishCoverage(
        const hb_perception::SectionModel&);

    void publishSceneCloud();

    void publishCameraPose(
        const Eigen::Isometry3d&,
        const std::string& ns,
        int id,
        std::optional<double> gain = std::nullopt);

    bool updateWorkpieceCollision(
        const hb_perception::WorkpieceModel&);

    std::optional<moveit_msgs::msg::CollisionObject>
    buildCuttingCollision(
        const ProfileEstimate&);

    bool commitInitialWorkpiece(
        const ProfileEstimate&,
        const std::shared_ptr<GoalHandle>&,
        hb_robot_interfaces::msg::WorkpieceReference&,
        std::string& error);

    hb_perception::WorkpieceExtractor extractor_;
    hb_perception::ProfileMatcher matcher_;
    hb_perception::ProfileObservationModel
        observation_model_;
    hb_perception::ProfileMeshBuilder mesh_builder_;

    std::shared_ptr<hb_perception::SceneModel>
        scene_model_;

    std::unique_ptr<hb_perception::RGBDAcquisition>
        rgbd_acquisition_;

    std::unique_ptr<PlatformSupervisionClient>
        platform_supervision_;

    std::unique_ptr<motion::NBVPlanner>
        nbv_planner_;

    std::unique_ptr<
        moveit::planning_interface::MoveGroupInterface>
        move_group_;

    planning_scene_monitor::PlanningSceneMonitorPtr
        planning_scene_monitor_;

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener>
        tf_listener_;

    rclcpp_action::Server<InspectProfile>::SharedPtr
        action_server_;

    rclcpp::Service<
        hb_robot_interfaces::srv::ApproveMotion>::SharedPtr
        approval_service_;

    rclcpp::Client<CommitWorkpiece>::SharedPtr
        commit_client_;

    rclcpp::Publisher<
        moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        display_traj_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        scene_cloud_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr
        profile_marker_pub_,
        coverage_marker_pub_,
        camera_pose_pub_;

    rclcpp::Publisher<
        hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr
        profile_estimate_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        rms_pub_,
        inlier_pub_,
        coverage_pub_;

    std::string planning_group_;
    std::string camera_link_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string scene_frame_;

    double planning_time_ = 5.0;
    double stability_timeout_ = 5.0;
    double acquisition_timeout_ = 5.0;
    double commit_timeout_ = 10.0;

    float rms_target_ = 0.01f;
    float inlier_fraction_target_ = 0.4f;
    float coverage_target_ = 0.4f;

    double exploration_weight_ = 100.0;

    double motion_weight_ = 0.001;
    double inspection_pos_tol_ = 0.03;
    double inspection_ang_tol_ = 0.1;
        
    bool skip_motion_ = true;
    bool require_plan_approval_ = true;

    std::mutex approval_mutex_;
    std::condition_variable approval_cv_;
    bool motion_approved_ = false;
    bool motion_rejected_ = false;

    std::mutex execution_mutex_;
    std::atomic<bool> busy_{false};
    std::thread worker_;

    std::optional<rclcpp::Time> first_observation_;
};

}  // namespace hb_robot_skills
