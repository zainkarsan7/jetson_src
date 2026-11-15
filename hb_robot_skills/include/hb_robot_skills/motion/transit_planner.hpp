
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/logger.hpp>
#include <functional>
#include <optional>
#include <memory>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace hb_robot_skills::motion {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using TransitValidator = std::function<bool(const MoveGroup::Plan&)>;

struct TransitPlannerOptions {
    double velocity_scaling = 0.10;
    double acceleration_scaling = 0.10;
    bool compare_all_fallbacks = false;

    // Guided PTP.
    bool guided_ptp_enabled = true;
    std::string guide_link = "ur10e_torch_link";

    // JMG variable order, obtained from a known home RobotState.
    std::vector<double> home_joints;
    double home_tolerance_rad = 0.05;
    // JMG variable order. Empty means use the uniform
    // guided_joint_consistency_rad limit instead.
    std::vector<double> guided_joint_limits_rad;
    // Structured lifted-midpoint sampling.
    double guided_lift_m = 0.20;
    double guided_lateral_m = 0.08;

    // Loose IK pose sampling.
    double guided_position_tolerance_m = 0.025;
    double guided_orientation_tolerance_rad = 0.35;
    double guided_joint_consistency_rad = 1.0;
    double guided_ik_timeout_s = 0.01;

    // Hard search limits.
    unsigned guided_max_waypoints = 10;
    unsigned guided_max_ik_calls = 100;
    unsigned guided_max_ptp_calls = 30;
    double guided_ptp_time_s = 1.0;

    // Full planning-scene snapshot for validation.
    planning_scene::PlanningSceneConstPtr scene;

    // Maximum joint-space interpolation increment
    // for additional collision checking.
    double guided_check_step_rad = 0.02;
};

std::optional<MoveGroup::Plan> planGuidedPTP(
    MoveGroup& group,
    const rclcpp::Logger& logger,
    const moveit::core::RobotState& start,
    const moveit::core::RobotState& goal,
    const TransitPlannerOptions& options);





std::optional<MoveGroup::Plan> planTransit(
    MoveGroup& group, const rclcpp::Logger& logger,
    const moveit::core::RobotState& start_state, const moveit::core::RobotState& target_state,
    const TransitValidator& validate = {}, bool compare_all_fallbacks = false);


std::optional<
    MoveGroup::Plan>
planTransit(
    MoveGroup& group,
    const rclcpp::Logger& logger,
    const moveit::core::RobotState& start,
    const moveit::core::RobotState& target,
    const TransitPlannerOptions& options);


}  // namespace hb_robot_skills::motion
