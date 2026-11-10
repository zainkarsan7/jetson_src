
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/logger.hpp>
#include <functional>
#include <optional>
#include <moveit/move_group_interface/move_group_interface.h>

namespace hb_robot_skills::motion {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using TransitValidator = std::function<bool(const MoveGroup::Plan&)>;

struct TransitPlannerOptions {
    bool try_pilz_ptp = true;
    bool try_ompl_rrtconnect = true;

    double pilz_planning_time_s = 1.5;
    double ompl_planning_time_s = 3.0;

    unsigned int pilz_attempts = 1;
    unsigned int ompl_attempts = 1;

    double velocity_scaling = 0.10;
    double acceleration_scaling = 0.10;
};


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
