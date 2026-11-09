#pragma once

#include <functional>
#include <optional>
#include <moveit/move_group_interface/move_group_interface.h>

namespace hb_robot_skills::motion {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using TransitValidator = std::function<bool(const MoveGroup::Plan&)>;

std::optional<MoveGroup::Plan> planTransit(
    MoveGroup& group, const rclcpp::Logger& logger,
    const moveit::core::RobotState& start_state, const moveit::core::RobotState& target_state,
    const TransitValidator& validate = {}, bool compare_all_fallbacks = false);
}
