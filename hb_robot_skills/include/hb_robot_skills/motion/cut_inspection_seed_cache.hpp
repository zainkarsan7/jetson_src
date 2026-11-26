#pragma once
#include <hb_robot_interfaces/msg/cut_inspection_seeds.hpp>
#include <moveit/robot_state/robot_state.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>

namespace hb_robot_skills::motion {
inline bool inspectionSeedsMatch(const hb_robot_interfaces::msg::CutInspectionSeeds& cache,
    const std::string& cut_id, const std::string& profile, const std::string& group,
    const std::string& frame, uint64_t geometry, uint64_t pose) {
  return !cut_id.empty() && cache.cut_id==cut_id && cache.profile_name==profile &&
    cache.planning_group==group && cache.header.frame_id==frame &&
    cache.geometry_version==geometry && cache.pose_version==pose &&
    cache.features.size()==cache.states.size();
}

// Only copy the planning group's complete joint vector. Preserve current
// auxiliary joints (including torch height) and all attached-body state.
inline moveit::core::RobotStatePtr inspectionSeedState(const sensor_msgs::msg::JointState& joints,
    const moveit::core::RobotState& current, const moveit::core::JointModelGroup* group) {
  if (!group || joints.name.size()!=joints.position.size() ||
      joints.name.size()!=group->getVariableCount()) return {};
  const auto& names = group->getVariableNames();
  std::set<std::string> seen;
  auto seed = std::make_shared<moveit::core::RobotState>(current);
  for (std::size_t i=0; i<joints.name.size(); ++i) {
    if (!std::isfinite(joints.position[i]) || !seen.insert(joints.name[i]).second ||
        std::find(names.begin(), names.end(), joints.name[i])==names.end()) return {};
    seed->setVariablePosition(joints.name[i], joints.position[i]);
  }
  seed->update();
  return seed->satisfiesBounds(group) ? seed : moveit::core::RobotStatePtr{};
}
} // namespace hb_robot_skills::motion
