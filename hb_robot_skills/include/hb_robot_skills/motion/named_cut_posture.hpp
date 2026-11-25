#pragma once
#include <moveit/robot_state/robot_state.h>
#include <cmath>
#include <map>
#include <string>

namespace hb_robot_skills::motion {
// Do not silently fill omitted arm joints from an unrelated reference posture.
// Collision and route feasibility are checked by the caller against its scene.
inline moveit::core::RobotStatePtr namedCutPosture(
    const moveit::core::RobotState& reference,
    const moveit::core::JointModelGroup* group,const std::string& name) {
  if(!group || name.empty()) return {};
  std::map<std::string,double> values;
  if(!group->getVariableDefaultPositions(name,values)) return {};
  for(const auto* joint:group->getActiveJointModels())
    for(const auto& variable:joint->getVariableNames()) {
      const auto it=values.find(variable);
      if(it==values.end() || !std::isfinite(it->second)) return {};
    }
  auto state=std::make_shared<moveit::core::RobotState>(reference);
  state->setVariablePositions(values);state->update();
  return state->satisfiesBounds(group)?state:moveit::core::RobotStatePtr{};
}
} // namespace hb_robot_skills::motion
