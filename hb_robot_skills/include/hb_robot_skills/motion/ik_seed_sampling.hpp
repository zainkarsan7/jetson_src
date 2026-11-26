#pragma once
#include <moveit/robot_state/robot_state.h>
#include <random>

namespace hb_robot_skills::motion {
struct IKSeed {
  moveit::core::RobotState state;
  bool constrain;
  bool global = false;
};

// Shared by cutting and inspection: preserve the geometric warm start, try the
// reference, then interleave global branches with bounded local perturbations.
inline IKSeed sampleIKSeed(const moveit::core::RobotState& reference,
    const moveit::core::RobotState* geometric,
    const moveit::core::JointModelGroup* group, std::size_t index,
    std::size_t global_count, double radius, std::mt19937& rng) {
  const bool global = global_count > 0 && index >= 2 &&
    (index-2)%3 == 0 && (index-2)/3 < global_count;
  const bool use_geometric = geometric && index != 1 && !global;
  IKSeed result{use_geometric ? *geometric : reference,
                use_geometric || (!global && index > 1), global};
  if (global) result.state.setToRandomPositions(group);
  else if (index > 1) {
    std::uniform_real_distribution<double> jitter(-radius, radius);
    std::vector<double> positions;
    result.state.copyJointGroupPositions(group, positions);
    for (double& position : positions) position += jitter(rng);
    result.state.setJointGroupPositions(group, positions);
    result.state.enforceBounds(group);
  }
  result.state.update();
  return result;
}
} // namespace hb_robot_skills::motion
