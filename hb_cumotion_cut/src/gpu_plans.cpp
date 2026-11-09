#include "cut_profile_server.hpp"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace hb_robot_skills {
using Plan = motion::CutGraphPlanner::Plan;

std::optional<Plan> CutProfileServer::planToState(
    const moveit::core::RobotState& start, const moveit::core::RobotState& target) {
    using Service = hb_cumotion_cut::srv::PlanTransit;
    auto req = std::make_shared<Service::Request>();
    const auto* group = start.getJointModelGroup(planning_group_);
    req->frame = start.getRobotModel()->getModelFrame();
    req->start.name = req->target.name = group->getVariableNames();
    start.copyJointGroupPositions(group, req->start.position);
    target.copyJointGroupPositions(group, req->target.position);
    if (!gpu_scene_) throw std::logic_error("Missing planning scene snapshot");
    gpu_scene_->getCollisionObjectMsgs(req->world.collision_objects);
    auto reply = gpuRequest<Service>(transit_client_, req);
    if (!reply->success) {
        RCLCPP_WARN(get_logger(), "cuMotion transit: %s", reply->message.c_str());
        return std::nullopt;
    }
    auto& jt = reply->trajectory.joint_trajectory;
    if (!sameJoints(jt.joint_names, req->start.name) || jt.points.empty()) return std::nullopt;
    moveit::core::RobotState state(start);
    double previous = -1.0;
    for (const auto& p : jt.points) {
        const double t = rclcpp::Duration(p.time_from_start).seconds();
        if (t < 0 || t <= previous || !finitePositions(p.positions, jt.joint_names.size()) ||
            (!p.velocities.empty() && !finitePositions(p.velocities, jt.joint_names.size())) ||
            (!p.accelerations.empty() && !finitePositions(p.accelerations, jt.joint_names.size())))
            return std::nullopt;
        previous = t;
        for (size_t j = 0; j < jt.joint_names.size(); ++j)
            state.setVariablePosition(jt.joint_names[j], p.positions[j]);
        state.update();
        if (!state.satisfiesBounds(group) || gpu_scene_->isStateColliding(state, planning_group_))
            return std::nullopt;
    }
    for (size_t j = 0; j < jt.joint_names.size(); ++j) {
        const auto& n = jt.joint_names[j];
        if (std::abs(jt.points.front().positions[j] - start.getVariablePosition(n)) > 1e-3 ||
            std::abs(jt.points.back().positions[j] - target.getVariablePosition(n)) > 1e-3)
            return std::nullopt;
    }
    jt.header.stamp = builtin_interfaces::msg::Time();
    Plan plan;
    moveit::core::robotStateToRobotStateMsg(start, plan.start_state_);
    plan.trajectory_ = std::move(reply->trajectory);
    return plan;
}

std::optional<Plan> CutProfileServer::timeProcess(
    const std::vector<moveit::core::RobotStatePtr>& states, double speed) const {
    if (states.size() < 2 || !std::isfinite(speed) || speed <= 0) return std::nullopt;
    robot_trajectory::RobotTrajectory path(states.front()->getRobotModel(), planning_group_);
    for (const auto& s : states) path.addSuffixWayPoint(*s, 0.0);
    trajectory_processing::IterativeParabolicTimeParameterization retimer;
    if (!retimer.computeTimeStamps(path, 0.2, 0.1)) return std::nullopt;
    // Uniform time scaling preserves joint velocity/acceleration consistency and
    // the accepted IK samples. Bound Cartesian chord speed for every interval.
    double scale = 1.0;
    for (size_t i = 1; i < states.size(); ++i) {
        const double dt = path.getWayPointDurationFromPrevious(i);
        const auto& from = states[i-1]->getGlobalLinkTransform(plasma_link_);
        const auto& to = states[i]->getGlobalLinkTransform(plasma_link_);
        if (!std::isfinite(dt) || dt <= 0) return std::nullopt;
        scale = std::max(scale, (to.translation()-from.translation()).norm() / (dt * speed));
    }
    Plan plan;
    moveit::core::robotStateToRobotStateMsg(*states.front(), plan.start_state_);
    path.getRobotTrajectoryMsg(plan.trajectory_);
    for (auto& p : plan.trajectory_.joint_trajectory.points) {
        p.time_from_start = rclcpp::Duration::from_seconds(
            rclcpp::Duration(p.time_from_start).seconds() * scale);
        for (auto& v : p.velocities) v /= scale;
        for (auto& a : p.accelerations) a /= scale * scale;
    }
    return plan;
}
}  // namespace hb_robot_skills
