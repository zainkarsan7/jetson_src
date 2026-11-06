

#include "hb_robot_skills/motion/cut_graph_planner.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

namespace hb_robot_skills::motion{


    CutGraphPlanner::CutGraphPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            TransitPlanner transit_planner,
            ProcessPlanner process_planner,
            Options options
        ):
        robot_model_(std::move(robot_model)),
        planning_group_(std::move(planning_group)),
        transit_planner_(std::move(transit_planner)),
        process_planner_(std::move(process_planner)),
        options_(options){


            if(!robot_model_){
                throw std::invalid_argument("null robot model");
            }
            joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
            if(!joint_model_group_){
                throw std::invalid_argument("unknonw planning group");

            }
            if(!transit_planner_ || !process_planner_){
                throw std::invalid_argument("missing planning callback");

            }

            if(options_.max_candidates_per_segment == 0||
                !std::isfinite(options_.boundary_tol) || options_.boundary_tol <=0.0||
                !std::isfinite(options_.process_weight) || options_.process_weight <0.0||
                !std::isfinite(options_.transit_weight) || options_.transit_weight <0.0 ||
                !std::isfinite(options_.preferred_joint_travel_rad) || options_.preferred_joint_travel_rad < 0.0 ||
                !std::isfinite(options_.excess_travel_weight) || options_.excess_travel_weight < 0.0 ||
                !std::isfinite(options_.max_transit_joint_travel_rad) || options_.max_transit_joint_travel_rad < 0.0
            ){
               throw std::invalid_argument("inbalid options");

            }
        };

// double estimatedTransitCost(
//     const moveit::core::RobotState& from,
//     const moveit::core::RobotState& to,
//     const moveit::core::JointModelGroup* jmg)
// {
//     const auto* joints = jmg->getActiveJointModels();

//     double total = 0.0;
//     double maximum = 0.0;

//     for (const auto* joint : joints)
//     {
//         const double d = joint->distance(
//             from.getJointPositions(joint),
//             to.getJointPositions(joint));

//         total += d;
//         maximum = std::max(maximum, d);
//     }

//     return total + 2.0 * maximum;
// }


        CutGraphPlanner::JointMotionMetrics
CutGraphPlanner::jointMotionMetrics(const Plan& plan) const
{
    moveit::core::RobotState reference(robot_model_);
    reference.setToDefaultValues();

    moveit::core::robotStateMsgToRobotState(
        plan.start_state_, reference);

    robot_trajectory::RobotTrajectory trajectory(
        robot_model_, planning_group_);

    trajectory.setRobotTrajectoryMsg(
        reference, plan.trajectory_);

    JointMotionMetrics metrics;

    const auto& active_joints =
        joint_model_group_->getActiveJointModels();

    for (const auto* joint : active_joints)
    {
        // This diagnostic assumes one variable per active joint,
        // as is typical for the UR10e arm group.
        if (joint->getVariableCount() != 1)
            continue;

        const std::string& variable =
            joint->getVariableNames().front();

        double joint_travel = 0.0;

        for (std::size_t i = 1;
             i < trajectory.getWayPointCount();
             ++i)
        {
            const auto& previous =
                trajectory.getWayPoint(i - 1);

            const auto& current =
                trajectory.getWayPoint(i);

            const double delta = std::abs(
                current.getVariablePosition(variable) -
                previous.getVariablePosition(variable));

            if (!std::isfinite(delta)) {
                metrics.total_travel = std::numeric_limits<double>::infinity();
                return metrics;
            }

            joint_travel += delta;

            metrics.maximum_step =
                std::max(metrics.maximum_step, delta);
        }

        metrics.total_travel += joint_travel;

        metrics.maximum_joint_travel = std::max(
            metrics.maximum_joint_travel,
            joint_travel);
    }

    return metrics;
}

        std::optional<moveit::core::RobotState> CutGraphPlanner::firstState(
            const Plan& plan
        )const{
            moveit::core::RobotState reference(robot_model_);
            reference.setToDefaultValues();

            moveit::core::robotStateMsgToRobotState(plan.start_state_,reference);
            robot_trajectory::RobotTrajectory trajectory(robot_model_,planning_group_);
            trajectory.setRobotTrajectoryMsg(reference,plan.trajectory_);
            if(trajectory.getWayPointCount()==0){
                return std::nullopt;
            }
            return trajectory.getFirstWayPoint();
        }
        std::optional<moveit::core::RobotState> CutGraphPlanner::lastState(
            const Plan& plan
        )const{
            moveit::core::RobotState reference(robot_model_);
            reference.setToDefaultValues();

            moveit::core::robotStateMsgToRobotState(plan.start_state_,reference);
            robot_trajectory::RobotTrajectory trajectory(robot_model_,planning_group_);
            trajectory.setRobotTrajectoryMsg(reference,plan.trajectory_);
            if(trajectory.getWayPointCount()==0){
                return std::nullopt;
            }
            return trajectory.getLastWayPoint();

        }



        double CutGraphPlanner::trajectoryCost(const Plan& plan)const{

            moveit::core::RobotState reference(robot_model_);
            reference.setToDefaultValues();

            moveit::core::robotStateMsgToRobotState(plan.start_state_,reference);
            robot_trajectory::RobotTrajectory trajectory(robot_model_,planning_group_);
            trajectory.setRobotTrajectoryMsg(reference,plan.trajectory_);

            double cost = 0.0;
            for (std::size_t i=1; i<trajectory.getWayPointCount(); i++){
                cost += trajectory.getWayPoint(i-1).distance(trajectory.getWayPoint(i),joint_model_group_);
            }
            return cost;

        }

        double CutGraphPlanner::transitEstimate(const moveit::core::RobotState& from,
                                                const moveit::core::RobotState& to, double endpoint_slack)const{
            double total = 0.0, maximum = 0.0;
            // Same commanded-position metric as jointMotionMetrics, including
            // actual winding for the arm's single-variable revolute joints.
            for (const auto* joint : joint_model_group_->getActiveJointModels()) {
                if (joint->getVariableCount() != 1) continue;
                const auto& name = joint->getVariableNames().front();
                const double travel = std::max(0.0, std::abs(to.getVariablePosition(name) - from.getVariablePosition(name)) - endpoint_slack);
                total += travel;
                maximum = std::max(maximum, travel);
            }
            const double excess = std::max(0.0, maximum - options_.preferred_joint_travel_rad);
            return total + 2.0 * maximum + options_.excess_travel_weight * excess * excess;
        }

        double CutGraphPlanner::transitCost(const Plan& plan)const{
            const auto metrics = jointMotionMetrics(plan);
            if (!std::isfinite(metrics.total_travel) ||
                (options_.max_transit_joint_travel_rad > 0.0 &&
                 metrics.maximum_joint_travel > options_.max_transit_joint_travel_rad))
                return std::numeric_limits<double>::infinity();
            const double excess = std::max(0.0, metrics.maximum_joint_travel - options_.preferred_joint_travel_rad);
            return metrics.total_travel + 2.0 * metrics.maximum_joint_travel +
                options_.excess_travel_weight * excess * excess;
        }

        bool CutGraphPlanner::statesMatch(const moveit::core::RobotState& a,
        const moveit::core::RobotState& b)const{
            if (a.distance(b,joint_model_group_) > options_.boundary_tol) return false;
            for (const auto& name : joint_model_group_->getVariableNames()) {
                const double delta = std::abs(a.getVariablePosition(name) - b.getVariablePosition(name));
                if (!std::isfinite(delta) || delta > options_.boundary_tol) return false;
            }
            return true;
        }

        bool CutGraphPlanner::validateProcess(const CutSegment& segment, const ProcessCandidate& candidate,
        const SegmentMotionPlan& process)const{

            if(!candidate.entry || !candidate.exit ||
                candidate.cut.empty()||
                !candidate.cut.front()||
                !candidate.cut.back()            
            ){
                return false;
            }
            const auto approach_first = firstState(process.approach);
            const auto approach_last = lastState(process.approach);

            const auto cut_first = firstState(process.cut);
            const auto cut_last = lastState(process.cut);

            const auto retract_first = firstState(process.retract);
            const auto retract_last = lastState(process.retract);

            if(!approach_first || 
                !approach_last ||
                !cut_first||
                !cut_last||
                !retract_first||
                !retract_last                 
            ){
                return false;
            }

            if(!statesMatch(*candidate.entry,*approach_first)){
                return false;
            }

            if(!statesMatch(*approach_last,*cut_first)){
                return false;
            }

            if(!statesMatch(*cut_last,*retract_first)){
                return false;
            }
            if(!statesMatch(*retract_last,*candidate.exit)){
                return false;
            }
            if(!statesMatch(*candidate.cut.front(),*cut_first)){
                return false;
            }
            if(!statesMatch(*candidate.cut.back(),*cut_last)){
                return false;
            }
            if (!options_.tcp_link.empty()) {
                if (!robot_model_->hasLinkModel(options_.tcp_link) ||
                    !std::isfinite(segment.pos_tol) || segment.pos_tol <= 0.0) return false;
                kinematic_constraints::KinematicConstraintSet orientation_check(robot_model_);
                moveit_msgs::msg::Constraints orientation_constraints;
                orientation_constraints.orientation_constraints = segment.start_constraints.orientation_constraints;
                const moveit::core::Transforms transforms(robot_model_->getModelFrame());
                if (!orientation_constraints.orientation_constraints.empty() &&
                    !orientation_check.add(orientation_constraints, transforms)) return false;
                const auto follows_line = [&](const Plan& plan, const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
                    moveit::core::RobotState reference(robot_model_);
                    reference.setToDefaultValues();
                    moveit::core::robotStateMsgToRobotState(plan.start_state_, reference);
                    robot_trajectory::RobotTrajectory trajectory(robot_model_, planning_group_);
                    trajectory.setRobotTrajectoryMsg(reference, plan.trajectory_);
                    const Eigen::Vector3d delta = b.translation() - a.translation();
                    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
                        auto state = trajectory.getWayPoint(i);
                        state.update();
                        const Eigen::Vector3d point = state.getGlobalLinkTransform(options_.tcp_link).translation();
                        if ((i == 0 && (point - a.translation()).norm() > segment.pos_tol) ||
                            (i + 1 == trajectory.getWayPointCount() && (point - b.translation()).norm() > segment.pos_tol))
                            return false;
                        const double t = delta.squaredNorm() > 1e-16 ?
                            std::clamp(delta.dot(point - a.translation()) / delta.squaredNorm(), 0.0, 1.0) : 0.0;
                        if (!orientation_check.decide(state).satisfied || !point.allFinite() || (point - a.translation() - t * delta).norm() > segment.pos_tol)
                            return false;
                    }
                    return true;
                };
                if (!follows_line(process.approach, segment.approach_pose, segment.start_pose) ||
                    !follows_line(process.cut, segment.start_pose, segment.end_pose) ||
                    !follows_line(process.retract, segment.end_pose, segment.retract_pose)) return false;
            }
            return true;
        }
        


std::optional<ExecutablePlan> CutGraphPlanner::plan(
    const CutPlan& cut_plan,
    const std::vector<std::vector<ProcessCandidate>>& candidates_by_segment,
    const moveit::core::RobotState& home_state,
    const std::string& plan_id, const std::string& estimate_id)
{
    statistics_ = {};
    const auto count = cut_plan.segments.size();
    if (!count || candidates_by_segment.size() != count) return std::nullopt;
    CutCandidateGraph graph;
    std::unordered_map<NodeId, CandidateGraphEntry> lookup;
    std::unordered_map<NodeId, SegmentMotionPlan> processes;
    std::unordered_map<EdgeId, Plan> transits;
    const auto start = graph.addNode(0, 0.0);
    std::vector<std::vector<NodeId>> layers(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto& candidates = candidates_by_segment[i];
        for (std::size_t j = 0; j < std::min(candidates.size(), options_.max_candidates_per_segment); ++j) {
            const auto& c = candidates[j];
            if (!c.entry || !c.exit || c.cut.empty() || !c.cut.front() || !c.cut.back()) continue;
            const auto lower_distance = [&](const auto& a, const auto& b) {
                return std::max(0.0, a.distance(b, joint_model_group_) - 2.0 * options_.boundary_tol);
            };
            const double lower = lower_distance(*c.entry, *c.cut.front()) +
                lower_distance(*c.cut.front(), *c.cut.back()) + lower_distance(*c.cut.back(), *c.exit);
            const auto node = graph.addNode(2*i+1, options_.process_weight * lower);
            layers[i].push_back(node);
            lookup.emplace(node, CandidateGraphEntry{node, i, j});
        }
        if (layers[i].empty()) return std::nullopt;
    }
    const auto goal = graph.addNode(2*count+1, 0.0);
    const auto candidate = [&](NodeId n) -> const ProcessCandidate& {
        const auto& info = lookup.at(n);
        return candidates_by_segment[info.segment_index][info.candidate_index];
    };
    const auto connect = [&](NodeId from, NodeId to) {
        const auto& a = lookup.count(from) ? *candidate(from).exit : home_state;
        const auto& b = lookup.count(to) ? *candidate(to).entry : home_state;
        const double estimate = to == goal && !options_.return_home_after_last_cut ? 0.0 :
            options_.transit_weight * transitEstimate(a, b, 4.0 * options_.boundary_tol);
        graph.addEdge(from, to, estimate);
    };
    for (auto n : layers.front()) connect(start, n);
    for (std::size_t i = 1; i < count; ++i) {
        // Keep a home alternative at EVERY boundary. The home node has a
        // single exact robot configuration, so its inbound/outbound plans can
        // safely be shared across candidate pairs without changing ownership.
        const auto home = graph.addNode(2*i, 0.0);
        for (auto n : layers[i-1]) connect(n, home);
        for (auto n : layers[i]) connect(home, n);
        // Compatibility option: true forces the legacy home-only route;
        // false lets the graph compare direct versus two-leg home transits.
        if (!options_.return_home_between_cuts) {
            for (auto from : layers[i-1])
                for (auto to : layers[i]) connect(from, to);
        }
    }
    for (auto n : layers.back()) connect(n, goal);
    GraphRoute selected;
    while (true) {
        auto route = graph.cheapestRoute(start, goal);
        if (!route.found()) return std::nullopt;
        bool changed = false;
        for (auto n : route.nodes) {
            if (!lookup.count(n) || processes.count(n)) continue;
            const auto& info = lookup.at(n);
            ++statistics_.candidates_tested;
            auto process = process_planner_(cut_plan.segments[info.segment_index], candidate(n));
            if (!process || !validateProcess(cut_plan.segments[info.segment_index], candidate(n), *process)) {
                ++statistics_.process_failures;
                for (const auto& edge : graph.edges())
                    if (edge.from == n || edge.to == n) graph.edge(edge.id).status = TransitStatus::Invalid;
            } else {
                graph.node(n).process_cost = options_.process_weight *
                    (trajectoryCost(process->approach) + trajectoryCost(process->cut) + trajectoryCost(process->retract));
                processes.emplace(n, std::move(*process));
            }
            changed = true;
            break;
        }
        if (changed) continue;
        for (auto id : route.edges) {
            auto& edge = graph.edge(id);
            if (edge.status == TransitStatus::Valid) continue;
            if (edge.to == goal && !options_.return_home_after_last_cut) {
                edge.actual_cost = 0.0;
                edge.status = TransitStatus::Valid; // logical terminal, no executable motion
                changed = true;
                break;
            }
            const auto from = lookup.count(edge.from) ? lastState(processes.at(edge.from).retract) :
                std::optional<moveit::core::RobotState>(home_state);
            const auto to = lookup.count(edge.to) ? firstState(processes.at(edge.to).approach) :
                std::optional<moveit::core::RobotState>(home_state);
            auto transit = from && to ? transit_planner_(*from, *to) : std::nullopt;
            const auto first = transit ? firstState(*transit) : std::nullopt;
            const auto last = transit ? lastState(*transit) : std::nullopt;
            const double cost = transit ? transitCost(*transit) : std::numeric_limits<double>::infinity();
            if (!first || !last || !from || !to || !statesMatch(*first, *from) ||
                !statesMatch(*last, *to) || !std::isfinite(cost)) {
                edge.status = TransitStatus::Invalid;
                if (lookup.count(edge.to)) ++statistics_.transit_in_failures;
                else ++statistics_.transit_out_failures;
            } else {
                edge.actual_cost = options_.transit_weight * cost;
                edge.status = TransitStatus::Valid;
                transits.emplace(id, std::move(*transit));
            }
            changed = true;
            break;
        }
        if (changed) continue;
        selected = std::move(route);
        break;
    }
    ExecutablePlan result;
    result.plan_id = plan_id;
    result.estimate_id = estimate_id;
    for (std::size_t i = 1; i+1 < selected.nodes.size(); ++i) {
        const auto n = selected.nodes[i];
        if (!lookup.count(n)) continue;
        const auto& info = lookup.at(n);
        ExecutableCut cut;
        cut.name = cut_plan.segments[info.segment_index].name;
        // A direct candidate->candidate edge belongs ONLY to its destination.
        cut.transit_in = transits.at(selected.edges.at(i-1));
        cut.process = processes.at(n);
        const auto incoming = selected.edges.at(i-1);
        std::cout << "[CUT GRAPH] " << cut.name << " transit_in="
                  << (lookup.count(graph.edge(incoming).from) ? "direct from previous retract" : "from initial/home state")
                  << " cost=" << graph.edge(incoming).actual_cost << '\n';
        const auto outgoing = selected.edges.at(i);
        if (!lookup.count(graph.edge(outgoing).to) && transits.count(outgoing)) {
            cut.transit_out = transits.at(outgoing);
            std::cout << "[CUT GRAPH] " << cut.name << " return_home cost="
                      << graph.edge(outgoing).actual_cost << '\n';
        }
        result.cuts.push_back(std::move(cut));
        ++statistics_.accepted_candidates;
    }

    for (const auto id : selected.edges) {
            const auto& edge = graph.edge(id);

            if (edge.status != TransitStatus::Valid)
                continue;

            const auto it = transits.find(id);
            if (it == transits.end())
                continue;  // Logical terminal edge, no trajectory.

            const auto m = jointMotionMetrics(it->second);

            std::cout
                << "[FINAL TRANSIT] edge=" << id
                << " total=" << m.total_travel
                << " max_joint=" << m.maximum_joint_travel
                << " cost=" << edge.actual_cost
                << '\n';
        }

    if (result.cuts.size() != count) return std::nullopt;
    return result;
}

} // namespace hb_robot_skills::motion
