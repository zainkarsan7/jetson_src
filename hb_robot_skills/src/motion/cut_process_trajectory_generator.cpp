
#include "hb_robot_skills/motion/cut_process_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <random>

namespace hb_robot_skills::motion
{
namespace
{

Eigen::Isometry3d rotatedAboutLocalZ(
    const Eigen::Isometry3d& pose,
    double angle)
{
    Eigen::Isometry3d result = pose;

    result.linear() =
        pose.linear() *
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    return result;
}

Eigen::Isometry3d interpolatePose(
    const Eigen::Isometry3d& a,
    const Eigen::Isometry3d& b,
    double t)
{
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();

    result.translation() =
        (1.0 - t) * a.translation() + t * b.translation();

    const Eigen::Quaterniond qa(a.linear());
    const Eigen::Quaterniond qb(b.linear());

    result.linear() = qa.slerp(t, qb).toRotationMatrix();

    return result;
}

double orientationError(
    const Eigen::Matrix3d& actual,
    const Eigen::Matrix3d& target)
{
    const Eigen::Matrix3d relative = target.transpose() * actual;

    return std::abs(Eigen::AngleAxisd(relative).angle());
}

}  // namespace

CutProcessTrajectoryGenerator::CutProcessTrajectoryGenerator(
    moveit::core::RobotModelConstPtr robot_model,
    std::string planning_group,
    std::string plasma_link,
    ProcessGeneratorOptions options)
    : robot_model_(std::move(robot_model)),
      planning_group_(std::move(planning_group)),
      plasma_link_(std::move(plasma_link)),
      options_(std::move(options))
{
    if (!robot_model_)
        throw std::invalid_argument("Null robot model");

    joint_model_group_ =
        robot_model_->getJointModelGroup(planning_group_);

    if (!joint_model_group_)
        throw std::invalid_argument("Unknown planning group");

    if (!robot_model_->hasLinkModel(plasma_link_))
        throw std::invalid_argument("Unknown plasma link");

    if (options_.roll_samples == 0 ||
        options_.seeds_per_roll == 0 ||
        options_.max_candidates == 0 ||
        options_.max_translation_step_m <= 0.0 ||
        options_.max_rotation_step_rad <= 0.0 ||
        options_.ik_timeout_s <= 0.0 ||
        options_.max_joint_step_rad <= 0.0 ||
        options_.position_tolerance_m <= 0.0 ||
        options_.orientation_tolerance_rad <= 0.0 ||
        options_.duplicate_joint_distance < 0.0 ||
        options_.max_roll_rad < options_.min_roll_rad ||
        options_.global_seeds_per_roll >= options_.seeds_per_roll ||
        !std::isfinite(options_.local_seed_radius_rad) || options_.local_seed_radius_rad < 0.0 ||
        !std::isfinite(options_.local_ik_consistency_rad) || options_.local_ik_consistency_rad <= 0.0 ||
        !std::isfinite(options_.retention_transit_weight) || options_.retention_transit_weight < 0.0 ||
        !std::isfinite(options_.retention_process_weight) || options_.retention_process_weight < 0.0 ||
        !std::isfinite(options_.preferred_joint_travel_rad) || options_.preferred_joint_travel_rad < 0.0 ||
        !std::isfinite(options_.excess_travel_weight) || options_.excess_travel_weight < 0.0)
    {
        throw std::invalid_argument(
            "Invalid process generator options");
    }
}

void CutProcessTrajectoryGenerator::appendInterpolated(
    std::vector<Eigen::Isometry3d>& output,
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end) const
{
    const double distance =
        (end.translation() - start.translation()).norm();

    const double rotation =
        orientationError(start.linear(), end.linear());

    const auto translation_steps =
        static_cast<std::size_t>(
            std::ceil(distance / options_.max_translation_step_m));

    const auto rotation_steps =
        static_cast<std::size_t>(
            std::ceil(rotation / options_.max_rotation_step_rad));

    const std::size_t steps =
        std::max<std::size_t>(
            1, std::max(translation_steps, rotation_steps));

    // Caller has already inserted start.
    for (std::size_t i = 1; i <= steps; ++i)
    {
        const double t =
            static_cast<double>(i) / static_cast<double>(steps);

        output.push_back(interpolatePose(start, end, t));
    }
}

CutProcessTrajectoryGenerator::CartesianPath
CutProcessTrajectoryGenerator::buildCartesianPath(
    const CutSegment& segment,
    double roll_rad) const
{
    CartesianPath path;

    const Eigen::Isometry3d approach =
        rotatedAboutLocalZ(segment.approach_pose, roll_rad);

    const Eigen::Isometry3d start =
        rotatedAboutLocalZ(segment.start_pose, roll_rad);

    const Eigen::Isometry3d end =
        rotatedAboutLocalZ(segment.end_pose, roll_rad);

    const Eigen::Isometry3d retract =
        rotatedAboutLocalZ(segment.retract_pose, roll_rad);

    path.poses.push_back(approach);

    appendInterpolated(path.poses, approach, start);
    path.approach_end = path.poses.size() - 1;

    appendInterpolated(path.poses, start, end);
    path.cut_end = path.poses.size() - 1;

    appendInterpolated(path.poses, end, retract);

    return path;
}

bool CutProcessTrajectoryGenerator::validState(
    const moveit::core::RobotState& state,
    const planning_scene::PlanningSceneConstPtr& scene,
    Failure& failure) const
{
    if (!state.satisfiesBounds(joint_model_group_))
    {
        failure = Failure::JointLimits;
        return false;
    }

    // Uses the planning scene's collision environment and ACM.
    if (scene->isStateColliding(state, planning_group_, false))
    {
        failure = Failure::Collision;
        return false;
    }

    failure = Failure::None;
    return true;
}

bool CutProcessTrajectoryGenerator::closeToTarget(
    const moveit::core::RobotState& state,
    const Eigen::Isometry3d& target) const
{
    const Eigen::Isometry3d& actual =
        state.getGlobalLinkTransform(plasma_link_);

    const double position_error =
        (actual.translation() - target.translation()).norm();

    const double angular_error =
        orientationError(actual.linear(), target.linear());

    return position_error <= options_.position_tolerance_m &&
           angular_error <= options_.orientation_tolerance_rad;
}

double CutProcessTrajectoryGenerator::jointDistance(
    const moveit::core::RobotState& a,
    const moveit::core::RobotState& b) const
{
    // MoveIt computes joint-model-aware distances, including
    // wraparound for continuous revolute joints.
    return a.distance(b, joint_model_group_);
}

bool CutProcessTrajectoryGenerator::jointStepValid(
    const moveit::core::RobotState& a,
    const moveit::core::RobotState& b) const
{
    const auto& joints = joint_model_group_->getActiveJointModels();

    for (const auto* joint : joints)
    {
        const double distance = joint->distance(
            a.getJointPositions(joint),
            b.getJointPositions(joint));

        if (!std::isfinite(distance) ||
            distance > options_.max_joint_step_rad)
        {
            return false;
        }
    }

    return true;
}

CutProcessTrajectoryGenerator::SolveResult
CutProcessTrajectoryGenerator::solvePath(
    const CartesianPath& path,
    const moveit::core::RobotState& seed,
    const planning_scene::PlanningSceneConstPtr& scene,
    bool backward,
    bool constrain_initial_ik) const
{
    SolveResult result;

    if (path.poses.empty())
    {
        result.failure = Failure::IK;
        return result;
    }

    moveit::core::RobotState current(seed);
    current.update();

    const std::size_t count = path.poses.size();

    // Solve in either direction. Backward search is only a
    // numerical IK strategy, not a reversed cutting operation.
    for (std::size_t k = 0; k < count; ++k)
    {
        const std::size_t index =
            backward ? count - 1 - k : k;

        const Eigen::Isometry3d& target = path.poses[index];

        moveit::core::RobotState next(current);

        // The previous solved configuration is the IK seed.
        // We check branch continuity explicitly afterward.
        // Continuation must stay on the current branch. Geometric/local seeds
        // also constrain the first solve; reference/global seeds can discover
        // a distant reachable branch, which is subsequently ranked by transit.
        const std::vector<double> consistency_limits(
            joint_model_group_->getVariableCount(),
            k == 0 ? options_.local_ik_consistency_rad : options_.max_joint_step_rad);
        const bool ik_ok = (k > 0 || constrain_initial_ik) ? next.setFromIK(
            joint_model_group_, target, plasma_link_, consistency_limits,
            options_.ik_timeout_s) : next.setFromIK(
            joint_model_group_,
            target,
            plasma_link_,
            options_.ik_timeout_s);

        if (!ik_ok)
        {
            result.failure = Failure::IK;
            result.states.clear();
            return result;
        }

        next.update();

        if (!closeToTarget(next, target))
        {
            result.failure = Failure::TCPError;
            result.states.clear();
            return result;
        }

        Failure validation_failure = Failure::None;

        if (!validState(next, scene, validation_failure))
        {
            result.failure = validation_failure;
            result.states.clear();
            return result;
        }

        if (!result.states.empty() &&
            !jointStepValid(current, next))
        {
            result.failure = Failure::JointJump;
            result.states.clear();
            return result;
        }

        result.states.push_back(
            std::make_shared<moveit::core::RobotState>(next));

        current = next;
    }

    if (backward)
        std::reverse(result.states.begin(), result.states.end());

    result.failure = Failure::None;
    return result;
}

double CutProcessTrajectoryGenerator::pathCost(
    const StatePath& states) const
{
    double cost = 0.0;

    for (std::size_t i = 1; i < states.size(); ++i)
    {
        cost += jointDistance(*states[i - 1], *states[i]);
    }

    return cost;
}

ProcessCandidate CutProcessTrajectoryGenerator::makeCandidate(
    const CutSegment& segment,
    const CartesianPath& path,
    StatePath states,
    double roll,
    std::size_t seed_index,
    bool backward) const
{
    ProcessCandidate candidate;

    candidate.segment_name = segment.name;
    candidate.torch_roll_rad = roll;
    candidate.seed_index = seed_index;
    candidate.solved_backward = backward;

    candidate.score = pathCost(states);

    candidate.entry = states.front();
    candidate.exit = states.back();

    // approach: [0, approach_end]
    candidate.approach.assign(
        states.begin(),
        states.begin() + path.approach_end + 1);

    // cut: [approach_end, cut_end]
    candidate.cut.assign(
        states.begin() + path.approach_end,
        states.begin() + path.cut_end + 1);

    // retract: [cut_end, final]
    candidate.retract.assign(
        states.begin() + path.cut_end,
        states.end());

    return candidate;
}

bool CutProcessTrajectoryGenerator::isDuplicate(
    const ProcessCandidate& candidate,
    const std::vector<ProcessCandidate>& existing) const
{
    for (const auto& other : existing)
    {
        // Cheap preliminary check. Both endpoints must match.
        if (jointDistance(*candidate.entry, *other.entry) >
            options_.duplicate_joint_distance)
            continue;

        if (jointDistance(*candidate.exit, *other.exit) >
            options_.duplicate_joint_distance)
            continue;

        // Compare the whole cut, not just the endpoints.
        if (candidate.cut.size() != other.cut.size())
            continue;

        double maximum_difference = 0.0;

        for (std::size_t i = 0; i < candidate.cut.size(); ++i)
        {
            maximum_difference = std::max(
                maximum_difference,
                jointDistance(*candidate.cut[i], *other.cut[i]));
        }

        if (maximum_difference <=
            options_.duplicate_joint_distance)
        {
            return true;
        }
    }

    return false;
}

ProcessGenerationResult CutProcessTrajectoryGenerator::generate(
    const CutSegment& segment,
    const moveit::core::RobotState& reference_state,
    const planning_scene::PlanningSceneConstPtr& scene) const
{
    if (!scene)
        throw std::invalid_argument("Null planning scene");

    ProcessGenerationResult result;

    const std::size_t direction_count =
        options_.enable_reverse_ik_search ? 2 : 1;

    // Preserve the useful orientation found by the expensive geometric solve.
    // This is only a seed: the complete Cartesian path is still revalidated.
    std::vector<double> rolls;
    const auto add_roll = [&](double roll) {
        if (roll < options_.min_roll_rad || roll > options_.max_roll_rad) return;
        for (double existing : rolls)
            if (std::abs(std::remainder(roll - existing, 2.0 * std::acos(-1.0))) < 1e-6) return;
        rolls.push_back(roll);
    };
    if (segment.start_state) {
        const Eigen::Matrix3d relative = segment.start_pose.linear().transpose() *
            segment.start_state->getGlobalLinkTransform(plasma_link_).linear();
        add_roll(std::atan2(relative(1, 0), relative(0, 0)));
    }
    add_roll(0.0);
    for (std::size_t r = 0; r < options_.roll_samples; ++r)
    {
        double roll = options_.min_roll_rad;

        if (options_.roll_samples > 1)
        {
            const double fraction =
                static_cast<double>(r) /
                static_cast<double>(options_.roll_samples - 1);

            roll += fraction *
                (options_.max_roll_rad - options_.min_roll_rad);
        }

        add_roll(roll);
    }
    std::mt19937 rng(0); // Reproducible bounded perturbations for tuning.
    std::uniform_real_distribution<double> jitter(
        -options_.local_seed_radius_rad, options_.local_seed_radius_rad);
    for (double roll : rolls) {
        const CartesianPath path =
            buildCartesianPath(segment, roll);

        for (std::size_t seed_index = 0;
             seed_index < options_.seeds_per_roll;
             ++seed_index)
        {
            for (std::size_t d = 0; d < direction_count; ++d)
            {
                const bool backward = d == 1;
                const auto geometric = backward ? segment.retract_state : segment.approach_state;
                const bool global = seed_index >= options_.seeds_per_roll - options_.global_seeds_per_roll;
                const bool use_geometric = geometric && seed_index != 1 && !global;
                moveit::core::RobotState seed(use_geometric ? *geometric : reference_state);
                if (global) {
                    seed.setToRandomPositions(joint_model_group_);
                } else if (seed_index > 1) {
                    std::vector<double> positions;
                    seed.copyJointGroupPositions(joint_model_group_, positions);
                    for (double& position : positions) position += jitter(rng);
                    seed.setJointGroupPositions(joint_model_group_, positions);
                    seed.enforceBounds(joint_model_group_);
                }
                seed.update();

                ++result.stats.attempts;

                SolveResult solved =
                    solvePath(path, seed, scene, backward, use_geometric || (!global && seed_index > 1));

                if (solved.failure != Failure::None)
                {
                    switch (solved.failure)
                    {
                        case Failure::IK:
                            ++result.stats.ik_failures;
                            break;

                        case Failure::Collision:
                            ++result.stats.collision_failures;
                            break;

                        case Failure::JointLimits:
                            ++result.stats.joint_limit_failures;
                            break;

                        case Failure::JointJump:
                            ++result.stats.joint_jump_failures;
                            break;

                        case Failure::TCPError:
                            ++result.stats.tcp_error_failures;
                            break;

                        case Failure::None:
                            break;
                    }

                    continue;
                }

                ProcessCandidate candidate = makeCandidate(
                    segment,
                    path,
                    std::move(solved.states),
                    roll,
                    seed_index,
                    backward);

                if (isDuplicate(candidate, result.candidates))
                {
                    ++result.stats.duplicate_candidates;
                    continue;
                }

                result.candidates.push_back(std::move(candidate));
            }
        }
    }

    // Match the graph's transit objective before truncating the candidate pool.
    const auto transit_rank = [&](const moveit::core::RobotState& state) {
        double total = 0.0, maximum = 0.0;
        for (const auto* joint : joint_model_group_->getActiveJointModels()) {
            if (joint->getVariableCount() != 1) continue;
            const auto& name = joint->getVariableNames().front();
            const double travel = std::abs(state.getVariablePosition(name) - reference_state.getVariablePosition(name));
            total += travel;
            maximum = std::max(maximum, travel);
        }
        const double excess = std::max(0.0, maximum - options_.preferred_joint_travel_rad);
        return total + 2.0 * maximum + options_.excess_travel_weight * excess * excess;
    };
    std::stable_sort(
        result.candidates.begin(),
        result.candidates.end(),
        [&](const ProcessCandidate& a,
           const ProcessCandidate& b)
        {
            const auto rank = [&](const ProcessCandidate& candidate) {
                return options_.retention_process_weight * candidate.score +
                    options_.retention_transit_weight *
                    (transit_rank(*candidate.entry) + transit_rank(*candidate.exit));
            };
            return rank(a) < rank(b);
        });

    if (result.candidates.size() > options_.max_candidates)
    {
        // Reserve a quarter of the budget for distinct configurations.
        const std::size_t best_count = options_.max_candidates - options_.max_candidates / 4;
        std::vector<ProcessCandidate> retained;
        std::vector<bool> selected(result.candidates.size(), false);
        for (std::size_t i = 0; i < best_count; ++i) {
            retained.push_back(result.candidates[i]);
            selected[i] = true;
        }
        for (std::size_t i = best_count; i < result.candidates.size() &&
             retained.size() < options_.max_candidates; ++i) {
            const auto& candidate = result.candidates[i];
            const bool distinct = std::all_of(retained.begin(), retained.end(),
                [&](const ProcessCandidate& other) {
                    return std::max(jointDistance(*candidate.entry, *other.entry),
                                    jointDistance(*candidate.exit, *other.exit)) > 0.75;
                });
            if (distinct) {
                retained.push_back(candidate);
                selected[i] = true;
            }
        }
        for (std::size_t i = best_count; i < result.candidates.size() &&
             retained.size() < options_.max_candidates; ++i)
            if (!selected[i]) retained.push_back(result.candidates[i]);
        result.candidates = std::move(retained);
    }

    result.stats.accepted = result.candidates.size();

    return result;
}

}  // namespace hb_robot_skills::motion
