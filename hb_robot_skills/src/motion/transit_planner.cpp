#include "hb_robot_skills/motion/transit_planner.hpp"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace hb_robot_skills::motion {

std::optional<MoveGroup::Plan> planGuidedPTP(
    MoveGroup& group,
    const rclcpp::Logger& logger,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& target_state,
    const TransitPlannerOptions& options)
{
    using State = moveit::core::RobotState;
    using Plan = MoveGroup::Plan;
    using Trajectory = robot_trajectory::RobotTrajectory;
    using Clock = std::chrono::steady_clock;

    const auto started = Clock::now();

    const auto model = group.getRobotModel();
    if (!model || !options.scene)
        return std::nullopt;

    const std::string planning_group = group.getName();
    const auto* jmg = model->getJointModelGroup(planning_group);

    if (!jmg ||
        !model->hasLinkModel(options.guide_link) ||
        options.guided_max_waypoints == 0 ||
        options.guided_max_ik_calls == 0 ||
        options.guided_max_ptp_calls == 0 ||
        options.guided_check_step_rad <= 0.0 ||
        options.guided_joint_consistency_rad <= 0.0 ||
        options.guided_ik_timeout_s <= 0.0 ||
        options.guided_ptp_time_s <= 0.0 ||
        options.guided_lift_m <= 0.0 ||
        options.guided_lateral_m < 0.0 ||
        options.guided_position_tolerance_m < 0.0 ||
        options.guided_orientation_tolerance_rad < 0.0)
    {
        return std::nullopt;
    }

    const auto* link = model->getLinkModel(options.guide_link);
    const std::size_t dof = jmg->getVariableCount();

    // Keep this implementation focused on the usual
    // one-variable-per-joint manipulator configuration.
    if (dof == 0)
        return std::nullopt;

    unsigned ik_calls = 0;
    unsigned ptp_calls = 0;
    unsigned accepted_waypoints = 0;

    // --------------------------------------------------
    // 1. Small local helpers
    // --------------------------------------------------

    const auto jointPositions =
        [&](const State& state)
    {
        std::vector<double> q;
        state.copyJointGroupPositions(jmg, q);
        return q;
    };

    const auto jointDifference =
        [&](const State& a, const State& b)
    {
        const auto qa = jointPositions(a);
        const auto qb = jointPositions(b);

        double maximum = 0.0;

        for (std::size_t i = 0; i < qa.size(); ++i)
        {
            // For bounded revolute joints this is the
            // actual variable difference. Continuous-joint
            // wraparound needs separate handling.
            maximum = std::max(
                maximum, std::abs(qa[i] - qb[i]));
        }

        return maximum;
    };

    const auto nearHome =
        [&](const State& state) -> bool
    {
        if (options.home_joints.size() != dof)
            return false;

        const auto q = jointPositions(state);

        for (std::size_t i = 0; i < dof; ++i)
        {
            if (!std::isfinite(q[i]) ||
                !std::isfinite(options.home_joints[i]) ||
                std::abs(q[i] - options.home_joints[i]) >
                    options.home_tolerance_rad)
            {
                return false;
            }
        }

        return true;
    };

    const auto tcpPose =
        [&](const State& state)
    {
        State copy(state);
        copy.update();
        return copy.getGlobalLinkTransform(link);
    };

    // The planning scene is the collision authority.
    // This function does not alter its allowed-collision matrix.
    const auto validState =
        [&](const State& state) -> bool
    {
        return state.satisfiesBounds(jmg) &&
               !options.scene->isStateColliding(
                   state, planning_group);
    };

    // Additional interpolation check, including states
    // between the waypoints returned by Pilz.
    const auto validEdge =
        [&](const State& a, const State& b) -> bool
    {
        if (!validState(a) || !validState(b))
            return false;

        const double distance = jointDifference(a, b);

        if (!std::isfinite(distance))
            return false;

        const unsigned steps = std::max(
            1u,
            static_cast<unsigned>(
                std::ceil(
                    distance /
                    options.guided_check_step_rad)));

        State interpolated(a);

        for (unsigned i = 1; i < steps; ++i)
        {
            const double t =
                static_cast<double>(i) / steps;

            a.interpolate(b, t, interpolated);
            interpolated.update();

            if (!validState(interpolated))
                return false;
        }

        return true;
    };

    const auto validTrajectory =
        [&](const Plan& plan,
            const State& expected_from,
            const State& expected_to) -> bool
    {
        Trajectory trajectory(model, planning_group);

        trajectory.setRobotTrajectoryMsg(
            expected_from, plan.trajectory_);

        if (trajectory.getWayPointCount() < 2)
            return false;

        if (jointDifference(
                trajectory.getFirstWayPoint(),
                expected_from) > 1e-3 ||
            jointDifference(
                trajectory.getLastWayPoint(),
                expected_to) > 1e-3)
        {
            return false;
        }

        for (std::size_t i = 0;
             i < trajectory.getWayPointCount(); ++i)
        {
            if (!validState(trajectory.getWayPoint(i)))
                return false;

            if (i > 0 &&
                !validEdge(
                    trajectory.getWayPoint(i - 1),
                    trajectory.getWayPoint(i)))
            {
                return false;
            }
        }

        return true;
    };

    const auto tryPTP =
        [&](const State& from,
            const State& to) -> std::optional<Plan>
    {
        if (ptp_calls >= options.guided_max_ptp_calls)
            return std::nullopt;

        ++ptp_calls;

        group.clearPoseTargets();
        group.clearPathConstraints();

        group.setStartState(from);

        if (!group.setJointValueTarget(to))
            return std::nullopt;

        group.setPlanningPipelineId(
            "pilz_industrial_motion_planner");

        group.setPlannerId("PTP");
        group.setPlanningTime(options.guided_ptp_time_s);
        group.setNumPlanningAttempts(1);

        group.setMaxVelocityScalingFactor(
            options.velocity_scaling);

        group.setMaxAccelerationScalingFactor(
            options.acceleration_scaling);

        Plan plan;

        if (group.plan(plan) !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            return std::nullopt;
        }

        if (!validTrajectory(plan, from, to))
            return std::nullopt;

        return plan;
    };

    // --------------------------------------------------
    // 2. Seed-constrained, loosely sampled IK
    // --------------------------------------------------

    const auto solveIK =
    [&](const State& seed,
        const Eigen::Isometry3d& nominal)
        -> std::optional<State>
{
    const std::array<Eigen::Vector3d, 7> offsets{{
        { 0.0,  0.0,  0.0},
        { 1.0,  0.0,  0.0},
        {-1.0,  0.0,  0.0},
        { 0.0,  1.0,  0.0},
        { 0.0, -1.0,  0.0},
        { 0.0,  0.0,  1.0},
        { 0.0,  0.0, -1.0}
    }};

    const std::array<Eigen::Vector3d, 7> rotations{{
        { 0.0,  0.0,  0.0},
        { 1.0,  0.0,  0.0},
        {-1.0,  0.0,  0.0},
        { 0.0,  1.0,  0.0},
        { 0.0, -1.0,  0.0},
        { 0.0,  0.0,  1.0},
        { 0.0,  0.0, -1.0}
    }};

    // Per-joint limits if provided, otherwise uniform.
    std::vector<double> consistency(
        dof, options.guided_joint_consistency_rad);

    if (!options.guided_joint_limits_rad.empty())
    {
        if (options.guided_joint_limits_rad.size() != dof)
            return std::nullopt;

        consistency = options.guided_joint_limits_rad;
    }

    for (double limit : consistency)
    {
        if (!std::isfinite(limit) || limit <= 0.0)
            return std::nullopt;
    }

    constexpr unsigned attempts_per_pose = 13;

    for (unsigned k = 0;
         k < attempts_per_pose &&
         ik_calls < options.guided_max_ik_calls;
         ++k)
    {
        Eigen::Isometry3d pose = nominal;

        if (k > 0 && k <= 6)
        {
            const auto& axis = rotations[k];

            pose.linear() =
                nominal.linear() *
                Eigen::AngleAxisd(
                    options.guided_orientation_tolerance_rad,
                    axis).toRotationMatrix();
        }
        else if (k > 6)
        {
            pose.translation() +=
                options.guided_position_tolerance_m *
                offsets[k - 6];
        }

        State candidate(seed);

        ++ik_calls;

        // MoveIt still performs all the IK solving.
        if (!candidate.setFromIK(
                jmg,
                pose,
                options.guide_link,
                consistency,
                options.guided_ik_timeout_s))
        {
            continue;
        }

        candidate.update();

        // Independently enforce the seed-relative
        // joint displacement limits.
        const auto seed_q = jointPositions(seed);
        const auto candidate_q = jointPositions(candidate);

        bool acceptable = true;

        for (std::size_t j = 0; j < dof; ++j)
        {
            const double delta =
                std::abs(candidate_q[j] - seed_q[j]);

            if (!std::isfinite(delta) ||
                delta > consistency[j] + 1e-4)
            {
                acceptable = false;
                break;
            }
        }

        if (!acceptable)
            continue;

        if (!validState(candidate))
            continue;

        return candidate;
    }

    return std::nullopt;
};

    // --------------------------------------------------
    // 3. Endpoint-aware lifted-midpoint sampling
    // --------------------------------------------------

    const bool from_home = nearHome(start_state);
    const bool to_home = nearHome(target_state);

    if (from_home && to_home)
        return std::nullopt;

    if (!validState(start_state) ||
        !validState(target_state))
    {
        RCLCPP_WARN(
            logger,
            "[GUIDED PTP] Invalid start or target state");
        return std::nullopt;
    }

    // The endpoint directions are recomputed as the
    // greedy frontiers advance.
    //
    // The home rule uses the non-home TCP's orientation.
    // The +/- convention must match the actual tool frame.
    const auto sampleBridge =
        [&](const State& from,
            const State& to,
            unsigned sample_index)
            -> Eigen::Vector3d
    {
        const auto A = tcpPose(from);
        const auto B = tcpPose(to);

        Eigen::Vector3d da =
            A.linear().col(2).normalized();

        Eigen::Vector3d db =
            B.linear().col(2).normalized();

        if (from_home)
            da = -db;

        if (to_home)
            db = -da;

        // Try the central midpoint first, then bias
        // slightly toward either endpoint.
        constexpr std::array<double, 3> alphas{
            0.5, 0.3, 0.7
        };

        constexpr std::array<double, 3> lift_scales{
            1.0, 1.5, 2.0
        };

        constexpr std::array<double, 3> lateral_scales{
            0.0, 1.0, -1.0
        };

        const double alpha =
            alphas[sample_index % alphas.size()];

        const double lift =
            options.guided_lift_m *
            lift_scales[
                (sample_index / alphas.size()) %
                lift_scales.size()];

        const Eigen::Vector3d lifted_a =
            A.translation() + lift * da;

        const Eigen::Vector3d lifted_b =
            B.translation() + lift * db;

        Eigen::Vector3d sample =
            (1.0 - alpha) * lifted_a +
            alpha * lifted_b;

        // Lateral offset is perpendicular to the bridge.
        Eigen::Vector3d bridge =
            B.translation() - A.translation();

        Eigen::Vector3d outward = da + db;

        if (outward.norm() < 1e-6)
            outward = da;

        Eigen::Vector3d lateral =
            bridge.cross(outward);

        if (lateral.norm() > 1e-6)
        {
            lateral.normalize();

            const double scale =
                lateral_scales[
                    (sample_index /
                     (alphas.size() * lift_scales.size())) %
                    lateral_scales.size()];

            sample +=
                scale * options.guided_lateral_m * lateral;
        }

        return sample;
    };

    // --------------------------------------------------
    // 4. Greedy bidirectional growth
    // --------------------------------------------------

    State front(start_state);
    State back(target_state);

    // Segments in travel order:
    // start -> front
    // back  -> target
    std::deque<Plan> front_legs;
    std::deque<Plan> back_legs;

    // A successful leg is kept only when the corresponding
    // frontier is advanced.
    //
    // The next iteration tries to connect the two frontiers.
    std::optional<Plan> bridge_leg;

    unsigned sample_index = 0;
    unsigned stagnant_samples = 0;

    const auto remainingBudget = [&]() {
        return ik_calls < options.guided_max_ik_calls &&
               ptp_calls < options.guided_max_ptp_calls;
    };

    while (remainingBudget())
{
    // First try connecting the existing frontiers.
    if (auto direct = tryPTP(front, back))
    {
        bridge_leg = std::move(direct);
        break;
    }

    if (accepted_waypoints >=
        options.guided_max_waypoints)
    {
        break;
    }

    const Eigen::Vector3d position =
        sampleBridge(front, back, sample_index++);

    const bool prefer_front =
        accepted_waypoints % 2 == 0;

    bool advanced = false;

    for (unsigned side = 0; side < 2; ++side)
    {
        const bool grow_front =
            side == 0 ? prefer_front : !prefer_front;

        const State& seed =
            grow_front ? front : back;

        Eigen::Isometry3d pose = tcpPose(seed);
        pose.translation() = position;

        auto middle = solveIK(seed, pose);

        if (!middle)
            continue;

        const double before =
            jointDifference(front, back);

        const double from_distance =
            jointDifference(*middle, front);

        const double to_distance =
            jointDifference(*middle, back);

        if (from_distance < 0.03 ||
            to_distance < 0.03)
        {
            continue;
        }

        // Try both connections before advancing.
        auto left = tryPTP(front, *middle);
        auto right = tryPTP(*middle, back);

        // Both legs work: complete the path.
        if (left && right)
        {
            front_legs.push_back(std::move(*left));
            bridge_leg = std::move(*right);
            break;
        }

        constexpr double max_regression_rad = 0.15;

        bool grew_front = false;

        if (left &&
            to_distance <= before + max_regression_rad)
        {
            front_legs.push_back(std::move(*left));
            front = std::move(*middle);

            ++accepted_waypoints;
            advanced = true;
            grew_front = true;
        }
        else if (right &&
                 from_distance <= before + max_regression_rad)
        {
            back_legs.push_front(std::move(*right));
            back = std::move(*middle);

            ++accepted_waypoints;
            advanced = true;
            grew_front = false;
        }

        if (!advanced)
            continue;

        RCLCPP_INFO(
            logger,
            "[GUIDED PTP] advanced=%s waypoints=%u "
            "IK=%u PTP=%u remaining_joint=%.3f",
            grew_front ? "front" : "back",
            accepted_waypoints,
            ik_calls,
            ptp_calls,
            jointDifference(front, back));

        break;
    }

    // The inner break above only exits the for loop.
    if (bridge_leg)
        break;

    if (advanced)
    {
        stagnant_samples = 0;
    }
    else
    {
        ++stagnant_samples;

        if (stagnant_samples >= 12)
            break;
    }
}
    if (!bridge_leg)
    {
        RCLCPP_INFO(
            logger,
            "[GUIDED PTP] failed waypoints=%u "
            "IK=%u PTP=%u",
            accepted_waypoints,
            ik_calls,
            ptp_calls);

        return std::nullopt;
    }

    // --------------------------------------------------
    // 5. Assemble the successful PTP segments
    // --------------------------------------------------

    std::vector<Plan> segments;
    segments.reserve(
        front_legs.size() + 1 + back_legs.size());

    for (const auto& leg : front_legs)
        segments.push_back(leg);

    segments.push_back(*bridge_leg);

    for (const auto& leg : back_legs)
        segments.push_back(leg);

    Trajectory combined(model, planning_group);

    State segment_start(start_state);

    for (const auto& segment : segments)
    {
        Trajectory part(model, planning_group);

        part.setRobotTrajectoryMsg(
            segment_start, segment.trajectory_);

        if (part.getWayPointCount() < 2)
            return std::nullopt;

        for (std::size_t i = 0;
             i < part.getWayPointCount(); ++i)
        {
            // Avoid duplicating the shared endpoint.
            if (!combined.empty() && i == 0)
                continue;

            combined.addSuffixWayPoint(
                part.getWayPoint(i), 0.0);
        }

        segment_start = part.getLastWayPoint();
    }

    if (combined.getWayPointCount() < 2)
        return std::nullopt;

    // Check exact requested endpoints.
    if (jointDifference(
            combined.getFirstWayPoint(),
            start_state) > 1e-3 ||
        jointDifference(
            combined.getLastWayPoint(),
            target_state) > 1e-3)
    {
        return std::nullopt;
    }

    // Recheck the entire combined trajectory.
    for (std::size_t i = 0;
         i < combined.getWayPointCount(); ++i)
    {
        if (!validState(combined.getWayPoint(i)))
            return std::nullopt;

        if (i > 0 &&
            !validEdge(
                combined.getWayPoint(i - 1),
                combined.getWayPoint(i)))
        {
            return std::nullopt;
        }
    }

    // --------------------------------------------------
    // 6. Time parameterization
    // --------------------------------------------------

    trajectory_processing::IterativeParabolicTimeParameterization
        time_parameterization;

    if (!time_parameterization.computeTimeStamps(
            combined,
            options.velocity_scaling,
            options.acceleration_scaling))
    {
        RCLCPP_WARN(
            logger,
            "[GUIDED PTP] Time parameterization failed");

        return std::nullopt;
    }

    Plan result;

    moveit::core::robotStateToRobotStateMsg(
        start_state, result.start_state_);

    combined.getRobotTrajectoryMsg(result.trajectory_);

    result.planning_time_ =
        std::chrono::duration<double>(
            Clock::now() - started).count();

    RCLCPP_INFO(
        logger,
        "[GUIDED PTP] SUCCESS waypoints=%u "
        "segments=%zu IK=%u PTP=%u "
        "trajectory_points=%zu time=%.3fs",
        accepted_waypoints,
        segments.size(),
        ik_calls,
        ptp_calls,
        combined.getWayPointCount(),
        result.planning_time_);

    return result;
}
    

std::optional<MoveGroup::Plan> planTransit(
    MoveGroup& group, const rclcpp::Logger& logger,
    const moveit::core::RobotState& start_state, const moveit::core::RobotState& target_state,
    const TransitPlannerOptions& options){

         using Plan = MoveGroup::Plan;

    struct Metrics {
        double total = 0.0;
        double max_joint = 0.0;
        double max_step = 0.0;
        double cost = std::numeric_limits<double>::infinity();
    };


    struct PlannerChoice {
        const char* pipeline;
        const char* planner;
        double time_s;
        int planning_attempts;
    };

   constexpr PlannerChoice fallback[] = {
        {"ompl", "RRTConnectkConfigDefault", 2.0, 1},
        {"ompl", "BiTRRTkConfigDefault",    3.0, 3},
        {"ompl", "APSConfigDefault",        5.0, 1},
        {"ompl", "LBKPIECEkConfigDefault",  3.0, 2},
        {"ompl", "LazyPRMkConfigDefault",   3.0, 2},
        {"ompl", "RRTstarkConfigDefault",   5.0, 2},
    };

    // Match CutGraphPlanner::transitCost().
    constexpr double preferred_travel = 1.0;
    constexpr double excess_weight = 8.0;

    // Quality triggers, not hard safety constraints.
    constexpr double good_cost = 75.0;
    constexpr double good_max_joint =3.5;


    const auto score = [&](const Plan& plan)
        -> std::optional<Metrics>
    {
        const auto& jt = plan.trajectory_.joint_trajectory;

        if (jt.points.empty() || jt.joint_names.empty())
            return std::nullopt;

        const auto& first = jt.points.front().positions;
        const auto& last = jt.points.back().positions;

        if (first.size() != jt.joint_names.size() ||
            last.size() != jt.joint_names.size())
            return std::nullopt;

        Metrics m;

        for (std::size_t j = 0; j < jt.joint_names.size(); ++j) {
            const auto& name = jt.joint_names[j];
            const auto& variables = start_state.getVariableNames();
            if (std::find(variables.begin(), variables.end(), name) == variables.end())
                return std::nullopt;

            // Reject a trajectory whose actual endpoints do not
            // match the requested joint configurations.
            const double endpoint_tol = 1e-3;

            const double q_start = start_state.getVariablePosition(name);
            const double q_goal  = target_state.getVariablePosition(name);

            if (!std::isfinite(first[j]) ||
                !std::isfinite(last[j]) ||
                !std::isfinite(q_start) ||
                !std::isfinite(q_goal) ||
                std::abs(first[j] - q_start) > endpoint_tol ||
                std::abs(last[j] - q_goal) > endpoint_tol)
            {
                return std::nullopt;
            }

            double travel = 0.0;

            for (std::size_t i = 1; i < jt.points.size(); ++i) {
                const auto& prev = jt.points[i - 1].positions;
                const auto& curr = jt.points[i].positions;

                if (prev.size() != jt.joint_names.size() ||
                    curr.size() != jt.joint_names.size())
                    return std::nullopt;

                const double delta =
                    std::abs(curr[j] - prev[j]);

                if (!std::isfinite(delta))
                    return std::nullopt;

                travel += delta;
                m.max_step = std::max(m.max_step, delta);
            }

            m.total += travel;
            m.max_joint = std::max(m.max_joint, travel);
        }

        const double excess =
            std::max(0.0, m.max_joint - preferred_travel);

        m.cost = m.total
            + 2.0 * m.max_joint
            + excess_weight * excess * excess;

        if (!std::isfinite(m.cost))
            return std::nullopt;

        return m;
    };

    std::optional<Plan> best;
    Metrics best_metrics;
    std::string best_planner;
    int successful = 0;
    int attempts = 0;

    const auto tryPlanner = [&](const std::string& pipeline,
                                const std::string& planner,
                                double planning_time,
                                unsigned int planning_attempts = 1
                            ) -> bool
    {

        ++attempts;

        group.clearPoseTargets();
        group.clearPathConstraints();

        group.setStartState(start_state);
        if (!group.setJointValueTarget(target_state))
            return false;

        group.setPlanningPipelineId(pipeline);
        group.setPlannerId(planner);
        group.setPlanningTime(planning_time);

        group.setMaxVelocityScalingFactor(options.velocity_scaling);
        group.setMaxAccelerationScalingFactor(options.acceleration_scaling);
        group.setNumPlanningAttempts(planning_attempts);
        const auto t0 = std::chrono::steady_clock::now();

        Plan candidate;
        const auto result = group.plan(candidate);

        const double elapsed_s =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_DEBUG(
                logger,
                "[TRANSIT TRY] %s/%s failed time=%.2fs",
                pipeline.c_str(), planner.c_str(), elapsed_s);
            return false;
        }

        auto metrics = score(candidate);

        if (!metrics) {
            RCLCPP_DEBUG(
                logger,
                "[TRANSIT TRY] %s/%s rejected by trajectory checks time=%.2fs",
                pipeline.c_str(), planner.c_str(), elapsed_s);
            return false;
        }

        RCLCPP_DEBUG(
            logger,
            "[TRANSIT TRY] %s/%s cost=%.2f max_joint=%.3f time=%.2fs",
            pipeline.c_str(), planner.c_str(),
            metrics->cost, metrics->max_joint, elapsed_s);

        ++successful;

        if (!best || metrics->cost < best_metrics.cost) {
            best = std::move(candidate);
            best_metrics = *metrics;
            best_planner = pipeline + "/" + planner;
        }

        return true;
    };

    const auto goodEnough = [&]() {
        return best &&
            best_metrics.cost <= good_cost &&
            best_metrics.max_joint <= good_max_joint;
    };

    // 1. Fast, deterministic joint-space motion.
    tryPlanner("pilz_industrial_motion_planner", "PTP", 2.0,1);
    if (!goodEnough() && options.guided_ptp_enabled)
        {
            auto guided = planGuidedPTP(
                group,
                logger,
                start_state,
                target_state,
                options);

            if (guided)
            {
                auto metrics = score(*guided);
                if(metrics)
                    RCLCPP_INFO(
                    logger,
                    "[TRANSIT DEBUG] guided ptp"
                    "total=%.3f max_joint=%.3f max_step=%.3f "
                    "cost=%.2f",
                    metrics->total,
                    metrics->max_joint,
                    metrics->max_step,
                    metrics->cost);

                if (metrics &&
                    (!best || metrics->cost < best_metrics.cost))
                {
                    best = std::move(*guided);
                    best_metrics = *metrics;
                    best_planner = "guided_ptp";
                }
            }
        }
    if (!goodEnough()) {
        for (const auto& choice : fallback) {
            tryPlanner(
                choice.pipeline,
                choice.planner,
                choice.time_s,
                choice.planning_attempts
            );

            if (!options.compare_all_fallbacks && goodEnough())
                break;
        }
    }
    

    

    // One summary per transit planning call.
    if (best) {
        RCLCPP_INFO(
            logger,
            "[TRANSIT SELECT] planner=%s attempts=%d success=%d "
            "total=%.3f max_joint=%.3f max_step=%.3f "
            "cost=%.2f",
            best_planner.c_str(),
            attempts,
            successful,
            best_metrics.total,
            best_metrics.max_joint,
            best_metrics.max_step,
            best_metrics.cost);

        return best;
    }

    RCLCPP_WARN(
        logger,
        "[TRANSIT SELECT] no valid plan after %d attempts",
        attempts);

    return std::nullopt;
}
       

std::optional<MoveGroup::Plan> planTransit(
    MoveGroup& group, const rclcpp::Logger& logger,
    const moveit::core::RobotState& start_state, const moveit::core::RobotState& target_state,
    const TransitValidator& validate, bool compare_all_fallbacks)
{
    using Plan = MoveGroup::Plan;

    struct Metrics {
        double total = 0.0;
        double max_joint = 0.0;
        double max_step = 0.0;
        double cost = std::numeric_limits<double>::infinity();
    };


    struct PlannerChoice {
        const char* pipeline;
        const char* planner;
        double time_s;
        int planning_attempts;
    };

    constexpr PlannerChoice fallback[] = {
        {"ompl", "RRTConnectkConfigDefault", 2.0, 1},
        {"ompl", "BiTRRTkConfigDefault",    3.0, 3},
        {"ompl", "APSConfigDefault",        5.0, 1},
        {"ompl", "LBKPIECEkConfigDefault",  3.0, 2},
        {"ompl", "LazyPRMkConfigDefault",   3.0, 2},
        {"ompl", "RRTstarkConfigDefault",   5.0, 2},
    };

    // Match CutGraphPlanner::transitCost().
    constexpr double preferred_travel = 1.0;
    constexpr double excess_weight = 8.0;

    // Quality triggers, not hard safety constraints.
    constexpr double good_cost = 75.0;
    constexpr double good_max_joint =3.5;


    const auto score = [&](const Plan& plan)
        -> std::optional<Metrics>
    {
        const auto& jt = plan.trajectory_.joint_trajectory;

        if (jt.points.empty() || jt.joint_names.empty())
            return std::nullopt;

        const auto& first = jt.points.front().positions;
        const auto& last = jt.points.back().positions;

        if (first.size() != jt.joint_names.size() ||
            last.size() != jt.joint_names.size())
            return std::nullopt;

        Metrics m;

        for (std::size_t j = 0; j < jt.joint_names.size(); ++j) {
            const auto& name = jt.joint_names[j];
            const auto& variables = start_state.getVariableNames();
            if (std::find(variables.begin(), variables.end(), name) == variables.end())
                return std::nullopt;

            // Reject a trajectory whose actual endpoints do not
            // match the requested joint configurations.
            const double endpoint_tol = 1e-3;

            const double q_start = start_state.getVariablePosition(name);
            const double q_goal  = target_state.getVariablePosition(name);

            if (!std::isfinite(first[j]) ||
                !std::isfinite(last[j]) ||
                !std::isfinite(q_start) ||
                !std::isfinite(q_goal) ||
                std::abs(first[j] - q_start) > endpoint_tol ||
                std::abs(last[j] - q_goal) > endpoint_tol)
            {
                return std::nullopt;
            }

            double travel = 0.0;

            for (std::size_t i = 1; i < jt.points.size(); ++i) {
                const auto& prev = jt.points[i - 1].positions;
                const auto& curr = jt.points[i].positions;

                if (prev.size() != jt.joint_names.size() ||
                    curr.size() != jt.joint_names.size())
                    return std::nullopt;

                const double delta =
                    std::abs(curr[j] - prev[j]);

                if (!std::isfinite(delta))
                    return std::nullopt;

                travel += delta;
                m.max_step = std::max(m.max_step, delta);
            }

            m.total += travel;
            m.max_joint = std::max(m.max_joint, travel);
        }

        const double excess =
            std::max(0.0, m.max_joint - preferred_travel);

        m.cost = m.total
            + 2.0 * m.max_joint
            + excess_weight * excess * excess;

        if (!std::isfinite(m.cost))
            return std::nullopt;

        return m;
    };

    std::optional<Plan> best;
    Metrics best_metrics;
    std::string best_planner;
    int successful = 0;
    int attempts = 0;

    const auto tryPlanner = [&](const std::string& pipeline,
                                const std::string& planner,
                                double planning_time,
                                unsigned int planning_attempts = 1
                            ) -> bool
    {

        ++attempts;

        group.clearPoseTargets();
        group.clearPathConstraints();

        group.setStartState(start_state);
        if (!group.setJointValueTarget(target_state))
            return false;

        group.setPlanningPipelineId(pipeline);
        group.setPlannerId(planner);
        group.setPlanningTime(planning_time);

        group.setMaxVelocityScalingFactor(0.05);
        group.setMaxAccelerationScalingFactor(0.01);
        group.setNumPlanningAttempts(planning_attempts);
        const auto t0 = std::chrono::steady_clock::now();

        Plan candidate;
        const auto result = group.plan(candidate);

        const double elapsed_s =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_DEBUG(
                logger,
                "[TRANSIT TRY] %s/%s failed time=%.2fs",
                pipeline.c_str(), planner.c_str(), elapsed_s);
            return false;
        }

        auto metrics = score(candidate);

        if (!metrics || (validate && !validate(candidate))) {
            RCLCPP_DEBUG(
                logger,
                "[TRANSIT TRY] %s/%s rejected by trajectory checks time=%.2fs",
                pipeline.c_str(), planner.c_str(), elapsed_s);
            return false;
        }

        RCLCPP_DEBUG(
            logger,
            "[TRANSIT TRY] %s/%s cost=%.2f max_joint=%.3f time=%.2fs",
            pipeline.c_str(), planner.c_str(),
            metrics->cost, metrics->max_joint, elapsed_s);

        ++successful;

        if (!best || metrics->cost < best_metrics.cost) {
            best = std::move(candidate);
            best_metrics = *metrics;
            best_planner = pipeline + "/" + planner;
        }

        return true;
    };

    const auto goodEnough = [&]() {
        return best &&
            best_metrics.cost <= good_cost &&
            best_metrics.max_joint <= good_max_joint;
    };

    // 1. Fast, deterministic joint-space motion.
    tryPlanner("pilz_industrial_motion_planner", "PTP", 2.0);
    // if (!goodEnough()) {
    //     //2. try chomp and if its goode enough return it 
    //     tryPlanner("chomp","CHOMP",2.0);
    // }
    // Successful, good-quality PTP: stop immediately.
    if (!goodEnough()) {

        for (const auto& choice : fallback) {
            tryPlanner(
                choice.pipeline,
                choice.planner,
                choice.time_s,
                choice.planning_attempts
            );

            if (!compare_all_fallbacks && goodEnough())
                break;
        }
    }

    // One summary per transit planning call.
    if (best) {
        RCLCPP_INFO(
            logger,
            "[TRANSIT SELECT] planner=%s attempts=%d success=%d "
            "total=%.3f max_joint=%.3f max_step=%.3f "
            "cost=%.2f",
            best_planner.c_str(),
            attempts,
            successful,
            best_metrics.total,
            best_metrics.max_joint,
            best_metrics.max_step,
            best_metrics.cost);

        return best;
    }

    RCLCPP_WARN(
        logger,
        "[TRANSIT SELECT] no valid plan after %d attempts",
        attempts);

    return std::nullopt;
}
}
