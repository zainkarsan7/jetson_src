#include "hb_robot_skills/motion/transit_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hb_robot_skills::motion {
    
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
        {"ompl", "RRTConnectkConfigDefault", 2.0, 4},
        {"ompl", "BiTRRTkConfigDefault",    3.0, 3},
        {"ompl", "APSConfigDefault",        5.0, 1},
        {"ompl", "LBKPIECEkConfigDefault",  3.0, 3},
        {"ompl", "LazyPRMkConfigDefault",   3.0, 3},
        {"ompl", "RRTstarkConfigDefault",   5.0, 3},
    };

    // Match CutGraphPlanner::transitCost().
    constexpr double preferred_travel = 1.0;
    constexpr double excess_weight = 8.0;

    // Quality triggers, not hard safety constraints.
    constexpr double good_cost = 75.0;
    constexpr double good_max_joint =3.0;


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

        group.setMaxVelocityScalingFactor(0.1);
        group.setMaxAccelerationScalingFactor(0.1);
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
