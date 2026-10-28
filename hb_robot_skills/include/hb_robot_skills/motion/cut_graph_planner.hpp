#pragma once

#include <vector>
#include <functional>
#include <cstddef>
#include <optional>
#include <string>



#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "hb_robot_skills/motion/cut_candidate_graph.hpp"

#include "hb_robot_skills/motion/cut_planner.hpp"
#include "hb_robot_skills/motion/cut_process_trajectory_generator.hpp"


namespace hb_robot_skills::motion{

struct TransitPlanningResult{
                bool success = false;
                std::vector<moveit::core::RobotStatePtr> states;
                double cost = 0.0;
            };


class CutGraphPlanner{
    public:
        

        using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

        using TransitPlanner = std::function<std::optional<Plan>(const moveit::core::RobotState&, const moveit::core::RobotState&)>;

        using ProcessPlanner = std::function<std::optional<SegmentMotionPlan>(const CutSegment& segment, const ProcessCandidate& candidate)>;

        struct Options{

            std::size_t max_candidates_per_segment = 24;
            double boundary_tol = 1e-3;
            double process_weight = 1.0;
            double transit_weight = 1.0;

        };

        struct Statistics{
            std::size_t candidates_tested = 0;
            std::size_t process_failures = 0;
            std::size_t transit_in_failures = 0;
            std::size_t transit_out_failures = 0;
            std::size_t accepted_candidates = 0;
        };

        CutGraphPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            TransitPlanner transit_planner,
            ProcessPlanner process_planner,
            Options options = Options{}
        );

        std::optional<ExecutablePlan> plan(
            const CutPlan& cut_plan, 
            const std::vector<std::vector<ProcessCandidate>>& candidates_by_segment,
            const moveit::core::RobotState& home_state,
            const std::string& plan_id,
            const std::string& estimate_id
        );

        const Statistics& statistics()const noexcept{
            return statistics_;
        };


    private:

        struct PlannedCandidate{
            ExecutableCut cut;
            double cost = 0.0;
            std::size_t candidate_index= 0;
        };

        std::optional<PlannedCandidate> evaluateCandidate(
            const CutSegment& segment,
            const ProcessCandidate& candidate,
            const moveit::core::RobotState& home_state,
            std::size_t candidate_index
        );

        std::optional<moveit::core::RobotState> firstState(
            const Plan& plan
        )const;
        std::optional<moveit::core::RobotState> lastState(
            const Plan& plan
        )const;

        double trajectoryCost(const Plan& plan)const;

        bool CutGraphPlanner::statesMatch(const moveit::core::RobotState& a,
        const moveit::core::RobotState& b)const;

        bool validateProcess(const ProcessCandidate& candidate, 
        const SegmentMotionPlan& process)const;

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_ = nullptr;

        std::string planning_group_;
        TransitPlanner transit_planner_;
        ProcessPlanner process_planner_;
        Options options_;
        Statistics statistics_;


};



}