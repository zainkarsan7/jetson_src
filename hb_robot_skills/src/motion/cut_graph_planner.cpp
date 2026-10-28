

#include "hb_robot_skills/motion/cut_graph_planner.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

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
            if(!transit_planner || !process_planner){
                throw std::invalid_argument("missing planning callback");

            }

            if(options_.max_candidates_per_segment == 0||
                options_.boundary_tol <=0.0||
                options_.process_weight <0.0||
                options_.transit_weight <0.0
            ){
               throw std::invalid_argument("inbalid options");

            }
        };

        std::optional<CutGraphPlanner::PlannedCandidate> CutGraphPlanner::evaluateCandidate(
            const CutSegment& segment,
            const ProcessCandidate& candidate,
            const moveit::core::RobotState& home_state,
            std::size_t candidate_index
        ){

            ++ statistics_.candidates_tested;
            if(!candidate.entry || !candidate.exit){
                ++statistics_.process_failures;
                return std::nullopt;
            }
            //pilz linear stuff
            auto process = process_planner_(segment,candidate);

            if(!process || !validateProcess(candidate, *process)){
                ++statistics_.process_failures;
                return std::nullopt;
            }
            const auto actual_entry = firstState(process->approach);
            const auto actual_exit = lastState(process->retract);
            if(!actual_entry || !actual_exit){
                ++statistics_.process_failures;
                return std::nullopt;
            }
            
            auto transit_in = transit_planner_(home_state,*actual_entry);
            if(!transit_in){
                ++statistics_.transit_in_failures;
                return std::nullopt;
            }
            const auto transit_in_first = firstState(*transit_in);
            const auto transit_in_last = lastState(*transit_in);

            if(!transit_in_first || !transit_in_last||
                statesMatch(*transit_in_first,home_state)||
                statesMatch(*transit_in_last,*actual_entry)
            ){
                ++statistics_.transit_in_failures;
                return std::nullopt;
            }

            auto transit_out = transit_planner_(*actual_exit, home_state);
            if(!transit_out){
                ++statistics_.transit_out_failures;
                return std::nullopt;
            }
            const auto transit_out_first = firstState(*transit_out);
            const auto transit_out_last = lastState(*transit_out);

            if(!transit_out_first || !transit_out_last||
                statesMatch(*transit_out_first,*actual_exit)||
                statesMatch(*transit_out_last,home_state)
            ){
                ++statistics_.transit_out_failures;
                return std::nullopt;
            }

            PlannedCandidate result;
            result.candidate_index = candidate_index;
            result.cut.name = segment.name;
            result.cut.transit_in = std::move(*transit_in);
            result.cut.process = std::move(*process);
            result.cut.transit_out = std::move(*transit_out);
            const double process_cost = 
            trajectoryCost(result.cut.process.approach)+
            trajectoryCost(result.cut.process.cut)+
            trajectoryCost(result.cut.process.retract);
            const double transit_cost = 
            trajectoryCost(result.cut.transit_in) +
            trajectoryCost(result.cut.transit_out);

            result.cost = transit_cost * options_.transit_weight + 
            process_cost * options_.process_weight;

            ++statistics_.accepted_candidates;
            return result;

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

        bool CutGraphPlanner::statesMatch(const moveit::core::RobotState& a,
        const moveit::core::RobotState& b)const{
            return a.distance(b,joint_model_group_) <= options_.boundary_tol;
        }

        bool CutGraphPlanner::validateProcess(const ProcessCandidate& candidate, 
        const SegmentMotionPlan& process)const{

            if(!candidate.entry || !candidate.exit){
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
            return true;
        }
        

        std::optional<ExecutablePlan> CutGraphPlanner::plan(
            const CutPlan& cut_plan, 
            const std::vector<std::vector<ProcessCandidate>>& candidates_by_segment,
            const moveit::core::RobotState& home_state,
            const std::string& plan_id,
            const std::string& estimate_id
        ){

            statistics_ = {};

            if(!cut_plan.segments.empty()){
                return std::nullopt;
            }
            if(!cut_plan.segments.size()!=candidates_by_segment.size()){
                return std::nullopt;
            }
            ExecutablePlan result;
            result.plan_id = plan_id;
            result.estimate_id = estimate_id;

            result.cuts.reserve(cut_plan.segments.size());

            for(std::size_t segment_index = 0; segment_index<cut_plan.segments.size(); segment_index++){
                const CutSegment& segment=cut_plan.segments[segment_index];
                const auto& candidates = candidates_by_segment[segment_index];

                if(candidates.empty()){
                    return std::nullopt;
                }
                std::optional<PlannedCandidate> best;
                const std::size_t count = std::min(candidates.size(),
            options_.max_candidates_per_segment);

                for(std::size_t cand_ind = 0; cand_ind<count; cand_ind++){
                    auto planned = evaluateCandidate(segment, 
                        candidates[cand_ind],
                        home_state,
                        cand_ind
                    );
                    if(!planned) continue;
                    if(!best || planned->cost < best->cost){
                        best = std::move(planned);
                    }
                    if(!best){
                        return std::nullopt;
                    }
                    result.cuts.push_back(std::move(best->cut));
                }
                return result;

            }



        }



}
