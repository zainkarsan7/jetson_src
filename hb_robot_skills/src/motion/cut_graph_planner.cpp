

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
                !statesMatch(*transit_in_first,home_state)||
                !statesMatch(*transit_in_last,*actual_entry)
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
                !statesMatch(*transit_out_first,*actual_exit)||
                !statesMatch(*transit_out_last,home_state)
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
                !retract_last ||
                !candidate.cut.front()||
                !candidate.cut.back() ||
                candidate.cut.empty()
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

            if(cut_plan.segments.empty()){
                return std::nullopt;
            }
            if(cut_plan.segments.size()!=candidates_by_segment.size()){
                return std::nullopt;
            }

            CutCandidateGraph graph;
            std::vector<CandidateGraphEntry> candidate_nodes;
            std::vector<TransitEdgeInfo> transit_edges;

            
            using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

            std::vector<std::optional<Plan>> edge_plans(graph.edges().size());
            std::vector<std::optional<SegmentMotionPlan>> process_plans(candidate_nodes.size());

            std::unordered_map<NodeId, Plan> transit_cache;
            std::unordered_map<EdgeId, SegmentMotionPlan> process_cache;
            

            std::unordered_map<NodeId, TransitEdgeInfo> edge_lookup;
            std::unordered_map<EdgeId, CandidateGraphEntry> candidate_lookup;
            
            
            
            
            
            
            std::vector<NodeId> home_nodes;
            home_nodes.reserve(cut_plan.segments.size()+1);

            for (std::size_t i=0; i<cut_plan.segments.size(); i++){
                home_nodes.push_back(graph.addNode(i*2,0.0));
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
                    const auto& candidate = candidates[cand_ind];
                    if(!candidate.entry || !candidate.exit)continue;
                    const NodeId candidate_node = graph.addNode(2*segment_index + 1,candidate.score);
                    candidate_nodes.push_back({candidate_node,segment_index,cand_ind});
                    const double in_estimate = home_state.distance(*candidate.entry,joint_model_group_);
                    const double out_estimate = home_state.distance(*candidate.exit,joint_model_group_);
                    const EdgeId in_edge = graph.addEdge(home_nodes[segment_index],candidate_node, in_estimate);
                    const EdgeId out_edge = graph.addEdge(candidate_node,home_nodes[segment_index+1], out_estimate);
                    transit_edges.push_back({
                        in_edge,segment_index,cand_ind,true
                    });
                    transit_edges.push_back({
                        out_edge,segment_index+1, cand_ind, false
                    });


                



                    auto planned = evaluateCandidate(segment, 
                        candidate,
                        home_state,
                        cand_ind
                    );




                    if(!planned) continue;
                    if(!best || planned->cost < best->cost){
                        best = std::move(planned);
                    }
                   
                }

                for(const auto& entry: candidate_nodes){
                candidate_lookup.emplace(entry.node_id,entry);
                }

                for(const auto& entry: transit_edges){
                    edge_lookup.emplace(entry.edge_id,entry);
                }



                constexpr double unknown_penalty = 0.0;
                constexpr double failed_penalty = 1e6;

                while(true){

                    GraphRoute route = graph.cheapestRoute(
                        home_nodes.front(),
                        home_nodes.back(),
                        unknown_penalty,
                        failed_penalty
                    );

                    if(route.nodes.empty()){
                        return std::nullopt;
                    }

                    bool changed = false;

                    for(NodeId node_id: route.nodes){
                        const auto it = candidate_lookup.find(node_id);
                    
                        if(it==candidate_lookup.end()){
                            continue;
                        }

                        if(process_cache.count(node_id)){
                            continue;
                        }

                        const auto& entry = it->second;
                        const auto& segment = cut_plan.segments[segment_index];
                        const auto& candidate = candidates_by_segment[entry.segment_index][entry.candidate_index];
                        auto process = process_planner_(segment,candidate);
                        if(!process || !validateProcess(candidate, *process)){
                            for(const auto& edge: graph.edges()){
                                if(edge.from == node_id || edge.to == node_id){
                                    graph.edge(edge.id).status = TransitStatus::Invalid;
                                }
                            }
                            changed= true;
                            break;
                        }
                        process_cache.emplace(node_id,std::move(*process));
                        changed= true;
                        break;
                    }
                    if(changed) continue;
                    for(EdgeId edge_id : route.edges){
                        auto& edge = graph.edge(edge_id);
                        if(edge.status == TransitStatus::Valid){
                            continue;
                        }
                        if(edge.status == TransitStatus::Invalid){
                            continue;
                        }
                        const auto info_it = edge_lookup.find(edge_id);
                        if(info_it == edge_lookup.end()){
                            throw std::runtime_error("meta data for edge not there");
                        }
                        const auto& info = info_it->second;
                        const auto& candidate = candidates_by_segment[info.segment_index][info.candidate_index];
                        const auto& process = process_cache.at(info.transit_in ? edge.to : edge.from);
                        const auto& actual_entry = firstState(process.approach);
                        const auto& actual_exit = lastState(process.retract);

                        if(!actual_entry || !actual_exit){
                            edge.status = TransitStatus::Invalid;
                            changed= true;
                            break;
                        }
                        auto transit = info.transit_in ? 
                        transit_planner_(home_state, *actual_entry) : 
                        transit_planner_(*actual_exit, home_state);
                        if(!transit){
                            edge.status = TransitStatus::Invalid;
                            changed=true;
                            break;
                        }
                        const auto first  = firstState(*transit);
                        const auto last = lastState(*transit);
                        const bool connected = first && last && (
                            info.transit_in? statesMatch(*first, home_state) && 
                            statesMatch(*last, *actual_entry) : 
                            statesMatch(*first, *actual_exit) && 
                            statesMatch(*last, home_state)
                    );
                    if(!connected){
                        edge.status = TransitStatus::Invalid;
                        changed=true;
                        break;
                    }
                    edge.actual_cost = trajectoryCost(*transit);
                    edge.status = TransitStatus::Valid;
                    transit_cache.emplace(edge_id, std::move(*transit));
                    changed = true;
                    break;





                    }
                    if (changed) continue;


                    break;

                }





                if(!best){
                    return std::nullopt;
                }
                result.cuts.push_back(std::move(best->cut));
          

            }
                  return result;



        }



}
