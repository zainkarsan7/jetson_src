

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
            (trajectoryCost(result.cut.process.approach)+
            trajectoryCost(result.cut.process.cut)+
            trajectoryCost(result.cut.process.retract));
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
            const std::size_t segment_count = cut_plan.segments.size();
            if(segment_count ==0 || candidates_by_segment.size()!=segment_count){
                                return std::nullopt;
            }
            //building the graph here:
            CutCandidateGraph graph;
            std::unordered_map<EdgeId, TransitEdgeInfo> edge_lookup;
            std::unordered_map<NodeId, CandidateGraphEntry> candidate_lookup;

            // each cut as a home node before and after and adjsect cuts share a home node
            std::vector<NodeId> home_nodes;
            home_nodes.reserve(segment_count+1);

            for (std::size_t i=0; i<=segment_count; i++){
                home_nodes.push_back(graph.addNode(i*2,0.0));
            }
            for(std::size_t segment_index = 0; segment_index<segment_count;++segment_index){

                const auto& candidates = candidates_by_segment[segment_index];
                const std::size_t count = std::min(candidates.size(),options_.max_candidates_per_segment);
                std::size_t added = 0;
                for(std::size_t cand_ind = 0; cand_ind<count; cand_ind++){
                    const auto& candidate = candidates[cand_ind];
                    if(!candidate.entry || !candidate.exit || candidate.cut.empty())continue;

                    const NodeId node_id = graph.addNode(2*segment_index + 1,options_.process_weight* candidate.score);
                    candidate_lookup.emplace(node_id,CandidateGraphEntry{
                        node_id,
                        segment_index,cand_ind
                    });
                    const double in_estimate = options_.transit_weight * home_state.distance(*candidate.entry,joint_model_group_);
                    const double out_estimate =  options_.transit_weight * candidate.exit->distance(home_state, joint_model_group_);

                    const EdgeId in_edge = graph.addEdge(home_nodes[segment_index],node_id, in_estimate);
                    const EdgeId out_edge = graph.addEdge(node_id,home_nodes[segment_index+1], out_estimate);
                    
                    edge_lookup.emplace(in_edge, TransitEdgeInfo{
                        in_edge,
                        segment_index,
                        cand_ind,
                        true
                    });

                    edge_lookup.emplace(out_edge, TransitEdgeInfo{
                        out_edge,
                        segment_index,
                        cand_ind,
                        false
                    });
                    ++added;
                }
                if(added ==0){
                    std::cout<<"failed to add any nodes/edges for segment"<<segment_index<<std::endl;
                    return std::nullopt;
                }
            }
            // now create caches 
            using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
            std::unordered_map<EdgeId, Plan> transit_cache;
            std::unordered_map<NodeId, SegmentMotionPlan> process_cache;


            const NodeId start_node = home_nodes.front();

            const NodeId goal_node = home_nodes.back();

            constexpr double unknown_penalty = 0.0;
            constexpr double failed_penalty = 1e6;

            GraphRoute selected_route;

            // do lazy search validation here:
            
            while(true){

                GraphRoute route = graph.cheapestRoute(
                    start_node,
                    goal_node,
                    unknown_penalty,
                    failed_penalty
                );

                if(route.nodes.empty()){
                    return std::nullopt;
                }

                bool changed = false;
                // now start planning the process nodes in the route:
                for(NodeId node_id: route.nodes){
                    const auto lookup_it = candidate_lookup.find(node_id);
                
                    if(lookup_it==candidate_lookup.end()){
                        continue; // home node
                    }

                    if(process_cache.count(node_id)!=0){
                        continue;//already planned and validated this node
                    }

                    //now do some rpcess call on this candidate
                    const auto& info = lookup_it->second;
                    const auto& segment = cut_plan.segments[info.segment_index];
                    const auto& candidate = candidates_by_segment[info.segment_index][info.candidate_index];
                    ++statistics_.candidates_tested;
                    auto process = process_planner_(segment,candidate);
                    if(!process || !validateProcess(candidate, *process)){
                        ++statistics_.process_failures;
                        //this node fails, so remove any connecting edges:
                        for(const auto& e: graph.edges()){
                            if(e.from == node_id || e.to == node_id){
                                graph.edge(e.id).status = TransitStatus::Invalid;
                            }
                        }

                        changed= true;
                        break;
                    }
                    const double actual_process_cost = options_.process_weight * 
                    (trajectoryCost(process->approach)+ 
                    trajectoryCost(process->cut)+
                    trajectoryCost(process->retract));
                    
                    graph.node(node_id).process_cost = actual_process_cost;
                    
                    process_cache.emplace(node_id,std::move(*process));
                    changed= true;
                    break;
                }

                //the graph changed so recompute route
                if(changed) continue;
                std::cout<<"all graph nodes validated"<<std::endl;
                //resolve the first unplanned transit edge, 
                //by now all the process nodes must have been validated
                for(EdgeId edge_id : route.edges){
                    auto& edge = graph.edge(edge_id);

                    if(edge.status == TransitStatus::Valid){
                        continue;
                    }
                    if(edge.status == TransitStatus::Invalid){
                        throw std::logic_error("one of the edges is invalid");
                    }
                    const auto info = edge_lookup.at(edge_id);
                    const NodeId candidate_node = info.transit_in ? edge.to : edge.from;
                    const auto& process = process_cache.at(candidate_node); // get the process node,
                    // start computing transit in transit out 
                    const auto& actual_entry = firstState(process.approach);
                    const auto& actual_exit = lastState(process.retract);

                    if(!actual_entry || !actual_exit){
                        edge.status = TransitStatus::Invalid;
                        changed= true;
                        break;
                    }
                    // now do move group planning call:
                    auto transit = info.transit_in ? 
                    transit_planner_(home_state, *actual_entry) : 
                    transit_planner_(*actual_exit, home_state);
                    if(!transit){
                        if(info.transit_in) ++statistics_.transit_in_failures;
                        else ++statistics_.transit_out_failures;
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
                    if(info.transit_in) ++statistics_.transit_in_failures;
                    else ++statistics_.transit_out_failures;
                    edge.status = TransitStatus::Invalid;
                    changed=true;
                    break;
                }
                edge.actual_cost = trajectoryCost(*transit)*options_.transit_weight;
                edge.status = TransitStatus::Valid;
                transit_cache.emplace(edge_id, std::move(*transit));
                changed = true;
                break;
                }
                if (changed) continue;
                //no unresolved processes or transits now, 
                std::cout<<"all edges transits, nodes processes resolved"<<std::endl;
                selected_route = std::move(route);
                break;
            } 
            // now assemble the executable plan from the route:                
                
            ExecutablePlan result;
            result.plan_id = plan_id;
            result.estimate_id = estimate_id;
            result.cuts.reserve(segment_count);

            for (std::size_t i = 1; i+1 < selected_route.nodes.size(); i++){
                const NodeId node_id = selected_route.nodes[i];
                const auto lookup_it = candidate_lookup.find(node_id);
                if(lookup_it==candidate_lookup.end()){
                    continue; //home node
                }
                const auto& info = lookup_it->second;
                // a node always is between two edges: both home nodes
                const EdgeId in_edge = selected_route.edges.at(i-1);
                const EdgeId out_edge = selected_route.edges.at(i);

                ExecutableCut cut;
                cut.name = cut_plan.segments[info.segment_index].name;
                cut.transit_in = transit_cache.at(in_edge);
                cut.process = process_cache.at(node_id);
                cut.transit_out = transit_cache.at(out_edge);
                result.cuts.push_back(cut);
                ++statistics_.accepted_candidates;

            }
            if(result.cuts.size()!=cut_plan.segments.size()) return std::nullopt;

        return result;
        }



}
