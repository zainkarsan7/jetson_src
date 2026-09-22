#include "hb_robot_skills/motion/trajectory_validator.hpp"
#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/exploration_types.hpp"
#include "moveit/robot_model/joint_model_group.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>


namespace hb_robot_skills::motion{

    namespace{
        // little function to make samples symmetric 
        std::vector<double> make_symmetric(double range, std::size_t count){

            if (count<=1 || range <= 0){
                return{0.0};

            }

            std::vector<double> vals;
            for (size_t i = 0; i<count; i++){
                double alpha = static_cast<double>(i)/static_cast<double>(count-1);

                vals.push_back(range  * 2.0 *alpha - range );
               
            }
            std::sort(vals.begin(),vals.end(), [](double a, double b){
                return std::abs(a)<std::abs(b);
            });
            return vals;
        };
    }

    ExplorationPlanner::ExplorationPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            std::string camera_link
        ):
        robot_model_(std::move(robot_model)),
        joint_model_group_(nullptr),
        planning_group_(std::move(planning_group)),
        camera_link_(std::move(camera_link))

        {
            if(!robot_model_){
                throw std::invalid_argument("Exploration Planner: robot model null");

            }

            joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
            if(!joint_model_group_){
                throw std::invalid_argument("Exploration Planner: planning group doesnt exist");

            }

            if(!robot_model_->hasLinkModel(camera_link_)){
                throw std::invalid_argument("Exploration Planner: camera link doesnt exist");

            }
        }


        

        std::vector<Eigen::Isometry3d> ExplorationPlanner::generateNominalViews(
            const ExplorationRequest& request
        )const{

            std::vector<Eigen::Isometry3d>  views;

            if (request.num_samples < 1){
                return views;
            }

            views.reserve(request.num_samples * request.num_samples);

            const auto xs = make_symmetric(request.range_x,request.num_samples);
            const auto ys = make_symmetric(request.range_y,request.num_samples);

            auto addView = [&](double rx,double ry){
                    Eigen::AngleAxisd Rx(rx,Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd Ry(ry,Eigen::Vector3d::UnitY());
                    Eigen::Isometry3d T_view = request.center_pose;
                    T_view.linear() = request.center_pose.linear() * Rx.toRotationMatrix() * Ry.toRotationMatrix();
                    views.push_back(T_view);
            };
            for (std::size_t ix = 0; ix<xs.size(); ix++){
                const double rx = xs[ix];
                if (ix%2==0){
                    for (double ry: ys){
                        addView(rx,ry); 
                    } 
                }
                else{
                    for (auto it = ys.rbegin(); it!=ys.rend(); it++){
                    addView(rx,*it);
                }
                }
                
            }
            return views;
            
        }

        std::vector<ViewCandidate> ExplorationPlanner::generateCandidates(
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request
        )const{

            std::vector<ViewCandidate> candidates;
            
            const auto pos_samples = make_symmetric(request.pos_tol,request.sample_attempts);
            const auto orn_samples = make_symmetric(request.roll_tol,request.sample_attempts);

            for (double dx : pos_samples){
                for (double dy: pos_samples){
                    for(double dz: pos_samples){
                        for (double dr : orn_samples){
                            ViewCandidate candidate;                         
                            Eigen::AngleAxisd Rdr(dr,Eigen::Vector3d::UnitX());
                            candidate.candidate_pose = nominal_pose;
                            candidate.candidate_pose.translation() += Eigen::Vector3d(dx,dy,dz);
                            candidate.candidate_pose.linear() = nominal_pose.linear() * Rdr.toRotationMatrix();
                            candidate.nominal_pose = nominal_pose;
                            candidate.pos_error = Eigen::Vector3d(dx,dy,dz).norm();
                            candidate.roll_offset = dr;
                            candidates.push_back(std::move(candidate));

                        }
                    }
                }
            }
        
            std::sort(candidates.begin(),candidates.end(),
        [](const ViewCandidate& cand_a,const ViewCandidate& cand_b){
            const double score_a = cand_a.pos_error + 0.1 * std::abs(cand_a.roll_offset);
            const double score_b = cand_b.pos_error + 0.1 * std::abs(cand_b.roll_offset);
            return (score_a < score_b);

        });
        return candidates;
        
        }


        std::optional<ViewSolution> ExplorationPlanner::solveCandidate(const moveit::core::RobotState& seed_state,
            const ViewCandidate& candidate,
            const ExplorationRequest& request
        )const{
            moveit::core::RobotState state(seed_state);
            std::vector<double> consistency_limits(joint_model_group_->getVariableCount(),
            request.joint_delta);
            const bool found = state.setFromIK(joint_model_group_,
                candidate.candidate_pose,
                camera_link_,
                consistency_limits,
                request.ik_timeout
            );

            if(!found){
                return std::nullopt;
            }

            state.update();

            if(!state.satisfiesBounds(joint_model_group_)){
                return std::nullopt;
            }
            ViewSolution solution(state);
            solution.cam_pose = candidate.candidate_pose;
            solution.motion_cost = scoreSoln(seed_state,state);
            solution.nominal_pose = candidate.nominal_pose;
            

            return solution;
        }

        std::optional<ViewSolution> ExplorationPlanner::solveView(const moveit::core::RobotState& seed_state,
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request)const{

                auto candidates = ExplorationPlanner::generateCandidates(
                    nominal_pose,request);
                
                
                std::optional<ViewSolution> best_soln;
                
                for(const auto &cand : candidates){
                        auto viewSol = ExplorationPlanner::solveCandidate(
                            seed_state,
                            cand, request);
                        if(!viewSol){
                            continue;
                        }
                        if(!best_soln || viewSol->motion_cost < best_soln->motion_cost){
                            best_soln = std::move(viewSol);
                        }

                }
                return best_soln;
            }

        double ExplorationPlanner::scoreSoln(
            const moveit::core::RobotState& state_from, 
            const moveit::core::RobotState& state_to
        ) const{
            std::vector<double> q_from;
            std::vector<double> q_to;

            state_from.copyJointGroupPositions(joint_model_group_,q_from);
            state_to.copyJointGroupPositions(joint_model_group_,q_to);
            double sq_q_delta= 0.0;

            for (size_t i = 0; i<q_from.size(); i++){
                const double q_delta = q_to[i] - q_from[i];

                sq_q_delta += q_delta *q_delta;
            }
            return std::sqrt(sq_q_delta);

        }


        ExplorationPlan ExplorationPlanner::plan(
            const moveit::core::RobotState& start_state,
            const ExplorationRequest& request
        ) const{
            ExplorationPlan result;
            
            auto nom_views  = ExplorationPlanner::generateNominalViews(request);

            result.n_requested_views = nom_views.size();
            moveit::core::RobotState seed = start_state;
            for (auto const & view: nom_views){
                // candidates are generated in this function
                auto best_view_soln = ExplorationPlanner::solveView(
                    seed, view,request);
                if (!best_view_soln){
                    continue;
                }
                seed = best_view_soln->robot_state;
                result.views.push_back(*best_view_soln);
                result.n_succeeded_views ++;
            }
            return result;
        }
     
 }