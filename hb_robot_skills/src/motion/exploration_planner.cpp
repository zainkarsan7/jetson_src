#include "hb_robot_skills/motion/trajectory_validator.hpp"
#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/exploration_types.hpp"

#include "moveit/kinematic_constraints/kinematic_constraint.h"
#include "moveit/kinematic_constraints/utils.h"

#include "moveit/robot_model/joint_model_group.h"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <random>
#include <iostream>
#include <array>

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

            if (request.num_viewpoints < 1){
                return views;
            }

            views.reserve(request.num_viewpoints * request.num_viewpoints);

            const auto xs = make_symmetric(request.range_x,request.num_viewpoints);
            const auto ys = make_symmetric(request.range_y,request.num_viewpoints);

            auto addView = [&](double rx,double ry){
                    Eigen::AngleAxisd Rx(rx,Eigen::Vector3d::UnitZ());
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
            /// make offsets 
            candidates.reserve(request.num_candidates);
            ViewCandidate nom;
            nom.candidate_pose = nominal_pose;
            nom.nominal_pose = nominal_pose;
            nom.pos_error = 0.0;
            nom.roll_offset = 0.0;
            candidates.push_back(nom);
            std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<double> dist(-1.0,1.0);
            auto makeOffset = [&](double dp, double dr,ViewCandidate candidate){
                candidate.roll_offset = dr * dist(rng);
                Eigen::AngleAxisd Rdr(candidate.roll_offset,Eigen::Vector3d::UnitX());
                Eigen::Vector3d offset(dist(rng),dist(rng),dist(rng));
                candidate.candidate_pose = nominal_pose;
                candidate.nominal_pose = nominal_pose;
                if (offset.norm()>1e-9){
                    offset = offset.normalized() * std::cbrt(std::abs(dist(rng)))* dp;
                }
                candidate.candidate_pose.translation() += offset;


                candidate.candidate_pose.linear() = nominal_pose.linear() * Rdr.toRotationMatrix();
                candidate.pos_error = offset.norm();
                candidates.push_back(std::move(candidate));
            };

            for (size_t ic = 0; ic<request.num_candidates;ic++){
                ViewCandidate candidate;
                makeOffset(request.pos_tol,request.roll_tol,candidate);
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
                uint8_t found_solutions= 0;
                for(const auto &cand : candidates){
                        auto viewSol = ExplorationPlanner::solveCandidate(
                            seed_state,
                            cand, request);
                        if(!viewSol){
                            

                            continue;
                        } 
                        found_solutions+=1;
                        if(!best_soln || viewSol->motion_cost < best_soln->motion_cost){
                            best_soln = std::move(viewSol);
                        }

                }

                RCLCPP_INFO(rclcpp::get_logger("ExplorationPlanner"),"View : %d/%zu solutions/candidates",found_solutions,candidates.size());
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


        //NBV STUFF 

        NBVPlanner::NBVPlanner(
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
                throw std::invalid_argument("NBV Planner: robot model null");

            }

            joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
            if(!joint_model_group_){
                throw std::invalid_argument("NBV Planner: planning group doesnt exist");

            }

            if(!robot_model_->hasLinkModel(camera_link_)){
                throw std::invalid_argument("NBV Planner: camera link doesnt exist");

            }
        }

        std::vector<ProfileViewCandidate>
NBVPlanner::debugCandidates(
    const ProfileInspectionRequest& request,
    const hb_perception::ProfileMatch& profile_estimate) const
{
    return generateCandidates(request, profile_estimate);
}


         std::vector<ProfileViewCandidate> NBVPlanner::generateCandidates(
        const ProfileInspectionRequest& request,
        const hb_perception::ProfileMatch& profile_estimate
    )const{

        std::vector<ProfileViewCandidate> result;
        const Eigen::Vector3d camera_axis = request.reference_camera_pose.linear().col(2).normalized();
        const Eigen::Vector3d member_axis = request.world_from_member.linear().col(2).normalized();

        const Eigen::Vector3d x = request.world_from_member.linear().col(0).normalized();
        const Eigen::Vector3d y = request.world_from_member.linear().col(1).normalized();

        /// based on camera position

        Eigen::Vector3d radial = request.reference_camera_pose.translation()-request.target_world ;
        radial -= radial.dot(member_axis) * member_axis;

        if (radial.norm()<1e-6){
            return {};
        }
        radial.normalize();

        const Eigen::Vector3d tangent = member_axis.cross(radial).normalized();

        constexpr std::array<double,7> angles{
            0.0,-M_PI/12.0,M_PI/12.0,-M_PI /6.0, M_PI/6.0, -M_PI/3.0, M_PI/3.0
        };

        for (double theta: angles){
            const Eigen::Vector3d direction = std::cos(theta)*radial + std::sin(theta)*tangent;
            const Eigen::Vector3d camera_position = request.target_world + request.cam_dist*direction;
            //x axis here
            const Eigen::Vector3d forward = (request.target_world - camera_position).normalized();
            Eigen::Vector3d desired_up = camera_axis - camera_axis.dot(forward)*forward;
            if(desired_up.norm()<1e-6) continue;
            desired_up.normalize();
            Eigen::Vector3d left = desired_up.cross(forward);
            // Eigen::Vector3d right = axis.cross(forward);
            if(left.norm() < 1e-6){
                continue;
            }
            left.normalize();
            const Eigen::Vector3d up = forward.cross(left).normalized();
            Eigen::Matrix3d R;
            R.col(0) = forward;
            R.col(1) = left;
            R.col(2) = up;

            ProfileViewCandidate candidate;
            candidate.nominal_cam_pose.linear() = R;
            candidate.nominal_cam_pose.translation() = camera_position;
            candidate.view_constraints = makePoseConstraints(request,candidate);
            result.push_back(candidate);
            if(static_cast<std::size_t>(result.size())>= request.num_candidates) break;

        }   
        return result;

    }


double NBVPlanner::predictedCoverageGain(
    const ProfileInspectionRequest& request,
    const Eigen::Isometry3d& camera_pose,
    const hb_perception::ProfileObservationModel& observation)const{

        double gain = 0.0;

        const auto& samples = observation.coverage();
        if (samples.empty()) return gain;
        const Eigen::Vector3d& camera = camera_pose.translation();
        for (const auto& sample: samples){

            // const Eigen::Vector2f p_sample = observation.match().transform* sample.position;

            const Eigen::Vector3d p_local(sample.position.x(),
        sample.position.y(),0.0);
        
            const Eigen::Vector3d p_world = request.world_from_member * p_local;
            Eigen::Vector3d n_local(sample.normal.x(),sample.normal.y(),0.0);
            Eigen::Vector3d n_world = request.world_from_member.linear() * n_local;
            if(n_world.norm() <1e-8){
                continue;
            }
            n_world.normalize();

            Eigen::Vector3d toward_cam = camera - p_world;

            if(toward_cam.norm() < 1e-8) continue;
            toward_cam.normalize();
            const double facing = std::max(0.0,n_world.dot(toward_cam));


            // const double incidence = n_world.dot(toward_cam);



            const double missing = 1.0 - std::clamp(static_cast<double>(sample.support),0.0,1.0);
            gain += missing * facing;


        }
        return gain/static_cast<double>(samples.size());

    }




    std::optional<ProfileViewSolution> NBVPlanner::plan(
    const moveit::core::RobotState& start_state,
    const ProfileInspectionRequest& request,
    const hb_perception::ProfileObservationModel& observed_profile,
    const planning_scene::PlanningSceneConstPtr& p_scene
)const{

    const auto candidates = generateCandidates(request,observed_profile.match());
    std::optional<ProfileViewSolution> best;
    double best_score = -std::numeric_limits<double>::infinity();

    for (const auto& cand : candidates){
        auto soln = solveViewConstraints(start_state,cand,request,p_scene);
        if(!soln){
            std::cerr<<"solution not found"<<std::endl;
            continue;
        }
        soln->candidate.predicted_gain = predictedInformationGain(request,*soln,observed_profile);
        soln->candidate.total_score = 10 * soln->candidate.predicted_gain - 0.15 * soln->candidate.motion_cost;
        std::cout<<"candidate gain: "<<soln->candidate.predicted_gain<<" motion cost: "<<soln->candidate.motion_cost<<" total score: "<< soln->candidate.total_score<<std::endl;
        const double score = soln->candidate.total_score;//.predicted_gain; // -0.15*soln->candidate.motion_cost;
        if(score>best_score){
            best_score = score;
            best = std::move(soln);
        }
    }
    return best;

}

double NBVPlanner::predictedInformationGain(
        const ProfileInspectionRequest& request,
        const ProfileViewSolution& solution,
        const hb_perception::ProfileObservationModel& observation
    )const{

        return predictedCoverageGain(request,solution.candidate.nominal_cam_pose,observation);
        

    }


moveit_msgs::msg::Constraints NBVPlanner::makePoseConstraints(
            const ProfileInspectionRequest& request,
            const ProfileViewCandidate candidate
        )const{
            moveit_msgs::msg::Constraints constraints;
            const std::string& ref_frame = robot_model_->getModelFrame();

            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header.frame_id = ref_frame;
            pos_constraint.link_name = camera_link_;
            pos_constraint.weight = 1.0;
            pos_constraint.target_point_offset.x=0.0;
            pos_constraint.target_point_offset.y=0.0;
            pos_constraint.target_point_offset.z=0.0;


            shape_msgs::msg::SolidPrimitive sphere;
            sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            sphere.dimensions.resize(1);
            sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = request.pos_tol;
            geometry_msgs::msg::Pose sphere_pose;
            sphere_pose.position.x = candidate.nominal_cam_pose.translation().x();
            sphere_pose.position.y = candidate.nominal_cam_pose.translation().y();
            sphere_pose.position.z = candidate.nominal_cam_pose.translation().z();
            sphere_pose.orientation.w = 1.0;

            pos_constraint.constraint_region.primitives.push_back(sphere);
            pos_constraint.constraint_region.primitive_poses.push_back(sphere_pose);

            constraints.position_constraints.push_back(pos_constraint);
   

            //orientation stuff
            moveit_msgs::msg::OrientationConstraint orn_constraint;
            orn_constraint.header.frame_id = ref_frame;
            orn_constraint.link_name = camera_link_;

            const Eigen::Quaterniond q(candidate.nominal_cam_pose.linear());
            orn_constraint.orientation.x = q.x();
            orn_constraint.orientation.y = q.y();
            orn_constraint.orientation.z = q.z();
            orn_constraint.orientation.w = q.w();

            orn_constraint.absolute_x_axis_tolerance = M_PI;
            orn_constraint.absolute_y_axis_tolerance = request.ang_tol;
            orn_constraint.absolute_z_axis_tolerance = request.ang_tol;

            orn_constraint.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
            orn_constraint.weight = 1.0;
            constraints.orientation_constraints.push_back(orn_constraint);
            return constraints;

        }



bool NBVPlanner::sampleConstraint(const moveit_msgs::msg::Constraints& constraints,
            moveit::core::RobotState& state,
            const moveit::core::RobotState& reference_state,
            const planning_scene::PlanningSceneConstPtr& p_scene,
            const ProfileInspectionRequest& request
            )const{

                if(!p_scene){
                    std::cerr<<"no planning scene"<<std::endl;
                    return false;
                }
                moveit::core::RobotState clean_reference(reference_state);
                clean_reference.update();

                auto sampler = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(
                    p_scene,
                    planning_group_,
                    constraints
                );


                if (!sampler || !sampler->isValid())return false;

        


                auto ik_sampler = std::dynamic_pointer_cast<constraint_samplers::IKConstraintSampler>(sampler);
                if(!ik_sampler){
                    std::cerr<<"ik sampler fucked"<<std::endl;
                    return false;
                }

                moveit::core::RobotState best(clean_reference);
                double best_cost  =std::numeric_limits<double>::infinity();
                double highest_cost = 0.0;
                bool found = false;
                kinematic_constraints::KinematicConstraintSet constraint_set(robot_model_);

                for (unsigned int i=0; i<request.num_samples; i++){
                   
                    Eigen::Vector3d cand_position;
                    Eigen::Quaterniond cand_orn;



                    if(!ik_sampler->samplePose(cand_position, cand_orn, clean_reference,1)){
                        
                        std::cerr<<"sampler couldnt find pose"<<std::endl;
                        continue;
                    } 
                    Eigen::Isometry3d cand_pose = Eigen::Isometry3d::Identity();
                    cand_pose.translation() = cand_position;
                    cand_pose.linear() = cand_orn.toRotationMatrix();
                    moveit::core::RobotState ik_candidate(clean_reference);
                    std::vector<double> consistency_limits(joint_model_group_->getVariableCount(),
                    2.0);
                    if(!ik_candidate.setFromIK(joint_model_group_,cand_pose,camera_link_,consistency_limits,0.05)){
                        // std::cerr<<"ik failed for sample "<<i<<std::endl;
                        continue;
                    }
                    ik_candidate.update();

                    if(!ik_candidate.satisfiesBounds(joint_model_group_)) continue;
                    if(p_scene->isStateColliding(ik_candidate,planning_group_)) continue;
                    if(!p_scene->isStateConstrained(ik_candidate,constraints)) continue;


                    double cost = dq_cost(clean_reference, ik_candidate, joint_model_group_);
                    if (cost<best_cost){
                        best_cost = cost;
                        best = ik_candidate;
                        found = true;
                    }
                    if (cost>highest_cost) highest_cost = cost;

                }
                if(!found){
                    return false;
                }
            std::cout<<"sampling candidates max cost:  "<<highest_cost<<"best cost: "<<best_cost<<std::endl;

            state = best;
            return true;

            }


 std::optional<ProfileViewSolution> NBVPlanner::solveViewConstraints(
    const moveit::core::RobotState& seed_state,
            const ProfileViewCandidate& candidate,
            const ProfileInspectionRequest& request,
            const planning_scene::PlanningSceneConstPtr& p_scene
        )const{

            moveit::core::RobotState state(seed_state);

            if(!sampleConstraint(candidate.view_constraints,
                    state,
                    seed_state,
                    p_scene,
                    request)){
                    std::cout<<"sampling failed"<<std::endl;
                    return std::nullopt;
                }
            state.update();

            if(!state.satisfiesBounds(joint_model_group_)){
                return std::nullopt;
            }
            ProfileViewSolution solution(state);
            solution.candidate = candidate;
            solution.candidate.actual_cam_pose = state.getGlobalLinkTransform(camera_link_);
            solution.candidate.motion_cost = dq_cost(seed_state, state, joint_model_group_);
            
            return solution;
        }
     
 }