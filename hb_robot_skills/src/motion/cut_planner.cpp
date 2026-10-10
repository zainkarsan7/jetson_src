#include "hb_robot_skills/motion/cut_planner.hpp"
#include "moveit/kinematic_constraints/kinematic_constraint.h"
#include "moveit/kinematic_constraints/utils.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hb_robot_skills::motion{

CutPlanner::CutPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            std::string plasma_link
        ): robot_model_(std::move(robot_model)),
            joint_model_group_(nullptr),
            planning_group_(std::move(planning_group)),
            plasma_link_(std::move(plasma_link))
        {
           if(!robot_model_){
                throw std::invalid_argument("Cut Planner: robot model null");

            }

            joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
            if(!joint_model_group_){
                throw std::invalid_argument("Cut Planner: planning group doesnt exist");

            }

            if(!robot_model_->hasLinkModel(plasma_link_)){
                throw std::invalid_argument("Cut Planner: plasma link doesnt exist");

            }
        }

std::optional<CutSegment> CutPlanner::selectWebCandidate(
            std::vector<CutSegment>& candidates,
            const moveit::core::RobotState& start_state,
            const planning_scene::PlanningSceneConstPtr& p_scene
        ) const{
    if(candidates.empty()){
        std::cerr<<"Looking for Best Web: no candidates"<<std::endl;
        return std::nullopt;
    }
    const Eigen::Vector3f current_tcp_pose =start_state.getGlobalLinkTransform(plasma_link_).translation().cast<float>();

    float best_score = -std::numeric_limits<float>::infinity();
    std::optional<CutSegment> best_web;
    for (auto& candidate: candidates){
        if (candidate.type != hb_perception::ProfileCutFeatureType::Web){
            continue;
        }

        if(!solveSegmentConstraints(candidate,start_state,p_scene)){
            std::cout<<"Looking for Best Web: failed planning "<<candidate.name<<std::endl;
            continue;
        }


        float score = approachScore(candidate,current_tcp_pose);

        std::cout<<"Looking for Best Web: "<<candidate.name<<" approach score "<<score<<std::endl;
        if (!best_web || score > best_score){
            best_web = candidate;
            best_score = score;
        }
    }
    return best_web;

    }
    
bool CutPlanner::solveSegmentIK(CutSegment &segment, const moveit::core::RobotState& seed_state) const{
    moveit::core::RobotState state(seed_state);

    if(!state.setFromIK(joint_model_group_,
    segment.approach_pose,plasma_link_,0.1)){
        std::cout<<"failed approach planning "<<segment.name.c_str()<<std::endl;
        return false;}
    state.update();
    segment.approach_state = std::make_shared<moveit::core::RobotState>(state);

    if(!state.setFromIK(joint_model_group_,
    segment.start_pose,plasma_link_,0.1)){
        std::cout<<"failed start planning "<<segment.name.c_str()<<std::endl;
        return false;}
    state.update();
    segment.start_state = std::make_shared<moveit::core::RobotState>(state);

    if(!state.setFromIK(joint_model_group_,
    segment.end_pose,plasma_link_,0.1)){
        std::cout<<"failed end planning "<<segment.name.c_str()<<std::endl;
        return false;}
    state.update();
    segment.end_state = std::make_shared<moveit::core::RobotState>(state);
    
    if(!state.setFromIK(joint_model_group_,
    segment.retract_pose,plasma_link_,0.1)){
        std::cout<<"failed retract planning "<<segment.name.c_str()<<std::endl;
        return false;}
    state.update();
    segment.retract_state = std::make_shared<moveit::core::RobotState>(state);
    
    return true;
    
}

double CutPlanner::dq_cost(const moveit::core::RobotState& a, const moveit::core::RobotState& b)const{
    const auto& joints = joint_model_group_->getActiveJointModels();
    double cost = 0.0;
    for (const auto * joint: joints){
        const double d = joint->distance(a.getJointPositions(joint),b.getJointPositions(joint));
        cost+= d*d; 
    }
    return cost;
}

collision_detection::CollisionResult CutPlanner::checkCol(const moveit::core::RobotState& state, 
const planning_scene::PlanningSceneConstPtr p_scene)const{
    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.group_name = planning_group_;
    col_req.contacts = true;
    col_req.max_contacts=  100;
    col_req.max_contacts_per_pair = 25;

    p_scene->checkCollision(col_req,col_res,state);
    return col_res;

}


bool CutPlanner::refineCollision(moveit::core::RobotState& candidate,
            const Eigen::Isometry3d& nominal_tcp_pose,
            const moveit_msgs::msg::Constraints& constraints,
            const collision_detection::CollisionResult collision_state,
            const planning_scene::PlanningSceneConstPtr& p_scene) const{
                
                /// decide from collision state if its a torch or body collision
                // two strategies if torch -> nudge point away within constraint 
                // if body, use contact jacobean solve min distance adjustment problem

                moveit::core::RobotState trial(candidate);
                std::vector<RobotWorkpieceContact> contacts =  extractWorkpieceContacts(collision_state);
                constexpr double clearance = 0.002;
                // get worst interms of depth
                const auto worst = std::max_element(contacts.begin(),contacts.end(),
                [](const auto& a, const auto& b){
                    return a.depth< b.depth;
                });
                
                bool is_torch = false;
                Eigen::Vector3d torch_outward = Eigen::Vector3d::Zero();
                double max_torch_collision_depth = 0.0;
                for (const auto & c : contacts){
                    if(c.robot_link == plasma_link_){
                        is_torch = true;
                        //collect all torch norms
                        torch_outward += c.normal_world.normalized();
                        if (c.depth > max_torch_collision_depth){
                            max_torch_collision_depth = c.depth;
                        }
                    }
                    

                }

                if(is_torch){
                    // do some profile standoff nudging
                    std::cout<<"torch in collision"<<std::endl;
                    torch_outward.normalize();
                    torch_outward *= max_torch_collision_depth + clearance;
                    Eigen::Isometry3d trial_pose =  nominal_tcp_pose;
                    trial_pose.translation() = max_torch_collision_depth*torch_outward;

                    std::vector<double> consistency_limits(joint_model_group_->getVariableCount(),
                    2.0);

                    if(!trial.setFromIK(joint_model_group_,trial_pose,plasma_link_,consistency_limits,0.05)){
                        std::cerr<<"ik trial pose failed" <<std::endl;
                        return false;
                    }
                }
                else{
                const auto* link = robot_model_->getLinkModel(worst->robot_link);
                const Eigen::Vector3d contact_pt = worst->pos_world;
                const Eigen::Isometry3d& T_World_Link = candidate.getGlobalLinkTransform(link);
                const Eigen::Vector3d contact_local = T_World_Link.inverse() * contact_pt;


                Eigen::MatrixXd J_contact;
                if(!candidate.getJacobian(joint_model_group_,
                    link,contact_local, J_contact)){
                        std::cerr<<"couldnt get contact jacobian"<<std::endl;
                        return false;
                    };
                const Eigen::MatrixXd J_c = J_contact.topRows(3);

                Eigen::MatrixXd J_tcp;
                const auto* tcp_link = robot_model_->getLinkModel(plasma_link_);
                if(!candidate.getJacobian(joint_model_group_,tcp_link,Eigen::Vector3d::Zero(),
                J_tcp)){
                    std::cerr<<"couldnt get tcp jacobian"<<std::endl;
                    return false;
                };

                const Eigen::Index n = J_c.cols();
                Eigen::Matrix<double, 6,1> tcp_weights;
                tcp_weights<<100.0,100.0,100.0,1.0,1.0,1.0;
                Eigen::Matrix<double,6,6> W_t = tcp_weights.asDiagonal();

                constexpr double lambda_tcp = 1.0;
                constexpr double lambda_q = 1e-3;
                
                // delta p is the normal direction * however deep the contact is plus some tiny clearance
                const Eigen::Vector3d delta_p = (std::max(0.0,worst->depth) + clearance)* worst->normal_world;


                // do a least squares min solve on Jc^T J_c + lambda_t * J_tcp^T * W* J_tcp + conditioner q
                
                Eigen::MatrixXd H = J_c.transpose() * J_c + lambda_tcp * J_tcp.transpose() * W_t * J_tcp + lambda_q * Eigen::MatrixXd::Identity(n,n);
                Eigen::VectorXd b = J_c.transpose() * delta_p;
                Eigen::VectorXd dq = H.ldlt().solve(b);
                if(!dq.allFinite()){
                    std::cerr<<"solve unstable"<<std::endl;
                    return false;
                }

                std::vector<double> q;
                candidate.copyJointGroupPositions(joint_model_group_,q);
                if(q.size()!=6){
                    std::cerr<<"not right number of joints in q"<<std::endl;
                    return false;
                }

                constexpr double max_joint_step = 0.03;
                const double max_abs = dq.cwiseAbs().maxCoeff();
                if(max_abs>max_joint_step){
                    dq *= max_abs/max_joint_step;
                }
                for(std::size_t i = 0; i<q.size();i++){
                    q[i] += dq[static_cast<Eigen::Index>(i)];
                }
                trial.setJointGroupPositions(joint_model_group_,q);
            }
                
                
                trial.update();
                if(!trial.satisfiesBounds(joint_model_group_)){
                    std::cerr<<"applying dq didnt work, out of bounds"<<std::endl;
                    return false;
                }

                // check kinematic constraint satisfaction

                kinematic_constraints::KinematicConstraintSet constraint_set(robot_model_);
                constraint_set.add(constraints, p_scene->getTransforms());
                if(!constraint_set.decide(trial).satisfied){
                    std::cout<<"contact nudging pushes out of constraint space"<<std::endl;
                    return false;
                }


                // another collision detection round:
                auto res = checkCol(trial,p_scene);
                if(!res.collision){
                    candidate=  trial;
                    return true;
                }
                return false; // or iterate a bunch more times and require depth decreases

                

}


bool CutPlanner::sampleConstraint(const moveit_msgs::msg::Constraints& constraints,
            moveit::core::RobotState& state,
            const moveit::core::RobotState& reference_state,
            const planning_scene::PlanningSceneConstPtr& p_scene
            )const{

                if(!p_scene){
                    std::cerr<<"no planning scene"<<std::endl;
                    return false;
                }
                auto sampler = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(
                    p_scene,
                    planning_group_,
                    constraints
                );

                auto ik_sampler = std::dynamic_pointer_cast<constraint_samplers::IKConstraintSampler>(sampler);
                if(!ik_sampler){
                    std::cerr<<"ik sampler fucked"<<std::endl;
                    return false;
                }

                if(!sampler){
                    std::cerr<<"samplers fucked"<<std::endl;
                    return false;
                }
                
                if(!sampler->isValid()){
                    std::cerr<<"samplers invalid and fucked"<<std::endl;
                    return false;
                }

                std::cout<<"reference colliding "<<p_scene->isStateColliding(reference_state,planning_group_)<<std::endl;
                std::cout<<"world objects: "<<p_scene->getWorld()->size()<<std::endl;

                moveit::core::RobotState best(reference_state);
                double best_cost  =std::numeric_limits<double>::infinity();
                double highest_cost = 0.0;
                bool found = false;

                for (unsigned int i=0; i<100; i++){
                   

                    Eigen::Vector3d cand_position;
                    Eigen::Quaterniond cand_orn;

                    if(!ik_sampler->samplePose(cand_position, cand_orn, reference_state,1)){
                        
                        std::cerr<<"sampler couldnt find pose"<<std::endl;
                        continue;
                    } 
                    Eigen::Isometry3d cand_pose = Eigen::Isometry3d::Identity();
                    cand_pose.translation() = cand_position;
                    cand_pose.linear() = cand_orn.toRotationMatrix();
                    moveit::core::RobotState candidate(reference_state);
                    std::vector<double> consistency_limits(joint_model_group_->getVariableCount(),
                    2.0);
                    if(!candidate.setFromIK(joint_model_group_,cand_pose,plasma_link_,consistency_limits,0.05)){
                        // std::cerr<<"ik failed" <<std::endl;
                        continue;
                    }
                    candidate.update();
                    // collision stuff:
                    auto col_res = checkCol(candidate,p_scene);
                    if(col_res.collision){
                        for (const auto& [pair, contacts]: col_res.contacts){
                            std::cout<<"    "<<
                            pair.first<<" <-> "<<
                            pair.second<< ": "<<contacts.size()<<"contacts"<<std::endl;
                        }
                    }



                    // if(!sampler->sample(candidate, reference_state,1)) continue;
                    double cost = CutPlanner::dq_cost(reference_state, candidate);
                    if (cost<best_cost){
                        best_cost = cost;
                        best = candidate;
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



                // constexpr unsigned int max_attempts=  100;
                // if(!sampler->sample(state,reference_state,max_attempts)){
                //     return false;
                // }
                // state.update();
                // return true;

            }




bool CutPlanner::solveSegmentConstraints(CutSegment& segment, 
            const moveit::core::RobotState& seed_state, 
            const planning_scene::PlanningSceneConstPtr& p_scene) const{

                moveit::core::RobotState state(seed_state);

                if(!sampleConstraint(segment.approach_constraints,
                    state,
                    seed_state,
                    p_scene)){
                    std::cout<<"Solving Constraints: failed approach planning for "<<segment.name.c_str()<<std::endl;
                    if(debug_vis_callback_)debug_vis_callback_(segment);
                    
                    
                    return false;}
                state.update();
                segment.approach_state = std::make_shared<moveit::core::RobotState>(state);
                if(debug_vis_callback_)debug_vis_callback_(segment);

                const moveit::core::RobotState approach_reference(state);
                if(!sampleConstraint(segment.start_constraints,
                state, approach_reference, p_scene)){
                    std::cout<<"Solving Constraints: failed start planning for "<<segment.name.c_str()<<std::endl;
                    if(debug_vis_callback_)debug_vis_callback_(segment);
                    return false;}
                state.update();
                segment.start_state = std::make_shared<moveit::core::RobotState>(state);
                if(debug_vis_callback_)debug_vis_callback_(segment);

                // segment constraint debug::
                const Eigen::Isometry3d& actual = state.getGlobalLinkTransform(plasma_link_);
                Eigen::Vector3d actual_z = actual.linear().col(2);
                Eigen::Vector3d nominal_z = segment.start_pose.linear().col(2);
                std::cout<<"Solving Constraints: TCP pointing error "<<
                std::acos(std::clamp(
                    actual_z.dot(nominal_z),-1.0,1.0))*180.0/M_PI<<" deg"<<std::endl;
                
                const moveit::core::RobotState start_reference(state);
                if(!sampleConstraint(segment.end_constraints,
                    state,
                start_reference, 
                p_scene)){
                    std::cout<<"Solving Constraints: failed end planning for "<<segment.name.c_str()<<std::endl;
                    
                   if(debug_vis_callback_)debug_vis_callback_(segment);
                    return false;
                    
                
                }
                state.update();
                segment.end_state = std::make_shared<moveit::core::RobotState>(state);
                if(debug_vis_callback_)debug_vis_callback_(segment);

                const moveit::core::RobotState end_reference(state);
                if(!sampleConstraint(segment.retract_constraints,
                state,end_reference,p_scene)){
                    std::cout<<"Solving Constraints: failed retract planning for "<<segment.name.c_str()<<std::endl;
                    
                    if(debug_vis_callback_)debug_vis_callback_(segment);
                    
                    return false;}
                state.update();
                segment.retract_state = std::make_shared<moveit::core::RobotState>(state);
                if(debug_vis_callback_)debug_vis_callback_(segment);
                
                std::cout<<"Solved Constraints for "<<segment.name.c_str()<<std::endl;
                return true;


            }



std::optional<CutPlan> CutPlanner::plan(
    const hb_robot_interfaces::msg::ProfileEstimate& estimate,
    const hb_perception::ProfileModel& profile,
    const CutRequest& request,
    const moveit::core::RobotState& start_state,
    const planning_scene::PlanningSceneConstPtr& p_scene
) const{


    if(estimate.profile_name!=profile.name){
        std::cerr<<"profile mismatch, estimate is "
        <<estimate.profile_name<<" and profile is "
        <<profile.name<<std::endl;

        return std::nullopt;
    }
    if(profile.cut_features.empty()){
        std::cerr<<"no cut features in profile"<<std::endl;
        return std::nullopt;

    }

    std::cout<<"incoming state is dirty: "<<start_state.dirty()<<std::endl;

    moveit::core::RobotState current_state(start_state);
    current_state.update();


    std::cout<<"incoming state is dirty: "<<current_state.dirty()<<std::endl;

    Eigen::Isometry3d world_from_profile_d;
    tf2::fromMsg(estimate.pose,world_from_profile_d);
    const Eigen::Isometry3f world_from_profile =world_from_profile_d.cast<float>();
    CutPlan plan;
    plan.profile_name=  profile.name;
    plan.segments.reserve(profile.cut_features.size());

    std::vector<CutSegment> flange_segments;
    std::vector<CutSegment> web_segments;

    for (const auto& feature : profile.cut_features){
        const float length = (feature.end - feature.start).norm();
        if(length < 1e-5f){
            continue;
        }

        //make profile segments into cut segments
        CutSegment candidate_cut = makeSegment(feature, world_from_profile,request);

        switch(feature.type){
            case hb_perception::ProfileCutFeatureType::Flange:
                flange_segments.push_back(std::move(candidate_cut));
                break;
            case hb_perception::ProfileCutFeatureType::Web:
                web_segments.push_back(std::move(candidate_cut));
                break;
            default:
                break;
            }
        }

        std::vector<CutSegment> debug_segments;
        debug_segments.reserve(flange_segments.size() + web_segments.size());
        debug_segments.insert(debug_segments.end(),flange_segments.begin(),flange_segments.end());
        debug_segments.insert(debug_segments.end(),web_segments.begin(),web_segments.end());

        auto best_web_segment = selectWebCandidate(web_segments,current_state,p_scene);
        if(!best_web_segment){
            return std::nullopt;
        }

        for (auto& flange: flange_segments){
            if(!solveSegmentConstraints(flange,current_state,p_scene)){
                std::cerr<<"failed flange planning "<<flange.name<<std::endl;
                return std::nullopt;
            }
            plan.segments.push_back(flange);
        }

        plan.segments.push_back(*best_web_segment);
       
  
        if (plan.segments.empty()){
            std::cerr<<"empty plan"<<std::endl; 
            return std::nullopt;
        }
            return plan;

    }

moveit_msgs::msg::Constraints CutPlanner::makeBoxConstraints(
            const CutSegment& segment,
            const double pos_tol, 
            const double ang_tol
        )const{

            moveit_msgs::msg::Constraints constraints;
            constraints.name = "use_equality_constraints";
            const std::string& ref_frame = robot_model_->getModelFrame();
            moveit_msgs::msg::PositionConstraint box_constraint_;
            box_constraint_.header.frame_id = ref_frame;
            box_constraint_.link_name = plasma_link_;
            box_constraint_.weight= 1.0;
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            Eigen::Vector3d sp = segment.start_pose.translation();
            Eigen::Vector3d ep = segment.end_pose.translation();
            
            const double length = (sp-ep).norm();
            Eigen::Isometry3d box_center = segment.start_pose;
            box_center.translation() = 0.5*(sp+ep);
            constexpr double eq_width = 0.0005;
            box.dimensions = {length + 0.01, eq_width,eq_width};
            box_constraint_.constraint_region.primitives.push_back(box);
            box_constraint_.constraint_region.primitive_poses.push_back(
                tf2::toMsg(box_center)
            );

            constraints.position_constraints.push_back(std::move(box_constraint_));
            return constraints;
        }


moveit_msgs::msg::Constraints CutPlanner::makePoseConstraints(
            const Eigen::Isometry3d nominal_pose,
            const double pos_tol, 
            const double ang_tol
        )const{

            moveit_msgs::msg::Constraints constraints;
            const std::string& ref_frame = robot_model_->getModelFrame();

            //position stuff
            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header.frame_id = ref_frame;
            pos_constraint.link_name = plasma_link_;
            pos_constraint.weight = 1.0;
            pos_constraint.target_point_offset.x=0.0;
            pos_constraint.target_point_offset.y=0.0;
            pos_constraint.target_point_offset.z=0.0;

            shape_msgs::msg::SolidPrimitive sphere;
            sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            sphere.dimensions.resize(1);
            sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = pos_tol;
            geometry_msgs::msg::Pose sphere_pose;
            sphere_pose.position.x = nominal_pose.translation().x();
            sphere_pose.position.y = nominal_pose.translation().y();
            sphere_pose.position.z = nominal_pose.translation().z();
            sphere_pose.orientation.w = 1.0;

            pos_constraint.constraint_region.primitives.push_back(sphere);
            pos_constraint.constraint_region.primitive_poses.push_back(sphere_pose);

            constraints.position_constraints.push_back(pos_constraint);
   
            //orientation stuff
            moveit_msgs::msg::OrientationConstraint orn_constraint;
            orn_constraint.header.frame_id = ref_frame;
            orn_constraint.link_name = plasma_link_;

            const Eigen::Quaterniond q(nominal_pose.linear());
            orn_constraint.orientation.x = q.x();
            orn_constraint.orientation.y = q.y();
            orn_constraint.orientation.z = q.z();
            orn_constraint.orientation.w = q.w();

            orn_constraint.absolute_x_axis_tolerance = ang_tol;
            orn_constraint.absolute_y_axis_tolerance = ang_tol;
            orn_constraint.absolute_z_axis_tolerance = M_PI;

            orn_constraint.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
            orn_constraint.weight = 1.0;
            constraints.orientation_constraints.push_back(orn_constraint);
            return constraints;
        }

Eigen::Isometry3d CutPlanner::makeToolPose(
            const Eigen::Vector3f& position,
            const Eigen::Vector3f& tangent,
            const Eigen::Vector3f& surface_normal)const{
            
        Eigen::Vector3d z_ax = surface_normal.cast<double>();
        z_ax.normalize();

        Eigen::Vector3d x_ax = tangent.cast<double>();
        x_ax -= x_ax.dot(z_ax)*z_ax;
        x_ax.normalize();
        Eigen::Vector3d y_ax = z_ax.cross(x_ax).normalized();
        x_ax = y_ax.cross(z_ax).normalized();

        Eigen::Isometry3d tool_pose  = Eigen::Isometry3d::Identity();
        tool_pose.linear().col(0) = x_ax;
        tool_pose.linear().col(1) = y_ax;
        tool_pose.linear().col(2) = z_ax;
        tool_pose.translation() = position.cast<double>();
            
        return tool_pose;
        }
float CutPlanner::approachScore(const CutSegment& segment, const Eigen::Vector3f& tcp_pos)const{
    if(segment.points.size()<2){
        std::cerr<<"segment doesnt have enough points"<<std::endl;
        return -std::numeric_limits<float>::infinity();
    }

    const Eigen::Vector3f midpt = 0.5f * (segment.points.front().pos + segment.points.back().pos);
    Eigen::Vector3f to_tcp = tcp_pos - midpt;
    if(to_tcp.norm() < 1e-6){
        std::cerr<<"to_tcp has zero norm"<<std::endl;
        return -std::numeric_limits<float>::infinity();
    }
    to_tcp.normalize();
    /**
     * +1 TCP if on the outward side, 0 if tangent,-1 if behind
     */
    return segment.points.front().srf_norm.dot(to_tcp);
}


CutSegment CutPlanner::makeSegment(
    const hb_perception::ProfileCutFeature& feature,
    const Eigen::Isometry3f& world_from_profile,
    const CutRequest request
)const{

    CutSegment segment;
    segment.name = feature.name;
    segment.type = feature.type;
    Eigen::Vector2f tangent_2d = feature.end - feature.start;
    tangent_2d.normalize();
    Eigen::Vector3f tangent_profile(tangent_2d.x(),tangent_2d.y(),0.0f);
    
    Eigen::Vector2f norm_2d = feature.outward_normal.normalized();
    Eigen::Vector3f norm_profile(norm_2d.x(),norm_2d.y(),0.0f);
    norm_profile.normalize();

    // transform to world;
    Eigen::Vector3f norm_world = world_from_profile.linear() * norm_profile;
    Eigen::Vector3f tan_world = world_from_profile.linear() * tangent_profile;
        
    Eigen::Vector3f start_world = world_from_profile* Eigen::Vector3f(feature.start.x(),feature.start.y(),0.0f);
    start_world += request.standoff * norm_world;
    segment.start_pose = makeToolPose(start_world,tan_world,norm_world);
    segment.start_constraints = makePoseConstraints(segment.start_pose,request.pos_tol,request.ang_tol);

    Eigen::Vector3f approach_world = start_world;
    approach_world += request.approach_dist * norm_world;
    segment.approach_pose = makeToolPose(approach_world,tan_world,norm_world);
    segment.approach_constraints = makePoseConstraints(segment.approach_pose,request.pos_tol,request.ang_tol);

    Eigen::Vector3f end_world = world_from_profile* Eigen::Vector3f(feature.end.x(),feature.end.y(),0.0f);
    end_world += request.standoff * norm_world;
    segment.end_pose = makeToolPose(end_world,tan_world,norm_world);
    segment.end_constraints = makePoseConstraints(segment.end_pose,request.pos_tol,request.ang_tol);
    
    Eigen::Vector3f retract_world = end_world;
    retract_world += request.retract_dist  * norm_world;
    segment.retract_pose = makeToolPose(retract_world,tan_world,norm_world);
    segment.retract_constraints = makePoseConstraints(segment.retract_pose,request.pos_tol,request.ang_tol);

    CutPathPoint start;
    start.pos = start_world;
    start.tangent = tan_world;
    start.srf_norm = norm_world;

    CutPathPoint end;
    end.pos = end_world;
    end.tangent = tan_world;
    end.srf_norm = norm_world;

    segment.points.push_back(start);
    segment.points.push_back(end);

    return segment;
}
    
}