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
#include "hb_robot_skills/motion/cut_process_trajectory_generator.hpp"

namespace hb_robot_skills::motion{

CutPlanner::CutPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            std::string plasma_link,
            CutPlannerOptions options
        ): robot_model_(std::move(robot_model)),
            joint_model_group_(nullptr),
            planning_group_(std::move(planning_group)),
            plasma_link_(std::move(plasma_link)), options_(options)
        {
           if (!options_.max_pose_samples || !options_.feasible_samples ||
               options_.feasible_samples > options_.max_pose_samples ||
               !options_.refinement_iterations || !options_.waypoint_beam_width ||
               options_.waypoint_beam_width > options_.max_pose_samples ||
               !std::isfinite(options_.ik_timeout_s) || options_.ik_timeout_s <= 0 ||
               !std::isfinite(options_.ik_consistency_rad) || options_.ik_consistency_rad <= 0 ||
               !std::isfinite(options_.repair_joint_step_rad) || options_.repair_joint_step_rad <= 0 ||
               !std::isfinite(options_.repair_roll_step_rad) || options_.repair_roll_step_rad <= 0)
               throw std::invalid_argument("Invalid cut geometry sampling options");
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

bool CutPlanner::constraintsSatisfied(moveit::core::RobotState& trial,
    const moveit_msgs::msg::Constraints& constraints,
    const planning_scene::PlanningSceneConstPtr& p_scene)const{
    
        if(!trial.satisfiesBounds(joint_model_group_)){
            std::cout<<"trial doesnt satisfy model group bounds"<<std::endl;
            return false;
        }
        kinematic_constraints::KinematicConstraintSet constraint_set(robot_model_);
        if(!constraint_set.add(constraints,p_scene->getTransforms())){
            std::cout<<"trial couldnt add constraints"<<std::endl;
            return false;
        }

        if(!constraint_set.decide(trial).satisfied){
            std::cout<<"trial coudlnt satisfy constrinats"<<std::endl;
            return false;
        }
        return true;

    }




bool CutPlanner::proposeJacobeanStep(moveit::core::RobotState& trial,
std::vector<RobotWorkpieceContact>& contacts)const{
    if(contacts.empty()){
        return false;
    }
    const auto worst = std::max_element(contacts.begin(),contacts.end(),
            [](const auto& a, const auto& b){
                return a.depth< b.depth;
            });


            const auto* link = robot_model_->getLinkModel(worst->robot_link);
                if (!link) return false;
                const Eigen::Vector3d contact_pt = worst->pos_world;
                const Eigen::Isometry3d& T_World_Link = trial.getGlobalLinkTransform(link);
                const Eigen::Vector3d contact_local = T_World_Link.inverse() * contact_pt;

                

                Eigen::MatrixXd J_contact;
                if(!trial.getJacobian(joint_model_group_,
                    link,contact_local, J_contact)){
                        std::cerr<<"couldnt get contact jacobian"<<std::endl;
                        return false;
                    };
                const Eigen::MatrixXd J_c = J_contact.topRows(3);

                if (!link || !worst->normal_world.allFinite() || worst->normal_world.norm() < 1e-10) return false;
                Eigen::Vector3d normal = worst->normal_world.normalized();
                // MoveIt Jacobians use the parent frame of the group's root joint.
                const auto* parent = joint_model_group_->getJointModels().front()->getParentLinkModel();
                if (parent) normal = trial.getGlobalLinkTransform(parent).linear().transpose() * normal;
                const Eigen::RowVectorXd J_normal = normal.transpose()* J_c;

                constexpr double clearance = 0.002;
                const double sep_dist = std::max(0.0,worst->depth) + clearance;
                // sep dist is the normal direction * however deep the contact is plus some tiny clearance
                      
                Eigen::MatrixXd J_tcp;
                const auto* tcp_link = robot_model_->getLinkModel(plasma_link_);
                if(!trial.getJacobian(joint_model_group_,tcp_link,Eigen::Vector3d::Zero(),
                J_tcp)){
                    std::cerr<<"couldnt get tcp jacobian"<<std::endl;
                    return false;
                };

                const Eigen::Index n = J_c.cols();

               

                // Move the contact in the null space of TCP translation.
                // Orientation may change; translation is projected back by IK
                // and rechecked against the original constraints before acceptance.
                const Eigen::MatrixXd J_position = J_tcp.topRows(3);
                const Eigen::MatrixXd inverse = J_position.completeOrthogonalDecomposition().pseudoInverse();
                const Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(n,n) - inverse * J_position;
                const Eigen::VectorXd direction = nullspace * J_normal.transpose();
                const double gain = (J_normal * direction)(0);
                if (!std::isfinite(gain) || gain < 1e-10) return false;
                Eigen::VectorXd dq = direction * (sep_dist / gain);
                if(!dq.allFinite()){
                    std::cerr<<"solve unstable"<<std::endl;
                    return false;
                }

                std::vector<double> q;
                trial.copyJointGroupPositions(joint_model_group_,q);
                if(q.size()!=static_cast<std::size_t>(dq.size())){
                    std::cerr<<"not right number of joints in q"<<std::endl;
                    return false;
                }

                const double max_joint_step = options_.repair_joint_step_rad;
                const double max_abs = dq.cwiseAbs().maxCoeff();
                if(max_abs>max_joint_step){
                    dq *= max_joint_step/max_abs;
                }
                for(std::size_t i = 0; i<q.size();i++){
                    q[i] += dq[static_cast<Eigen::Index>(i)];
                }
                trial.setJointGroupPositions(joint_model_group_,q);
    return true;    


} 



bool CutPlanner::refineCollision(moveit::core::RobotState& candidate,
    const Eigen::Isometry3d& nominal_tcp_pose,
    const moveit_msgs::msg::Constraints& constraints,
    const collision_detection::CollisionResult& collision_state,
    const planning_scene::PlanningSceneConstPtr& p_scene) const
{
    kinematic_constraints::KinematicConstraintSet constraint_set(robot_model_);
    if (!constraint_set.add(constraints, p_scene->getTransforms())) return false;
    const auto admissible = [&](const moveit::core::RobotState& state) {
        return state.satisfiesBounds(joint_model_group_) && constraint_set.decide(state).satisfied;
    };
    const auto merit = [](const collision_detection::CollisionResult& collision) {
        if (!collision.collision) return 0.0;
        const auto contacts = extractWorkpieceContacts(collision);
        if (contacts.empty()) return std::numeric_limits<double>::infinity();
        double total = 0.0, deepest = 0.0;
        for (const auto& contact : contacts) {
            const double depth = std::max(0.0, contact.depth);
            total += depth;
            deepest = std::max(deepest, depth);
        }
        return 1e-9 + deepest + total / contacts.size();
    };
    moveit::core::RobotState current(candidate);
    auto collision = collision_state;
    for (std::size_t iter = 0; iter < options_.refinement_iterations; ++iter) {
        if (!collision.collision && admissible(current)) { candidate = current; return true; }
        auto contacts = extractWorkpieceContacts(collision);
        if (contacts.empty()) return false;
        double best_merit = merit(collision);
        std::optional<moveit::core::RobotState> best;
        collision_detection::CollisionResult best_collision;
        const auto consider = [&](moveit::core::RobotState seed, Eigen::Isometry3d pose) {
            // Never push the torch away from a contact. All proposals keep the
            // original Cartesian target, irrespective of accumulated iterations.
            pose.translation() = nominal_tcp_pose.translation();
            const std::vector<double> limits(joint_model_group_->getVariableCount(), 0.2);
            if (!seed.setFromIK(joint_model_group_, pose, plasma_link_, limits, options_.ik_timeout_s)) return;
            seed.update();
            if (!admissible(seed)) return;
            const auto result = checkCol(seed, p_scene);
            const double value = merit(result);
            if (value + 1e-12 < best_merit) {
                best_merit = value;
                best = seed;
                best_collision = result;
            }
        };
        // Body-contact step: bounded joint displacement in the translation nullspace.
        moveit::core::RobotState body(current);
        if (proposeJacobeanStep(body, contacts)) {
            body.update();
            consider(body, body.getGlobalLinkTransform(plasma_link_));
        }
        // Torch/body fallback: roll first, then small tilts. Hard orientation
        // constraints are checked on every proposal, not only at the end.
        for (int axis : {2, 0, 1}) {
            for (double sign : {-1.0, 1.0}) {
                auto pose = current.getGlobalLinkTransform(plasma_link_);
                const double angle = sign * (axis == 2 ? options_.repair_roll_step_rad : 0.01);
                pose.linear() = pose.linear() * Eigen::AngleAxisd(angle, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
                consider(current, pose);
            }
            if (best && !best_collision.collision) break;
        }
        if (!best) return false; // no monotone improvement; try another initial orientation
        current = *best;
        collision = best_collision;
        if (collision_debug_vis_callback_) {
            CollisionDebugEvent event(current, nominal_tcp_pose);
            event.iter = iter;
            event.strategy = "fixed-position orientation repair";
            event.label = collision.collision ? "improved contact" : "collision free";
            event.contacts = extractWorkpieceContacts(collision);
            collision_debug_vis_callback_(event);
        }
    }
    // Include the result of the final iteration (previously discarded).
    if (!collision.collision && admissible(current)) { candidate = current; return true; }
    return false;
}


std::vector<moveit::core::RobotStatePtr> CutPlanner::sampleConstraints(
    const moveit_msgs::msg::Constraints& constraints,
    const Eigen::Isometry3d& nominal_pose,
    const moveit::core::RobotState& reference_state,
    const planning_scene::PlanningSceneConstPtr& p_scene,
    std::size_t budget) const
{
    std::vector<moveit::core::RobotStatePtr> accepted;
    if (!p_scene || constraints.orientation_constraints.empty()) return accepted;
    kinematic_constraints::KinematicConstraintSet constraint_set(robot_model_);
    if (!constraint_set.add(constraints, p_scene->getTransforms())) return accepted;
    auto sampler = std::dynamic_pointer_cast<constraint_samplers::IKConstraintSampler>(
        constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(p_scene, planning_group_, constraints));
    if (!sampler || !sampler->isValid()) return accepted;
    std::size_t attempted=0, pose_failed=0, ik_failed=0, constraint_rejected=0,
        collision_rejected=0, repaired=0, duplicates=0;
    const auto cost = [&](const auto& state) { return dq_cost(reference_state, state); };
    for (std::size_t i=0; i<budget; ++i) {
        Eigen::Isometry3d pose=nominal_pose;
        if (i==0) pose.linear()=reference_state.getGlobalLinkTransform(plasma_link_).linear();
        else if (i>=2) {
            // Restore MoveIt's distribution over the COMPLETE orientation
            // constraint; do not invent a roll/tilt distribution with different
            // rotation-vector semantics. Deliberately discard sampled position.
            Eigen::Vector3d ignored_position;
            Eigen::Quaterniond orientation;
            if (!sampler->samplePose(ignored_position, orientation, reference_state, 1)) {
                ++pose_failed; continue;
            }
            pose.linear()=orientation.normalized().toRotationMatrix();
        }
        moveit::core::RobotState trial(reference_state);
        const bool explore=options_.explore_ik_branches && i>=3 && i%4==3;
        if (explore) trial.setToRandomPositions(joint_model_group_);
        else if (!accepted.empty() && i%2==0) trial=*accepted.front();
        trial.update();
        std::optional<moveit::core::RobotState> repair_seed;
        collision_detection::CollisionResult repair_collision;
        double repair_depth=std::numeric_limits<double>::infinity();
        // Reject bad solutions INSIDE IK so a solver supporting callbacks can
        // try another branch for the SAME orientation within its timeout.
        const moveit::core::GroupStateValidityCallbackFn valid =
            [&](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* q) {
                state->setJointGroupPositions(group,q);
                state->update();
                if (!state->satisfiesBounds(group) || !constraint_set.decide(*state).satisfied) {
                    ++constraint_rejected; return false;
                }
                const auto collision=checkCol(*state,p_scene);
                if (!collision.collision) return true;
                ++collision_rejected;
                const auto contacts=extractWorkpieceContacts(collision);
                if (!contacts.empty()) {
                    double depth=0;
                    for (const auto& c:contacts) depth=std::max(depth,c.depth);
                    if (depth<repair_depth) { repair_depth=depth; repair_seed=*state; repair_collision=collision; }
                }
                return false;
            };
        ++attempted;
        const std::vector<double> limits(joint_model_group_->getVariableCount(),options_.ik_consistency_rad);
        bool solved=explore ? trial.setFromIK(joint_model_group_,pose,plasma_link_,options_.ik_timeout_s,valid) :
            trial.setFromIK(joint_model_group_,pose,plasma_link_,limits,options_.ik_timeout_s,valid);
        if (!solved) {
            ++ik_failed;
            // Repair only the shallowest admissible contact, after IK has tried
            // collision-free alternatives. The repair anchor never moves.
            if (repair_seed && refineCollision(*repair_seed,nominal_pose,constraints,repair_collision,p_scene)) {
                trial=*repair_seed; solved=true; ++repaired;
            }
        }
        if (!solved) continue;
        trial.update();
        // Recheck independently: not every third-party IK plugin honors callbacks.
        if (!trial.satisfiesBounds(joint_model_group_) || !constraint_set.decide(trial).satisfied ||
            p_scene->isStateColliding(trial,planning_group_)) continue;
        if (std::any_of(accepted.begin(),accepted.end(),[&](const auto& other) {
            return trial.distance(*other,joint_model_group_)<0.05;
        })) { ++duplicates; continue; }
        accepted.push_back(std::make_shared<moveit::core::RobotState>(trial));
        std::stable_sort(accepted.begin(),accepted.end(),[&](const auto& a,const auto& b){return cost(*a)<cost(*b);});
        // A target count is an early-stop goal, NEVER a minimum success count.
        if (accepted.size()>=options_.feasible_samples) break;
    }
    std::cout << "[CUT GEOMETRY] budget=" << budget << " attempted=" << attempted
        << " pose_failed=" << pose_failed << " IK_failed=" << ik_failed
        << " constraint_rejected=" << constraint_rejected << " collision_rejected=" << collision_rejected
        << " repaired=" << repaired << " duplicates=" << duplicates << " feasible=" << accepted.size() << '\n';
    return accepted;
}

std::optional<ConstraintChain> searchConstraintChain(
    const moveit::core::RobotState& seed,const moveit::core::JointModelGroup* group,
    std::size_t width,std::size_t stage_budget,const ConstraintExpander& expand)
{
    if (!group || !width || stage_budget<width || !expand) return std::nullopt;
    struct Branch { ConstraintChain states{}; double cost=0.0; };
    std::vector<Branch> beam(1);
    for (std::size_t phase=0; phase<4; ++phase) {
        std::vector<Branch> next;
        std::size_t remaining=stage_budget;
        for (std::size_t parent=0; parent<beam.size(); ++parent) {
            const auto& branch=beam[parent];
            const auto& from=phase ? *branch.states[phase-1] : seed;
            // Same TOTAL pose-attempt budget per phase as a single greedy solve;
            // split across branches rather than multiplying by beam width.
            const auto allocation=remaining/(beam.size()-parent);
            remaining-=allocation;
            for (auto& state:expand(phase,from,allocation)) {
                if (!state) continue;
                auto child=branch;
                child.states[phase]=state;
                const double distance=from.distance(*state,group);
                if (!std::isfinite(distance)) continue;
                child.cost+=distance*distance;
                next.push_back(std::move(child));
            }
        }
        if (next.empty()) return std::nullopt;
        std::stable_sort(next.begin(),next.end(),[](const auto& a,const auto& b){return a.cost<b.cost;});
        beam.clear();
        // Distinct end configurations preserve continuation options; duplicates
        // from multiple parents need only the lowest-cost history.
        for (auto& branch:next) {
            if (std::any_of(beam.begin(),beam.end(),[&](const auto& other) {
                return branch.states[phase]->distance(*other.states[phase],group)<0.05;
            })) continue;
            beam.push_back(std::move(branch));
            if (beam.size()>=width) break;
        }
    }
    return beam.front().states;
}

bool CutPlanner::solveSegmentConstraints(CutSegment& segment,
    const moveit::core::RobotState& seed_state,
    const planning_scene::PlanningSceneConstPtr& p_scene) const
{
    const std::array<const moveit_msgs::msg::Constraints*,4> constraints={
        &segment.approach_constraints,&segment.start_constraints,&segment.end_constraints,&segment.retract_constraints};
    const std::array<const Eigen::Isometry3d*,4> poses={
        &segment.approach_pose,&segment.start_pose,&segment.end_pose,&segment.retract_pose};
    const std::array<const char*,4> names={"approach","start","end","retract"};
    auto chain=searchConstraintChain(seed_state,joint_model_group_,options_.waypoint_beam_width,
        options_.max_pose_samples,[&](std::size_t phase,const auto& reference,std::size_t budget) {
            std::cout << "[CUT GEOMETRY] " << segment.name << '/' << names[phase] << '\n';
            return sampleConstraints(*constraints[phase],*poses[phase],reference,p_scene,budget);
        });
    if (!chain) {
        std::cout << "[CUT GEOMETRY] no complete constraint chain for " << segment.name << '\n';
        return false;
    }
    segment.approach_state=(*chain)[0];
    segment.start_state=(*chain)[1];
    segment.end_state=(*chain)[2];
    segment.retract_state=(*chain)[3];
    if (debug_vis_callback_) debug_vis_callback_(segment);
    return true;
}

std::optional<CutPlan> CutPlanner::plan(
    const hb_robot_interfaces::msg::ProfileEstimate& estimate,
    const hb_perception::ProfileModel& profile,
    const CutRequest& request,
    const moveit::core::RobotState& start_state,
    const planning_scene::PlanningSceneConstPtr& p_scene
) const{


    if (!std::isfinite(request.pos_tol) || request.pos_tol <= 0.0 ||
        !std::isfinite(request.ang_tol) || request.ang_tol < 0.0 ||
        !std::isfinite(request.standoff) || !std::isfinite(request.approach_dist) ||
        !std::isfinite(request.retract_dist) || request.approach_dist < 0 || request.retract_dist < 0)
        return std::nullopt;
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
        if(!web_segments.empty() && !best_web_segment){
            return std::nullopt;
        }

        for (auto& flange: flange_segments){
            // if (flange.name != "top_flange"){
            // continue;
            // }
            if(!solveSegmentConstraints(flange,current_state,p_scene)){
                std::cerr<<"failed flange planning "<<flange.name<<std::endl;
                return std::nullopt;
            }
            plan.segments.push_back(flange);
        }

        if(best_web_segment) plan.segments.push_back(*best_web_segment);
       
  
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
            if (!std::isfinite(pos_tol) || pos_tol <= 0.0 || !std::isfinite(ang_tol) || ang_tol < 0.0)
                throw std::invalid_argument("Invalid cut path tolerance");
            constraints.name = "cut_path_constraints";
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
            // Box cross-section inscribed in the requested radial tolerance.
            // No extra length/standoff beyond the original cut endpoints.
            box.dimensions = {std::max(length, 1e-9), std::sqrt(2.0) * pos_tol, std::sqrt(2.0) * pos_tol};
            box_constraint_.constraint_region.primitives.push_back(box);
            box_constraint_.constraint_region.primitive_poses.push_back(
                tf2::toMsg(box_center)
            );

            constraints.position_constraints.push_back(std::move(box_constraint_));
            constraints.orientation_constraints = makePoseConstraints(segment.start_pose, pos_tol, ang_tol).orientation_constraints;
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
    segment.pos_tol = request.pos_tol;
    segment.ang_tol = request.ang_tol;
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

    Eigen::Vector3f torch_feature_start = Eigen::Vector3f(feature.start.x(),feature.start.y(),0.0f);
    Eigen::Vector3f torch_feature_end = Eigen::Vector3f(feature.end.x(),feature.end.y(),0.0f);
    if(feature.type == hb_perception::ProfileCutFeatureType::Web)  {
        // apply an offset of torch rad away from start and end :
        torch_feature_start += tangent_profile.normalized() * request.torch_diameter * 0.5;
        torch_feature_end -= tangent_profile.normalized() * request.torch_diameter * 0.5;

    }
    
    Eigen::Vector3f start_world = world_from_profile* torch_feature_start;
    start_world += request.standoff * norm_world;
    segment.start_pose = makeToolPose(start_world,tan_world,norm_world);
    segment.start_constraints = makePoseConstraints(segment.start_pose,request.pos_tol,request.ang_tol);

    Eigen::Vector3f approach_world = start_world;
    approach_world += request.approach_dist * norm_world;
    segment.approach_pose = makeToolPose(approach_world,tan_world,norm_world);
    segment.approach_constraints = makePoseConstraints(segment.approach_pose,request.pos_tol,request.ang_tol);

    Eigen::Vector3f end_world = world_from_profile* torch_feature_end;
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