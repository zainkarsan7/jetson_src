#include "hb_robot_skills/motion/cut_planner.hpp"
#include <Eigen/Geometry>
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
            const std::vector<CutSegment> candidates,
            const moveit::core::RobotState& start_state) const{
    if(candidates.empty()){
        std::cerr<<"no candidates"<<std::endl;
        return std::nullopt;
    }
    const Eigen::Vector3f current_tcp_pose =start_state.getGlobalLinkTransform(plasma_link_).translation();

    float best_score = -std::numeric_limits<float>::infinity();
    std::optional<CutSegment> best_web;
    for (const auto& candidate: candidates){
        if (candidate.type != hb_perception::ProfileCutFeatureType::Web){
            continue;
        }
        float score = approachScore(candidate,current_tcp_pose);

        std::cout<<"web cand "<<candidate.name<<" approach score "<<score<<std::endl;
        if (!best_web || score > best_score){
            best_web = std::move(candidate);
            best_score = score;
        }
    }
    return *best_web;

    }
    




std::optional<CutPlan> CutPlanner::plan(
    const hb_robot_interfaces::msg::ProfileEstimate& estimate,
    const hb_perception::ProfileModel& profile,
    const CutRequest& request,
    const moveit::core::RobotState& start_state
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
    Eigen::Isometry3d world_from_profile_d;
    tf2::fromMsg(estimate.pose,world_from_profile_d);
    const Eigen::Isometry3f world_from_profile =world_from_profile_d.cast<float>();
    CutPlan plan;
    plan.profile_name=  profile.name;
    plan.segments.reserve(profile.cut_features.size());

    for (const auto& feature : profile.cut_features){
        const float length = (feature.end - feature.start).norm();
        if(length < 1e-5f){
            continue;
        }
        // choose web.
        CutSegment candidate_cut = makeSegment(feature, world_from_profile,request.standoff);

       
        else{
            plan.segments.emplace_back(std::move(candidate_cut));
        }
        if(best_web){
            plan.segments.emplace_back(std::move(*best_web));
        }
    }
  
    if (plan.segments.empty()){
        std::cerr<<"empty plan"<<std::endl; 
        return std::nullopt;
    }
        return plan;

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
    float standoff
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
    start_world += standoff * norm_world;

    Eigen::Vector3f end_world = world_from_profile* Eigen::Vector3f(feature.end.x(),feature.end.y(),0.0f);
    end_world += standoff * norm_world;

    CutPathPoint start;
    start.pos = start_world;
    start.tangent = tan_world;
    start.srf_norm = norm_world;

    CutPathPoint end;
    end.pos = end_world;
    start.tangent = tan_world;
    start.srf_norm = norm_world;

    segment.points.push_back(start);
    segment.points.push_back(end);
    return segment;
}
    
}