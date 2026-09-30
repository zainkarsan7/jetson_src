#include "hb_robot_skills/motion/cut_planner.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hb_robot_skills::motion{

std::optional<CutPlan> CutPlanner::plan(
    const hb_robot_interfaces::msg::ProfileEstimate& estimate,
    const hb_perception::ProfileModel& profile,
    const CutRequest& request
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
        plan.segments.emplace_back(makeSegment(feature,world_from_profile,request.standoff));
    }
    if (plan.segments.empty()){
        std::cerr<<"empty plan"<<std::endl; 
        return std::nullopt;
    }
    return plan;

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