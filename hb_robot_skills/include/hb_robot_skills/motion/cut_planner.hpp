#pragma once
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

namespace hb_robot_skills::motion{

struct CutPathPoint{

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f tangent = Eigen::Vector3f::UnitX();
    Eigen::Vector3f srf_norm = Eigen::Vector3f::UnitZ();

};

struct CutSegment{
    std::string name;
    hb_perception::ProfileCutFeatureType type = hb_perception::ProfileCutFeatureType::Unknown;
    std::vector<CutPathPoint> points;
};

struct CutPlan{
    std::string profile_name;
    std::vector<CutSegment> segments;

};

struct CutRequest{
    float standoff = 0.0f;
};

class CutPlanner{
    public: 
        std::optional<CutPlan> plan(
            const hb_robot_interfaces::msg::ProfileEstimate& estimate,
            const hb_perception::ProfileModel& profile,
            const CutRequest& request = CutRequest{}
        ) const;
    private:
        CutSegment makeSegment(
            const hb_perception::ProfileCutFeature& feature,
            const Eigen::Isometry3f& world_from_profile,
            float standoff
        )const;
    
};



}