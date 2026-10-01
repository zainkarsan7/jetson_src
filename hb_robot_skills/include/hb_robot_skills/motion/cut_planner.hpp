#pragma once
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
    float approach_dist = 0.05f;
    float retract_dist = 0.05f;
};

class CutPlanner{
    public: 

        CutPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            std::string plasma_link
        );



        std::optional<CutPlan> plan(
            const hb_robot_interfaces::msg::ProfileEstimate& estimate,
            const hb_perception::ProfileModel& profile,
            const CutRequest& request = CutRequest{},
            const moveit::core::RobotState& start_state

        ) const;

        std::optional<CutSegment> selectWebCandidate(
            const std::vector<CutSegment> candidates,
            const moveit::core::RobotState& start_state) const;
    private:

        CutSegment makeSegment(
            const hb_perception::ProfileCutFeature& feature,
            const Eigen::Isometry3f& world_from_profile,
            float standoff
        )const;

        float approachScore(const CutSegment& segment, const Eigen::Vector3f& tcp_pos)const;

        
        

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_;
        std::string planning_group_;
        std::string plasma_link_;
    
};



}