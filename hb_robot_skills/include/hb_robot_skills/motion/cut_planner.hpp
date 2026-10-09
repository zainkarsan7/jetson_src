#pragma once
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>
#include <functional>

namespace hb_robot_skills::motion{

struct CutPathPoint{

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f tangent = Eigen::Vector3f::UnitX();
    Eigen::Vector3f srf_norm = Eigen::Vector3f::UnitZ();

};

struct RobotWorkpieceContact
{

    std::string robot_link;
    Eigen::Vector3d pos_world;
    Eigen::Vector3d normal_world;
    double depth;
};

static std::vector<RobotWorkpieceContact> extractWorkpieceContacts(
    const collision_detection::CollisionResult& col_res
){
    std::vector<RobotWorkpieceContact> out;
    for (const auto& [pair,contacts]: col_res.contacts){
        for (const auto& c: contacts){
            const bool robot_first = 
            c.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK &&
            c.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT &&
            c.body_name_2 == "wk_section_collision";
            
            const bool robot_second = 
            c.body_type_2== collision_detection::BodyTypes::ROBOT_LINK &&
            c.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT &&
            c.body_name_1 == "wk_section_collision";

            if(!robot_first && !robot_second){
                continue;
            }
            RobotWorkpieceContact contact;
            contact.robot_link = robot_first? c.body_name_1 : c.body_name_2;
            contact.pos_world = c.pos;
            // because it goes from body 1 to body 2 
            contact.normal_world = robot_first? -c.normal : c.normal;
            contact.depth = c.depth;
            out.push_back(contact);

        }
    }
    return out;
}


struct CutSegment{
    // this struct is lke the glue to get from 2D profiles to 3D poses to robot states
    std::string name;
    hb_perception::ProfileCutFeatureType type = hb_perception::ProfileCutFeatureType::Unknown;
    
    std::vector<CutPathPoint> points;

    // poses
    Eigen::Isometry3d approach_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d retract_pose = Eigen::Isometry3d::Identity();

    moveit_msgs::msg::Constraints approach_constraints;
    moveit_msgs::msg::Constraints start_constraints;
    moveit_msgs::msg::Constraints end_constraints;
    moveit_msgs::msg::Constraints retract_constraints;


    //IK Solns

    moveit::core::RobotStatePtr approach_state;
    moveit::core::RobotStatePtr start_state;
    moveit::core::RobotStatePtr end_state;
    moveit::core::RobotStatePtr retract_state;



};


struct SegmentMotionPlan{

    moveit::planning_interface::MoveGroupInterface::Plan approach;
    moveit::planning_interface::MoveGroupInterface::Plan cut;
    moveit::planning_interface::MoveGroupInterface::Plan retract;
};

struct ExecutableCut{
    std::string name;
    moveit::planning_interface::MoveGroupInterface::Plan transit_in;
    SegmentMotionPlan process;
    moveit::planning_interface::MoveGroupInterface::Plan transit_out;
};



struct CutPlan{
    std::string profile_name;
    std::vector<CutSegment> segments;

};

struct CutRequest{
    float standoff = 0.0f;
    float approach_dist = 0.05f;
    float retract_dist = 0.05f;

    double pos_tol = 0.003;
    double ang_tol = 0.03;
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
            const CutRequest& request,
            const moveit::core::RobotState& start_state,
            const planning_scene::PlanningSceneConstPtr& p_scene

        ) const;

        std::optional<CutSegment> selectWebCandidate(
            std::vector<CutSegment>& candidates,
            const moveit::core::RobotState& start_state,
            const planning_scene::PlanningSceneConstPtr& p_scene
        ) const;

        using DebugVisCallback = std::function<void(const CutSegment&)>;
        void setDebugVisCallback(DebugVisCallback callback){
            debug_vis_callback_ = std::move(callback);
        }
        moveit_msgs::msg::Constraints makeBoxConstraints(
            const CutSegment& segment,
            const double pos_tol, 
            const double ang_tol
        )const;
    private:

        DebugVisCallback debug_vis_callback_;


        bool refineCollision(moveit::core::RobotState& candidate,
            const Eigen::Isometry3d& nominal_tcp_pose,
            const moveit_msgs::msg::Constraints& constraints,
            const collision_detection::CollisionResult collision_state,
            const planning_scene::PlanningSceneConstPtr& p_scene) const;

        bool isCloseEnough(const moveit::core::RobotState& seed_state, const moveit::core::RobotState& candidate_state)const;

        
        double dq_cost(const moveit::core::RobotState& a, const moveit::core::RobotState& b)const;


        CutSegment makeSegment(
            const hb_perception::ProfileCutFeature& feature,
            const Eigen::Isometry3f& world_from_profile,
            const CutRequest request
        )const;

        float approachScore(const CutSegment& segment, const Eigen::Vector3f& tcp_pos)const;

        

        moveit_msgs::msg::Constraints makePoseConstraints(
            const Eigen::Isometry3d nominal_pose,
            const double pos_tol, 
            const double ang_tol
        )const;


        Eigen::Isometry3d makeToolPose(
            const Eigen::Vector3f& position,
            const Eigen::Vector3f& tangent,
            const Eigen::Vector3f& surface_normal)const;
        

        bool sampleConstraint(const moveit_msgs::msg::Constraints& constraints,
            moveit::core::RobotState& state,
            const moveit::core::RobotState& reference_state,
            const planning_scene::PlanningSceneConstPtr& p_scene
            )const;

        bool solveSegmentConstraints(CutSegment& segment, 
            const moveit::core::RobotState& seed_state, 
            const planning_scene::PlanningSceneConstPtr& p_scene) const;


        collision_detection::CollisionResult checkCol(const moveit::core::RobotState& state,
        const planning_scene::PlanningSceneConstPtr p_scene) const;

        bool solveSegmentIK(CutSegment &segment, const moveit::core::RobotState& seed_state) const;

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_;
        std::string planning_group_;
        std::string plasma_link_;
    
};



}