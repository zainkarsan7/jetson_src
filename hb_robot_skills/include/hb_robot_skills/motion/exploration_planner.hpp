#pragma once
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <hb_robot_skills/motion/exploration_types.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include "hb_robot_perception/profile_types.hpp"
#include <moveit/planning_scene/planning_scene.h>



namespace hb_robot_skills::motion{


class NBVPlanner{
public:
NBVPlanner(moveit::core::RobotModelConstPtr robot_model,
std::string planning_group,
std::string camera_link);

std::optional<ProfileViewSolution> plan(
    const moveit::core::RobotState& start_state,
    const ProfileInspectionRequest& request,
    const hb_perception::ProfileObservationModel& observed_profile,
    const planning_scene::PlanningSceneConstPtr& p_scene
)const;

private:

    std::vector<ProfileViewCandidate> generateCandidates(
        const ProfileInspectionRequest& request,
        const hb_perception::ProfileMatch& profile_estimate
    )const;

    double predictedInformationGain(
        const ProfileViewCandidate& candidate,
        const hb_perception::ProfileMatch& profile_estimate
    )const;

    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;

    std::string planning_group_;
    std::string camera_link_;

};


class ExplorationPlanner{

    public:
        /**
         * constructor for an inspection planner
         */
        ExplorationPlanner(
            moveit::core::RobotModelConstPtr robot_model,
            std::string planning_group,
            std::string camera_link
        ); 
        
        /**
         * take the request configuration
         * plan a full set of view candidates 
         */
        ExplorationPlan plan(
            const moveit::core::RobotState& start_state,
            const ExplorationRequest& request
        ) const;
        /**
         * solve one view with all its candidates
         */
        std::optional<ViewSolution> solveView(
            const moveit::core::RobotState& seed_state,
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request)const;
         

    private:
        /**
         * from the request get the views in range x and y
         */
        std::vector<Eigen::Isometry3d> generateNominalViews(
            const ExplorationRequest& request
        )const; 
        /**
         * for each nominal view get the candidate samples
         */
        std::vector<ViewCandidate> generateCandidates(
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request
        )const;   
        /**
         * solve a candidate sample from current state
         */
        std::optional<ViewSolution> solveCandidate(
            const moveit::core::RobotState& seed_state,
            const ViewCandidate& candidate,
            const ExplorationRequest& request
        )const;
        /**
         * score the ik solution
         */
        double scoreSoln(
            const moveit::core::RobotState& state_from, 
            const moveit::core::RobotState& state_to
        ) const;

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_;

        std::string planning_group_;
        std::string camera_link_;
};

}

