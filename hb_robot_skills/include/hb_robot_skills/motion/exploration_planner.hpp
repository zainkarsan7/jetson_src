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



namespace hb_robot_skills::motion{


class ExplorationPlanner{

    public:
        /**
         * constructor for an inspection planner
         */
        ExplorationPlanner(
            const moveit::core::RobotModelConstPtr& robot_model,
            const std::string& planning_group,
            const std::string& camera_link
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

