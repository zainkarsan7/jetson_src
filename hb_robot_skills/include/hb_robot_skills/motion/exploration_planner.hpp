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
        ExplorationPlanner(
            const moveit::core::RobotModelConstPtr& robot_model,
            const std::string& planning_group,
            const std::string& camera_link
        );

        ExplorationPlan plan(
            const moveit::core::RobotState& start_state,
            const ExplorationRequest& request
        ) const;

        std::optional<ViewSolution> solveView(
            const moveit::core::RobotState& seed_state,
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request)const;


    private:
        std::vector<Eigen::Isometry3d> generateNominalViews(
            const ExplorationRequest& request
        )const;

        std::vector<Eigen::Isometry3d> generateCandidates(
            const Eigen::Isometry3d& nominal_pose,
            const ExplorationRequest& request
        )const;    

        double scoreSoln(
            const moveit::core::RobotState& state_from, 
            const moveit::core::RobotState& state_to
        ) const;

        moveit::core::RobotModelConstPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_;

        std::string planning_group_;
        std::string camera_link;
};

}

