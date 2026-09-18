#pragma once
#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include <cstddef>
namespace hb_robot_skills::motion{

    struct ExplorationRequest{
        Eigen::Isometry3d center_pose = Eigen::Isometry3d::Identity();

        double range_x{0.0};
        double range_y{0.0};

        double pos_tol{0.05};
        double roll_tol{0.1};

        std::size_t num_samples{1};

        std::size_t sample_attempts{3};
        double ik_timeout{0.02};
        double joint_delta{0.5};

    };

    struct ViewCandidate{
        Eigen::Isometry3d nominal_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d candidate_pose = Eigen::Isometry3d::Identity();

        double pos_error{0.0};
        double roll_offset{0.0};
    };


    struct ViewSolution{

        Eigen::Isometry3d nominal_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d cam_pose = Eigen::Isometry3d::Identity();
        moveit::core::RobotState robot_state;
        double motion_cost{0.0};
        

        ViewSolution(const moveit::core::RobotState& state):
        robot_state(state){};

    };

    struct ExplorationPlan{
        std::vector<ViewSolution> views;
        std::size_t n_requested_views;
        std::size_t n_succeeded_views;
        
        bool complete{false};
    };


}