#pragma once
#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include <cstddef>
namespace hb_robot_skills::motion{




    struct ExplorationRequest{
        Eigen::Isometry3d center_pose = Eigen::Isometry3d::Identity();
        // pass the center pose and ask for samples in this
        // range x and y, then give some sphere of candidates
        // with pos tolerance and orientation tolerance
        double range_x{0.0};
        double range_y{0.0};

        double pos_tol{0.05};
        double roll_tol{0.1};

        std::size_t num_viewpoints{1};

        std::size_t num_candidates{3};
        double ik_timeout{0.02};
        double joint_delta{1.2};
        

    };

    struct ProfileInspectionRequest{

        Eigen::Isometry3d world_from_member = Eigen::Isometry3d::Identity();
        Eigen::Vector3d target_world = Eigen::Vector3d::Zero();
        Eigen::Isometry3d current_camera_pose= Eigen::Isometry3d::Identity();
        double cam_dist = 0.5;
        

        double min_information_gain = 0.05;
        double max_motion_cost = 3.0;

        double pos_tol{0.1};
        double ang_tol{0.3};

        std::size_t num_samples{25};

        std::size_t num_candidates{12};
        double ik_timeout{0.02};
        double joint_delta{1.2};

    };

    struct ProfileViewCandidate{

        Eigen::Isometry3d nominal_cam_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d actual_cam_pose = Eigen::Isometry3d::Identity();
        
        
        double predicted_gain = 0.0;
        double visibility_score = 0.0;
        double motion_cost = 0.0;
        double total_score = 0.0;

        moveit_msgs::msg::Constraints view_constraints;


    };

    struct ProfileViewSolution{
        moveit::core::RobotState robot_state;
        ProfileViewCandidate candidate;
        explicit ProfileViewSolution(const moveit::core::RobotState& state):
        robot_state(state){}
    };



    struct ViewCandidate{
        Eigen::Isometry3d nominal_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d candidate_pose = Eigen::Isometry3d::Identity();
        // organize the candidates about a nominal pose, 
        // get candidates about this pose.
        double pos_error{0.0};
        double roll_offset{0.0};
    };


    struct ViewSolution{

        Eigen::Isometry3d nominal_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d cam_pose = Eigen::Isometry3d::Identity();
        moveit::core::RobotState robot_state;
        double motion_cost{0.0};
        //save the robot state, the ik solution, and 
        // the cost of motion calculated from scoreSoln

        ViewSolution(const moveit::core::RobotState& state):
        robot_state(state){};

    };

    struct ExplorationPlan{
        std::vector<ViewSolution> views;
        std::size_t n_requested_views;
        std::size_t n_succeeded_views;
        // after a request return this plan with view solutions
        bool complete{false};
    };


}