#pragma once
#include <string>

#include <moveit_msgs/msg/robot_trajectory.hpp>


namespace hb_robot_skills::motion{

struct TrajLim {
    double max_total_joint_motion;
    double max_single_joint_delta;
};

struct TrajValRes{
    bool valid;
    std::string reason;
    double total_joint_motion;
    double max_joint_delta;
};

class TrajValidator{
    public: 
    TrajValRes validate(
        const moveit_msgs::msg::RobotTrajectory & traj,
        const TrajLim& limits) const;
    
};

}