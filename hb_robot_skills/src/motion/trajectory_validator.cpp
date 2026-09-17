#include "hb_robot_skills/motion/trajectory_validator.hpp"
#include "hb_robot_skills/motion/motion_planner.hpp"


namespace hb_robot_skills::motion{

    TrajValRes TrajValidator::validate(const moveit_msgs::msg::RobotTrajectory & traj,
        const TrajLim& limits) const{
            return {true, "Trajectory accepted"};

        }


}