#include "hb_robot_skills/motion/trajectory_validator.hpp"
#include "hb_robot_skills/motion/motion_planner.hpp"

namespace hb_robot_skills::motion{

bool MotionPlanner::planToPose(const geometry_msgs::msg::Pose& pose, 
    const std::string& link_name, 
    moveit::planning_interface::MoveGroupInterface::Plan& plan){
        move_group_->setStartStateToCurrentState();
        // move_group_->setPoseTarget(pose,link_name);

        const bool ik_success = move_group_->setJointValueTarget(
            pose,link_name
        );
        if (!ik_success){
            RCLCPP_WARN(node_->get_logger(),"couldnt find ik soln");
            return false;
        }

        

        const auto plan_result = move_group_->plan(plan);
        return plan_result == moveit::core::MoveItErrorCode::SUCCESS;

    }



 MotionPlanner::MotionPlanner(const rclcpp::Node::SharedPtr& node,
        const std::string & planning_group
    ): node_(node){
        move_group_ = std::make_unique<MoveGroupInterface>(node_,
        planning_group
    );
        move_group_->setEndEffectorLink("camera_visor");

        move_group_->startStateMonitor();
        
    }





 }