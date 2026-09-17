#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <hb_robot_interfaces/action/inspect_scene.hpp>
#include "hb_robot_skills/inspect_scene_server.hpp"

using InspectScene = hb_robot_interfaces::action::InspectScene;
using GoalHandleInspectScene = rclcpp_action::ServerGoalHandle<InspectScene>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

InspectSceneServer::InspectSceneServer(const rclcpp::NodeOptions & options):
Node("inspect_scene_server",options){
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<InspectScene>(this,
    "inspect_scene",
    std::bind(
        &InspectSceneServer::handleGoal,
        this,
        _1,_2
    ),
    std::bind(
        &InspectSceneServer::handleCancel,
        this,
        _1
    ),
    std::bind(
        &InspectSceneServer::handleAccepted,
        this,
        _1
    ));
}

void InspectSceneServer::initializeMoveit(){
    move_group_ = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        "manipulator"
    );
    move_group_->setEndEffector("camera_visor");

    move_group_->startStateMonitor();
    RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
    RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
}

rclcpp_action::GoalResponse InspectSceneServer::handleGoal(const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const InspectScene::Goal> goal){
                RCLCPP_INFO(get_logger(),"recieved inspection goal with %zu viewpoints", goal->viewpoints.size());
                if (goal->viewpoints.empty()){
                    RCLCPP_WARN(get_logger(), "Rejecting, no viewpoints");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            }
rclcpp_action::CancelResponse InspectSceneServer::handleCancel(const std::shared_ptr<GoalHandleInspectScene> goal_handle){
    RCLCPP_INFO(get_logger(),"cancel requested");
    if(move_group_){
        move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void InspectSceneServer::handleAccepted(const std::shared_ptr<GoalHandleInspectScene> goal_handle){
    std::thread{
        std::bind(&InspectSceneServer::execute,
        this,
        goal_handle        
        )
    }.detach();
}
void InspectSceneServer::execute(const std::shared_ptr<GoalHandleInspectScene> goal_handle){
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<InspectScene::Feedback>();
    
}





int main(int argc, char **argv){

    auto node = std::make_shared<InspectSceneServer>();
    node->initializeMoveit();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}