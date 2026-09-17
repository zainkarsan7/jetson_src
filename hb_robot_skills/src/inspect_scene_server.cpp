#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <hb_robot_interfaces/action/inspect_scene.hpp>
#include "hb_robot_skills/inspect_scene_server.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "hb_robot_skills/motion/motion_planner.hpp"
#include "hb_robot_skills/motion/trajectory_validator.hpp"


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

    display_traj_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
    10);
    execute_motion_ = this->get_parameter("execute_inspection_motion").as_bool();

    viewpoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inspect_scene/viewpoints",10);
}

void InspectSceneServer::publishViewpointMarker(const std::vector<geometry_msgs::msg::Pose> &viewpoints){
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (std::size_t m_ind = 0; m_ind<viewpoints.size(); m_ind++){
        visualization_msgs::msg::Marker view_marker;
        view_marker.header.frame_id = move_group_->getPoseReferenceFrame();
        view_marker.header.stamp = this->now();
        view_marker.ns = "inspection_viewpoints";
        view_marker.id = static_cast<int>(m_ind);
        view_marker.type = visualization_msgs::msg::Marker::ARROW;
        view_marker.action = visualization_msgs::msg::Marker::ADD;

        const auto& m_pose  = viewpoints[m_ind];
        view_marker.pose = m_pose;
        view_marker.scale.x = 0.015;
        view_marker.scale.y = 0.015;
        view_marker.scale.z = 0.05;
        view_marker.color.r = 1.0;
        view_marker.color.b = 0.2;
        view_marker.color.g = 0.2;
        view_marker.color.a = 1.0;
        view_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        markers.markers.push_back(view_marker);

    }
    viewpoint_marker_pub_->publish(markers);
}


void InspectSceneServer::initializeMoveit(){
    

    // motion_planner_ = std::make_unique<hb_robot_skills::motion::MotionPlanner>(
    //     shared_from_this(),
    //     "manipulator"
    // );

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),"manipulator");
    move_group_->startStateMonitor();


    
    RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
    RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(get_logger(),"End-effector link %s",move_group_->getEndEffectorLink().c_str());

}

rclcpp_action::GoalResponse InspectSceneServer::handleGoal(const rclcpp_action::GoalUUID & , 
            std::shared_ptr<const InspectScene::Goal> goal){
                RCLCPP_INFO(get_logger(),"recieved inspection goal with %zu viewpoints", goal->viewpoints.size());
                if (goal->viewpoints.empty()){
                    RCLCPP_WARN(get_logger(), "Rejecting, no viewpoints");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            }
rclcpp_action::CancelResponse InspectSceneServer::handleCancel(const std::shared_ptr<GoalHandleInspectScene> ){
    RCLCPP_INFO(get_logger(),"cancel requested");
    if(!move_group_){
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

    publishViewpointMarker(goal->viewpoints);

    auto feedback = std::make_shared<InspectScene::Feedback>();

    auto result = std::make_shared<InspectScene::Result>();

    std::size_t completed = 0;
    auto current = move_group_->getCurrentPose("camera_visor");
    const bool ik_success = 
    move_group_->setJointValueTarget(current, "camera_visor");
    RCLCPP_INFO(get_logger(), "IK_success %s", ik_success ? "yes":"no");



    for (std::size_t view_index = 0; view_index < goal->viewpoints.size(); view_index++){
        if (goal_handle->is_canceling()){
            move_group_->stop();
            result->success = false;
            result->viewpoints_captured = completed;
            result->result_code = result->CANCELLED;
            result->message = "Cancelled Inspection";
            goal_handle->canceled(result);
            return;
        }
        feedback->current_viewpoint = view_index;
        feedback->total_viewpoints = goal->viewpoints.size();
        feedback->current_pose = move_group_->getCurrentPose("camera_visor").pose;
        goal_handle->publish_feedback(feedback);

        const auto& target = goal->viewpoints[view_index];
        MoveGroupInterface::Plan plan;
        // talk to the moveit action server
        const auto plan_result = move_group_->plan(plan);
        move_group_->setJointValueTarget(target,"camera_visor");
        // bool ik_success = planToPose(
        //     target,"camera_visor",plan
        // );
        // if(!ik_success){
        if(plan_result != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_WARN(get_logger(), "planning failed for viewpoint %zu",view_index);
            
            
            continue;
        }
        moveit_msgs::msg::DisplayTrajectory display_msg;
        display_msg.trajectory_start = plan.start_state_;
        display_msg.trajectory.push_back(plan.trajectory_);
        display_traj_pub_->publish(display_msg);
        
        
        if(!execute_motion_){
            RCLCPP_INFO(get_logger(),"motion disabled going to next viewpoint");
            
            continue;
        }


        const auto execute_result = move_group_->execute(plan);
        
        if(execute_result!=moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_WARN(get_logger(),"failed to traverse to viewpoint %zu", view_index);
            continue;
        }
        
        ++completed;

        RCLCPP_INFO(get_logger(),"reached viewpoint %zu/%zu",view_index,goal->viewpoints.size());

       
    }

     if (completed == goal->viewpoints.size()){
            result->success = true;
            result->message = "all views captured";
            goal_handle->succeed(result);

        }
        else{
            result->success= false;
            result->result_code = result->ACQUISITION_FAILED;
            result->message="one or more views couldnt be captured";
            goal_handle->abort(result);

        }

    

}





int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<InspectSceneServer>(options);

    try{
    node->initializeMoveit();
    RCLCPP_INFO(node->get_logger(),"initalized moveit");
    }
    catch(std::exception &e){
        RCLCPP_FATAL(node->get_logger(),"%s",e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}