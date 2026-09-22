#include "hb_robot_skills/inspect_scene_server.hpp"
#include <thread>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hb_robot_skills{ 
InspectSceneServer::InspectSceneServer(const rclcpp::NodeOptions & options):Node("inspect_scene_server",options){
    // pass stuff to the private variables using some parameters
    planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
    camera_link_ = declare_parameter<std::string>("camera_link","ur10e_tool0");
    planning_time_ = declare_parameter<double>("planning_time",5.0);
    stability_timeout_ = declare_parameter<double>("stability_timeout",5.0);
}
   
void InspectSceneServer::initialize(){
    
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
    move_group_->setPlanningTime(planning_time_);
    exploration_planner_ = std::make_unique<motion::ExplorationPlanner>(
        move_group_->getRobotModel(),
        planning_group_,
        camera_link_);
    
    action_server_ = rclcpp_action::create_server<InspectScene>(
        shared_from_this(),
    "inspect_scene",
    std::bind(
        &InspectSceneServer::handleGoal,
        this,
        std::placeholders::_1,std::placeholders::_2
    ),
    std::bind(
        &InspectSceneServer::handleCancel,
        this,
        std::placeholders::_1
    ),
    std::bind(
        &InspectSceneServer::handleAccepted,
        this,
        std::placeholders::_1
    ));

    RCLCPP_INFO(get_logger(),"Inspect Scene Server Initialized");  
    RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
    RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(get_logger(),"End-effector link %s",move_group_->getEndEffectorLink().c_str());

}

rclcpp_action::GoalResponse InspectSceneServer::handleGoal(const rclcpp_action::GoalUUID & , 
    std::shared_ptr<const InspectScene::Goal> goal){
        
        // RCLCPP_INFO(get_logger(),"recieved inspection goal with %zu viewpoints", goal->viewpoints.size());
        // if (goal-> < 1){
        //     RCLCPP_WARN(get_logger(), "Rejecting, no viewpoints");
        //     return rclcpp_action::GoalResponse::REJECT;
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

rclcpp_action::CancelResponse InspectSceneServer::handleCancel(const std::shared_ptr<GoalHandleInspectScene> ){
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

motion::ExplorationRequest InspectSceneServer::makeExplorationRequest(
    const InspectScene::Goal& goal) const{
        motion::ExplorationRequest request;
        tf2::fromMsg(goal.center_pose,request.center_pose);
        request.range_x = goal.range_x;
        request.range_y = goal.range_y;
        request.pos_tol = goal.pos_tol;
        request.roll_tol = goal.orn_tol;
        request.num_samples = goal.num_viewpoints;
             
        
        return request;
    }

void InspectSceneServer::execute(const std::shared_ptr<GoalHandleInspectScene> goal_handle){
    std::lock_guard<std::mutex> lock(execution_mutex_);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<InspectScene::Result>();
    result->success = false;
    result->viewpoints_captured = 0;

    auto current_state = move_group_->getCurrentState(2.0);

    if(!current_state){
        result->result_code = InspectScene::Result::PLANNING_FAILED;
        result->message = "couldnt get robot state";

        goal_handle->abort(result);
        return;
    }

    publishFeedback(goal_handle,0,0,0,0);
    const auto request = InspectSceneServer::makeExplorationRequest(*goal);

    auto exploration = exploration_planner_->plan(*current_state,request);

    if (exploration.views.empty()){
        result->result_code = InspectScene::Result::PLANNING_FAILED;
        result->message="couldnt reach anything";
        goal_handle->abort(result);
        return;
    }

    const uint32_t total_views=  static_cast<uint32_t>(exploration.views.size());

    // visit each view

    for (uint32_t i = 0; i<total_views; i++){
        if(goal_handle->is_canceling()){
            move_group_->stop();
            result->result_code = InspectScene::Result::CANCELLED;
            result->viewpoints_captured = i;
            result->message = "exploration cancelled";
            goal_handle->canceled(result);
            return;
        }
        const auto& view = exploration.views[i];
        
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::MOVING,i);
        // MOVE
        if(!moveToView(view.robot_state)){
            result->result_code = InspectScene::Result::MOTION_FAILED;
            result->viewpoints_captured = i;
            result->message = "motion failed";
            goal_handle->abort(result);
            return;
        }
        // WAIT FOR SETTLING
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::SETTLING,i);

        if(!waitForStability()){
            result->result_code =InspectScene::Result::STABILITY_TIMEOUT;
            result->viewpoints_captured = i;
            result->message = "couldnt reach stable pose";
            goal_handle->abort(result);
            return;
        }
        // ACQUIRE SAMPLES
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::ACQUIRING,i);
        if(!acquireSamples(goal->samples_per_viewpoint)){
            result->result_code =InspectScene::Result::ACQUISITION_FAILED;
            result->viewpoints_captured = i;
            result->message = "couldnt acquire samples";
            goal_handle->abort(result);
            return;
        }

        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::REGISTERING,i);
        if(!registerView()){
            result->result_code =InspectScene::Result::REGISTRATION_FAILED;
            result->viewpoints_captured = i;
            result->message = "couldnt register sample";
            goal_handle->abort(result);
            return;
        }
        result->viewpoints_captured = i+1;
    }
    result->success = true;
    result->result_code = InspectScene::Result::SUCCESS;
    result->message = "finished exploration";
    goal_handle->succeed(result);
}


bool InspectSceneServer::moveToView(const moveit::core::RobotState& target_state){
    auto actual_state = move_group_->getCurrentState(2.0);
    if(!actual_state){
        RCLCPP_ERROR(get_logger(),"couldnt get robot state");
        return false;
    }
    move_group_->setStartState(*actual_state);
    std::vector<double> joint_target;
    target_state.copyJointGroupPositions(move_group_->getRobotModel()->getJointModelGroup(planning_group_),joint_target);
    if(!move_group_->setJointValueTarget(joint_target)){
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS){
        return false;
    }
    const auto execution_result= move_group_->execute(plan);
    if(execution_result !=moveit::core::MoveItErrorCode::SUCCESS){
        return false;
    }
    return true;
}

void InspectSceneServer::publishFeedback(const std::shared_ptr<GoalHandleInspectScene>& goal_handle,
            uint32_t current_viewpoint, uint32_t total_viewpoints, 
            uint8_t phase, uint8_t samples_acquired
        ){
            auto feedback = std::make_shared<InspectScene::Feedback>();
            feedback->phase = phase;
            feedback->current_pose = move_group_->getCurrentPose(camera_link_).pose;
            feedback->current_viewpoint = current_viewpoint;
            feedback->total_viewpoints = total_viewpoints;
            feedback->phase = phase;

            goal_handle->publish_feedback(feedback);


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
        view_marker.scale.x = 0.05;
        view_marker.scale.y = 0.015;
        view_marker.scale.z = 0.015;
        view_marker.color.r = 1.0;
        view_marker.color.b = 0.2;
        view_marker.color.g = 0.2;
        view_marker.color.a = 1.0;
        view_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        markers.markers.push_back(view_marker);

    }
    viewpoint_marker_pub_->publish(markers);
}

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<InspectSceneServer>(options);

    try{
    node->initialize();
    RCLCPP_INFO(node->get_logger(),"initalized server");
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
}