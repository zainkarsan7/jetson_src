#include "hb_robot_skills/inspect_scene_server.hpp"
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

namespace hb_robot_skills{ 
InspectSceneServer::InspectSceneServer(const rclcpp::NodeOptions & options):Node("inspect_scene_server",options)
, scene_model_(std::make_shared<hb_perception::SceneModel>())
{
    // pass stuff to the private variables using some parameters
    planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
    camera_link_ = declare_parameter<std::string>("camera_link","camera_visor");
    planning_time_ = declare_parameter<double>("planning_time",5.0);
    stability_timeout_ = declare_parameter<double>("stability_timeout",5.0);
    skip_motion_ = declare_parameter<bool>("skip_inspection_motion", true);
    rgb_topic_ = declare_parameter<std::string>("perception_rgb_topic","/k4a/depth_to_rgb/image_raw");
    depth_topic_=declare_parameter<std::string>("perception_depth_topic","/k4a/rgb/image_raw");
    camera_info_topic_=declare_parameter<std::string>("perception_camera_info_topic","k4a/depth_to_rgb/camera_info");
    scene_frame_= declare_parameter<std::string>("scene_base_frame","world");
    acquisition_timeout_ = declare_parameter<double>("acquisition timeout",2.0);
}
   
void InspectSceneServer::initialize(){
    
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
    move_group_->setPlanningTime(planning_time_);

    RCLCPP_INFO(
    get_logger(),
    "Exploration camera link parameter: '%s'",
    camera_link_.c_str());

const auto robot_model =
    move_group_->getRobotModel();

RCLCPP_INFO(
    get_logger(),
    "MoveIt model frame: '%s'",
    robot_model->getModelFrame().c_str());

for (const auto* link : robot_model->getLinkModels())
{
    RCLCPP_INFO(
        get_logger(),
        "MoveIt link: '%s'",
        link->getName().c_str());
}

    exploration_planner_ = std::make_unique<motion::ExplorationPlanner>(
        move_group_->getRobotModel(),
        planning_group_,
        camera_link_);
    ///initialize acquistision stuff:
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    rgbd_acquisition_ = std::make_unique<hb_perception::RGBDAcquisition>(this,
        tf_buffer_.get(),scene_frame_,rgb_topic_,depth_topic_,camera_info_topic_);
    RCLCPP_INFO(get_logger(),"Acquisition Initialized");
    
    display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
        10
        //rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
    );
    viewpoint_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("inspection_viewpoints", 10);
    scene_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scene_cloud",
            rclcpp::QoS(1).transient_local().reliable());
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

    approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_motion",
            std::bind(&InspectSceneServer::handleApproval,this,
                std::placeholders::_1,std::placeholders::_2)
        );

    RCLCPP_INFO(get_logger(),"Inspect Scene Server Initialized");  
    RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
    RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(get_logger(),"End-effector link %s",move_group_->getEndEffectorLink().c_str());

    
    

}

void InspectSceneServer::handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
        std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response){
            
            {std::lock_guard<std::mutex> lock(approval_mutex_);
            motion_approved_ = request->approve;}
            response->accepted = true;
            if(request->approve){
                response->message = "motion approved";
                approval_cv_.notify_all();
            }
            else{
                response->message = "motion not approved";
            }

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
        request.num_viewpoints = goal.num_viewpoints;
        request.num_candidates = goal.num_candidates;
        
             
        
        return request;
    }

// bool InspectSceneServer::moveHome(){
//     auto current_state = move_group_->getCurrentState(2.0);
//     if (!current_state){
//         RCLCPP_ERROR(get_logger(), "coudlnt get state");
//         return false;
//     }
//     move_group_->setStartState(*current_state);
//     if(!move_group_->setNamedTarget("inspection_home")){
//         RCLCPP_ERROR(get_logger(), "target home not defined");
//     }
//     moveit::planning_interface::MoveGroupInterface::Plan home_plan;
//     const auto planning_result = move_group_

// }

void InspectSceneServer::execute(const std::shared_ptr<GoalHandleInspectScene> goal_handle){

    


    std::lock_guard<std::mutex> lock(execution_mutex_);
    {
        std::lock_guard<std::mutex> lock(approval_mutex_);
        motion_approved_ = false;
        scene_model_->clear();

    }
    

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

    publishFeedback(goal_handle,0,goal->num_viewpoints*goal->num_viewpoints,InspectScene::Feedback::PLANNING);
    const auto request = InspectSceneServer::makeExplorationRequest(*goal);

    auto exploration = exploration_planner_->plan(*current_state,request);
    
    if (exploration.views.empty()){
        result->result_code = InspectScene::Result::PLANNING_FAILED;
        result->message="couldnt reach anything";
        goal_handle->abort(result);
        return;
    }

    const uint32_t total_views=  static_cast<uint32_t>(exploration.views.size());
    // draw all the views:
    RCLCPP_INFO(get_logger(),"planned %d view solutions",total_views);
    const auto* jmg=  move_group_->getRobotModel()->getJointModelGroup(planning_group_);
        
    std::vector<geometry_msgs::msg::Pose> solved_viewpoints;
    solved_viewpoints.reserve(total_views);
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> motion_plans;
    moveit::core::RobotState planning_state(*current_state);
    

    // visualization run
    for (std::size_t i=0; i<total_views; i++){
        const auto& view = exploration.views[i];
        auto plan = planToView(planning_state,view.robot_state);
        if(!plan){
            RCLCPP_ERROR(get_logger(),"planning failed at segment %zu",i);

            result->result_code = InspectScene::Result::PLANNING_FAILED;
            result->viewpoints_captured = 0;
            result->message = "failed planning at segment" + std::to_string(i);
            goal_handle->abort(result);
            return;
            
        }
        motion_plans.push_back(std::move(*plan));
        planning_state = exploration.views[i].robot_state;
        solved_viewpoints.push_back(tf2::toMsg(view.cam_pose));
    }
    publishViewpointMarker(solved_viewpoints);

    // ADD RETURN HOME ROUTINE
    moveit::core::RobotState home_state(move_group_->getRobotModel());
    home_state = planning_state;
    if(!home_state.setToDefaultValues(planning_group_,"inspection_home")){
        RCLCPP_ERROR(get_logger(),"couldnt find inspection home");
        

            result->result_code = InspectScene::Result::PLANNING_FAILED;
            result->viewpoints_captured = 0;
            result->message = "failed planning to return home";
            goal_handle->abort(result);
            return;

    }
    const auto& joint_names = jmg->getVariableNames();
    for (const auto jn: joint_names){
        RCLCPP_INFO(get_logger(), "home state %s: %.2f",jn,home_state.getVariablePosition(jn));
    }

    auto home_plan = planToState(planning_state,home_state);
    moveit_msgs::msg::RobotTrajectory combined_msg;
    robot_trajectory::RobotTrajectory combined_traj(move_group_->getRobotModel(),planning_group_);
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.trajectory_start = motion_plans.front().start_state_;
    for (const auto& plan : motion_plans){
        robot_trajectory::RobotTrajectory traj_seg(move_group_->getRobotModel(),planning_group_);
        // conversion bullshit
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        moveit::core::robotStateMsgToRobotState(plan.start_state_,start_state);
        traj_seg.setRobotTrajectoryMsg(start_state,plan.trajectory_);
        combined_traj.append(traj_seg,0.0);

        // display_msg.trajectory.push_back(plan.trajectory_);
    }
    robot_trajectory::RobotTrajectory return_home_traj(move_group_->getRobotModel(),planning_group_);
    moveit::core::RobotState home_start_state(move_group_->getRobotModel());
    moveit::core::robotStateMsgToRobotState(home_plan->start_state_,home_start_state);
    return_home_traj.setRobotTrajectoryMsg(home_start_state,home_plan->trajectory_);
    combined_traj.append(return_home_traj,0.0);
    
    combined_traj.getRobotTrajectoryMsg(combined_msg);
    display_msg.trajectory_start = motion_plans.front().start_state_;
    display_msg.trajectory.push_back(combined_msg);
    display_traj_pub_->publish(display_msg);
    RCLCPP_INFO(get_logger(),"publishing combined trajectory with %zu points",combined_msg.joint_trajectory.points.size());
    RCLCPP_INFO(get_logger(),"publishing %zu trajectories",display_msg.trajectory.size());
    for (size_t i=0;i<motion_plans.size();i++){
        RCLCPP_INFO(get_logger(),"traj %zu : %zu points",i,motion_plans[i].trajectory_.joint_trajectory.points.size());
    }

    if(skip_motion_){
        RCLCPP_INFO(get_logger(),"skip motion is true, no motion executed");
    }
    else{
        RCLCPP_INFO(get_logger(),"skip motion is false, awaiting approval");
        while(rclcpp::ok()){
            if (goal_handle->is_canceling()){
                move_group_->stop();
                result->result_code = InspectScene::Result::CANCELLED;
                result->viewpoints_captured = 0;
                result->message = "exploration cancelled while awaiting approval";
                goal_handle->canceled(result);
                return;
            }
            std::unique_lock<std::mutex> approval_lock(
                approval_mutex_
            );
            if(approval_cv_.wait_for(approval_lock, std::chrono::milliseconds(100),
        [this](){return motion_approved_;})){break;}

        }
        RCLCPP_INFO(get_logger(),"motion_approved");
    
    
    }
    

    // execution run
    for (uint32_t i = 0; i<motion_plans.size(); i++){
        if(goal_handle->is_canceling()){
            move_group_->stop();
            result->result_code = InspectScene::Result::CANCELLED;
            result->viewpoints_captured = i+1;
            result->message = "exploration cancelled";
            goal_handle->canceled(result);
            return;
        }
        const auto& view = exploration.views[i];
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::MOVING);
        // MOVE

        // if(!moveToView(view.robot_state)){
        //     result->result_code = InspectScene::Result::MOTION_FAILED;
        //     result->viewpoints_captured = i;
        //     result->message = "motion failed";
        //     goal_handle->abort(result);
        //     return;
        // }
        if(!skip_motion_){
            auto exec_result = move_group_->execute(motion_plans[i]);
            if(!static_cast<bool>(exec_result)){
                result->result_code = InspectScene::Result::MOTION_FAILED;
                result->viewpoints_captured = i;
                result->message = "motion failed";
                goal_handle->abort(result);
                return;
            }
        }
        else{
            RCLCPP_INFO(get_logger(),"skipping motion for segment %zu", i);
        }
        
        // WAIT FOR SETTLING
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::SETTLING);

        if(!waitForStability()){
            result->result_code =InspectScene::Result::STABILITY_TIMEOUT;
            result->viewpoints_captured = i;
            result->message = "couldnt reach stable pose";
            goal_handle->abort(result);
            return;
        }
        // ACQUIRE SAMPLES
        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::ACQUIRING);
        if(!acquireSamples(goal->sample_attempts)){
            result->result_code =InspectScene::Result::ACQUISITION_FAILED;
            result->viewpoints_captured = i;
            result->message = "couldnt acquire samples";
            goal_handle->abort(result);
            return;
        }

        publishFeedback(goal_handle,i,total_views,InspectScene::Feedback::REGISTERING);
        if(!registerView()){
            result->result_code =InspectScene::Result::REGISTRATION_FAILED;
            result->viewpoints_captured = i;
            result->message = "couldnt register sample";
            goal_handle->abort(result);
            return;
        }
        result->viewpoints_captured = i+1;
    }
    // DONE THE INSPECTION SWEEP
    auto exec_home_plan = move_group_->execute(*home_plan);
    if(!static_cast<bool>(exec_home_plan)){
            result->result_code = InspectScene::Result::MOTION_FAILED;
            result->message = "motion failed returning home";
            goal_handle->abort(result);
            return;
        }




    result->success = true;
    result->result_code = InspectScene::Result::SUCCESS;
    result->message = "finished exploration";
    goal_handle->succeed(result);
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> InspectSceneServer::planToState(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& target_state
        ){
            move_group_->setStartState(start_state);
            move_group_->setJointValueTarget(target_state);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if(!static_cast<bool>(move_group_->plan(plan))){
                return std::nullopt;
            }
            return plan;
        }

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> InspectSceneServer::planToView(
            const moveit::core::RobotState& start_state,
            const motion::ViewSolution &view 

        ){
            move_group_->setStartState(start_state);
            move_group_->setJointValueTarget(view.robot_state);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if(!static_cast<bool>(move_group_->plan(plan))){
                return std::nullopt;
            }
            return plan;
        }


bool InspectSceneServer::moveToView(const moveit::core::RobotState& target_state){
    auto actual_state = move_group_->getCurrentState(2.0);
    if(!actual_state){
        RCLCPP_ERROR(get_logger(),"couldnt get robot state");
        return false;
    }
    move_group_->setStartState(*actual_state);
    
    if(!move_group_->setJointValueTarget(target_state)){
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
            uint8_t phase
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

    void InspectSceneServer::publishSceneCloud(){
        const auto cloud = scene_model_->cloud();
        if(!cloud||cloud->empty()){
            return;
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id  = scene_frame_;
        msg.header.stamp = this->now();
        scene_cloud_pub_->publish(msg);
    
    };

    bool InspectSceneServer::waitForStability()
    {return true;
    }
    bool InspectSceneServer::acquireSamples(uint32_t sample_count)
    {
        if(!rgbd_acquisition_){
            RCLCPP_ERROR(get_logger(),"acquisition not initialized");
            return false;
        }
        if (sample_count==0){
            RCLCPP_ERROR(get_logger(),"sample_count is 0");
                return false;
            
        }
        rclcpp::Time capture_boundary = now();

        for (uint32_t i = 0; i<sample_count; i++){
            auto observation = rgbd_acquisition_->acquireAfter(capture_boundary,
                std::chrono::milliseconds(static_cast<int64_t>(acquisition_timeout_*1000.0)));

            if(!observation){
                RCLCPP_ERROR(get_logger(),"capture failed for sample %zu/%zu",i,sample_count);
                continue;    
            }
            capture_boundary = observation->stamp;
            // observation_buffer_.addObservation(std::move(*observation));
            RCLCPP_INFO(get_logger(),"scene_model is %s",scene_model_?"valid":"null");
            RCLCPP_INFO(get_logger(),"observation contains cloud %s",observation->point_cloud_->data.empty() ?"empty":"poplated");
            if(!scene_model_->addObservation(std::move(*observation))){
                RCLCPP_ERROR(get_logger(),"Couldnt integrate point cloud");

            }
            RCLCPP_INFO(get_logger(),"Captured sample %u/%u at %.6f",
            i,sample_count, observation->stamp.seconds()
        );



        }
        RCLCPP_INFO(get_logger(),"observation buffer has %d captures",scene_model_->observationCount());
        return true;
    }
    bool InspectSceneServer::registerView()
    {return true;
    }
}
int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions();//.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hb_robot_skills::InspectSceneServer>(options);

    try{
    node->initialize();
    RCLCPP_INFO(node->get_logger(),"initalized server");
    }
    catch(std::exception &e){
        RCLCPP_FATAL(node->get_logger(),"%s",e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
