#include "hb_robot_skills/cut_profile_server.hpp"
#include <chrono>
#include <thread>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>


using namespace std::chrono_literals;
namespace hb_robot_skills{

    CutProfileServer::CutProfileServer(const rclcpp::NodeOptions &options):Node("cut_profile_server",options){
        planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
        plasma_link_ = declare_parameter<std::string>("plasma_link","ur10e_torch_link");
        planning_time_ = declare_parameter<double>("planning_time",5.0);

        approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_motion",
            std::bind(&CutProfileServer::handleApproval,this,
                std::placeholders::_1,std::placeholders::_2)
        );  

        action_server_ = rclcpp_action::create_server<CutProfile>(this,"cut_profile",
            std::bind(
            &CutProfileServer::handleGoal,
            this,
            std::placeholders::_1,std::placeholders::_2
            ),
        std::bind(
                &CutProfileServer::handleCancel,
                this,
                std::placeholders::_1
            ),
        std::bind(
                &CutProfileServer::handleAccepted,
                this,
                std::placeholders::_1
            ));

        auto estimate_qos = rclcpp::QoS(1).reliable().transient_local();



        profile_estimate_sub_ = create_subscription<hb_robot_interfaces::msg::ProfileEstimate>("perception/profile_estimate",
        estimate_qos, std::bind(&CutProfileServer::profileEstimateCallback,this,
        std::placeholders::_1));

        auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
        visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cut_profile/cut_plan",marker_qos);

    }

    void CutProfileServer::initialize(){

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
        move_group_->setPlanningTime(planning_time_);
        

        
        const auto robot_model =
                move_group_->getRobotModel();
        
        cut_planner_ = std::make_unique<motion::CutPlanner>(            
        move_group_->getRobotModel(),
        planning_group_,
        plasma_link_);
        

        cut_planner_->setDebugVisCallback([this](const motion::CutSegment segment){
            publishCandidateVisualization(segment);
        }
        );

        display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
        10
        //rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
    );

        RCLCPP_INFO(
                get_logger(),
                "Plasma link parameter: '%s'",
                plasma_link_.c_str());
        RCLCPP_INFO(get_logger(),"moveit initialized cam TCP manipulator group");
        RCLCPP_INFO(get_logger(),"Planning Frame: %s",move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(get_logger(),"End-effector link %s",move_group_->getEndEffectorLink().c_str());
        
        RCLCPP_INFO(get_logger(),"CutProfileServer Initialized"); 


    }




    rclcpp_action::GoalResponse CutProfileServer::handleGoal(const rclcpp_action::GoalUUID & , 
    std::shared_ptr<const CutProfile::Goal> goal){
        
        // RCLCPP_INFO(get_logger(),"recieved inspection goal with %zu viewpoints", goal->viewpoints.size());
        // if (goal-> < 1){
        //     RCLCPP_WARN(get_logger(), "Rejecting, no viewpoints");
        //     return rclcpp_action::GoalResponse::REJECT;
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

    rclcpp_action::CancelResponse CutProfileServer::handleCancel(const std::shared_ptr<GoalHandleCutProfile> ){
        RCLCPP_INFO(get_logger(),"cancel requested");
    
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CutProfileServer::handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle){
        std::thread{
            std::bind(&CutProfileServer::execute,
            this,
            goal_handle        
            )
        }.detach();
    }

    void CutProfileServer::profileEstimateCallback(
        const hb_robot_interfaces::msg::ProfileEstimate::SharedPtr msg
    ){
        std::lock_guard<std::mutex>lock(estimate_mutex_);
        latest_estimate_ = *msg;
    }

    std::optional<hb_robot_interfaces::msg::ProfileEstimate> CutProfileServer::latestProfileEstimate()const{
        std::lock_guard<std::mutex> lock(estimate_mutex_);
        return latest_estimate_;
    }

    void CutProfileServer::publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& stage){
                auto feedback = std::make_shared<CutProfile::Feedback>();
                feedback->stage = stage;
                goal_handle->publish_feedback(feedback);
            }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> CutProfileServer::makeLinPlan(
        const moveit::core::RobotState& start_state,
        const   moveit::core::RobotState& goal_state){
                move_group_->setStartState(start_state);
                const Eigen::Isometry3d goal_tf = goal_state.getGlobalLinkTransform(plasma_link_);
                geometry_msgs::msg::Pose goal_pose = tf2::toMsg(goal_tf);
                move_group_->setPoseTarget(goal_pose, plasma_link_);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                const auto result = move_group_->plan(plan);
                if(result!= moveit::core::MoveItErrorCode::SUCCESS){
                    return std::nullopt;
                }  
                return plan;
            };

    
        std::optional<motion::SegmentMotionPlan> CutProfileServer::planSegmentPilzLinear(
            const motion::CutSegment& segment
        ){
            motion::SegmentMotionPlan segment_motion_plan;
            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            move_group_->setMaxVelocityScalingFactor(0.1);
            move_group_->setMaxAccelerationScalingFactor(0.1);
            move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_->setPlannerId("LIN");
            // try the approach linear
            auto approach_ = makeLinPlan(*segment.approach_state,*segment.start_state);
            if(!approach_){
                RCLCPP_ERROR(get_logger(),"failed on approach linear");
                return std::nullopt;
            }
            segment_motion_plan.approach = approach_.value();
            auto start_ = makeLinPlan(getFinalState(segment_motion_plan.approach),*segment.end_state);
            if(!start_){
                RCLCPP_ERROR(get_logger(),"failed on start linear");
                return std::nullopt;
            }            
            segment_motion_plan.cut = start_.value();
            auto retract_ = makeLinPlan(getFinalState(segment_motion_plan.cut),*segment.retract_state);
            if(!retract_){
                RCLCPP_ERROR(get_logger(),"failed on retract linear");
                return std::nullopt;
            }
            segment_motion_plan.retract = retract_.value();           
            return segment_motion_plan;
        }


    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> CutProfileServer::planConstrainedCut(
                const motion::CutSegment& segment,
                double pos_tol,
                double ang_tol
            ){

                
                const auto path_constraints = cut_planner_->makeBoxConstraints(segment,pos_tol,ang_tol);
                move_group_->clearPoseTargets();
                move_group_->clearPathConstraints();
                move_group_->setPlanningPipelineId("ompl");
                move_group_->setStartState(*segment.start_state);
                move_group_->setPathConstraints(path_constraints);
                Eigen::Isometry3d end_pose = segment.start_pose;
                end_pose.translate(Eigen::Vector3d(0.005,0.0,0.0));


                move_group_->setPoseTarget(tf2::toMsg(end_pose),plasma_link_);
                move_group_->setPlanningTime(planning_time_);
                moveit::planning_interface::MoveGroupInterface::Plan plan;


                kinematic_constraints::KinematicConstraintSet constraint_set(move_group_->getRobotModel());
                constraint_set.add(path_constraints, move_group_->getRobotModel()->getModelFrame());
                auto check_state = [&](const std::string& name, const moveit::core::RobotState& state){
                    const auto result = constraint_set.decide(state,true);


                    const Eigen::Isometry3d tcp = state.getGlobalLinkTransform(plasma_link_);
                     const Eigen::Vector3d local =
                            segment.start_pose.linear().transpose() *
                            (tcp.translation() -
                            0.5 * (segment.start_pose.translation() +
                                    segment.end_pose.translation()));
                      RCLCPP_INFO(
                                get_logger(),
                                "%s: constraint=%s | "
                                "box local [mm] = %.3f %.3f %.3f",
                                name.c_str(),
                                result.satisfied ? "YES" : "NO",
                                1000.0 * local.x(),
                                1000.0 * local.y(),
                                1000.0 * local.z());
                };
                check_state("START", *segment.start_state);
                check_state("END",   *segment.end_state);

                

                moveit::core::RobotState test_goal =
                    *segment.start_state;

                const auto* jmg =
                    test_goal.getJointModelGroup(planning_group_);

                const bool ik_ok =
                    test_goal.setFromIK(
                        jmg,
                        end_pose,
                        plasma_link_,
                        1.0);

                test_goal.update();

                RCLCPP_INFO(
                    get_logger(),
                    "5 mm goal exact IK: %s",
                    ik_ok ? "YES" : "NO");

                if (ik_ok)
                {
                    check_state("5MM IK GOAL", test_goal);
                }


                const auto result = move_group_->plan(plan);
                move_group_->clearPoseTargets();
                move_group_->clearPathConstraints();
                if(result != moveit::core::MoveItErrorCode::SUCCESS){
                    RCLCPP_ERROR(get_logger(),"constrained planner failed");
                    return std::nullopt;

                }

                RCLCPP_ERROR(get_logger(),"constrained planner worked");

                return plan;

                

            }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> CutProfileServer::planToState(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& target_state
    ){
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();
        move_group_->setStartState(start_state);
        move_group_->setJointValueTarget(target_state);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(!static_cast<bool>(move_group_->plan(plan))){
            RCLCPP_ERROR(get_logger(), "failed to plan joint motion");
            return std::nullopt;
        }
        return plan;
    }

    moveit::core::RobotState CutProfileServer::getFinalState(const moveit::planning_interface::MoveGroupInterface::Plan& plan){
        moveit::core::RobotState ref_state (move_group_->getRobotModel());
        moveit::core::robotStateMsgToRobotState(plan.start_state_,ref_state);
        robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(),planning_group_);
        trajectory.setRobotTrajectoryMsg(ref_state,plan.trajectory_);
        return trajectory.getLastWayPoint();

    }

    std::shared_ptr<robot_trajectory::RobotTrajectory> CutProfileServer::planToTrajectory(
        const moveit::planning_interface::MoveGroupInterface::Plan& plan
    ){
        
        const auto robot_model = move_group_->getRobotModel();
        auto combined_ = std::make_shared<robot_trajectory::RobotTrajectory>(
            robot_model, planning_group_);
        moveit::core::RobotState start_state(robot_model);
        moveit::core::robotStateMsgToRobotState(plan.start_state_,start_state);
        robot_trajectory::RobotTrajectory traj(robot_model,planning_group_);
        traj.setRobotTrajectoryMsg(start_state,plan.trajectory_);
        combined_->append(traj,0.0);
        return combined_;
    }


    std::shared_ptr<robot_trajectory::RobotTrajectory> CutProfileServer::collateSegmentTrajectory(
        const motion::SegmentMotionPlan& smp
    ){
        const auto robot_model = move_group_->getRobotModel();
        auto combined_ = std::make_shared<robot_trajectory::RobotTrajectory>(
            robot_model, planning_group_);
        moveit::core::RobotState approach_start(robot_model);
        moveit::core::robotStateMsgToRobotState(smp.approach.start_state_,approach_start);
        robot_trajectory::RobotTrajectory ap_traj(robot_model,planning_group_);
        ap_traj.setRobotTrajectoryMsg(approach_start,smp.approach.trajectory_);
        combined_->append(ap_traj,0.0);
        /// cut
        moveit::core::RobotState cut_start(robot_model);
        moveit::core::robotStateMsgToRobotState(smp.cut.start_state_,cut_start);
        robot_trajectory::RobotTrajectory cut_traj(robot_model,planning_group_);
        cut_traj.setRobotTrajectoryMsg(cut_start,smp.cut.trajectory_);
        combined_->append(cut_traj,0.0);
        /// retract
        moveit::core::RobotState retract_start(robot_model);
        moveit::core::robotStateMsgToRobotState(smp.retract.start_state_,retract_start);
        robot_trajectory::RobotTrajectory ret_traj(robot_model,planning_group_);
        ret_traj.setRobotTrajectoryMsg(retract_start,smp.retract.trajectory_);
        combined_->append(ret_traj,0.0);
        
        return combined_;
    }

    void CutProfileServer::execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle){

        //approval machienry 
        std::lock_guard<std::mutex> lock(approval_mutex_);
        {
            motion_approved_ = false;
        }


        //un pack / get the goal
        auto const goal = goal_handle->get_goal();

        //make empty result
        auto result = std::make_shared<CutProfile::Result>();

        publishFeedback(goal_handle,"acquiring estimate");

        const auto estimate_opt = latestProfileEstimate();
        if(!estimate_opt){
            result->success = false;
            result->message= "couldnt get estimate";
            goal_handle->abort(result);
            return;
        }
        //get the estimate;
        const auto estimate = *estimate_opt;
        RCLCPP_INFO(get_logger(),"Using profile estimate %s | rms %.2f | inlier %.2f | score %.2f", 
        estimate.profile_name.c_str(), estimate.rms_distance,estimate.inlier_fraction,estimate.score);
        RCLCPP_INFO(get_logger(),
        "profile frame %s | Moveit planning frame %s",estimate.header.frame_id.c_str(),move_group_->getPlanningFrame().c_str());
        publishFeedback(goal_handle,"loading profile geom");

        const auto profile_opt = hb_perception::ProfileLibrary::find(estimate.profile_name);
        if(!profile_opt){
            result->success = false;
            std::ostringstream ss;
            ss << "looking up " << estimate.profile_name.c_str();
            result->message= ss.str();
            goal_handle->abort(result);
            return;
        }

        publishFeedback(goal_handle,"generating cut plan");

        motion::CutRequest request;
        request.standoff = goal->standoff;
        request.ang_tol = goal->ang_tol;
        request.pos_tol = goal->pos_tol;
        auto current_state = move_group_->getCurrentState(2.0);
        if(!current_state){
            RCLCPP_ERROR(get_logger(),"couldnt get current state");
            result->success = false;
            result->message= "couldnt get current state";
            goal_handle->abort(result);
            return;
        }
        current_state->update();

        RCLCPP_ERROR(get_logger(),"current state dirty after update: %s",current_state->dirty()?"true":"false");
        
        {
            std::lock_guard<std::mutex>lock(debug_segments_mutex);
            debug_segs_.clear();
        }
        const moveit::core::RobotState arbitrary_start_state = *current_state;

        auto p_scene = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());
        const auto plan_opt = cut_planner_->plan(estimate,*profile_opt,request,*current_state,p_scene);
        if(!plan_opt){
            result->success = false;
            result->message= "planner failed";
            goal_handle->abort(result);
            return;
        }
        const motion::CutPlan plan = *plan_opt;
        RCLCPP_INFO(get_logger(),"generated %zu cut segments",plan.segments.size());


        for (const auto& segment : plan.segments){
            
        // TEST MOTION TO APPROACH TO WEB
            if(segment.type != hb_perception::ProfileCutFeatureType::Web){
                continue;
            }
            current_state->update();
            // free motion to approach
            auto approach_traj = CutProfileServer::planToState(*current_state,*segment.approach_state);
            // lin motion leadin cut retract       
            auto cut_traj = CutProfileServer::planSegmentPilzLinear(segment);
            // return back to arbitrary start
            auto return_traj = CutProfileServer::planToState(*segment.approach_state, arbitrary_start_state);

            // auto test_traj = CutProfileServer::planConstrainedCut(segment,goal->pos_tol,goal->ang_tol);
            if(!cut_traj || !approach_traj || !return_traj){
                result->success = false;
                result->message= "planner failed";
                goal_handle->abort(result);
                return;
            }


            auto combined_trajectory = collateSegmentTrajectory(*cut_traj);
            auto approach_trajectory_obj = CutProfileServer::planToTrajectory(*approach_traj);
            auto retract_trajectory_obj = CutProfileServer::planToTrajectory(*return_traj);

            // auto combined_trajectory = CutProfileServer::planToTrajectory(*test_traj);
            moveit_msgs::msg::RobotTrajectory combined_msg;
            combined_trajectory->getRobotTrajectoryMsg(combined_msg);
            moveit_msgs::msg::DisplayTrajectory display_msg;
            display_msg.model_id = move_group_->getRobotModel()->getName();
            moveit::core::robotStateToRobotStateMsg(combined_trajectory->getFirstWayPoint(),
            display_msg.trajectory_start);
            display_msg.trajectory.push_back(combined_msg);
            display_traj_pub_->publish(display_msg);
            std::cout<<"published trajectory"<<std::endl;
        }


        

        publishVisualization(plan,estimate.header.frame_id);
        publishFeedback(goal_handle,"visualizing, cut plan ready");


        if(!goal->execute){
            result->success = true;
            result->message = "finished";
            goal_handle->succeed(result);
            return;
        }

        motion_approved_ = true;
        if(motion_approved_){
        RCLCPP_INFO(get_logger(),"skip motion is true, no motion executed");
        }
        else{
            RCLCPP_INFO(get_logger(),"skip motion is false, awaiting approval");
            while(rclcpp::ok()){
                if (goal_handle->is_canceling()){
                    result->message = "cancelled";
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
            publishFeedback(goal_handle, "motion approved");
        }

        result->success = true;
        result->message = "cut approved, exec not implemented yet";
        goal_handle->succeed(result);


    }

    
    
    void CutProfileServer::publishCandidateVisualization(const motion::CutSegment segment){
        motion::CutPlan debug_plan;
        debug_plan.profile_name = "debug_";
        {
        std::lock_guard<std::mutex>lock(debug_segments_mutex);
            debug_segs_[segment.name] = segment;

            debug_plan.segments.reserve(debug_segs_.size());

            for( const auto& [name,cached_seg]: debug_segs_){
                debug_plan.segments.push_back(cached_seg);
            }
            
        }
        visualization_pub_->publish(makeVisualization(debug_plan,move_group_->getPlanningFrame()));
    }
    
    namespace
{

geometry_msgs::msg::Point
toPoint(const Eigen::Vector3d& p)
{
    geometry_msgs::msg::Point msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();
    return msg;
}

visualization_msgs::msg::Marker
makeArrow(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int id,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double length)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;

    marker.ns = ns;
    marker.id = id;

    marker.type =
        visualization_msgs::msg::Marker::ARROW;

    marker.action =
        visualization_msgs::msg::Marker::ADD;

    marker.points.push_back(
        toPoint(origin));

    marker.points.push_back(
        toPoint(
            origin +
            direction.normalized() * length));

    marker.scale.x = 0.003;
    marker.scale.y = 0.007;
    marker.scale.z = 0.010;

    marker.color.a = 1.0;

    return marker;
}


/*
 * Draw the process-relevant part of the tool frame.
 *
 * makeToolPose() convention:
 *
 *   X = cut tangent
 *   Z = surface normal / torch axis
 *
 * Green = tangent
 * Red   = surface normal
 */


void drawFrame(
    visualization_msgs::msg::MarkerArray& array,
    const Eigen::Isometry3d& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int& id,
    double length = 0.04,
    double alpha = 1.0    
)
{
    const Eigen::Vector3d origin =
        pose.translation();

    const Eigen::Vector3d tangent =
        pose.linear().col(0);

    const Eigen::Vector3d normal =
        pose.linear().col(2);


    auto tangent_arrow =
        makeArrow(
            frame_id,
            stamp,
            ns + "_tangent",
            id++,
            origin,
            tangent,
            length);

    tangent_arrow.color.r = 0.0;
    tangent_arrow.color.g = 1.0;
    tangent_arrow.color.b = 0.0;
    tangent_arrow.color.a = alpha;

    array.markers.push_back(
        std::move(tangent_arrow));


    auto normal_arrow =
        makeArrow(
            frame_id,
            stamp,
            ns + "_normal",
            id++,
            origin,
            normal,
            length);

    normal_arrow.color.r = 1.0;
    normal_arrow.color.g = 0.0;
    normal_arrow.color.b = 0.0;
    normal_arrow.color.a = alpha;

    array.markers.push_back(
        std::move(normal_arrow));
}


void drawRobotStateFrame(
visualization_msgs::msg::MarkerArray& array,
const moveit::core::RobotState& state,
const std::string link_name,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int& id,
    double length,
    double alpha

){
    const Eigen::Isometry3d act_pose = state.getGlobalLinkTransform(link_name);
    drawFrame(array,act_pose,frame_id,stamp,ns,id, length,alpha);

}
void makeLabel(
    visualization_msgs::msg::MarkerArray& array,
    const Eigen::Vector3d& position,
    const std::string& text,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int& id)
{
    visualization_msgs::msg::Marker label;

    label.header.frame_id = frame_id;
    label.header.stamp = stamp;

    label.ns = ns;
    label.id = id++;

    label.type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    label.action =
        visualization_msgs::msg::Marker::ADD;

    label.pose.position =
        toPoint(
            position +
            Eigen::Vector3d(
                0.0,
                0.0,
                0.025));

    label.pose.orientation.w = 1.0;

    label.scale.z = 0.018;

    label.color.r = 1.0;
    label.color.g = 1.0;
    label.color.b = 1.0;
    label.color.a = 1.0;

    label.text = text;

    array.markers.push_back(
        std::move(label));
}

} // namespace
visualization_msgs::msg::MarkerArray
CutProfileServer::makeVisualization(
    const motion::CutPlan& plan,
    const std::string& frame_id) const
{
    visualization_msgs::msg::MarkerArray array;

    const auto stamp = now();

    /*
     * Clear previous visualization.
     */
    visualization_msgs::msg::Marker clear;

    clear.action =
        visualization_msgs::msg::Marker::DELETEALL;

    array.markers.push_back(clear);


    int id = 0;


    for (const auto& segment : plan.segments)
    {
        /*
         * --------------------------------------------------
         * Actual requested cut line
         * --------------------------------------------------
         *
         * Use the process poses rather than CutPathPoint here.
         * These already include standoff.
         */
        visualization_msgs::msg::Marker cut_line;

        cut_line.header.frame_id = frame_id;
        cut_line.header.stamp = stamp;

        cut_line.ns = "cut_segments";
        cut_line.id = id++;

        cut_line.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        cut_line.action =
            visualization_msgs::msg::Marker::ADD;

        cut_line.scale.x = 0.005;

        cut_line.points.push_back(
            toPoint(
                segment.start_pose.translation()));

        cut_line.points.push_back(
            toPoint(
                segment.end_pose.translation()));

        // Cyan
        cut_line.color.r = 0.0;
        cut_line.color.g = 1.0;
        cut_line.color.b = 1.0;
        cut_line.color.a = 1.0;

        array.markers.push_back(
            std::move(cut_line));


        /*
         * --------------------------------------------------
         * Approach -> start
         * --------------------------------------------------
         *
         * Thin line so we can see the intended approach
         * direction as well.
         */
        visualization_msgs::msg::Marker approach_line;

        approach_line.header.frame_id = frame_id;
        approach_line.header.stamp = stamp;

        approach_line.ns = "approach_paths";
        approach_line.id = id++;

        approach_line.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        approach_line.action =
            visualization_msgs::msg::Marker::ADD;

        approach_line.scale.x = 0.002;

        approach_line.points.push_back(
            toPoint(
                segment.approach_pose.translation()));

        approach_line.points.push_back(
            toPoint(
                segment.start_pose.translation()));

        approach_line.color.r = 1.0;
        approach_line.color.g = 1.0;
        approach_line.color.b = 1.0;
        approach_line.color.a = 0.6;

        array.markers.push_back(
            std::move(approach_line));


        /*
         * --------------------------------------------------
         * End -> retract
         * --------------------------------------------------
         */
        visualization_msgs::msg::Marker retract_line;

        retract_line.header.frame_id = frame_id;
        retract_line.header.stamp = stamp;

        retract_line.ns = "retract_paths";
        retract_line.id = id++;

        retract_line.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        retract_line.action =
            visualization_msgs::msg::Marker::ADD;

        retract_line.scale.x = 0.002;

        retract_line.points.push_back(
            toPoint(
                segment.end_pose.translation()));

        retract_line.points.push_back(
            toPoint(
                segment.retract_pose.translation()));

        retract_line.color.r = 1.0;
        retract_line.color.g = 1.0;
        retract_line.color.b = 1.0;
        retract_line.color.a = 0.6;

        array.markers.push_back(
            std::move(retract_line));


        /*
         * --------------------------------------------------
         * Nominal tool frames
         * --------------------------------------------------
         */

        const std::string base_ns =
            "cut_pose_" + segment.name;

        drawFrame(
            array,
            segment.approach_pose,
            frame_id,
            stamp,
            base_ns + "_approach",
            id,0.04,0.35);

        drawFrame(
            array,
            segment.start_pose,
            frame_id,
            stamp,
            base_ns + "_start",
            id,0.04,0.35);

        drawFrame(
            array,
            segment.end_pose,
            frame_id,
            stamp,
            base_ns + "_end",
            id,0.04,0.35);

        drawFrame(
            array,
            segment.retract_pose,
            frame_id,
            stamp,
            base_ns + "_retract",
            id,0.04,0.35);

        if (segment.start_state){
            drawRobotStateFrame(array, *segment.start_state,plasma_link_,frame_id,stamp,base_ns+"_start",id,0.02,1.0);
        }
        if (segment.approach_state){
            drawRobotStateFrame(array, *segment.approach_state,plasma_link_,frame_id,stamp,base_ns+"_appr",id,0.02,1.0);
        }
        if (segment.end_state){
            drawRobotStateFrame(array, *segment.end_state,plasma_link_,frame_id,stamp,base_ns+"_end",id,0.02,1.0);
        }
        if (segment.retract_state){
            drawRobotStateFrame(array, *segment.retract_state,plasma_link_,frame_id,stamp,base_ns+"_retr",id,0.02,1.0);
        }
        /*
         * --------------------------------------------------
         * Labels
         * --------------------------------------------------
         */

        makeLabel(
            array,
            segment.approach_pose.translation(),
            segment.name + " approach",
            frame_id,
            stamp,
            base_ns + "_labels",
            id);

        makeLabel(
            array,
            segment.start_pose.translation(),
            segment.name + " start",
            frame_id,
            stamp,
            base_ns + "_labels",
            id);

        makeLabel(
            array,
            segment.end_pose.translation(),
            segment.name + " end",
            frame_id,
            stamp,
            base_ns + "_labels",
            id);

        makeLabel(
            array,
            segment.retract_pose.translation(),
            segment.name + " retract",
            frame_id,
            stamp,
            base_ns + "_labels",
            id);
    }


    return array;
}


void
CutProfileServer::publishVisualization(
    const motion::CutPlan& plan,
    const std::string& frame_id)
{
    visualization_pub_->publish(
        makeVisualization(
            plan,
            frame_id));
}

    void CutProfileServer::handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
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




}


int main (int argc, char** argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions();//.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hb_robot_skills::CutProfileServer>(options);

    try{
    node->initialize();
    RCLCPP_INFO(node->get_logger(),"initalized cut profile server");
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