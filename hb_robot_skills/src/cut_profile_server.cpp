#include "hb_robot_skills/cut_profile_server.hpp"
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;
namespace hb_robot_skills{

    CutProfileServer::CutProfileServer(const rclcpp::NodeOptions &options):Node("cut_profile_server",options){
        if(!has_parameter("planning_group")){
            planning_group_ = declare_parameter<std::string>("planning_group", "manipulator");
        }
        get_parameter("planning_group",planning_group_);
        
        if(!has_parameter("plasma_link")){
            plasma_link_ = declare_parameter<std::string>("plasma_link","ur10e_torch_link");
        }
        get_parameter("plasma_link",plasma_link_);
        if(!has_parameter("planning_time")){
            planning_time_ = declare_parameter<double>("planning_time",5.0);
        }
        get_parameter("planning_time",planning_time_);


        if(!has_parameter("require_plan_approval")){
            require_plan_approval_ = declare_parameter<bool>("require_plan_approval",true);
        }
        get_parameter("require_plan_approval",require_plan_approval_);

        if(!has_parameter("skip_cutting_motion")){
            skip_motion_ = declare_parameter<bool>("skip_cutting_motion",false);
        }
        get_parameter("skip_cutting_motion",skip_motion_);




        approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_cut_motion",
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



        profile_estimate_sub_ = create_subscription<hb_robot_interfaces::msg::ProfileEstimate>("/profile_inspection/estimate",
        estimate_qos, std::bind(&CutProfileServer::profileEstimateCallback,this,
        std::placeholders::_1));

        auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
        visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cut_profile/cut_plan",marker_qos);
        collision_debug_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
        "/cut_profile/debug_collision", marker_qos);
        debug_robot_pub_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("/cut_profile/debug_robot",marker_qos);

    }

    void CutProfileServer::initialize(){

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
        move_group_->setPlanningTime(planning_time_);
        
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(),"robot_description");

        if(!planning_scene_monitor_->getPlanningScene()){
            throw std::runtime_error("failed to create planning scene");
        }
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor();
        const bool scene_recieved  = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
        if(!scene_recieved){
             RCLCPP_INFO(
                get_logger(),
                "couldnt synchoroniz psm with move group");
            throw std::runtime_error("cuttin gplan sync failed");
            
        }
        RCLCPP_INFO(
                get_logger(),
                "synchoronized epsm with move group");
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

        cut_planner_->setCollisionDebugVisCallback([this](const motion::CollisionDebugEvent& event){
            publishCollisionDebug(event);
        });

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
        RCLCPP_INFO(get_logger(),"Skipping Motion %s",skip_motion_?"yes":"no");
        
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
        if(move_group_){
            move_group_->stop();

        }
        approval_cv_.notify_all();

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
            const motion::CutSegment& segment, const moveit::core::RobotState& actual_approach
        ){
            motion::SegmentMotionPlan segment_motion_plan;
            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            move_group_->setMaxVelocityScalingFactor(0.1);
            move_group_->setMaxAccelerationScalingFactor(0.1);
            move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_->setPlannerId("LIN");
            // try the approach linear
            auto approach_ = makeLinPlan(actual_approach,*segment.start_state);
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
        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPlanningTime(planning_time_);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
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


    CutProfileServer::MotionOutcome CutProfileServer::waitForPlanApproval(
                const std::shared_ptr<GoalHandleCutProfile> goal_handle
            ){

                if (!require_plan_approval_) {
                        return MotionOutcome::SUCCESS;
                    }

                RCLCPP_INFO(
                    get_logger(),
                    "Waiting for cutting plan approval"
                );
                publishFeedback(goal_handle, "awaiting plan approval");

                std::unique_lock<std::mutex> lock(approval_mutex_);

                approval_cv_.wait(lock, [&]() {
                    return motion_approved_ ||
                        motion_rejected_ ||
                        goal_handle->is_canceling() ||
                        !rclcpp::ok();
                });

                if (goal_handle->is_canceling()) {
                    return MotionOutcome::CANCELED;
                }

                if (!rclcpp::ok()) {
                    return MotionOutcome::FAILED;
                }

                if (motion_rejected_) {
                    return MotionOutcome::REJECTED;
                }

                return MotionOutcome::SUCCESS;
            }


    CutProfileServer::MotionOutcome CutProfileServer::executeMotion(
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::shared_ptr<GoalHandleCutProfile> goal_handle,
        const std::string& description
    ){

        if(goal_handle->is_canceling()){
            return MotionOutcome::CANCELED;
        }
        RCLCPP_INFO(
        get_logger(),
        "Executing: %s",
        description.c_str()
    );

    publishFeedback(goal_handle, description);

    const auto execution_result = move_group_->execute(plan);
    if(goal_handle->is_canceling()){
        return MotionOutcome::CANCELED;
    }
    if(!static_cast<bool>(execution_result)){
        
        RCLCPP_ERROR(
            get_logger(),
            "Motion execution failed: %s",
            description.c_str()
        );
        return MotionOutcome::FAILED;
    }
    return MotionOutcome::SUCCESS;
    }
            

    CutProfileServer::MotionOutcome CutProfileServer::executeCuttingPlan(const motion::ExecutablePlan& executable_plan_,
                const std::shared_ptr<GoalHandleCutProfile>goal_handle){

        for (const auto& cut: executable_plan_.cuts){
            RCLCPP_INFO(
            get_logger(),
            "Starting cut: %s",
            cut.name.c_str());

            auto run = [&](const auto& plan, const std::string& phase){
                return executeMotion(
                    plan,goal_handle,cut.name+": " + phase
                );
            };


            auto outcome = run(cut.transit_in,"transit in");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            outcome = run(cut.process.approach,"approach");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            outcome = run(cut.process.cut,"cut");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }

            outcome = run(cut.process.retract,"retract");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            outcome = run(cut.transit_out,"transit out");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            

         }
         return MotionOutcome::SUCCESS;
        }

    void CutProfileServer::execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle){

        //approval machienry 
        
        std::lock_guard<std::mutex> execution_lock(execution_mutex_);      

        //un pack / get the goal
        auto const goal = goal_handle->get_goal();

        //make empty result
        auto result = std::make_shared<CutProfile::Result>();

        {
        std::lock_guard<std::mutex> approval_lock(
            approval_mutex_);
            motion_approved_ = false;
            motion_rejected_ = false;
        }

        // handy lamba expressions 

        auto fail = [&](uint8_t code, const std::string& message)
        {
            result->success = false;
            result->message = message;
            goal_handle->abort(result);
        };

        auto canceled = [&]()
        {
            if (!goal_handle->is_canceling())
                return false;

            move_group_->stop();
            result->message = "Cutting Motion cancelled";
            goal_handle->canceled(result);
            return true;
        };

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
        clearCollisionDebug();
        const moveit::core::RobotState arbitrary_start_state = *current_state;

        // 
        auto check_psm =[](const planning_scene::PlanningScenePtr& scene_)->bool{
            const auto* world = scene_->getWorld().get();
            if(!scene_){
                return false;
            }

            return    (!world->hasObject("inspection_workpiece") &&
                world->hasObject("wk_workpiece_collision") &&
                world->hasObject("wk_section_collision"));
            };

        /// check the planning scene
        moveit::planning_interface::PlanningSceneInterface psi;
        const auto objects = psi.getObjects({"inspection_workpiece", "wk_workpiece_collision","wk_section_collision"});
        
        const bool authoritative_ready =
            objects.count("inspection_workpiece") == 0 &&
            objects.count("wk_workpiece_collision") == 1 &&
            objects.count("wk_section_collision") == 1;

        if (!authoritative_ready)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "move_group scene is not ready for cutting"
                );

                fail(0, "Authoritative cutting scene is incomplete");
                return;
            }
        const bool request_psm_update = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");

        if(!request_psm_update){
             RCLCPP_INFO(
                get_logger(),
                "request update failed");
           
            
        }
        RCLCPP_INFO(
                get_logger(),
                "retrieved planning_scene");

        planning_scene::PlanningScenePtr scene_snapshot;

        {
            planning_scene_monitor::LockedPlanningSceneRO locked_scene(
                planning_scene_monitor_
            );

            if (!locked_scene) {
                fail(0, "Could not lock local planning scene");
                return;
            }

            scene_snapshot = locked_scene->diff();
            scene_snapshot->decoupleParent();
        }
        

       if (check_psm(scene_snapshot)) {

            RCLCPP_INFO(
                get_logger(),
                "Local planning scene is synchronized; "
                "using snapshot directly"
            );

        } else {

            RCLCPP_WARN(
                get_logger(),
                "Local planning scene is missing cutting geometry; "
                "reconciling snapshot"
            );

            for (const auto& obj: {"inspection_workpiece", "wk_workpiece_collision","wk_section_collision"}){
            
                scene_snapshot->getWorldNonConst()->removeObject(obj);

            }
            for (const auto& obj: {"wk_workpiece_collision","wk_section_collision"}){
                auto object = objects.at(obj);

                object.operation =
                    moveit_msgs::msg::CollisionObject::ADD;

                if (!scene_snapshot->processCollisionObjectMsg(object))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to copy %s into local cutting scene",
                        obj
                    );

                    fail(0, "Could not copy cutting collision geometry");
                    return;
                }
            }
            RCLCPP_INFO(
                    get_logger(),
                    "Cutting scene snapshot reconciled successfully"
                );


        }        
        const auto* world = scene_snapshot->getWorld().get();

        RCLCPP_INFO(
            get_logger(),
            "FINAL CUTTING SNAPSHOT: box=%d coarse=%d section=%d",
            world->hasObject("inspection_workpiece"),
            world->hasObject("wk_workpiece_collision"),
            world->hasObject("wk_section_collision")
        );

        if (!check_psm(scene_snapshot)) {
            RCLCPP_ERROR(
                get_logger(),
                "Final cutting scene snapshot is invalid"
            );

            fail(0, "Cutting scene validation failed");
            return;
        }

        const auto plan_opt = cut_planner_->plan(estimate,*profile_opt,request,*current_state,scene_snapshot);
        if(!plan_opt){
            result->success = false;
            result->message= "planner failed";
            goal_handle->abort(result);
            return;
        }
        const motion::CutPlan plan = *plan_opt;
        RCLCPP_INFO(get_logger(),"generated %zu cut segments",plan.segments.size());

        motion::ExecutablePlan executable_plan_;
        executable_plan_.plan_id = "cut_plan_001";
        executable_plan_.estimate_id = estimate.profile_name;




            for (const auto& segment : plan.segments){
                
                RCLCPP_INFO(get_logger(), "planning trajectory for %s",segment.name.c_str());

                auto transit_in_traj = CutProfileServer::planToState(arbitrary_start_state,*segment.approach_state);
                if(!transit_in_traj){
                    RCLCPP_WARN(get_logger(), "Couldnt get to %s approach", segment.name.c_str());
                    executable_plan_.cuts.clear();
                    break;
                }
                auto actual_appr = getFinalState(*transit_in_traj);
                // lin motion leadin cut retract       
                auto cut_smp = CutProfileServer::planSegmentPilzLinear(segment, actual_appr);
                // return back to arbitrary start
                if(!cut_smp){
                    RCLCPP_WARN(get_logger(), "Couldnt plan cuttin %s", segment.name.c_str());
                    executable_plan_.cuts.clear();
                    break;
                }
                auto actual_retr = getFinalState(cut_smp->retract);

                auto transit_out_traj = CutProfileServer::planToState(actual_retr,arbitrary_start_state);

                if(!transit_out_traj){
                    RCLCPP_WARN(get_logger(), "Couldnt return from %s retract", segment.name.c_str());
                    executable_plan_.cuts.clear();
                    break;
                }

                motion::ExecutableCut ex_cut_;
                ex_cut_.name = segment.name;
                ex_cut_.transit_in = *transit_in_traj;
                ex_cut_.transit_out = *transit_out_traj;
                ex_cut_.process = *cut_smp;
                executable_plan_.cuts.push_back(std::move(ex_cut_));

            }

            if(executable_plan_.cuts.empty()){
                result->success = false;
                result->message= "planner failed";
                goal_handle->abort(result);
                return;
            }
            const auto robot_model = move_group_->getRobotModel();

            using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
            moveit_msgs::msg::DisplayTrajectory display_msg;

            display_msg.model_id = move_group_->getRobotModel()->getName();

            auto appendPlan = [&](const Plan& plan, const std::string& name)->bool{
              const auto & jt = plan.trajectory_.joint_trajectory;
              if(jt.joint_names.empty() || jt.points.empty()){
                RCLCPP_ERROR(get_logger(), "emtpy trajectory, %s", name.c_str());
                return false;
              }

              RCLCPP_INFO(get_logger(),"%s: start joints = %zu, traj joint= %zu, points = %zu",
            name.c_str(),plan.start_state_.joint_state.name.size(),jt.joint_names.size(),jt.points.size());
            display_msg.trajectory.push_back(plan.trajectory_);
            return true;
            };

            if(executable_plan_.cuts.empty()){
                fail(0,"no executable cuts");
                return;

            }

            const auto& first_plan = executable_plan_.cuts.front().transit_in;
            display_msg.trajectory_start = first_plan.start_state_;
            if(display_msg.trajectory_start.joint_state.name.empty()){
                 moveit::core::robotStateToRobotStateMsg(arbitrary_start_state,
                    display_msg.trajectory_start);
            }
            bool valid = true;
            // auto full_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model,planning_group_);
            RCLCPP_INFO(get_logger(), "got %zu executable cuts",executable_plan_.cuts.size());
           
            for (const auto& cut: executable_plan_.cuts){
                valid&= appendPlan(cut.transit_in, cut.name + "/transit_in");
                valid&= appendPlan(cut.process.approach, cut.name + "/approach");
                valid&= appendPlan(cut.process.cut, cut.name + "/cut");
                valid&= appendPlan(cut.process.retract, cut.name + "/retract");
                valid&= appendPlan(cut.transit_out, cut.name + "/transit_out");
            }
            if(!valid){
                fail(0,"empty trajectories found");
                return;
            }
            display_traj_pub_->publish(display_msg);
            RCLCPP_INFO(get_logger(), "published %zu cutting traj", display_msg.trajectory.size());
            
        
       

        publishVisualization(plan,estimate.header.frame_id);
        publishFeedback(goal_handle,"visualizing, cut plan ready");

            // cutting actions happen below now that cuts are planned
        if(skip_motion_ ||  !goal->execute){
            result->success = true;
            result->message= "skipping motion, dry run planned not executed";
            goal_handle->succeed(result);
            return;

        }

        if(canceled()){
            return;
        }
        const auto approval = waitForPlanApproval(goal_handle);
        if(approval != MotionOutcome::SUCCESS){
            if(approval == MotionOutcome::CANCELED){
                canceled();
                return;
            }
            fail(
                0,
                approval == MotionOutcome::REJECTED
            ? "Cutting plan rejected"
            : "Cutting plan approval failed"
            );
            return;  
        }
        // maybe revalidate scene planning here
        
        const auto execution_success = executeCuttingPlan(executable_plan_,goal_handle);
        
        switch (execution_success) {
            case MotionOutcome::SUCCESS:
                result->success = true;
                result->message = "Cutting job completed";
                goal_handle->succeed(result);
                return;

            case MotionOutcome::CANCELED:
                canceled();
                return;

            case MotionOutcome::REJECTED:
                fail(0, "Cutting execution rejected");
                return;

            case MotionOutcome::FAILED:
                fail(0, "Cutting execution failed");
                return;
        }
    }

    
    void CutProfileServer::clearCollisionDebug()
{
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;

    clear.action =
        visualization_msgs::msg::Marker::DELETEALL;

    array.markers.push_back(clear);

    collision_debug_pub_->publish(array);
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

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
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

void CutProfileServer::publishCollisionDebug(
    const motion::CollisionDebugEvent& event)
{
    using Marker = visualization_msgs::msg::Marker;

    visualization_msgs::msg::MarkerArray array;

    const auto stamp = now();
    const std::string frame_id = move_group_->getPlanningFrame();

    // Separate namespace for each sample/iteration.
    // This preserves the history of the correction steps in RViz.
    const std::string ns =
        "sample_" + std::to_string(event.sample) +
        "_iter_" + std::to_string(event.iter);

    int id = 0;

    // -------------------------------------------------------
    // 1. Nominal TCP frame
    // -------------------------------------------------------

    drawFrame(
        array,
        event.nominal_tcp_pose,
        frame_id,
        stamp,
        ns + "_nominal_tcp",
        id,
        0.045,
        0.35);

    // -------------------------------------------------------
    // 2. Proposed TCP frame
    // -------------------------------------------------------

    const Eigen::Isometry3d actual_tcp =
        event.state.getGlobalLinkTransform(plasma_link_);

    drawFrame(
        array,
        actual_tcp,
        frame_id,
        stamp,
        ns + "_actual_tcp",
        id,
        0.035,
        1.0);

    // -------------------------------------------------------
    // 3. TCP displacement from nominal
    // -------------------------------------------------------

    const Eigen::Vector3d nominal_pos =
        event.nominal_tcp_pose.translation();

    const Eigen::Vector3d actual_pos =
        actual_tcp.translation();

    const Eigen::Vector3d tcp_delta =
        actual_pos - nominal_pos;

    if (tcp_delta.norm() > 1e-6)
    {
        auto arrow = makeArrow(
            frame_id,
            stamp,
            ns + "_tcp_displacement",
            id++,
            nominal_pos,
            tcp_delta,
            tcp_delta.norm());

        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.2;
        arrow.color.a = 1.0;

        array.markers.push_back(std::move(arrow));
    }

    // -------------------------------------------------------
    // 4. Collision contacts
    // -------------------------------------------------------

    for (const auto& contact : event.contacts)
    {
        // Contact point sphere
        Marker sphere;

        sphere.header.frame_id = frame_id;
        sphere.header.stamp = stamp;

        sphere.ns = ns + "_contacts";
        sphere.id = id++;

        sphere.type = Marker::SPHERE;
        sphere.action = Marker::ADD;

        sphere.pose.position = toPoint(contact.pos_world);
        sphere.pose.orientation.w = 1.0;

        sphere.scale.x = 0.0015;
        sphere.scale.y = 0.0015;
        sphere.scale.z = 0.0015;

        sphere.color.r = 1.0;
        sphere.color.g = 0.0;
        sphere.color.b = 0.0;
        sphere.color.a = 0.9;

        array.markers.push_back(std::move(sphere));

        // Contact separation normal
        if (contact.normal_world.norm() > 1e-8)
        {
            auto arrow = makeArrow(
                frame_id,
                stamp,
                ns + "_contact_normals",
                id++,
                contact.pos_world,
                contact.normal_world,
                0.03);

            arrow.color.r = 1.0;
            arrow.color.g = 0.5;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;

            array.markers.push_back(std::move(arrow));
        }

        // Label the colliding link and penetration depth
        std::ostringstream ss;

        ss << contact.robot_link
           << " "
           << std::fixed
           << std::setprecision(1)
           << (1000.0 * contact.depth)
           << " mm";

        makeLabel(
            array,
            contact.pos_world,
            ss.str(),
            frame_id,
            stamp,
            ns + "_contact_labels",
            id);
    }

    // -------------------------------------------------------
    // 5. Iteration/status label
    // -------------------------------------------------------

    std::ostringstream status;

    status
        << event.label
        << " | sample " << event.sample
        << " | iter " << event.iter
        << " | " << event.strategy
        << " | contacts " << event.contacts.size()
        << " | TCP offset "
        << std::fixed << std::setprecision(2)
        << (1000.0 * tcp_delta.norm())
        << " mm";

    makeLabel(
        array,
        actual_pos,
        status.str(),
        frame_id,
        stamp,
        ns + "_status",
        id);

    // -------------------------------------------------------
    // 6. Publish markers and proposed robot state
    // -------------------------------------------------------

    collision_debug_pub_->publish(array);

    publishDebugRobot(event.state);
}
    
    void CutProfileServer::publishDebugRobot(
    const moveit::core::RobotState& state)
{
    moveit_msgs::msg::DisplayRobotState msg;

    moveit::core::robotStateToRobotStateMsg(
        state, msg.state);

    msg.highlight_links.clear();

    debug_robot_pub_->publish(msg);
}

void CutProfileServer::handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
        std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response){
            
            {
                std::lock_guard<std::mutex> lock(approval_mutex_);
            motion_approved_ = request->approve;
            motion_rejected_ = !request->approve;
        }
            approval_cv_.notify_all();

            response->accepted = true;

            response->message =
                request->approve
                    ? "Cutting plan approved"
                    : "Cutting plan rejected";

        }




}


int main (int argc, char** argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
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