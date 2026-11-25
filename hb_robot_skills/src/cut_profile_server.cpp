#include "hb_robot_skills/motion/named_cut_posture.hpp"
#include "hb_robot_skills/motion/transit_planner.hpp"
#include <set>
#include <cstdint>
#include <future>
#include "hb_robot_skills/cut_profile_server.hpp"
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "hb_robot_skills/workpiece_commit_validation.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>


using namespace std::chrono_literals;

namespace hb_robot_skills{

namespace {
// Local to the planning worker: recovery/height callbacks never inherit it.
class PlanningCanceled final : public std::runtime_error {
public:
    PlanningCanceled() : std::runtime_error("Cut planning canceled before execution") {}
};
thread_local std::function<bool()> planning_canceled;
class PlanningCancellationScope {
public:
    explicit PlanningCancellationScope(std::function<bool()> canceled)
        : previous_(std::move(planning_canceled)) {
        planning_canceled = std::move(canceled);
    }
    ~PlanningCancellationScope() { planning_canceled = std::move(previous_); }
    PlanningCancellationScope(const PlanningCancellationScope&) = delete;
    PlanningCancellationScope& operator=(const PlanningCancellationScope&) = delete;
    void clear() { planning_canceled = {}; }
private:
    std::function<bool()> previous_;
};
void checkPlanningCancellation() {
    if (planning_canceled && planning_canceled()) throw PlanningCanceled();
}
} // namespace


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

         if(!has_parameter("task_state_timeout_ms")){
            task_state_timeout_ms_ = declare_parameter<int>("task_state_timeout_ms",1000);
        }
        get_parameter("task_state_timeout_ms",task_state_timeout_ms_);
        

        if(!has_parameter("require_plan_approval")){
            require_plan_approval_ = declare_parameter<bool>("require_plan_approval",true);
        }
        get_parameter("require_plan_approval",require_plan_approval_);

        if(!has_parameter("skip_cutting_motion")){
            skip_motion_ = declare_parameter<bool>("skip_cutting_motion",false);
        }
        get_parameter("skip_cutting_motion",skip_motion_);

         if(!has_parameter("transit.velocity_scaling")){
        transit_velocity_scaling_ =
            declare_parameter<double>(
                "transit.velocity_scaling", 0.05);
            }
        get_parameter("transit.velocity_scaling",transit_velocity_scaling_);
        
         if(!has_parameter("transit.acceleration_scaling")){
        transit_acceleration_scaling_ =
            declare_parameter<double>(
                "transit.acceleration_scaling", 0.01);
            }
        get_parameter("transit.acceleration_scaling",transit_acceleration_scaling_);

        // A restarted cutting server must not reuse an earlier evidence epoch.
        execution_version_=static_cast<uint64_t>(now().nanoseconds());
        if(!has_parameter("enable_cutting_process")){
            process_enabled_=declare_parameter<bool>("enable_cutting_process",false);
        }
        get_parameter("enable_cutting_process",process_enabled_);
        if (!has_parameter("flap_travel_ms")) declare_parameter<int>("flap_travel_ms", 500);
        flap_travel_time_ = std::chrono::milliseconds(get_parameter("flap_travel_ms").as_int());
        if (flap_travel_time_.count() < 0) throw std::invalid_argument("flap_travel_ms must be nonnegative");
        flap_client_ = create_client<std_srvs::srv::SetBool>("/camera_flap/set_closed");
        task_sub_=create_subscription<TaskState>("/task_state",rclcpp::QoS(1).transient_local(),
          [this](TaskState::ConstSharedPtr msg) {std::lock_guard<std::mutex> lock(task_mutex_);
            task_state_=*msg;task_received_=std::chrono::steady_clock::now();});
        execution_pub_=create_publisher<std_msgs::msg::UInt64>(
          "/cut_profile/execution_version",rclcpp::QoS(1).transient_local());
        supervision_timer_=create_wall_timer(20ms,[this]{
          superviseHeight();
          if(retreat_active_) {

            TaskState state;
            {
                std::lock_guard<std::mutex> lock(task_mutex_);
                state=task_state_;
              if(std::chrono::steady_clock::now()-task_received_>std::chrono::milliseconds(task_state_timeout_ms_)) state.supervision_valid=false;
            }
            const auto t=torch_controller_->snapshot();
            if(!state.supervision_valid || !state.imu_stable || !state.workpiece_pose_valid ||
               state.motion_fault_active || !torch_controller_->hasFreshIO() ||
               t.output_on!=std::optional<bool>(false) ||
               t.transfer_detected!=std::optional<bool>(false) ||
               !hb_robot::recovery::sameVersions(state,execution_state_)) {
              cut_stop_requested_=true;
              torch_controller_->emergencyShutdown("Retreat supervision lost");
              if(move_group_) move_group_->stop();
            }
          }
          if (recovery_motion_active_.load()) {
                const bool torch_safe =
                    torch_controller_ &&
                    torch_controller_->hasFreshIO() &&
                    !torch_controller_->hasFault();

                bool off_confirmed = false;

                if (torch_safe) {
                    const auto t = torch_controller_->snapshot();
                    off_confirmed =
                        t.output_on == std::optional<bool>(false) &&
                        t.transfer_detected == std::optional<bool>(false);
                }

                const auto task = taskStateFreshness();
                if (!off_confirmed || !heightAtHomeAndReady(false) || !task.fresh ||
                    !hb_robot::recovery::geometryPermitted(task.state) ||
                    !hb_robot::recovery::sameVersions(task.state, execution_state_) ||
                    task.state.cut_environment_id != execution_state_.cut_environment_id) {
                    latchObservedMotionStop("recovery_hardware_or_authorization_lost");

                    if (torch_controller_)
                        torch_controller_->requestOutputOff();

                    if (move_group_)
                        move_group_->stop();
                }
            }
        
        
        if (!motion_active_)
            return;



        const auto task = taskStateFreshness();

        const bool geometry_ok =
            hb_robot::recovery::geometryPermitted(task.state);

        const bool permitted =
            task.fresh && geometry_ok;

        const bool versions_match =
            hb_robot::recovery::sameVersions(
                task.state, execution_state_);

        if (!permitted || !versions_match)
        {
             
            
            if (latchObservedMotionStop("supervision_fault")) {
                RCLCPP_ERROR(
            get_logger(),
            "Execution supervision lost: "
            "task_received=%d task_fresh=%d "
            "task_age_ms=%.1f "
            "geometry_ok=%d versions_match=%d "
            "scene_valid=%d pose_valid=%d "
            "supervision_valid=%d imu_stable=%d "
            "motion_fault=%d process_fault=%d "
            "geometry=%llu/%llu "
            "pose=%llu/%llu "
            "scene=%llu/%llu",
            int(task.received),
            int(task.fresh),
            task.age_ms,
            int(geometry_ok),
            int(versions_match),
            int(task.state.scene_valid),
            int(task.state.workpiece_pose_valid),
            int(task.state.supervision_valid),
            int(task.state.imu_stable),
            int(task.state.motion_fault_active),
            int(task.state.process_fault_active),
            static_cast<unsigned long long>(
                task.state.geometry_version),
            static_cast<unsigned long long>(
                execution_state_.geometry_version),
            static_cast<unsigned long long>(
                task.state.pose_version),
            static_cast<unsigned long long>(
                execution_state_.pose_version),
            static_cast<unsigned long long>(
                task.state.scene_version),
            static_cast<unsigned long long>(
                execution_state_.scene_version));
                
                
                RCLCPP_ERROR(
            get_logger(),
            "Execution supervision lost, stopping motion");
                 if (torch_controller_)
                {
                    // Request OFF without inventing a process fault.
                    torch_controller_->requestOutputOff();
                }

                if (move_group_)
                    move_group_->stop();
                
            }
            
               



               
            
        }

        });

        recover_cut_server_ =
            rclcpp_action::create_server<RecoverCut>(
                this,
                "recover_cut",
                std::bind(
                    &CutProfileServer::handleRecoveryGoal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
                std::bind(
                    &CutProfileServer::handleRecoveryCancel,
                    this,
                    std::placeholders::_1),
                std::bind(
                    &CutProfileServer::handleRecoveryAccepted,
                    this,
                    std::placeholders::_1));


        using Escape=hb_robot_interfaces::action::Escape;
        escape_server_=rclcpp_action::create_server<Escape>(this,"escape",
          [this](const auto&,auto goal) {
            if(!std::isfinite(goal->retreat_distance) || goal->retreat_distance<0 ||
               goal->retreat_distance>0.10) return rclcpp_action::GoalResponse::REJECT;
            bool idle=false;
            if(!escape_busy_.compare_exchange_strong(idle,true)) return rclcpp_action::GoalResponse::REJECT;
            requestCutStop(); return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [this](auto){requestCutStop();return rclcpp_action::CancelResponse::ACCEPT;},
          [this](auto h){if(escape_worker_.joinable()) escape_worker_.join();
            escape_worker_=std::thread([this,h]{escape(h);escape_busy_=false;});});

        TorchIOConfig torch_config_;
        torch_config_.ignition_enabled=process_enabled_;

        if(!has_parameter("torch_output_pin")){
            torch_config_.torch_output_pin = declare_parameter<int>("torch_output_pin",1);
        }
        torch_config_.torch_output_pin = get_parameter("torch_output_pin").as_int();
        
        
        if(!has_parameter("transfer_input_pin")){
            torch_config_.transfer_input_pin = declare_parameter<int>("transfer_input_pin",0);
        }
        torch_config_.transfer_input_pin = get_parameter("transfer_input_pin").as_int();

        if(!has_parameter("voltage_input_pin")){
            torch_config_.voltage_input_pin = declare_parameter<int>("voltage_input_pin",0);
        }
        torch_config_.voltage_input_pin = get_parameter("voltage_input_pin").as_int();


        torch_controller_ = std::make_unique<TorchController>(*this,torch_config_);

        torch_controller_->setFaultCallback([this](const std::string& reason){
            RCLCPP_ERROR(get_logger(), "TORCH PROCESS FAULT %s", reason.c_str());
            latchMotionStop("torch_fault");
            if(move_group_) move_group_->stop();
                
        });

        workpiece_reference_sub_ =
            create_subscription<WorkpieceReference>(
                "/workpiece_reference/accepted",
                rclcpp::QoS(1).reliable().transient_local(),
                [this](WorkpieceReference::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        reference_mutex_);

                    latest_reference_ = *msg;
                });
        approval_service_ = this->create_service<hb_robot_interfaces::srv::ApproveMotion>(
            "approve_cut_motion",
            std::bind(&CutProfileServer::handleApproval,this,
                std::placeholders::_1,std::placeholders::_2)
        );  

        clear_torch_fault_service_ = this->create_service<hb_robot_interfaces::srv::ClearTorchFault>(
            "/cut_profile/clear_torch_fault",
            std::bind(&CutProfileServer::handleTorchFault,this,
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


        initializeHeightInterface();
        // Read-only readiness; never resets faults, homes height or arms motion.
        recovery_ready_service_ = create_service<std_srvs::srv::Trigger>(
            "/cut_profile/recovery_ready",
            [this](std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr reply) {
                reply->success = recoveryHardwareReady();
                reply->message = reply->success ? "Torch OFF and height HOME confirmed" :
                    "Waiting for fresh torch OFF/no transfer/no fault and referenced height HOME";
            });

        auto estimate_qos = rclcpp::QoS(1).reliable().transient_local();

        

        // profile_estimate_sub_ = create_subscription<hb_robot_interfaces::msg::ProfileEstimate>("/profile_inspection/estimate",
        // estimate_qos, std::bind(&CutProfileServer::profileEstimateCallback,this,
        // std::placeholders::_1));

        auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
        visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cut_profile/cut_plan",marker_qos);
        collision_debug_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
        "/cut_profile/debug_collision", marker_qos);
        debug_robot_pub_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("/cut_profile/debug_robot",marker_qos);

    }

    void CutProfileServer::initialize(){

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),planning_group_);
        move_group_->setPlanningTime(planning_time_);
        initializeRobotStationaryMonitor();
        

        if (!has_parameter("cut_planning.mode")) {
            declare_parameter<std::string>(
                "cut_planning.mode", "monolithic");
        }

        const auto mode =
            get_parameter("cut_planning.mode").as_string();

        if (mode == "monolithic") {
            cut_planning_mode_ = CutPlanningMode::Monolithic;
        } else if (mode == "independent") {
            cut_planning_mode_ = CutPlanningMode::Independent;
        } else {
            throw std::invalid_argument(
                "cut_planning.mode must be 'monolithic' or 'independent'");
        }

        RCLCPP_INFO(
            get_logger(),
            "Cut planning mode: %s",
            mode.c_str());




        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(),"robot_description");
        if(!planning_scene_monitor_->getPlanningScene()){
            throw std::runtime_error("failed to create planning scene");
        }

        
        
        planning_scene_monitor_->startStateMonitor();
        if (const auto monitor = planning_scene_monitor_->getStateMonitor())
            monitor->enableCopyDynamics(true);
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor();
        // const bool scene_recieved  = 
        bool synchronized = false;
        for(int attempt = 0; attempt <5; attempt++){
            synchronized = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
            if(synchronized) break;
            RCLCPP_WARN(get_logger(), " couldnt get PSM on attempt %zu",attempt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        
        if(!synchronized){
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

        

        
        const auto count_parameter = [this](const std::string& name, int64_t fallback, bool allow_zero = false) {
            if (!has_parameter(name)) declare_parameter<int64_t>(name, fallback);
            const auto value = get_parameter(name).as_int();
            if (value < (allow_zero ? 0 : 1)) throw std::invalid_argument(name + " is out of range");
            return static_cast<std::size_t>(value);
        };
        const auto double_parameter = [this](const std::string& name, double fallback) {
            if (!has_parameter(name)) declare_parameter<double>(name, fallback);
            return get_parameter(name).as_double();
        };
        const auto bool_parameter = [this](const std::string& name, bool fallback) {
            if (!has_parameter(name)) declare_parameter<bool>(name, fallback);
            return get_parameter(name).as_bool();
        };
        motion::CutPlannerOptions geometry_options;
        if (!has_parameter("cut_geometry.direction_seed_states"))
            declare_parameter<std::vector<std::string>>("cut_geometry.direction_seed_states",
                {"cut_forward", "cut_down", "cut_up"});
        geometry_options.direction_seed_states =
            get_parameter("cut_geometry.direction_seed_states").as_string_array();
        geometry_options.direction_seed_pose_samples =
            count_parameter("cut_geometry.direction_seed_pose_samples", 12);
        geometry_options.max_pose_samples = count_parameter("cut_geometry.max_pose_samples", 48);
        geometry_options.feasible_samples = count_parameter("cut_geometry.feasible_samples", 4);
        geometry_options.refinement_iterations = count_parameter("cut_geometry.refinement_iterations", 8);
        geometry_options.ik_timeout_s = double_parameter("cut_geometry.ik_timeout_s", 0.05);
        geometry_options.waypoint_beam_width = count_parameter("cut_geometry.waypoint_beam_width", 3);
        geometry_options.explore_ik_branches = bool_parameter("cut_geometry.explore_ik_branches", true);
        geometry_options.ik_consistency_rad = double_parameter("cut_geometry.ik_consistency_rad", 2.0);
        geometry_options.repair_joint_step_rad = double_parameter("cut_geometry.repair_joint_step_rad", 0.03);
        geometry_options.repair_roll_step_rad = double_parameter("cut_geometry.repair_roll_step_rad", 0.15);
        cut_planner_ = std::make_unique<motion::CutPlanner>(robot_model, planning_group_, plasma_link_, geometry_options);
        motion::CutGraphPlanner::Options graph_options;
        graph_options.return_home_between_cuts = bool_parameter("cut_transit.return_home_between_cuts", true);
        graph_options.return_home_after_last_cut = bool_parameter("cut_transit.return_home_after_last_cut", true);
        graph_options.tcp_link = plasma_link_;
        graph_options.max_candidates_per_segment = count_parameter("cut_sampling.max_candidates", 32);
        graph_options.transit_weight = double_parameter("cut_cost.transit_weight", 1.0);
        graph_options.process_weight = double_parameter("cut_cost.process_weight", 1.0);
        graph_options.preferred_joint_travel_rad = double_parameter("cut_cost.preferred_joint_travel_rad", 1.57);
        graph_options.excess_travel_weight = double_parameter("cut_cost.excess_travel_weight", 2.0);
        graph_options.max_transit_joint_travel_rad = double_parameter("cut_cost.max_transit_joint_travel_rad", 0.0);

        motion::ProcessGeneratorOptions options;
        options.max_candidates_per_roll = count_parameter("cut_sampling.max_candidates_per_roll", 2, true);
        options.rank_transits_via_home = graph_options.return_home_between_cuts;
        options.roll_samples = count_parameter("cut_sampling.roll_samples", 12);
        options.seeds_per_roll = count_parameter("cut_sampling.seeds_per_roll", 16);
        options.global_seeds_per_roll = count_parameter("cut_sampling.global_seeds_per_roll", 2, true);
        options.local_seed_radius_rad = double_parameter("cut_sampling.local_seed_radius_rad", 0.35);
        options.local_ik_consistency_rad = double_parameter("cut_sampling.local_ik_consistency_rad", 0.75);
        options.retention_transit_weight = graph_options.transit_weight;
        options.retention_process_weight = graph_options.process_weight;
        options.preferred_joint_travel_rad = graph_options.preferred_joint_travel_rad;
        options.excess_travel_weight = graph_options.excess_travel_weight;
        options.max_translation_step_m = 0.003;
        options.max_joint_step_rad = 0.20;
        options.max_candidates = graph_options.max_candidates_per_segment;

        process_generator_ = std::make_unique<motion::CutProcessTrajectoryGenerator>
        (   move_group_->getRobotModel(),
            planning_group_,
            plasma_link_,
            options);

        

        cut_graph_planner_ = std::make_unique<motion::CutGraphPlanner>(
            move_group_->getRobotModel(),
            planning_group_,
            [this](const moveit::core::RobotState& from, 
            const moveit::core::RobotState& to
        ){
            return planToState(from,to);
        },

        [this](const motion::CutSegment& segment, 
            const motion::ProcessCandidate& candidate
        )->std::optional<motion::SegmentMotionPlan>{
            
            (void)segment;

            if(candidate.approach.empty() ||
            candidate.cut.empty()||
            candidate.retract.empty()){
                return std::nullopt;
            }
            
            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            move_group_->setMaxVelocityScalingFactor(0.2);
            move_group_->setMaxAccelerationScalingFactor(0.2);
            move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_->setPlannerId("LIN");

            motion::SegmentMotionPlan result;

            auto approach_ = makeLinPlan(*candidate.approach.front(),
            *candidate.approach.back(),cut_process_params_.approach_vel_m_s);
            if(!approach_){
                RCLCPP_ERROR(get_logger(),"failed on approach linear");
                return std::nullopt;
            }
            result.approach = *approach_;
            auto cut_ = makeLinPlan(getFinalState(result.approach),*candidate.cut.back(),cut_process_params_.cut_speed_m_s);
            if(!cut_){
                RCLCPP_ERROR(get_logger(),"failed on start linear");
                return std::nullopt;
            }            
            result.cut = *cut_;
            auto retract_ = makeLinPlan(getFinalState(result.cut),*candidate.retract.back(),cut_process_params_.retract_vel_m_s);
            if(!retract_){
                RCLCPP_ERROR(get_logger(),"failed on retract linear");
                return std::nullopt;
            }
            result.retract = *retract_;           
            return result;
        },
        graph_options
            );
        

        cut_planner_->setDebugVisCallback([this](const motion::CutSegment segment){
            publishCandidateVisualization(segment);
        }
        );

        cut_planner_->setCollisionDebugVisCallback([this](const motion::CollisionDebugEvent& event){
            publishCollisionDebug(event);
        });
        const auto trajectory_qos = rclcpp::QoS(1).transient_local().reliable();

        display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
        trajectory_qos
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

        RCLCPP_INFO(
            get_logger(),
            "CutProfile handleGoal entered: "
            "execute=%d plan_id=%s cut_id=%s",
            goal->execute,
            goal->plan_id.c_str(),
            goal->cut_id.c_str());

        

        const bool escape_busy = escape_busy_.load();
        const bool cut_id_empty = goal->cut_id.empty();

        const bool standoff_finite =
            std::isfinite(goal->standoff);
        const bool standoff_valid =
            standoff_finite && goal->standoff >= 0.0;

        const bool pos_tol_finite =
            std::isfinite(goal->pos_tol);
        const bool pos_tol_valid =
            pos_tol_finite && goal->pos_tol > 0.0;

        const bool ang_tol_finite =
            std::isfinite(goal->ang_tol);
        const bool ang_tol_valid =
            ang_tol_finite && goal->ang_tol > 0.0;

        const bool has_intervals =
            !goal->intervals.empty();

        const bool parent_missing =
            has_intervals && goal->parent_plan_id.empty();

        RCLCPP_INFO(
        get_logger(),
        "[CUT GOAL] RECEIVED "
        "execute=%d plan_id='%s' cut_id='%s' "
        "parent_plan_id='%s' "
        "busy=%d escape_busy=%d "
        "standoff=%.9f finite=%d valid=%d "
        "pos_tol=%.9f finite=%d valid=%d "
        "ang_tol=%.9f finite=%d valid=%d "
        "intervals=%zu parent_missing=%d "
        "skip_motion=%d process_enabled=%d "
        "approval_required=%d",
        static_cast<int>(goal->execute),
        goal->plan_id.c_str(),
        goal->cut_id.c_str(),
        goal->parent_plan_id.c_str(),
        static_cast<int>(busy_.load()),
        static_cast<int>(escape_busy),
        goal->standoff,
        static_cast<int>(standoff_finite),
        static_cast<int>(standoff_valid),
        goal->pos_tol,
        static_cast<int>(pos_tol_finite),
        static_cast<int>(pos_tol_valid),
        goal->ang_tol,
        static_cast<int>(ang_tol_finite),
        static_cast<int>(ang_tol_valid),
        goal->intervals.size(),
        static_cast<int>(parent_missing),
        static_cast<int>(skip_motion_),
        static_cast<int>(process_enabled_),
        static_cast<int>(require_plan_approval_));

            // --------------------------------------------------
            // Gate 1: basic goal validation
            // --------------------------------------------------

            if (escape_busy ||
                cut_id_empty ||
                !standoff_valid ||
                !pos_tol_valid ||
                !ang_tol_valid)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[CUT GOAL] REJECTED: basic validation "
                    "escape_busy=%d "
                    "cut_id_empty=%d "
                    "standoff_invalid=%d "
                    "pos_tol_invalid=%d "
                    "ang_tol_invalid=%d",
                    static_cast<int>(escape_busy),
                    static_cast<int>(cut_id_empty),
                    static_cast<int>(!standoff_valid),
                    static_cast<int>(!pos_tol_valid),
                    static_cast<int>(!ang_tol_valid));

                return rclcpp_action::GoalResponse::REJECT;
            }

            RCLCPP_INFO(
                get_logger(),
                "[CUT GOAL] PASS: basic validation");

                if (cut_planning_mode_ == CutPlanningMode::Independent &&
                goal->execute &&
                goal->plan_id.empty()) {

                RCLCPP_ERROR(
                    get_logger(),
                    "[CUT GOAL] REJECT: Independent mode requires a stored plan_id "
                    "for execution; batch planning cannot execute directly");

                return rclcpp_action::GoalResponse::REJECT;
        }

            // --------------------------------------------------
            // Gate 2: interval validation
            // --------------------------------------------------

            for (std::size_t i = 0;
                i < goal->intervals.size(); ++i)
            {
                const auto& interval = goal->intervals[i];

                if (!hb_robot::recovery::validInterval(interval))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "[CUT GOAL] REJECTED: "
                        "invalid interval index=%zu "
                        "total_intervals=%zu "
                        "execute=%d plan_id='%s'",
                        i,
                        goal->intervals.size(),
                        static_cast<int>(goal->execute),
                        goal->plan_id.c_str());

                    return rclcpp_action::GoalResponse::REJECT;
                }
            }

            RCLCPP_INFO(
                get_logger(),
                "[CUT GOAL] PASS: all %zu intervals valid",
                goal->intervals.size());

            // --------------------------------------------------
            // Gate 3: recut parent validation
            // --------------------------------------------------

            if (parent_missing)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[CUT GOAL] REJECTED: "
                    "recut intervals provided but "
                    "parent_plan_id is empty; "
                    "intervals=%zu cut_id='%s'",
                    goal->intervals.size(),
                    goal->cut_id.c_str());

                return rclcpp_action::GoalResponse::REJECT;
            }

            RCLCPP_INFO(
                get_logger(),
                "[CUT GOAL] PASS: parent-plan validation");

            // --------------------------------------------------
            // Gate 4: atomic execution ownership
            // --------------------------------------------------

            bool idle = false;

            if (!busy_.compare_exchange_strong(idle, true))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[CUT GOAL] REJECTED: SERVER BUSY "
                    "execute=%d plan_id='%s' cut_id='%s'",
                    static_cast<int>(goal->execute),
                    goal->plan_id.c_str(),
                    goal->cut_id.c_str());
                    

                return rclcpp_action::GoalResponse::REJECT;
            }


            RCLCPP_INFO(
                get_logger(),
                "[CUT GOAL] ACCEPTED "
                "execute=%d plan_id='%s' cut_id='%s'",
                static_cast<int>(goal->execute),
                goal->plan_id.c_str(),
                goal->cut_id.c_str());
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;


        }

        
        
       

    void CutProfileServer::requestCutStop(){

        latchMotionStop("requestCutStop");
        
        // Stop requests precede any feedback wait and never wait for a BT tick.
         if (torch_controller_)
    {
        // An idle stop or canceled planner is not a torch process fault.
        // Physical cut/height interruption retains the existing fault latch.
        if (process_enabled_ &&
            (motion_active_.load() || recovery_motion_active_.load() ||
             retreat_active_.load() || height_phase_.load() != HeightPhase::IDLE))
            torch_controller_->emergencyShutdown(
                "Cut stop requested");
        else
            torch_controller_->requestOutputOff();
    }
        if(move_group_) move_group_->stop();
        approval_cv_.notify_all();
    }

    rclcpp_action::CancelResponse CutProfileServer::handleCancel(
        const std::shared_ptr<GoalHandleCutProfile> goal_handle) {
        const auto goal = goal_handle->get_goal();
        const bool planning_only = !goal->execute && goal->plan_id.empty();
        RCLCPP_WARN(get_logger(),
            "[CUT CANCEL] planning_only=%d plan='%s'; %s",
            int(planning_only), goal->plan_id.c_str(),
            planning_only ? "waiting for planning checkpoint" : "requesting physical stop");
        // MoveGroup::stop() stops trajectory execution; it does not reliably
        // interrupt a blocking planning request. The worker checks cancellation.
        if (!planning_only) requestCutStop();
        approval_cv_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

   void CutProfileServer::releaseGoalAdmission()
        {
            busy_.store(false, std::memory_order_release);

            RCLCPP_INFO(
                get_logger(),
                "[CUT HANDOFF] Admission released");
        }
    void CutProfileServer::handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle){
      
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);

    // A previously completed action may still have a
    // joinable worker thread. Join it before replacing worker_.
    if (worker_.joinable())
        worker_.join();

    worker_ = std::thread([this, goal_handle]()
    {
        try
        {
            execute(goal_handle);
        }
        catch (const PlanningCanceled& e)
        {
            RCLCPP_WARN(get_logger(), "[CUT PLAN CANCELED] %s", e.what());
            auto result = std::make_shared<CutProfile::Result>();
            result->message = e.what();
            result->cut_id = goal_handle->get_goal()->cut_id;
            result->success = false;
            result->executed = false;
            result->interrupted = false;
            releaseGoalAdmission();
            if (goal_handle->is_canceling()) goal_handle->canceled(result);
            else goal_handle->abort(result); // A containment request canceled planning.
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[CUT WORKER] Exception: %s",
                e.what());

            const bool interrupted_motion = motion_active_.load();
            if (goal_handle->get_goal()->execute || interrupted_motion)
                requestCutStop();
            if (interrupted_motion) stopHeightAsync();
            // A cut interruption is not evidence that the workpiece moved.
            // TaskState's IMU supervision owns transform invalidation.

            auto result =
                std::make_shared<CutProfile::Result>();

            result->message = e.what();
            result->plan_id =
                goal_handle->get_goal()->plan_id;
            result->cut_id =
                goal_handle->get_goal()->cut_id;

            result->success = false;
            result->interrupted = motion_active_;

            motion_active_ = false;

            // Release admission BEFORE the client receives
            // the terminal result.
            releaseGoalAdmission();

            if (goal_handle->is_canceling())
                goal_handle->canceled(result);
            else
                goal_handle->abort(result);
        }

        // IMPORTANT:
        // No busy_=false here.
        // Every normal terminal path releases admission
        // before publishing its result.
    });
}






    std::optional<WorkpieceReference>
            CutProfileServer::latestWorkpieceReference() const
            {
                std::lock_guard<std::mutex> lock(
                    reference_mutex_);

                return latest_reference_;
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

    bool CutProfileServer::prepareCuttingScene(
            const planning_scene::PlanningScenePtr& scene)
        {
            if (!scene)
                return false;

            scene->getWorldNonConst()->removeObject(
                "inspection_workpiece");

            const auto world = scene->getWorld();

            return world &&
                !world->hasObject("inspection_workpiece") &&
                world->hasObject("wk_workpiece_collision");
        }

    void CutProfileServer::publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& stage){
                auto feedback = std::make_shared<CutProfile::Feedback>();
                feedback->stage = stage;
                goal_handle->publish_feedback(feedback);
            }








    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> CutProfileServer::makeLinPlan(
        const moveit::core::RobotState& start_state,
        const   moveit::core::RobotState& goal_state,
        double tcp_velocity_m_s
    ){
            checkPlanningCancellation();
            constexpr double max_pilz_tcp_vel_m_s =  1.0;
  
            double vel_scale = tcp_velocity_m_s / max_pilz_tcp_vel_m_s;
            if(!std::isfinite(vel_scale)|| vel_scale <=0.0 || vel_scale > 1.0){
                return std::nullopt;
            }
                move_group_->setStartState(start_state);
                move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
                move_group_->setPlannerId("LIN");
                move_group_->setMaxVelocityScalingFactor(vel_scale);
                move_group_->setMaxAccelerationScalingFactor(0.1);
                move_group_->setJointValueTarget(goal_state);
                // const Eigen::Isometry3d goal_tf = goal_state.getGlobalLinkTransform(plasma_link_);
                // geometry_msgs::msg::Pose goal_pose = tf2::toMsg(goal_tf);
                // move_group_->setPoseTarget(goal_pose, plasma_link_);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                const auto result = move_group_->plan(plan);
                checkPlanningCancellation();
                if(result!= moveit::core::MoveItErrorCode::SUCCESS){
                    return std::nullopt;
                }  
                return plan;
            };

    
        // std::optional<motion::SegmentMotionPlan> CutProfileServer::planSegmentPilzLinear(
        //     const motion::CutSegment& segment, const moveit::core::RobotState& actual_approach
        // ){
        //     motion::SegmentMotionPlan segment_motion_plan;
        //     move_group_->clearPoseTargets();
        //     move_group_->clearPathConstraints();
        //     move_group_->setMaxVelocityScalingFactor(0.2);
        //     move_group_->setMaxAccelerationScalingFactor(0.2);
        //     move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        //     move_group_->setPlannerId("LIN");
        //     // try the approach linear
        //     auto approach_ = makeLinPlan(actual_approach,*segment.start_state, cut_process_params_.approach_vel_m_s)
        //     if(!approach_){
        //         RCLCPP_ERROR(get_logger(),"failed on approach linear");
        //         return std::nullopt;
        //     }
        //     segment_motion_plan.approach = approach_.value();
        //     auto start_ = makeLinPlan(getFinalState(segment_motion_plan.approach),*segment.end_state,cut_process_params_.approach_vel_m_s);
        //     if(!start_){
        //         RCLCPP_ERROR(get_logger(),"failed on start linear");
        //         return std::nullopt;
        //     }            
        //     segment_motion_plan.cut = start_.value();
        //     auto retract_ = makeLinPlan(getFinalState(segment_motion_plan.cut),*segment.retract_state,cut_process_params_.cut_speed_m_s);
        //     if(!retract_){
        //         RCLCPP_ERROR(get_logger(),"failed on retract linear");
        //         return std::nullopt;
        //     }
        //     segment_motion_plan.retract = retract_.value();           
        //     return segment_motion_plan;
        // }


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

    
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
    CutProfileServer::planToState(
        const moveit::core::RobotState& start_state,
        const moveit::core::RobotState& target_state)
    {
        checkPlanningCancellation();
        motion::TransitPlannerOptions options;
        options.velocity_scaling = transit_velocity_scaling_;
        options.acceleration_scaling = transit_acceleration_scaling_;
        // Enable the new guided PTP planner.
        options.guided_ptp_enabled = true;
        options.guide_link = plasma_link_;

        // Use the actual SRDF named home configuration.
        const auto* jmg =
            move_group_->getRobotModel()->getJointModelGroup(
                planning_group_);

        if (!jmg)
            return std::nullopt;

        
        options.guided_joint_limits_rad = {
            1.25,  // shoulder_pan_joint
            1.25,  // shoulder_lift_joint
            1.25,  // elbow_joint
            1.25,  // wrist_1_joint
            1.25,  // wrist_2_joint
            1.25   // wrist_3_joint
        };
        moveit::core::RobotState home(
            move_group_->getRobotModel());

        home.setToDefaultValues();

        // Replace "home" if your SRDF uses another name.
        const auto& state_names = jmg->getDefaultStateNames();

            if (std::find(
                    state_names.begin(),
                    state_names.end(),
                    "inspection_home") == state_names.end())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[GUIDED PTP] No named 'inspection_home' state");

                return std::nullopt;
            }
        home.setToDefaultValues(jmg, "inspection_home");

        home.copyJointGroupPositions(
            jmg, options.home_joints);

        // On a staging-to-cut leg, guide relative to that named posture.
        // The graph separately plans inspection_home <-> staging, so this
        // never substitutes a fictional robot start configuration.
        const auto direction_names=get_parameter("cut_geometry.direction_seed_states").as_string_array();
        for (const auto& name : direction_names) {
            const auto posture=motion::namedCutPosture(start_state,jmg,name);
            if (!posture) continue;
            const auto matches=[&](const auto& state) {
                for(const auto& variable:jmg->getVariableNames())
                    if(std::abs(state.getVariablePosition(variable)-posture->getVariablePosition(variable))>1e-6)
                        return false;
                return true;
            };
            if(matches(start_state) || matches(target_state)) {
                posture->copyJointGroupPositions(jmg,options.home_joints);
                RCLCPP_INFO(get_logger(),"[CUT SEED TRANSIT] guided reference=%s",name.c_str());
                break;
            }
        }

        // Use a consistent snapshot of the planning scene.
        {
            planning_scene_monitor::LockedPlanningSceneRO locked(
                planning_scene_monitor_);

            if (!locked)
                return std::nullopt;

            planning_scene::PlanningScenePtr snapshot =
                locked->diff();

            snapshot->decoupleParent();
            if (!prepareCuttingScene(snapshot)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Transit cutting scene normalization failed");
                return std::nullopt;
            }

            options.scene = snapshot;
        }

        // Initial sampling settings.
        options.guided_lift_m = 0.20;
        options.guided_lateral_m = 0.08;

        options.guided_position_tolerance_m = 0.025;
        options.guided_orientation_tolerance_rad = 0.35;

        options.guided_joint_consistency_rad = 1.0;

        options.guided_max_waypoints = 5;
        options.guided_max_ik_calls = 50;
        options.guided_max_ptp_calls = 12;
        auto planned = motion::planTransit(
            *move_group_,
            get_logger(),
            start_state,
            target_state,
            options);
        checkPlanningCancellation();
        return planned;
        // return motion::planTransit(*move_group_, get_logger(), start_state, target_state);
    }


    // std::optional<moveit::planning_interface::MoveGroupInterface::Plan> CutProfileServer::planToState(
    //         const moveit::core::RobotState& start_state,
    //         const moveit::core::RobotState& target_state
    // ){

    //     using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

    //     struct Metrics {
    //     double total = 0.0;
    //     double max_joint = 0.0;
    //     double max_step = 0.0;
    //     double cost = std::numeric_limits<double>::infinity();
    // };



    //     move_group_->clearPoseTargets();
    //     move_group_->clearPathConstraints();
    //     move_group_->setPlanningTime(planning_time_);
    //     move_group_->setMaxVelocityScalingFactor(0.1);
    //     move_group_->setMaxAccelerationScalingFactor(0.1);
    //     move_group_->setStartState(start_state);
    //     move_group_->setJointValueTarget(target_state);


    //     move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    //     move_group_->setPlannerId("PTP");

    //     Plan ptp_plan;
    //     if(move_group_->plan(ptp_plan)==moveit::core::MoveItErrorCode::SUCCESS){
    //         RCLCPP_INFO(get_logger(),"ptp worked with pilz");
    //         return ptp_plan;
    //     }
    //     RCLCPP_INFO(get_logger(),"pilz ptp failed, try omply");

    //     move_group_->clearPoseTargets();
    //     move_group_->clearPathConstraints();
    //     move_group_->setPlanningTime(planning_time_);
    //     move_group_->setMaxVelocityScalingFactor(0.1);
    //     move_group_->setMaxAccelerationScalingFactor(0.1);
    //     move_group_->setStartState(start_state);
    //     move_group_->setJointValueTarget(target_state);

    //     move_group_->setPlanningPipelineId("ompl");
    //     move_group_->setPlannerId("RRTConnectkConfigDefault");
    //     // move_group_->setPlannerId("RRTstarkConfigDefault");
    //     move_group_->setPlanningTime(5.0);

        
    //     moveit::planning_interface::MoveGroupInterface::Plan plan;
    //     if(!static_cast<bool>(move_group_->plan(plan))){
    //         RCLCPP_ERROR(get_logger(), "failed to plan joint motion");
    //         return std::nullopt;
    //     }
        
    //     return plan;
    // }

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


    CutProfileServer::MotionOutcome
CutProfileServer::waitForPlanApproval(
    const std::shared_ptr<GoalHandleCutProfile> goal_handle)
{
    using namespace std::chrono_literals;
    if (!require_plan_approval_)
        return MotionOutcome::SUCCESS;

    RCLCPP_INFO(
        get_logger(),
        "Waiting for cutting plan approval");

    publishFeedback(
        goal_handle, "awaiting plan approval");

   {
        std::lock_guard<std::mutex> lock(approval_mutex_);

        motion_approved_ = false;
        motion_rejected_ = false;

        approval_operation_ =
            "cut/" + goal_handle->get_goal()->plan_id;

        approval_pending_ = true;
    }
    auto closeApprovalWindow = [&]()
    {
        std::lock_guard<std::mutex> lock(approval_mutex_);

        approval_pending_ = false;
        approval_operation_.clear();
    };

    while (rclcpp::ok())
    {
        // Explicit cancellation and stop requests remain terminal.
        if (goal_handle->is_canceling())
        {
            closeApprovalWindow();
            return MotionOutcome::CANCELED;
        }

        if (cut_stop_requested_.load() ||
            escape_busy_.load())
        {
            closeApprovalWindow();
            return MotionOutcome::FAILED;
        }

        bool approved = false;
        bool rejected = false;

        {
            std::lock_guard<std::mutex> lock(approval_mutex_);

            approved = motion_approved_;
            rejected = motion_rejected_;
        }

        if (rejected)
        {
            closeApprovalWindow();
            return MotionOutcome::REJECTED;
        }

        // Do not allow a temporarily stale TaskState to
        // terminate the approval window.
        const auto task = taskStateFreshness();

        if (!task.fresh)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "[CUT APPROVAL] Waiting for fresh TaskState "
                "age_ms=%.1f",
                task.age_ms);

            publishFeedback(
                goal_handle,
                "waiting for fresh TaskState");
        }
        else
        {
            // A fresh TaskState must still authorize execution.
            // This does not replace the final execution preflight.
            // const bool permitted =
            //     taskPermitsExecution();
            const bool permitted =
    hb_robot::recovery::geometryPermitted(task.state);

            if (!permitted)
            {   
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "[CUT APPROVAL] Authorization unavailable: "
                    "age_ms=%.1f supervision=%d imu=%d "
                    "scene=%d pose=%d motion_fault=%d "
                    "process_fault=%d status='%s'",
                    task.age_ms,
                    int(task.state.supervision_valid),
                    int(task.state.imu_stable),
                    int(task.state.scene_valid),
                    int(task.state.workpiece_pose_valid),
                    int(task.state.motion_fault_active),
                    int(task.state.process_fault_active),
                    task.state.status_message.c_str());

                if (task.state.motion_fault_active ||
                    task.state.process_fault_active ||
                    !task.state.workpiece_pose_valid ||
                    !task.state.scene_valid)
                {
                closeApprovalWindow();

                RCLCPP_ERROR(
                    get_logger(),
                    "[CUT APPROVAL] Fresh TaskState "
                    "does not permit execution");

                return MotionOutcome::FAILED;
                }
            }

            else if (approved)
            {
                // Recheck authorization immediately before returning.
                TaskState final_state;

                if (!taskPermitsExecution(&final_state))
                    continue;
                // Final approval-stage authorization check.
                // executeStored() must still perform its own
                // live scene, start-state and collision checks.
                // const bool final_permitted =
                //     taskPermitsExecution();

                // if (!final_permitted)
                // {
                //     // Freshness could have changed between checks.
                //     // Do not execute, but don't discard approval
                //     // solely because the state became stale.
                //     continue;
                // }

                closeApprovalWindow();

                RCLCPP_INFO(
                    get_logger(),
                    "[CUT APPROVAL] Approved and authorized");

                return MotionOutcome::SUCCESS;
            }
        }

        std::unique_lock<std::mutex> lock(approval_mutex_);

        // approval_cv_.wait_for(
        //     lock,
        //     100ms,
        //     [&]()
        //     {
        //         return motion_approved_ ||
        //                motion_rejected_ ||
        //                goal_handle->is_canceling() ||
        //                cut_stop_requested_.load() ||
        //                escape_busy_.load() ||
        //                !rclcpp::ok();
        //     });
        approval_cv_.wait_for(lock, 100ms);
    }

    closeApprovalWindow();
    return MotionOutcome::FAILED;
}






    CutProfileServer::MotionOutcome CutProfileServer::executeMotion(
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::shared_ptr<GoalHandleCutProfile> goal_handle,
        const std::string& description
    ){

        if(goal_handle->is_canceling()) return MotionOutcome::CANCELED;
        if(cut_stop_requested_ || !taskPermitsExecution()) return MotionOutcome::FAILED;
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
    if(cut_stop_requested_ || !taskPermitsExecution()) return MotionOutcome::FAILED;
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
           
    ////////////////////////////////////////////////////////////////
    bool CutProfileServer::setFlapClosed(bool closed) {
        using namespace std::chrono_literals;
        const char* command = closed ? "close" : "open";
        try {
            if (!flap_client_->wait_for_service(1s)) {
                RCLCPP_ERROR(get_logger(), "Flap service unavailable during %s", command);
                return false;
            }
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = closed;
            auto pending = flap_client_->async_send_request(request);
            if (pending.wait_for(2s) != std::future_status::ready) {
                flap_client_->remove_pending_request(pending);
                RCLCPP_ERROR(get_logger(), "Flap %s acknowledgement timed out", command);
                return false;
            }
            const auto response = pending.get();
            if (!response->success) {
                RCLCPP_ERROR(get_logger(), "Flap %s failed: %s", command, response->message.c_str());
                return false;
            }
            std::this_thread::sleep_for(flap_travel_time_);
            return true;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Flap %s failed: %s", command, error.what());
            return false;
        }
    }

void CutProfileServer::initializeHeightInterface()
{
    auto integer = [this](const char* key, int fallback) {
        if (!has_parameter(key)) declare_parameter<int>(key, fallback);
        return static_cast<int>(get_parameter(key).as_int());
    };
    height_status_timeout_ms_ = integer("height.status_timeout_ms", 500);
    height_prepare_timeout_ms_ = integer("height.prepare_timeout_ms", 5000);
    height_search_timeout_ms_ = integer("height.search_timeout_ms", 8000);
    height_home_timeout_ms_ = integer("height.home_timeout_ms", 20000);
    height_pierce_dwell_ms_ = integer("height.pierce_dwell_ms", 0);
    if (!has_parameter("height.enable_voltage_tracking"))
        declare_parameter<bool>("height.enable_voltage_tracking", false);
    height_use_tracking_ = get_parameter("height.enable_voltage_tracking").as_bool();
    if (height_status_timeout_ms_ < 100 || height_prepare_timeout_ms_ <= 0 ||
        height_search_timeout_ms_ <= 0 || height_home_timeout_ms_ <= 0 ||
        height_pierce_dwell_ms_ < 0)
        throw std::invalid_argument("Invalid height timing parameters");

    height_callbacks_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto trigger = [this](const char* topic) {
        return create_client<std_srvs::srv::Trigger>(
            topic, rmw_qos_profile_services_default, height_callbacks_);
    };
    height_prepare_client_ = trigger("/torch_height/prepare");
    height_search_client_ = trigger("/torch_height/arc_search");
    height_home_client_ = trigger("/torch_height/home");
    height_stop_client_ = trigger("/torch_height/stop");
    height_reset_client_ = trigger("/torch_height/reset_fault");
    // A waiting operator service must not block TaskState/timers on the default
    // callback group. Height replies remain in the existing reentrant group.
    height_recovery_callbacks_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    height_reset_home_service_ = create_service<std_srvs::srv::Trigger>(
        "/cut_profile/reset_height_and_home",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            resetHeightAndHome(*response);
        }, rmw_qos_profile_services_default, height_recovery_callbacks_);
    height_tracking_client_ = create_client<std_srvs::srv::SetBool>(
        "/torch_height/set_tracking_enabled", rmw_qos_profile_services_default,
        height_callbacks_);
    rclcpp::SubscriptionOptions options;
    height_status_callbacks_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

options.callback_group = height_status_callbacks_;

height_full_status_sub_ =
    create_subscription<std_msgs::msg::String>(
        "/torch_height/status_full",
        rclcpp::QoS(1),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
            const auto callback_entered = HeightClock::now();
            {
                std::lock_guard<std::mutex> lock(height_mutex_);
                if (height_last_callback_ != HeightClock::time_point{}) {
                    height_callback_gap_ms_ =
                        std::chrono::duration<double, std::milli>(
                            callback_entered - height_last_callback_).count();
                    height_max_callback_gap_ms_ = std::max(
                        height_max_callback_gap_ms_, height_callback_gap_ms_);
                }
                height_last_callback_ = callback_entered;
                ++height_callback_count_;
            }

            try {
                std::istringstream input(msg->data);
                std::vector<std::string> f;
                std::string part;

                while (std::getline(input, part, ','))
                    f.push_back(part);

                if (f.size() != 12)
                    throw std::invalid_argument("Height field count");

                auto number = [&f](
                    size_t i, int64_t low, int64_t high) -> int64_t {
                    size_t end = 0;
                    const auto value = std::stoll(f[i], &end);

                    if (end != f[i].size() ||
                        value < low || value > high) {
                        throw std::invalid_argument("Height field");
                    }

                    return value;
                };

                HeightStatus h;
                h.state = f[0];
                h.position = number(1, INT32_MIN, INT32_MAX);
                h.switch_active = number(2, 0, 1);
                h.fault = number(3, 0, 255);
                h.referenced = number(4, 0, 1);
                h.session = number(5, 0, UINT32_MAX);
                h.active_id = number(6, 0, UINT32_MAX);
                h.done_id = number(7, 0, UINT32_MAX);
                h.done_result = number(8, 0, 2);
                h.calibration = number(9, 0, 1);
                h.configured = number(10, 0, 1);
                h.tracking = number(11, 0, 1);
                h.received = callback_entered;

                double gap_ms = 0.0;
                double max_gap_ms = 0.0;
                uint64_t count = 0;

                {
                    std::lock_guard<std::mutex> lock(height_mutex_);

                    if (height_status_.received !=
                        HeightClock::time_point{}) {
                        gap_ms =
                            std::chrono::duration<double, std::milli>(
                                callback_entered -
                                height_status_.received).count();
                    }

                    if (gap_ms > height_status_max_gap_ms_)
                        height_status_max_gap_ms_ = gap_ms;

                    count = ++height_status_rx_count_;
                    max_gap_ms = height_status_max_gap_ms_;
                    height_status_ = h;
                }

                height_cv_.notify_all();

                // // Diagnostic threshold only; does not alter permission.
                // if (gap_ms > 750.0) {
                //     RCLCPP_WARN(
                //         get_logger(),
                //         "[HEIGHT RX GAP] count=%llu gap_ms=%.1f "
                //         "max_gap_ms=%.1f state=%s session=%u",
                //         static_cast<unsigned long long>(count),
                //         gap_ms,
                //         max_gap_ms,
                //         h.state.c_str(),
                //         h.session);
                // }

            } catch (const std::exception& e) {
                // Malformed status must not refresh the cached timestamp.
                {
                    std::lock_guard<std::mutex> lock(height_mutex_);
                    ++height_rejected_count_;
                }
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "[HEIGHT RX INVALID] %s",
                    e.what());
            }
        },
        options);
    RCLCPP_INFO(get_logger(),
        "[HEIGHT RX CONFIG] topic=/torch_height/status_full "
        "executor=main_owned_dedicated watchdog_ms=%d "
        "freshness=steady_callback_entry producer_timestamp=unavailable",
        height_status_timeout_ms_);
}

CutProfileServer::HeightStatus CutProfileServer::heightSnapshot() const
{
    std::lock_guard<std::mutex> lock(height_mutex_);
    return height_status_;
}

bool CutProfileServer::heightValid(
    const HeightStatus& h, HeightClock::time_point at) const
{
    return heightCommunicationValid(h, at) && h.fault == 0 &&
        h.state != "FAULT" && h.state != "PAUSED";
}

bool CutProfileServer::heightAtHomeAndReady(bool log_status) const
{
    HeightStatus h;
    HeightClock::time_point checked_at;
    {
        std::lock_guard<std::mutex> lock(height_mutex_);
        h = height_status_;
        checked_at = HeightClock::now();
    }
    const bool ready = heightValid(h, checked_at) && h.state == "HOME" &&
        h.position == 0 && h.switch_active && h.referenced && !h.tracking;
    const double age_ms = h.received == HeightClock::time_point{}
        ? -1.0 : std::chrono::duration<double, std::milli>(
            checked_at - h.received).count();
    // Evaluate before logging: console delay must not age an old snapshot
    // while the dedicated receiver continues to accept newer status.
    if (log_status) RCLCPP_INFO(get_logger(),
        "[HEIGHT PREFLIGHT] state=%s position=%d switch=%d fault=%d "
        "referenced=%d session=%u configured=%d age_ms=%.1f limit_ms=%d ready=%d",
        h.state.c_str(), static_cast<int>(h.position), int(h.switch_active),
        h.fault, int(h.referenced), h.session, int(h.configured),
        age_ms, height_status_timeout_ms_, int(ready));
    return ready;
}

bool CutProfileServer::recoveryHardwareReady() const
{
    if (!torch_controller_ || !torch_controller_->hasFreshIO() ||
        torch_controller_->hasFault() || !heightAtHomeAndReady(false)) return false;
    const auto t = torch_controller_->snapshot();
    return t.output_on == std::optional<bool>(false) &&
           t.transfer_detected == std::optional<bool>(false);
}

bool CutProfileServer::requestHeightCommand(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
    const std::function<bool()>& canceled, uint32_t& command_id)
{
    command_id = 0;
    if (!rclcpp::ok() || (canceled && canceled()) ||
        !client || !client->wait_for_service(500ms)) return false;
    auto pending = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline = HeightClock::now() + 3s;
    while (rclcpp::ok() && HeightClock::now() < deadline) {
        if (canceled && canceled()) break;
        if (pending.wait_for(20ms) != std::future_status::ready) continue;
        try {
            const auto reply = pending.get();
            if (!reply->success) {
                RCLCPP_ERROR(get_logger(), "Height rejected: %s", reply->message.c_str());
                return false;
            }
            std::istringstream input(reply->message);
            std::string token;
            input >> token;
            if (token.rfind("id=", 0) != 0) return false;
            const auto text = token.substr(3);
            size_t end = 0;
            const auto id = std::stoull(text, &end);
            if (end != text.size() || id == 0 || id > UINT32_MAX) return false;
            command_id = static_cast<uint32_t>(id);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Height ACK error: %s", e.what());
            return false;
        }
    }
    client->remove_pending_request(pending);
    // Removing a ROS request does NOT cancel a command already accepted downstream.
    stopHeightAsync();
    return false;
}

bool CutProfileServer::waitHeightCompletion(
    uint32_t session, uint32_t command_id, const std::string& expected_state,
    std::chrono::milliseconds timeout, const std::function<bool()>& canceled)
{
    const auto deadline = HeightClock::now() + timeout;
    while (rclcpp::ok() && HeightClock::now() < deadline) {
        if (canceled && canceled()) return false;
        std::unique_lock<std::mutex> lock(height_mutex_);
        const auto& h = height_status_;
        if (!heightValid(h) || h.session != session) return false;
        if (h.done_id == command_id) {
            if (h.done_result != 1 || h.state != expected_state || !h.referenced)
                return false;
            if (expected_state == "HOME")
                return h.position == 0 && h.switch_active && !h.tracking;
            return h.position < 0 && !h.switch_active && !h.tracking;
        }
        if (h.done_id > command_id) return false; // Another command superseded ours.
        height_cv_.wait_for(lock, 20ms);
    }
    return false;
}

bool CutProfileServer::prepareHeightAndWaitReady(
    const std::function<bool()>& canceled)
{
    if (!heightAtHomeAndReady()) return false;
    const auto session = heightSnapshot().session;
    uint32_t id = 0;
    return requestHeightCommand(height_prepare_client_, canceled, id) &&
        waitHeightCompletion(session, id, "READY",
            std::chrono::milliseconds(height_prepare_timeout_ms_), canceled);
}

bool CutProfileServer::searchHeightAndWaitForArc(
    const std::function<bool()>& canceled)
{
    const auto h = heightSnapshot();

    if (!heightValid(h) ||
        !h.referenced ||
        h.tracking ||
        h.state != "READY") {
        RCLCPP_ERROR(
            get_logger(),
            "[HEIGHT SEARCH PREFLIGHT] "
            "state=%s fault=%d referenced=%d tracking=%d",
            h.state.c_str(),
            int(h.fault),
            int(h.referenced),
            int(h.tracking));
        return false;
    }

    uint32_t id = 0;

    if (!requestHeightCommand(
            height_search_client_, canceled, id)) {
        return false;
    }

    if (!waitHeightCompletion(
            h.session,
            id,
            "HOLD",
            std::chrono::milliseconds(height_search_timeout_ms_),
            canceled)) {
        return false;
    }

    // Preserve the TorchController IGNITING -> ESTABLISHED transition.
    return torch_controller_ &&
        torch_controller_->waitForArcTransfer(250ms, canceled) ==
            TorchController::WaitResult::CONFIRMED;
}


bool CutProfileServer::setHeightTrackingAndWait(bool enabled)
{
    try {
    if (!height_tracking_client_ ||
        !height_tracking_client_->wait_for_service(500ms)) return false;
    const auto before = heightSnapshot();
    if (!heightValid(before)) return false;
    if (enabled && (!before.referenced || before.state != "HOLD" ||
        !torch_controller_ || !torch_controller_->hasFreshIO() ||
        torch_controller_->hasFault())) return false;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto pending = height_tracking_client_->async_send_request(request);
    if (pending.wait_for(1500ms) != std::future_status::ready) {
        height_tracking_client_->remove_pending_request(pending);
        return false;
    }
    if (!pending.get()->success) return false;
    const auto acknowledged = HeightClock::now();
    const auto deadline = acknowledged + 1500ms;
    while (rclcpp::ok() && HeightClock::now() < deadline) {
        std::unique_lock<std::mutex> lock(height_mutex_);
        const auto& h = height_status_;
        if (!heightValid(h) || h.session != before.session) return false;
        // Require a later actual Nano snapshot, not the service's local flag.
        if (h.received > acknowledged && h.tracking == enabled &&
            (enabled ? (h.state == "HOLD" || h.state == "TRACK") : h.state != "TRACK"))
            return true;
        height_cv_.wait_for(lock, 20ms);
    }
    return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Height tracking error: %s", e.what());
        return false;
    }
}

bool CutProfileServer::enableHeightTracking()
{
    if (!height_use_tracking_) return true;
    const bool ok = setHeightTrackingAndWait(true);
    height_tracking_expected_.store(ok);
    return ok;
}

bool CutProfileServer::disableHeightTracking()
{
    height_tracking_expected_.store(false);
    return setHeightTrackingAndWait(false);
}

bool CutProfileServer::returnHeightHomeAndWait(
    const std::function<bool()>& canceled)
{
    if ((canceled && canceled()) ||
        !torch_controller_ ||
        !torch_controller_->hasFreshIO() ||
        torch_controller_->hasFault()) {
        return false;
    }

    const auto torch = torch_controller_->snapshot();

    if (torch.command_on ||
        torch.output_on != std::optional<bool>(false) ||
        torch.transfer_detected != std::optional<bool>(false)) {
        RCLCPP_ERROR(
            get_logger(),
            "HOME denied: torch OFF is not confirmed");
        return false;
    }

    const auto h = heightSnapshot();

    // A cached HOME state cannot authorize return without fresh status.
    if (!heightValid(h))
        return false;

    // Already safely retracted and tracking is off.
    // No service round trip is needed.
    if (h.state == "HOME" &&
        h.position == 0 &&
        h.switch_active &&
        h.referenced &&
        !h.tracking) {
        height_tracking_expected_.store(false);
        return true;
    }

    // Otherwise disable tracking before any HOME motion.
    if (!disableHeightTracking())
        return false;

        const auto after_disable = heightSnapshot();

    if (!heightValid(after_disable) ||
        after_disable.tracking ||
        after_disable.session != h.session) {
        RCLCPP_ERROR(
            get_logger(),
            "HOME denied: height status invalid or session changed");
        return false;
    }

    const bool permitted_state =
        after_disable.state == "UNKNOWN" ||
        after_disable.state == "HOME" ||
        after_disable.state == "READY" ||
        after_disable.state == "HOLD";

    if (!permitted_state) {
        RCLCPP_ERROR(
            get_logger(),
            "HOME denied from state %s",
            after_disable.state.c_str());
        return false;
    }

    uint32_t command_id = 0;

    if (!requestHeightCommand(
            height_home_client_,
            canceled,
            command_id)) {
        return false;
    }

    const bool completed = waitHeightCompletion(
        after_disable.session,
        command_id,
        "HOME",
        std::chrono::milliseconds(height_home_timeout_ms_),
        canceled);

    if (!completed) {
        // A completion timeout/cancellation must not leave a
        // previously accepted seek running without this caller.
        stopHeightAsync();
        return false;
    }

    return heightAtHomeAndReady();
}

bool CutProfileServer::rearmHeightStop()
{
    std::lock_guard<std::mutex> lock(height_mutex_);
    if (height_stop_pending_) return false;
    height_stop_sent_.store(false);
    height_last_stop_request_ = HeightClock::time_point{};
    return true;
}

void CutProfileServer::resetHeightAndHome(std_srvs::srv::Trigger::Response& response)
{
    response.success = false;
    bool idle = false;
    if (!busy_.compare_exchange_strong(idle, true)) {
        response.message = "Cut/planning/recovery operation active";
        return;
    }
    struct ReleaseAdmission {
        std::atomic<bool>& busy;
        ~ReleaseAdmission() { busy.store(false); }
    } release{busy_};
    std::unique_lock<std::mutex> execution_lock(execution_mutex_, std::try_to_lock);
    if (!execution_lock.owns_lock() || motion_active_.load() || retreat_active_.load() ||
        recovery_motion_active_.load() || escape_busy_.load()) {
        response.message = "Motion/containment worker has not finished";
        return;
    }

    const auto generation = stop_generation_.load();
    const auto before = heightSnapshot();
    // Bench admission does not depend on a workpiece transform or process latch.
    // A locally latched torch fault can be cleared once OFF is confirmed below.
    const auto bench_invalid = [this, generation, session = before.session]() {
        const auto task = taskStateFreshness();
        const auto h = heightSnapshot();
        if (!rclcpp::ok() || escape_busy_.load() || stop_generation_.load() != generation ||
            !task.fresh || !task.state.supervision_valid || !task.state.imu_stable ||
            task.state.motion_fault_active || !heightCommunicationValid(h) ||
            h.session != session || !torch_controller_ || !torch_controller_->hasFreshIO()) return true;
        const auto torch = torch_controller_->snapshot();
        return torch.command_on || torch.output_on != std::optional<bool>(false) ||
            torch.transfer_detected != std::optional<bool>(false);
    };
    const auto canceled = [&]() {
        return bench_invalid() || torch_controller_->hasFault();
    };
    bool command_attempted = false;
    try {
        if (bench_invalid()) throw std::runtime_error("Fresh stable supervision, height session and confirmed torch OFF required");
        if (before.active_id || before.tracking ||
            (before.state != "UNKNOWN" && before.state != "HOME" && before.state != "READY" &&
             before.state != "HOLD" && before.state != "FAULT" && before.state != "PAUSED"))
            throw std::runtime_error("Height must be stopped before RESET_FAULT");
        const auto* jmg = move_group_ ? move_group_->getRobotModel()->getJointModelGroup(planning_group_) : nullptr;
        if (!move_group_ || !recoveryRobotStationary(jmg) || bench_invalid())
            throw std::runtime_error("Robot not confirmed stationary or supervision changed");
        if (!rearmHeightStop()) throw std::runtime_error("Previous height STOP response still pending; retry after it finishes");

        if (torch_controller_->hasFault()) {
            const auto reason = torch_controller_->faultReason();
            if (!torch_controller_->clearFault())
                throw std::runtime_error("Torch OFF fault clearance failed");
            RCLCPP_WARN(get_logger(),
                "[HEIGHT RECOVER] Local torch fault cleared for explicit homing: %s; TaskState cut fault retained",
                reason.c_str());
        }
        if (canceled()) throw std::runtime_error("Supervision changed before height reset");

        // This operation owns no cutting phase. UNKNOWN/unreferenced is the
        // expected reset state and must not trigger the cut-phase supervisor.
        height_phase_.store(HeightPhase::IDLE);
        height_tracking_expected_.store(false);
        uint32_t id = 0;
        command_attempted = true;
        RCLCPP_INFO(get_logger(), "[HEIGHT RECOVER] RESET_FAULT session=%u", before.session);
        if (!requestHeightCommand(height_reset_client_, canceled, id))
            throw std::runtime_error("RESET_FAULT request failed");
        const auto ack = HeightClock::now();
        const auto deadline = ack + 2s;
        bool confirmed = false;
        while (rclcpp::ok() && HeightClock::now() < deadline) {
            if (canceled()) throw std::runtime_error("RESET_FAULT session/supervision lost or stop requested");
            const auto h = heightSnapshot();
            if (h.received > ack && h.session == before.session && h.state == "UNKNOWN" &&
                !h.fault && !h.referenced && !h.active_id && !h.tracking) {
                confirmed = true;
                break;
            }
            std::this_thread::sleep_for(20ms);
        }
        if (!confirmed) throw std::runtime_error("RESET_FAULT status not confirmed");
        RCLCPP_INFO(get_logger(), "[HEIGHT RECOVER] RESET_FAULT confirmed; requesting HOME");
        if (!requestHeightCommand(height_home_client_, canceled, id) ||
            !waitHeightCompletion(before.session, id, "HOME",
                std::chrono::milliseconds(height_home_timeout_ms_), canceled) || canceled())
            throw std::runtime_error("HOME completion not confirmed");
        response.success = true;
        response.message = "Height reset and HOME confirmed; TaskState cut fault retained, no cut resumed";
        RCLCPP_INFO(get_logger(), "[HEIGHT RECOVER] HOME confirmed session=%u command=%u", before.session, id);
    } catch (const std::exception& error) {
        if (command_attempted) stopHeightAsync();
        response.message = error.what();
        RCLCPP_ERROR(get_logger(), "[HEIGHT RECOVER FAILED] %s", error.what());
    }
}

void CutProfileServer::stopHeightAsync()
{
    height_tracking_expected_.store(false);
    if (!height_stop_client_ || !height_stop_client_->service_is_ready()) return;
    HeightStatus h;
    {
        std::lock_guard<std::mutex> lock(height_mutex_);
        const auto now = HeightClock::now();
        if (height_stop_sent_.load() || height_stop_pending_) return;
        // First STOP is immediate. A failed request retries at most once per
        // second, rather than on every 20 ms supervision tick.
        if (height_last_stop_request_ != HeightClock::time_point{} &&
            now - height_last_stop_request_ < 1s) return;
        height_stop_sent_.store(true);
        height_stop_pending_ = true;
        height_last_stop_request_ = now;
        h = height_status_;
    }
    RCLCPP_ERROR(get_logger(),
        "[HEIGHT STOP REQUEST] phase=%d state=%s fault=%d referenced=%d session=%u cut_stop=%d",
        int(height_phase_.load()), h.state.c_str(), h.fault, int(h.referenced), h.session,
        int(cut_stop_requested_.load()));
    const auto completed = [this](bool success) {
        std::lock_guard<std::mutex> lock(height_mutex_);
        if (!success) height_stop_sent_.store(false);
        height_stop_pending_ = false;
    };
    try {
        height_stop_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this, completed](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    const auto reply = future.get();
                    completed(reply && reply->success);
                    if (!reply || !reply->success)
                        RCLCPP_ERROR(get_logger(), "[HEIGHT STOP REJECTED] %s; retry interval=1000ms",
                            reply ? reply->message.c_str() : "empty response");
                } catch (const std::exception& error) {
                    completed(false);
                    RCLCPP_ERROR(get_logger(), "[HEIGHT STOP RESPONSE ERROR] %s", error.what());
                }
            });
    } catch (const std::exception& error) {
        completed(false);
        RCLCPP_ERROR(get_logger(), "[HEIGHT STOP SEND ERROR] %s", error.what());
    }
}


bool CutProfileServer::heightCommunicationValid(
    const HeightStatus& h, HeightClock::time_point at) const
{
    return h.received != HeightClock::time_point{} &&
        at >= h.received && at - h.received <=
            std::chrono::milliseconds(
                height_status_timeout_ms_) &&
        h.configured &&
        !h.calibration &&
        h.session != 0 &&
        h.state != "OFFLINE";
}







void CutProfileServer::superviseHeight()
{
    const auto phase = height_phase_.load();
    if (phase == HeightPhase::IDLE)
        return;

    HeightStatus h;
    HeightClock::time_point checked_at, last_callback;
    uint64_t rx_count, callbacks, rejected;
    double max_gap_ms, callback_gap_ms, max_callback_gap_ms;
    {
        // State, counters and the check time describe the same cache snapshot.
        std::lock_guard<std::mutex> lock(height_mutex_);
        h = height_status_;
        checked_at = HeightClock::now();
        rx_count = height_status_rx_count_;
        max_gap_ms = height_status_max_gap_ms_;
        callbacks = height_callback_count_;
        rejected = height_rejected_count_;
        last_callback = height_last_callback_;
        callback_gap_ms = height_callback_gap_ms_;
        max_callback_gap_ms = height_max_callback_gap_ms_;
    }
    const double callback_age_ms = last_callback == HeightClock::time_point{}
        ? -1.0 : std::chrono::duration<double, std::milli>(
            checked_at - last_callback).count();

    const double age_ms =
        h.received == HeightClock::time_point{}
            ? -1.0
            : std::chrono::duration<double, std::milli>(
                  checked_at - h.received).count();

    const bool stale =
        age_ms < 0.0 || age_ms > height_status_timeout_ms_;

    const bool invalid = !heightValid(h, checked_at);
    const bool unreferenced = !h.referenced;
    const bool session_changed =
        h.session != height_cycle_session_.load();

    const bool wrong_cut_state =
        phase == HeightPhase::CUT &&
        h.state != "HOLD" && h.state != "TRACK";

    const bool tracking_lost =
        phase == HeightPhase::CUT &&
        height_tracking_expected_.load() &&
        !h.tracking;

    // An existing stop requires containment, but is not itself
    // evidence of a newly occurring height fault.
    if (cut_stop_requested_.load()) {
        stopHeightAsync();
        return;
    }

    const bool bad =
        invalid || unreferenced || session_changed ||
        wrong_cut_state || tracking_lost;

    if (!bad)
        return;

    // Atomic with respect to explicit stop requests/recovery arming.
    const bool first =
        latchObservedMotionStop("height_fault_or_stale");

    if (first) {
        if (torch_controller_)
            torch_controller_->requestOutputOff();

        RCLCPP_ERROR(
            get_logger(),
            "[HEIGHT FIRST FAILURE] "
            "phase=%d state=%s age_ms=%.1f limit_ms=%d "
            "stale=%d invalid=%d referenced=%d fault=%d "
            "configured=%d session=%u expected_session=%u "
            "wrong_cut_state=%d tracking_lost=%d "
            "rx_count=%llu max_rx_gap_ms=%.1f "
            "callbacks=%llu rejected=%llu callback_age_ms=%.1f "
            "callback_gap_ms=%.1f max_callback_gap_ms=%.1f "
            "timestamp_check=callback_receive_only",
            static_cast<int>(phase),
            h.state.c_str(),
            age_ms,
            height_status_timeout_ms_,
            int(stale),
            int(invalid),
            int(h.referenced),
            h.fault,
            int(h.configured),
            h.session,
            height_cycle_session_.load(),
            int(wrong_cut_state),
            int(tracking_lost),
            static_cast<unsigned long long>(rx_count),
            max_gap_ms,
            static_cast<unsigned long long>(callbacks),
            static_cast<unsigned long long>(rejected),
            callback_age_ms,
            callback_gap_ms,
            max_callback_gap_ms);
    }

    stopHeightAsync();

    if (first && move_group_)
        move_group_->stop();
}

CutProfileServer::MotionOutcome CutProfileServer::executeCut(
    const motion::ExecutableCut& cut,
    const std::shared_ptr<GoalHandleCutProfile> goal_handle)
{
    struct ClearHeightPhase {
        std::atomic<HeightPhase>& phase;
        ~ClearHeightPhase() { phase.store(HeightPhase::IDLE); }
    } clear_height_phase{height_phase_};
    if (!torch_controller_) return MotionOutcome::FAILED;
    auto& torch = *torch_controller_;
    const auto canceled = [&]() {
        return !rclcpp::ok() || goal_handle->is_canceling() ||
            cut_stop_requested_.load();
    };
    if (canceled() || torch.hasFault() || !torch.hasFreshIO())
        return MotionOutcome::FAILED;
    const auto before = torch.snapshot();
    if (before.command_on || before.output_on != std::optional<bool>(false) ||
        before.transfer_detected != std::optional<bool>(false)) return MotionOutcome::FAILED;

    if (!rearmHeightStop()) {
        RCLCPP_ERROR(get_logger(), "Height STOP response still pending; cut not started");
        return MotionOutcome::FAILED;
    }
    height_tracking_expected_.store(false);
    MotionOutcome outcome = MotionOutcome::FAILED;
    bool normal = false;
    torch.setActiveSegment(cut.name);
    try {
        do {
            if (!returnHeightHomeAndWait(canceled)) {
                RCLCPP_ERROR(get_logger(), "[CUT] Initial height HOME check failed");
                break;
            }

            if (canceled()) break;

            if (!setFlapClosed(true)) {
                RCLCPP_ERROR(get_logger(), "[CUT] Flap close failed");
                break;
            }
            height_cycle_session_.store(heightSnapshot().session);
            height_phase_.store(HeightPhase::PREP);
            if (!prepareHeightAndWaitReady(canceled)) break;

            height_phase_.store(HeightPhase::IGNITION);
            publishFeedback(goal_handle, cut.name + ": ignition");
            if (torch.requestIgnition(1500ms, canceled) != TorchCommandResult::CONFIRMED)
                break;
            height_phase_.store(HeightPhase::SEARCH);
            if (!searchHeightAndWaitForArc(canceled) || canceled()) break;
            if (!torch.markCutting()) break;
            height_phase_.store(HeightPhase::CUT);

            const auto pierce_until = HeightClock::now() +
                std::chrono::milliseconds(height_pierce_dwell_ms_);
            while (HeightClock::now() < pierce_until && !canceled() && !torch.hasFault())
                std::this_thread::sleep_for(10ms);
            if (canceled() || torch.hasFault() || !enableHeightTracking()) break;

            outcome = executeMotion(cut.process.cut, goal_handle, cut.name + ": cut");
            normal = outcome == MotionOutcome::SUCCESS && !canceled() && !torch.hasFault();
        } while (false);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Cut exception: %s", e.what());
    }

    bool tracking_off = false;
    if (normal) {
        // Normal completion: stop corrections before removing the arc.
        tracking_off = disableHeightTracking();
        normal = tracking_off && !canceled() && !torch.hasFault();
    }
    if (!normal) {
        // Interrupted cut: OFF precedes every potentially blocking height wait.
        torch.requestOutputOff();
        stopHeightAsync();
    }
    const bool off = torch.requestShutdown(1500ms) == TorchCommandResult::CONFIRMED;
    const bool idle = off && torch.finishCut();
    bool homed = false;
    if (normal && tracking_off && idle && !canceled()) {
        height_phase_.store(HeightPhase::RETURN);
        homed = returnHeightHomeAndWait(canceled);
    }
    if (!homed) {
        torch.requestOutputOff();
        stopHeightAsync();
    }
    height_phase_.store(HeightPhase::IDLE);
    torch.setActiveSegment("");
    if (goal_handle->is_canceling()) return MotionOutcome::CANCELED;
    if (!homed || !idle || canceled()) return MotionOutcome::FAILED;
    if (!setFlapClosed(false)) return MotionOutcome::FAILED;
    return outcome;
}


    CutProfileServer::MotionOutcome CutProfileServer::executeCuttingPlan(const motion::ExecutablePlan& executable_plan_,
                const std::shared_ptr<GoalHandleCutProfile>goal_handle){

        for (const auto& cut: executable_plan_.cuts){
            active_segment_=cut.name;
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
            RCLCPP_INFO(
                get_logger(),
                "[CUT SEQUENCE] Approach finished: "
                "feature=%s outcome=%d stop=%d",
                cut.name.c_str(),
                static_cast<int>(outcome),
                static_cast<int>(cut_stop_requested_.load()));
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            RCLCPP_INFO(
                get_logger(),
                "[CUT SEQUENCE] Entering height/torch sequence: %s",
                cut.name.c_str());
            outcome = process_enabled_ ? executeCut(cut,goal_handle) : run(cut.process.cut,"cut (torch disabled)");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }

            outcome = run(cut.process.retract,"retract");
            if(outcome != MotionOutcome::SUCCESS){
                return outcome;
            }
            ++completed_segments_;
            if (cut.transit_out) {
                outcome = run(*cut.transit_out, "return home");
                if (outcome != MotionOutcome::SUCCESS) return outcome;
            }
            

         }
         return MotionOutcome::SUCCESS;
        }

    void CutProfileServer::execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle){

        const auto planning_stop_generation = stop_generation_.load();
        std::lock_guard<std::mutex> execution_lock(execution_mutex_);      

        //un pack / get the goal
        auto const goal = goal_handle->get_goal();

        if(!goal->plan_id.empty()) {
            executeStored(goal->plan_id,goal_handle); // stored plan id 
            return;
        }

        PlanningCancellationScope planning_scope([this, goal_handle, planning_stop_generation] {
            return !rclcpp::ok() || goal_handle->is_canceling() || escape_busy_.load() ||
                stop_generation_.load() != planning_stop_generation;
        });
        checkPlanningCancellation();

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
            releaseGoalAdmission();
            goal_handle->abort(result);
        };

        // ----------------------------------------------------
        // 1. Acquire one accepted workpiece identity and
        //    one matching TaskState version snapshot.
        // ----------------------------------------------------

        TaskState planning_versions;

        if (!readFreshTaskState(planning_versions) ||
            !planning_versions.scene_valid ||
            !planning_versions.workpiece_pose_valid ||
            !planning_versions.supervision_valid ||
            !planning_versions.imu_stable ||
            planning_versions.motion_fault_active ||
            planning_versions.process_fault_active)
        {
            fail(
                0,
                "No valid supervised TaskState "
                "for cutting planning");
            return;
        }

        if (goal->geometry_version !=
                planning_versions.geometry_version ||
            goal->pose_version !=
                planning_versions.pose_version ||
            goal->scene_version !=
                planning_versions.scene_version)
        {
            fail(
                0,
                "Cut goal versions do not match "
                "the authoritative TaskState");
            return;
        }

        publishFeedback(
            goal_handle,
            "acquiring accepted workpiece");

        const auto reference_opt =
            latestWorkpieceReference();

        if (!reference_opt)
        {
            fail(
                0,
                "No TaskState-accepted workpiece reference");
            return;
        }

        // This is the reference used for the entire
        // planning operation. Never silently substitute
        // a newer estimate midway through planning.
        const WorkpieceReference reference =
            *reference_opt;

        const std::string reference_id =
            hb_robot_skills::workpiece::referenceId(
                reference);

        if (reference_id.empty() ||
            reference_id != planning_versions.scene_id)
        {
            fail(
                0,
                "Accepted workpiece identity does not "
                "match TaskState.scene_id");
            return;
        }

        if (reference.estimate.profile_name.empty() ||
            reference.estimate.header.frame_id.empty() ||
            reference.cloud.data.empty())
        {
            fail(
                0,
                "Accepted workpiece reference is malformed");
            return;
        }

        const auto estimate =
            reference.estimate;

        RCLCPP_INFO(
            get_logger(),
            "Accepted workpiece: "
            "id=%s profile=%s "
            "geometry=%llu pose=%llu scene=%llu",
            reference_id.c_str(),
            estimate.profile_name.c_str(),
            static_cast<unsigned long long>(
                planning_versions.geometry_version),
            static_cast<unsigned long long>(
                planning_versions.pose_version),
            static_cast<unsigned long long>(
                planning_versions.scene_version));

        // ----------------------------------------------------
        // 2. Load the profile geometry.
        // ----------------------------------------------------

        publishFeedback(
            goal_handle,
            "loading accepted profile geometry");

        const auto profile_opt =
            hb_perception::ProfileLibrary::find(
                estimate.profile_name);

        if (!profile_opt)
        {
            fail(
                0,
                "Accepted profile is missing from "
                "the profile library: " +
                estimate.profile_name);
            return;
        }

        // ----------------------------------------------------
        // 3. Acquire the current robot state.
        // ----------------------------------------------------

        motion::CutRequest request;

        request.standoff = goal->standoff;
        request.ang_tol = goal->ang_tol;
        request.pos_tol = goal->pos_tol;

        auto current_state =
            move_group_->getCurrentState(2.0);

        if (!current_state)
        {
            fail(
                0,
                "Cannot obtain current robot state");
            return;
        }

        current_state->update();

        {
            std::lock_guard<std::mutex> lock(
                debug_segments_mutex);

            debug_segs_.clear();
        }

        clearCollisionDebug();

        const moveit::core::RobotState
            arbitrary_start_state =
                *current_state;



        // Independent plans must use the actual SRDF home,
        // never the current robot configuration.
        moveit::core::RobotState inspection_home(
            move_group_->getRobotModel());

        inspection_home.setToDefaultValues();

        const auto* home_jmg =
            move_group_->getRobotModel()->getJointModelGroup(
                planning_group_);

        if (!home_jmg) {
            fail(0, "Unknown planning group for inspection_home");
            return;
        }

        const auto& home_names =
            home_jmg->getDefaultStateNames();

        if (std::find(
                home_names.begin(),
                home_names.end(),
                "inspection_home") == home_names.end()) {
            fail(0, "SRDF inspection_home state is missing");
            return;
        }

        inspection_home.setToDefaultValues(
            home_jmg, "inspection_home");

        inspection_home.update();

        // ----------------------------------------------------
        // 4. Obtain the authoritative MoveIt workpiece.
        // ----------------------------------------------------

        publishFeedback(
            goal_handle,
            "synchronizing authoritative cutting scene");

        moveit::planning_interface::
            PlanningSceneInterface psi;

        const auto objects =
            psi.getObjects({
                "inspection_workpiece",
                "wk_workpiece_collision"
            });

        // if (objects.count(
        //         "inspection_workpiece") != 0)
        // {
        //     fail(
        //         0,
        //         "Temporary inspection collision "
        //         "geometry is still present");
        //     return;
        // }

        const auto object_it =
            objects.find(
                "wk_workpiece_collision");

        if (object_it == objects.end())
        {
            fail(
                0,
                "Authoritative workpiece collision "
                "mesh is missing");
            return;
        }

        // Make a local copy. Do not mutate the
        // authoritative object returned by MoveIt.
        moveit_msgs::msg::CollisionObject object =
            object_it->second;

        std::string validation_error;

        if (!hb_robot_skills::workpiece::consistent(
                object,
                reference,
                validation_error))
        {
            fail(
                0,
                "Accepted reference and MoveIt mesh "
                "disagree: " + validation_error);
            return;
        }

        // ----------------------------------------------------
        // 5. Refresh and detach the planning scene.
        // ----------------------------------------------------

        if (!planning_scene_monitor_
                 ->requestPlanningSceneState(
                     "/get_planning_scene"))
        {
            fail(
                0,
                "Could not synchronize the MoveIt "
                "planning scene");
            return;
        }

        planning_scene::PlanningScenePtr
            scene_snapshot;

        {
            planning_scene_monitor::
                LockedPlanningSceneRO locked(
                    planning_scene_monitor_);

            if (!locked)
            {
                fail(
                    0,
                    "Cannot lock the planning scene");
                return;
            }

            scene_snapshot = locked->diff();
            scene_snapshot->decoupleParent();
        }

        if (!scene_snapshot)
        {
            fail(
                0,
                "Failed to create planning snapshot");
            return;
        }

        // Remove any stale temporary geometry from
        // the local snapshot. The authoritative
        // MoveIt scene was checked above.
        // scene_snapshot
        //     ->getWorldNonConst()
        //     ->removeObject(
        //         "inspection_workpiece");

        // Replace the snapshot's workpiece object
        // with the one just read from MoveIt.
        //
        // This avoids trusting an older local PSM
        // copy merely because the object ID matches.
        scene_snapshot
            ->getWorldNonConst()
            ->removeObject(
                "wk_workpiece_collision");

        object.operation =
            moveit_msgs::msg::
                CollisionObject::ADD;

        if (!scene_snapshot
                 ->processCollisionObjectMsg(object))
        {
            fail(
                0,
                "Could not install the authoritative "
                "workpiece into the planning snapshot");
            return;
        }

        if (!prepareCuttingScene(scene_snapshot)) {
            fail(0, "Cutting scene normalization failed");
            return;
        }

        // ----------------------------------------------------
        // 6. Validate reference pose against mesh pose.
        // ----------------------------------------------------

        if (object.meshes.size() != 1 ||
            object.mesh_poses.size() != 1 ||
            !scene_snapshot->knowsFrameTransform(
                object.header.frame_id) ||
            !scene_snapshot->knowsFrameTransform(
                estimate.header.frame_id))
        {
            fail(
                0,
                "Cannot resolve workpiece mesh and "
                "profile estimate frames");
            return;
        }

        Eigen::Isometry3d object_pose =
            Eigen::Isometry3d::Identity();

        Eigen::Isometry3d mesh_pose =
            Eigen::Isometry3d::Identity();

        Eigen::Isometry3d estimate_pose =
            Eigen::Isometry3d::Identity();

        tf2::fromMsg(
            object.pose,
            object_pose);

        tf2::fromMsg(
            object.mesh_poses.front(),
            mesh_pose);

        tf2::fromMsg(
            estimate.pose,
            estimate_pose);

        const Eigen::Isometry3d scene_from_mesh =
            scene_snapshot->getFrameTransform(
                object.header.frame_id) *
            object_pose *
            mesh_pose;

        const Eigen::Isometry3d scene_from_estimate =
            scene_snapshot->getFrameTransform(
                estimate.header.frame_id) *
            estimate_pose;

        if (!scene_from_mesh.matrix().allFinite() ||
            !scene_from_estimate.matrix().allFinite())
        {
            fail(
                0,
                "Nonfinite authoritative workpiece pose");
            return;
        }

        const double position_error =
            (scene_from_mesh.translation() -
             scene_from_estimate.translation()).norm();

        const double rotation_error =
            (scene_from_mesh.linear() -
             scene_from_estimate.linear()).norm();

        RCLCPP_INFO(
            get_logger(),
            "Workpiece snapshot validation: "
            "id=%s position_error=%.4f mm "
            "rotation_matrix_error=%.8f",
            reference_id.c_str(),
            position_error * 1000.0,
            rotation_error);

        if (position_error > 1e-4 ||
            rotation_error > 1e-4)
        {
            fail(
                0,
                "Accepted reference and authoritative "
                "extrusion pose disagree");
            return;
        }

        // ----------------------------------------------------
        // 7. Validate the detached planning snapshot.
        // ----------------------------------------------------

        const auto* world =
            scene_snapshot->getWorld().get();

        if (!world ||
            world->hasObject(
                "inspection_workpiece") ||
            !world->hasObject(
                "wk_workpiece_collision"))
        {
            fail(
                0,
                "Final cutting snapshot is incomplete");
            return;
        }

        // ----------------------------------------------------
        // 8. Recheck TaskState after scene acquisition.
        // ----------------------------------------------------

        TaskState refreshed_versions;

        if (!readFreshTaskState(
                refreshed_versions) ||
            !hb_robot::recovery::sameVersions(
                planning_versions,
                refreshed_versions) ||
            refreshed_versions.scene_id !=
                reference_id ||
            !refreshed_versions.scene_valid ||
            !refreshed_versions.workpiece_pose_valid ||
            !refreshed_versions.supervision_valid ||
            !refreshed_versions.imu_stable ||
            refreshed_versions.motion_fault_active ||
            refreshed_versions.process_fault_active)
        {
            fail(
                0,
                "Workpiece or scene changed while "
                "acquiring the planning snapshot");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Authoritative cutting snapshot ready: "
            "reference=%s geometry=%llu "
            "pose=%llu scene=%llu",
            reference_id.c_str(),
            static_cast<unsigned long long>(
                planning_versions.geometry_version),
            static_cast<unsigned long long>(
                planning_versions.pose_version),
            static_cast<unsigned long long>(
                planning_versions.scene_version));

        publishFeedback(
            goal_handle,
            "authoritative cutting snapshot ready");

        // Continue with the existing geometry planner.

        RCLCPP_INFO(get_logger(), "cut entering geo planner");
        auto geometry=*profile_opt;
        if(!goal->intervals.empty()) {
          const auto parent=stored_plans_.find(goal->parent_plan_id);
          if(parent==stored_plans_.end() || parent->second.cut_id!=goal->cut_id ||
             parent->second.versions.geometry_version!=planning_versions.geometry_version) {
            fail(0,"Unknown parent cut or changed reference geometry");return;
          }
          // Regenerate process entry/exit from original geometry. Never execute a
          // trajectory slice: piercing, standoff, IK and collision checks run again.
          geometry=parent->second.geometry;
          std::vector<hb_perception::ProfileCutFeature> selected;
          std::set<std::string> seen;
          for(const auto& interval:goal->intervals) {
            if(!seen.insert(interval.segment_id).second) {fail(0,"Merge residual bins per feature first");return;}
            auto feature=std::find_if(geometry.cut_features.begin(),geometry.cut_features.end(),
              [&](const auto& f){return f.name==interval.segment_id;});
            const auto executed_feature=std::find_if(parent->second.executable.cuts.begin(),
              parent->second.executable.cuts.end(),[&](const auto& c){return c.name==interval.segment_id;});
            if(feature==geometry.cut_features.end() ||
               executed_feature==parent->second.executable.cuts.end()) {
              fail(0,"Residual feature not in original planned path");return;
            }
            for(const auto& used:attempted_[goal->cut_id])
              if(hb_robot::recovery::overlaps(interval,used)) {fail(0,"Residual region already attempted");return;}
            auto part=*feature;
            if(part.type==hb_perception::ProfileCutFeatureType::Web) {
              const Eigen::Vector2f tangent=(part.end-part.start).normalized();
              part.start+=0.5f*request.torch_diameter*tangent;
              part.end-=0.5f*request.torch_diameter*tangent;
            }
            const Eigen::Vector2f delta=part.end-part.start;
            part.end=part.start+static_cast<float>(interval.s1)*delta;
            part.start+=static_cast<float>(interval.s0)*delta;
            selected.push_back(part);
          }
          geometry.cut_features=std::move(selected);
          request.torch_diameter=0; // End clearance was applied before interval slicing.
        }
        checkPlanningCancellation();
        const auto plan_opt = cut_planner_->plan(estimate,geometry,request,*current_state,scene_snapshot);
        checkPlanningCancellation();
        if(!plan_opt){
           fail(0,"planner_failed");
           return;
        }
        const motion::CutPlan plan = *plan_opt;
        RCLCPP_INFO(get_logger(),"generated %zu cut segments",plan.segments.size());

        std::vector<std::vector<motion::ProcessCandidate>> candidates_by_segment;
        candidates_by_segment.reserve(plan.segments.size());
        const auto& candidate_reference =
            cut_planning_mode_ == CutPlanningMode::Independent
                ? inspection_home
                : arbitrary_start_state;
        for (const auto& segment : plan.segments)
            {

                checkPlanningCancellation();
                RCLCPP_INFO(get_logger(), "Segment: %s generating candidates", segment.name.c_str());





                auto generated = process_generator_->generate(
                    segment,
                    candidate_reference,
                    scene_snapshot
                );

               checkPlanningCancellation();
               RCLCPP_INFO(get_logger(),"Segment: %s generated %zu candidates",
               segment.name.c_str(), generated.candidates.size());
               RCLCPP_INFO(get_logger(),
                   "Process sampling: attempts=%zu, IK=%zu, collision=%zu, bounds=%zu, jump=%zu, TCP=%zu, duplicates=%zu",
                   generated.stats.attempts, generated.stats.ik_failures, generated.stats.collision_failures,
                   generated.stats.joint_limit_failures, generated.stats.joint_jump_failures,
                   generated.stats.tcp_error_failures, generated.stats.duplicate_candidates);
               if(!goal->intervals.empty()) {
                 const auto& parent=stored_plans_.at(goal->parent_plan_id);
                 const auto old=std::find_if(parent.executable.cuts.begin(),parent.executable.cuts.end(),
                   [&](const auto& c){return c.name==segment.name;});
                 const auto interval=std::find_if(goal->intervals.begin(),goal->intervals.end(),
                   [&](const auto& i){return i.segment_id==segment.name;});
                 if(old!=parent.executable.cuts.end() && interval!=goal->intervals.end()) {
                   const auto& trajectory=old->process.cut.trajectory_.joint_trajectory;
                   if(!trajectory.points.empty()) {
                     // Sample index is only a warm-start heuristic, not a claim
                     // that time or joint distance equals Cartesian arc length.
                     const size_t index=static_cast<size_t>(interval->s0*(trajectory.points.size()-1));
                     const auto& positions=trajectory.points[index].positions;
                     if(positions.size()==trajectory.joint_names.size()) {
                       auto seeded=segment;
                       seeded.direction_solutions.clear(); // Preserve the executed-parent warm start.
                       seeded.start_state=std::make_shared<moveit::core::RobotState>(arbitrary_start_state);
                       for(size_t j=0;j<positions.size();++j)
                         seeded.start_state->setVariablePosition(trajectory.joint_names[j],positions[j]);
                       seeded.start_state->update();
                       checkPlanningCancellation();
                       auto warm=process_generator_->generate(seeded,candidate_reference,scene_snapshot);
                       checkPlanningCancellation();
                       generated.candidates.insert(generated.candidates.end(),
                         std::make_move_iterator(warm.candidates.begin()),std::make_move_iterator(warm.candidates.end()));
                     }
                   }
                 }
               }


               if (generated.candidates.empty() &&
                    cut_planning_mode_ ==
                        CutPlanningMode::Monolithic) {
                    fail(0, "no process candidates generated");
                    return;
                }

                candidates_by_segment.push_back(
                    std::move(generated.candidates));

            }
        checkPlanningCancellation();
        RCLCPP_INFO(get_logger(), "starting cut graph planner");

        

            const bool independent =
                cut_planning_mode_ == CutPlanningMode::Independent;

            const std::string batch_id =
                independent
                    ? "cut_batch_" + std::to_string(++next_batch_)
                    : "";

            std::vector<motion::ExecutablePlan> ready_plans;

            if (independent) {
                const auto batch = cut_graph_planner_->planIndependent(
                    plan,
                    candidates_by_segment,
                    inspection_home,
                    batch_id,
                    estimate.profile_name);

                checkPlanningCancellation();
                result->batch_id = batch_id;

                for (const auto& feature : batch.features) {
                    hb_robot_interfaces::msg::CutFeaturePlan status;

                    status.feature_id = feature.feature_id;

                    if (!feature.executable) {
                        status.status =
                            hb_robot_interfaces::msg::CutFeaturePlan::UNPLANNABLE;

                        status.message = feature.failure_reason;

                        RCLCPP_WARN(
                            get_logger(),
                            "[CUT BATCH] feature=%s unplannable: %s",
                            feature.feature_id.c_str(),
                            feature.failure_reason.c_str());
                    } else {
                        status.status =
                            hb_robot_interfaces::msg::CutFeaturePlan::READY;

                        status.message = "Home-to-home trajectory planned";

                        motion::ExecutablePlan single;
                        single.plan_id =
                            batch_id + "/" + feature.feature_id;
                        single.estimate_id = estimate.profile_name;
                        single.cuts.push_back(*feature.executable);

                        ready_plans.push_back(std::move(single));
                    }

                    result->feature_plans.push_back(
                        std::move(status));
                }
            } 
            else {
                auto executable = cut_graph_planner_->plan(
                    plan,
                    candidates_by_segment,
                    arbitrary_start_state,
                    "cut_plan_001",
                    estimate.profile_name);

                checkPlanningCancellation();
                if (!executable) {
                    fail(0, "could not connect cutting operations");
                    return;
                }

                ready_plans.push_back(std::move(*executable));
            }

            const auto& stats = cut_graph_planner_->statistics();

            RCLCPP_INFO(
                get_logger(),
                "graph planning: tested=%zu process_fail=%zu "
                "transit_in_fail=%zu transit_out_fail=%zu accepted=%zu",
                stats.candidates_tested,
                stats.process_failures,
                stats.transit_in_failures,
                stats.transit_out_failures,
                stats.accepted_candidates);

            if (ready_plans.empty()) {
                result->cut_id = goal->cut_id;
                result->parent_plan_id = goal->parent_plan_id;

                for (const auto& segment : plan.segments)
                    result->required_segments.push_back(segment.name);

                fail(0, "No executable feature plans generated");
                return;
            }




        ////////////////////////////////////////////////////////////////////    
        // auto executable_opt = cut_graph_planner_->plan(
        //    plan,
        //    candidates_by_segment,
        //    arbitrary_start_state,
        //    "cut_plan_001" ,
        //    estimate.profile_name
        // );

        // RCLCPP_INFO(get_logger(), "returned  cut graph planner");
        // if(!executable_opt){
        //     fail(0,"could not connect cutting operations");
        //     return;
        // }
        // motion::ExecutablePlan executable_plan_ = std::move(*executable_opt);
        // const auto& stats = cut_graph_planner_->statistics();
        // RCLCPP_INFO(get_logger(),
        // "graph planning: tested %zu, process fail %zu, "
        // "transit_in_fail: %zu, transit_out_fail: %zu, accepted: %zu",
        // stats.candidates_tested,
        // stats.process_failures,
        // stats.transit_in_failures,
        // stats.transit_out_failures,
        // stats.accepted_candidates);
        ////////////////////////////////////////////////////////////////////

            using Plan =
            moveit::planning_interface::MoveGroupInterface::Plan;

        moveit_msgs::msg::DisplayTrajectory display_msg;
        display_msg.model_id =
            move_group_->getRobotModel()->getName();

        auto appendPlan = [&](const Plan& phase,
                            const std::string& name) -> bool {
            const auto& jt = phase.trajectory_.joint_trajectory;

            if (jt.joint_names.empty() || jt.points.empty()) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Empty trajectory: %s",
                    name.c_str());
                return false;
            }

            display_msg.trajectory.push_back(
                phase.trajectory_);

            return true;
        };

        const auto& first =
            ready_plans.front().cuts.front().transit_in;

        display_msg.trajectory_start = first.start_state_;

        if (display_msg.trajectory_start.joint_state.name.empty()) {
            moveit::core::robotStateToRobotStateMsg(
                independent
                    ? inspection_home
                    : arbitrary_start_state,
                display_msg.trajectory_start);
        }

        bool valid = true;

        for (const auto& executable : ready_plans) {
            for (const auto& cut : executable.cuts) {
                valid &= appendPlan(
                    cut.transit_in,
                    cut.name + "/transit_in");

                valid &= appendPlan(
                    cut.process.approach,
                    cut.name + "/approach");

                valid &= appendPlan(
                    cut.process.cut,
                    cut.name + "/cut");

                valid &= appendPlan(
                    cut.process.retract,
                    cut.name + "/retract");

                if (cut.transit_out) {
                    valid &= appendPlan(
                        *cut.transit_out,
                        cut.name + "/return_home");
                } else if (independent) {
                    valid = false;
                    RCLCPP_ERROR(
                        get_logger(),
                        "Independent cut %s has no return-home trajectory",
                        cut.name.c_str());
                }
            }
        }

        if (!valid) {
            fail(0, "Empty or incomplete cutting trajectories");
            return;
        }

        display_traj_pub_->publish(display_msg);

        RCLCPP_INFO(
            get_logger(),
            "Published %zu cutting trajectories",
            display_msg.trajectory.size());










            RCLCPP_INFO(get_logger(), "published %zu cutting traj", display_msg.trajectory.size());
            
        
       

        publishVisualization(plan,estimate.header.frame_id);
        publishFeedback(goal_handle,"visualizing, cut plan ready");

        checkPlanningCancellation();
        
        TaskState current_versions;

        const bool permitted =
            taskPermitsExecution(&current_versions);

        const bool current =
            permitted &&
            hb_robot::recovery::sameVersions(
                planning_versions,
                current_versions) &&
            current_versions.scene_id == reference_id;

        



        

        if (!current)
        {
            RCLCPP_WARN(
                get_logger(),
                "Cut candidate is not executable: "
                "planned geometry=%llu pose=%llu scene=%llu "
                "current geometry=%llu pose=%llu scene=%llu "
                "reference=%s current_reference=%s",
                static_cast<unsigned long long>(
                    planning_versions.geometry_version),
                static_cast<unsigned long long>(
                    planning_versions.pose_version),
                static_cast<unsigned long long>(
                    planning_versions.scene_version),
                static_cast<unsigned long long>(
                    current_versions.geometry_version),
                static_cast<unsigned long long>(
                    current_versions.pose_version),
                static_cast<unsigned long long>(
                    current_versions.scene_version),
                reference_id.c_str(),
                current_versions.scene_id.c_str());
        }

        // const auto id="cut_plan_"+std::to_string(++next_plan_);
        // // Keep original geometry even for a recovery plan, so future interval
        // // coordinates always refer to the initial feature, never to a prior slice.
        // stored_plans_[id]=StoredPlan{executable_plan_,*profile_opt,planning_versions,
        //   goal->cut_id,goal->parent_plan_id,goal->intervals,false};
        // for(const auto& segment:plan.segments) result->required_segments.push_back(segment.name);
        // result->plan_id=id;
        // result->cut_id=goal->cut_id;
        // result->parent_plan_id=goal->parent_plan_id;
        // if(goal->execute && !skip_motion_) {
        //     executeStored(id,goal_handle);
        //     return;
        // }
        // result->success=true;
       
        // std::ostringstream summary;
        // summary << "Candidate stored for geometry=" << planning_versions.geometry_version
        //         << ", pose=" << planning_versions.pose_version << ", scene=" << planning_versions.scene_version;
        // if(current) summary << "; execution requires live checks";
        // else summary << "; execution blocked at preview completion (current geometry="
        //              << current_versions.geometry_version << ", pose=" << current_versions.pose_version
        //              << ", scene=" << current_versions.scene_version << ", " << current_versions.status_message << ")";
        // result->message=summary.str();

        // RCLCPP_INFO(
        //     get_logger(),
        //     "[CUT HANDOFF] Planning finished "
        //     "plan_id='%s'; publishing result",
        //     result->plan_id.c_str());

        // // Release admission before the BT can observe SUCCESS.
        // releaseGoalAdmission();

        // goal_handle->succeed(result);
        // The complete requested feature set remains authoritative,
// even when only some features were planned successfully.
        for (const auto& segment : plan.segments) {
            result->required_segments.push_back(segment.name);
        }

        result->cut_id = goal->cut_id;
        result->parent_plan_id = goal->parent_plan_id;

        for (auto& executable : ready_plans) {
            const std::string id =
                "cut_plan_" + std::to_string(now().nanoseconds()) + "_" + std::to_string(++next_plan_);

            executable.plan_id = id;

            // Preserve original geometry for recovery interval slicing.
            StoredPlan stored{
                std::move(executable),
                *profile_opt,
                planning_versions,
                goal->cut_id,
                goal->parent_plan_id,
                goal->intervals,
                false
            };

            // If you added StoredPlan::batch_id, assign it here.
            // stored.batch_id = batch_id;

            const std::string feature_name =
                independent
                    ? stored.executable.cuts.front().name
                    : "";

            if (independent) {
                stored.intervals.erase(
                    std::remove_if(stored.intervals.begin(), stored.intervals.end(),
                        [&](const auto& interval) {
                            return interval.segment_id != feature_name;
                        }),
                    stored.intervals.end());
            }

            stored_plans_.emplace(
                id, std::move(stored));

            if (independent) {
                auto status = std::find_if(
                    result->feature_plans.begin(),
                    result->feature_plans.end(),
                    [&](const auto& item) {
                        return item.feature_id == feature_name &&
                            item.status ==
                                hb_robot_interfaces::msg::
                                    CutFeaturePlan::READY;
                    });

                if (status == result->feature_plans.end()) {
                    fail(0, "Internal feature result mismatch");
                    return;
                }

                status->plan_id = id;

                RCLCPP_INFO(
                    get_logger(),
                    "[CUT BATCH] feature=%s stored plan=%s",
                    feature_name.c_str(),
                    id.c_str());
            } else {
                result->plan_id = id;
            }
        }

        if (!independent &&
            goal->execute &&
            !skip_motion_) {
            checkPlanningCancellation();
            planning_scope.clear();
            executeStored(result->plan_id, goal_handle);
            return;
        }

        result->success = true;

        std::ostringstream summary;

        if (independent) {
            const auto ready = std::count_if(
                result->feature_plans.begin(),
                result->feature_plans.end(),
                [](const auto& feature) {
                    return feature.status ==
                        hb_robot_interfaces::msg::
                            CutFeaturePlan::READY;
                });

            summary
                << "Independent batch " << batch_id
                << ": " << ready << "/"
                << result->feature_plans.size()
                << " features planned";

            if (ready != result->feature_plans.size())
                summary << "; task remains incomplete";
        } else {
            summary << "Candidate stored";
        }

        summary
            << "; geometry=" << planning_versions.geometry_version
            << " pose=" << planning_versions.pose_version
            << " scene=" << planning_versions.scene_version;

        if (!current)
            summary << "; execution requires renewed TaskState validation";

        result->message = summary.str();

        RCLCPP_INFO(
            get_logger(),
            "[CUT HANDOFF] %s",
            result->message.c_str());

        releaseGoalAdmission();
        goal_handle->succeed(result);
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
           std::string operation; 
            {
                std::lock_guard<std::mutex> lock(approval_mutex_);

                    if (!approval_pending_ ||
                        motion_approved_ ||
                        motion_rejected_) {
                        response->accepted = false;
                        response->message =
                            "No pending motion approval";
                        return;
                    }


                    operation = approval_operation_;       
                    motion_approved_ = request->approve;
                    motion_rejected_ = !request->approve;
        }
            approval_cv_.notify_all();

            response->accepted = true;

            response->message =
                request->approve
                    ? "motion approved"
                    : "motion rejected";

        }




void CutProfileServer::handleTorchFault(
    const std::shared_ptr<
        hb_robot_interfaces::srv::ClearTorchFault::Request> request,
    std::shared_ptr<
        hb_robot_interfaces::srv::ClearTorchFault::Response> response)
{
    response->success = false;

    if (!request->operator_approved) {
        response->message = "Operator approval required";
        return;
    }

    if (busy_.load() ||
        motion_active_.load() ||
        retreat_active_.load() ||
        escape_busy_.load()) {
        response->message =
            "Cutting, recovery, or another action is active";
        return;
    }

    if (!torch_controller_) {
        response->message = "Torch controller unavailable";
        return;
    }

    // Do not permit a fault reset while an execution worker
    // owns the motion path.
    std::unique_lock<std::mutex> execution_lock(
        execution_mutex_, std::try_to_lock);

    if (!execution_lock.owns_lock()) {
        response->message =
            "Execution worker has not finished";
        return;
    }

    // Recheck after acquiring the execution lock.
    if (busy_.load() ||
        motion_active_.load() ||
        retreat_active_.load() ||
        escape_busy_.load()) {
        response->message = "Execution state changed";
        return;
    }

    const auto task = taskStateFreshness();

    if (!task.fresh ||
        !task.state.supervision_valid ||
        !task.state.imu_stable ||
        !task.state.workpiece_pose_valid ||
        task.state.motion_fault_active) {
        response->message =
            "Fresh, stable TaskState and valid pose required";
        return;
    }

    if (!torch_controller_->hasFreshIO()) {
        response->message = "Torch I/O stale";
        return;
    }

    const auto telemetry = torch_controller_->snapshot();

    if (telemetry.output_on != std::optional<bool>(false) ||
        telemetry.transfer_detected != std::optional<bool>(false)) {
        response->message =
            "Torch OFF and transfer absent must be confirmed";
        return;
    }

    if (!torch_controller_->clearFault()) {
        response->message =
            "Torch fault could not be cleared";
        return;
    }

    // Only the local torch controller is cleared here.
    // TaskStateServer owns its own latched process fault.
    response->success = true;
    response->message =
        "Torch controller fault cleared; "
        "clear TaskState process latch separately";

    RCLCPP_WARN(
        get_logger(),
        "[TORCH CLEAR] Local fault cleared by operator request");
}

    /////////////////////// RECOVERY GOALL HANDLING //////////////////////////
   
    rclcpp_action::GoalResponse
        CutProfileServer::handleRecoveryGoal(
            const rclcpp_action::GoalUUID&,
            std::shared_ptr<const RecoverCut::Goal> goal)
        {
            if (goal->interrupted_plan_id.empty() ||
                goal->cut_id.empty() ||
                escape_busy_.load())
                return rclcpp_action::GoalResponse::REJECT;

            bool idle = false;

            if (!busy_.compare_exchange_strong(idle, true))
                return rclcpp_action::GoalResponse::REJECT;

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse
CutProfileServer::handleRecoveryCancel(
    const std::shared_ptr<GoalHandleRecoverCut>)
{
    requestRecoveryStop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

    void CutProfileServer::handleRecoveryAccepted(
        const std::shared_ptr<GoalHandleRecoverCut> goal)
    {
        if (recovery_worker_.joinable())
            recovery_worker_.join();

        recovery_worker_ = std::thread(
            [this, goal] {
                try {
                    recoverCut(goal);
                } catch (const std::exception& e) {
                    requestRecoveryStop();
                    auto result =
                        std::make_shared<RecoverCut::Result>();

                    result->success = false;
                    result->interrupted = true;
                    result->pose_revalidation_required = !taskStateFreshness().state.workpiece_pose_valid;
                    result->message = e.what();

                    releaseGoalAdmission();

                    if (goal->is_canceling())
                        goal->canceled(result);
                    else
                        goal->abort(result);
                }
            });
    }


       
CutProfileServer::MotionOutcome
CutProfileServer::waitForRecoveryApproval(
    const std::shared_ptr<GoalHandleRecoverCut>& goal)
{
    while (rclcpp::ok()) {
        if (goal->is_canceling())
            return MotionOutcome::CANCELED;

        if (escape_busy_.load() || !recoveryHardwareReady())
            return MotionOutcome::FAILED;

        const auto task = taskStateFreshness();

        if (!task.fresh ||
            !hb_robot::recovery::geometryPermitted(task.state) ||
            !task.state.scene_valid ||
            !hb_robot::recovery::sameVersions(
                task.state, execution_state_) ||
            !torch_controller_ ||
            !torch_controller_->hasFreshIO() ||
            torch_controller_->hasFault())
            return MotionOutcome::FAILED;

        const auto t = torch_controller_->snapshot();

        if (t.output_on != std::optional<bool>(false) ||
            t.transfer_detected != std::optional<bool>(false))
            return MotionOutcome::FAILED;

        std::unique_lock<std::mutex> lock(approval_mutex_);

        if (motion_rejected_)
            return MotionOutcome::REJECTED;

        if (motion_approved_)
            return MotionOutcome::SUCCESS;

        approval_cv_.wait_for(
            lock, std::chrono::milliseconds(100));
    }

    return MotionOutcome::FAILED;
}

void CutProfileServer::requestRecoveryStop()
{
    latchMotionStop("requestRecoveryStop");

    if (torch_controller_)
        torch_controller_->requestOutputOff();

    if (move_group_ &&
        (motion_active_.load() ||
         recovery_motion_active_.load()))
        move_group_->stop();

    approval_cv_.notify_all();
}


    
    CutProfileServer::MotionOutcome
    CutProfileServer::executeRecoveryMotion(
            const moveit::planning_interface::MoveGroupInterface::Plan& plan,
            const std::shared_ptr<GoalHandleRecoverCut>& goal,
            const std::string& phase)
        {
            if (goal->is_canceling())
                return MotionOutcome::CANCELED;

            const auto safe = [&]() {
                const auto task = taskStateFreshness();

                if (!recoveryHardwareReady() || !task.fresh ||
                    !hb_robot::recovery::geometryPermitted(task.state) ||
                    !task.state.scene_valid ||
                    !hb_robot::recovery::sameVersions(
                        task.state, execution_state_) ||
                    task.state.cut_environment_id != execution_state_.cut_environment_id ||
                    !torch_controller_ ||
                    !torch_controller_->hasFreshIO() ||
                    torch_controller_->hasFault())
                    return false;

                const auto t = torch_controller_->snapshot();

                return t.output_on == std::optional<bool>(false) &&
                    t.transfer_detected == std::optional<bool>(false);
            };

            if (cut_stop_requested_.load() || !safe())
                return MotionOutcome::FAILED;

            auto feedback = std::make_shared<RecoverCut::Feedback>();
            feedback->stage = phase;
            goal->publish_feedback(feedback);

            const auto outcome = move_group_->execute(plan);

            if (goal->is_canceling())
                return MotionOutcome::CANCELED;

            if (cut_stop_requested_.load() ||
                !safe() ||
                outcome != moveit::core::MoveItErrorCode::SUCCESS)
                return MotionOutcome::FAILED;

            return MotionOutcome::SUCCESS;
        }

                    namespace {

                    using RecoveryPlan =
                        moveit::planning_interface::MoveGroupInterface::Plan;

                    bool recoveryJointMatch(
                        const moveit::core::RobotState& actual,
                        const moveit::core::RobotState& target,
                        const moveit::core::JointModelGroup* jmg,
                        double tolerance_rad = 0.02)
                    {
                        if (!jmg) return false;

                        std::vector<double> a, b;
                        actual.copyJointGroupPositions(jmg, a);
                        target.copyJointGroupPositions(jmg, b);

                        if (a.size() != b.size() || a.empty())
                            return false;

                        for (size_t i = 0; i < a.size(); ++i) {
                            if (!std::isfinite(a[i]) ||
                                !std::isfinite(b[i]) ||
                                std::abs(a[i] - b[i]) > tolerance_rad)
                                return false;
                        }
                        return true;
                    }

                    bool recoveryPathValid(
                        const RecoveryPlan& plan,
                        const planning_scene::PlanningSceneConstPtr& scene,
                        const std::string& group)
                    {
                        if (!scene) return false;

                        const auto& jt = plan.trajectory_.joint_trajectory;
                        if (jt.points.empty() || jt.joint_names.empty())
                            return false;

                        auto state = scene->getCurrentState();
                        const auto* jmg = state.getJointModelGroup(group);
                        if (!jmg ||
                            jt.joint_names.size() !=
                                jmg->getActiveJointModels().size())
                            return false;

                        for (const auto* joint : jmg->getActiveJointModels()) {
                            if (std::count(jt.joint_names.begin(),
                                        jt.joint_names.end(),
                                        joint->getName()) != 1)
                                return false;
                        }

                        for (const auto& point : jt.points) {
                            if (point.positions.size() != jt.joint_names.size())
                                return false;
                            for (double q : point.positions)
                                if (!std::isfinite(q)) return false;
                        }

                        auto collisionFree = [&](const std::vector<double>& q) {
                            for (size_t j = 0; j < q.size(); ++j)
                                state.setVariablePosition(jt.joint_names[j], q[j]);

                            state.update();
                            
                            return state.satisfiesBounds(jmg) &&
                                !scene->isStateColliding(state, group);
                        };

                        if (!collisionFree(jt.points.front().positions))
                            return false;

                        for (size_t i = 1; i < jt.points.size(); ++i) {
                            const auto& a = jt.points[i - 1].positions;
                            const auto& b = jt.points[i].positions;

                            double max_delta = 0.0;
                            for (size_t j = 0; j < a.size(); ++j)
                                max_delta =
                                    std::max(max_delta, std::abs(b[j] - a[j]));

                            const int steps = std::max(
                                1, static_cast<int>(
                                    std::ceil(max_delta / 0.01)));

                            for (int k = 1; k <= steps; ++k) {
                                std::vector<double> q(a.size());

                                const double alpha =
                                    static_cast<double>(k) / steps;

                                for (size_t j = 0; j < a.size(); ++j)
                                    q[j] = a[j] + alpha * (b[j] - a[j]);

                                if (!collisionFree(q))
                                    return false;
                            }
                        }

                        return true;
                    }

                    } // anonymous namespace


bool CutProfileServer::recoveryRobotStationary(
    const moveit::core::JointModelGroup* jmg)
{
    return jmg && waitForRobotStationary(
        std::chrono::milliseconds(3000), std::chrono::milliseconds(300));
}

bool CutProfileServer::armApprovedRecoveryMotion(
    uint64_t expected_generation,
    const std::shared_ptr<GoalHandleRecoverCut>& goal)
{
    std::lock_guard<std::mutex> lock(stop_mutex_);

    if (stop_generation_.load() != expected_generation ||
        goal->is_canceling() ||
        escape_busy_.load() ||
        !busy_.load() ||
        recovery_motion_active_.load()) {
        return false;
    }

    const auto task = taskStateFreshness();

    if (!recoveryHardwareReady() || !task.fresh ||
        !hb_robot::recovery::geometryPermitted(task.state) ||
        !task.state.scene_valid ||
        !hb_robot::recovery::sameVersions(
            task.state, execution_state_) ||
        task.state.cut_environment_id != execution_state_.cut_environment_id ||
        !torch_controller_ ||
        !torch_controller_->hasFreshIO() ||
        torch_controller_->hasFault()) {
        return false;
    }

    const auto telemetry = torch_controller_->snapshot();

    if (telemetry.output_on != std::optional<bool>(false) ||
        telemetry.transfer_detected != std::optional<bool>(false)) {
        return false;
    }

    // Recheck the same feedback window immediately before recovery is armed.
    if (!robotStationaryConfirmed(std::chrono::milliseconds(300)))
        return false;

    // Clear only the previous interruption's stop latch.
    // New stop requests must acquire stop_mutex_.
    cut_stop_requested_.store(false);
    recovery_motion_active_.store(true);
    motion_active_.store(true);

    return true;
}

void CutProfileServer::finishRecoveryMotion()
{
    // Call only after MoveIt execution is terminal.
    std::lock_guard<std::mutex> lock(stop_mutex_);
    recovery_motion_active_.store(false);
    motion_active_.store(false);
}
                
    void CutProfileServer::recoverCut(
        const std::shared_ptr<GoalHandleRecoverCut> goal)
            {
                using Clock = std::chrono::steady_clock;

                auto result =
                    std::make_shared<RecoverCut::Result>();


                    result->success = false;
                    result->home_reached = false;
                    result->interrupted = true;

                result->pose_revalidation_required = !taskStateFreshness().state.workpiece_pose_valid;

                auto fail = [&](const std::string& message) {
                    

                    result->success = false;
                    result->home_reached = false;
                    result->interrupted = true;
                    result->message = message;
                    result->pose_revalidation_required = !taskStateFreshness().state.workpiece_pose_valid;

                    releaseGoalAdmission();

                    if (goal->is_canceling())
                        goal->canceled(result);
                    else
                        goal->abort(result);
                };

                // Do not wait indefinitely behind an active cut.
                std::unique_lock<std::mutex> execution_lock(
                    execution_mutex_, std::try_to_lock);

                if (!execution_lock.owns_lock()) {
                    fail("Execution worker still active");
                    return;
                }

                const auto request = goal->get_goal();

                const auto it =
                    stored_plans_.find(request->interrupted_plan_id);

                if (it == stored_plans_.end()) {
                    fail("Interrupted stored plan not found");
                    return;
                }

                const auto& stored = it->second;

                if (stored.cut_id != request->cut_id ||
                    !stored.consumed ||
                    stored.executable.cuts.size() != 1) {
                    fail("Invalid interrupted feature plan");
                    return;
                }

                if (!torch_controller_ ||
                    !torch_controller_->hasFreshIO()) {
                    fail("Torch telemetry unavailable");
                    return;
                }

                const auto telemetry =
                    torch_controller_->snapshot();

                if (telemetry.output_on !=
                        std::optional<bool>(false) ||
                    telemetry.transfer_detected !=
                        std::optional<bool>(false) ||
                    torch_controller_->hasFault()) {
                    fail("Torch is not confirmed safe for recovery");
                    return;
                }

                if (!recoveryHardwareReady()) {
                    fail("Recovery requires fresh torch OFF and referenced height HOME (position=0, switch active, tracking off)");
                    return;
                }
                const uint64_t recovery_stop_generation = stop_generation_.load();
                const auto task = taskStateFreshness();

                if (!task.fresh ||
                    !hb_robot::recovery::geometryPermitted(
                        task.state) ||
                    !hb_robot::recovery::sameVersions(
                        task.state, stored.versions) ||
                    task.state.cut_environment_id != stored.versions.cut_environment_id ||
                    !task.state.scene_valid) {
                    fail("TaskState is not valid for recovery");
                    return;
                }

                

                // Continue with flap opening, stationary-state checks,
                // fresh scene validation and recovery planning.

                ////
                const auto& cut =
                    stored.executable.cuts.front();

                const auto retract_goal =
                    getFinalState(cut.process.retract);

                /// recover to home:


                if (!waitForRobotStationary(std::chrono::milliseconds(3000),
                                            std::chrono::milliseconds(300))) {
                    fail("Robot not confirmed stationary before recovery planning");
                    return;
                }

                auto actual = move_group_->getCurrentState(2.0);

                if (!actual) {
                    fail("Cannot obtain actual robot state");
                    return;
                }

                actual->update();

                // Construct inspection_home from the SRDF,
                // as already done in your planning code.
                moveit::core::RobotState home(
                    move_group_->getRobotModel());

                home.setToDefaultValues();

                const auto* jmg =
                    home.getJointModelGroup(planning_group_);

                if (!jmg) {
                    fail("Planning group unavailable");
                    return;
                }

                const auto& names = jmg->getDefaultStateNames();

                if (std::find(
                        names.begin(),
                        names.end(),
                        "inspection_home") == names.end()) {
                    fail("SRDF inspection_home missing");
                    return;
                }

                home.setToDefaultValues(
                    jmg, "inspection_home");

                home.update();

                std::optional<
                    moveit::planning_interface::
                        MoveGroupInterface::Plan> to_retract;

                std::optional<
                    moveit::planning_interface::
                        MoveGroupInterface::Plan> to_home;



                if (request->prefer_retract_endpoint) {
                    to_retract = planToState(
                        *actual, retract_goal);

                    if (goal->is_canceling() || escape_busy_.load() ||
                        stop_generation_.load()!=recovery_stop_generation || !recoveryHardwareReady()) {
                        fail("Recovery canceled or preconditions changed during planning");
                        return;
                    }
                    if (to_retract) {
                        to_home = planToState(
                            getFinalState(*to_retract), home);
                    }
                }
                if (goal->is_canceling() || escape_busy_.load() ||
                    stop_generation_.load()!=recovery_stop_generation || !recoveryHardwareReady()) {
                    fail("Recovery canceled or preconditions changed during planning");
                    return;
                }
                if (!setFlapClosed(false)) {
                    fail("Could not open camera flap for recovery");
                    return;
                }

                // Do not execute anything yet.
                // Both legs must pass fresh collision and
                // trajectory-continuity validation.
               
                // Both legs must exist before asking for approval.
                if (!to_retract || !to_home) {
                    fail("Two-leg recovery route unavailable");
                    return;
                }
           
            ///////////////////////////////////////////////////////////////


            if (!planning_scene_monitor_->requestPlanningSceneState(
                    "/get_planning_scene")) {
                fail("Cannot refresh recovery planning scene");
                return;
            }

            planning_scene::PlanningScenePtr scene;
            {
                planning_scene_monitor::LockedPlanningSceneRO locked(
                    planning_scene_monitor_);

                if (!locked) {
                    fail("Cannot lock recovery scene");
                    return;
                }

                auto copy = locked->diff();
                copy->decoupleParent();
                scene = copy;
            }
            if (!prepareCuttingScene(scene)) {
                    fail("Recovery scene normalization failed");
                    return;
                }

            if (!scene ||
                !scene->getWorld()->hasObject("wk_workpiece_collision") ||
                scene->getWorld()->hasObject("inspection_workpiece")) {
                fail("Recovery scene is missing authoritative workpiece");
                return;
            }

            if (!recoveryPathValid(*to_retract, scene, planning_group_) ||
                !recoveryPathValid(*to_home, scene, planning_group_)) {
                fail("Recovery trajectory collision/bounds validation failed");
                return;
            }

            const auto first_start =
                [&] {
                    moveit::core::RobotState s(move_group_->getRobotModel());
                    moveit::core::robotStateMsgToRobotState(
                        to_retract->start_state_, s);
                    s.update();
                    return s;
                }();

            const auto first_end = getFinalState(*to_retract);

            const auto second_start =
                [&] {
                    moveit::core::RobotState s(move_group_->getRobotModel());
                    moveit::core::robotStateMsgToRobotState(
                        to_home->start_state_, s);
                    s.update();
                    return s;
                }();

            if (!recoveryJointMatch(*actual, first_start, jmg)) {
                fail("First recovery plan does not start at stopped robot state");
                return;
            }

            if (!recoveryJointMatch(first_end, second_start, jmg)) {
                fail("Recovery legs are not joint-continuous");
                return;
            }

            if (!recoveryJointMatch(getFinalState(*to_home), home, jmg)) {
                fail("Recovery plan does not terminate at inspection_home");
                return;
            }
            
            if (!actual->satisfiesBounds(jmg) ||
                scene->isStateColliding(*actual, planning_group_)) {
                fail("Stopped robot state is invalid or colliding");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "[CUT RECOVERY] Both recovery legs validated "
                "against synchronized scene");


            auto feedback = std::make_shared<RecoverCut::Feedback>();
            feedback->stage = "awaiting recovery motion approval";
            goal->publish_feedback(feedback);

               

            execution_state_ = task.state;

            {
                std::lock_guard<std::mutex> lock(approval_mutex_);

                motion_approved_ = false;
                motion_rejected_ = false;
                approval_operation_ =
                    "recovery/" + request->interrupted_plan_id;
                approval_pending_ = true;
            }





            RCLCPP_WARN(
                get_logger(),
                "[CUT RECOVERY] Awaiting operator approval: "
                "plan=%s route=actual->retract->inspection_home",
                request->interrupted_plan_id.c_str());
            const uint64_t stop_generation_before_approval = recovery_stop_generation;
            const auto approval = waitForRecoveryApproval(goal);

            {
                std::lock_guard<std::mutex> lock(approval_mutex_);
                approval_pending_ = false;
                approval_operation_.clear();
            }

            if (approval != MotionOutcome::SUCCESS) {
                fail("Recovery approval rejected or invalidated");
                return;
            }

            // Recheck fresh TaskState, actual start configuration,
            // torch OFF, flap clearance, scene and both trajectories
            // after approval. Approval alone must not bypass these gates.

            ///////////////////////////////////////////////////

            // Recheck after approval.
            const auto refreshed = taskStateFreshness();

            if (!recoveryHardwareReady() || !refreshed.fresh ||
                !hb_robot::recovery::geometryPermitted(refreshed.state) ||
                !refreshed.state.scene_valid ||
                !hb_robot::recovery::sameVersions(
                    refreshed.state, execution_state_) ||
                refreshed.state.cut_environment_id != execution_state_.cut_environment_id) {
                fail("TaskState changed after recovery approval");
                return;
            }

            if (goal->is_canceling() || escape_busy_.load()) {
                fail("Recovery canceled before execution");
                return;
            }

            // Approval may have been pending for minutes. Refresh and validate
            // BOTH legs again before arming, with the same scene normalization.
            if (!planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene")) {
                fail("Cannot refresh recovery scene after approval");
                return;
            }
            {
                planning_scene_monitor::LockedPlanningSceneRO locked(planning_scene_monitor_);
                if (!locked) { fail("Cannot lock post-approval recovery scene"); return; }
                scene = locked->diff();
                scene->decoupleParent();
            }
            if (!prepareCuttingScene(scene) ||
                !scene->getWorld()->hasObject("wk_workpiece_collision") ||
                scene->getWorld()->hasObject("inspection_workpiece") ||
                !recoveryPathValid(*to_retract, scene, planning_group_) ||
                !recoveryPathValid(*to_home, scene, planning_group_)) {
                fail("Recovery paths invalid after approval");
                return;
            }
            if (!recoveryRobotStationary(jmg)) {
                fail("Robot not confirmed stationary");
                return;
            }

            auto measured_start = move_group_->getCurrentState(2.0);

            if (!measured_start ||
                !recoveryJointMatch(*measured_start, first_start, jmg)) {
                fail("Recovery start state changed after approval");
                return;
            }

            // REQUIRED: synchronized recovery admission that clears
            // only the previous stop latch and cannot erase a new stop.
            // Do not replace this with an unconditional store(false).
            if (!armApprovedRecoveryMotion(
                    stop_generation_before_approval, goal)) {
                fail("Recovery motion admission denied");
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "[CUT RECOVERY] Approved recovery motion armed");

            ////////////////////////////////////

            const auto first = executeRecoveryMotion(
                *to_retract, goal, "recovery: to retract");

            if (first != MotionOutcome::SUCCESS) {
                requestRecoveryStop();
                // Keep admission owned until execution is terminal.
                finishRecoveryMotion();
                fail("Recovery retract motion failed");
                return;
            }

            auto measured_retract = move_group_->getCurrentState(2.0);

            if (!measured_retract ||
                !recoveryJointMatch(
                    *measured_retract, second_start, jmg)) {
                requestRecoveryStop();
                finishRecoveryMotion();
                fail("Robot not at second recovery leg start");
                return;
            }
            /////////////////////////////
            const auto second = executeRecoveryMotion(
                *to_home, goal, "recovery: return home");

            if (second != MotionOutcome::SUCCESS) {
                requestRecoveryStop();
                finishRecoveryMotion();
                fail("Recovery return-home motion failed");
                return;
            }

            if (!recoveryRobotStationary(jmg) || !recoveryHardwareReady() ||
                goal->is_canceling() || cut_stop_requested_.load()) {
                requestRecoveryStop();
                finishRecoveryMotion();
                fail("Recovery home stop/hardware confirmation failed");
                return;
            }
            auto measured_home = move_group_->getCurrentState(2.0);

            if (!measured_home || !robotStationaryConfirmed(std::chrono::milliseconds(300)) ||
                !recoveryJointMatch(*measured_home, home, jmg)) {
                requestRecoveryStop();
                finishRecoveryMotion();
                fail("Inspection home not confirmed");
                return;
            }

            const auto final_task=taskStateFreshness();
            if (goal->is_canceling() || cut_stop_requested_.load() ||
                !recoveryHardwareReady() || !final_task.fresh ||
                !hb_robot::recovery::geometryPermitted(final_task.state) ||
                !hb_robot::recovery::sameVersions(final_task.state,execution_state_) ||
                final_task.state.cut_environment_id!=execution_state_.cut_environment_id) {
                requestRecoveryStop();
                finishRecoveryMotion();
                fail("Recovery authorization lost at completion");
                return;
            }
            finishRecoveryMotion();


            //////////////////////////////////////////////////////

            result->success = true;
            result->home_reached = true;
            result->interrupted = false;
            result->pose_revalidation_required = !taskStateFreshness().state.workpiece_pose_valid;
            result->message = "Recovery completed at inspection_home";

            releaseGoalAdmission();
            goal->succeed(result);

            }
            bool CutProfileServer::latchObservedMotionStop(const char* reason)
{
    std::lock_guard<std::mutex> lock(stop_mutex_);

    // A timer seeing an already-latched stop is not a new stop event.
    if (cut_stop_requested_.load())
        return false;

    ++stop_generation_;
    cut_stop_requested_.store(true);

    RCLCPP_ERROR(
        get_logger(),
        "[CUT STOP FIRST] reason=%s "
        "generation=%llu motion_active=%d "
        "recovery_active=%d retreat_active=%d",
        reason,
        static_cast<unsigned long long>(stop_generation_.load()),
        int(motion_active_.load()),
        int(recovery_motion_active_.load()),
        int(retreat_active_.load()));

    return true;
}
    bool CutProfileServer::latchMotionStop(const char* reason)
{
    std::lock_guard<std::mutex> lock(stop_mutex_);

    const bool first_stop =
        !cut_stop_requested_.load();

    ++stop_generation_;
    cut_stop_requested_.store(true);

    if (first_stop) {
        RCLCPP_ERROR(
            get_logger(),
            "[CUT STOP FIRST] reason=%s "
            "generation=%llu motion_active=%d "
            "recovery_active=%d retreat_active=%d",
            reason,
            static_cast<unsigned long long>(
                stop_generation_.load()),
            int(motion_active_.load()),
            int(recovery_motion_active_.load()),
            int(retreat_active_.load()));
    }

    return first_stop;
}


}//namescape skills



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

    rclcpp::executors::SingleThreadedExecutor height_executor;

    height_executor.add_callback_group(
        node->heightStatusCallbackGroup(),
        node->get_node_base_interface());

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 4);

    executor.add_node(node);

    std::thread height_thread([&]() {
        height_executor.spin();
    });

    executor.spin();

    height_executor.cancel();

    if (height_thread.joinable())
        height_thread.join();

    rclcpp::shutdown();

    return 0;
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    // rclcpp::shutdown();

    // return 0;
}
