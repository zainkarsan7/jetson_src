#pragma once
#include <cstdint>
#include <future>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "hb_robot_skills/motion/cut_graph_planner.hpp"
#include "hb_robot_skills/motion/cut_process_trajectory_generator.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include "hb_robot_skills/motion/cut_planner.hpp"
#include "hb_robot_skills/motion/torch_controller.hpp"
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_interfaces/msg/workpiece_reference.hpp"
#include "hb_robot_interfaces/srv/clear_torch_fault.hpp"
#include "hb_robot_interfaces/action/recover_cut.hpp"

#include "hb_robot_interfaces/srv/approve_motion.hpp"
#include "hb_robot_interfaces/action/cut_profile.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <std_srvs/srv/set_bool.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/action/escape.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <functional>

#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <mutex>
#include <optional>

namespace hb_robot_skills{
    using WorkpieceReference =
        hb_robot_interfaces::msg::WorkpieceReference;

    class CutProfileServer: public rclcpp::Node{
        public:
            using CutProfile = hb_robot_interfaces::action::CutProfile;
            using GoalHandleCutProfile = rclcpp_action::ServerGoalHandle<CutProfile>;

        
            explicit CutProfileServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            void initialize();
            enum class CutPlanningMode {
                Monolithic,
                Independent
            };

            enum class MotionOutcome{
                            SUCCESS,
                            REJECTED,
                            CANCELED,
                            FAILED
                        };

            MotionOutcome waitForPlanApproval(
                const std::shared_ptr<GoalHandleCutProfile> goal_handle
            );


            MotionOutcome executeMotion(
                const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                const std::shared_ptr<GoalHandleCutProfile> goal_handle,
                const std::string& description
            );

            MotionOutcome executeCut(
                const motion::ExecutableCut& cut,
                const std::shared_ptr<GoalHandleCutProfile> goal_handle
                );
            
            

            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CutProfile::Goal> goal);

            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleCutProfile>goal_handle);

            void requestCutStop();

            void handleAccepted(const std::shared_ptr<GoalHandleCutProfile> goal_handle);
            void execute(const std::shared_ptr<GoalHandleCutProfile> goal_handle);

            CutProfileServer::MotionOutcome executeCuttingPlan(const motion::ExecutablePlan& executable_plan_,
                const std::shared_ptr<GoalHandleCutProfile>goal_handle);

            void abortGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            void cancelGoal(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& message);

            void profileEstimateCallback(const hb_robot_interfaces::msg::ProfileEstimate::SharedPtr msg);

            std::optional<hb_robot_interfaces::msg::ProfileEstimate> latestProfileEstimate() const;

            visualization_msgs::msg::MarkerArray makeVisualization(const motion::CutPlan& plan, const std::string& frame_id)const;
            
            void publishVisualization(const motion::CutPlan& plan, const std::string& frame_id);
            void clearCollisionDebug();
            void publishCollisionDebug(const motion::CollisionDebugEvent& event);

            void publishFeedback(const std::shared_ptr<GoalHandleCutProfile>& goal_handle,
            const std::string& stage);

            std::optional<WorkpieceReference> latestWorkpieceReference() const;
            

            void handleApproval(const std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Request> request,
            std::shared_ptr<hb_robot_interfaces::srv::ApproveMotion::Response>response);
            
            void handleTorchFault(
                const std::shared_ptr<
                    hb_robot_interfaces::srv::ClearTorchFault::Request> request,
                std::shared_ptr<
                    hb_robot_interfaces::srv::ClearTorchFault::Response> response);
        private:

                rclcpp::CallbackGroup::SharedPtr height_status_callbacks_;

// Protected by height_mutex_.
uint64_t height_status_rx_count_{0};
double height_status_max_gap_ms_{0.0};
            std::shared_ptr<robot_trajectory::RobotTrajectory> planToTrajectory(
            const moveit::planning_interface::MoveGroupInterface::Plan& plan
            );
            std::unordered_map<std::string, motion::CutSegment> debug_segs_;
            std::mutex debug_segments_mutex;

            void publishDebugRobot(const moveit::core::RobotState& state);

bool latchObservedMotionStop(const char* reason);
            void publishCandidateVisualization(const motion::CutSegment segment);
            
            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planConstrainedCut(
                const motion::CutSegment& segment,
                double pos_tol,
                double ang_tol
            );

            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> makeLinPlan(
                const moveit::core::RobotState& start_state,
                const moveit::core::RobotState& goal_state,
                double tcp_velocity_m_s
            );

            std::shared_ptr<robot_trajectory::RobotTrajectory> collateSegmentTrajectory(
                const motion::SegmentMotionPlan& smp
            );

            moveit::core::RobotState getFinalState(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
            std::optional<motion::SegmentMotionPlan> planSegmentPilzLinear(
            const motion::CutSegment& segment, const moveit::core::RobotState& actual_approach
        );

        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planLinear(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& goal_state
        );


        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planToState(
            const moveit::core::RobotState& start_state,
            const moveit::core::RobotState& target_state
        );
        
        using TaskState = hb_robot::recovery::State;
        struct StoredPlan {
            motion::ExecutablePlan executable;
            hb_perception::ProfileModel geometry;
            TaskState versions;
            std::string cut_id, parent_id;
            std::vector<hb_robot::recovery::Interval> intervals;
            bool consumed=false;
        };
        struct TaskStateFreshness {
                TaskState state;
                bool received = false;
                bool fresh = false;
                double age_ms = -1.0;
            };
        std::unordered_map<std::string, StoredPlan> stored_plans_;
        std::unordered_map<std::string, std::vector<hb_robot::recovery::Interval>> attempted_;
        uint64_t next_plan_=0, execution_version_=0;
        uint32_t completed_segments_=0;
        std::string active_segment_;
        mutable std::mutex task_mutex_;
        TaskState task_state_, execution_state_;
        std::chrono::steady_clock::time_point task_received_{};
        std::atomic<bool> motion_active_{false}, retreat_active_{false}, busy_{false};
        bool process_enabled_=false;
        rclcpp::Subscription<TaskState>::SharedPtr task_sub_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr execution_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr invalidate_pose_pub_;
        rclcpp::TimerBase::SharedPtr supervision_timer_;
        rclcpp_action::Server<hb_robot_interfaces::action::Escape>::SharedPtr escape_server_;
        std::thread worker_, escape_worker_;
        std::atomic<bool> escape_busy_{false};
        TaskStateFreshness taskStateFreshness() const;

        bool readFreshTaskState(TaskState& snapshot) const;
        bool taskPermitsExecution(TaskState* snapshot=nullptr) const;
        void executeStored(const std::string& id,
            const std::shared_ptr<GoalHandleCutProfile>& handle);
        void escape(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            hb_robot_interfaces::action::Escape>>& handle);
        public:
        ~CutProfileServer() override;
        private:
        std::string planning_group_;
        std::string plasma_link_;
        double planning_time_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr approval_service_;


       

        rclcpp::Service<
            hb_robot_interfaces::srv::ClearTorchFault>::SharedPtr
            clear_torch_fault_service_;


        mutable std::mutex approval_mutex_;
        std::condition_variable approval_cv_;
        std::mutex execution_mutex_;
        bool require_plan_approval_{true};
        bool skip_motion_{true};
        bool motion_approved_{false};

        bool motion_rejected_{false};

        
        CutPlanningMode cut_planning_mode_ =
        CutPlanningMode::Monolithic;

        uint64_t next_batch_ = 0;


        void releaseGoalAdmission();

        std::mutex worker_mutex_;

        
        double transit_velocity_scaling_ = 0.05;
        double transit_acceleration_scaling_ = 0.01;

        std::atomic<bool> cut_stop_requested_{false};
        bool setFlapClosed(bool closed);
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr flap_client_;
        std::chrono::milliseconds flap_travel_time_{500};
        std::unique_ptr<TorchController> torch_controller_;
        int task_state_timeout_ms_ = 1000;

        

        rclcpp::Subscription<WorkpieceReference>::SharedPtr
            workpiece_reference_sub_;

        mutable std::mutex reference_mutex_;

        std::optional<WorkpieceReference>
            latest_reference_;

        hb_robot_interfaces::msg::ProfileEstimate latest_estimate_;
        std::unique_ptr<motion::CutPlanner> cut_planner_;

        //////////////////////////////////////////////////////
        using RecoverCut =
            hb_robot_interfaces::action::RecoverCut;

        using GoalHandleRecoverCut =
            rclcpp_action::ServerGoalHandle<RecoverCut>;

        rclcpp_action::Server<RecoverCut>::SharedPtr
            recover_cut_server_;

        std::thread recovery_worker_;

        std::atomic<bool> recovery_busy_{false};

        rclcpp_action::GoalResponse handleRecoveryGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const RecoverCut::Goal> goal);

        rclcpp_action::CancelResponse handleRecoveryCancel(
            const std::shared_ptr<GoalHandleRecoverCut> goal);

        void handleRecoveryAccepted(
            const std::shared_ptr<GoalHandleRecoverCut> goal);

        void recoverCut(
            const std::shared_ptr<GoalHandleRecoverCut> goal);
            
        bool recoveryRobotStationary(
            const moveit::core::JointModelGroup* jmg);
        MotionOutcome waitForRecoveryApproval(
            const std::shared_ptr<GoalHandleRecoverCut>& goal);
        MotionOutcome executeRecoveryMotion(
            const moveit::planning_interface::MoveGroupInterface::Plan& plan,
            const std::shared_ptr<GoalHandleRecoverCut>& goal,
            const std::string& phase);
        std::atomic<bool> recovery_motion_active_{false};
            void requestRecoveryStop();
            std::mutex stop_mutex_;
            std::atomic<uint64_t> stop_generation_{0};

            bool armApprovedRecoveryMotion(
                uint64_t expected_generation,
                const std::shared_ptr<GoalHandleRecoverCut>& goal);
            void finishRecoveryMotion();
        bool latchMotionStop(const char* reason);
        ///////////////////////////////////////////////////////////
        std::unique_ptr<motion::CutProcessTrajectoryGenerator> process_generator_;
        std::unique_ptr<motion::CutGraphPlanner> cut_graph_planner_;
        rclcpp_action::Server<CutProfile>::SharedPtr action_server_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
        mutable std::mutex estimate_mutex_;
        rclcpp::Subscription<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr profile_estimate_sub_;
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_ ;
        rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr debug_robot_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            collision_debug_pub_;



        ///////////////////////////////////////////////////////
using HeightClock = std::chrono::steady_clock;
struct HeightStatus {
    std::string state = "OFFLINE";
    int32_t position = 0;
    bool switch_active = false;
    int fault = 2;
    bool referenced = false;
    uint32_t session = 0, active_id = 0, done_id = 0;
    int done_result = 0;
    bool calibration = true, configured = false, tracking = false;
    HeightClock::time_point received{};
};
enum class HeightPhase { IDLE, PREP, IGNITION, SEARCH, CUT, RETURN };
mutable std::mutex height_mutex_;
std::condition_variable height_cv_;
HeightStatus height_status_;
rclcpp::CallbackGroup::SharedPtr height_callbacks_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr height_full_status_sub_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr height_prepare_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr height_search_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr height_home_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr height_stop_client_;
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr height_tracking_client_;
std::atomic<HeightPhase> height_phase_{HeightPhase::IDLE};
std::atomic<uint32_t> height_cycle_session_{0};
std::atomic<bool> height_stop_sent_{false}, height_tracking_expected_{false};
bool height_use_tracking_ = false;
int height_status_timeout_ms_ = 250;
int height_prepare_timeout_ms_ = 5000;
int height_search_timeout_ms_ = 8000;
int height_home_timeout_ms_ = 20000;
int height_pierce_dwell_ms_ = 0;
void initializeHeightInterface();
HeightStatus heightSnapshot() const;
bool heightValid(const HeightStatus& h) const;
bool heightAtHomeAndReady() const;
bool requestHeightCommand(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
    const std::function<bool()>& canceled, uint32_t& command_id);
bool waitHeightCompletion(uint32_t session, uint32_t command_id,
    const std::string& expected_state, std::chrono::milliseconds timeout,
    const std::function<bool()>& canceled);
bool prepareHeightAndWaitReady(const std::function<bool()>& canceled);
bool searchHeightAndWaitForArc(const std::function<bool()>& canceled);
bool setHeightTrackingAndWait(bool enabled);
bool enableHeightTracking();
bool disableHeightTracking();
bool returnHeightHomeAndWait(const std::function<bool()>& canceled);
void stopHeightAsync();
void superviseHeight();

        bool approval_pending_ = false;  // guarded by approval_mutex_
        std::string approval_operation_; // guarded by approval_mutex_


        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
        motion::CutProcessParams cut_process_params_;

    };



}