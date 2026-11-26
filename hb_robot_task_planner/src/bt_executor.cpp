#include <hb_robot_task_planner/for_each_pending.hpp>
#include <hb_robot_interfaces/msg/task_session_status.hpp>
#include <hb_robot_interfaces/supervision_policy.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <set>
#include <hb_robot_interfaces/msg/cut_feature_plan.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/srv/capture_cut_environment.hpp>
#include <hb_robot_interfaces/action/inspect_profile.hpp>
#include <hb_robot_interfaces/action/inspect_cut.hpp>
#include <hb_robot_interfaces/action/cut_profile.hpp>
#include <hb_robot_interfaces/action/revalidate_workpiece.hpp>
#include <hb_robot_interfaces/action/escape.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/msg/platform_supervision.hpp>
#include "hb_robot_interfaces/action/recover_cut.hpp"
#include <type_traits>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <filesystem>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace recovery = hb_robot::recovery;

using Inspect = hb_robot_interfaces::action::InspectProfile;
using Kerf = hb_robot_interfaces::action::InspectCut;
using Cut = hb_robot_interfaces::action::CutProfile;
using Revalidate = hb_robot_interfaces::action::RevalidateWorkpiece;
using Escape = hb_robot_interfaces::action::Escape;
using Supervision = hb_robot_interfaces::msg::PlatformSupervision;

using RecoverCut = hb_robot_interfaces::action::RecoverCut;
using Clock = std::chrono::steady_clock;

struct Context {
  std::mutex mutex;
  const int max_age_ms_ = 750;
  recovery::State state;
  recovery::State requested_plan_state;
  hb_robot_interfaces::msg::ProfileEstimate estimate;
  recovery::Evidence evidence;

  Clock::time_point received{};
  Clock::time_point supervision_received{};

  Supervision supervision;
  bool supervision_received_any = false;

  std::string cut_id;
  std::string plan_id;

  struct FeaturePlanEntry {
      std::string feature_id;
      std::string plan_id;

      enum class Status {
          Ready,
          Unplannable,
          Selected,
          Completed,
          Interrupted,
          RecoveredPendingInspection,
          ExecutionFailed,
          Rejected,
          Deferred
      };

      Status status = Status::Unplannable;
      std::string message;
  };

  std::string batch_id;
  std::vector<FeaturePlanEntry> feature_plans;
  bool independent_batch = false;
  bool batch_interrupted = false;

  std::string interrupted_plan_id;
    std::string interrupted_feature_id;
    bool recovery_home_reached = false;
    bool recovery_admitted = false; // Set only by the contained session command.
    bool recovery_goal_sent = false;



  std::map<std::string, recovery::FeatureRecoveryState> feature_recovery;
  std::string recovery_feature;
  std::string parent_id;
  std::string inspection_reference_id;

  std::vector<std::string> required_segments;
  recovery::Verdict verdict;

  uint64_t execution_version = 0;
  unsigned inspection_attempts = 0;

  std::optional<recovery::State> captured_scene;

  std::string last_action;
  std::string last_error;
  std::string first_action_timeout; // Tick-thread only; retained until explicit resume.
  double planning_action_timeout_s = 600.0;
  bool execution_uncertain = false;
  std::vector<std::function<bool()>> cancellation_waiters;
  bool executed = false;
  bool preparing_candidate = false;

  recovery::State snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    auto result = state;

    if (Clock::now() - received > std::chrono::milliseconds(max_age_ms_)) {
      result.supervision_valid = false;
      result.imu_stable = false;
      // A transport gap revokes authorization, not the cached geometry.
    }

    return result;
  }

  bool platformStable() {
    std::lock_guard<std::mutex> lock(mutex);

    if (!supervision_received_any ||
        Clock::now() - supervision_received > std::chrono::milliseconds(max_age_ms_))
      return false;

    hb_robot::supervision::Evidence e;
    e.fresh = true;
    e.authorized = supervision.supervision_valid && supervision.imu_stable;
    e.uncertain = supervision.state == Supervision::UNCERTAIN;
    e.hard_motion = supervision.hard_motion_detected;
    return hb_robot::supervision::decide(e, hb_robot::supervision::Operation::AcquirePose) ==
        hb_robot::supervision::Decision::Allow;
  }

  std::string platformReason() {
    std::lock_guard<std::mutex> lock(mutex);

    if (!supervision_received_any ||
        Clock::now() - supervision_received > std::chrono::milliseconds(max_age_ms_))
      return "Platform supervision heartbeat stale";

    return supervision.reason;
  }

  bool authorized(const recovery::State& s) {
    return platformStable() &&
           recovery::geometryPermitted(s);
  }

  


  bool planMatches(const recovery::State& s) const {
    return recovery::sameVersions(s, requested_plan_state);
  }

  // void discardCandidate() {
  //   plan_id.clear();
  //   captured_scene.reset();
  //   preparing_candidate = false;
  // }
  void discardCandidate() {
    plan_id.clear();
    batch_id.clear();
    feature_plans.clear();
    independent_batch = false;
    batch_interrupted = false;
    captured_scene.reset();
    preparing_candidate = false;
}

void clearSelectedFeature() {
    plan_id.clear();
    preparing_candidate = false;
}

};
///////////////////////////////////////////////////////////////////////
// Generic asynchronous ROS action adapter.
// Action callbacks never mutate BT-owned state.
// Result handlers run on the BT tick thread.
template<class Action>
class ActionNode : public BT::StatefulActionNode {
  public:
    using Handle = rclcpp_action::ClientGoalHandle<Action>;
    using GoalBuilder =
        std::function<bool(typename Action::Goal&)>;
    using ResultHandler =
        std::function<bool(const typename Action::Result&)>;

    ActionNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        const std::string& endpoint,
        GoalBuilder builder,
        ResultHandler handler, std::shared_ptr<Context> context)
        : BT::StatefulActionNode(name, config),
          context_(std::move(context)),
          build_(std::move(builder)),
          finish_(std::move(handler)) {
      client_ = rclcpp_action::create_client<Action>(
          node, endpoint);
    }

    static BT::PortsList providedPorts() {
      return {};
    }

    BT::NodeStatus onStart() override {
      cancel_ = std::make_shared<std::atomic<bool>>(false);
      handle_.reset();
      goal_ = {};
      result_ = {};
      sent_ = false;
      cancellation_registered_ = false;
      result_handled_ = false;
      started_ = Clock::now();
      context_->last_action = registrationName();
      context_->last_error.clear();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
      const auto elapsed = Clock::now() - started_;
      const bool planning = registrationName() == "PlanCut" || registrationName() == "PlanRecut";
      const auto deadline = std::chrono::duration<double>(
          planning ? context_->planning_action_timeout_s : 600.0);
      const bool acceptance_timeout = !handle_ && elapsed > 10s &&
          (!sent_ || !goal_.valid() || goal_.wait_for(0s) != std::future_status::ready);
      // Consume a ready terminal result before applying the local deadline.
      const bool result_ready = handle_ && result_.valid() &&
          result_.wait_for(0s) == std::future_status::ready;
      if (!result_ready && (elapsed > deadline || acceptance_timeout)) {
        context_->last_error = std::string(acceptance_timeout ?
            "Action acceptance deadline exceeded: " : "Action result deadline exceeded: ") + registrationName();
        if (context_->first_action_timeout.empty())
          context_->first_action_timeout = context_->last_error;
        RCLCPP_ERROR(rclcpp::get_logger("bt_executor"),
            "[ACTION TIMEOUT] node=%s elapsed_s=%.3f limit_s=%.3f "
            "sent=%d accepted=%d planning=%d; cancel requested",
            registrationName().c_str(), std::chrono::duration<double>(elapsed).count(),
            acceptance_timeout ? 10.0 : deadline.count(), int(sent_), int(bool(handle_)), int(planning));
        if constexpr (std::is_same_v<Action, Cut>)
          if (registrationName() == "ExecuteCut" || registrationName() == "ExecuteRecut")
            context_->execution_uncertain = true;
        onHalted();
        return BT::NodeStatus::FAILURE;
      }

      if (!sent_) {
        if (!client_->action_server_is_ready())
          return BT::NodeStatus::RUNNING;

        typename Action::Goal goal;

        if (!build_(goal)){
          context_->last_error = registrationName() + ": goal preconditions not satisfied";

          RCLCPP_ERROR(
          rclcpp::get_logger("bt_executor"),
          "Action '%s': goal construction rejected",
          registrationName().c_str());
          return BT::NodeStatus::FAILURE;
          }

        typename rclcpp_action::Client<Action>::
            SendGoalOptions options;

        auto cancellation = cancel_;
        auto client = client_;

        options.goal_response_callback =
            [client, cancellation](
                typename Handle::SharedPtr handle) {
          if (handle && cancellation->load())
            client->async_cancel_goal(handle);
        };

        goal_ = client_->async_send_goal(goal, options);
        sent_ = true;
      }

      if (!handle_) {
        if (goal_.wait_for(0s) !=
            std::future_status::ready)
          return BT::NodeStatus::RUNNING;

        handle_ = goal_.get();

        if (!handle_){
          context_->last_error = registrationName() + ": action server rejected the goal";

        RCLCPP_ERROR(
          rclcpp::get_logger("bt_executor"),
          "Action '%s': goal REJECTED by ROS action server",
          registrationName().c_str());


          return BT::NodeStatus::FAILURE;
        }
        result_ = client_->async_get_result(handle_);
      }
      

      if (result_.wait_for(0s) !=
          std::future_status::ready)
        return BT::NodeStatus::RUNNING;

      const auto wrapped = result_.get();
      result_handled_ = true;

      if (!wrapped.result) {
          if constexpr (std::is_same_v<Action, Cut>)
            if (registrationName()=="ExecuteCut" || registrationName()=="ExecuteRecut") {
              context_->execution_uncertain=true;
              if (context_->independent_batch) context_->batch_interrupted=true;
            }
          RCLCPP_ERROR(
              rclcpp::get_logger("bt_executor"),
              "Action '%s' returned no result; code=%d",
              registrationName().c_str(),
              static_cast<int>(wrapped.code));
          return BT::NodeStatus::FAILURE;
      }

      // Let the action-specific handler inspect the payload,
      // including diagnostic fields on an aborted result.
      const bool accepted = finish_(*wrapped.result);
      if ((!accepted || wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) && context_->last_error.empty())
        context_->last_error = registrationName() + ": " + wrapped.result->message;

      // An operator-rejected cut is a successfully handled
        // task decision, even though the ROS action was aborted.
        if constexpr (std::is_same_v<Action, Cut>) {
            if (wrapped.result->approval_rejected) {
                return accepted
                    ? BT::NodeStatus::SUCCESS
                    : BT::NodeStatus::FAILURE;
            }
        }


      if (wrapped.code !=
              rclcpp_action::ResultCode::SUCCEEDED ||
          !accepted) {
          RCLCPP_ERROR(
              rclcpp::get_logger("bt_executor"),
              "Action '%s' failed: code=%d message='%s'",
              registrationName().c_str(),
              static_cast<int>(wrapped.code),
              wrapped.result->message.c_str());

          return BT::NodeStatus::FAILURE;
      }

      return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
      if (sent_ && !result_handled_ && !cancellation_registered_) {
        cancellation_registered_ = true;
        auto client = client_;
        auto goal_future = goal_;
        auto result_future = result_;
        auto handle = handle_;
        auto finish = finish_;
        const bool execution = registrationName()=="ExecuteCut" || registrationName()=="ExecuteRecut";
        if (execution) {
          context_->execution_uncertain=true;
          if (context_->independent_batch) context_->batch_interrupted=true;
        }
        context_->cancellation_waiters.push_back(
            [client, goal_future, result_future, handle, finish, execution]() mutable {
              if (!handle) {
                if (!goal_future.valid() || goal_future.wait_for(0s) != std::future_status::ready) return false;
                handle = goal_future.get();
                if (!handle) return true; // Rejected goal cannot execute.
              }
              if (!result_future.valid()) result_future = client->async_get_result(handle);
              if (result_future.wait_for(0s) != std::future_status::ready) return false;
              // Tick-thread drain: retain interrupted motion lineage even when
              // the action was halted before its terminal payload arrived.
              if constexpr (std::is_same_v<Action, Cut>) {
                if (execution) {
                  const auto wrapped = result_future.get();
                  if (wrapped.result) (void)finish(*wrapped.result);
                }
              }
              return true;
            });
      }
      if (cancel_)
        cancel_->store(true);

      if (handle_) {
        client_->async_cancel_goal(handle_);
      } else if (sent_ && goal_.valid() &&
                goal_.wait_for(0s) ==
                    std::future_status::ready) {
        auto handle = goal_.get();
        if (handle)
          client_->async_cancel_goal(handle);
      }
    }

  private:
    std::shared_ptr<Context> context_;
    GoalBuilder build_;
    ResultHandler finish_;

    typename rclcpp_action::Client<Action>::SharedPtr client_;
    typename Handle::SharedPtr handle_;

    std::shared_future<typename Handle::SharedPtr> goal_;
    std::shared_future<typename Handle::WrappedResult> result_;

    std::shared_ptr<std::atomic<bool>> cancel_;

    bool sent_ = false;
    bool cancellation_registered_ = false;
    bool result_handled_ = false;
    Clock::time_point started_{};
  };

class WaitForStablePlatform : public BT::StatefulActionNode {
    public:
      WaitForStablePlatform(
          const std::string& name,
          const BT::NodeConfiguration& config,
          std::shared_ptr<Context> context)
          : BT::StatefulActionNode(name, config),
            context_(std::move(context)) {}

      static BT::PortsList providedPorts() {
        return {};
      }

      BT::NodeStatus onStart() override {
        context_->last_action = registrationName();
        context_->last_error.clear();
        started_ = Clock::now();
        stable_since_.reset();
        return onRunning();
      }

      BT::NodeStatus onRunning() override {
        const auto state = context_->snapshot();

        if (state.motion_fault_active ||
            state.process_fault_active) {
          context_->last_error = "Platform wait stopped: active motion/process fault";
          RCLCPP_ERROR(
              rclcpp::get_logger("bt_executor"),
              "Platform wait stopped: motion/process fault");
          return BT::NodeStatus::FAILURE;
        }

        if (state.scene_valid &&
            state.supervision_valid &&
            state.imu_stable &&
            context_->platformStable()) {
          if (!stable_since_)
            stable_since_ = Clock::now();

          if (Clock::now() - *stable_since_ >= 1s)
            return BT::NodeStatus::SUCCESS;
        } else {
          stable_since_.reset();
        }

        if (Clock::now() - started_ >= 30s) {
          context_->last_error = "Platform wait expired: " + context_->platformReason();
          RCLCPP_WARN(
              rclcpp::get_logger("bt_executor"),
              "Platform stability wait timed out: %s",
              context_->platformReason().c_str());
          return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
      }

      void onHalted() override {
        stable_since_.reset();
      }

    private:
      std::shared_ptr<Context> context_;
      Clock::time_point started_{};
      std::optional<Clock::time_point> stable_since_;
    };







class AwaitGeometry : public BT::StatefulActionNode {
  public:
    AwaitGeometry(
        const std::string& name,
        const BT::NodeConfiguration& config,
        std::shared_ptr<Context> context,
        bool planning = false)
        : BT::StatefulActionNode(name, config),
          context_(context),
          planning_(planning) {}

    static BT::PortsList providedPorts() {
      return {};
    }

    BT::NodeStatus onStart() override {
      context_->last_action = registrationName();
      context_->last_error.clear();
      start_ = Clock::now();
      return onRunning();
    }

    BT::NodeStatus onRunning() override {
      const auto state = context_->snapshot();

      const bool reference_matches =
          state.scene_valid &&
          state.scene_id ==
              context_->inspection_reference_id;

      if (planning_) {
        if (reference_matches)
          return BT::NodeStatus::SUCCESS;
      } else {
        if (reference_matches &&
            context_->authorized(state))
          return BT::NodeStatus::SUCCESS;
      }

      if (Clock::now() - start_ <= 15s)
        return BT::NodeStatus::RUNNING;

      context_->last_error = registrationName() + ": matching geometry/authorization was not committed before the deadline";

      RCLCPP_ERROR(
          rclcpp::get_logger("bt_executor"),
          "AwaitGeometry timeout: planning=%d "
          "reference_matches=%d pose=%d "
          "supervision=%d imu=%d platform=%d "
          "expected_reference=%s actual_reference=%s "
          "reason=%s",
          planning_,
          reference_matches,
          state.workpiece_pose_valid,
          state.supervision_valid,
          state.imu_stable,
          context_->platformStable(),
          context_->inspection_reference_id.c_str(),
          state.scene_id.c_str(),
          state.status_message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    void onHalted() override {}

  private:
    std::shared_ptr<Context> context_;
    bool planning_;
    Clock::time_point start_{};
  };


class AwaitExecutionReady : public BT::StatefulActionNode {
public:
  AwaitExecutionReady(const std::string& name, const BT::NodeConfiguration& config,
                      std::shared_ptr<Context> context)
      : BT::StatefulActionNode(name, config), context_(std::move(context)) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { started_ = Clock::now(); return onRunning(); }
  BT::NodeStatus onRunning() override {
    context_->last_action = "AwaitExecutionReady";
    const auto state = context_->snapshot();
    if (!context_->planMatches(state) || !context_->captured_scene ||
        context_->captured_scene->cut_environment_id != state.cut_environment_id ||
        context_->plan_id.empty()) {
      context_->last_error = "Candidate geometry/scene changed; replan required";
      return BT::NodeStatus::FAILURE;
    }
    if (state.motion_fault_active || state.process_fault_active || !state.workpiece_pose_valid) {
      context_->last_error = "Fault clearance or workpiece revalidation required";
      return BT::NodeStatus::FAILURE;
    }
    if (context_->authorized(state))
      return BT::NodeStatus::SUCCESS;
    if (Clock::now() - started_ >= 30s) {
      context_->last_error = "Readiness wait expired; candidate retained";
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  std::shared_ptr<Context> context_;
  Clock::time_point started_;
};

class AwaitTorchFaultClear : public BT::StatefulActionNode {
  using Service = std_srvs::srv::Trigger;
public:
  AwaitTorchFaultClear(const std::string& name, const BT::NodeConfiguration& config,
                      rclcpp::Node::SharedPtr node, std::shared_ptr<Context> context)
      : BT::StatefulActionNode(name,config), context_(std::move(context)),
        client_(node->create_client<Service>("/cut_profile/recovery_ready")) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    context_->last_action=registrationName();
    next_poll_=Clock::time_point{};
    return check();
  }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {
    if (pending_.valid()) client_->remove_pending_request(request_id_);
    pending_={};
  }
private:
  BT::NodeStatus check() {
    const auto state=context_->snapshot();
    if (!context_->recovery_admitted || !context_->independent_batch ||
        !context_->batch_interrupted || context_->interrupted_plan_id.empty() ||
        context_->interrupted_feature_id.empty()) {
      context_->last_error="No contained interrupted feature admitted for recovery";
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
    if (!context_->captured_scene ||
        !recovery::sameVersions(state,*context_->captured_scene) ||
        state.cut_environment_id!=context_->captured_scene->cut_environment_id) {
      context_->last_error="Recovery requires renewed geometry validation";
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
    if (state.process_fault_active || state.motion_fault_active ||
        !state.scene_valid || !state.workpiece_pose_valid || !context_->authorized(state)) {
      onHalted();
      context_->last_error="Waiting for operator fault clearance and live geometry authorization";
      return BT::NodeStatus::RUNNING;
    }
    // The ledger establishes the interruption. Clearance may predate this node.
    // Server-side query reads the actual height/torch predicates; no duplicate
    // height parser or persisted permission is introduced in the BT.
    if (pending_.valid()) {
      if (pending_.wait_for(0s)==std::future_status::ready) {
        const auto reply=pending_.get();
        pending_={};
        context_->last_error=reply->success?"":reply->message;
        next_poll_=Clock::now()+250ms;
        if (reply->success) return BT::NodeStatus::SUCCESS;
      } else if (Clock::now()-sent_at_>2s) {
        onHalted();
        context_->last_error="Recovery readiness reply timed out";
        next_poll_=Clock::now()+250ms;
      }
    }
    if (!pending_.valid() && Clock::now()>=next_poll_) {
      if (!client_->service_is_ready()) {
        context_->last_error="Waiting for /cut_profile/recovery_ready";
        return BT::NodeStatus::RUNNING;
      }
      sent_at_=Clock::now();
      auto pending=client_->async_send_request(std::make_shared<Service::Request>());
      request_id_=pending.request_id;
      pending_=pending.share();
    }
    return BT::NodeStatus::RUNNING;
  }
  std::shared_ptr<Context> context_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Client<Service>::SharedFuture pending_;
  int64_t request_id_=0;
  Clock::time_point sent_at_{}, next_poll_{};
};

  class CaptureEnvironment : public BT::StatefulActionNode {
  using Service =
      hb_robot_interfaces::srv::CaptureCutEnvironment;

    public:
      CaptureEnvironment(
          const std::string& name,
          const BT::NodeConfiguration& config,
          rclcpp::Node::SharedPtr node,
          std::shared_ptr<Context> context)
          : BT::StatefulActionNode(name, config),
            context_(context),
            client_(node->create_client<Service>(
                "capture_cut_environment")) {}

      static BT::PortsList providedPorts() {
        return {};
      }

      BT::NodeStatus onStart() override {
        context_->last_action = registrationName();
        context_->last_error.clear();
        scene_retries_ = 0;
        context_->captured_scene.reset();

        expected_ = context_->snapshot();
        deadline_ = Clock::now() + 30s;

        last_scene_version_ = expected_.scene_version;
        last_environment_id_.clear();
        settled_since_.reset();

        return onRunning();
      }

      BT::NodeStatus onRunning() override {
        const auto state = context_->snapshot();

        if (!state.scene_valid ||
            state.motion_fault_active ||
            state.process_fault_active ||
            state.geometry_version !=
                expected_.geometry_version ||
            state.pose_version !=
                expected_.pose_version ||
            Clock::now() > deadline_) {
                context_->last_error = "CaptureCutEnvironment: live state changed or capture transaction timed out";
                RCLCPP_ERROR(rclcpp::get_logger("bt_executor"),
                    "[CAPTURE INVALID] transaction_timeout=%d scene_valid=%d motion_fault=%d "
                    "process_fault=%d geometry=%llu/%llu pose=%llu/%llu active_node=%s",
                    int(Clock::now() > deadline_), int(state.scene_valid), int(state.motion_fault_active),
                    int(state.process_fault_active),
                    static_cast<unsigned long long>(state.geometry_version),
                    static_cast<unsigned long long>(expected_.geometry_version),
                    static_cast<unsigned long long>(state.pose_version),
                    static_cast<unsigned long long>(expected_.pose_version), context_->last_action.c_str());
                onHalted();
                return BT::NodeStatus::FAILURE;
              }

        if (!response_ && !future_.valid()) {
          if (!client_->service_is_ready())
            return BT::NodeStatus::RUNNING;
          // Refresh the complete TaskState snapshot before
          // creating the capture request. The scene monitor
          // may have processed the commit's final update
          // after the inspection action returned.
          expected_ = state;

          auto request =
              std::make_shared<Service::Request>();

          request->geometry_version =
              expected_.geometry_version;
          request->pose_version =
              expected_.pose_version;
          request->scene_version =
              expected_.scene_version;

          auto pending = client_->async_send_request(request);
          request_id_ = pending.request_id;
          future_ = pending.share();
          return BT::NodeStatus::RUNNING;
        }

        if (!response_) {
          if (future_.wait_for(0s) !=
              std::future_status::ready)
            return BT::NodeStatus::RUNNING;

          response_ = future_.get();
          future_ = {};

          if (!response_->success) {

            // Re-read TaskState at the point of failure.
            // The scene may have advanced while the
            // capture service was validating the request.
            const auto current = context_->snapshot();

            constexpr unsigned max_scene_retries = 2;
            const bool same_reference =
                            current.scene_valid &&
                            current.geometry_version ==
                                expected_.geometry_version &&
                            current.pose_version ==
                                expected_.pose_version;

                        const bool supervision_ok =
                            context_->authorized(current) &&
                            !current.motion_fault_active &&
                            !current.process_fault_active;

                        const bool scene_advanced =
                            current.scene_version >
                                expected_.scene_version;

                        if (scene_retries_ < max_scene_retries &&
                            same_reference &&
                            supervision_ok &&
                            scene_advanced &&
                            Clock::now() < deadline_)
                        {
                            ++scene_retries_;

                            RCLCPP_WARN(
                                rclcpp::get_logger("bt_executor"),
                                "Capture scene revision changed "
                                "%llu -> %llu; retry %u/%u",
                                static_cast<unsigned long long>(
                                    expected_.scene_version),
                                static_cast<unsigned long long>(
                                    current.scene_version),
                                scene_retries_,
                                max_scene_retries);

                            response_.reset();
                            settled_since_.reset();

                            // Next tick sends a new request using
                            // a fresh TaskState snapshot.
                            return BT::NodeStatus::RUNNING;
                        }

                        RCLCPP_ERROR(
                            rclcpp::get_logger("bt_executor"),
                            "Environment capture failed: %s",
                            response_->message.c_str());

                        response_.reset();
                        return BT::NodeStatus::FAILURE;
                    }
                }

        const bool acknowledged =
            !response_->environment_id.empty() &&
            state.cut_environment_id ==
                response_->environment_id &&
            (!response_->scene_changed ||
            state.scene_version >
                expected_.scene_version);

        if (!acknowledged) {
          settled_since_.reset();
          return BT::NodeStatus::RUNNING;
        }

        // Diagnostic settling interval only.
        // This does NOT replace a completion barrier in
        // the CaptureCutEnvironment service implementation.
        if (state.scene_version != last_scene_version_ ||
            state.cut_environment_id !=
                last_environment_id_) {
          last_scene_version_ = state.scene_version;
          last_environment_id_ = state.cut_environment_id;
          settled_since_ = Clock::now();
          return BT::NodeStatus::RUNNING;
        }

        if (!settled_since_) {
          settled_since_ = Clock::now();
          return BT::NodeStatus::RUNNING;
        }

        if (Clock::now() - *settled_since_ < 500ms)
          return BT::NodeStatus::RUNNING;

        context_->captured_scene = state;

        RCLCPP_INFO(
            rclcpp::get_logger("bt_executor"),
            "Capture complete: geometry=%llu pose=%llu "
            "scene=%llu environment=%s",
            static_cast<unsigned long long>(
                state.geometry_version),
            static_cast<unsigned long long>(
                state.pose_version),
            static_cast<unsigned long long>(
                state.scene_version),
            state.cut_environment_id.c_str());

        response_.reset();
        return BT::NodeStatus::SUCCESS;
      }

      void onHalted() override {
        if (future_.valid())
          client_->remove_pending_request(request_id_);

        future_ = {};
        response_.reset();
        settled_since_.reset();
      }

    private:
      std::shared_ptr<Context> context_;
      rclcpp::Client<Service>::SharedPtr client_;

      rclcpp::Client<Service>::SharedFuture future_;
      Service::Response::SharedPtr response_;

      int64_t request_id_ = 0;
      recovery::State expected_;

      Clock::time_point deadline_{};
      std::optional<Clock::time_point> settled_since_;
      unsigned scene_retries_ = 0;
      uint64_t last_scene_version_ = 0;
      std::string last_environment_id_;
    };

template<class Action, class Goal, class Result>
void registerAction(
    BT::BehaviorTreeFactory& factory,
    const std::string& name,
    rclcpp::Node::SharedPtr node,
    const std::string& endpoint,
    Goal goal,
    Result result, std::shared_ptr<Context> context) {

  factory.registerBuilder<ActionNode<Action>>(
      name,
      [=](const auto& n, const auto& c) {
        return std::make_unique<ActionNode<Action>>(
            n, c, node, endpoint, goal, result, context);
      });
}






#include <hb_robot_task_planner/independent_recovery.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<rclcpp::Node>("hb_bt_executor");

  auto context = std::make_shared<Context>();
  context->planning_action_timeout_s = node->declare_parameter<double>(
      "planning_action_timeout_s", 600.0);
  if (!std::isfinite(context->planning_action_timeout_s) || context->planning_action_timeout_s <= 0.0) {
    RCLCPP_FATAL(node->get_logger(), "planning_action_timeout_s must be finite and positive");
    rclcpp::shutdown();
    return 1;
  }

  context->cut_id =
      node->declare_parameter<std::string>(
          "cut_id",
          "cut_" +
              std::to_string(node->now().nanoseconds()));

  const double standoff =
      node->declare_parameter("standoff", 0.01);
  const double pos_tol =
      node->declare_parameter("pos_tol", 0.01);
  const double ang_tol =
      node->declare_parameter("ang_tol", 0.3);


  auto state_sub =
      node->create_subscription<recovery::State>(
          "/task_state",
          rclcpp::QoS(1).transient_local(),
          [context, node](recovery::State::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            const double age = (node->now() - rclcpp::Time(m->header.stamp)).seconds();
            if (age < -0.005 || age > 5.0) return;
            context->state = *m;
            context->received = Clock::now() - std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(std::max(0.0, age)));
          });

  auto supervision_sub =
      node->create_subscription<Supervision>(
          "/platform_supervision",
          rclcpp::QoS(1).reliable(),
          [context, node](Supervision::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            const double age = (node->now() - rclcpp::Time(m->header.stamp)).seconds();
            if (age < -0.005 || age > 5.0) return;
            context->supervision = *m;
            context->supervision_received = Clock::now() - std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(std::max(0.0, age)));
            context->supervision_received_any = true;
          });

  auto reference_sub =
      node->create_subscription<
          hb_robot_interfaces::msg::WorkpieceReference>(
          "/workpiece_reference/accepted",
          rclcpp::QoS(1).transient_local(),
          [context](
              hb_robot_interfaces::msg::
                  WorkpieceReference::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            context->estimate = m->estimate;
          });

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<AwaitExecutionReady>("AwaitExecutionReady",
      [context](const auto& name, const auto& config) {
        return std::make_unique<AwaitExecutionReady>(name, config, context);
      });
  factory.registerBuilder<hb_robot_task_planner::ForEachPending>("ForEachPending",
      [context](const auto& name, const auto& config) {
        return std::make_unique<hb_robot_task_planner::ForEachPending>(name, config, [context] {
          return !context->plan_id.empty() || std::any_of(context->feature_plans.begin(),
              context->feature_plans.end(), [](const auto& feature) {
                return feature.status == Context::FeaturePlanEntry::Status::Ready;
              });
        });
      });

  



  factory.registerBuilder<CaptureEnvironment>(
      "CaptureCutEnvironment",
      [=](const auto& n, const auto& c) {
        return std::make_unique<CaptureEnvironment>(
            n, c, node, context);
      });


  factory.registerBuilder<WaitForStablePlatform>(
    "WaitForStablePlatform",
    [=](const auto& n, const auto& c) {
      return std::make_unique<WaitForStablePlatform>(
          n, c, context);
    });

  auto condition =
      [&](const std::string& name, auto predicate) {
        factory.registerSimpleCondition(
            name,
            [predicate](BT::TreeNode&) {
              return predicate()
                  ? BT::NodeStatus::SUCCESS
                  : BT::NodeStatus::FAILURE;
            });
      };

  factory.registerSimpleAction(
    "SelectNextCutFeature",
    [=](BT::TreeNode&) {
        if (!context->independent_batch)
            return BT::NodeStatus::FAILURE;

        if (context->batch_interrupted ||
            !context->plan_id.empty())
            return BT::NodeStatus::FAILURE;

        const auto it = std::find_if(
            context->feature_plans.begin(),
            context->feature_plans.end(),
            [](const auto& feature) {
                return feature.status ==
                    Context::FeaturePlanEntry::Status::Ready;
            });

        if (it == context->feature_plans.end())
            return BT::NodeStatus::FAILURE;

        context->recovery_feature = it->feature_id;
        context->plan_id = it->plan_id;
        it->status =
            Context::FeaturePlanEntry::Status::Selected;

        RCLCPP_INFO(
            node->get_logger(),
            "[CUT SELECT] feature=%s plan=%s",
            it->feature_id.c_str(),
            it->plan_id.c_str());

        return BT::NodeStatus::SUCCESS;
    });
    condition("AllCutFeaturesResolved", [=] {
    using Status = Context::FeaturePlanEntry::Status;

    if (!context->independent_batch ||
        context->feature_plans.empty() ||
        context->batch_interrupted ||
        !context->plan_id.empty()) {
        return false;
    }

    return std::all_of(
        context->feature_plans.begin(),
        context->feature_plans.end(),
        [](const auto& feature) {
            return feature.status == Status::Completed ||
                   feature.status == Status::Rejected ||
                   feature.status == Status::Unplannable ||
                   feature.status ==
                       Status::RecoveredPendingInspection;
        });
    });
  
  condition("AllCutFeaturesCompleted", [=] {
    if (!context->independent_batch)
        return false;

    if (context->feature_plans.empty())
        return false;

    const bool completed = std::all_of(
        context->feature_plans.begin(),
        context->feature_plans.end(),
        [](const auto& feature) {
            return feature.status ==
                Context::FeaturePlanEntry::Status::Completed;
        });
    if (!completed) {
      context->last_action = "AllCutFeaturesCompleted";
      context->last_error = "Batch incomplete: review per-feature states; rejected or unplanned features are not completed cuts";
    }
    return completed;
  });



  condition("PendingCutFeatures", [=] {
    return context->independent_batch &&
        !context->batch_interrupted &&
        std::any_of(
            context->feature_plans.begin(),
            context->feature_plans.end(),
            [](const auto& feature) {
                return feature.status ==
                    Context::FeaturePlanEntry::Status::Ready;
            });
  });

  condition("ReferenceExists", [=] {
    std::lock_guard<std::mutex> lock(context->mutex);

    return Clock::now() - context->received <= std::chrono::milliseconds(context->max_age_ms_) &&
           context->state.scene_valid &&
           !context->estimate.profile_name.empty() &&
           context->state.scene_id ==
               std::to_string(
                   rclcpp::Time(
                       context->estimate.header.stamp)
                       .nanoseconds());
  });

  condition("WorkpiecePoseValid", [=] {
    const auto s = context->snapshot();
    return context->authorized(s);
  });

  condition("ValidateTaskState", [=] {
    const auto state = context->snapshot();
    const auto& planned =
        context->requested_plan_state;

    const bool platform_ok =
        context->platformStable();

    const bool task_ok =
        recovery::geometryPermitted(state);

    const bool versions_ok =
        recovery::sameVersions(state, planned);

    const bool environment_ok =
        context->captured_scene &&
        state.cut_environment_id ==
            context->captured_scene->cut_environment_id;

    // const bool candidate_ok =
    //     !context->plan_id.empty();
    const bool candidate_ok =
    !context->plan_id.empty() ||
    (context->independent_batch &&
     !context->batch_id.empty() &&
     !context->batch_interrupted &&
     std::any_of(
         context->feature_plans.begin(),
         context->feature_plans.end(),
         [](const auto& feature) {
             return feature.status ==
                 Context::FeaturePlanEntry::Status::Ready;
         }));

    if (platform_ok && task_ok &&
        versions_ok && environment_ok &&
        candidate_ok)
      return true;

    RCLCPP_WARN(
        node->get_logger(),
        "ValidateTaskState FAILED: "
        "platform=%d task=%d versions=%d "
        "environment=%d candidate=%d "
        "supervision=%d imu=%d pose=%d "
        "process_fault=%d motion_fault=%d "
        "geometry=%llu/%llu pose_version=%llu/%llu "
        "scene=%llu/%llu "
        "status=%s platform_reason=%s",
        platform_ok,
        task_ok,
        versions_ok,
        environment_ok,
        candidate_ok,
        state.supervision_valid,
        state.imu_stable,
        state.workpiece_pose_valid,
        state.process_fault_active,
        state.motion_fault_active,
        static_cast<unsigned long long>(
            state.geometry_version),
        static_cast<unsigned long long>(
            planned.geometry_version),
        static_cast<unsigned long long>(
            state.pose_version),
        static_cast<unsigned long long>(
            planned.pose_version),
        static_cast<unsigned long long>(
            state.scene_version),
        static_cast<unsigned long long>(
            planned.scene_version),
        state.status_message.c_str(),
        context->platformReason().c_str());

    return false;
  });

  condition("FaultActive", [=] {
    const auto s = context->snapshot();

    return s.process_fault_active ||
           s.motion_fault_active ||
           (context->executed &&
            !context->preparing_candidate &&
            !context->authorized(s));
  });

  factory.registerBuilder<AwaitGeometry>(
      "AwaitGeometry",
      [=](const auto& n, const auto& c) {
        return std::make_unique<AwaitGeometry>(
            n, c, context, false);
      });

  factory.registerBuilder<AwaitGeometry>(
      "AwaitPlanningReference",
      [=](const auto& n, const auto& c) {
        return std::make_unique<AwaitGeometry>(
            n, c, context, true);
      });

  registerAction<Inspect>(
      factory, "InspectProfile", node,
      "inspect_profile",
      [](auto& goal) {
        goal.max_views = 3;
        goal.samples_per_view = 3;
        goal.depth_range = 0.75;
        goal.camera_standoff = 0.4;
        return true;
      },
      [=](const auto& result) {
        if (!result.success)
          return false;

        context->inspection_reference_id =
            std::to_string(
                rclcpp::Time(
                    result.profile_pose.header.stamp)
                    .nanoseconds());

        return std::isfinite(result.rms_error) &&
               result.rms_error < 0.01 &&
               result.inlier_fraction > 0.3 &&
               result.coverage > 0.2;
      }, context);

  registerAction<Revalidate>(
      factory, "RevalidateWorkpiece", node,
      "revalidate_workpiece",
      [=](auto& goal) {
        goal.expected_pose_version =
            context->snapshot().pose_version;
        return true;
      },
      [=](const auto& result) {
        if (!result.localized ||
            result.poorly_constrained)
          return false;

        context->inspection_reference_id =
            std::to_string(
                rclcpp::Time(
                    result.estimate.header.stamp)
                    .nanoseconds());

        return true;
      }, context);

  auto planGoal = [=](auto& goal, bool recut) {
    const auto state = context->snapshot();
    if (!state.scene_valid ||
        (recut && !context->authorized(state)))
      return false;

    if (!context->captured_scene ||
        !recovery::sameVersions(
            state, *context->captured_scene) ||
        state.cut_environment_id !=
            context->captured_scene->cut_environment_id)
      return false;

    goal.standoff = standoff;
    goal.pos_tol = pos_tol;
    goal.ang_tol = ang_tol;
    goal.cut_id = context->cut_id;

    context->requested_plan_state = state;

    goal.geometry_version = state.geometry_version;
    goal.pose_version = state.pose_version;
    goal.scene_version = state.scene_version;

    goal.execute = false;
    goal.plan_id.clear();

    if (recut) {
      if (context->verdict.status !=
          recovery::State::RESIDUAL_MATERIAL)
        return false;

      goal.parent_plan_id = context->parent_id;

      const auto residuals=recovery::repairIntervals(context->verdict.residuals);
      if(!residuals.empty()) goal.intervals={residuals.front()};

      if (goal.intervals.empty())
        return false;
    }

    context->preparing_candidate = true;

    RCLCPP_INFO(
        node->get_logger(),
        "Plan request: geometry=%llu pose=%llu "
        "scene=%llu environment=%s",
        static_cast<unsigned long long>(
            state.geometry_version),
        static_cast<unsigned long long>(
            state.pose_version),
        static_cast<unsigned long long>(
            state.scene_version),
        state.cut_environment_id.c_str());

    return true;
  };

  // auto planResult = [=](const auto& result) {
  //   RCLCPP_INFO(
  //       node->get_logger(),
  //       "PlanCut result: success=%d executed=%d "
  //       "interrupted=%d plan_id=%s message=%s",
  //       result.success,
  //       result.executed,
  //       result.interrupted,
  //       result.plan_id.c_str(),
  //       result.message.c_str());

  //   if (!result.success ||
  //       result.plan_id.empty() ||
  //       result.executed)
  //     return false;

  //   context->plan_id = result.plan_id;

  //   if (context->parent_id.empty()) {
  //     context->parent_id = result.plan_id;
  //     context->required_segments =
  //         result.required_segments;
  //   }

  //   const auto current = context->snapshot();

  //   if (!recovery::sameVersions(
  //           current, context->requested_plan_state)) {
  //     RCLCPP_WARN(
  //         node->get_logger(),
  //         "Candidate stored but versions changed "
  //         "during planning: scene=%llu expected=%llu",
  //         static_cast<unsigned long long>(
  //             current.scene_version),
  //         static_cast<unsigned long long>(
  //             context->requested_plan_state.scene_version));
  //   }

  //   return true;
  // };


   auto planResult = [=](const auto& result) {
        RCLCPP_INFO(
            node->get_logger(),
            "PlanCut result: success=%d batch=%s "
            "plan=%s features=%zu message=%s",
            result.success,
            result.batch_id.c_str(),
            result.plan_id.c_str(),
            result.feature_plans.size(),
            result.message.c_str());

        if (!result.success || result.executed) {
            // Preserve diagnostics for an entirely failed batch too. Failed
            // planning results never introduce an executable candidate.
            if (context->feature_plans.empty() && !result.executed) {
                context->batch_id = result.batch_id;
                for (const auto& feature : result.feature_plans) {
                    Context::FeaturePlanEntry entry;
                    entry.feature_id = feature.feature_id;
                    entry.message = feature.message;
                    entry.status = Context::FeaturePlanEntry::Status::Unplannable;
                    context->feature_plans.push_back(std::move(entry));
                }
            }
            return false;
        }

        context->plan_id.clear();
        context->batch_id.clear();
        context->feature_plans.clear();
        context->independent_batch = false;
        context->batch_interrupted = false;

        if (!result.feature_plans.empty()) {
            if (result.batch_id.empty() ||
                !result.plan_id.empty() ||
                result.required_segments.empty()) {
                return false;
            }

            std::set<std::string> required(
                result.required_segments.begin(),
                result.required_segments.end());

            if (required.size() != result.required_segments.size() ||
                required.count("")) {
                return false;
            }

            std::set<std::string> seen;
            std::set<std::string> plan_ids;
            std::size_t ready = 0;

            for (const auto& feature : result.feature_plans) {
                if (!required.count(feature.feature_id) ||
                    !seen.insert(feature.feature_id).second) {
                    return false;
                }

                Context::FeaturePlanEntry entry;
                entry.feature_id = feature.feature_id;
                entry.plan_id = feature.plan_id;
                entry.message = feature.message;

                if (feature.status ==
                        hb_robot_interfaces::msg::
                            CutFeaturePlan::READY) {

                    if (entry.plan_id.empty() ||
                        !plan_ids.insert(entry.plan_id).second) {
                        return false;
                    }

                    entry.status =
                        Context::FeaturePlanEntry::Status::Ready;
                    ++ready;
                } else if (feature.status ==
                        hb_robot_interfaces::msg::
                            CutFeaturePlan::UNPLANNABLE) {

                    if (!entry.plan_id.empty())
                        return false;

                    entry.status = Context::FeaturePlanEntry::Status::Unplannable;
                } else {
                    return false;
                }

                context->feature_plans.push_back(
                    std::move(entry));
            }

            if (seen != required || ready == 0)
                return false;

            context->independent_batch = true;
            context->batch_id = result.batch_id;
            context->required_segments = result.required_segments;
            for(auto& entry:context->feature_plans) {
                auto& recovery=context->feature_recovery[entry.feature_id];
                if(recovery.original_plan_id.empty()) recovery.original_plan_id=entry.plan_id;
                if(recovery.original_executed && !recovery.interrupted)
                    entry.status=Context::FeaturePlanEntry::Status::Completed;
            }

            // // A recut batch needs an original stored plan as parent.
            // // Do not substitute the batch ID.
            // if (context->parent_id.empty()) {
            //     const auto first = std::find_if(
            //         context->feature_plans.begin(),
            //         context->feature_plans.end(),
            //         [](const auto& feature) {
            //             return feature.status ==
            //                 Context::FeaturePlanEntry::Status::Ready;
            //         });

            //     context->parent_id = first->plan_id;
            // }

            RCLCPP_INFO(
                node->get_logger(),
                "[CUT BATCH] %s: %zu/%zu ready",
                context->batch_id.c_str(),
                ready,
                context->feature_plans.size());

        } else {
            if (result.plan_id.empty())
                return false;

            context->plan_id = result.plan_id;

            if (context->parent_id.empty()) {
                context->parent_id = result.plan_id;
                context->required_segments =
                    result.required_segments;
            }
        }

        const auto current = context->snapshot();

        if (!recovery::sameVersions(
                current, context->requested_plan_state)) {
            RCLCPP_WARN(
                node->get_logger(),
                "Stored plans require revalidation: "
                "scene=%llu expected=%llu",
                static_cast<unsigned long long>(
                    current.scene_version),
                static_cast<unsigned long long>(
                    context->requested_plan_state.scene_version));
        }

        return true;
    };

  registerAction<Cut>(
      factory, "PlanCut", node, "cut_profile",
      [=](auto& goal) {
        return planGoal(goal, false);
      },
      planResult, context);

    registerAction<RecoverCut>(
        factory,
        "RecoverCut",
        node,
        "recover_cut",

        [=](RecoverCut::Goal& goal) -> bool {
        const auto state=context->snapshot();
        if (!context->recovery_admitted || !knownInterruptedFeature(context) ||
            !context->authorized(state) || !context->captured_scene ||
            !recovery::sameVersions(state,*context->captured_scene) ||
            state.cut_environment_id!=context->captured_scene->cut_environment_id)
            return false;
        context->recovery_goal_sent=true;

        goal.interrupted_plan_id =
            context->interrupted_plan_id;

        goal.cut_id = context->cut_id;
        goal.prefer_retract_endpoint = true;

        return true;
        },

        [=](const RecoverCut::Result& result) -> bool {
            context->recovery_home_reached =
                result.success &&
                result.home_reached &&
                !result.interrupted;

            RCLCPP_INFO(
                node->get_logger(),
                "[CUT RECOVERY] plan=%s success=%d home=%d: %s",
                context->interrupted_plan_id.c_str(),
                int(result.success),
                int(result.home_reached),
                result.message.c_str());

            return context->recovery_home_reached;
    }, context);
    condition("CutFeatureInterrupted", [=] {
        return context->recovery_admitted && knownInterruptedFeature(context);
    });
    factory.registerBuilder<AwaitTorchFaultClear>("AwaitTorchFaultClear",
        [node,context](const auto& name,const auto& config) {
            return std::make_unique<AwaitTorchFaultClear>(name,config,node,context);
        });
    factory.registerSimpleAction("MarkFeatureRecoveryPendingInspection",[=](BT::TreeNode&) {
        if (!markFeatureRecovered(context)) return BT::NodeStatus::FAILURE;
        RCLCPP_WARN(node->get_logger(),"[CUT RECOVERY] feature=%s: RECOVERED_PENDING_INSPECTION",
                    context->recovery_feature.c_str());
        return BT::NodeStatus::SUCCESS;
    });

  // Condition for XML to determine whether the
  // stored candidate remains usable.
  // condition("CandidateVersionsMatch", [=] {
  //   const auto state = context->snapshot();

  //   return !context->plan_id.empty() &&
  //          recovery::sameVersions(
  //              state, context->requested_plan_state);
  // });
  condition("CandidateVersionsMatch", [=] {
    const auto state = context->snapshot();

    const bool candidate_available =
        !context->plan_id.empty() ||
        (context->independent_batch &&
         !context->batch_id.empty() &&
         !context->batch_interrupted &&
         std::any_of(
             context->feature_plans.begin(),
             context->feature_plans.end(),
             [](const auto& feature) {
                 return feature.status ==
                     Context::FeaturePlanEntry::Status::Ready;
             }));

    return candidate_available &&
           recovery::sameVersions(
               state, context->requested_plan_state);
  });


  auto executeGoal = [=](auto& goal) {

    


    context->preparing_candidate = false;  

    const auto state = context->snapshot();
    RCLCPP_INFO(
      node->get_logger(),
      "ExecuteCut request: "
      "plan_id=%s cut_id=%s "
      "authorized=%d versions_match=%d "
      "scene=%llu expected_scene=%llu",
      context->plan_id.c_str(),
      context->cut_id.c_str(),
      context->authorized(state),
      context->planMatches(state),
      static_cast<unsigned long long>(
          state.scene_version),
      static_cast<unsigned long long>(
          context->requested_plan_state.scene_version));


    const bool platform_ok = context->platformStable();
    const bool task_ok = recovery::geometryPermitted(state);
    const bool versions_ok = context->planMatches(state);

    const bool environment_ok =
        context->captured_scene &&
    state.cut_environment_id ==
            context->captured_scene->cut_environment_id;

    const bool candidate_ok = !context->plan_id.empty();
    

    // if (!context->authorized(state) ||
    //     !context->planMatches(state) ||
    //     !context->captured_scene ||
    //     state.cut_environment_id !=
    //         context->captured_scene->cut_environment_id ||
    //     context->plan_id.empty())
    
    
    if (!platform_ok ||
        !task_ok ||
        !versions_ok ||
        !environment_ok ||
        !candidate_ok) 
    {

      RCLCPP_WARN(
        node->get_logger(),
        "[EXECUTE PREFLIGHT] "
        "platform=%d task=%d versions=%d "
        "environment=%d candidate=%d "
        "supervision=%d imu=%d pose=%d "
        "motion_fault=%d process_fault=%d "
        "status='%s' platform_reason='%s'",
        int(platform_ok),
        int(task_ok),
        int(versions_ok),
        int(environment_ok),
        int(candidate_ok),
        int(state.supervision_valid),
        int(state.imu_stable),
        int(state.workpiece_pose_valid),
        int(state.motion_fault_active),
        int(state.process_fault_active),
        state.status_message.c_str(),
        context->platformReason().c_str());

      RCLCPP_ERROR(
          node->get_logger(),
          "Execution denied: authorization, "
          "candidate or environment mismatch");
      return false;
    }

    if (context->independent_batch) {
    const auto selected = std::find_if(
        context->feature_plans.begin(),
        context->feature_plans.end(),
        [&](const auto& feature) {
            return feature.plan_id == context->plan_id &&
                   feature.status ==
                       Context::FeaturePlanEntry::Status::Selected;
        });

    if (selected == context->feature_plans.end() ||
        context->batch_interrupted) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Independent cut execution denied: "
            "no valid selected feature or batch interrupted");
        return false;
    }
}

    goal.standoff = standoff;
    goal.pos_tol = pos_tol;
    goal.ang_tol = ang_tol;

    goal.cut_id = context->cut_id;
    goal.plan_id = context->plan_id;
    goal.execute = true;

    return true;
  };

  // auto executeResult = [=](const auto& result) {
  //   context->execution_version =
  //       result.execution_version;
  //   context->executed = result.executed;
  //   context->plan_id.clear();

  //   return result.success &&
  //          result.executed &&
  //          !result.interrupted;
  // };
  auto executeResult = [=](const auto& result) {
    if (context->independent_batch &&
        (result.cut_id!=context->cut_id || result.plan_id!=context->plan_id)) {
      context->execution_uncertain=true;
      context->batch_interrupted=true;
      context->last_error="Cut result identity mismatch; physical outcome unknown";
      return false;
    }

    if(context->independent_batch && result.executed) {
      const auto it=std::find_if(context->feature_plans.begin(),context->feature_plans.end(),
        [&](const auto& entry){return entry.plan_id==result.plan_id;});
      if(it!=context->feature_plans.end()) {
        context->recovery_feature=it->feature_id;
        auto& feature=context->feature_recovery[it->feature_id];
        feature.original_executed=true;feature.last_motion_epoch=result.execution_version;
        feature.interrupted=result.interrupted || !result.success || result.completed_segments!=1;
        feature.evidence=recovery::Evidence{};feature.verdict={};
      }
    }
    context->execution_uncertain = result.interrupted ||
        (result.executed && (!result.success || (context->independent_batch && result.completed_segments!=1)));
    context->execution_version =
        result.execution_version;

    context->executed =
        context->executed || result.executed;

    if (context->independent_batch) {
        const auto it = std::find_if(
            context->feature_plans.begin(),
            context->feature_plans.end(),
            [&](const auto& feature) {
                return feature.plan_id ==
                    context->plan_id &&
                    feature.status ==
                        Context::FeaturePlanEntry::Status::Selected;
            });

        if (it == context->feature_plans.end()) {
            context->batch_interrupted = true;
            context->clearSelectedFeature();
            // context->plan_id.clear();
            return false;
        }

        const bool completed =
            result.success &&
            result.executed &&
            !result.interrupted &&
            result.completed_segments == 1;

        const bool rejected =
            result.approval_rejected &&
            !result.executed &&
            !result.interrupted &&
            result.completed_segments == 0;

        const bool interrupted =
            !completed &&
            !rejected &&
            result.executed;

        if (completed) {
            it->status =
                Context::FeaturePlanEntry::Status::Completed;
        } else if(rejected) {
            it->status = Context::FeaturePlanEntry::Status::Rejected;
            it->message = result.message;
            // No motion occurred. Keep the other features
            // ready and do not initiate recovery.
            context->batch_interrupted = false;



        }else if (interrupted) {
            it->status =
                Context::FeaturePlanEntry::Status::Interrupted;

            context->batch_interrupted = true;
            context->interrupted_plan_id = it->plan_id;
            context->interrupted_feature_id = it->feature_id;
            context->recovery_home_reached = false;
        } 
        else if (!result.executed && !result.interrupted) {
            it->message = result.message;
            context->batch_interrupted = false;
            return false; // Keep Selected and plan_id for a preflight retry.
        }
        else {
            it->status =
                Context::FeaturePlanEntry::Status::ExecutionFailed;

            // Stop this batch, but do not offer motion recovery.
            context->batch_interrupted = true;
            context->interrupted_plan_id.clear();
            context->interrupted_feature_id.clear();
            context->recovery_home_reached = false;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "[CUT FEATURE] %s: %s",
            it->feature_id.c_str(),
            completed ? "COMPLETED" :
            rejected ? "REJECTED" :
            interrupted ? "INTERRUPTED" : "EXECUTION_FAILED");


        // context->plan_id.clear();
        context->clearSelectedFeature();
        // Both completion and deliberate rejection allow
        // the independent batch to advance.
        return completed || rejected;
    }

    context->plan_id.clear();

    return result.success &&
           result.executed &&
           !result.interrupted;
  };





  registerAction<Cut>(
      factory, "ExecuteCut", node, "cut_profile",
      executeGoal, executeResult, context);

  installRecovery(factory,node,context,standoff,pos_tol,ang_tol,
      planGoal,planResult,executeGoal,executeResult);

  registerAction<Escape>(
      factory, "Escape", node, "escape",
      [](auto& goal) {
        goal.allow_retreat = false;
        return true;
      },
      [=](const auto& result) {
        RCLCPP_WARN(
            node->get_logger(),
            "Escape outcome=%u message=%s "
            "torch_off=%d stationary=%d",
            result.status,
            result.message.c_str(),
            result.torch_confirmed_off,
            result.robot_stationary);

        return result.torch_confirmed_off &&
               result.robot_stationary && result.height_stationary;
      }, context);

  // Commands are queued by ROS callbacks; only the tick thread mutates task state.
  std::atomic<int> command{0}; // 1 resume, 2 cancel, 3 containment, 4 recover, 5 inspect, 6 defer
  const auto command_service = [&](const char* endpoint, int value) {
    return node->create_service<std_srvs::srv::Trigger>(endpoint,
        [&, value](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
          int empty = 0;
          response->success = command.compare_exchange_strong(empty, value);
          response->message = response->success ? "Command queued; observe /task_session/status" :
              "Another task command is pending";
        });
  };
  auto resume_service = command_service("/task_session/resume", 1);
  auto cancel_service = command_service("/task_session/cancel", 2);
  auto contain_service = command_service("/task_session/retry_containment", 3);
  auto recover_service = command_service("/task_session/recover", 4);
  auto inspect_service = command_service("/task_session/inspect", 5);
  auto defer_service = command_service("/task_session/defer_feature", 6);
  auto status_pub = node->create_publisher<hb_robot_interfaces::msg::TaskSessionStatus>(
      "/task_session/status", rclcpp::QoS(1).transient_local());
  auto contain_client = rclcpp_action::create_client<Escape>(node, "escape");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  int code = 0;
  try {
    const auto file = node->declare_parameter<std::string>("tree_file", "");
    const auto log_path = node->declare_parameter<std::string>("bt_log_path", "");
    auto tree = factory.createTreeFromFile(file);
    const auto recovery_file = node->declare_parameter<std::string>("recovery_tree_file",
        (std::filesystem::path(file).parent_path()/"interrupted_cut_recovery.xml").string());
    bool recovery_trees_loaded=false;
    std::unique_ptr<BT::Tree> recovery_tree, inspection_tree;
    const auto load_recovery_trees = [&] {
      if (!recovery_trees_loaded) {
        factory.registerBehaviorTreeFromFile(recovery_file);
        recovery_trees_loaded=true;
      }
    };
    const auto halt_workflows = [&] {
      tree.haltTree();
      if (recovery_tree) recovery_tree->haltTree();
      if (inspection_tree) inspection_tree->haltTree();
      context->recovery_admitted=false;
    };
    const auto matching_geometry = [&] {
      const auto current=context->snapshot();
      return context->authorized(current) && context->captured_scene &&
          recovery::sameVersions(current,*context->captured_scene) &&
          current.cut_environment_id==context->captured_scene->cut_environment_id;
    };
    const auto inspection_pending = [&] {
      return context->independent_batch && featureReadyForInspection(context) &&
          !allFeaturesVerified(context);
    };
    const auto start_inspection = [&] {
      load_recovery_trees();
      // Reset BT checkpoints, not the execution ledger. The selected interrupted
      // original remains consumed and is reached through inspection admission.
      halt_workflows();
      inspection_tree=std::make_unique<BT::Tree>(factory.createTree("RecoveredFeatureInspection"));
    };
    std::unique_ptr<BT::FileLogger> file_logger;
    if (!log_path.empty()) file_logger = std::make_unique<BT::FileLogger>(tree, log_path.c_str());
    // Resume is safe only for a tree whose root preserves completed checkpoints.
    const bool checkpointed = tree.rootNode()->registrationName() == "SequenceStar";
    RCLCPP_INFO(node->get_logger(),
        "[TASK CONFIG] tree_file=%s root=%s planning_action_timeout_s=%.3f",
        file.c_str(), tree.rootNode()->registrationName().c_str(), context->planning_action_timeout_s);
    std::string outcome = "RUNNING", reason;
    bool contained = false, cancel_requested = false, containment_pending = false;
    bool containment_result_done = false, physical_containment = false;
    bool containment_after_drain = false;
    bool containment_timeout_logged = false;
    using EscapeHandle = rclcpp_action::ClientGoalHandle<Escape>;
    std::shared_future<EscapeHandle::SharedPtr> containment_goal;
    std::shared_future<EscapeHandle::WrappedResult> containment_result;
    EscapeHandle::SharedPtr containment_handle;
    Clock::time_point containment_started{}, published_at{};
    std::string last_published_outcome;
    const auto begin_containment = [&] {
      contained = false;
      context->recovery_admitted=false;
      outcome = "CONTAINING";
      containment_started = Clock::now();
      containment_timeout_logged = false;
      containment_pending = true;
      containment_result_done = false;
      physical_containment = false;
      containment_after_drain = context->cancellation_waiters.empty();
      containment_goal = {};
      containment_result = {};
      containment_handle.reset();
    };
    while (rclcpp::ok()) {
      const int requested = command.exchange(0);
      if (requested == 2 && outcome != "CANCELED_SAFE") {
        cancel_requested = true;
        halt_workflows(); // Cancel whichever workflow owns motion, then contain.
        reason = "Operator canceled the task";
        if (!containment_pending) begin_containment();
      } else if (requested == 3 && outcome == "FAULT_UNCONTAINED" && !containment_pending) {
        begin_containment();
      } else if (requested == 4) {
        if (outcome=="RECOVERY_REQUIRED" && contained && !cancel_requested &&
            !containment_pending && context->cancellation_waiters.empty() && knownInterruptedFeature(context)) {
          try {
            load_recovery_trees();
            halt_workflows();
            recovery_tree=std::make_unique<BT::Tree>(factory.createTree("InterruptedCutRecovery"));
            context->recovery_admitted=true;
            context->recovery_goal_sent=false;
            context->recovery_home_reached=false;
            context->last_error.clear();
            context->first_action_timeout.clear();
            outcome="AWAITING_RECOVERY";
            reason="Waiting for fault clearance, live authorization and height HOME";
          } catch (const std::exception& error) {
            context->recovery_admitted=false;
            reason=std::string("Recovery tree could not be loaded: ")+error.what();
          }
        } else reason="Recovery denied: first confirm containment of a known interrupted feature";
      } else if (requested == 5) {
        if (outcome=="NEEDS_INSPECTION" && contained && !cancel_requested &&
            context->cancellation_waiters.empty() && !context->batch_interrupted &&
            inspection_pending() && matching_geometry()) {
          try {
            start_inspection();
            contained=false;
            outcome="RUNNING";
            reason="Operator requested renewed feature inspection";
            context->last_error.clear();
            context->first_action_timeout.clear();
          } catch (const std::exception& error) { reason=error.what(); }
        } else reason="Inspection denied: contain/recover motion and restore matching geometry first";
      } else if (requested == 6) {
        if (outcome=="NEEDS_INSPECTION" && contained && !cancel_requested &&
            !containment_pending && context->cancellation_waiters.empty() &&
            !context->batch_interrupted && inspection_pending() && matching_geometry()) {
          try {
            // Prepare the replacement flow before changing the ledger. Failed
            // XML loading must leave the operator's inspection target intact.
            load_recovery_trees();
            auto continuation=std::make_unique<BT::Tree>(factory.createTree("RecoveredFeatureInspection"));
            const auto deferred=context->recovery_feature;
            halt_workflows();
            if (!deferInspectionFeature(context)) throw std::runtime_error("Feature is not eligible for deferral");
            inspection_tree=std::move(continuation);
            contained=allActiveFeaturesVerified(context); // No new motion when nothing remains.
            outcome="RUNNING";
            reason="Deferred "+deferred+" as unverified; continuing remaining features";
            context->last_error.clear();
            context->first_action_timeout.clear();
            RCLCPP_WARN(node->get_logger(),"[CUT DEFERRED] feature=%s; original and recut history retained",deferred.c_str());
          } catch (const std::exception& error) { reason=error.what(); }
        } else reason="Deferral denied: requires NEEDS_INSPECTION, confirmed containment, recovered feature and matching live geometry";
      } else if (requested == 1) {
        if (outcome == "PAUSED" && contained && checkpointed && !context->execution_uncertain &&
            !context->batch_interrupted && !cancel_requested) {
          const auto current = context->snapshot();
          const bool obtaining_geometry = context->plan_id.empty() &&
              context->feature_plans.empty() && !context->executed;
          const bool ready = obtaining_geometry ?
              (context->platformStable() && !current.motion_fault_active && !current.process_fault_active) :
              context->authorized(current);
          if (ready &&
              (context->plan_id.empty() || context->planMatches(current))) {
            outcome = "RUNNING";
            contained=false;
            reason.clear();
            context->last_error.clear();
            context->first_action_timeout.clear();
          } else reason = "Resume denied: restore supervision/pose and matching plan versions first";
        } else reason = "Resume denied: task is canceled, uncontained, requires inspection, or has no safe checkpoint";
      }
      // Publish RECOVERED_PENDING_INSPECTION for a full iteration before
      // admitting the continuation; fault clearance alone never reaches here.
      if (outcome=="RECOVERED_PENDING_INSPECTION") {
        if (matching_geometry() && featureReadyForInspection(context)) {
          try {
            start_inspection();
            contained=false;
            outcome="RUNNING";
            reason="Inspecting recovered feature before any remaining original cut";
            context->last_error.clear();
            context->first_action_timeout.clear();
          } catch (const std::exception& error) {
            reason=error.what();
            begin_containment();
          }
        } else reason="Recovered; waiting for live authorization and unchanged geometry before inspection";
      }
      if (outcome=="AWAITING_RECOVERY" || outcome=="RECOVERING") {
        try {
          const auto status=recovery_tree->tickRoot();
          if (context->recovery_goal_sent) { outcome="RECOVERING"; contained=false; }
          if (status==BT::NodeStatus::SUCCESS) {
            context->recovery_admitted=false;
            outcome="RECOVERED_PENDING_INSPECTION";
            contained=true; // RecoverCut confirmed OFF, height HOME and stationary inspection_home.
            reason="Recovery confirmed; cut outcome still requires inspection";
          } else if (status==BT::NodeStatus::FAILURE) {
            reason=context->last_error.empty()?"Recovery failed; containing again":context->last_error;
            recovery_tree->haltTree();
            begin_containment();
          } else if (!context->last_error.empty()) reason=context->last_error;
        } catch (const std::exception& error) {
          reason=error.what();
          recovery_tree->haltTree();
          begin_containment();
        }
      }
      if (outcome == "RUNNING") {
        try {
          auto& active_tree=inspection_tree?*inspection_tree:tree;
          const auto status = active_tree.tickRoot();
          if (status == BT::NodeStatus::SUCCESS) {
 outcome = hasRejectedFeatures(context) ? "COMPLETED_WITH_SKIPPED" :
                hasDeferredFeatures(context) ? "COMPLETED_WITH_DEFERRED" : "COMPLETED";
            reason = (hasDeferredFeatures(context) || hasRejectedFeatures(context)) ?
                "Remaining active features verified; rejected/deferred features remain unresolved and separation is not verified" :
                "Selected workflow completed; separation verification is reported separately";
          } else if (status == BT::NodeStatus::FAILURE) {
            reason = !context->first_action_timeout.empty() ? context->first_action_timeout :
                context->last_error.empty() ? "Workflow step requires attention" : context->last_error;
            // Deliberately do not halt the checkpoint tree on a terminal child failure.
            begin_containment();
          }
        } catch (const std::exception& error) {
          reason = error.what();
          context->execution_uncertain = true;
          halt_workflows();
          begin_containment();
        }
      }
      auto& waiters = context->cancellation_waiters;
      waiters.erase(std::remove_if(waiters.begin(), waiters.end(), [](auto& settled) {
        try { return settled(); } catch (...) { return false; }
      }), waiters.end());
      if (containment_pending) {
        if (!containment_goal.valid() && contain_client->action_server_is_ready()) {
          Escape::Goal goal;
          goal.allow_retreat = false;
          containment_goal = contain_client->async_send_goal(goal);
        }
        if (!containment_handle && containment_goal.valid() &&
            containment_goal.wait_for(0s) == std::future_status::ready) {
          containment_handle = containment_goal.get();
          if (containment_handle) containment_result = contain_client->async_get_result(containment_handle);
          else { outcome = "FAULT_UNCONTAINED"; containment_pending = false; }
        }
        if (!containment_result_done && containment_result.valid() && containment_result.wait_for(0s) == std::future_status::ready) {
          const auto reply = containment_result.get();
          physical_containment = reply.code == rclcpp_action::ResultCode::SUCCEEDED && reply.result &&
              reply.result->torch_confirmed_off && reply.result->robot_stationary && reply.result->height_stationary;
          containment_result_done = true;
          RCLCPP_INFO(node->get_logger(),
              "[CONTAINMENT RESULT] physical_stop_confirmed=%d pending_cancellations=%zu after_drain=%d",
              int(physical_containment), waiters.size(), int(containment_after_drain));
        }
        if (containment_result_done && waiters.empty()) {
          // A goal may be accepted after the first stop. Confirm containment
          // again after every canceled action has reached a terminal state.
          if (!containment_after_drain) {
            containment_after_drain = true;
            containment_result_done = false;
            physical_containment = false;
            containment_goal = {};
            containment_result = {};
            containment_handle.reset();
            containment_started = Clock::now();
            containment_timeout_logged = false;
            continue;
          }
          contained = physical_containment;
          containment_pending = false;
          outcome = !contained ? "FAULT_UNCONTAINED" : cancel_requested ? "CANCELED_SAFE" :
              knownInterruptedFeature(context) ? "RECOVERY_REQUIRED" :
              (context->execution_uncertain || context->batch_interrupted || inspection_pending()) ?
                  "NEEDS_INSPECTION" : "PAUSED";
        } else if (Clock::now() - containment_started > 15s) {
          outcome = "FAULT_UNCONTAINED";
          if (!containment_timeout_logged) {
            containment_timeout_logged = true;
            RCLCPP_ERROR(node->get_logger(),
                "[CONTAINMENT TIMEOUT] result_received=%d physical_stop_confirmed=%d "
                "pending_cancellations=%zu; original_reason=%s",
                int(containment_result_done), int(physical_containment), waiters.size(), reason.c_str());
          }
          // A late result is still observed. Do not send overlapping containment goals.
          if (!containment_goal.valid()) containment_pending = false;
        }
      }
      if (outcome != last_published_outcome || Clock::now() - published_at >= 1s) {
        hb_robot_interfaces::msg::TaskSessionStatus status;
        status.header.stamp = node->now();
        status.task_id = context->cut_id;
        status.outcome = outcome;
        status.reason = reason;
        status.active_node = context->last_action;
        status.plan_id = context->plan_id;
        status.batch_id = context->batch_id;
        status.interrupted_plan_id = context->interrupted_plan_id;
        status.interrupted_feature_id = context->interrupted_feature_id;
        status.recovery_feature = context->recovery_feature;
        status.contained = contained;
        status.separation_verified = context->independent_batch ? allFeaturesVerified(context) :
            recovery::evaluate(context->evidence,context->snapshot(),context->cut_id).status==recovery::State::VERIFIED;
        const char* feature_states[] = {"READY", "UNPLANNABLE", "SELECTED", "COMPLETED", "INTERRUPTED",
            "RECOVERED_PENDING_INSPECTION", "EXECUTION_FAILED", "REJECTED", "DEFERRED"};
        for (const auto& feature : context->feature_plans) {
          status.feature_ids.push_back(feature.feature_id);
          status.feature_states.push_back(feature_states[static_cast<unsigned>(feature.status)]);
          const auto& ledger=context->feature_recovery[feature.feature_id];
          status.feature_execution_epochs.push_back(ledger.last_motion_epoch);
          status.feature_evidence_epochs.push_back(ledger.evidence.execution_version);
          status.feature_kerf_status.push_back(recovery::evaluateFeature(ledger.evidence,context->snapshot(),
            context->cut_id,feature.feature_id,ledger.last_motion_epoch).status);
          status.feature_original_plan_ids.push_back(ledger.original_plan_id);
          status.feature_recut_plan_ids.push_back(ledger.recut_plan_id);
        }
        status_pub->publish(status);
        if (outcome != last_published_outcome)
          RCLCPP_INFO(node->get_logger(), "[TASK SESSION] %s: %s", outcome.c_str(), reason.c_str());
        last_published_outcome = outcome;
        published_at = Clock::now();
      }
      std::this_thread::sleep_for(50ms);
    }
    halt_workflows();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Task session initialization failed: %s", error.what());
    code = 1;
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return code;
}
