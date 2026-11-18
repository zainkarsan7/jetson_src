
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
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
          ExecutionFailed
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



  std::string parent_id;
  std::string inspection_reference_id;

  std::vector<std::string> required_segments;
  recovery::Verdict verdict;

  uint64_t execution_version = 0;
  unsigned inspection_attempts = 0;

  std::optional<recovery::State> captured_scene;

  bool executed = false;
  bool preparing_candidate = false;

  recovery::State snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    auto result = state;

    if (Clock::now() - received > 500ms) {
      result.supervision_valid = false;
      result.imu_stable = false;
      result.scene_valid = false;
    }

    return result;
  }

  bool platformStable() {
    std::lock_guard<std::mutex> lock(mutex);

    if (!supervision_received_any ||
        Clock::now() - supervision_received > 500ms)
      return false;

    return 
           supervision.supervision_valid &&
          //  supervision.both_channels_valid &&
          //  supervision.both_channels_stable &&
           !supervision.hard_motion_detected;
  }

  std::string platformReason() {
    std::lock_guard<std::mutex> lock(mutex);

    if (!supervision_received_any ||
        Clock::now() - supervision_received > 500ms)
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
        ResultHandler handler)
        : BT::StatefulActionNode(name, config),
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
      sent_ = false;
      started_ = Clock::now();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
      const auto elapsed = Clock::now() - started_;

      if (elapsed > 10min ||
          (!sent_ && elapsed > 10s)) {
        onHalted();
        return BT::NodeStatus::FAILURE;
      }

      if (!sent_) {
        if (!client_->action_server_is_ready())
          return BT::NodeStatus::RUNNING;

        typename Action::Goal goal;

        if (!build_(goal)){

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

      if (!wrapped.result) {
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
    GoalBuilder build_;
    ResultHandler finish_;

    typename rclcpp_action::Client<Action>::SharedPtr client_;
    typename Handle::SharedPtr handle_;

    std::shared_future<typename Handle::SharedPtr> goal_;
    std::shared_future<typename Handle::WrappedResult> result_;

    std::shared_ptr<std::atomic<bool>> cancel_;

    bool sent_ = false;
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
        started_ = Clock::now();
        stable_since_.reset();
        return onRunning();
      }

      BT::NodeStatus onRunning() override {
        const auto state = context_->snapshot();

        if (state.motion_fault_active ||
            state.process_fault_active) {
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


class AwaitTorchFaultClear : public BT::StatefulActionNode {
public:
    AwaitTorchFaultClear(
        const std::string& name,
        const BT::NodeConfiguration& config,
        std::shared_ptr<Context> context)
        : BT::StatefulActionNode(name, config),
          context_(std::move(context)) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus onStart() override {
        
        RCLCPP_INFO(
            rclcpp::get_logger("bt_executor"),
            "[CUT RECOVERY] Waiting for operator fault clearance");
        return check();
    }

    BT::NodeStatus onRunning() override {
        return check();
    }

    void onHalted() override {
        saw_fault_ = false;
    }

private:
    BT::NodeStatus check() {
        const auto state = context_->snapshot();

        if (!state.scene_valid &&
            !state.supervision_valid) {
            return BT::NodeStatus::RUNNING;
        }

        if (state.process_fault_active ||
            state.motion_fault_active) {
                if (!saw_fault_) {
                        RCLCPP_WARN(
                            rclcpp::get_logger("bt_executor"),
                            "[CUT RECOVERY] Fault observed; "
                            "waiting for operator clearance");
                    }
            saw_fault_ = true;
            return BT::NodeStatus::RUNNING;
        }
        // RCLCPP_INFO(
        //    rclcpp::get_logger("bt_executor"),
        //     "[AWAIT FAULT CLEAR] saw_fault=%d "
        //     "process_fault=%d motion_fault=%d "
        //     "authorized=%d scene_valid=%d",
        //     static_cast<int>(saw_fault_),
        //     static_cast<int>(state.process_fault_active),
        //     static_cast<int>(state.motion_fault_active),
        //     static_cast<int>(context_->authorized(state)),
        //     static_cast<int>(state.scene_valid));

        if (!saw_fault_)
            return BT::NodeStatus::RUNNING;

        if (!context_->authorized(state) ||
            !state.scene_valid)
            return BT::NodeStatus::RUNNING;

        return BT::NodeStatus::SUCCESS;
    }
    bool saw_fault_ = false;

    std::shared_ptr<Context> context_;
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
                RCLCPP_ERROR(
                    rclcpp::get_logger("bt_executor"),
                    "Environment capture expired or reference changed");
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
    Result result) {

  factory.registerBuilder<ActionNode<Action>>(
      name,
      [=](const auto& n, const auto& c) {
        return std::make_unique<ActionNode<Action>>(
            n, c, node, endpoint, goal, result);
      });
}






int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<rclcpp::Node>("hb_bt_executor");

  auto context = std::make_shared<Context>();

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
          [context](recovery::State::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            context->state = *m;
            context->received = Clock::now();
          });

  auto supervision_sub =
      node->create_subscription<Supervision>(
          "/platform_supervision",
          rclcpp::QoS(10).reliable(),
          [context](Supervision::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            context->supervision = *m;
            context->supervision_received = Clock::now();
            context->supervision_received_any = true;
          });

  auto reference_sub =
      node->create_subscription<
          hb_robot_interfaces::msg::WorkpieceReference>(
          "/workpiece_reference",
          rclcpp::QoS(1).transient_local(),
          [context](
              hb_robot_interfaces::msg::
                  WorkpieceReference::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(
                context->mutex);
            context->estimate = m->estimate;
          });

  BT::BehaviorTreeFactory factory;

  



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

  
  condition("AllCutFeaturesCompleted", [=] {
    if (!context->independent_batch)
        return false;

    if (context->feature_plans.empty())
        return false;

    return std::all_of(
        context->feature_plans.begin(),
        context->feature_plans.end(),
        [](const auto& feature) {
            return feature.status ==
                Context::FeaturePlanEntry::Status::Completed;
        });
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

    return Clock::now() - context->received <= 500ms &&
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

  condition("KerfVerified", [=] {
    return context->verdict.status ==
           recovery::State::VERIFIED;
  });

  condition("ResidualDetected", [=] {
    return context->verdict.status ==
           recovery::State::RESIDUAL_MATERIAL;
  });

  condition("KerfInconclusive", [=] {
    return context->verdict.status ==
           recovery::State::INCONCLUSIVE;
  });

  condition("ValidateKerf", [=] {
    const auto s = context->snapshot();

    context->verdict = recovery::evaluate(
        context->evidence,
        s,
        context->cut_id);

    return true;
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
        goal.max_views = 10;
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
      });

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
      });

  auto planGoal = [=](auto& goal, bool recut) {
    const auto state = context->snapshot();
    if (recut && context->independent_batch) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Independent recut lineage is not implemented");
        return false;
    }
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

      std::map<std::string, recovery::Interval> merged;

      for (const auto& i : context->verdict.residuals) {
        auto it = merged.find(i.segment_id);

        if (it == merged.end()) {
          merged[i.segment_id] = i;
        } else {
          it->second.s0 =
              std::min(it->second.s0, i.s0);
          it->second.s1 =
              std::max(it->second.s1, i.s1);
        }
      }

      for (const auto& pair : merged)
        goal.intervals.push_back(pair.second);

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

        if (!result.success || result.executed)
            return false;

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

                    entry.status =
                        Context::FeaturePlanEntry::Status::Unplannable;
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
      planResult);

  registerAction<Cut>(
      factory, "PlanRecut", node, "cut_profile",
      [=](auto& goal) {
        return planGoal(goal, true);
      },
      planResult);

    
    registerAction<RecoverCut>(
        factory,
        "RecoverCut",
        node,
        "recover_cut",

        [=](RecoverCut::Goal& goal) -> bool {
        if (!context->independent_batch ||
            !context->batch_interrupted ||
            context->interrupted_plan_id.empty() ||
            context->interrupted_feature_id.empty())
            return false;

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
    });
    condition("CutFeatureInterrupted", [=] {
            using Status = Context::FeaturePlanEntry::Status;

            if (context->interrupted_plan_id.empty() ||
                context->interrupted_feature_id.empty())
                return false;

            return std::any_of(
                context->feature_plans.begin(),
                context->feature_plans.end(),
                [&](const auto& feature) {
                    return feature.plan_id ==
                            context->interrupted_plan_id &&
                        feature.feature_id ==
                            context->interrupted_feature_id &&
                        feature.status == Status::Interrupted;
                });
        });


    factory.registerBuilder<AwaitTorchFaultClear>(
    "AwaitTorchFaultClear",
    [context](const auto& name, const auto& config) {
        return std::make_unique<AwaitTorchFaultClear>(
            name, config, context);
    });



    factory.registerSimpleAction(
    "MarkFeatureRecoveryPendingInspection",
    [=](BT::TreeNode&) {
        using Status =
            Context::FeaturePlanEntry::Status;

        if (!context->independent_batch ||
            !context->batch_interrupted ||
            !context->recovery_home_reached)
            return BT::NodeStatus::FAILURE;

        const auto it = std::find_if(
            context->feature_plans.begin(),
            context->feature_plans.end(),
            [&](const auto& feature) {
                return feature.plan_id ==
                    context->interrupted_plan_id &&
                    feature.feature_id ==
                    context->interrupted_feature_id &&
                    feature.status == Status::Interrupted;
            });

        if (it == context->feature_plans.end())
            return BT::NodeStatus::FAILURE;

        it->status =
            Status::RecoveredPendingInspection;

        it->message =
            "Returned home after interrupted cut; "
            "cut outcome awaits inspection";

        RCLCPP_WARN(
            node->get_logger(),
            "[CUT RECOVERY] feature=%s: "
            "RECOVERED_PENDING_INSPECTION",
            it->feature_id.c_str());

        context->batch_interrupted = false;
        context->recovery_home_reached = false;
        context->interrupted_plan_id.clear();
        context->interrupted_feature_id.clear();
        context->plan_id.clear();

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

        const bool interrupted =
            !completed &&
            result.interrupted &&
            result.executed;

        if (completed) {
            it->status =
                Context::FeaturePlanEntry::Status::Completed;
        } else if (interrupted) {
            it->status =
                Context::FeaturePlanEntry::Status::Interrupted;

            context->batch_interrupted = true;
            context->interrupted_plan_id = it->plan_id;
            context->interrupted_feature_id = it->feature_id;
            context->recovery_home_reached = false;
        } else {
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
            interrupted ? "INTERRUPTED" : "EXECUTION_FAILED");


        // context->plan_id.clear();
        context->clearSelectedFeature();
        return completed;
    }

    context->plan_id.clear();

    return result.success &&
           result.executed &&
           !result.interrupted;
  };





  registerAction<Cut>(
      factory, "ExecuteCut", node, "cut_profile",
      executeGoal, executeResult);

  registerAction<Cut>(
      factory, "ExecuteRecut", node, "cut_profile",
      executeGoal, executeResult);

  registerAction<Kerf>(
      factory, "InspectKerf", node, "inspect_cut",
      [=](auto& goal) {
        const auto state = context->snapshot();

        if (!context->authorized(state) ||
            ++context->inspection_attempts > 3)
          return false;

        {
          std::lock_guard<std::mutex> lock(
              context->mutex);
          goal.estimate = context->estimate;
        }

        goal.cut_id = context->cut_id;
        goal.required_segments =
            context->required_segments;

        goal.geometry_version =
            state.geometry_version;
        goal.pose_version = state.pose_version;
        goal.execution_version =
            context->execution_version;

        goal.samples_per_viewpoint = 3;
        goal.camera_standoff =
            context->inspection_attempts == 1
                ? 0.30
                : 0.35;

        return true;
      },
      [=](const auto& result) {
        context->evidence = result.evidence;
        return result.success;
      });

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
               result.robot_stationary;
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin([&] {
    executor.spin();
  });

  int code = 1;

  try {
    const auto file =
        node->declare_parameter<std::string>(
            "tree_file", "");
    const auto bt_log_path =
        node->declare_parameter<std::string>(
            "bt_log_path", "/ssd/data/execution.fbl");

    auto tree = factory.createTreeFromFile(file);
    BT::FileLogger bt_logger(tree, bt_log_path.c_str());

    // BT::PublisherZMQ groot_publisher(
    //     tree, 25, 1666, 1667);

    BT::StdCoutLogger console_logger(tree);

    BT::NodeStatus status =
        BT::NodeStatus::RUNNING;

    while (rclcpp::ok() &&
           status == BT::NodeStatus::RUNNING) {
      status = tree.tickRoot();
      std::this_thread::sleep_for(50ms);
    }

    tree.haltTree();

    code = status == BT::NodeStatus::SUCCESS
        ? 0
        : 1;

    RCLCPP_INFO(
        node->get_logger(),
        "Tree finished: %s",
        code == 0 ? "SUCCESS" : "FAILURE");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(
        node->get_logger(), "%s", e.what());
  }

  executor.cancel();
  spin.join();
  rclcpp::shutdown();

  return code;
}
