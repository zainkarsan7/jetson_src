#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/srv/capture_cut_environment.hpp>
#include <optional>
#include <hb_robot_interfaces/action/inspect_profile.hpp>
#include <hb_robot_interfaces/action/inspect_cut.hpp>
#include <hb_robot_interfaces/action/cut_profile.hpp>
#include <hb_robot_interfaces/action/revalidate_workpiece.hpp>
#include <hb_robot_interfaces/action/escape.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>

using namespace std::chrono_literals;
namespace recovery=hb_robot::recovery;
using Inspect=hb_robot_interfaces::action::InspectProfile;
using Kerf=hb_robot_interfaces::action::InspectCut;
using Cut=hb_robot_interfaces::action::CutProfile;
using Revalidate=hb_robot_interfaces::action::RevalidateWorkpiece;
using Escape=hb_robot_interfaces::action::Escape;

// ROS callbacks only write subscribed data under mutex. BT-owned plan/evidence
// state is changed on the tick thread after an action result has been received.
struct Context {
  std::mutex mutex;
  recovery::State state,requested_plan_state;
  hb_robot_interfaces::msg::ProfileEstimate estimate;
  recovery::Evidence evidence;
  std::chrono::steady_clock::time_point received{};
  std::string cut_id,plan_id,parent_id,inspection_reference_id;
  std::vector<std::string> required_segments;
  recovery::Verdict verdict;
  uint64_t execution_version=0;
  unsigned inspection_attempts=0;
  std::optional<recovery::State> captured_scene;
  bool executed=false;
  bool preparing_candidate=false; // Planning and stationary pose confirmation issue no cut motion.
  recovery::State snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    auto result=state;
    if(std::chrono::steady_clock::now()-received>500ms) {
      result.supervision_valid=false;
      result.scene_valid=false;
    }
    return result;
  }
};

// Shared asynchronous adapter. No blocking ROS waits occur inside a BT tick.
// A pending goal can be accepted after halt; its response callback cancels it,
// avoiding the orphan-goal race in the old duplicated action nodes.
template<class Action>
class ActionNode : public BT::StatefulActionNode {
 public:
  using Handle=rclcpp_action::ClientGoalHandle<Action>;
  using GoalBuilder=std::function<bool(typename Action::Goal&)>;
  using ResultHandler=std::function<bool(const typename Action::Result&)>;
  ActionNode(const std::string& name,const BT::NodeConfiguration& config,
      rclcpp::Node::SharedPtr node,std::string action,GoalBuilder goal,ResultHandler result)
      :BT::StatefulActionNode(name,config),build_(goal),finish_(result) {
    client_=rclcpp_action::create_client<Action>(node,action);
  }
  static BT::PortsList providedPorts(){return {};}
  BT::NodeStatus onStart() override {
    cancel_=std::make_shared<std::atomic<bool>>(false);handle_.reset();sent_=false;
    started_=std::chrono::steady_clock::now();return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const auto elapsed=std::chrono::steady_clock::now()-started_;
    if(elapsed>10min || (!sent_ && elapsed>10s)) {
      onHalted();return BT::NodeStatus::FAILURE;
    }
    if(!sent_) {
      if(!client_->action_server_is_ready()) return BT::NodeStatus::RUNNING;
      typename Action::Goal goal;
      if(!build_(goal)) return BT::NodeStatus::FAILURE;
      typename rclcpp_action::Client<Action>::SendGoalOptions options;
      auto cancellation=cancel_;auto client=client_;
      options.goal_response_callback=[client,cancellation](typename Handle::SharedPtr h){
        if(h && cancellation->load()) client->async_cancel_goal(h);
      };
      goal_=client_->async_send_goal(goal,options);sent_=true;
    }
    if(!handle_) {
      if(goal_.wait_for(0s)!=std::future_status::ready) return BT::NodeStatus::RUNNING;
      handle_=goal_.get();if(!handle_) return BT::NodeStatus::FAILURE;
      result_=client_->async_get_result(handle_);
    }
    if(result_.wait_for(0s)!=std::future_status::ready) return BT::NodeStatus::RUNNING;
    const auto wrapped=result_.get();
    
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !wrapped.result)
      {
          RCLCPP_ERROR(
              rclcpp::get_logger("bt_executor"),
              "Action '%s' failed: ROS result code=%d, has_result=%d, server message: %s",
              registrationName().c_str(),
              static_cast<int>(wrapped.code),
              static_cast<bool>(wrapped.result),
              wrapped.result ? wrapped.result->message.c_str() : "(no result)"
          );
          return BT::NodeStatus::FAILURE;
      }

      if (finish_(*wrapped.result)) return BT::NodeStatus::SUCCESS;
      RCLCPP_ERROR(rclcpp::get_logger("bt_executor"),
          "Action '%s' returned SUCCEEDED but its result failed the BT check: %s",
          registrationName().c_str(), wrapped.result->message.c_str());
      return BT::NodeStatus::FAILURE;


  } 

  
  void onHalted() override {
    if(cancel_) *cancel_=true;
    if(handle_) client_->async_cancel_goal(handle_);
    else if(sent_ && goal_.valid() && goal_.wait_for(0s)==std::future_status::ready) {
      auto handle=goal_.get();if(handle) client_->async_cancel_goal(handle);
    }
  }
 private:
  GoalBuilder build_;ResultHandler finish_;
  typename rclcpp_action::Client<Action>::SharedPtr client_;
  typename Handle::SharedPtr handle_;
  std::shared_future<typename Handle::SharedPtr> goal_;
  std::shared_future<typename Handle::WrappedResult> result_;
  std::shared_ptr<std::atomic<bool>> cancel_;
  bool sent_=false;std::chrono::steady_clock::time_point started_;
};
class AwaitGeometry : public BT::StatefulActionNode {
 public:
  AwaitGeometry(const std::string& n,const BT::NodeConfiguration& c,std::shared_ptr<Context> context,bool planning=false):
    BT::StatefulActionNode(n,c),context_(context),planning_(planning){}
  static BT::PortsList providedPorts(){return {};}
  BT::NodeStatus onStart() override {start_=std::chrono::steady_clock::now();return onRunning();}
  BT::NodeStatus onRunning() override {
    const auto state=context_->snapshot();
    if(planning_) {
      if(state.scene_valid && state.scene_id==context_->inspection_reference_id)
        return BT::NodeStatus::SUCCESS;
    } else if(recovery::geometryPermitted(state) && state.scene_id==context_->inspection_reference_id)
      return BT::NodeStatus::SUCCESS;
    if(std::chrono::steady_clock::now()-start_<=3s) return BT::NodeStatus::RUNNING;
    if(planning_) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_executor"),
        "Inspection reference %s was not acknowledged for planning (latest accepted=%s)",
        context_->inspection_reference_id.c_str(),state.scene_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_ERROR(rclcpp::get_logger("bt_executor"),
      "AwaitGeometry timed out: supervision_valid=%d imu_stable=%d pose_valid=%d "
      "process_fault=%d motion_fault=%d; task_state: %s",
      state.supervision_valid,state.imu_stable,state.workpiece_pose_valid,
      state.process_fault_active,state.motion_fault_active,state.status_message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  void onHalted() override {}
 private:
  std::shared_ptr<Context> context_;bool planning_;std::chrono::steady_clock::time_point start_;
};
class CaptureEnvironment : public BT::StatefulActionNode {
  using Service = hb_robot_interfaces::srv::CaptureCutEnvironment;
 public:
  CaptureEnvironment(const std::string& name, const BT::NodeConfiguration& cfg,
                     rclcpp::Node::SharedPtr node, std::shared_ptr<Context> context)
      : BT::StatefulActionNode(name, cfg), context_(context),
        client_(node->create_client<Service>("capture_cut_environment")) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    context_->captured_scene.reset();
    expected_ = context_->snapshot();
    deadline_ = std::chrono::steady_clock::now() + 20s;
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    const auto state = context_->snapshot();
    if (!state.scene_valid || state.motion_fault_active || state.process_fault_active || expected_.geometry_version != state.geometry_version ||
        expected_.pose_version != state.pose_version || std::chrono::steady_clock::now() > deadline_) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_executor"), "Environment capture expired or reference changed");
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
    if (!response_ && !future_.valid()) {
      if (!client_->service_is_ready()) return BT::NodeStatus::RUNNING;
      auto request = std::make_shared<Service::Request>();
      request->geometry_version = expected_.geometry_version;
      request->pose_version = expected_.pose_version;
      request->scene_version = expected_.scene_version;
      auto pending = client_->async_send_request(request);
      request_id_ = pending.request_id;
      future_ = pending.share();
    }
    if (!response_) {
      if (future_.wait_for(0s) != std::future_status::ready) return BT::NodeStatus::RUNNING;
      response_ = future_.get();
      future_ = {};
      if (!response_->success) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_executor"), "Environment capture failed: %s",
                     response_->message.c_str());
        response_.reset();
        return BT::NodeStatus::FAILURE;
      }
    }
    if (state.cut_environment_id != response_->environment_id ||
        (response_->scene_changed && state.scene_version <= expected_.scene_version))
      return BT::NodeStatus::RUNNING;
    context_->captured_scene = state;
    response_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {
    if (future_.valid()) client_->remove_pending_request(request_id_);
    future_ = {};
    response_.reset();
  }
 private:
  std::shared_ptr<Context> context_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Client<Service>::SharedFuture future_;
  Service::Response::SharedPtr response_;
  int64_t request_id_ = 0;
  recovery::State expected_;
  std::chrono::steady_clock::time_point deadline_;
};

template<class Action,class Goal,class Result>
void action(BT::BehaviorTreeFactory& factory,const std::string& name,
    rclcpp::Node::SharedPtr node,const std::string& endpoint,Goal goal,Result result) {
  factory.registerBuilder<ActionNode<Action>>(name,[=](const auto& n,const auto& c){
    return std::make_unique<ActionNode<Action>>(n,c,node,endpoint,goal,result);
  });
}

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("hb_bt_executor");
  auto context=std::make_shared<Context>();
  context->cut_id=node->declare_parameter<std::string>("cut_id","cut_"+std::to_string(node->now().nanoseconds()));
  const double standoff=node->declare_parameter("standoff",0.01);
  const double pos_tol=node->declare_parameter("pos_tol",0.01);
  const double ang_tol=node->declare_parameter("ang_tol",0.3);
  auto state_sub=node->create_subscription<recovery::State>("/task_state",rclcpp::QoS(1).transient_local(),
    [context](recovery::State::ConstSharedPtr m){std::lock_guard<std::mutex> lock(context->mutex);
      context->state=*m;context->received=std::chrono::steady_clock::now();});
  auto reference_sub=node->create_subscription<hb_robot_interfaces::msg::WorkpieceReference>(
    "/workpiece_reference",rclcpp::QoS(1).transient_local(),[context](hb_robot_interfaces::msg::WorkpieceReference::ConstSharedPtr m){
      std::lock_guard<std::mutex> lock(context->mutex);context->estimate=m->estimate;});
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<CaptureEnvironment>("CaptureCutEnvironment",[=](const auto& n,const auto& c){
    return std::make_unique<CaptureEnvironment>(n,c,node,context);});
  auto condition=[&](const std::string& name,auto predicate){
    factory.registerSimpleCondition(name,[predicate](BT::TreeNode&){
      return predicate()?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
    });
  };
  condition("ReferenceExists",[=]{
    std::lock_guard<std::mutex> lock(context->mutex);
    return std::chrono::steady_clock::now()-context->received<=500ms && context->state.scene_valid &&
      !context->estimate.profile_name.empty() && context->state.scene_id==
          std::to_string(rclcpp::Time(context->estimate.header.stamp).nanoseconds());
  });
  condition("WorkpiecePoseValid",[=]{return recovery::geometryPermitted(context->snapshot());});
  condition("ValidateTaskState",[=]{
    const auto state=context->snapshot();
    if(recovery::geometryPermitted(state) && recovery::sameVersions(state,context->requested_plan_state)) return true;
    RCLCPP_WARN(node->get_logger(),"Execution blocked for candidate %s (requires live permission and matching versions): %s",
                context->plan_id.c_str(),state.status_message.c_str());
    return false;
  });
  condition("FaultActive",[=]{const auto s=context->snapshot();
    return s.process_fault_active || s.motion_fault_active || (context->executed && !context->preparing_candidate &&
        (!s.supervision_valid || !s.imu_stable));});
  condition("KerfVerified",[=]{return context->verdict.status==recovery::State::VERIFIED;});
  condition("ResidualDetected",[=]{return context->verdict.status==recovery::State::RESIDUAL_MATERIAL;});
  condition("KerfInconclusive",[=]{return context->verdict.status==recovery::State::INCONCLUSIVE;});
  condition("ValidateKerf",[=]{
    auto state=context->snapshot();
    // The action result carries the new execution epoch; require the state manager
    // to have observed it too. A missed progress event must not validate old evidence.
    context->verdict=recovery::evaluate(context->evidence,state,context->cut_id);
    return true;
  });
  factory.registerBuilder<AwaitGeometry>("AwaitGeometry",[=](const auto& n,const auto& c){
    return std::make_unique<AwaitGeometry>(n,c,context);});
  factory.registerBuilder<AwaitGeometry>("AwaitPlanningReference",[=](const auto& n,const auto& c){
    return std::make_unique<AwaitGeometry>(n,c,context,true);});
  action<Inspect>(factory,"InspectProfile",node,"inspect_profile",[](auto& goal){
    goal.max_views=10;goal.samples_per_view=3;goal.depth_range=0.75;goal.camera_standoff=0.4;return true;
  },[=](const auto& result){
    context->inspection_reference_id=std::to_string(rclcpp::Time(result.profile_pose.header.stamp).nanoseconds());
    return result.success && std::isfinite(result.rms_error) && result.rms_error<0.01 &&
      result.inlier_fraction>0.3 && result.coverage>0.2;
  });
  action<Revalidate>(factory,"RevalidateWorkpiece",node,"revalidate_workpiece",[=](auto& goal){
    goal.expected_pose_version=context->snapshot().pose_version;return true;
  },[=](const auto& result){
    context->inspection_reference_id=std::to_string(rclcpp::Time(result.estimate.header.stamp).nanoseconds());
    return result.localized && !result.poorly_constrained;
  });
  auto planGoal=[=](auto& goal,bool recut) {
    const auto state=context->snapshot();
    if(!state.scene_valid || (recut && !recovery::geometryPermitted(state))) return false;
    if (!context->captured_scene || !recovery::sameVersions(state, *context->captured_scene) ||
        state.cut_environment_id != context->captured_scene->cut_environment_id) return false;
    goal.standoff=standoff;
    goal.pos_tol=pos_tol;
    goal.ang_tol=ang_tol;
    goal.cut_id=context->cut_id;
    context->requested_plan_state=state;
    goal.geometry_version=state.geometry_version;
    goal.pose_version=state.pose_version;
    goal.scene_version=state.scene_version;
    goal.execute = false;
    goal.plan_id.clear();
    if(recut) {
      if(context->verdict.status!=recovery::State::RESIDUAL_MATERIAL) return false;
      goal.parent_plan_id=context->parent_id;
      // Merge bins into one bounded envelope per feature. No interval crosses a
      // disconnected feature; the server independently enforces one recut attempt.
      std::map<std::string,recovery::Interval> merged;
      for(const auto& i:context->verdict.residuals) {
        auto it=merged.find(i.segment_id);
        if(it==merged.end()) merged[i.segment_id]=i;
        else {it->second.s0=std::min(it->second.s0,i.s0);it->second.s1=std::max(it->second.s1,i.s1);}
      }
      for(const auto& pair:merged) goal.intervals.push_back(pair.second);
      if(goal.intervals.empty()) return false;
    }
    context->preparing_candidate=true;
    return true;
  };
  auto planResult=[=](const auto& result){
        RCLCPP_INFO(
        node->get_logger(),
        "PlanCut result: success=%d executed=%d interrupted=%d "
        "plan_id='%s' message='%s'",
        result.success,
        result.executed,
        result.interrupted,
        result.plan_id.c_str(),
        result.message.c_str()
    );
    if(!result.success || result.plan_id.empty() || result.executed) return false;
    context->plan_id=result.plan_id;
    if(context->parent_id.empty()) {
      context->parent_id=result.plan_id;context->required_segments=result.required_segments;
    }
    return true;
  };
  action<Cut>(factory,"PlanCut",node,"cut_profile",[=](auto& g){return planGoal(g,false);},planResult);
  action<Cut>(factory,"PlanRecut",node,"cut_profile",[=](auto& g){return planGoal(g,true);},planResult);
  auto executeGoal=[=](auto& goal) {
    context->preparing_candidate=false;
    if(!recovery::geometryPermitted(context->snapshot())) return false;
    goal.standoff=standoff;goal.pos_tol=pos_tol;goal.ang_tol=ang_tol;
    goal.cut_id=context->cut_id;goal.plan_id=context->plan_id;goal.execute=true;
    return !goal.plan_id.empty();
  };
  auto executeResult=[=](const auto& result){
    context->execution_version=result.execution_version;context->executed=result.executed;
    context->plan_id.clear(); // Never retick/resume a consumed plan.
    return result.success && result.executed && !result.interrupted;
  };
  action<Cut>(factory,"ExecuteCut",node,"cut_profile",executeGoal,executeResult);
  action<Cut>(factory,"ExecuteRecut",node,"cut_profile",executeGoal,executeResult);
  action<Kerf>(factory,"InspectKerf",node,"inspect_cut",[=](auto& goal) {
    const auto state=context->snapshot();
    if(!recovery::geometryPermitted(state) || ++context->inspection_attempts>3) return false;
    {std::lock_guard<std::mutex> lock(context->mutex);goal.estimate=context->estimate;}
    goal.cut_id=context->cut_id;goal.required_segments=context->required_segments;
    goal.geometry_version=state.geometry_version;goal.pose_version=state.pose_version;
    goal.execution_version=context->execution_version;
    goal.samples_per_viewpoint=3;
    // A second bounded pass changes standoff rather than repeating identical views.
    goal.camera_standoff=context->inspection_attempts==1?0.30:0.35;
    return true;
  },[=](const auto& result){context->evidence=result.evidence;return result.success;});
  action<Escape>(factory,"Escape",node,"escape",[](auto& goal){
    // Automatic fault handling holds position. A separately authorized recovery
    // request may enable a validated retreat once localization is trustworthy.
    goal.allow_retreat=false;return true;
  },[=](const auto& result){
    RCLCPP_WARN(node->get_logger(),"Escape outcome %u: %s; off=%d stationary=%d",
      result.status,result.message.c_str(),result.torch_confirmed_off,result.robot_stationary);
    return result.torch_confirmed_off && result.robot_stationary;
  });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&]{executor.spin();});
  int code=1;
  try {
    const auto file=node->declare_parameter<std::string>("tree_file","");
    auto tree=factory.createTreeFromFile(file);

    /// monitoring in groot:
    constexpr unsigned max_msg_per_second = 25;
    constexpr unsigned publisher_port = 1666;
    constexpr unsigned server_port = 1667;

    BT::PublisherZMQ groot_publisher(
      tree,
      max_msg_per_second,
      publisher_port,
      server_port
    );
    RCLCPP_INFO(node->get_logger(), "groot monitoring enabled, PUB: %u, SERVER: %u", publisher_port, server_port);
    BT::NodeStatus status=BT::NodeStatus::RUNNING;
    while(rclcpp::ok() && status==BT::NodeStatus::RUNNING) {
      status=tree.tickRoot();
      std::this_thread::sleep_for(50ms);
    }
    tree.haltTree();
    code=status==BT::NodeStatus::SUCCESS?0:1;
    RCLCPP_INFO(node->get_logger(), "Tree '%s' finished with %s", file.c_str(),
        status == BT::NodeStatus::RUNNING ? "INTERRUPTED" : (code ? "FAILURE" : "SUCCESS"));
  } catch(const std::exception& e){
    RCLCPP_ERROR(node->get_logger(),"%s",e.what());}
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return code;
}
