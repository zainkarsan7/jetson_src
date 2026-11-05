#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hb_robot_interfaces/recovery.hpp>
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
  recovery::State state;
  hb_robot_interfaces::msg::ProfileEstimate estimate;
  recovery::Evidence evidence;
  std::chrono::steady_clock::time_point received{};
  std::string cut_id,plan_id,parent_id;
  std::vector<std::string> required_segments;
  recovery::Verdict verdict;
  uint64_t execution_version=0;
  unsigned inspection_attempts=0;
  bool executed=false;
  recovery::State snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    auto result=state;
    if(std::chrono::steady_clock::now()-received>500ms) result.supervision_valid=false;
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
    return wrapped.code==rclcpp_action::ResultCode::SUCCEEDED && wrapped.result && finish_(*wrapped.result)?
      BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
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
  AwaitGeometry(const std::string& n,const BT::NodeConfiguration& c,std::shared_ptr<Context> context):
    BT::StatefulActionNode(n,c),context_(context){}
  static BT::PortsList providedPorts(){return {};}
  BT::NodeStatus onStart() override {start_=std::chrono::steady_clock::now();return onRunning();}
  BT::NodeStatus onRunning() override {
    if(recovery::geometryPermitted(context_->snapshot())) return BT::NodeStatus::SUCCESS;
    return std::chrono::steady_clock::now()-start_>3s?BT::NodeStatus::FAILURE:BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
 private:
  std::shared_ptr<Context> context_;std::chrono::steady_clock::time_point start_;
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
  auto condition=[&](const std::string& name,auto predicate){
    factory.registerSimpleCondition(name,[predicate](BT::TreeNode&){
      return predicate()?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
    });
  };
  condition("ReferenceExists",[=]{
    std::lock_guard<std::mutex> lock(context->mutex);return !context->estimate.profile_name.empty();
  });
  condition("WorkpiecePoseValid",[=]{return recovery::geometryPermitted(context->snapshot());});
  condition("ValidateTaskState",[=]{return recovery::geometryPermitted(context->snapshot());});
  condition("FaultActive",[=]{const auto s=context->snapshot();
    return s.process_fault_active || s.motion_fault_active || (context->executed && (!s.supervision_valid || !s.imu_stable));});
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
  action<Inspect>(factory,"InspectProfile",node,"inspect_profile",[](auto& goal){
    goal.max_views=10;goal.samples_per_view=3;goal.depth_range=0.75;goal.camera_standoff=0.4;return true;
  },[=](const auto& result){
    return result.success && std::isfinite(result.rms_error) && result.rms_error<0.01 &&
      result.inlier_fraction>0.3 && result.coverage>0.2;
  });
  action<Revalidate>(factory,"RevalidateWorkpiece",node,"revalidate_workpiece",[=](auto& goal){
    goal.expected_pose_version=context->snapshot().pose_version;return true;
  },[](const auto& result){return result.localized && !result.poorly_constrained;});
  auto planGoal=[=](auto& goal,bool recut) {
    const auto state=context->snapshot();if(!recovery::geometryPermitted(state)) return false;
    goal.standoff=standoff;goal.pos_tol=pos_tol;goal.ang_tol=ang_tol;
    goal.cut_id=context->cut_id;goal.geometry_version=state.geometry_version;
    goal.pose_version=state.pose_version;goal.scene_version=state.scene_version;
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
    return true;
  };
  auto planResult=[=](const auto& result){
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
    RCLCPP_INFO(node->get_logger(),"Recovery tree finished: %s",code?"unverified / intervention required":"separation verified");
  } catch(const std::exception& e){
    RCLCPP_ERROR(node->get_logger(),"%s",e.what());}
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return code;
}
