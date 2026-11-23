#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include "hb_robot_skills/motion/torch_controller.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Clock = std::chrono::steady_clock;

class CutHeightDebugNode final : public rclcpp::Node {
public:
  CutHeightDebugNode() : Node("cut_height_debug_node") {
    group_ = declare_parameter<std::string>("planning_group", "manipulator");
    tcp_ = declare_parameter<std::string>("tcp_link", "ur10e_torch_link");
    axis_ = declare_parameter<std::string>("axis", "x");
    distance_ = declare_parameter<double>("distance_m", 0.05);
    vel_ = declare_parameter<double>("velocity_scaling", 0.05);
    acc_ = declare_parameter<double>("acceleration_scaling", 0.01);
    plan_time_ = declare_parameter<double>("planning_time_s", 2.0);
    execute_ = declare_parameter<bool>("execute", false);
    live_ = declare_parameter<bool>("confirm_live_test", false);
    ignition_enabled_ = declare_parameter<bool>("ignition_enabled", false);
    tracking_enabled_ = declare_parameter<bool>("enable_voltage_tracking", true);
    dwell_ms_ = declare_parameter<int>("pierce_dwell_ms", 0);
    status_timeout_ms_ = declare_parameter<int>("height_status_timeout_ms", 500);
    prepare_timeout_ms_ = declare_parameter<int>("prepare_timeout_ms", 5000);
    search_timeout_ms_ = declare_parameter<int>("search_timeout_ms", 8000);
    home_timeout_ms_ = declare_parameter<int>("home_timeout_ms", 20000);
    task_timeout_ms_ = declare_parameter<int>("task_timeout_ms", 1000);
    const auto torch_topic = declare_parameter<std::string>("torch_status_topic", "/cut_height_debug/torch_status");
    const auto height_topic = declare_parameter<std::string>("height_status_topic", "/torch_height/status_full");
    TorchIOConfig cfg;
    cfg.io_topic = declare_parameter<std::string>("io_topic", "/io_and_status_controller/io_states");
    cfg.set_io_service = declare_parameter<std::string>("io_service", "/io_and_status_controller/set_io");
    cfg.torch_output_pin = declare_parameter<int>("output_pin", 1);
    cfg.transfer_input_pin = declare_parameter<int>("transfer_pin", 0);
    cfg.transfer_active_high = declare_parameter<bool>("transfer_active_high", true);
    cfg.output_timeout_s = declare_parameter<double>("io_timeout_s", 1.0);
    cfg.ignition_enabled = ignition_enabled_;
    torch_ = std::make_unique<TorchController>(*this, cfg);
    torch_->setFaultCallback([this](const std::string& why) {
      RCLCPP_ERROR(get_logger(), "Torch fault: %s", why.c_str());
      stop_.store(true);
      if (move_) move_->stop();
    });
    status_pub_ = create_publisher<hb_robot_interfaces::msg::TorchStatus>(torch_topic, 10);
    status_timer_ = create_wall_timer(50ms, [this] { publishTorch(); });
    height_sub_ = create_subscription<std_msgs::msg::String>(height_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveHeight(msg->data); });
    task_sub_ = create_subscription<hb_robot_interfaces::msg::TaskState>(
      "/task_state", rclcpp::QoS(1).transient_local(),
      [this](hb_robot_interfaces::msg::TaskState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        task_ok_ = msg->supervision_valid && msg->imu_stable && msg->scene_valid &&
          msg->workpiece_pose_valid && !msg->motion_fault_active && !msg->process_fault_active;
        task_at_ = Clock::now();
      });
    home_ = create_client<std_srvs::srv::Trigger>("/torch_height/home");
    prep_ = create_client<std_srvs::srv::Trigger>("/torch_height/prepare");
    search_ = create_client<std_srvs::srv::Trigger>("/torch_height/arc_search");
    stop_height_ = create_client<std_srvs::srv::Trigger>("/torch_height/stop");
    tracking_ = create_client<std_srvs::srv::SetBool>("/torch_height/set_tracking_enabled");
    monitor_ = create_wall_timer(20ms, [this] { monitor(); });
  }

  void run() {
    if (axis_ != "x" && axis_ != "y" && axis_ != "z") throw std::runtime_error("axis must be x/y/z");
    if (!std::isfinite(distance_) || distance_ == 0 || std::abs(distance_) > 0.1)
      throw std::runtime_error("distance_m must be nonzero and within +/-0.1");
    if (!(vel_ > 0 && vel_ <= 1 && acc_ > 0 && acc_ <= 1) || plan_time_ <= 0 || dwell_ms_ < 0 ||
        status_timeout_ms_ < 100 || prepare_timeout_ms_ <= 0 || search_timeout_ms_ <= 0 || home_timeout_ms_ <= 0)
      throw std::runtime_error("invalid limits/timing");
    if (execute_ && (!live_ || !ignition_enabled_))
      throw std::runtime_error("execution requires confirm_live_test=true AND ignition_enabled=true");

    // All planning and collision checks finish BEFORE the torch/height handoff.
    move_ = std::make_shared<MoveGroup>(shared_from_this(), group_);
    move_->setEndEffectorLink(tcp_);
    move_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_->setPlannerId("LIN");
    move_->setPlanningTime(plan_time_);
    move_->setNumPlanningAttempts(1);
    move_->setMaxVelocityScalingFactor(vel_);
    move_->setMaxAccelerationScalingFactor(acc_);
    move_->setStartStateToCurrentState();
    auto target = move_->getCurrentPose(tcp_).pose;
    if (axis_ == "x") target.position.x += distance_;
    if (axis_ == "y") target.position.y += distance_;
    if (axis_ == "z") target.position.z += distance_;
    move_->setPoseTarget(target, tcp_);
    MoveGroup::Plan plan;
    const auto planned = move_->plan(plan);
    move_->clearPoseTargets();
    if (planned != moveit::core::MoveItErrorCode::SUCCESS ||
        plan.trajectory_.joint_trajectory.points.empty()) throw std::runtime_error("Pilz LIN plan failed");
    RCLCPP_INFO(get_logger(), "Cached LIN plan: %zu points; no more planning after handoff",
      plan.trajectory_.joint_trajectory.points.size());
    if (!execute_) return;

    // Verify planned trajectory against a fresh scene, without changing the ACM.
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    psm->startStateMonitor();
    psm->startSceneMonitor("/monitored_planning_scene");
    psm->startWorldGeometryMonitor();
    if (!psm->requestPlanningSceneState("/get_planning_scene"))
      throw std::runtime_error("planning scene refresh failed");
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm);
      if (!scene) throw std::runtime_error("no planning scene");
      moveit::core::RobotState start(move_->getRobotModel());
      moveit::core::robotStateMsgToRobotState(plan.start_state_, start);
      robot_trajectory::RobotTrajectory traj(move_->getRobotModel(), group_);
      traj.setRobotTrajectoryMsg(start, plan.trajectory_);
      for (size_t i=0; i<traj.getWayPointCount(); ++i) {
        if (!traj.getWayPoint(i).satisfiesBounds() || scene->isStateColliding(traj.getWayPoint(i), group_))
          throw std::runtime_error("trajectory waypoint collision/bounds failure: " + std::to_string(i));
      }
    }
    if (!safeTask() || !torch_->hasFreshIO() || torch_->hasFault())
      throw std::runtime_error("supervision/torch preflight failed");
    auto t = torch_->snapshot();
    if (t.command_on || t.output_on != std::optional<bool>(false) ||
        t.transfer_detected != std::optional<bool>(false))
      throw std::runtime_error("torch must be confirmed OFF before HOME");

    active_.store(true);
    try {
      ensureHome();
      auto h = height();
      const uint32_t session = h.session;
      phase_.store(1);
      auto prep_id = command(prep_, "PREP");
      waitDone(session, prep_id, "READY", prepare_timeout_ms_);
      phase_.store(2);
      torch_->setActiveSegment("cut_height_debug");
      if (torch_->requestIgnition(1500ms, [this] {return stop_.load();}) != TorchCommandResult::CONFIRMED)
        throw std::runtime_error("ignition not confirmed");
      phase_.store(3);
      auto search_id = command(search_, "SEARCH");
      waitDone(session, search_id, "HOLD", search_timeout_ms_);
      if (torch_->waitForArcTransfer(250ms, [this]{return stop_.load();}) !=
          TorchController::WaitResult::CONFIRMED)
        throw std::runtime_error("transfer not confirmed");
      if (!torch_->markCutting()) throw std::runtime_error("markCutting failed");
      const auto dwell_end = Clock::now() + std::chrono::milliseconds(dwell_ms_);
      while (Clock::now() < dwell_end) { ensureSafe(); std::this_thread::sleep_for(10ms); }
      if (tracking_enabled_) setTracking(true);
      phase_.store(4);
      ensureSafe();
      if (move_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("cached LIN execution failed");
      ensureSafe();
      if (tracking_enabled_) setTracking(false);
      phase_.store(5);
      if (torch_->requestShutdown(1500ms) != TorchCommandResult::CONFIRMED)
        throw std::runtime_error("torch OFF unconfirmed");
      if (!torch_->finishCut()) throw std::runtime_error("torch finishCut failed");
      ensureHome();
      torch_->setActiveSegment("");
      phase_.store(0);
      active_.store(false);
      RCLCPP_INFO(get_logger(), "Height-controlled test cut complete");
    } catch (...) {
      stop_.store(true);
      if (move_) move_->stop();
      torch_->requestOutputOff();
      requestHeightStop();
      (void)torch_->requestShutdown(1500ms);
      torch_->setActiveSegment("");
      phase_.store(0);
      active_.store(false);
      throw; // Remain stopped; do not auto-home a faulted axis.
    }
  }
private:
  struct Height {
    std::string state;
    int32_t position{0};
    bool sw{false}, referenced{false}, configured{false}, calibration{false}, tracking{false};
    int fault{0}, result{0};
    uint32_t session{0}, active_id{0}, done_id{0};
    Clock::time_point received{};
  };
  void receiveHeight(const std::string& data) {
    std::istringstream in(data);
    std::vector<std::string> f;
    std::string part;
    while (std::getline(in, part, ',')) f.push_back(part);
    if (f.size()!=12) return;
    try {
      auto n = [&](size_t i)->int64_t { size_t used=0; auto v=std::stoll(f.at(i),&used); if(used!=f[i].size()) throw std::runtime_error("field"); return v; };
      Height h;
      h.state=f[0]; h.position=static_cast<int32_t>(n(1)); h.sw=n(2)==1;
      h.fault=static_cast<int>(n(3)); h.referenced=n(4)==1;
      h.session=static_cast<uint32_t>(n(5)); h.active_id=static_cast<uint32_t>(n(6));
      h.done_id=static_cast<uint32_t>(n(7)); h.result=static_cast<int>(n(8));
      h.calibration=n(9)==1; h.configured=n(10)==1; h.tracking=n(11)==1;
      h.received=Clock::now();
      std::lock_guard<std::mutex> lk(mu_); height_=std::move(h);
    } catch (...) { /* Invalid frames do not refresh freshness. */ }
  }
  Height height() const { std::lock_guard<std::mutex> lk(mu_); return height_; }
  bool valid(const Height& h) const {
    return h.received != Clock::time_point{} &&
      Clock::now()-h.received <= std::chrono::milliseconds(status_timeout_ms_) &&
      h.configured && !h.calibration && h.session && !h.fault &&
      h.state!="OFFLINE" && h.state!="FAULT" && h.state!="PAUSED";
  }
  bool safeTask() const {
    std::lock_guard<std::mutex> lk(mu_);
    return task_at_ != Clock::time_point{} &&
      Clock::now()-task_at_ < std::chrono::milliseconds(task_timeout_ms_) && task_ok_;
  }
  void ensureSafe() const {
    if (stop_.load() || !rclcpp::ok() || !safeTask() || torch_->hasFault() || !torch_->hasFreshIO())
      throw std::runtime_error("stop/supervision/torch fault");
    auto h=height();
    if (!valid(h) || !h.referenced) throw std::runtime_error("height fault/stale/unreferenced");
    if (phase_.load()==4 && h.state!="HOLD" && h.state!="TRACK")
      throw std::runtime_error("height left HOLD/TRACK during cut");
    if (phase_.load()==4 && tracking_enabled_ && !h.tracking)
      throw std::runtime_error("height tracking lost");
  }
  void monitor() {
    if (!active_.load() || stop_.load()) return;
    auto h=height();
    const bool invalid_height = phase_.load()!=0 && (!valid(h) || !h.referenced);
    if (!safeTask() || !torch_->hasFreshIO() || torch_->hasFault() || invalid_height ||
        (phase_.load()==4 && (h.state!="HOLD" && h.state!="TRACK" ||
          tracking_enabled_ && !h.tracking))) {
      if (!stop_.exchange(true)) {
        RCLCPP_ERROR(get_logger(), "Supervision stop: phase=%d height=%s fault=%d", phase_.load(), h.state.c_str(), h.fault);
        torch_->requestOutputOff();
        requestHeightStop();
        if (move_) move_->stop();
      }
    }
  }
  uint32_t command(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client, const char* name) {
    if (!client->wait_for_service(500ms)) throw std::runtime_error(std::string(name)+" unavailable");
    auto pending=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline=Clock::now()+3s;
    while (Clock::now()<deadline && rclcpp::ok() && !stop_.load()) {
      if (pending.wait_for(20ms)!=std::future_status::ready) continue;
      auto reply=pending.get();
      if (!reply->success) throw std::runtime_error(std::string(name)+" rejected: "+reply->message);
      std::istringstream ss(reply->message); std::string token; ss>>token;
      if (token.rfind("id=",0)!=0) throw std::runtime_error("bad height ACK: "+reply->message);
      size_t used=0; auto id=std::stoull(token.substr(3),&used);
      if (used!=token.size()-3 || id==0 || id>UINT32_MAX) throw std::runtime_error("bad command id");
      return static_cast<uint32_t>(id);
    }
    client->remove_pending_request(pending);
    throw std::runtime_error(std::string(name)+" ACK timeout/stop");
  }
  void waitDone(uint32_t session, uint32_t id, const std::string& state, int timeout_ms) {
    const auto deadline=Clock::now()+std::chrono::milliseconds(timeout_ms);
    while (Clock::now()<deadline && rclcpp::ok() && !stop_.load()) {
      auto h=height();
      if (!valid(h) || h.session!=session) throw std::runtime_error("height invalid/session changed");
      if (h.done_id==id) {
        if (h.result!=1 || h.state!=state || !h.referenced) throw std::runtime_error("height command completed unsuccessfully");
        if (state=="HOME" && (h.position!=0 || !h.sw || h.tracking)) throw std::runtime_error("HOME inconsistent");
        if (state!="HOME" && (h.position>=0 || h.sw || h.tracking)) throw std::runtime_error("height end position inconsistent");
        return;
      }
      if (h.done_id>id) throw std::runtime_error("height command superseded");
      std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error("height completion timeout/stop");
  }
  void setTracking(bool enabled) {
    if (!tracking_->wait_for_service(500ms)) throw std::runtime_error("tracking service unavailable");
    auto before=height(); if (!valid(before)) throw std::runtime_error("height invalid before tracking");
    auto req=std::make_shared<std_srvs::srv::SetBool::Request>(); req->data=enabled;
    auto future=tracking_->async_send_request(req);
    if (future.wait_for(1500ms)!=std::future_status::ready) {
      tracking_->remove_pending_request(future); throw std::runtime_error("tracking ACK timeout");
    }
    if (!future.get()->success) throw std::runtime_error("tracking rejected");
    auto after_ack=Clock::now(); auto deadline=after_ack+1500ms;
    while (Clock::now()<deadline && !stop_.load()) {
      auto h=height();
      if (!valid(h) || h.session!=before.session) throw std::runtime_error("tracking session/fault");
      if (h.received>after_ack && h.tracking==enabled &&
          (enabled ? (h.state=="HOLD" || h.state=="TRACK") : h.state!="TRACK")) return;
      std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error("tracking state confirmation timeout");
  }
  void ensureHome() {
    auto h=height();
    if (!valid(h)) throw std::runtime_error("height invalid before HOME");
    auto t=torch_->snapshot();
    if (!torch_->hasFreshIO() || torch_->hasFault() || t.command_on ||
        t.output_on!=std::optional<bool>(false) || t.transfer_detected!=std::optional<bool>(false))
      throw std::runtime_error("torch not confirmed OFF for HOME");
    if (h.state=="HOME" && h.position==0 && h.sw && h.referenced && !h.tracking) return;
    if (h.tracking) setTracking(false);
    h=height();
    if (!valid(h) || (h.state!="UNKNOWN" && h.state!="HOME" && h.state!="READY" && h.state!="HOLD"))
      throw std::runtime_error("HOME invalid starting state");
    auto id=command(home_,"HOME");
    waitDone(h.session,id,"HOME",home_timeout_ms_);
  }
  void requestHeightStop() {
    if (!stop_height_->service_is_ready()) return;
    stop_height_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try { if (!future.get()->success) RCLCPP_ERROR(get_logger(),"height STOP rejected"); }
        catch (...) { RCLCPP_ERROR(get_logger(),"height STOP response failed"); }
      });
  }
  void publishTorch() {
    const auto t=torch_->snapshot();
    hb_robot_interfaces::msg::TorchStatus msg;
    msg.stamp=t.stamp; msg.io_connected=torch_->hasFreshIO(); msg.command_on=t.command_on;
    msg.output_on=t.output_on.value_or(false); msg.transfer_detected=t.transfer_detected.value_or(false);
    msg.analog_valid=t.analog_value.has_value(); msg.analog_input_value=t.analog_value.value_or(0.0);
    msg.analog_domain=static_cast<uint8_t>(t.analog_domain.value_or(0));
    msg.arc_voltage_valid=t.arc_voltage_v.has_value(); msg.arc_voltage_v=t.arc_voltage_v.value_or(0.0);
    msg.ready=false; msg.fault=torch_->hasFault(); status_pub_->publish(msg);
  }
  std::string group_,tcp_,axis_;
  double distance_{},vel_{},acc_{},plan_time_{};
  bool execute_{},live_{},ignition_enabled_{},tracking_enabled_{};
  int dwell_ms_{},status_timeout_ms_{},prepare_timeout_ms_{},search_timeout_ms_{},home_timeout_ms_{},task_timeout_ms_{};
  mutable std::mutex mu_;
  Height height_;
  bool task_ok_{false}; Clock::time_point task_at_{};
  std::atomic<bool> stop_{false},active_{false};
  std::atomic<int> phase_{0};
  std::unique_ptr<TorchController> torch_;
  std::shared_ptr<MoveGroup> move_;
  rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr height_sub_;
  rclcpp::Subscription<hb_robot_interfaces::msg::TaskState>::SharedPtr task_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_,monitor_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_,prep_,search_,stop_height_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracking_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<CutHeightDebugNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinning([&]{executor.spin();});
  int rc=0;
  try {node->run();} catch(const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"Height cut debug failed: %s",e.what()); rc=1;
  }
  executor.cancel(); spinning.join(); rclcpp::shutdown(); return rc;
}
