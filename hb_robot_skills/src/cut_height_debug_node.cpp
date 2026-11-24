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
    const auto torch_topic = declare_parameter<std::string>("torch_status_topic", "/cut_profile/torch_status_debug");
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
    });
    status_pub_ = create_publisher<hb_robot_interfaces::msg::TorchStatus>(torch_topic, 10);
    status_timer_ = create_wall_timer(50ms, [this] { publishTorch(); });
    height_sub_ = create_subscription<std_msgs::msg::String>(height_topic, rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveHeight(msg->data); });
    home_ = create_client<std_srvs::srv::Trigger>("/torch_height/home");
    prep_ = create_client<std_srvs::srv::Trigger>("/torch_height/prepare");
    search_ = create_client<std_srvs::srv::Trigger>("/torch_height/arc_search");
    stop_height_ = create_client<std_srvs::srv::Trigger>("/torch_height/stop");
    tracking_ = create_client<std_srvs::srv::SetBool>("/torch_height/set_tracking_enabled");
    reset_ = create_client<std_srvs::srv::Trigger>("/torch_height/reset_fault");
    bench_ = declare_parameter<bool>("confirm_bench_clearance", false);
    if (execute_ || live_ || ignition_enabled_)
      throw std::runtime_error("This bench debugger does not authorize live cutting");
    if (status_timeout_ms_ < 100 || home_timeout_ms_ <= 0 || prepare_timeout_ms_ <= 0)
      throw std::runtime_error("Invalid height timeouts");
    ops_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    for (const std::string op : {"home", "prepare", "reset_fault", "recover", "plan"}) {
      services_.push_back(create_service<std_srvs::srv::Trigger>(
        "~/" + op, [this, op](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          operate(op, *r);
        }, rmw_qos_profile_services_default, ops_group_));
    }
    services_.push_back(create_service<std_srvs::srv::Trigger>("~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        stop_.store(true);
        torch_->requestOutputOff();
        requestHeightStop();
        r->success = true;
        r->message = "STOP requested; inspect terminal status (not completion ACK)";
      }, rmw_qos_profile_services_default, stop_group_));
    monitor_ = create_wall_timer(20ms, [this] {
      if (active_.load() && (!offConfirmed() || !communicationValid(height()))) {
        stop_.store(true);
        torch_->requestOutputOff();
        requestHeightStop();
      }
    });
    RCLCPP_INFO(get_logger(), "Persistent bench debugger ready; ignition disabled");
  }
private:
  bool offConfirmed() const {
    const auto t = torch_->snapshot();
    return torch_->hasFreshIO() && !torch_->hasFault() && !t.command_on &&
      t.output_on == std::optional<bool>(false) &&
      t.transfer_detected == std::optional<bool>(false);
  }
  void checkBench() const {
    if (!bench_ || !offConfirmed() || stop_.load() || !rclcpp::ok())
      throw std::runtime_error("Bench clearance, confirmed OFF and no STOP required");
  }
  void operate(const std::string& op, std_srvs::srv::Trigger::Response& r) {
    r.success = false;
    // Operations are serialized by ops_group_. STOP is independent.
    try {
      if (op == "plan") {
        planOnly();
      } else {
        if (!bench_ || !offConfirmed()) throw std::runtime_error("Bench clearance / fresh OFF required");
        auto h = height();
        if (!communicationValid(h) || h.active_id || h.tracking)
          throw std::runtime_error("Height communication invalid or axis not idle");
        if (stop_pending_.load()) throw std::runtime_error("STOP acknowledgement pending");
        // Only explicit recovery can rearm after STOP; never resume a cut.
        if (op == "recover" || op == "reset_fault") {
          if (h.state != "FAULT" && h.state != "UNKNOWN")
            throw std::runtime_error("Reset requires FAULT or UNKNOWN");
          stop_.store(false);
        }
        checkBench();
        active_.store(true);
        if (op == "recover" || op == "reset_fault") resetHeight();
        if (op == "home" || op == "recover") ensureHome();
        if (op == "prepare") {
          h = height();
          if (!valid(h) || h.state != "HOME" || h.position != 0 || !h.sw || !h.referenced)
            throw std::runtime_error("PREP requires verified HOME");
          auto id = command(prep_, "PREP");
          waitDone(h.session, id, "READY", prepare_timeout_ms_);
        }
        checkBench();
      }
      r.success = true; r.message = op + " completed";
    } catch (const std::exception& e) {
      if (active_.load()) {
        stop_.store(true); torch_->requestOutputOff(); requestHeightStop();
      }
      r.message = e.what();
      RCLCPP_ERROR(get_logger(), "%s: %s", op.c_str(), e.what());
    }
    active_.store(false);
  }
  void resetHeight() {
    checkBench();
    auto before = height();
    (void)command(reset_, "RESET_FAULT");
    const auto ack = Clock::now();
    const auto deadline = ack + 2s;
    while (Clock::now() < deadline) {
      checkBench();
      auto h = height();
      if (!communicationValid(h) || h.session != before.session)
        throw std::runtime_error("Reset session lost");
      if (h.received > ack && h.state == "UNKNOWN" && !h.fault &&
          !h.referenced && !h.active_id && !h.tracking) return;
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("RESET_FAULT status not confirmed");
  }
  void planOnly() {
    if (!offConfirmed()) throw std::runtime_error("Fresh OFF required");
    if ((axis_ != "x" && axis_ != "y" && axis_ != "z") ||
        !std::isfinite(distance_) || distance_ == 0 || std::abs(distance_) > 0.1 ||
        !(vel_ > 0 && vel_ <= 1 && acc_ > 0 && acc_ <= 1 && plan_time_ > 0))
      throw std::runtime_error("Invalid planning parameters");
    auto move = std::make_shared<MoveGroup>(shared_from_this(), group_);
    move->setEndEffectorLink(tcp_);
    move->setPlanningPipelineId("pilz_industrial_motion_planner");
    move->setPlannerId("LIN");
    move->setPlanningTime(plan_time_);
    move->setNumPlanningAttempts(1);
    move->setMaxVelocityScalingFactor(vel_);
    move->setMaxAccelerationScalingFactor(acc_);
    move->setStartStateToCurrentState();
    auto target = move->getCurrentPose(tcp_).pose;
    if (axis_ == "x") target.position.x += distance_;
    if (axis_ == "y") target.position.y += distance_;
    if (axis_ == "z") target.position.z += distance_;
    move->setPoseTarget(target, tcp_);
    MoveGroup::Plan plan;
    if (move->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
        plan.trajectory_.joint_trajectory.points.empty())
      throw std::runtime_error("LIN planning failed");
    RCLCPP_INFO(get_logger(), "Plan-only: %zu points; no execution", plan.trajectory_.joint_trajectory.points.size());
  }

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
      for (auto i : {2,4,9,10,11}) if (n(i)<0 || n(i)>1) throw std::runtime_error("boolean field");
      for (auto i : {5,6,7}) if (n(i)<0 || n(i)>UINT32_MAX) throw std::runtime_error("id field");
      if (n(1)<INT32_MIN || n(1)>INT32_MAX || n(3)<0 || n(3)>255 || n(8)<0 || n(8)>2)
        throw std::runtime_error("numeric field");
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
  bool communicationValid(const Height& h) const {
    return h.received != Clock::time_point{} &&
      Clock::now()-h.received <= std::chrono::milliseconds(status_timeout_ms_) &&
      h.configured && !h.calibration && h.session && h.state != "OFFLINE";
  }
  bool valid(const Height& h) const {
    return communicationValid(h) && !h.fault && h.state != "FAULT" && h.state != "PAUSED";
  }
  uint32_t command(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client, const char* name) {
    checkBench();
    if (!client->wait_for_service(500ms)) throw std::runtime_error(std::string(name)+" unavailable");
    auto pending=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline=Clock::now()+3s;
    while (Clock::now()<deadline && rclcpp::ok() && !stop_.load()) {
      if (!offConfirmed()) break;
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
      checkBench();
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
  void ensureHome() {
    auto h=height();
    if (!valid(h)) throw std::runtime_error("height invalid before HOME");
    auto t=torch_->snapshot();
    if (!torch_->hasFreshIO() || torch_->hasFault() || t.command_on ||
        t.output_on!=std::optional<bool>(false) || t.transfer_detected!=std::optional<bool>(false))
      throw std::runtime_error("torch not confirmed OFF for HOME");
    if (h.state=="HOME" && h.position==0 && h.sw && h.referenced && !h.tracking) return;
    if (h.tracking) throw std::runtime_error("Tracking must be disabled");
    h=height();
    if (!valid(h) || (h.state!="UNKNOWN" && h.state!="HOME" && h.state!="READY" && h.state!="HOLD"))
      throw std::runtime_error("HOME invalid starting state");
    auto id=command(home_,"HOME");
    waitDone(h.session,id,"HOME",home_timeout_ms_);
  }
  void requestHeightStop() {
    if (!stop_height_->service_is_ready() || stop_pending_.exchange(true)) return;
    try {
      stop_height_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
          try {
            if (!f.get()->success) RCLCPP_ERROR(get_logger(), "STOP rejected; inspect hardware");
          } catch (...) { RCLCPP_ERROR(get_logger(), "STOP response failed"); }
          stop_pending_.store(false);
        });
    } catch (...) { stop_pending_.store(false); }
  }
  void publishTorch() {
    const auto t=torch_->snapshot();
    hb_robot_interfaces::msg::TorchStatus msg;
    msg.stamp=t.stamp; msg.io_connected=torch_->hasFreshIO() && t.output_on.has_value() && t.transfer_detected.has_value(); msg.command_on=t.command_on;
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
  std::atomic<bool> stop_{false},active_{false};
  std::atomic<int> phase_{0};
  std::unique_ptr<TorchController> torch_;
  bool bench_{false};
  std::atomic<bool> stop_pending_{false};
  rclcpp::CallbackGroup::SharedPtr ops_group_, stop_group_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_,monitor_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_,prep_,search_,stop_height_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracking_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<CutHeightDebugNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
