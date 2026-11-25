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
#include <cstdint>
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
    transfer_test_enabled_ =
    declare_parameter<bool>(
        "enable_transfer_test", false);
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
    cfg.voltage_input_pin = declare_parameter<int>("voltage_input_pin", 0);
    cfg.ignition_enabled = ignition_enabled_;
    torch_ = std::make_unique<TorchController>(*this, cfg);
    torch_->setFaultCallback([this](const std::string& why) {
      RCLCPP_ERROR(get_logger(), "Torch fault: %s", why.c_str());
      stop_.store(true);
    });
    task_sub_ = create_subscription<
    hb_robot_interfaces::msg::TaskState>(
    "/task_state",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](
        hb_robot_interfaces::msg::TaskState::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(task_mu_);
        task_state_ = *msg;
        task_received_ = Clock::now();
    });
    status_pub_ = create_publisher<hb_robot_interfaces::msg::TorchStatus>(torch_topic, 10);
    status_timer_ = create_wall_timer(50ms, [this] { publishTorch(); });
    height_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions height_options;
    height_options.callback_group = height_group_;
    height_sub_ = create_subscription<std_msgs::msg::String>(height_topic, rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveHeight(msg->data); }, height_options);
    home_ = create_client<std_srvs::srv::Trigger>("/torch_height/home");
    prep_ = create_client<std_srvs::srv::Trigger>("/torch_height/prepare");
    search_ = create_client<std_srvs::srv::Trigger>("/torch_height/arc_search");
    stop_height_ = create_client<std_srvs::srv::Trigger>("/torch_height/stop");
    tracking_ = create_client<std_srvs::srv::SetBool>("/torch_height/set_tracking_enabled");
    reset_ = create_client<std_srvs::srv::Trigger>("/torch_height/reset_fault");
    bench_ = declare_parameter<bool>("confirm_bench_clearance", false);
    dry_ = declare_parameter<bool>("confirm_plasma_inhibited", false);
    if (status_timeout_ms_ < 100 ||
    task_timeout_ms_ <= 0 ||
    home_timeout_ms_ <= 0 ||
    prepare_timeout_ms_ <= 0 ||
    search_timeout_ms_ <= 0)
      throw std::runtime_error("Invalid height timeouts");
    ops_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    for (const std::string op : {"home", "prepare", "reset_fault", "recover", "plan", "dry_search"}) {
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
    if (!active_.load())
        return;

    const bool search = search_active_.load();
    const auto h = height();

    const bool unsafe =
    !platformPermitted() ||
    (search
        ? searchUnsafe(h)
        : (!offConfirmed() || !communicationValid(h)));

    // Discard a decision made across a phase transition.
    if (search != search_active_.load())
        return;

    if (active_.load() &&
        (unsafe || stop_.load()) &&
        !stop_dispatched_.exchange(true))
    {
        RCLCPP_ERROR(
            get_logger(),
            "[SUPERVISION STOP] search=%d unsafe=%d "
            "state=%s fault=%d session=%u",
            int(search),
            int(unsafe),
            h.state.c_str(),
            h.fault,
            h.session);

        stop_.store(true);
        torch_->requestOutputOff();
        requestHeightStop();
    }
});
    RCLCPP_INFO(get_logger(), "Persistent hardware debugger ready; robot execution prohibited");
  }
  rclcpp::CallbackGroup::SharedPtr heightGroup() const { return height_group_; }
private:
  struct Height {
    std::string state;
    int32_t position{0};
    bool sw{false}, referenced{false}, configured{false}, calibration{false}, tracking{false};
    int fault{0}, result{0};
    uint32_t session{0}, active_id{0}, done_id{0};
    Clock::time_point received{};
  };

  bool platformPermitted() const {
    std::lock_guard<std::mutex> lock(task_mu_);

    return
        task_received_ != Clock::time_point{} &&
        Clock::now() - task_received_ <=
            std::chrono::milliseconds(task_timeout_ms_) &&
        task_state_.supervision_valid &&
        task_state_.imu_stable &&
        !task_state_.motion_fault_active &&
        !task_state_.process_fault_active;
}
  bool searchUnsafe(const Height& h) const {
    const auto t = torch_->snapshot();

    return
        (!valid(h) && !expectedDryEnd(h)) ||
        !h.referenced ||
        h.tracking ||
        h.session != search_session_.load() ||
        !torch_->hasFreshIO() ||
        torch_->hasFault() ||
        !t.output_on.has_value() ||
        !platformPermitted();
}

  bool offConfirmed() const {
    const auto t = torch_->snapshot();
    return torch_->hasFreshIO() && !torch_->hasFault() && !t.command_on &&
      t.output_on == std::optional<bool>(false) &&
      t.transfer_detected == std::optional<bool>(false);
  }
  void checkBench() const {
    if (!bench_ || !offConfirmed() || stop_.load() || !rclcpp::ok() || !platformPermitted())
      throw std::runtime_error("Bench clearance, confirmed OFF and no STOP required");
  }
  void operate(const std::string& op, std_srvs::srv::Trigger::Response& r) {
    r.success = false;
    // Operations are serialized by ops_group_. STOP is independent.
    try {
      if (op == "test_search" || op == "dry_search" || op == "live_search") {
        if ((op == "dry_search" && !dry_) || (op == "live_search" && !live_))
          throw std::runtime_error("Requested mode not authorized at startup");
        testSearch();
      } else if (op == "plan") {
        planOnly();
      } else {
        if (!bench_ || !offConfirmed()) throw std::runtime_error("Bench clearance / fresh OFF required");
        auto h = height();
        if (!communicationValid(h) || h.active_id || h.tracking)
          throw std::runtime_error("Height communication invalid or axis not idle");
        if (stop_pending_.load()) throw std::runtime_error("STOP acknowledgement pending");
        // Only explicit recovery can rearm after STOP; never resume a cut.
        if (op == "recover" || op == "reset_fault") {
          if (h.state != "FAULT" && h.state != "UNKNOWN" && h.state != "PAUSED")
            throw std::runtime_error("Reset requires FAULT or UNKNOWN");
          stop_.store(false);
          stop_dispatched_.store(false);
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
      r.success = true; r.message = op + (dry_end_.load() ? " completed: expected SEARCH_END; OFF confirmed; explicit recovery required" : " completed");
    } catch (const std::exception& e) {
      if (active_.load()) {
        stop_.store(true); torch_->requestOutputOff(); requestHeightStop();
        if (search_active_.load()) {
           const bool off =
        torch_->requestShutdown(1500ms) ==
        TorchCommandResult::CONFIRMED;

        const bool idle = off && torch_->finishCut();

        if (!idle) {
            RCLCPP_ERROR(
                get_logger(),
                "[SHUTDOWN] Failed to restore IDLE: "
                "off=%d state=%d fault=%d reason=%s",
                int(off),
                static_cast<int>(torch_->state()),
                int(torch_->hasFault()),
                torch_->faultReason().c_str());
        } else {
            RCLCPP_INFO(
                get_logger(),
                "[SHUTDOWN] OFF confirmed; IDLE restored");
        }

        torch_->setActiveSegment("");
        }
      }
      r.message = e.what();
      RCLCPP_ERROR(get_logger(), "%s: %s", op.c_str(), e.what());
    }
    active_.store(false);
    search_active_.store(false);
    dry_end_.store(false);
    search_id_.store(0);
  }
  void checkSearch() const {
    const auto h = height();
    if (!bench_ || stop_.load() || !rclcpp::ok() || searchUnsafe(h))
      throw std::runtime_error("SEARCH supervision failed");
  }
  void testSearch() {
    if ((!dry_ && !live_) || !ignition_enabled_ || !bench_)
      throw std::runtime_error("SEARCH requires explicit mode authorization and clearance");
    checkBench();
    if (torch_->state() != TorchController::State::IDLE) {
    throw std::runtime_error(
        "SEARCH denied: TorchController not IDLE, state=" +
        std::to_string(static_cast<int>(torch_->state())));
}
    auto h = height();
    if (!valid(h) || h.active_id || h.tracking || stop_pending_.load())
      throw std::runtime_error("Height not idle/healthy");
    active_.store(true);
    ensureHome();
    h = height();
    const auto session = h.session;
    RCLCPP_INFO(get_logger(), "[SEARCH TEST] PREP session=%u", session);
    waitDone(session, command(prep_, "PREP"), "READY", prepare_timeout_ms_);
    checkBench();
    search_session_.store(session);
    search_active_.store(true);
    checkSearch();
    torch_->setActiveSegment("bench_search_hardware");
    RCLCPP_INFO(get_logger(), "[SEARCH TEST] IGNITION");
    const auto ignition_result = torch_->requestIgnition(
    1500ms,
    [this] {
        return stop_.load() || !rclcpp::ok();
    });
    if (ignition_result != TorchCommandResult::CONFIRMED) {
        RCLCPP_ERROR(
            get_logger(),
            "[IGNITION] result=%d state=%d stop=%d fault=%d",
            static_cast<int>(ignition_result),
            static_cast<int>(torch_->state()),
            int(stop_.load()),
            int(torch_->hasFault()));

        throw std::runtime_error("Ignition not confirmed");
    }

    checkSearch();
    if (height().state != "READY") throw std::runtime_error("SEARCH requires READY as in production");
    RCLCPP_INFO(get_logger(), "[SEARCH TEST] SEARCH");
    const auto id = command(search_, "SEARCH");
    search_id_.store(id);
    
    const auto deadline = Clock::now() + std::chrono::milliseconds(search_timeout_ms_);
    while (Clock::now() < deadline) {
      checkSearch();
      const auto t = torch_->snapshot();
      // if (t.transfer_detected != std::optional<bool>(false))
      //   throw std::runtime_error("Unexpected transfer in inhibited test");
      if (expectedDryEnd(height())) {
        dry_end_.store(true);
        if (torch_->requestShutdown(1500ms) != TorchCommandResult::CONFIRMED || !torch_->finishCut() || !offConfirmed())
          throw std::runtime_error("Dry search OFF not confirmed");
        if (!expectedDryEnd(height()) || stop_.load())
          throw std::runtime_error("Dry search terminal state changed");
        torch_->setActiveSegment("");
        RCLCPP_INFO(get_logger(), "Expected SEARCH_END id=%u; OFF confirmed; recover explicitly", id);
        return;
      }
      
    }
    
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


  bool expectedDryEnd(const Height& h) const {
    return dry_ && search_active_.load() && search_id_.load() != 0 &&
      communicationValid(h) && h.session == search_session_.load() &&
      h.state == "FAULT" && h.fault == 4 && h.active_id == 0 &&
      h.done_id == search_id_.load() && h.result == 2 && h.referenced && !h.tracking;
  }
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
      std::lock_guard<std::mutex> lk(mu_);
      if (h.state != height_.state || h.fault != height_.fault || h.session != height_.session)
        RCLCPP_INFO(get_logger(), "[HEIGHT] %s -> %s fault=%d pos=%d session=%u active=%u done=%u result=%d",
          height_.state.c_str(), h.state.c_str(), h.fault, h.position, h.session, h.active_id, h.done_id, h.result);
      height_=std::move(h);
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
    if (search_active_.load()) checkSearch(); else checkBench();
    if (!client->wait_for_service(500ms)) throw std::runtime_error(std::string(name)+" unavailable");
    auto pending=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline=Clock::now()+3s;
    while (Clock::now()<deadline && rclcpp::ok() && !stop_.load()) {
      if (search_active_.load()) {
        try { checkSearch(); } catch (...) { client->remove_pending_request(pending); throw; }
      } else if (!offConfirmed()) break;
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
      if (search_active_.load()) checkSearch(); else checkBench();
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
  bool bench_{false}, dry_{false};
  std::atomic<bool> dry_end_{false}, stop_dispatched_{false};
  std::atomic<uint32_t> search_id_{0};
  std::atomic<bool> search_active_{false};
  std::atomic<uint32_t> search_session_{0};
  std::atomic<bool> stop_pending_{false};
  rclcpp::CallbackGroup::SharedPtr ops_group_, stop_group_, height_group_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_,monitor_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_,prep_,search_,stop_height_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracking_;
  mutable std::mutex task_mu_;
  bool transfer_test_enabled_{false};

std::atomic<bool> transfer_test_active_{false};
std::atomic<bool> shutdown_active_{false};

hb_robot_interfaces::msg::TaskState task_state_;

Clock::time_point task_received_{};

rclcpp::Subscription<
    hb_robot_interfaces::msg::TaskState
>::SharedPtr task_sub_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<CutHeightDebugNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4);
  rclcpp::executors::SingleThreadedExecutor height_executor;
  height_executor.add_callback_group(node->heightGroup(), node->get_node_base_interface());
  executor.add_node(node);
  std::thread height_thread([&] { height_executor.spin(); });
  executor.spin();
  height_executor.cancel();
  height_thread.join();
  rclcpp::shutdown();
  return 0;
}
