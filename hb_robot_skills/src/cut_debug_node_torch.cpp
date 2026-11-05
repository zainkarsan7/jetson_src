#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include "hb_robot_skills/motion/torch_controller.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

// Exercises the production TorchController; never sends SetIO directly.
class CutDebugNode : public rclcpp::Node {
public:
  CutDebugNode() : Node("cut_debug_node") {
    group_ = declare_parameter<std::string>("planning_group", "manipulator");
    tcp_link_ = declare_parameter<std::string>("tcp_link", "ur10e_torch_link");
    axis_ = declare_parameter<std::string>("axis", "x");
    distance_m_ = declare_parameter<double>("distance_m", 0.1);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 0.1);
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.1);
    execute_ = declare_parameter<bool>("execute", false);
    mode_ = declare_parameter<std::string>("mode", "dry");
    ignition_enabled_ = declare_parameter<bool>("ignition_enabled", false);
    confirm_live_test_ = declare_parameter<bool>("confirm_live_test", false);
    ignition_timeout_ms_ = declare_parameter<int>("ignition_timeout_ms", 1500);
    transfer_timeout_ms_ = declare_parameter<int>("transfer_timeout_ms", 2000);
    shutdown_timeout_ms_ = declare_parameter<int>("shutdown_timeout_ms", 1500);
    dwell_ms_ = declare_parameter<int>("pierce_dwell_ms", 500);

    TorchIOConfig config;
    config.io_topic = declare_parameter<std::string>(
        "io_topic", "/io_and_status_controller/io_states");
    config.set_io_service = declare_parameter<std::string>(
        "io_service", "/io_and_status_controller/set_io");
    config.torch_output_pin = declare_parameter<int>("output_pin", 1);
    config.transfer_input_pin = declare_parameter<int>("transfer_pin", 0);
    config.transfer_active_high = declare_parameter<bool>("transfer_active_high", true);
    config.output_timeout_s = declare_parameter<double>("io_timeout_s", 1.0);
    config.ignition_enabled = ignition_enabled_;
    torch_ = std::make_unique<TorchController>(*this, config);
    torch_->setFaultCallback([this](const std::string& reason) {
      RCLCPP_ERROR(get_logger(), "TorchController FAULT: %s", reason.c_str());
      stop_requested_.store(true);
      if (move_group_) move_group_->stop();
    });

    status_pub_ = create_publisher<hb_robot_interfaces::msg::TorchStatus>(
                "/cut_debug/torch_status",
                rclcpp::QoS(10)
            );
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100),
            [this](){
                publishDebugStatus();
            });

  }

  void run() {
    if (mode_ != "dry" && mode_ != "io_cycle" && mode_ != "cut")
      throw std::runtime_error("mode must be dry, io_cycle, or cut");
    if (axis_ != "x" && axis_ != "y" && axis_ != "z")
      throw std::runtime_error("axis must be x, y, or z");
    if (!std::isfinite(distance_m_) || std::abs(distance_m_) > 0.5 || distance_m_ == 0.0)
      throw std::runtime_error("distance_m must be nonzero and within +/-0.1 m");
    if (!(velocity_scale_ > 0.0 && velocity_scale_ <= 1.0) ||
        !(acceleration_scale_ > 0.0 && acceleration_scale_ <= 1.0))
      throw std::runtime_error("invalid Pilz scaling factor");
    if (ignition_timeout_ms_ <= 0 || transfer_timeout_ms_ <= 0 ||
        shutdown_timeout_ms_ <= 0 || dwell_ms_ < 0)
      throw std::runtime_error("invalid timeout or dwell");

    // A fresh IO message must arrive before any state-machine operations.
    const auto ready_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && !torch_->hasFreshIO() &&
           std::chrono::steady_clock::now() < ready_deadline)
      std::this_thread::sleep_for(20ms);
    if (!torch_->hasFreshIO())
      throw std::runtime_error("TorchController has no fresh IO feedback");

    // Always command OFF on startup, including when the previous state is unknown.
    shutdownOrThrow("initial shutdown");
    if (torch_->hasFault())
      throw std::runtime_error("Torch fault at startup: " + torch_->faultReason());
    if (!torch_->finishCut())
      throw std::runtime_error("Cannot enter IDLE: OFF or transfer-low not confirmed");
    printStatus("initial IDLE");

    if (mode_ == "dry") {
      auto plan = planTraverse();
      if (execute_) {
        if (torch_->hasFault() || !torch_->hasFreshIO() ||
            torch_->snapshot().output_on != std::optional<bool>(false))
          throw std::runtime_error("Torch not confirmed OFF before dry traverse");
        executeTraverse(plan);
      }
      RCLCPP_INFO(get_logger(), "Dry test complete");
      return;
    }

    // Explicitly arm any test that can assert the physical torch output.
    if (!ignition_enabled_ || !confirm_live_test_)
      throw std::runtime_error(
          "ON test requires ignition_enabled:=true AND confirm_live_test:=true");
    if (mode_ == "cut" && !execute_)
      throw std::runtime_error("cut mode requires execute:=true");

    MoveGroup::Plan plan;
    if (mode_ == "cut") plan = planTraverse(); // plan BEFORE ignition

    bool ignition_attempted = false;
    bool passed = false;
    std::string failure;
    try {
      ignition_attempted = true;
      torch_->setActiveSegment("cut_debug");
      const auto on = torch_->requestIgnition(
          std::chrono::milliseconds(ignition_timeout_ms_),
          [this] { return stop_requested_.load() || !rclcpp::ok(); });
      
      if (on != TorchCommandResult::CONFIRMED)
        throw std::runtime_error("requestIgnition failed, result=" +
                                 std::to_string(static_cast<int>(on)));
      printStatus("ignition confirmed");

      if (mode_ == "cut") {
        const auto transfer = torch_->waitForArcTransfer(
            std::chrono::milliseconds(transfer_timeout_ms_),
            [this] { return stop_requested_.load() || !rclcpp::ok(); });
        if (transfer != TorchController::WaitResult::CONFIRMED)
          throw std::runtime_error("arc transfer not confirmed, result=" +
                                   std::to_string(static_cast<int>(transfer)));
        printStatus("arc established");
        const auto dwell_end = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(dwell_ms_);
        while (std::chrono::steady_clock::now() < dwell_end) {
          if (stop_requested_.load() || torch_->hasFault() || !rclcpp::ok())
            throw std::runtime_error("fault or stop during pierce dwell");
          std::this_thread::sleep_for(10ms);
        }
        if (!torch_->markCutting())
          throw std::runtime_error("markCutting rejected");
        executeTraverse(plan);
        if (stop_requested_.load() || torch_->hasFault())
          throw std::runtime_error("fault/stop during cut");
      } else {
        // io_cycle intentionally does NOT require arc transfer.
        // It verifies ON feedback and immediately commands OFF.
        RCLCPP_INFO(get_logger(), "I/O cycle: ON feedback confirmed; requesting OFF");
        std::this_thread::sleep_for(1000ms);
      }
      passed = true;
    } catch (const std::exception& e) {
      failure = e.what();
    }

    // OFF must be attempted on success and on every failure after ignition.
    bool off_confirmed = false;
    if (ignition_attempted) {
      const auto off = torch_->requestShutdown(
          std::chrono::milliseconds(shutdown_timeout_ms_));
      off_confirmed = (off == TorchCommandResult::CONFIRMED);
      printStatus("after shutdown");
    }
    torch_->setActiveSegment("");

    if (!off_confirmed)
      throw std::runtime_error("OFF not confirmed; original error: " + failure);

    // In a faulted state, finishCut() must not clear the fault.
    if (torch_->hasFault())
      throw std::runtime_error("Torch fault latched: " + torch_->faultReason() +
                               "; original error: " + failure);
    // Output feedback can fall before the arc-transfer input drops.
    const auto transfer_off_deadline = std::chrono::steady_clock::now() + 2s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < transfer_off_deadline) {
      if (torch_->hasFault()) break;
      const auto t = torch_->snapshot();
      if (torch_->hasFreshIO() &&
          t.output_on == std::optional<bool>(false) &&
          t.transfer_detected == std::optional<bool>(false))
        break;
      std::this_thread::sleep_for(20ms);
    }
    if (!torch_->finishCut())
      throw std::runtime_error("OFF confirmed but finishCut rejected "
                               "(check transfer input); " + failure);
    if (!passed)
      throw std::runtime_error(failure);
    RCLCPP_INFO(get_logger(), "TorchController %s test complete", mode_.c_str());
  }

private:
  void printStatus(const char* stage) {
    const auto t = torch_->snapshot();
    RCLCPP_INFO(get_logger(),
      "%s: state=%d fresh=%d DO=%d DI_arc=%d AnIn=%.3f AnDom=%d command=%d fault=%d",
      stage, static_cast<int>(torch_->state()), torch_->hasFreshIO(),
      t.output_on.has_value() ? static_cast<int>(*t.output_on) : -1,
      t.transfer_detected.has_value() ? static_cast<int>(*t.transfer_detected) : -1,
      t.analog_value.has_value() ? static_cast<double>(*t.analog_value): 0.0,
      t.analog_domain.has_value() ? static_cast<int>(*t.analog_domain) : -1,
      t.command_on, torch_->hasFault())
      ;
  }

   void publishDebugStatus(){

        const auto data = torch_->snapshot();

        hb_robot_interfaces::msg::TorchStatus msg;
        msg.stamp = data.stamp;
        msg.io_connected = torch_->hasFreshIO();
        msg.command_on = data.command_on;
        msg.output_on = data.output_on.value_or(false);
        msg.transfer_detected = data.transfer_detected.value_or(false);
        msg.analog_valid = data.analog_value.has_value();
        msg.analog_input_value = data.analog_value.value_or(0.0);
        msg.analog_domain = static_cast<uint8_t>(data.analog_domain.value_or(0));        
        msg.arc_voltage_valid = data.arc_voltage_v.has_value();
        msg.arc_voltage_v = data.arc_voltage_v.value_or(0.0);
        msg.ready = false;
        msg.fault =torch_->hasFault();
        status_pub_->publish(msg);
    }

  void shutdownOrThrow(const char* stage) {
    const auto result = torch_->requestShutdown(
        std::chrono::milliseconds(shutdown_timeout_ms_));
    printStatus(stage);
    if (result != TorchCommandResult::CONFIRMED)
      throw std::runtime_error(std::string(stage) + ": OFF not confirmed");
  }

  MoveGroup::Plan planTraverse() {
    move_group_ = std::make_shared<MoveGroup>(shared_from_this(), group_);
    move_group_->setEndEffectorLink(tcp_link_);
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group_->setStartStateToCurrentState();
    const auto current = move_group_->getCurrentPose(tcp_link_);
    auto target = current.pose;
    if (axis_ == "x") target.position.x += distance_m_;
    if (axis_ == "y") target.position.y += distance_m_;
    if (axis_ == "z") target.position.z += distance_m_;
    RCLCPP_INFO(get_logger(), "Planning %+.3f m %s in %s",
                distance_m_, axis_.c_str(), current.header.frame_id.c_str());
    move_group_->setPoseTarget(target, tcp_link_);
    MoveGroup::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    move_group_->clearPoseTargets();
    if (!planned || plan.trajectory_.joint_trajectory.points.empty())
      throw std::runtime_error("Pilz LIN planning failed");
    RCLCPP_INFO(get_logger(), "LIN plan has %zu points",
                plan.trajectory_.joint_trajectory.points.size());
    return plan;
  }

  void executeTraverse(const MoveGroup::Plan& plan) {
    if (stop_requested_.load() || torch_->hasFault())
      throw std::runtime_error("stop/fault before motion");
    if (!static_cast<bool>(move_group_->execute(plan)))
      throw std::runtime_error("MoveIt execution failed");
  }

  std::string group_, tcp_link_, axis_, mode_;
  double distance_m_{}, velocity_scale_{}, acceleration_scale_{};
  bool execute_{false}, ignition_enabled_{false}, confirm_live_test_{false};
  int ignition_timeout_ms_{1500}, transfer_timeout_ms_{2000};
  int shutdown_timeout_ms_{1500}, dwell_ms_{500};
  std::atomic<bool> stop_requested_{false};
  std::unique_ptr<TorchController> torch_;
  std::shared_ptr<MoveGroup> move_group_;
  rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;



};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CutDebugNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&] { executor.spin(); });
  int status = 0;
  try { node->run(); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Debug test failed: %s", e.what());
    status = 1;
  }
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return status;
}
