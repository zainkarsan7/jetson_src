#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

// Deliberately does not energize the torch. This is an OFF-only I/O
// verification + Pilz LIN dry-motion integration test.
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
    io_topic_ = declare_parameter<std::string>("io_topic", "/io_and_status_controller/io_states");
    io_service_ = declare_parameter<std::string>("io_service", "/io_and_status_controller/set_io");
    output_pin_ = declare_parameter<int>("output_pin", 1);
    io_sub_ = create_subscription<ur_msgs::msg::IOStates>(
        io_topic_, rclcpp::QoS(10).reliable(),
        [this](ur_msgs::msg::IOStates::ConstSharedPtr msg) {
          for (const auto& out : msg->digital_out_states) {
            if (static_cast<int>(out.pin) == output_pin_) {
              observed_off_.store(!out.state);
              observed_valid_.store(true);
            }
          }
        });
    io_client_ = create_client<ur_msgs::srv::SetIO>(io_service_);
  }

  void run() {
    if (axis_ != "x" && axis_ != "y" && axis_ != "z")
      throw std::runtime_error("axis must be x, y, or z");
    if (!std::isfinite(distance_m_) || std::abs(distance_m_) > 0.1 || distance_m_ == 0.0)
      throw std::runtime_error("distance_m must be nonzero and within +/-0.1 m");
    if (!(velocity_scale_ > 0.0 && velocity_scale_ <= 1.0) ||
        !(acceleration_scale_ > 0.0 && acceleration_scale_ <= 1.0))
      throw std::runtime_error("invalid Pilz scaling factor");

    // OFF is requested regardless of initial feedback. This does not
    // establish that the physical relay contacts are open.
    if (!commandOff()) throw std::runtime_error("SetIO OFF not acknowledged");
    const auto feedback_deadline = std::chrono::steady_clock::now() + 2s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < feedback_deadline &&
           !(observed_valid_.load() && observed_off_.load()))
      std::this_thread::sleep_for(20ms);
    if (!observed_valid_.load() || !observed_off_.load())
      throw std::runtime_error("DO OFF feedback not confirmed");

    auto move_group = std::make_shared<MoveGroup>(shared_from_this(), group_);
    move_group->setEndEffectorLink(tcp_link_);
    move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group->setPlannerId("LIN");
    move_group->setMaxVelocityScalingFactor(velocity_scale_);
    move_group->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group->setStartStateToCurrentState();

    const auto current = move_group->getCurrentPose(tcp_link_);
    auto target = current.pose;
    // Translation in the MoveGroup pose reference frame, not tool-local X.
    if (axis_ == "x") target.position.x += distance_m_;
    if (axis_ == "y") target.position.y += distance_m_;
    if (axis_ == "z") target.position.z += distance_m_;

    RCLCPP_INFO(get_logger(), "Planning %+.3f m along %s in frame %s; execute=%s",
                distance_m_, axis_.c_str(), current.header.frame_id.c_str(),
                execute_ ? "true" : "false");
    move_group->setPoseTarget(target, tcp_link_);
    MoveGroup::Plan plan;
    const bool planned = static_cast<bool>(move_group->plan(plan));
    move_group->clearPoseTargets();
    if (!planned || plan.trajectory_.joint_trajectory.points.empty())
      throw std::runtime_error("Pilz LIN planning failed");
    RCLCPP_INFO(get_logger(), "LIN plan: %zu points", plan.trajectory_.joint_trajectory.points.size());
    if (!execute_) {
      RCLCPP_INFO(get_logger(), "Plan-only mode: no motion executed");
      return;
    }
    if (!observed_valid_.load() || !observed_off_.load())
      throw std::runtime_error("Torch output no longer confirmed OFF");
    if (!static_cast<bool>(move_group->execute(plan)))
      throw std::runtime_error("MoveIt execution failed");
    if (!commandOff())
      throw std::runtime_error("Final SetIO OFF not acknowledged");
    RCLCPP_INFO(get_logger(), "Dry traverse complete; torch ON was never requested");
  }

private:
  bool commandOff() {
    if (!io_client_->wait_for_service(2s)) return false;
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = 1;  // digital output
    request->pin = static_cast<int8_t>(output_pin_);
    request->state = 0.0;
    auto pending = io_client_->async_send_request(request);
    if (pending.wait_for(2s) != std::future_status::ready) {
      io_client_->remove_pending_request(pending);
      return false;
    }
    auto response = pending.get();
    return response && response->success;
  }

  std::string group_, tcp_link_, axis_, io_topic_, io_service_;
  double distance_m_{}, velocity_scale_{}, acceleration_scale_{};
  bool execute_{};
  int output_pin_{};
  std::atomic<bool> observed_valid_{false}, observed_off_{false};
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_sub_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
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
