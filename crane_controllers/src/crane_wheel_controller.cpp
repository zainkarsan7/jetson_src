#include "crane_controllers/crane_wheel_controller.hpp"
#include <cmath>
#include "pluginlib/class_list_macros.hpp"

namespace crane_controllers {
controller_interface::CallbackReturn CraneWheelController::on_init() {
  targets_.writeFromNonRT({0.0, 0.0});
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration CraneWheelController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {"G0_wheels/velocity", "G1_wheels/velocity"}};
}
controller_interface::InterfaceConfiguration CraneWheelController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}
controller_interface::CallbackReturn CraneWheelController::on_configure(const rclcpp_lifecycle::State &) {
  // Private ~/commands resolves to /<controller_name>/commands.
  subscriber_ = get_node()->create_subscription<Msg>(
      "~/commands", rclcpp::QoS(10), [this](const Msg::SharedPtr msg) {
        if (msg->data.size() != 2 || !std::isfinite(msg->data[0]) || !std::isfinite(msg->data[1])) {
          RCLCPP_WARN(get_node()->get_logger(), "Expected exactly two finite wheel velocities in rad/s");
          return;
        }
        targets_.writeFromNonRT({msg->data[0], msg->data[1]});
      });
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CraneWheelController::on_activate(const rclcpp_lifecycle::State &) {
  targets_.writeFromNonRT({0.0, 0.0});
  for (auto & command : command_interfaces_) command.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CraneWheelController::on_deactivate(const rclcpp_lifecycle::State &) {
  for (auto & command : command_interfaces_) command.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type CraneWheelController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  const auto * targets = targets_.readFromRT();
  if (!targets || command_interfaces_.size() != 2) return controller_interface::return_type::ERROR;
  command_interfaces_[0].set_value((*targets)[0]);
  command_interfaces_[1].set_value((*targets)[1]);
  return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneWheelController, controller_interface::ControllerInterface)