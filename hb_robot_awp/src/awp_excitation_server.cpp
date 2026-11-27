#include "hb_robot_awp/server.hpp"
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<AwpExcitationServer>(false);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
  executor.add_node(node);
  std::thread spin([&] {executor.spin();});
  int result=0;
  try { node->initialize(); spin.join(); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    executor.cancel(); if(spin.joinable()) spin.join(); result=1;
  }
  rclcpp::shutdown(); return result;
}
