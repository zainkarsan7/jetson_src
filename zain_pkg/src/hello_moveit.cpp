#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char *argv[]){


    rclcpp::init(argc,argv);
    auto const node = std::make_shared<rclcpp::Node>("hello_moveit basfs",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("hello_moveit");

    



    rclcpp::shutdown();


    return 0;
}