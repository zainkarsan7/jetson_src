#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char *argv[]){
    
    
    rclcpp::init(argc,argv);
    auto const node = std::make_shared<rclcpp::Node>("hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("hello_moveit");


    auto mg_interface = MoveGroupInterface(node,"manipulator");

    auto const t_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.5;
        msg.position.y = 0.5;
        msg.position.z = 0.75;
        return msg;
    }();

    mg_interface.setPoseTarget(t_pose);

    auto const [success,plan] = [&mg_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(mg_interface.plan(msg));
        return std::make_pair(ok,msg);

    }();
    if (success){
        mg_interface.execute(plan);

    }
    else{
        RCLCPP_ERROR(logger,"couldnt plan it");
    }
    rclcpp::shutdown();


    return 0;
}