#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <moveit/planning_interface/planning_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char *argv[]){
    
    
    rclcpp::init(argc,argv);
    auto const node = std::make_shared<rclcpp::Node>("hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});


    auto const logger = rclcpp::get_logger("hello_moveit");


    auto mg_interface = MoveGroupInterface(node,"manipulator");
    mg_interface.setEndEffectorLink("ur10e_wrist_3_link");
    auto vis_tools = moveit_visual_tools::MoveItVisualTools{
        node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        mg_interface.getRobotModel()};
    
    vis_tools.deleteAllMarkers();
    vis_tools.loadRemoteControl();
    
    /// closure visualization stuff
    auto const draw_title = [&vis_tools](auto text){
        auto const text_pose = []{
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        vis_tools.publishText(text_pose,text,rviz_visual_tools::CYAN,rviz_visual_tools::XXLARGE);
    };

    auto const prompt = [&vis_tools](auto text){
        vis_tools.prompt(text);
    };

    auto const draw_trajectory_tp = [&vis_tools, lmp = mg_interface.getRobotModel()->getLinkModel("ur10e_tool0"),jmg = mg_interface.getRobotModel()->getJointModelGroup("manipulator")](auto const trajectory){
        vis_tools.publishTrajectoryLine(trajectory,lmp,jmg,rviz_visual_tools::RED);
    };
    

    auto const t_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.5;
        msg.position.y = 0.5;
        msg.position.z = 0.75;
        return msg;
    }();

    mg_interface.setPoseTarget(t_pose);

    prompt("start planning");
    draw_title("planning");
    vis_tools.trigger();
    auto const [success,plan] = [&mg_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(mg_interface.plan(msg));
        return std::make_pair(ok,msg);

    }();
    if (success){
        draw_trajectory_tp(plan.trajectory_);
        vis_tools.trigger();
        prompt("this is the path, click to run it");
        draw_title("running");
        vis_tools.trigger();
        mg_interface.execute(plan);

    }
    else{
        draw_title("planning failed");
        vis_tools.trigger();
        
        RCLCPP_ERROR(logger,"couldnt plan it");
    }
    rclcpp::shutdown();
    spinner.join();


    return 0;
}