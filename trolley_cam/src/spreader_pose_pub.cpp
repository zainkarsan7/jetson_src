#include "rclcpp/rclcpp.hpp"
#include "trolley_cam/spreader_pose_node.hpp"


int main(int argc ,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<trolley_cam::SpreaderPoseNode>(rclcpp::NodeOptions));
    rclcpp::shutdown();
    return 0;
}