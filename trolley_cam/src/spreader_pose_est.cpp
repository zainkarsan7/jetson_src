#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "trolley_cam/aruco_detector.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


// make a node that looks for two aruco markers, using one or the other's pose
// to estimate a rigid transform describing the pose of the spreader. 


class SpreaderPoseNode: public rclcpp::Node{
    public:

    SpreaderPoseNode():Node("spreader_pose_node"){}
    private:
    //members
    image_transport::Subscriber trolley_image_sub_;
    image_transport::Publisher anno_trolley_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr trolley_cam_info_sub_;
    // 
    sensor_msgs::msg::CameraInfo trolley_cam_info;
    std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_left_pose_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_right_pose_broadcaster_;

};

int main(int argc ,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SpreaderPoseNode>());
    rclcpp::shutdown();
    return 0;
}