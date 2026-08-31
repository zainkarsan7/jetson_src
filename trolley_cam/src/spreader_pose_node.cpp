#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
#include "trolley_cam/spreader_pose_node.hpp"


// make a node that looks for two aruco markers, using one or the other's pose
// to estimate a rigid transform describing the pose of the spreader. 

namespace trolley_cam
{
SpreaderPoseNode::SpreaderPoseNode(const rclcpp::NodeOptions & options)
:Node("spreader_pose_node",options), detector_(cv::aruco::DICT_6X6_250,0.2){
        trolley_image_sub_ = image_transport::create_subscription(this,
            "/Trolley_RS/Trolley_Cam/color/image_raw",
            std::bind(&SpreaderPoseNode::get_detections_callback, this,std::placeholders::_1),"raw"
        );

        trolley_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/Trolley_RS/Trolley_Cam/color/camera_info",
            rclcpp::SensorDataQoS(),
            std::bind(&SpreaderPoseNode::get_cam_info_callback,this,std::placeholders::_1)
        );

        anno_trolley_image_pub = image_transport::create_publisher(this,"Trolley_Detection");

        spreader_left_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        spreader_right_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        left_t_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("Trolley/left_spreader_transform",10);
        right_t_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("Trolley/right_spreader_transform",10);

    }

    void SpreaderPoseNode::get_cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg){
        cv::Mat cam_mat_ = cv::Mat(3,3,CV_64F);

        for(size_t i = 0; i<9; i++){
            cam_mat_.at<double>(i/3,i%3) = msg->k[i];
            }
        cv::Mat dist_coeffs (msg->d);
        
        detector_.setCamInfo(cam_mat_, dist_coeffs);
        cam_frame_id = msg->header.frame_id;
        cam_info_received_ = true;
    }
    
    void SpreaderPoseNode::get_detections_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
        if (!cam_info_received_) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "Waiting for camera_info");
                    return;
                    }

        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);

        }catch (cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(),"CV Bridge exception '%s'",e.what());
            return;
        }  
        cv::Mat annotated_ = cv_ptr->image.clone();
        auto detections = detector_.detect(annotated_);
        detector_.draw_detections(annotated_,detections);
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),2000,"got '%zu' detections",detections.size());

        sensor_msgs::msg::Image::SharedPtr anno_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated_).toImageMsg();
        anno_trolley_image_pub.publish(anno_msg);
        for (const auto& detection : detections){
            if (detection.id== 25){
                geometry_msgs::msg::TransformStamped msg_out;
                msg_out.header.stamp = msg->header.stamp;
                msg_out.header.frame_id = "base";
                msg_out.child_frame_id = "left_pose";
                msg_out.transform = tf2::toMsg(detection.T_cam_marker);
                spreader_left_pose_broadcaster_->sendTransform(msg_out);
                left_t_pub->publish(msg_out);
            }
            if (detection.id== 50){
                geometry_msgs::msg::TransformStamped msg_out;
                msg_out.header.stamp = msg->header.stamp;
                msg_out.header.frame_id = "base";
                msg_out.child_frame_id = "right_pose";
                msg_out.transform = tf2::toMsg(detection.T_cam_marker);
                spreader_right_pose_broadcaster_->sendTransform(msg_out);
                right_t_pub->publish(msg_out);
            }

        }
    }
    //members in the hpp

}
RCLCPP_COMPONENTS_REGISTER_NODE(trolley_cam::SpreaderPoseNode)

