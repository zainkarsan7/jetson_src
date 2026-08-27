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

    SpreaderPoseNode():Node("spreader_pose_node"), detector_(cv::aruco::DICT_6X6_250,0.25)
    {
        trolley_image_sub_ = image_transport::create_subscription(this,
            "/Trolley_RS/Trolley_Cam/color/image_raw",
            std::bind(&SpreaderPoseNode::get_detections_callback, this,std::placeholders::_1),"raw"
        );

        trolley_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/Trolley_RS/Trolley_Cam/color/camera_info",
            rclcpp::SensorDataQoS(),
            std::bind(&SpreaderPoseNode::get_cam_info_callback,this,std::placeholders::_1)
        );

        anno_trolley_image_pub = image_transport::create_publisher(this,"Trolley_RS/Trolley_Cam/color/image_aruco");

        spreader_left_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        spreader_right_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);



    }
    private:

    void get_cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg){
        cv::Mat cam_mat_ = cv::Mat(3,3,CV_64F);

        for(size_t i = 0; i<9; i++){
            cam_mat_.at<double>(i/3,i%3) = msg->k[i];
            }
        cv::Mat dist_coeffs (msg->d);
        
        detector_.setCamInfo(cam_mat_, dist_coeffs);
        cam_frame_id = msg->header.frame_id;
        cam_info_received_ = true;
    }
    
    void get_detections_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
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
        RCLCPP_INFO_THROTTLE(this->get_logger(),this->get_clock(),2000,"got '%d' detections",detections.size());

        sensor_msgs::msg::Image::SharedPtr anno_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated_).toImageMsg();
        anno_trolley_image_pub.publish(anno_msg);

    }
    //members


    ArucoDetector detector_;
    image_transport::Subscriber trolley_image_sub_;
    image_transport::Publisher anno_trolley_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr trolley_cam_info_sub_;
    sensor_msgs::msg::CameraInfo trolley_cam_info;

    std::string cam_frame_id;
    bool cam_info_received_{false};


    std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_left_pose_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_right_pose_broadcaster_;

};

int main(int argc ,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SpreaderPoseNode>());
    rclcpp::shutdown();
    return 0;
}