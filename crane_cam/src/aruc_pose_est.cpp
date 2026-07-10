#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class ArucoNode: public rclcpp::Node{
    public:
    ArucoNode():Node("aruco_pose_node"){
        // do stuff here, subscribe to camera/image_raw and camera/camera_info
        // call two callbacks, get the frame, detect aruco
        img_sub_ = image_transport::create_subscription(
            this,
            "camera/image_raw",
            std::bind(&ArucoNode::get_pose_callback,this,
            std::placeholders::_1),
            "raw"
        );

        cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&ArucoNode::get_cam_info_callback,this,
            std::placeholders::_1));


        anno_img_pub = image_transport::create_publisher(this,
        "camera/image_aruco");

        pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    }
    private:
    void get_cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg){
        cam_mat_ = cv::Mat(3,3,CV_64F);

        for(size_t i = 0; i<9; i++){
            cam_mat_.at<double>(i/3,i%3) = msg->k[i];
            }
        dist_coeffs = cv::Mat(msg->d).clone();
        cam_frame_id = msg->header.frame_id;
        cam_info_received_ = true;
    }
    void get_pose_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
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
        
        

        sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header,"bgr8",annotated_).toImageMsg();
        anno_img_pub.publish(out_msg);
    }

    cv::Mat cam_mat_;
    cv::Mat dist_coeffs;
    std::string cam_frame_id;
    bool cam_info_received_{false};

    image_transport::Subscriber img_sub_;
    image_transport::Publisher anno_img_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    sensor_msgs::msg::CameraInfo cam_info;
    std::unique_ptr<tf2_ros::TransformBroadcaster> pose_broadcaster_;


};




int main(int argc, char *argv[]){



    return 0;
}

