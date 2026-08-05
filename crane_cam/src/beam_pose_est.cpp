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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class BeamDetectorNode: public rclcpp::Node{
    public:
    BeamDetectorNode():Node("beam_pose_node"){

        // 
        img_sub_ = image_transport::create_subscription(
            this,
            "/Cam_RS/Crane_RS/color/image_raw",
            std::bind(&BeamDetectorNode::get_image_callback,
                this,std::placeholders::_1),"raw"
        );
        depth_sub_ = image_transport::create_subscription(
            this,
            "/Cam_RS/Crane_RS/depth/image_rect_raw", 
            std::bind(&BeamDetectorNode::get_depth_callback,this, std::placeholders::_1),"raw");


        cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/Cam_RS/Crane_RS/color/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&BeamDetectorNode::get_cam_info_callback,this,
            std::placeholders::_1));

        detect_pub = image_transport::create_publisher(this,
        "/Cam_RS/Crane_RS/detection/detection_image");

    }
    private:
        void get_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
            if(!cam_info_received_){
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"waiting for cam_info"); 
                return;
            }
            
            try{
                cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);

            }catch(cv_bridge::Exception &e){
                RCLCPP_ERROR(this->get_logger(),"CV bridge error %s",e.what());
                return;
            }
            cv::Mat detected_ = cv_ptr->image.clone();
            sensor_msgs::msg::Image::SharedPtr output_img_msg = cv_bridge::CvImage(msg->header,"bgr8",detected_).toImageMsg();
            detect_pub.publish(output_img_msg);
        }
        void get_depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & dmsg){
            

        }
        void get_cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &c_info){
            cam_mat_ = cv::Mat(3,3,CV_64F);

            for(size_t i = 0; i<9; i++){
                cam_mat_.at<double>(i/3,i%3) = c_info->k[i];
                }
                dist_coeffs = cv::Mat(c_info->d).clone();
                cam_frame_id = c_info->header.frame_id;
                cam_info_received_ = true;

        }

        image_transport::Subscriber img_sub_;
        image_transport::Subscriber depth_sub_;
        image_transport::Publisher detect_pub;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        sensor_msgs::msg::CameraInfo cam_info;

        cv::Mat cam_mat_;
        cv::Mat dist_coeffs;
        std::string cam_frame_id;
        bool cam_info_received_{false};

        cv_bridge::CvImageConstPtr cv_ptr;

        geometry_msgs::msg::PoseStamped beam_pose;
        std::unique_ptr<tf2_ros::TransformBroadcaster> beam_pose_broadcaster_;
        

};



int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<BeamDetectorNode>());
    rclcpp::shutdown();
    return 0;
}