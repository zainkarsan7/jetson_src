#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


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

    void get_tf2_quat(const cv::Vec3d &rvec, tf2::Quaternion &quat){
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec,rotation_matrix);
        tf2::Matrix3x3 tf_rotation(
            rotation_matrix.at<double>(0, 0),
            rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2),

            rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1),
            rotation_matrix.at<double>(1, 2),

            rotation_matrix.at<double>(2, 0),
            rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
        tf_rotation.getRotation(quat);
        quat.normalize();
    }

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
        

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(annotated_,dictionary,mCorners,mIds,parameters);
        if (mIds.size()>0){

            cv::aruco::drawDetectedMarkers(annotated_,mCorners,mIds);
            cv::aruco::estimatePoseSingleMarkers(mCorners,0.05,cam_mat_,dist_coeffs,rvecs,tvecs);
            for (size_t i=0; i<mIds.size(); i++){
                cv::aruco::drawAxis(annotated_,cam_mat_,dist_coeffs,rvecs[i],tvecs[i],0.01);
                RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "tvec = [%.4f, %.4f, %.4f]",
            tvecs[i][0],tvecs[i][1],tvecs[i][2]
        );
            }
        
        tf2::Quaternion quat;
        get_tf2_quat(rvecs.back(),quat);
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "spreader_link";
        transform.child_frame_id = "tag";
        transform.transform.translation.x = tvecs.back()[0];
        transform.transform.translation.y = tvecs.back()[1];
        transform.transform.translation.z = tvecs.back()[2];

        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        pose_broadcaster_->sendTransform(transform);                

        }
        


        sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header,"bgr8",annotated_).toImageMsg();
        anno_img_pub.publish(out_msg);
    }
    std::vector<int> mIds;
    std::vector<std::vector<cv::Point2f>> mCorners, rejCand;
    std::vector<cv::Vec3d> rvecs,tvecs;

    cv::Mat cam_mat_;
    cv::Mat dist_coeffs;
    std::string cam_frame_id;
    bool cam_info_received_{false};
    geometry_msgs::msg::TransformStamped transform;

    image_transport::Subscriber img_sub_;
    image_transport::Publisher anno_img_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    sensor_msgs::msg::CameraInfo cam_info;
    std::unique_ptr<tf2_ros::TransformBroadcaster> pose_broadcaster_;


};




int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ArucoNode>());
    rclcpp::shutdown();
    return 0;
}

