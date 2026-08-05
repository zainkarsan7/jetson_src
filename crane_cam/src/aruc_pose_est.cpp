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

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


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

        spreader_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        camera_pose_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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

    tf2::Transform invert_cv_pose(const cv::Vec3d & rvec, const cv::Vec3d & tvec){
        cv::Mat R_cam_tag;
        cv::Rodrigues(rvec,R_cam_tag);
        cv::Mat R_tag_cam = R_cam_tag.t();
        cv::Mat t_cam_tag = (cv::Mat_<double>(3,1)<<tvec[0],tvec[1],tvec[2]);
        cv::Mat t_tag_cam = -R_tag_cam * t_cam_tag;

        // then get into quaternion
        tf2::Matrix3x3 tf_rotation(
            R_tag_cam.at<double>(0, 0),
            R_tag_cam.at<double>(0, 1),
            R_tag_cam.at<double>(0, 2),

            R_tag_cam.at<double>(1, 0),
            R_tag_cam.at<double>(1, 1),
            R_tag_cam.at<double>(1, 2),

            R_tag_cam.at<double>(2, 0),
            R_tag_cam.at<double>(2, 1),
            R_tag_cam.at<double>(2, 2));
        tf2::Quaternion q_tag_cam;
        tf_rotation.getRotation(q_tag_cam);
        q_tag_cam.normalize();
        tf2::Transform result;
        result.setOrigin(tf2::Vector3(t_tag_cam.at<double>(0),
        t_tag_cam.at<double>(1),
        t_tag_cam.at<double>(2)
                                                ));
        result.setRotation(q_tag_cam);
        return result;
    }

    tf2::Transform get_Cam_to_Spreader(const tf2::Transform & tag_2_cam){
        tf2::Transform cam_spreader;
        cam_spreader.setOrigin(tf2::Vector3(0,-0.14,-0.3));
        tf2::Quaternion q_cam_spreader;
        q_cam_spreader.setRPY(0.0,0.0,0);
        q_cam_spreader.normalize();
        cam_spreader.setRotation(q_cam_spreader);
        // cam_spreader.setIdentity();
        tf2::Transform T_tag_2_spreader = tag_2_cam * cam_spreader;
        return T_tag_2_spreader;
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
            cv::aruco::estimatePoseSingleMarkers(mCorners,0.1,cam_mat_,dist_coeffs,rvecs,tvecs);
            for (size_t i=0; i<mIds.size(); i++){
                cv::drawFrameAxes(annotated_,cam_mat_,dist_coeffs,rvecs[i],tvecs[i],0.01);
                RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "tvec = [%.4f, %.4f, %.4f]",
            tvecs[i][0],tvecs[i][1],tvecs[i][2]
        );
            }
        
        // tf2::Quaternion quat;
        // get_tf2_quat(rvecs.back(),quat);

        tf2::Transform T_measured_cam = invert_cv_pose(rvecs.back(),tvecs.back());
        tf2::Transform T_measured_spreader = get_Cam_to_Spreader(T_measured_cam);
        cam_transform.header.stamp = msg->header.stamp;
        cam_transform.header.frame_id = "Swivel_Tag";
        cam_transform.child_frame_id = "Camera_Measured";
        cam_transform.transform = tf2::toMsg(T_measured_cam);
        
        spreader_transform.header.stamp = msg->header.stamp;
        spreader_transform.header.frame_id = "Swivel_Tag";
        spreader_transform.child_frame_id = "Spreader_Measured";
        spreader_transform.transform = tf2::toMsg(T_measured_spreader);


        camera_pose_broadcaster_->sendTransform(cam_transform);       
        spreader_pose_broadcaster_->sendTransform(spreader_transform);                
         

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
    geometry_msgs::msg::TransformStamped cam_transform;
    geometry_msgs::msg::TransformStamped spreader_transform;


    tf2::Transform T_webcam;



    image_transport::Subscriber img_sub_;
    image_transport::Publisher anno_img_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    sensor_msgs::msg::CameraInfo cam_info;
    std::unique_ptr<tf2_ros::TransformBroadcaster> spreader_pose_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> camera_pose_broadcaster_;


};




int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ArucoNode>());
    rclcpp::shutdown();
    return 0;
}

