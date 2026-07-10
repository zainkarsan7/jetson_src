#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;
class cam_publisher : public rclcpp::Node{
    public: cam_publisher():Node("crane_cam"){
        // make the topic publish
        img_pub_ = image_transport::create_publisher(this,"camera/image_raw");
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info",10);
        cap_.open(0);
        if (!cap_.isOpened()){
            RCLCPP_ERROR(this->get_logger(),"couldnt open cam");
            rclcpp::shutdown();
            return;
        }
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
        std::bind(&cam_publisher::frame_callback,this));
    }
    private:
    
    void frame_callback(){
        // get frame
        cv::Mat frame;
        cap_>>frame;
        if (frame.empty()){
            RCLCPP_INFO(this->get_logger(),"empty frame");
            return;
        }
        std_msgs::msg::Header header;
        header.stamp = this->now();
        
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header,"bgr8",frame).toImageMsg();
        img_pub_.publish(msg);  
        
        sensor_msgs::msg::CameraInfo cam_info;
        cam_info.header = header;
        cam_info.height = frame.rows;
        cam_info.width  = frame.cols;

        cam_info.distortion_model = "plumb_bob";
        cam_info.d.assign(5, 0.0);

        cam_info.k = {
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        };

        cam_info.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        cam_info.p = {
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        };       
        
        cam_info_pub_->publish(cam_info);

    }
    image_transport::Publisher img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<cam_publisher>());
    rclcpp::shutdown();
    return 0;
}
