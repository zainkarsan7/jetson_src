#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

class WebCamPublisher : public rclcpp::Node{
    public:
    WebCamPublisher():Node("webcam_publisher"){
        publisher_ = image_transport::create_publisher(this,"camera/image_raw");
        cap_.open(0);
        if (!cap_.isOpened()){
            RCLCPP_ERROR(this->get_logger(),"Failed to open Webcam");
            rclcpp::shutdown();
            return;
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),std::bind(&WebCamPublisher::timer_callback,this)
        );
    }
    private:
    void timer_callback(){
        cv::Mat frame;
        cap_ >>frame;
        if (frame.empty()){
            RCLCPP_WARN(this->get_logger(),"Empty Frame");
            return;
        }
        std_msgs::msg::Header header;
        header.stamp = now();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            header,"bgr8",frame).toImageMsg();
        publisher_.publish(msg);
    }
    image_transport::Publisher publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<WebCamPublisher>());
    rclcpp::shutdown();
    return 0;
}