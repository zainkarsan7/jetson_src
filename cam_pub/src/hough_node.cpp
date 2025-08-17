#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>

class HoughNode: public rclcpp::Node{
    public:
    HoughNode():Node("hough_node"){
        image_sub_ = image_transport::create_subscription(
            this,
            "camera/image_raw",
            [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg){
                this->image_callback(msg);
            },
            "raw"
        );
        image_pub_ = image_transport::create_publisher(this,"camera/image_hough");

    }
    private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");

        }catch (cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(),"CV Bridge exception '%s'",e.what());
            return;
        }
        cv::Mat gray, edges, hough_output;
        cv::cvtColor(cv_ptr->image,gray,cv::COLOR_BGR2GRAY);
        cv::Canny(gray,edges,50,150,3);
        cv_ptr->image.copyTo(hough_output);

        std::vector<cv::Vec2f> lines;
        cv::HoughLines(edges,lines,1,CV_PI/180,100);
        for (size_t i=0; i<lines.size();++i){
            float rho = lines[i][0], theta=lines[i][1];
            double a = cos(theta),b=sin(theta);
            double x0=a*rho,y0= b*rho;
            cv::Point pt1(cvRound(x0+500*(-b)),cvRound(y0+500*(a)));
            cv::Point pt2(cvRound(x0-500*(-b)),cvRound(y0-500*(a)));
            cv::line(hough_output,pt1,pt2,cv::Scalar(0,0,255),2);
        }
        auto out_msg = cv_bridge::CvImage(msg->header,"bgr8",hough_output).toImageMsg();
        image_pub_.publish(out_msg);


    }
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};




int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<HoughNode>());
    rclcpp::shutdown();
    return 0;
}