#include <memory>
#include <chrono>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <hb_robot_perception/perception_types.hpp>
#include <hb_robot_perception/workpiece_extractor.hpp>
#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class PerceptionDebugNode : public rclcpp::Node {
    public:
        PerceptionDebugNode():
        Node("perception_debug_node"){
            scene_frame_ = declare_parameter<std::string>("scene_frame", "world");
            rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/k4a/rgb/image_raw");
            depth_topic_ = declare_parameter<std::string>("depth_topic", "/k4a/depth_to_rgb/image_raw");
            camera_info_topic_ = declare_parameter<std::string>("info_topic","k4a/depth_to_rgb/camera_info");

            


        }
    private:

    void publishPCA();

    std::string scene_frame_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<hb_perception::RGBDAcquisition> rgbd_acquisition_;
    hb_perception::WorkpieceExtractor extractor_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ob_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wk_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mk_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    return 0;
}

