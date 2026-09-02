#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"


class ChassisPoseNode: public rclcpp::Node{

public:
private:

std::unique_ptr<tf2_ros::TransformBroadcaster> trolley_to_chassis_broadcaster_;
};



int main (int argv, char* argc[]){
    return 0;
}