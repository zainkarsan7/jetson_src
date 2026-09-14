#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "crane_estimation/crane_attitude_estimator.hpp"
#include "crane_estimation/crane_attitude_utils.hpp"
#include "rclcpp/time.hpp"

class ChassisPoseNode: public rclcpp::Node{

public:

    ChassisPoseNode():Node("chassis_pose_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        trolley_to_chassis_broadcaster_(*this)
        {
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(20),[this](){
            //     compute_trolley_to_chassis_transform_callback(const geometry_msgs::msg::TransformStamped & trolley_left_spreader_msg);
            // });

            left_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                "Trolley/left_spreader_transform",rclcpp::SensorDataQoS(),[this](const geometry_msgs::msg::TransformStamped::SharedPtr trolley_left_spreader_msg){
                    compute_trolley_to_chassis_transform_callback(* trolley_left_spreader_msg);
                }
            );
        

    }
    


private:

    tf2_ros::TransformBroadcaster trolley_to_chassis_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr left_sub_;
    void ChassisPoseNode::compute_trolley_to_chassis_transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr trolley_left_spreader_msg){
        // try both transforms, choose left, lookup from ros buffer

        try{
            
        // auto trolley_to_left = tf_buffer_.lookupTransform("Trolley_link","left_spreader_pose",tf2::TimePointZero);

        auto chassis_to_left = tf_buffer_.lookupTransform("chassis_link","left_spreader_pose",tf2::TimePointZero);

        Eigen::Isometry3d T_trol_to_left_ = tf2::transformToEigen(trolley_left_spreader_msg);
        Eigen::Isometry3d T_left_to_chassis_ = tf2::transformToEigen(chassis_to_left).inverse();
        const Eigen::Isometry3d T_trol_to_chassis = T_trol_to_left_ * T_left_to_chassis_;

        auto output = tf2::eigenToTransform(T_trol_to_chassis);
        output.header.frame_id = "Trolley_link";
        output.child_frame_id = "chassis_link";
        output.header.stamp = this->now();

        trolley_to_chassis_broadcaster_.sendTransform(output);}
        catch(const tf2::TransformException &ex){
            RCLCPP_WARN_THROTTLE(get_logger(),get_clock(),2000,"error: %s",ex.what());
        }
    }


};



int main (int argv, char* argc[]){
    return 0;
}