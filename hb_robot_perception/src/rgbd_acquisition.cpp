#include "hb_robot_perception/rgbd_acquisition.hpp"
#include <cmath>
#include <functional>
#include <utility>
#include <rmw/qos_profiles.h>
#include <tf2/exceptions.h>

namespace hb_perception{

     RGBDAcquisition::RGBDAcquisition(rclcpp::Node * node, tf2_ros::Buffer* tf_buffer, 
        const std::string& target_frame,
        const std::string& rgb_topic = "/k4a/rgb/image_raw", 
        const std::string& depth_topic = "/k4a/depth_to_rgb/image_raw" , 
        const std::string& camera_info_topic = "k4a/depth_to_rgb/camera_info"):
        node_(node),tf_buffer_(tf_buffer),target_frame_(target_frame)
        {
            rgb_sub_.subscribe(node_,rgb_topic,rmw_qos_profile_sensor_data);

            depth_sub_.subscribe(node_,depth_topic,rmw_qos_profile_sensor_data);


            /**
             * queue depth is the synchronizer queu not the ros dds queue
             * smaller queue because kinect owns both these streams and 
             * latest data more important than history transient local
             */
            synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(5), rgb_sub_,depth_sub_
            );
            
        }

}
