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

            synchronizer_->registerCallback(std::bind(&RGBDAcquisition::synchronizedCallback,this,
            std::placeholders::_1,std::placeholders::_2));

            camera_info_sub_ = node->create_subscription<CameraInfo>(camera_info_topic,
                rclcpp::SensorDataQoS().keep_last(1),
                std::bind(&RGBDAcquisition::cameraInfoCallback,this, std::placeholders::_1));

            RCLCPP_INFO(node->get_logger(),"initialized RGBD Acquisition");
            RCLCPP_INFO(node->get_logger(),"RGB: %s",rgb_topic.c_str());
            RCLCPP_INFO(node->get_logger(),"Depth: %s",depth_topic.c_str());
            RCLCPP_INFO(node->get_logger(),"Cam Info: %s",camera_info_topic.c_str());
            RCLCPP_INFO(node->get_logger(),"Target: %s",target_frame.c_str());
            
        }

        void RGBDAcquisition::cameraInfoCallback(const CameraInfo::ConstSharedPtr& camera_info){
            std::lock_guard<std::mutex> lock(observation_mutex_);
            latest_camera_info_ = camera_info;
        }

        void RGBDAcquisition::synchronizedCallback(const Image::ConstSharedPtr& rgb,
            const Image::ConstSharedPtr& depth){
                // decide a timestamp to choose

                const rclcpp::Time rgb_t = rgb->header.stamp;
                const rclcpp::Time depth_t = depth->header.stamp;
                const rclcpp::Time obs_stamp = rgb_t;
                

                geometry_msgs::msg::TransformStamped cam_pose;
                try{
                cam_pose = tf_buffer_->lookupTransform(target_frame_,rgb->header.frame_id,obs_stamp);
                }
                catch(const tf2::TransformException &ex){
                    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
                "couldnt get %s to %s transform at timestamp %s",
            rgb->header.frame_id,target_frame_,ex.what());
                    return;
                }

                CameraInfo::ConstSharedPtr camera_info;
                {
                    std::lock_guard<std::mutex>lock(observation_mutex_);
                    camera_info = latest_camera_info_;
                }
                if(!camera_info){
                    RCLCPP_WARN_THROTTLE(node_->get_logger(),
                    *node_->get_clock(),2000,"couldnt get camera info");
                    return;
                }
                Observation ob;
                ob.rgb = rgb;
                ob.depth_to_rgb = depth;
                ob.stamp = rgb->header.stamp;
                ob.camera_pose=std::move(cam_pose);
                
                ob.rgb_camera_info = camera_info;


                


                

            }
}
