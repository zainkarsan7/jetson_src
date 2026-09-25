#include "hb_robot_perception/rgbd_acquisition.hpp"
#include <cmath>
#include <functional>
#include <utility>
#include <rmw/qos_profiles.h>
#include <tf2/exceptions.h>

namespace hb_perception{

     RGBDAcquisition::RGBDAcquisition(rclcpp::Node * node, tf2_ros::Buffer* tf_buffer, 
        const std::string& target_frame,
        const std::string& rgb_topic, 
        const std::string& depth_topic, 
        const std::string& camera_info_topic):
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
            if (camera_info_){return;}
            camera_info_ = camera_info;
            RCLCPP_INFO(node_->get_logger(),"got camera info");
            camera_info_sub_.reset(); 
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

                if(!camera_info_){
                    RCLCPP_WARN_THROTTLE(node_->get_logger(),
                    *node_->get_clock(),2000,"couldnt get camera info");
                    return;
                }

                Observation ob;
                ob.rgb = rgb;
                ob.depth_to_rgb = depth;
                ob.stamp = rgb->header.stamp;
                ob.camera_pose=std::move(cam_pose);
                ob.rgb_camera_info = camera_info_;

                {
                    std::lock_guard<std::mutex> lock(observation_mutex_);
                    latest_observation_ = std::move(ob);
                }
                observation_cv_.notify_all();
            }
        

        std::optional<Observation> RGBDAcquisition::acquireAfter(const rclcpp::Time& min_stamp, std::chrono::milliseconds timeout){
            // 
            std::unique_lock<std::mutex> lock(observation_mutex_);
            const bool recieved = observation_cv_.wait_for(lock,timeout,[this, &min_stamp](){
                return latest_observation_.has_value() && latest_observation_->stamp>min_stamp;
                // this is a snychronization trick to say consider this _cv_ which inspect and camera are 
                //using to coordinate r/w of latest_observation_. the switch only happens if 
                // the latest obs is new enough. when this function is called, it blocks rw access to 
                // latest_observtaion_ . its basically a sleep until timeout function

            });

            if(!recieved){
                RCLCPP_WARN(node_->get_logger(), "coudlnt get observation witihn timeout");
                return std::nullopt;
            }

            RCLCPP_INFO(node_->get_logger(),"got a new obs at stamp : %.2f", latest_observation_->stamp.seconds());
            return latest_observation_;



        }

}
