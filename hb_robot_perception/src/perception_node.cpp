#include <memory>
#include <chrono>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <hb_robot_interfaces/msg/profile_estimate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <hb_robot_perception/perception_types.hpp>
#include <hb_robot_perception/workpiece_extractor.hpp>
#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <hb_robot_perception/profile_matcher.hpp>
#include <hb_robot_perception/profile_library.hpp>
#include <hb_robot_perception/profile_types.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;
namespace hb_perception{
class PerceptionDebugNode : public rclcpp::Node {
    public:
        PerceptionDebugNode():
        Node("perception_debug_node"){
            scene_frame_ = declare_parameter<std::string>("scene_frame", "world");
            rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/k4a/rgb/image_raw");
            depth_topic_ = declare_parameter<std::string>("depth_topic", "/k4a/depth_to_rgb/image_raw");
            camera_info_topic_ = declare_parameter<std::string>("info_topic","k4a/depth_to_rgb/camera_info");
            depth_range_ = declare_parameter<double>("depth_range", 1.5);
            processing_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // make a buffer listener combo
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(30.0));
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            //construct an rgbd acuiqision object
            rgbd_acquisition_= std::make_unique<hb_perception::RGBDAcquisition>(
                this,tf_buffer_.get(),scene_frame_,rgb_topic_,depth_topic_,camera_info_topic_);
            
            // depth_sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic_, rclcpp::SensorDataQoS(),
            //     std::bind(&PerceptionDebugNode::depthCallback,this,std::placeholders::_1));
            // cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, rclcpp::SensorDataQoS(),
            //     std::bind(&PerceptionDebugNode::cameraInfoCallback,this,std::placeholders::_1));



            // make all the publishers  
            ob_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ob_cloud",1);
            wk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/wk_cloud",1);
            mk_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/perception/wk_axes",1);
            sc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/section_cloud",1);
            // auto col_pub_qos =rclcpp::SensorDataQoS().keep_last(1);
            // col_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/collision_cloud",col_pub_qos);

            auto pe_qos = rclcpp::QoS(1).reliable().transient_local();

            profile_marker_pub_ =create_publisher<visualization_msgs::msg::MarkerArray>("/perception/profiles",pe_qos);

            profile_estimate_pub_ = create_publisher<hb_robot_interfaces::msg::ProfileEstimate>("/perception/profile_estimate",pe_qos);
            
            // collision_timer_ = create_wall_timer(std::chrono::milliseconds(500),
            // std::bind(&PerceptionDebugNode::publishCollisionCloud,this));

            timer_ = create_wall_timer(2s, std::bind(&PerceptionDebugNode::process, this),processing_group_);

            
            //get an observation pass it to workpiece extractor
            //publish marker array 


        }
    private:

    bool publishSectionCollision(const PointCloud& cloud, moveit::planning_interface::PlanningSceneInterface& scene){
        if (cloud.empty()){
            RCLCPP_WARN(get_logger(),"cloud is empty, no collision objects published");
            return false;
        }
        moveit_msgs::msg::CollisionObject col_obj;
        col_obj.header.frame_id=scene_frame_;
        col_obj.id = "wk_section_collision";
        col_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        constexpr double rad = 0.003;
        col_obj.primitives.reserve(cloud.size());
        col_obj.primitive_poses.reserve(cloud.size());
        for (const auto& p: cloud.points){
            if(!std::isfinite(p.x)|| !std::isfinite(p.y) || !std::isfinite(p.z)){
                continue;
            }
            shape_msgs::msg::SolidPrimitive sphere;
            sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            sphere.dimensions = {rad};
            geometry_msgs::msg::Pose pose;
            pose.orientation.w = 1.0;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = p.z;
            col_obj.primitives.push_back(sphere);
            col_obj.primitive_poses.push_back(pose);
            
        }
        if(col_obj.primitives.empty()){
            RCLCPP_WARN(get_logger(),"no primitives made it to col_obj");

            return false;
        }
        return scene.applyCollisionObject(col_obj);
    }

    void process(){

        std::optional<Observation> observation;
        try{
            observation = rgbd_acquisition_->latest();
        }
        catch(std::exception &e){
            RCLCPP_WARN(get_logger(),"couldnt get acquisition: %s",e.what());
            return;
        }
        if(!observation){
            RCLCPP_WARN(get_logger(),"couldnt get acquisition null");
            return;
        }
        
        RCLCPP_INFO(get_logger(),"latest observation %.6f is %.3f",observation->stamp.seconds(), (now() - observation->stamp).seconds());
        
        
        last_processed_stamp_ = observation->stamp;
        RCLCPP_INFO(get_logger(),"latest observation at %ld",observation->stamp);
        
        auto cloud = observationToCloud(*observation, depth_range_);
        if(!cloud){
            RCLCPP_INFO(get_logger(),"no cloud");
        }
        RCLCPP_INFO(get_logger(),"got cloud with %zu",cloud->points.size());
        
        const Eigen::Isometry3d T_scene_cam = tf2::transformToEigen(observation->camera_pose);
        
        pcl::transformPointCloud(*cloud,*cloud,T_scene_cam.matrix().cast<float>());
        
        publishCloud(cloud,observation->camera_pose.header.frame_id,ob_pub_);

       

        
            

        auto wkpiece = extractor_.extract(cloud);
        if (!wkpiece){
            RCLCPP_ERROR(get_logger(),"couldnt get cluster");
            return;
        }

        publishCloud(wkpiece->cloud,observation->camera_pose.header.frame_id,wk_pub_);
        publishPCA(*wkpiece,observation->camera_pose.header.frame_id,observation->stamp);

        RCLCPP_INFO(get_logger(),"Workpiece model: %zu points, | eigs %0.5f,%0.5f,%0.5f | axis %0.3f,%0.3f,%0.3f",
        wkpiece->cloud->size(),wkpiece->eigs.x(),
        wkpiece->eigs.y(),wkpiece->eigs.z(),
        wkpiece->l_axes.x(),wkpiece->l_axes.y(),wkpiece->l_axes.z());
        RCLCPP_INFO(get_logger(), "Extracting section");
        auto section_model = extractor_.extractSection(*wkpiece,0.01f,0.015f);
        if(!section_model){
            RCLCPP_WARN(get_logger(),"SOMETHING WRONG IN SECTION EXTRACTION");
            return;
        }
        auto profiles = ProfileLibrary::ipnProfiles();
        RCLCPP_INFO(get_logger(),"made ipn profiles");
        auto matches = matcher_.match(*section_model,profiles);
        if(matches.empty()){
            RCLCPP_INFO(get_logger(),"matching didnt work");
            return;
        }
        for (const auto& match : matches){
            RCLCPP_INFO(get_logger(),"best profile %s | RMS %.2f mm | inliers %.1f | score %.3f",
          match.profile.name.c_str(), match.rms_dist,match.inlier_fraction,match.score);
        }
        publishProfiles(
        *section_model,
        matches, observation->stamp
    );
        

        publishCloud(section_model->cloud,observation->camera_pose.header.frame_id,sc_pub_);
        publishSectionCollision(*section_model->cloud,planning_scene_interface_);
 
    }

    void publishProfiles(
    const SectionModel& section,
    const std::vector<ProfileMatch>& matches,
    const rclcpp::Time& observation_stamp
)
{
    auto msg =
        matcher_.getVisualization(
            section,
            matches,
            scene_frame_,1);

    profile_marker_pub_->publish(msg);
        
    hb_robot_interfaces::msg::ProfileEstimate profile_estimate_msg;
    try{
        profile_estimate_msg = matcher_.getProfileEstimateMsg(scene_frame_,section,matches.front());
    } catch(std::exception &e){
        RCLCPP_INFO(get_logger(),"something wrong in get profile msg, %s",e.what());
        return;
    }
    
    profile_estimate_msg.header.stamp = observation_stamp;
    profile_estimate_pub_->publish(profile_estimate_msg);


}

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr depth){
            {std::lock_guard<std::mutex>lock(collision_mutex_);
            latest_depth_ = depth;}
 
    }
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        {
            std::lock_guard<std::mutex>lock(collision_mutex_);
            latest_cam_info_ = msg;
        }
    }

    void publishPCA(const WorkpieceModel& model,std::string frame_id, const rclcpp::Time& observation_stamp){

        visualization_msgs::msg::MarkerArray array_;
        geometry_msgs::msg::Point start;
        start.x = model.centroid.x();
        start.y = model.centroid.y();
        start.z = model.centroid.z();
        for (int i = 0; i<3; i++){
            visualization_msgs::msg::Marker mark;
            mark.header.frame_id = frame_id;
            mark.header.stamp = observation_stamp;
            mark.ns = "workpiece_pca";
            mark.id = i;
            mark.type = visualization_msgs::msg::Marker::ARROW;
            mark.action =visualization_msgs::msg::Marker::ADD;
            const float length = std::sqrt(std::max(model.eigs(i),0.0f))*2.0f;
            const Eigen::Vector3f end = model.centroid + length * model.p_axes.col(i);
            geometry_msgs::msg::Point end_geo;
            end_geo.x = end.x();
            end_geo.y = end.y();
            end_geo.z = end.z();
            mark.points.push_back(start);
            mark.points.push_back(end_geo);
            mark.scale.x = 0.008;
            mark.scale.y = 0.008;
            mark.scale.z = 0.01;
            mark.color.a = 1.0;
            if (i==0){mark.color.r=1;}
            else if (i==1){mark.color.g=1;}
            else if (i==2){mark.color.b=1;}
            array_.markers.push_back(mark);
        }
        mk_pub_->publish(array_);

    }

    void publishCollisionCloud(){
         // COLLISION
        sensor_msgs::msg::Image::ConstSharedPtr depth;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info;
        {
            std::lock_guard<std::mutex>lock(collision_mutex_);
            depth = latest_depth_;
            cam_info = latest_cam_info_;
        }
        if(!depth||!cam_info){
            return;
        }
        
        auto cloud = depthToCloud(depth,cam_info,depth_range_,10);
        // PointCloud::Ptr col_cloud(new PointCloud);
        // pcl::VoxelGrid<PointT> vox;
        // vox.setInputCloud(cloud);
        // vox.setLeafSize(0.01f,0.01f,0.01f);
        // vox.filter(*col_cloud);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id  = depth->header.frame_id;
        msg.header.stamp = depth->header.stamp;
        const rclcpp::Time stamp(depth->header.stamp);

        // const double age = (now()-stamp).seconds();
        // RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
        // "collision cloud stamp %.3f, now %.3f s, age %.3f, frame= %s",
        // stamp.seconds(),now().seconds(),age,depth->header.frame_id.c_str());

        // if(!tf_buffer_->canTransform(scene_frame_,depth->header.frame_id,stamp,tf2::durationFromSec(0.05))){
        //     RCLCPP_WARN(get_logger(),"no TF %s <- %s at cloud stamp %.6f aged %.3f",
        //     scene_frame_.c_str(),depth->header.frame_id.c_str(), stamp.seconds(),(now()-stamp).seconds());
        //     return;
        // }
        col_pub_->publish(msg);
    }

    void publishCloud(const PointCloud::Ptr& cloud, 
        const std::string& frame_id, 
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& cloud_pub_){
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.frame_id  = frame_id;
            msg.header.stamp = this->now();
            cloud_pub_->publish(msg);
    }

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr col_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mk_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr profile_marker_pub_;
    rclcpp::Publisher<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr profile_estimate_pub_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::TimerBase::SharedPtr collision_timer_;

    double depth_range_;
    hb_perception::ProfileMatcher matcher_;
    rclcpp::CallbackGroup::SharedPtr processing_group_;
    rclcpp::Time last_processed_stamp_{0,0,RCL_ROS_TIME};

    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_cam_info_;
    std::mutex collision_mutex_;

};
}   

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<hb_perception::PerceptionDebugNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),4
    );
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
