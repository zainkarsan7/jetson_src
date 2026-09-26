#include "hb_robot_perception/scene_model.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/image_encodings.hpp>

namespace hb_perception{

    SceneModel::SceneModel(): scene_cloud_(std::make_shared<PointCloud>()){

    }


    SceneModel::PointCloud::Ptr SceneModel::observationToCloud(const Observation& ob)const{
        auto cloud = std::make_shared<PointCloud>();


        if (!ob.rgb)
        {
            std::cout << "NO RGB" << std::endl;
            return cloud;
        }

        if (!ob.depth_to_rgb)
        {
            std::cout << "NO DEPTH" << std::endl;
            return cloud;
        }

        if (!ob.rgb_camera_info)
        {
            std::cout << "NO CAMERA INFO" << std::endl;
            return cloud;
        }

        if(!ob.rgb || !ob.depth_to_rgb || !ob.rgb_camera_info){
            return cloud;
        }


        const auto& depth = *ob.depth_to_rgb;
        const auto& rgb = *ob.rgb;
        const auto& info = *ob.rgb_camera_info;


        std::cout << "depth encoding: "
              << depth.encoding << std::endl;

    std::cout << "depth dimensions: "
              << depth.width << " x "
              << depth.height << std::endl;

    std::cout << "depth step: "
              << depth.step << std::endl;

    std::cout << "depth bytes: "
              << depth.data.size() << std::endl;



        const double fx = info.k[0];
        const double fy = info.k[4];
        const double cx = info.k[2];
        const double cy = info.k[5];


        std::cout << "intrinsics: "
              << "fx=" << fx
              << " fy=" << fy
              << " cx=" << cx
              << " cy=" << cy
              << std::endl;

        cloud->points.reserve(static_cast<std::size_t>(depth.width)*depth.height);

        auto depth_cv = cv_bridge::toCvShare(ob.depth_to_rgb);//,sensor_msgs::image_encodings::TYPE_32FC1);
        const cv::Mat& depth_mat = depth_cv->image;
        std::cout << "ROS encoding: "
          << ob.depth_to_rgb->encoding << std::endl;

        std::cout << "OpenCV type: "
                << depth_mat.type() << std::endl;

        std::cout << "Expected CV_32FC1: "
          << CV_32FC1 << std::endl;


        double min_val;
        double max_val;

        cv::minMaxLoc(depth_mat, &min_val, &max_val);

        std::cout << "depth min/max = "
                << min_val << " / "
                << max_val << std::endl;
        float min_z = std::numeric_limits<float>::max();
        float max_z = 0.0f;


        std::size_t valid = 0;
        std::size_t invalid = 0;
        
        float min_depth = std::numeric_limits<float>::max();
        float max_depth = 0.0f;
        for (std::uint32_t v = 0; v<depth.height; ++v){
            for(std::uint32_t u=0; u<depth.width; ++u){
                const auto* depth_row = reinterpret_cast<const float*>(depth.data.data()+v*depth.step);
                const float raw_depth = depth_row[u];
                if (!std::isfinite(raw_depth) || raw_depth <= 0.0f){
                    invalid++;
                    continue;
                }
                valid++;
                min_depth = std::min(min_depth, raw_depth);
                max_depth = std::max(max_depth, raw_depth);



                const float z = static_cast<float>(raw_depth);


                PointT pt;
                pt.x = static_cast<float>((u-cx)* z / fx);
                pt.y = static_cast<float>((v-cy)*z/fy);
                pt.z = z;
                cloud->points.push_back(pt);
            }
        }
        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;

          std::cout << "valid depth pixels: "
              << valid << std::endl;

    std::cout << "invalid depth pixels: "
              << invalid << std::endl;

    if (valid > 0)
    {
        std::cout << "raw depth range: "
                  << min_depth << " -> "
                  << max_depth << std::endl;
    }

    std::cout << "generated cloud points: "
              << cloud->size() << std::endl;

        return cloud;



    }
    

    bool SceneModel::addObservation(Observation ob){
        std::cout<<"adding observation"<<std::endl;
        
        const Eigen::Isometry3d T_scene_cam = tf2::transformToEigen(ob.camera_pose);
        
        /*
        * exprss the incoming camera - point cloud to base frame.
        */
        PointCloud transformed_cloud_;
        std::cout<<"could get the camera pose"<<std::endl;
        auto cam_cloud = observationToCloud(ob);
        if (!cam_cloud){
            std::cout<<"failed to get observation to cloud"<<std::endl;
        }
        pcl::transformPointCloud(*cam_cloud,transformed_cloud_,T_scene_cam.matrix().cast<float>());

        std::cout<<"transformed cloud "<< transformed_cloud_.points.size()<<std::endl;
        PointCloud valid_cloud;
        valid_cloud.points.reserve(transformed_cloud_.points.size());
        for (const auto& pt: transformed_cloud_.points){
            if (!std::isfinite(pt.x)||
            !std::isfinite(pt.y)||
            !std::isfinite(pt.z)){
                continue;
            }
            valid_cloud.points.push_back(pt);

        }

        if(valid_cloud.empty()){
            
            return false;
        
        }
        valid_cloud.width = static_cast<uint32_t>(valid_cloud.points.size());
        valid_cloud.height = 1;
        valid_cloud.is_dense = true;

        {std::lock_guard<std::mutex> lock(scene_mutex);
        *scene_cloud_+=valid_cloud;
        }
        observation_buffer_.addObservation(ob);
        return true;
    }
    
    void SceneModel::clear(){
        observation_buffer_.clear();
        std::lock_guard<std::mutex>lock(scene_mutex);
        scene_cloud_->clear();
    }

    std::vector<Observation> SceneModel::observations() const{
        return observation_buffer_.observations();
    }

    SceneModel::PointCloud::Ptr SceneModel::cloud() const{
        std::lock_guard<std::mutex>lock(scene_mutex);
        return std::make_shared<PointCloud>(*scene_cloud_);
    }

    std::size_t SceneModel::observationCount() const{
        return observation_buffer_.size();
    }

    std::size_t SceneModel::pointCount() const{
        std::lock_guard<std::mutex>lock(scene_mutex);
        return scene_cloud_->size();
    }





}
