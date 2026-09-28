#include "hb_robot_perception/scene_model.hpp"
#include "hb_robot_perception/perception_types.hpp"
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
    

        bool SceneModel::addObservation(Observation ob, double depth_range){
            std::cout<<"adding observation"<<std::endl;
        
        const Eigen::Isometry3d T_scene_cam = tf2::transformToEigen(ob.camera_pose);
        
        /*
        * exprss the incoming camera - point cloud to base frame.
        */
        PointCloud transformed_cloud_;
        std::cout<<"could get the camera pose"<<std::endl;
        auto cam_cloud = observationToCloud(ob,depth_range);
        if (!cam_cloud){
            std::cout<<"failed to get observation to cloud"<<std::endl;
        }
        pcl::transformPointCloud(*cam_cloud,transformed_cloud_,T_scene_cam.matrix().cast<float>());
        PointCloud voxel_downsampled_;
        std::cout<< "PointCloud before filtering: "<< transformed_cloud_.points.size()<< " data points"<<std::endl;
        pcl::VoxelGrid<PointT> vox;
        vox.setInputCloud(std::make_shared<PointCloud>(transformed_cloud_));
        vox.setLeafSize(0.005f,0.005f,0.005f);
        vox.filter(voxel_downsampled_);

        std::cout<< "PointCloud after filtering: "<< voxel_downsampled_.points.size()<< " data points"<<std::endl;

        PointCloud valid_cloud;
        valid_cloud.points.reserve(voxel_downsampled_.points.size());
        for (const auto& pt: voxel_downsampled_.points){
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
    
    void SceneModel::downsample(float leafsize){
        std::lock_guard<std::mutex>lock(scene_mutex);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(scene_cloud_);
        voxel.setLeafSize(leafsize,leafsize,leafsize);
        auto filtered = std::make_shared<PointCloud>();
        voxel.filter(*filtered);
        scene_cloud_ = std::move(filtered);
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
