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

namespace hb_perception{

    SceneModel::SceneModel(): scene_cloud_(std::make_shared<PointCloud>()){

    }


    bool SceneModel::addObservation(Observation ob){
        if(!ob.point_cloud_||ob.point_cloud_->data.empty()){
            return false;
        }

        PointCloud cam_cloud;

        pcl::fromROSMsg(*ob.point_cloud_,cam_cloud);
        if (cam_cloud.empty()){return false;}

        const Eigen::Isometry3d T_scene_cam = tf2::transformToEigen(ob.camera_pose);
        
        /*
        * exprss the incoming camera - point cloud to base frame.
        */
        PointCloud transformed_cloud_;
        pcl::transformPointCloud(cam_cloud,transformed_cloud_,T_scene_cam.matrix().cast<float>());


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

        if(valid_cloud.empty()){return false;}
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
