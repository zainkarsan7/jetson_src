#include "hb_robot_perception/scene_model.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hb_perception{

    SceneModel::SceneModel(): scene_cloud_(std::make_shared<PointCloud>()){

    }


    bool SceneModel::addObservation(Observation ob){
        if(!ob.rgb || !ob.depth_to_rgb || !ob.rgb_camera_info){
            return false;
        }
        auto camera_cloud = observationToCloud(ob);
        if(!camera_cloud || camera_cloud->empty()){
            return false;
        }



        
        return true;
    }
    SceneModel::PointCloud::Ptr SceneModel::observationToCloud(const Observation& ob)const{
        const auto& rgb = *ob.rgb;
        const auto& depth = *ob.depth_to_rgb;
        const auto& info = *ob.rgb_camera_info;
    

    // theres some encoding check here
         if (depth.encoding != sensor_msgs::image_encodings::TYPE_16UC1){
            return nullptr;
         }

         const double fx = info.k[0];
         const double fy = info.k[4];
         const double cx = info.k[2];
         const double cy = info.k[5];

         auto cloud = std::make_shared<PointCloud>();
         cloud->reserve()

            // PointCloud::Ptr transformObsCloud(const PointCloud &cloud, const Observation & ob)const;

    }
    void SceneModel::clear(){
        observation_buffer_.clear();
        std::lock_guard<std::mutex>lock(scene_mutex);
        scene_cloud_->clear();
    }

    const std::vector<Observation>& SceneModel::observations() const{
        return observation_buffer_.observations();
    }

    SceneModel::PointCloud::Ptr SceneModel::cloud() const{
        std::lock_guard<std::mutex>lock(scene_mutex);
        return std::make_shared<PointCloud>(*scene_cloud_);
    }

    const std::size_t SceneModel::observationCount() const{
        return observation_buffer_.size();
    }





}
