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

        
        
        return true;
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
