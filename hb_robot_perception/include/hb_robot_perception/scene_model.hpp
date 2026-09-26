#pragma once

#include "hb_robot_perception/observation_buffer.hpp"
#include "hb_robot_perception/perception_types.hpp"
#include <mutex>
#include <vector>
#include <optional>
#include <cstddef>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hb_perception{

    class SceneModel{
        public:

        using PointT = pcl::PointXYZRGB;
        using PointCloud = pcl::PointCloud<PointT>;

            SceneModel();
            
            /**
             * transform the depth map to point cloud
             */
            SceneModel::PointCloud::Ptr observationToCloud(const Observation& ob)const;

            /**
             * add an observation, convert it to a point cloud
             * maybe need to transform but i dont think so
             * return false if cant be done
             */
            bool addObservation(Observation ob);
            /*get all the observations*/
            std::vector<Observation> observations() const;
            /*remove all observations*/
            void clear();
            /*get number of observations*/
            std::size_t observationCount() const;
            /*get number of points in cloud*/
            std::size_t pointCount() const;
            /*return the current point cloud*/
            PointCloud::Ptr cloud() const;

        private:

            mutable std::mutex scene_mutex;
            PointCloud::Ptr scene_cloud_;
            ObservationBuffer observation_buffer_;
            
    };

}