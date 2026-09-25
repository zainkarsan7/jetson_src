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

            SceneModel() = default;
            
            /**
             * add an observation, convert it to a point cloud
             * maybe need to transform but i dont think so
             * return false if cant be done
             */
            bool addObservation(Observation ob);
            /*get all the observations*/
            const std::vector<Observation>& observations() const;
            /*remove all observations*/
            void clear();
            /*get number of observations*/
            const std::size_t observationCount() const;
            /*return the current point cloud*/
            PointCloud::Ptr cloud() const;

        private:

            PointCloud::Ptr observationToCloud(const Observation& ob)const;
            PointCloud::Ptr transformObsCloud(const PointCloud &cloud, const Observation & ob)const;
            /*integrate a cloud into the accumulated scene*/
            void integrateCloud(const PointCloud& cloud);
            
            mutable std::mutex scene_mutex;
            PointCloud::Ptr scene_cloud_;
            ObservationBuffer observation_buffer_;
            
    };

}