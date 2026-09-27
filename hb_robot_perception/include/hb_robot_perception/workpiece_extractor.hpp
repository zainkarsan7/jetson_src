#pragma once
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

class WorkpieceExtractor{
    public:
        using PointT = pcl::PointXYZRGB;
        using PointCloud = pcl::PointCloud<PointT>;
        WorkpieceExtractor() = default;
        std::optional<WorkpieceModel> extract(const PointCloud::ConstPtr &cloud) const;

    private:

        bool estimatePrincipalGeometry(const PointCloud::ConstPtr& cloud, WorkpieceModel& workpiece) const;
        PointCloud::Ptr getNearestCluster(const PointCloud::ConstPtr& cloud, bool downsample)const;

};




}
