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
        /**
         * extract a cross section model using a 3d point.
         * workpiece axis is used as the plane
         */
        std::optional<SectionModel>extractSection(const WorkpieceModel& model, 
        const Eigen::Vector3f &section_point,
        float thickness)const;

        /**
         * extract a cross section model using a longitudinal point.
         * workpiece axis is used as the plane
         */
        std::optional<SectionModel>extractSection(const WorkpieceModel& model, 
        float long_pos,
        float thickness)const;






    private:
        /**
         * from the long axis Z and one of the others,get a deterministic frame
         */
        Eigen::Isometry3f makeSectionFrame(const WorkpieceModel &model, const Eigen::Vector3f &origin)const;
        bool estimatePrincipalGeometry(const PointCloud::ConstPtr& cloud, WorkpieceModel& workpiece) const;
        PointCloud::Ptr getNearestCluster(const PointCloud::ConstPtr& cloud, bool downsample)const;

};




}
