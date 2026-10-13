#pragma once
#include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include "hb_robot_perception/profile_matcher.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <array>
#include <optional>
#include <vector>
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

        Eigen::Vector3f estimateAxisFromNormals(
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
            float radius = 0.025f
        )const;

        WorkpieceModel withAxis(const WorkpieceModel& source, Eigen::Vector3f axis)const;



    private:
        /**
         * from the long axis Z and one of the others,get a deterministic frame
         */
        Eigen::Isometry3f makeSectionFrame(const WorkpieceModel &model, const Eigen::Vector3f &origin)const;
        bool estimatePrincipalGeometry(const PointCloud::ConstPtr& cloud, WorkpieceModel& workpiece) const;
        PointCloud::Ptr getNearestCluster(const PointCloud::ConstPtr& cloud, bool downsample)const;

};


 struct AxisValidation {
    Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    int valid_sections = 0;
    float mean_rms = std::numeric_limits<float>::infinity();
    float mean_inlier_fraction = 0.0f;
    float score = std::numeric_limits<float>::infinity();
};

AxisValidation validateAxis(
    const WorkpieceExtractor& extractor,
    const ProfileMatcher& matcher,
    const WorkpieceModel& model,
    const std::vector<ProfileModel>& profiles);



}
