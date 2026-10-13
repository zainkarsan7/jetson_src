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

        Eigen::Vector3f WorkpieceExtractor::estimateAxisFromNormals(
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
            float radius = 0.025f
        )const;

        WorkpieceModel withAxis(const WorkpieceModel& source, Eigen::Vector3f axis);



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
    const std::vector<ProfileModel>& profiles)
{
    AxisValidation result;
    result.axis = model.l_axes;

    constexpr std::array<float, 5> offsets{
        -0.06f, -0.03f, 0.0f, 0.03f, 0.06f
    };

    float rms_sum = 0.0f;
    float inlier_sum = 0.0f;

    for (const float s : offsets)
    {
        const auto section =
            extractor.extractSection(model, s, 0.015f);

        if (!section || section->points_2d.size() < 30)
            continue;

        const auto matches =
            matcher.match(*section, profiles);

        if (matches.empty())
            continue;

        const auto& best = matches.front();

        if (!std::isfinite(best.rms_dist))
            continue;

        ++result.valid_sections;
        rms_sum += best.rms_dist;
        inlier_sum += best.inlier_fraction;
    }

    if (result.valid_sections == 0)
        return result;

    result.mean_rms =
        rms_sum / result.valid_sections;

    result.mean_inlier_fraction =
        inlier_sum / result.valid_sections;

    // Diagnostic ranking, not yet a calibrated confidence score.
    result.score =
        result.mean_rms
        + 0.01f * (1.0f - result.mean_inlier_fraction)
        + 0.01f * (5 - result.valid_sections);

    return result;
}




}
