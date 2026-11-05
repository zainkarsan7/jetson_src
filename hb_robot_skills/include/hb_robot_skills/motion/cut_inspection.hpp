#pragma once
#include "hb_robot_perception/profile_types.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace hb_robot_skills::motion {
struct CutInspectionStation {
  std::string feature;
  Eigen::Vector3d target;
  std::vector<Eigen::Isometry3d> camera_poses;
};

// Share CutPlanner's named material features, retaining BOTH web faces.
// Camera optical +Z points at the cut; optical +Y follows the extrusion.
inline std::vector<CutInspectionStation> inspectionStations(
    const hb_perception::ProfileModel& profile, const Eigen::Isometry3d& world_from_cut,
    double spacing, double standoff) {
  if (!std::isfinite(spacing) || spacing <= 0 || !std::isfinite(standoff) || standoff <= 0)
    throw std::invalid_argument("Inspection spacing and standoff must be positive");
  std::vector<CutInspectionStation> stations;
  for (const auto& feature : profile.cut_features) {
    const Eigen::Vector2d a = feature.start.cast<double>();
    const Eigen::Vector2d b = feature.end.cast<double>();
    const double length = (b-a).norm();
    if (!std::isfinite(length) || length <= 0 || !feature.outward_normal.allFinite() ||
        feature.outward_normal.norm() < 1e-6)
      throw std::invalid_argument("Invalid cut feature");
    const double count = std::ceil(length / spacing);
    if (count > 1000 || stations.size() + count + 1 > 1000)
      throw std::invalid_argument("Inspection exceeds 1000 station budget");
    const auto intervals = static_cast<std::size_t>(count);
    const Eigen::Vector3d normal(feature.outward_normal.x(), feature.outward_normal.y(), 0);
    for (std::size_t i = 0; i <= intervals; ++i) {
      const Eigen::Vector2d xy = a + (b-a) * (static_cast<double>(i)/intervals);
      const Eigen::Vector3d target(xy.x(), xy.y(), 0);
      CutInspectionStation station{feature.name, world_from_cut * target, {}};
      for (double roll : {0.0, -0.35, 0.35}) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = target + standoff * normal.normalized();
        const Eigen::Vector3d z = -normal.normalized();
        const Eigen::Vector3d x = Eigen::Vector3d::UnitZ().cross(z).normalized();
        pose.linear().col(0) = x;
        pose.linear().col(1) = z.cross(x);
        pose.linear().col(2) = z;
        pose.linear() *= Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        station.camera_poses.push_back(world_from_cut * pose);
      }
      stations.push_back(std::move(station));
    }
  }
  return stations;
}

// Keep axial depth: a section-indexable band, not a flattened 2D cloud.
inline bool inKerfBand(const Eigen::Vector3d& p,
                       const hb_perception::ProfileModel& profile,
                       double half_band, double margin) {
  return p.allFinite() && std::abs(p.z()) <= half_band &&
    std::abs(p.x()) <= profile.width/2.0 + margin &&
    std::abs(p.y()) <= profile.height/2.0 + margin;
}
}  // namespace hb_robot_skills::motion
