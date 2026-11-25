#pragma once
#include "hb_robot_perception/profile_types.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace hb_robot_skills::motion {
struct CutInspectionStation {
  std::string feature;
  Eigen::Vector3d target;
  std::vector<Eigen::Isometry3d> camera_poses; // World-from-RGB-optical poses.
};

// Share CutPlanner's named material features, retaining BOTH web faces.
// Camera optical +Z points at the cut. Preserve reference camera roll rather
// than tying optical +Y (image DOWN) to the workpiece extrusion direction.
inline std::vector<CutInspectionStation> inspectionStations(
    const hb_perception::ProfileModel& profile, const Eigen::Isometry3d& world_from_cut,
    double spacing, double standoff,
    const Eigen::Matrix3d& reference_optical_rotation = Eigen::Matrix3d::Identity()) {
  if (!std::isfinite(spacing) || spacing <= 0 || !std::isfinite(standoff) || standoff <= 0)
    throw std::invalid_argument("Inspection spacing and standoff must be positive");
  if (!reference_optical_rotation.allFinite() ||
      !(reference_optical_rotation.transpose()*reference_optical_rotation)
        .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(reference_optical_rotation.determinant()-1.0)>1e-6)
    throw std::invalid_argument("Invalid reference optical rotation");
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
      Eigen::Isometry3d nominal = Eigen::Isometry3d::Identity();
      nominal.translation() = world_from_cut * (target + standoff * normal.normalized());
      const Eigen::Vector3d z = (station.target - nominal.translation()).normalized();
      Eigen::Vector3d y = reference_optical_rotation.col(1);
      y -= y.dot(z)*z;
      Eigen::Vector3d x;
      if (y.norm() > 1e-6) {
        y.normalize();
        x = y.cross(z).normalized();
      } else {
        // Looking along reference image-down: use reference image-right to
        // resolve the roll singularity, never an arbitrary extrusion sign.
        x = reference_optical_rotation.col(0);
        x -= x.dot(z)*z;
        x.normalize();
      }
      nominal.linear().col(0) = x;
      nominal.linear().col(1) = z.cross(x).normalized();
      nominal.linear().col(2) = z;
      for (double roll : {0.0, -0.35, 0.35}) {
        Eigen::Isometry3d pose = nominal;
        pose.linear() *= Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        station.camera_poses.push_back(pose);
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
