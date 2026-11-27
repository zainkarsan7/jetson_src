#pragma once
#include "hb_robot_skills/kerf_evidence.hpp"
#include <hb_robot_interfaces/msg/kerf_depth_profile.hpp>

namespace hb_robot_skills {
struct FeatureCoordinates {
  Eigen::Vector3d start, end, inward;
  double length;
  template<class Feature> explicit FeatureCoordinates(const Feature& f) {
    start={f.start.x(),f.start.y(),0}; end={f.end.x(),f.end.y(),0};
    const Eigen::Vector3d tangent=(end-start).normalized();
    // Match CutPlanner and initializeEvidence; flange junctions are not web cuts.
    if (f.type==hb_perception::ProfileCutFeatureType::Web) {
      start+=0.0125*tangent; end-=0.0125*tangent;
    }
    length=(end-start).dot(tangent);
    inward=Eigen::Vector3d(-f.outward_normal.x(),-f.outward_normal.y(),0).normalized();
    if (!(length>0) || !inward.allFinite()) throw std::invalid_argument("Invalid feature coordinates");
  }
  double s(const Eigen::Vector3d& p) const { return (p-start).dot(end-start)/(length*length); }
  double depth(const Eigen::Vector3d& p) const { return (p-start).dot(inward); }
};

// Dense diagnostic evidence, using actual calibrated pixels, not interpolated
// cloud holes. Each pixel is kept once per frame. Bins remain the existing
// conservative evaluator's output; dense samples do not authorize recutting.
inline hb_robot_interfaces::msg::KerfDepthProfile featureDepthProfile(
    const hb_perception::ProfileModel& profile, const std::string& feature,
    const hb_perception::Observation& ob, const Eigen::Isometry3d& world_from_cut,
    const KerfRayConfig& config, double half_band, size_t stride,
    const hb_robot::recovery::Evidence& evidence) {
  const auto it=std::find_if(profile.cut_features.begin(),profile.cut_features.end(),
                           [&](const auto& f){return f.name==feature;});
  if (it==profile.cut_features.end() || stride<1 || half_band<=0)
    throw std::invalid_argument("Invalid depth-profile scope");
  FeatureCoordinates coordinates(*it);
  hb_robot_interfaces::msg::KerfDepthProfile out;
  out.feature_id=feature; out.header.stamp=ob.stamp; out.header.frame_id="workpiece";
  out.start.x=coordinates.start.x();out.start.y=coordinates.start.y();
  out.end.x=coordinates.end.x();out.end.y=coordinates.end.y();
  out.inward_normal.x=coordinates.inward.x();out.inward_normal.y=coordinates.inward.y();
  out.axial_half_band_m=half_band;out.uncertainty_m=config.depth_uncertainty;
  const hb_perception::ValidatedRGBDView depth(ob);
  const auto& k=depth.info().k;
  const Eigen::Isometry3d cut_from_camera=world_from_cut.inverse()*tf2::transformToEigen(ob.camera_pose);
  const Eigen::Isometry3d camera_from_cut=cut_from_camera.inverse();
  const Eigen::Vector3d origin=cut_from_camera.translation();
  // Bound CPU and message size to the projected feature ribbon, not the whole image.
  double umin=depth.width(),vmin=depth.height(),umax=0,vmax=0;
  for (const auto& endpoint:{coordinates.start,coordinates.end}) for (double z:{-half_band,half_band}) {
    Eigen::Vector3d p=endpoint;p.z()=z;p=camera_from_cut*p;
    if (p.z()<=0) return out;
    const double u=k[0]*p.x()/p.z()+k[2],v=k[4]*p.y()/p.z()+k[5];
    umin=std::min(umin,u);umax=std::max(umax,u);vmin=std::min(vmin,v);vmax=std::max(vmax,v);
  }
  const int left=std::max(0,int(std::floor(umin))-1),right=std::min(int(depth.width())-1,int(std::ceil(umax))+1);
  const int top=std::max(0,int(std::floor(vmin))-1),bottom=std::min(int(depth.height())-1,int(std::ceil(vmax))+1);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for (int v=top;v<=bottom && out.s.size()<100000;v+=stride) for (int u=left;u<=right;u+=stride) {
    const Eigen::Vector3d pixel((u-k[2])/k[0],(v-k[5])/k[4],1.0);
    const Eigen::Vector3d direction=cut_from_camera.linear()*pixel.normalized();
    // Reject grazing views; surface-normal projection becomes ill-conditioned.
    if (direction.dot(coordinates.inward)<0.25) continue;
    const auto interval=materialRayInterval(profile,origin,direction);
    if (!interval) continue;
    const Eigen::Vector3d entry=origin+interval->first*direction;
    const Eigen::Vector3d exit=origin+interval->second*direction;
    const double s=coordinates.s(entry);
    // A front hit on a flange or another face cannot be evidence for this web.
    if (s<0 || s>1 || std::abs(coordinates.depth(entry))>config.depth_uncertainty ||
        std::abs(entry.z())>half_band) continue;
    const double z=depth.depthMetres(u,v);
    const double range=z*pixel.norm();
    const bool valid=std::isfinite(range) && range>0;
    const bool within=std::max(std::abs(entry.z()),std::abs(exit.z()))<=config.half_kerf_width;
    const auto kind=classifyKerfRay(*interval,valid?range:nan,config.depth_uncertainty,within);
    out.s.push_back(s);
    out.measured_depth_m.push_back(valid?coordinates.depth(origin+range*direction):nan);
    out.material_entry_m.push_back(coordinates.depth(entry));
    out.material_exit_m.push_back(coordinates.depth(exit));
    out.axial_m.push_back(entry.z());
    out.classification.push_back(static_cast<uint8_t>(kind));out.within_kerf.push_back(within);
    if (out.s.size()>=100000) break;
  }
  for (const auto& bin:evidence.segments) if (bin.interval.segment_id==feature) {
    out.bin_s.push_back((bin.interval.s0+bin.interval.s1)/2);
    out.bin_classification.push_back(bin.residual_fraction>0?2:bin.open_fraction>0?1:0);
  }
  return out;
}
} // namespace hb_robot_skills
