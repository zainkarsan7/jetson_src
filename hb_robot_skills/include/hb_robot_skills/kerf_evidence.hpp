#pragma once
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_perception/profile_types.hpp>
#include <hb_robot_skills/kerf_observation.hpp>
#include <algorithm>
#include <tf2_eigen/tf2_eigen.hpp>
#include <optional>

namespace hb_robot_skills {
enum class RayEvidence { UNKNOWN, OPEN, MATERIAL };
struct KerfRayConfig {
  double depth_uncertainty=0.002; // Must include sensor AND registration uncertainty.
  double half_kerf_width=0.001;
  double bin_width=0.002;       // Reported verification is at this spatial resolution.
};
// Returns the first material interval of a closed straight-sided profile. Curves,
// tangencies and a camera inside the model are deliberately unsupported.
inline std::optional<std::pair<double,double>> materialRayInterval(
    const hb_perception::ProfileModel& profile,const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction) {
  auto cross=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b){return a.x()*b.y()-a.y()*b.x();};
  std::vector<double> hits;
  for(const auto& primitive:profile.boundary) {
    const auto* line=std::get_if<hb_perception::LineSegment2D>(&primitive);
    if(!line) return std::nullopt;
    const Eigen::Vector2d a=line->a.cast<double>(),edge=line->b.cast<double>()-a;
    const Eigen::Vector2d d=direction.head<2>(),offset=a-origin.head<2>();
    const double denominator=cross(d,edge);
    if(std::abs(denominator)<1e-10) continue;
    const double t=cross(offset,edge)/denominator,u=cross(offset,d)/denominator;
    if(t>0 && u>=0 && u<=1) hits.push_back(t);
  }
  std::sort(hits.begin(),hits.end());
  hits.erase(std::unique(hits.begin(),hits.end(),[](double a,double b){return std::abs(a-b)<1e-6;}),hits.end());
  if(hits.size()<2 || hits.size()%2 || hits[1]-hits[0]<1e-5) return std::nullopt;
  return std::make_pair(hits[0],hits[1]);
}
inline RayEvidence classifyKerfRay(const std::pair<double,double>& material,
    double measured_distance,double uncertainty,bool ray_stays_in_kerf) {
  if(!ray_stays_in_kerf || !std::isfinite(measured_distance) || measured_distance<=0 ||
     !std::isfinite(uncertainty) || uncertainty<=0) return RayEvidence::UNKNOWN;
  // A valid measured return beyond the back surface provides free-space evidence.
  // Zero/NaN/no-return depth is UNKNOWN, never OPEN.
  if(measured_distance-uncertainty>material.second) return RayEvidence::OPEN;
  // A return whose entire uncertainty interval lies inside material is a bridge.
  // Boundary hits are ambiguous (calibration error or remaining surface).
  if(measured_distance-uncertainty>material.first &&
     measured_distance+uncertainty<material.second) return RayEvidence::MATERIAL;
  return RayEvidence::UNKNOWN;
}
inline hb_robot::recovery::Evidence initializeEvidence(
    const hb_perception::ProfileModel& profile,const std::vector<std::string>& required,
    const KerfRayConfig& config) {
  hb_robot::recovery::Evidence evidence;evidence.required_segments=required;
  for(const auto& name:required) {
    auto it=std::find_if(profile.cut_features.begin(),profile.cut_features.end(),
      [&](const auto& f){return f.name==name;});
    if(it==profile.cut_features.end()) throw std::invalid_argument("Unknown required cut feature");
    double length=(it->end-it->start).norm();
    if(it->type==hb_perception::ProfileCutFeatureType::Web) length-=0.025;
    if(length<=0 || config.bin_width<=0) throw std::invalid_argument("Invalid kerf bin geometry");
    const size_t count=static_cast<size_t>(std::ceil(length/config.bin_width));
    if(count>10000) throw std::invalid_argument("Kerf bin budget exceeded");
    for(size_t i=0;i<count;++i) {
      hb_robot_interfaces::msg::KerfSegmentEvidence bin;
      bin.interval.segment_id=name;bin.interval.s0=double(i)/count;bin.interval.s1=double(i+1)/count;
      evidence.segments.push_back(bin);
    }
  }
  return evidence;
}
// Fuse three depth rays across each bin. A residual observation is sticky: another
// viewpoint cannot average it away. This is a resolution-limited geometric test,
// not a mechanical proof of separation or a calibrated probability estimate.
inline void accumulateKerfEvidence(hb_robot::recovery::Evidence& evidence,
    const hb_perception::ProfileModel& profile,const hb_perception::Observation& observation,
    const Eigen::Isometry3d& world_from_cut,const KerfRayConfig& config) {
  const auto cut_from_camera=world_from_cut.inverse()*tf2::transformToEigen(observation.camera_pose);
  const auto camera_from_cut=cut_from_camera.inverse();
  const auto& info=*observation.rgb_camera_info;
  const auto& image=*observation.depth_to_rgb;
  cv::Mat depth;
  if(image.encoding==sensor_msgs::image_encodings::TYPE_16UC1)
    cv_bridge::toCvShare(observation.depth_to_rgb,image.encoding)->image.convertTo(depth,CV_32FC1,0.001);
  else if(image.encoding==sensor_msgs::image_encodings::TYPE_32FC1)
    depth=cv_bridge::toCvShare(observation.depth_to_rgb,image.encoding)->image;
  else return;
  for(auto& bin:evidence.segments) {
    if(bin.residual_fraction>0) continue;
    const auto it=std::find_if(profile.cut_features.begin(),profile.cut_features.end(),
      [&](const auto& f){return f.name==bin.interval.segment_id;});
    if(it==profile.cut_features.end()) continue;
    Eigen::Vector2d start=it->start.cast<double>(),end=it->end.cast<double>();
    if(it->type==hb_perception::ProfileCutFeatureType::Web) {
      const Eigen::Vector2d tangent=(end-start).normalized();start+=0.0125*tangent;end-=0.0125*tangent;
    }
    unsigned open=0,material=0;
    for(double f:{0.0,0.5,1.0}) {
      const double s=bin.interval.s0+f*(bin.interval.s1-bin.interval.s0);
      const Eigen::Vector2d xy=start+s*(end-start);
      const Eigen::Vector3d target(xy.x(),xy.y(),0);
      const Eigen::Vector3d camera_target=camera_from_cut*target;
      if(camera_target.z()<=0) continue;
      const int u=std::lround(info.k[0]*camera_target.x()/camera_target.z()+info.k[2]);
      const int v=std::lround(info.k[4]*camera_target.y()/camera_target.z()+info.k[5]);
      if(u<0 || v<0 || u>=depth.cols || v>=depth.rows) continue;
      const double z=depth.at<float>(v,u);
      const Eigen::Vector3d pixel_ray((u-info.k[2])/info.k[0],(v-info.k[5])/info.k[4],1);
      const Eigen::Vector3d direction=cut_from_camera.linear()*pixel_ray.normalized();
      const auto interval=materialRayInterval(profile,cut_from_camera.translation(),direction);
      if(!interval) continue;
      const auto entry=cut_from_camera.translation()+interval->first*direction;
      const auto exit=cut_from_camera.translation()+interval->second*direction;
      // Reject rays hitting a different front face or leaving the transverse slit.
      if((entry.head<2>()-xy).norm()>config.bin_width ||
         std::max(std::abs(entry.z()),std::abs(exit.z()))>config.half_kerf_width) continue;
      const auto kind=classifyKerfRay(*interval,z*pixel_ray.norm(),
        config.depth_uncertainty,true);
      open+=kind==RayEvidence::OPEN;material+=kind==RayEvidence::MATERIAL;
    }
    if(material) {
      bin.observed_fraction=1;bin.residual_fraction=1;bin.open_fraction=0;
      bin.confidence=1;bin.through_thickness_observed=false;
    } else if(open==3) {
      bin.observed_fraction=1;bin.open_fraction=1;bin.confidence=1;
      bin.through_thickness_observed=true;
    }
  }
  double observed=0,open=0;
  for(const auto& bin:evidence.segments) {observed+=bin.observed_fraction;open+=bin.open_fraction;}
  if(!evidence.segments.empty()) {
    evidence.observed_fraction=observed/evidence.segments.size();
    evidence.open_fraction=open/evidence.segments.size();evidence.unknown_fraction=1-evidence.observed_fraction;
  }
}
} // namespace hb_robot_skills
