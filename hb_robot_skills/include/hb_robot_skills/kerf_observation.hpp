#pragma once
#include "hb_robot_perception/perception_types.hpp"
#include <stdexcept>

namespace hb_robot_skills {
// Dense aligned RGB-D conversion. No outlier removal or voxel filtering:
// edge points and duplicate samples remain available to later evidence fusion.
inline hb_perception::PointCloud::Ptr kerfObservationCloud(
    const hb_perception::Observation& ob, double range) {
  if (!ob.rgb || !ob.depth_to_rgb || !ob.rgb_camera_info)
    throw std::invalid_argument("Missing RGB-D observation fields");
  const auto& info=*ob.rgb_camera_info;
  const auto& depth=*ob.depth_to_rgb;
  if (depth.width!=ob.rgb->width || depth.height!=ob.rgb->height ||
      info.width!=depth.width || info.height!=depth.height ||
      !std::isfinite(info.k[0]) || !std::isfinite(info.k[4]) ||
      !std::isfinite(info.k[2]) || !std::isfinite(info.k[5]) ||
      info.k[0]<=0 || info.k[4]<=0)
    throw std::invalid_argument("Aligned RGB-D dimensions or camera intrinsics invalid");
  cv::Mat meters;
  if (depth.encoding==sensor_msgs::image_encodings::TYPE_16UC1) {
    cv_bridge::toCvShare(ob.depth_to_rgb, depth.encoding)->image.convertTo(meters,CV_32FC1,0.001);
  } else if (depth.encoding==sensor_msgs::image_encodings::TYPE_32FC1) {
    meters=cv_bridge::toCvShare(ob.depth_to_rgb, depth.encoding)->image;
  } else throw std::invalid_argument("Depth must be 16UC1 millimeters or 32FC1 meters");
  const auto rgb=cv_bridge::toCvShare(ob.rgb,sensor_msgs::image_encodings::RGB8);
  auto cloud=std::make_shared<hb_perception::PointCloud>();
  for (int v=0;v<meters.rows;++v) {
    for (int u=0;u<meters.cols;++u) {
      const float z=meters.at<float>(v,u);
      if (!std::isfinite(z) || z<0.02 || z>range) continue;
      hb_perception::PointT p;
      p.x=(u-info.k[2])*z/info.k[0]; p.y=(v-info.k[5])*z/info.k[4]; p.z=z;
      const auto color=rgb->image.at<cv::Vec3b>(v,u);
      p.r=color[0]; p.g=color[1]; p.b=color[2];
      cloud->push_back(p);
    }
  }
  cloud->is_dense=true;
  return cloud;
}
} // namespace hb_robot_skills
