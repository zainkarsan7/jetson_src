#pragma once

// #include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_perception/rgbd_cloud_conversion.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace hb_robot_skills {

// inline hb_perception::PointCloud::Ptr kerfObservationCloud(
//     const hb_perception::Observation& ob,
//     double range)
// {
//     if (!ob.rgb || !ob.depth_to_rgb || !ob.rgb_camera_info)
//         throw std::invalid_argument("Missing RGB-D observation fields");

//     if (!std::isfinite(range) || range <= 0.0)
//         throw std::invalid_argument("Invalid depth range");

//     const auto& depth = *ob.depth_to_rgb;

//     const bool millimetres =
//         depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
//     const bool metres =
//         depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1;

//     if (!millimetres && !metres)
//         throw std::invalid_argument("Depth must be 16UC1 or 32FC1");

//     const std::uint64_t row_bytes =
//         static_cast<std::uint64_t>(depth.width) *
//         (millimetres ? 2u : 4u);

//     if (depth.step < row_bytes ||
//         static_cast<std::uint64_t>(depth.step) * depth.height >
//             depth.data.size())
//         throw std::invalid_argument("Invalid depth image buffer");

//     if (metres)
//         return hb_perception::observationToCloud(ob, range);

//     cv::Mat depth_metres;
//     cv_bridge::toCvShare(ob.depth_to_rgb, depth.encoding)
//         ->image.convertTo(depth_metres, CV_32FC1, 0.001);

//     auto normalized = ob;
//     normalized.depth_to_rgb = cv_bridge::CvImage(
//         depth.header,
//         sensor_msgs::image_encodings::TYPE_32FC1,
//         depth_metres).toImageMsg();

//     return hb_perception::observationToCloud(normalized, range);
// }
// Validate aligned RGB-D buffers and preserve 16-bit millimeter depth scaling.
// Keep the full 3-D samples; kerf cropping/registration happens in the caller.
inline hb_perception::PointCloud::Ptr kerfObservationCloud(
    const hb_perception::Observation& ob, double range) {
  return hb_perception::validatedRGBDCloud(ob, range);
}

} // namespace hb_robot_skills