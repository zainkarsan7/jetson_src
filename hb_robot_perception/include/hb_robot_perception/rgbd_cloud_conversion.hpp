
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include "hb_robot_perception/perception_types.hpp"

namespace hb_perception {

inline PointCloud::Ptr observationToCloud(
    const Observation& ob,
    double range)
{
    if (!ob.rgb || !ob.depth_to_rgb ||
        !ob.rgb_camera_info)
    {
        throw std::invalid_argument(
            "Missing RGB-D observation fields");
    }

    if (!std::isfinite(range) || range <= 0.0)
        throw std::invalid_argument("Invalid depth range");

    const auto& rgb = *ob.rgb;
    const auto& depth = *ob.depth_to_rgb;
    const auto& info = *ob.rgb_camera_info;

    if (rgb.width == 0 || rgb.height == 0 ||
        depth.width != rgb.width ||
        depth.height != rgb.height ||
        info.width != rgb.width ||
        info.height != rgb.height)
    {
        throw std::invalid_argument(
            "RGB-D dimensions do not match");
    }

    const auto validBuffer = [](
        const sensor_msgs::msg::Image& image,
        std::uint32_t bytes_per_pixel)
    {
        const std::uint64_t minimum_step =
            static_cast<std::uint64_t>(image.width) *
            bytes_per_pixel;

        const std::uint64_t required =
            static_cast<std::uint64_t>(image.step) *
            image.height;

        return image.step >= minimum_step &&
               required <= image.data.size();
    };

    const bool depth16 =
        depth.encoding ==
        sensor_msgs::image_encodings::TYPE_16UC1;

    const bool depth32 =
        depth.encoding ==
        sensor_msgs::image_encodings::TYPE_32FC1;

    if ((!depth16 && !depth32) ||
        !validBuffer(depth, depth16 ? 2 : 4))
    {
        throw std::invalid_argument(
            "Invalid depth encoding or buffer");
    }

    if (!std::isfinite(info.k[0]) ||
        !std::isfinite(info.k[4]) ||
        !std::isfinite(info.k[2]) ||
        !std::isfinite(info.k[5]) ||
        info.k[0] <= 0.0 ||
        info.k[4] <= 0.0)
    {
        throw std::invalid_argument(
            "Invalid camera intrinsics");
    }

    // cv_bridge performs the encoding conversion and
    // validates the RGB image's declared layout.
    const auto color = cv_bridge::toCvShare(
        ob.rgb,
        sensor_msgs::image_encodings::RGB8);

    const auto depth_view = cv_bridge::toCvShare(
        ob.depth_to_rgb,
        depth.encoding);

    if (color->image.rows !=
            static_cast<int>(depth.height) ||
        color->image.cols !=
            static_cast<int>(depth.width))
    {
        throw std::invalid_argument(
            "Converted RGB/depth dimensions differ");
    }

    auto cloud = std::make_shared<PointCloud>();

    const auto pixel_count =
        static_cast<std::uint64_t>(depth.width) *
        depth.height;

    if (pixel_count > 4000000)
        throw std::invalid_argument(
            "Unexpectedly large RGB-D image");

    cloud->reserve(
        static_cast<std::size_t>(pixel_count));

    for (int v = 0; v < depth_view->image.rows; ++v)
    {
        for (int u = 0; u < depth_view->image.cols; ++u)
        {
            float z;

            if (depth16)
            {
                z = depth_view->image.at<std::uint16_t>(
                    v, u) * 0.001f;
            }
            else
            {
                z = depth_view->image.at<float>(v, u);
            }

            if (!std::isfinite(z) ||
                z < 0.02f ||
                z > range)
            {
                continue;
            }

            PointT point;

            point.x = static_cast<float>(
                (u - info.k[2]) * z / info.k[0]);

            point.y = static_cast<float>(
                (v - info.k[5]) * z / info.k[4]);

            point.z = z;

            const auto pixel =
                color->image.at<cv::Vec3b>(v, u);

            point.r = pixel[0];
            point.g = pixel[1];
            point.b = pixel[2];

            cloud->push_back(point);
        }
    }

    cloud->is_dense = true;
    return cloud;
}

}  // namespace hb_perception
