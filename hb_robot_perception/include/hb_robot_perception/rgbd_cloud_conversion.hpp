#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include "hb_robot_perception/perception_types.hpp"

namespace hb_perception {

// Own the ROS buffers for the entire read. No cv_bridge colour conversion or
// cv::Mat allocation is needed for the camera's native BGRA8 / 32FC1 stream.
class ValidatedRGBDView {
public:
  explicit ValidatedRGBDView(const Observation& ob)
      : rgb_(ob.rgb), depth_(ob.depth_to_rgb), info_(ob.rgb_camera_info) {
    if (!rgb_ || !depth_ || !info_)
      throw std::invalid_argument("Missing RGB-D observation fields");
    const std::uint64_t pixels = std::uint64_t(depth_->width) * depth_->height;
    if (!pixels || pixels > 4000000 || rgb_->width != depth_->width ||
        rgb_->height != depth_->height || info_->width != depth_->width ||
        info_->height != depth_->height)
      throw std::invalid_argument("Invalid or mismatched RGB-D dimensions");
    if (rgb_->encoding == "bgra8") { channels_ = 4; red_ = 2; blue_ = 0; }
    else if (rgb_->encoding == "bgr8") { channels_ = 3; red_ = 2; blue_ = 0; }
    else if (rgb_->encoding == "rgba8") { channels_ = 4; red_ = 0; blue_ = 2; }
    else if (rgb_->encoding == "rgb8") { channels_ = 3; red_ = 0; blue_ = 2; }
    else throw std::invalid_argument("RGB must be bgra8, bgr8, rgba8 or rgb8");
    depth16_ = depth_->encoding == "16UC1";
    if (!depth16_ && depth_->encoding != "32FC1")
      throw std::invalid_argument("Depth must be 16UC1 or 32FC1");
    const auto valid = [](const sensor_msgs::msg::Image& image, unsigned bytes) {
      return std::uint64_t(image.step) >= std::uint64_t(image.width) * bytes &&
          std::uint64_t(image.step) * image.height <= image.data.size();
    };
    if (!valid(*rgb_, channels_) || !valid(*depth_, depth16_ ? 2 : 4) ||
        depth_->is_bigendian > 1 || rgb_->is_bigendian > 1)
      throw std::invalid_argument("Invalid RGB-D image buffer layout");
    if (!std::isfinite(info_->k[0]) || !std::isfinite(info_->k[4]) ||
        !std::isfinite(info_->k[2]) || !std::isfinite(info_->k[5]) ||
        info_->k[0] <= 0 || info_->k[4] <= 0)
      throw std::invalid_argument("Invalid camera intrinsics");
  }
  std::uint32_t width() const { return depth_->width; }
  std::uint32_t height() const { return depth_->height; }
  const sensor_msgs::msg::CameraInfo& info() const { return *info_; }
  float depthMetres(std::uint32_t u, std::uint32_t v) const {
    checkPixel(u, v);
    const unsigned bytes = depth16_ ? 2 : 4;
    const auto* pixel = depth_->data.data() + std::size_t(v) * depth_->step +
        std::size_t(u) * bytes;
    std::uint32_t bits = 0;
    for (unsigned i = 0; i < bytes; ++i)
      bits |= std::uint32_t(pixel[i]) << (8 * (depth_->is_bigendian ? bytes - 1 - i : i));
    if (depth16_) return float(bits) * 0.001f;
    float metres;
    static_assert(sizeof(metres) == sizeof(bits), "32FC1 requires a 32-bit float");
    std::memcpy(&metres, &bits, sizeof(metres));
    return metres;
  }
  std::array<std::uint8_t, 3> color(std::uint32_t u, std::uint32_t v) const {
    checkPixel(u, v);
    const auto* pixel = rgb_->data.data() + std::size_t(v) * rgb_->step +
        std::size_t(u) * channels_;
    return {pixel[red_], pixel[1], pixel[blue_]};
  }
private:
  void checkPixel(std::uint32_t u, std::uint32_t v) const {
    if (u >= width() || v >= height()) throw std::out_of_range("RGB-D pixel outside image");
  }
  sensor_msgs::msg::Image::ConstSharedPtr rgb_, depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  unsigned channels_ = 0, red_ = 0, blue_ = 0;
  bool depth16_ = false;
};

inline PointCloud::Ptr validatedRGBDCloud(const Observation& ob, double range) {
  if (!std::isfinite(range) || range <= 0)
    throw std::invalid_argument("Invalid depth range");
  const ValidatedRGBDView view(ob);
  const auto& info = view.info();
  auto cloud = std::make_shared<PointCloud>();
  cloud->reserve(std::size_t(view.width()) * view.height());
  for (std::uint32_t v = 0; v < view.height(); ++v) {
    for (std::uint32_t u = 0; u < view.width(); ++u) {
      const float z = view.depthMetres(u, v);
      if (!std::isfinite(z) || z < 0.02f || z > range) continue;
      PointT point{};
      point.x = float((double(u) - info.k[2]) * z / info.k[0]);
      point.y = float((double(v) - info.k[5]) * z / info.k[4]);
      point.z = z;
      const auto color = view.color(u, v);
      point.r = color[0]; point.g = color[1]; point.b = color[2]; point.a = 255;
      cloud->push_back(point);
    }
  }
  cloud->is_dense = true;
  return cloud;
}
} // namespace hb_perception
