// Copyright (c) 2026. Licensed under the MIT License.
#ifndef AZURE_KINECT_MJPEG_DECODER_H
#define AZURE_KINECT_MJPEG_DECODER_H

#include <k4a/k4a.hpp>
#include <turbojpeg.h>
#include <stdexcept>

namespace azure_kinect_ros2_driver
{
// Worker-owned decoder. Returned pixels are valid until the next decode call;
// ROS messages must own their copied pixels before that call (no buffer loaning).
class MjpegDecoder
{
public:
  MjpegDecoder() : decoder_(tjInitDecompress())
  {
    if (!decoder_) throw std::runtime_error("Failed to initialize TurboJPEG");
  }
  ~MjpegDecoder() { tjDestroy(decoder_); }
  MjpegDecoder(const MjpegDecoder&) = delete;
  MjpegDecoder& operator=(const MjpegDecoder&) = delete;

  k4a::image decode(const k4a::image& encoded, int expected_width, int expected_height)
  {
    if (!encoded || encoded.get_format() != K4A_IMAGE_FORMAT_COLOR_MJPG ||
        expected_width <= 0 || expected_height <= 0) return {};
    int width = 0, height = 0, subsampling = 0, colorspace = 0;
    if (tjDecompressHeader3(decoder_, encoded.get_buffer(), encoded.get_size(),
        &width, &height, &subsampling, &colorspace) != 0 ||
        width != expected_width || height != expected_height) return {};
    // Allocate only after validating the JPEG dimensions against calibration.
    if (!output_ || output_.get_width_pixels() != width || output_.get_height_pixels() != height)
      output_ = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4);
    if (tjDecompress2(decoder_, encoded.get_buffer(), encoded.get_size(), output_.get_buffer(),
        width, output_.get_stride_bytes(), height, TJPF_BGRA, TJFLAG_FASTDCT) != 0) return {};
    k4a_image_set_device_timestamp_usec(output_.handle(), encoded.get_device_timestamp().count());
    k4a_image_set_system_timestamp_nsec(output_.handle(), encoded.get_system_timestamp().count());
    output_.set_exposure_time(encoded.get_exposure());
    output_.set_white_balance(encoded.get_white_balance());
    output_.set_iso_speed(encoded.get_iso_speed());
    return output_;
  }

private:
  tjhandle decoder_;
  k4a::image output_;
};
}
#endif
