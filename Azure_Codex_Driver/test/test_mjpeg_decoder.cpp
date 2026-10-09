// Copyright (c) 2026. Licensed under the MIT License.
#include "azure_kinect_ros2_driver_codex/mjpeg_decoder.h"
#include <gtest/gtest.h>
#include <vector>
#include <cstring>

using azure_kinect_ros2_driver_codex::MjpegDecoder;

static k4a::image jpegFrame()
{
  auto encoder = tjInitCompress();
  std::vector<unsigned char> pixels(16 * 8 * 4, 255);
  unsigned char* jpeg = nullptr;
  unsigned long size = 0;
  const int result = tjCompress2(encoder, pixels.data(), 16, 16 * 4, 8, TJPF_BGRA,
                                 &jpeg, &size, TJSAMP_444, 95, 0);
  tjDestroy(encoder);
  if (result) throw std::runtime_error("Test JPEG encoding failed");
  // MJPG needs create_from_buffer: k4a_image_create does not allocate MJPG.
  auto image = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, 16, 8, 0,
      jpeg, size, [](void* buffer, void*) { tjFree(static_cast<unsigned char*>(buffer)); }, nullptr);
  k4a_image_set_device_timestamp_usec(image.handle(), 123456);
  k4a_image_set_system_timestamp_nsec(image.handle(), 987654321);
  return image;
}

TEST(MjpegDecoder, PreservesFormatDimensionsTimestampsAndPixels)
{
  MjpegDecoder decoder;
  auto decoded = decoder.decode(jpegFrame(), 16, 8);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(decoded.get_format(), K4A_IMAGE_FORMAT_COLOR_BGRA32);
  EXPECT_EQ(decoded.get_width_pixels(), 16);
  EXPECT_EQ(decoded.get_height_pixels(), 8);
  EXPECT_EQ(decoded.get_device_timestamp().count(), 123456);
  EXPECT_EQ(decoded.get_system_timestamp().count(), 987654321);
  for (size_t i = 0; i < decoded.get_size(); ++i) EXPECT_GE(decoded.get_buffer()[i], 250);
}

TEST(MjpegDecoder, RejectsMismatchedDimensionsBeforeDecode)
{
  MjpegDecoder decoder;
  EXPECT_FALSE(decoder.decode(jpegFrame(), 1920, 1080));
  EXPECT_FALSE(decoder.decode({}, 16, 8));
}

TEST(MjpegDecoder, CorruptFrameDoesNotReturnOldPixelsAndNextFrameRecovers)
{
  MjpegDecoder decoder;
  auto good = jpegFrame();
  auto first = decoder.decode(good, 16, 8);
  ASSERT_TRUE(first);
  auto corrupt = jpegFrame();
  std::memset(corrupt.get_buffer(), 0, corrupt.get_size());
  EXPECT_FALSE(decoder.decode(corrupt, 16, 8));
  auto next = decoder.decode(good, 16, 8);
  ASSERT_TRUE(next);
  EXPECT_EQ(next.handle(), first.handle());  // Reuse the worker's decoded buffer.
}

TEST(MjpegDecoder, DecodesIntoCallerStorageAndRejectsUndersizedBuffer)
{
  MjpegDecoder decoder;
  auto encoded = jpegFrame();
  std::vector<uint8_t> output(16 * 8 * 4, 0);
  EXPECT_FALSE(decoder.decodeInto(encoded, 16, 8, output.data(), output.size() - 1, 64));
  EXPECT_EQ(output.front(), 0);
  ASSERT_TRUE(decoder.decodeInto(encoded, 16, 8, output.data(), output.size(), 64));
  for (const auto pixel : output) EXPECT_GE(pixel, 250);
}
