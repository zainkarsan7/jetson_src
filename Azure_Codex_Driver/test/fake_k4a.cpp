// Copyright (c) 2026. Licensed under the MIT License.
// Test-only LD_PRELOAD device. Real SDK capture/image allocation and real JPEG
// decoding are retained; hardware I/O and calibration transformation creation
// are replaced. This library is never installed with the driver.
#include <k4a/k4a.h>
#include <turbojpeg.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

namespace
{
std::atomic_int starts{0};
std::atomic_int frames{0};
bool mode(const char* value)
{
  const char* setting = std::getenv("K4A_TEST_FAULT");
  return setting && std::strcmp(setting, value) == 0;
}
void release_pixels(void* pixels, void*) { delete[] static_cast<uint8_t*>(pixels); }
std::vector<uint8_t> make_jpeg()
{
  std::vector<uint8_t> pixels(16 * 8 * 4, 180);
  auto encoder = tjInitCompress();
  unsigned char* buffer = nullptr;
  unsigned long size = 0;
  if (tjCompress2(encoder, pixels.data(), 16, 64, 8, TJPF_BGRA, &buffer, &size, TJSAMP_444, 90, 0))
    std::abort();
  std::vector<uint8_t> result(buffer, buffer + size);
  tjFree(buffer);
  tjDestroy(encoder);
  return result;
}
}

extern "C"
{
uint32_t k4a_device_get_installed_count() { return 1; }
k4a_result_t k4a_device_open(uint32_t, k4a_device_t* device)
{
  *device = reinterpret_cast<k4a_device_t>(1);
  return K4A_RESULT_SUCCEEDED;
}
void k4a_device_close(k4a_device_t) {}
k4a_buffer_result_t k4a_device_get_serialnum(k4a_device_t, char* serial, size_t* size)
{
  const char value[] = "SIMULATED";
  if (!serial || *size < sizeof(value)) { *size = sizeof(value); return K4A_BUFFER_RESULT_TOO_SMALL; }
  std::memcpy(serial, value, sizeof(value));
  return K4A_BUFFER_RESULT_SUCCEEDED;
}
k4a_result_t k4a_device_get_version(k4a_device_t, k4a_hardware_version_t* version)
{
  *version = {};
  return K4A_RESULT_SUCCEEDED;
}
k4a_result_t k4a_device_get_calibration(k4a_device_t, k4a_depth_mode_t depth_mode,
                                      k4a_color_resolution_t resolution, k4a_calibration_t* calibration)
{
  *calibration = {};
  calibration->depth_mode = depth_mode;
  calibration->color_resolution = resolution;
  calibration->depth_camera_calibration.resolution_width = 4;
  calibration->depth_camera_calibration.resolution_height = 4;
  calibration->color_camera_calibration.resolution_width = 16;
  calibration->color_camera_calibration.resolution_height = 8;
  for (auto& from : calibration->extrinsics)
    for (auto& to : from) to.rotation[0] = to.rotation[4] = to.rotation[8] = 1;
  return K4A_RESULT_SUCCEEDED;
}
k4a_transformation_t k4a_transformation_create(const k4a_calibration_t*)
{
  return reinterpret_cast<k4a_transformation_t>(1);
}
void k4a_transformation_destroy(k4a_transformation_t) {}
k4a_result_t k4a_device_start_cameras(k4a_device_t, const k4a_device_configuration_t* config)
{
  ++starts;
  if (mode("startup_failure")) return K4A_RESULT_FAILED;
  // The driver must request compressed USB frames while publishing BGRA.
  return config->color_format == K4A_IMAGE_FORMAT_COLOR_MJPG ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}
void k4a_device_stop_cameras(k4a_device_t) {}
k4a_result_t k4a_device_start_imu(k4a_device_t) { return K4A_RESULT_SUCCEEDED; }
void k4a_device_stop_imu(k4a_device_t) {}
k4a_wait_result_t k4a_device_get_imu_sample(k4a_device_t, k4a_imu_sample_t*, int32_t)
{
  return K4A_WAIT_RESULT_TIMEOUT;
}
k4a_wait_result_t k4a_device_get_capture(k4a_device_t, k4a_capture_t* capture, int32_t timeout_ms)
{
  if (mode("timeout"))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    return K4A_WAIT_RESULT_TIMEOUT;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(33));
  const int frame = ++frames;
  if (mode("always_fail") || (mode("recover") && starts == 1 && frame == 10)) return K4A_WAIT_RESULT_FAILED;
  if (k4a_capture_create(capture) != K4A_RESULT_SUCCEEDED) return K4A_WAIT_RESULT_FAILED;
  const uint64_t system_ns = mode("stale") ? 1 : std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  k4a_image_t depth, color;
  k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 4, 4, 8, &depth);
  auto values = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth));
  for (int i = 0; i < 16; ++i) values[i] = 1000;
  k4a_image_set_device_timestamp_usec(depth, frame * 33000);
  k4a_image_set_system_timestamp_nsec(depth, system_ns);
  k4a_capture_set_depth_image(*capture, depth);
  k4a_capture_set_ir_image(*capture, depth);
  k4a_image_release(depth);
  static const auto jpeg = make_jpeg();
  auto bytes = new uint8_t[jpeg.size()];
  if (mode("corrupt")) std::memset(bytes, 0, jpeg.size());
  else std::memcpy(bytes, jpeg.data(), jpeg.size());
  k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, 16, 8, 0, bytes, jpeg.size(), release_pixels, nullptr, &color);
  k4a_image_set_device_timestamp_usec(color, frame * 33000);
  k4a_image_set_system_timestamp_nsec(color, system_ns);
  k4a_capture_set_color_image(*capture, color);
  k4a_image_release(color);
  return K4A_WAIT_RESULT_SUCCEEDED;
}
}
