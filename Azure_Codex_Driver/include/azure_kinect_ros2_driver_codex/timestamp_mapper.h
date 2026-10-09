// Copyright (c) 2026. Licensed under the MIT License.
#ifndef AZURE_KINECT_TIMESTAMP_MAPPER_H
#define AZURE_KINECT_TIMESTAMP_MAPPER_H

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>

namespace azure_kinect_ros2_driver_codex
{
// Single-writer device -> monotonic -> ROS time mapping. Host arrival includes
// transport delay: its lower envelope is an estimate, not exposure ground truth.
class TimestampMapper
{
public:
  struct Estimate
  {
    bool ready = false;
    bool device_reset = false;
    bool ros_clock_jump = false;
    int64_t offset_ns = 0;
    int64_t arrival_residual_ns = 0;
  };

  TimestampMapper(int64_t window_ns = 2000000000LL, int64_t slew_ppm = 500)
    : window_ns_(window_ns), slew_ppm_(slew_ppm) {}

  void reset() { initialized_ = false; minima_.clear(); }

  Estimate observe(int64_t device_ns, int64_t arrival_ns, int64_t ros_now_ns,
                   int64_t steady_before_ns, int64_t steady_after_ns)
  {
    Estimate result;
    // A preemption between ROS and monotonic reads is not a clock adjustment.
    if (steady_after_ns < steady_before_ns || steady_after_ns - steady_before_ns > 1000000 ||
        arrival_ns <= 0 || device_ns < 0) return result;
    const int64_t bridge = ros_now_ns - (steady_before_ns + (steady_after_ns - steady_before_ns) / 2);
    const int64_t candidate = arrival_ns - device_ns;
    const bool device_reset = initialized_ && device_ns < last_device_ns_;
    if (!initialized_ || device_reset)
    {
      minima_.clear();
      device_to_steady_ns_ = candidate;
      steady_to_ros_ns_ = bridge;
      result.device_reset = device_reset;
      initialized_ = true;
    }
    else
    {
      // The paired host clocks distinguish real wall/ROS clock steps from USB
      // arrival jitter. Large true clock steps are visible, never smoothed away.
      const int64_t bridge_error = bridge - steady_to_ros_ns_;
      if (std::abs(bridge_error) > 5000000)
      {
        steady_to_ros_ns_ = bridge;
        result.ros_clock_jump = true;
      }
      else
      {
        steady_to_ros_ns_ += bridge_error / 16;
      }
    }
    while (!minima_.empty() && minima_.front().device_ns < device_ns - window_ns_) minima_.pop_front();
    while (!minima_.empty() && minima_.back().offset_ns >= candidate) minima_.pop_back();
    minima_.push_back({device_ns, candidate});
    if (!result.device_reset)
    {
      const int64_t elapsed = std::clamp<int64_t>(device_ns - last_device_ns_, 0, 100000000);
      const int64_t max_step = elapsed * slew_ppm_ / 1000000;
      device_to_steady_ns_ += std::clamp(minima_.front().offset_ns - device_to_steady_ns_, -max_step, max_step);
    }
    last_device_ns_ = device_ns;
    result.ready = true;
    result.offset_ns = device_to_steady_ns_ + steady_to_ros_ns_;
    result.arrival_residual_ns = candidate - device_to_steady_ns_;
    return result;
  }

private:
  struct Sample { int64_t device_ns; int64_t offset_ns; };
  std::deque<Sample> minima_;
  const int64_t window_ns_;
  const int64_t slew_ppm_;
  bool initialized_ = false;
  int64_t device_to_steady_ns_ = 0;
  int64_t steady_to_ros_ns_ = 0;
  int64_t last_device_ns_ = 0;
};
}
#endif
