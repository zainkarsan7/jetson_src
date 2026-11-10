
#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <string>

namespace hb_perception {

struct ImuStabilityConfig {
  double max_ang_vel_rms = 0.01;
  double max_lin_acc_rms = 0.1;
  double max_mean_ang_vel = 0.03;

  std::size_t min_samples = 20;

  std::chrono::milliseconds window{500};
  std::chrono::milliseconds max_sample_age{200};
  std::chrono::milliseconds confidence_window{2000};

  // Kept for compatibility with existing task_state_server.
  // This is now a complete secondary six-axis IMU.
  std::string auxiliary_accel_topic;
  std::array<double, 3> mpu6050_gyro_bias{
      0.0, 0.0, 0.0
  };

};

struct ImuChannelMetrics {
  bool valid = false;
  bool stable = false;

  std::size_t samples_count = 0;
  std::uint64_t reception_gaps = 0;

  double sample_age_s =
      std::numeric_limits<double>::infinity();

  std::array<double, 3> gyro_mean{};
  std::array<double, 3> gyro_stddev{};
  std::array<double, 3> accel_mean{};
  std::array<double, 3> accel_stddev{};

  double ang_vel_rms_rad_s = 0.0;
  double lin_acc_rms_m_s2 = 0.0;
  double mean_ang_vel_rad_s = 0.0;

  double peak_ang_vel_rad_s = 0.0;
  double peak_acc_deviation_m_s2 = 0.0;

  double latest_ang_vel_rad_s = 0.0;
  double latest_acc_deviation_m_s2 = 0.0;

  double accel_mean_norm_m_s2 = 0.0;

  bool gyro_rms_pass = false;
  bool accel_rms_pass = false;
  bool mean_gyro_pass = false;
};

struct ImuStabilityMetrics {
  ImuChannelMetrics unilidar;
  ImuChannelMetrics mpu6050;

  bool valid = false;
  bool stable = false;

  double stable_confidence = 0.0;
  double confidence_coverage_s = 0.0;

  // These are raw measurements, not a calibrated
  // safety-certified motion decision.
  bool hard_motion_detected = false;
};

class ImuStabilityMonitor {
public:
  ImuStabilityMonitor(
      rclcpp::Node& node,
      const std::string& imu_topic,
      ImuStabilityConfig config = {});

  ImuStabilityMonitor(
      rclcpp::Node& node,
      const std::string& imu_topic,
      ImuStabilityConfig config,
      rclcpp::CallbackGroup::SharedPtr callback_group);

  bool waitForStable(
      std::chrono::milliseconds timeout,
      const std::function<bool()>& canceled = {});

  ImuStabilityMetrics metrics() const;

  void reset();

  const ImuStabilityConfig& config() const {
    return config_;
  }

private:
  using Clock = std::chrono::steady_clock;

  struct Sample {
    Clock::time_point received_at;
    std::array<double, 3> gyro{};
    std::array<double, 3> accel{};
  };

  struct Channel {
    std::deque<Sample> samples;
    std::uint64_t reception_gaps = 0;
  };

  struct Evidence {
    Clock::time_point at;
    bool valid = false;
    bool stable = false;
  };

  void onImu(
      const sensor_msgs::msg::Imu::SharedPtr msg,
      bool secondary);

  ImuChannelMetrics evaluateWindow(
      Clock::time_point now,
      const Channel& channel) const;

  ImuStabilityMetrics evaluateLocked(
      Clock::time_point now) const;

  void appendEvidenceLocked(
      Clock::time_point now,
      bool valid,
      bool stable) const;

  void scoreEvidenceLocked(
      Clock::time_point now,
      ImuStabilityMetrics& metrics) const;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      sub_, accel_sub_;

  ImuStabilityConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;

  Channel unilidar_;
  Channel mpu6050_;

  mutable std::deque<Evidence> evidence_;
};

enum class PlatformMotionState {
  UNKNOWN,
  STABLE,
  SETTLING,
  DISTURBED,
  LATCHED
};

struct PlatformSupervisorConfig {
  double max_peak_acc_deviation_m_s2 = 0.0;
  double max_peak_ang_vel_rad_s = 0.0;
  double max_ang_vel_rms_rad_s = 0.0;
  double max_lin_acc_rms_m_s2 = 0.0;
  std::chrono::milliseconds stable_dwell{0};
};

// Preserved for compatibility with other code that may
// use the separate PlatformMotionSupervisor API.
class PlatformMotionSupervisor {
public:
  explicit PlatformMotionSupervisor(
      PlatformSupervisorConfig config);

  PlatformMotionState update(
      const ImuStabilityMetrics& metrics);

  bool executionPermitted();
  bool disturbanceLatched();
  void resetLatch();

private:
  PlatformMotionState state_ =
      PlatformMotionState::UNKNOWN;

  PlatformSupervisorConfig config_;
};

}  // namespace hb_perception
