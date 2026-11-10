
#include "hb_robot_perception/imu_stability_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hb_perception {

namespace {

double norm3(const std::array<double, 3>& v) {
  return std::sqrt(
      v[0] * v[0] +
      v[1] * v[1] +
      v[2] * v[2]);
}

}  // namespace

ImuStabilityMonitor::ImuStabilityMonitor(
    rclcpp::Node& node,
    const std::string& topic,
    ImuStabilityConfig config)
    : ImuStabilityMonitor(
          node, topic, config, nullptr) {}

ImuStabilityMonitor::ImuStabilityMonitor(
    rclcpp::Node& node,
    const std::string& imu_topic,
    ImuStabilityConfig config,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : config_(config) {

  if (config_.window.count() <= 0 ||
      config_.confidence_window.count() <= 0 ||
      config_.max_sample_age.count() <= 0 ||
      config_.min_samples == 0) {
    throw std::invalid_argument(
        "Invalid IMU stability configuration");
  }

  auto qos = rclcpp::SensorDataQoS();

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  sub_ = node.create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        onImu(msg, false);
      },
      options);

  if (!config_.auxiliary_accel_topic.empty()) {
    accel_sub_ =
        node.create_subscription<sensor_msgs::msg::Imu>(
            config_.auxiliary_accel_topic,
            qos,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
              onImu(msg, true);
            },
            options);
  }
}

void ImuStabilityMonitor::onImu(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    bool secondary) {

  if (msg->angular_velocity_covariance[0] == -1.0 ||
      msg->linear_acceleration_covariance[0] == -1.0) {
    return;
  }

  Sample s;
  s.received_at = Clock::now();

  s.gyro = {
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z};

  if (secondary) {
    for (int i = 0; i < 3; ++i) {
        s.gyro[i] -= config_.mpu6050_gyro_bias[i];
    }
}

  s.accel = {
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};

  for (const double v : s.gyro) {
    if (!std::isfinite(v)) return;
  }

  for (const double v : s.accel) {
    if (!std::isfinite(v)) return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    Channel& channel =
        secondary ? mpu6050_ : unilidar_;

    if (!channel.samples.empty() &&
        s.received_at -
            channel.samples.back().received_at >
                config_.max_sample_age) {

      ++channel.reception_gaps;
      channel.samples.clear();
    }

    channel.samples.push_back(s);

    const auto cutoff =
        s.received_at - config_.window;

    while (!channel.samples.empty() &&
           channel.samples.front().received_at < cutoff) {
      channel.samples.pop_front();
    }
  }

  cv_.notify_all();
}

ImuChannelMetrics ImuStabilityMonitor::evaluateWindow(
    Clock::time_point now,
    const Channel& channel) const {

  ImuChannelMetrics out;
  out.reception_gaps = channel.reception_gaps;

  const auto& samples = channel.samples;
  if (samples.empty()) return out;

  const Sample& latest = samples.back();

  out.sample_age_s =
      std::chrono::duration<double>(
          now - latest.received_at).count();

  if (now - latest.received_at >
      config_.max_sample_age) {
    return out;
  }

  const auto cutoff =
      latest.received_at - config_.window;

  std::array<double, 3> gyro_sum{};
  std::array<double, 3> accel_sum{};

  const Sample* first = nullptr;

  for (const auto& s : samples) {
    if (s.received_at < cutoff) continue;

    if (!first) first = &s;

    for (int i = 0; i < 3; ++i) {
      gyro_sum[i] += s.gyro[i];
      accel_sum[i] += s.accel[i];
    }

    ++out.samples_count;
  }

  if (!first ||
      out.samples_count < config_.min_samples) {
    return out;
  }

  const auto span =
      latest.received_at - first->received_at;

  if (span < config_.window * 9 / 10) {
    return out;
  }

  const double n =
      static_cast<double>(out.samples_count);

  for (int i = 0; i < 3; ++i) {
    out.gyro_mean[i] = gyro_sum[i] / n;
    out.accel_mean[i] = accel_sum[i] / n;
  }

  std::array<double, 3> gyro_variance{};
  std::array<double, 3> accel_variance{};

  for (const auto& s : samples) {
    if (s.received_at < cutoff) continue;

    double gyro_norm_sq = 0.0;
    double accel_deviation_sq = 0.0;

    for (int i = 0; i < 3; ++i) {
      const double dg =
          s.gyro[i] - out.gyro_mean[i];

      const double da =
          s.accel[i] - out.accel_mean[i];

      gyro_variance[i] += dg * dg;
      accel_variance[i] += da * da;

      gyro_norm_sq += s.gyro[i] * s.gyro[i];
      accel_deviation_sq += da * da;
    }

    out.peak_ang_vel_rad_s = std::max(
        out.peak_ang_vel_rad_s,
        std::sqrt(gyro_norm_sq));

    out.peak_acc_deviation_m_s2 = std::max(
        out.peak_acc_deviation_m_s2,
        std::sqrt(accel_deviation_sq));
  }

  double gyro_residual_sq = 0.0;
  double accel_residual_sq = 0.0;

  for (int i = 0; i < 3; ++i) {
    const double gyro_var = gyro_variance[i] / n;
    const double accel_var = accel_variance[i] / n;

    out.gyro_stddev[i] = std::sqrt(gyro_var);
    out.accel_stddev[i] = std::sqrt(accel_var);

    gyro_residual_sq += gyro_var;
    accel_residual_sq += accel_var;
  }

  out.ang_vel_rms_rad_s =
      std::sqrt(gyro_residual_sq);

  out.lin_acc_rms_m_s2 =
      std::sqrt(accel_residual_sq);

  out.mean_ang_vel_rad_s =
      norm3(out.gyro_mean);

  out.accel_mean_norm_m_s2 =
      norm3(out.accel_mean);

  out.latest_ang_vel_rad_s =
      norm3(latest.gyro);

  std::array<double, 3> latest_acc_deviation{};

  for (int i = 0; i < 3; ++i) {
    latest_acc_deviation[i] =
        latest.accel[i] - out.accel_mean[i];
  }

  out.latest_acc_deviation_m_s2 =
      norm3(latest_acc_deviation);

  out.gyro_rms_pass =
      out.ang_vel_rms_rad_s <
          config_.max_ang_vel_rms;

  out.accel_rms_pass =
      out.lin_acc_rms_m_s2 <
          config_.max_lin_acc_rms;

  out.mean_gyro_pass =
      out.mean_ang_vel_rad_s <
          config_.max_mean_ang_vel;

  out.valid = true;

  out.stable =
      out.gyro_rms_pass &&
      out.accel_rms_pass &&
      out.mean_gyro_pass;

  return out;
}

ImuStabilityMetrics ImuStabilityMonitor::evaluateLocked(
    Clock::time_point now) const {

  ImuStabilityMetrics out;

  out.unilidar = evaluateWindow(now, unilidar_);

  if (accel_sub_) {
    out.mpu6050 = evaluateWindow(now, mpu6050_);

    out.valid =
        out.unilidar.valid &&
        out.mpu6050.valid;

    out.stable =
        out.valid &&
        out.unilidar.stable &&
        out.mpu6050.stable;
  } else {
    // Supports single-channel users of this monitor.
    out.valid = out.unilidar.valid;
    out.stable = out.unilidar.stable;
  }

  return out;
}

void ImuStabilityMonitor::appendEvidenceLocked(
    Clock::time_point now,
    bool valid,
    bool stable) const {

  if (!evidence_.empty() &&
      now <= evidence_.back().at) {
    return;
  }

  evidence_.push_back({now, valid, stable});

  const auto cutoff =
      now - config_.confidence_window -
      std::chrono::milliseconds(150);

  while (evidence_.size() > 1 &&
         evidence_[1].at < cutoff) {
    evidence_.pop_front();
  }
}

void ImuStabilityMonitor::scoreEvidenceLocked(
    Clock::time_point now,
    ImuStabilityMetrics& metrics) const {

  const auto cutoff =
      now - config_.confidence_window;

  double good_s = 0.0;
  double covered_s = 0.0;

  for (std::size_t i = 0;
       i < evidence_.size(); ++i) {

    const auto start = std::max(
        evidence_[i].at, cutoff);

    const auto next =
        i + 1 < evidence_.size()
            ? evidence_[i + 1].at
            : now;

    // A single evaluation cannot claim arbitrarily
    // long periods of good IMU data.
    const auto end = std::min(
        next,
        evidence_[i].at +
            std::chrono::milliseconds(100));

    if (end <= start) continue;

    const double dt =
        std::chrono::duration<double>(
            end - start).count();

    if (evidence_[i].valid) {
      covered_s += dt;
    }

    if (evidence_[i].valid &&
        evidence_[i].stable) {
      good_s += dt;
    }
  }

  const double window_s =
      std::chrono::duration<double>(
          config_.confidence_window).count();

  metrics.stable_confidence = std::clamp(
      good_s / window_s, 0.0, 1.0);

  metrics.confidence_coverage_s =
      std::clamp(covered_s, 0.0, window_s);
}

ImuStabilityMetrics ImuStabilityMonitor::metrics() const {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = Clock::now();
  auto result = evaluateLocked(now);

  appendEvidenceLocked(
      now, result.valid, result.stable);

  scoreEvidenceLocked(now, result);

  return result;
}

void ImuStabilityMonitor::reset() {
  std::lock_guard<std::mutex> lock(mutex_);

  unilidar_.samples.clear();
  mpu6050_.samples.clear();
  evidence_.clear();

  // Preserve monotonic reception-gap counters.
}

bool ImuStabilityMonitor::waitForStable(
    std::chrono::milliseconds timeout,
    const std::function<bool()>& canceled) {

  const auto deadline = Clock::now() + timeout;

  std::unique_lock<std::mutex> lock(mutex_);

  while (rclcpp::ok()) {
    if (canceled && canceled()) return false;

    if (evaluateLocked(Clock::now()).stable) {
      return true;
    }

    if (Clock::now() >= deadline) {
      return false;
    }

    cv_.wait_for(
        lock, std::chrono::milliseconds(25));
  }

  return false;
}

}  // namespace hb_perception
