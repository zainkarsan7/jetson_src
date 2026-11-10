
#pragma once

#include <hb_robot_interfaces/msg/platform_supervision.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace hb_robot_skills {

class PlatformSupervisionClient {
public:
  using Message =
      hb_robot_interfaces::msg::PlatformSupervision;

  explicit PlatformSupervisionClient(
      rclcpp::Node& node,
      std::chrono::milliseconds max_age =
          std::chrono::milliseconds(500))
      : max_age_(max_age) {

    sub_ = node.create_subscription<Message>(
        "/platform_supervision",
        rclcpp::QoS(10).reliable(),
        [this](Message::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = *msg;
            received_ = Clock::now();
            received_any_ = true;
          }
          cv_.notify_all();
        });
  }

  bool stable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stableLocked(Clock::now());
  }

  bool fresh() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshLocked(Clock::now());
  }

  bool uncertain() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshLocked(Clock::now()) &&
           latest_.state == Message::UNCERTAIN;
  }

  bool untrusted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return freshLocked(Clock::now()) &&
           latest_.state == Message::UNTRUSTED;
  }

  bool waitForStable(
      std::chrono::milliseconds timeout,
      const std::function<bool()>& canceled = {}) {

    const auto deadline = Clock::now() + timeout;
    std::unique_lock<std::mutex> lock(mutex_);

    while (rclcpp::ok()) {
      if (canceled && canceled()) {
        return false;
      }

      if (stableLocked(Clock::now())) {
        return true;
      }

      if (Clock::now() >= deadline) {
        return false;
      }

      cv_.wait_for(
          lock,
          std::chrono::milliseconds(50));
    }

    return false;
  }

  Message snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

  std::string reason() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!freshLocked(Clock::now())) {
      return "Platform supervision heartbeat missing or stale";
    }

    return latest_.reason;
  }

private:
  using Clock = std::chrono::steady_clock;

  bool freshLocked(Clock::time_point now) const {
    return received_any_ &&
           now - received_ <= max_age_;
  }

  bool stableLocked(Clock::time_point now) const {
    return freshLocked(now) &&
           latest_.state == Message::STABLE &&
           latest_.supervision_valid &&
           latest_.imu_stable &&
           latest_.both_channels_valid &&
           latest_.both_channels_stable &&
           !latest_.hard_motion_detected;
  }

  std::chrono::milliseconds max_age_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;

  Message latest_;
  Clock::time_point received_{};
  bool received_any_ = false;

  rclcpp::Subscription<Message>::SharedPtr sub_;
};

}  // namespace hb_robot_skills
