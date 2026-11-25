#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace hb_robot_skills {

// Caller serializes access. Confirmation uses measured velocity messages,
// never a cached RobotState or position-only estimate of zero velocity.
class RobotStationaryWindow {
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  static constexpr double velocity_limit = 0.01;
  static constexpr std::int64_t freshness_ns = 250000000;
  static constexpr std::int64_t future_tolerance_ns = 5000000;

  explicit RobotStationaryWindow(std::vector<std::string> joints)
    : joints_(std::move(joints)) {}

  void reset(Time started, std::int64_t ros_started) {
    started_ = started;
    ros_started_ = ros_started;
    last_received_ = Time{};
    last_stamp_ = 0;
    received_ = 0;
    max_velocity_ = 0;
    reject("waiting_for_fresh_joint_states");
  }

  void reject(const std::string& reason) {
    quiet_ = false;
    quiet_samples_ = 0;
    reason_ = reason;
  }

  void observe(const std::vector<std::string>& names,
               const std::vector<double>& positions,
               const std::vector<double>& velocities,
               std::int64_t stamp, std::int64_t ros_now, Time received) {
    // Other devices may publish on the same topic. They cannot refresh or
    // invalidate the arm's measurements, which must contain its full joint set.
    if (!names.empty() && std::none_of(names.begin(), names.end(), [&](const auto& name) {
          return std::find(joints_.begin(),joints_.end(),name)!=joints_.end();
        })) return;

    ++received_;
    const bool delivery_gap = last_stamp_ > 0 &&
      (received-last_received_ > std::chrono::nanoseconds(freshness_ns) ||
       stamp-last_stamp_ > freshness_ns);
    last_received_ = received;
    const auto age = ros_now-stamp;
    if (stamp <= 0 || stamp <= ros_started_ || received < started_) {
      reject("feedback_predates_confirmation"); return;
    }
    if (age < -future_tolerance_ns || age > freshness_ns) {
      reject("joint_state_source_timestamp_not_fresh"); return;
    }
    if (stamp <= last_stamp_) {
      reject("joint_state_timestamp_not_advancing"); return;
    }
    last_stamp_ = stamp;
    if (joints_.empty() || names.size()!=positions.size() || names.size()!=velocities.size()) {
      reject("missing_positions_or_velocities"); return;
    }

    max_velocity_ = 0;
    for (const auto& joint : joints_) {
      const auto it = std::find(names.begin(),names.end(),joint);
      if (it==names.end() || std::count(names.begin(),names.end(),joint)!=1) {
        reject("missing_or_duplicate_joint:" + joint); return;
      }
      const auto index = static_cast<std::size_t>(it-names.begin());
      if (!std::isfinite(positions[index]) || !std::isfinite(velocities[index])) {
        reject("nonfinite_joint_feedback:" + joint); return;
      }
      max_velocity_ = std::max(max_velocity_,std::abs(velocities[index]));
    }
    if (max_velocity_ > velocity_limit) {
      reject("robot_still_moving"); return;
    }
    if (!quiet_ || delivery_gap) {
      quiet_since_ = received;
      quiet_stamp_ = stamp;
      quiet_samples_ = 0;
    }
    quiet_ = true;
    ++quiet_samples_;
    reason_ = "collecting_stationary_feedback";
  }

  bool confirmed(Time now, std::int64_t ros_now, std::chrono::milliseconds stable) {
    if (!quiet_) return false;
    const auto age = ros_now-last_stamp_;
    if (now < last_received_ || now-last_received_ > std::chrono::nanoseconds(freshness_ns) ||
        age < -future_tolerance_ns || age > freshness_ns) {
      reject("stationary_feedback_expired"); return false;
    }
    // Elapsed time alone cannot confirm a stop. Both receipt times and source
    // timestamps of DISTINCT valid messages must span the requested interval.
    return stable.count()>0 && quiet_samples_>=2 &&
      last_received_-quiet_since_ >= stable &&
      last_stamp_-quiet_stamp_ >= std::chrono::duration_cast<std::chrono::nanoseconds>(stable).count();
  }

  const std::string& reason() const { return reason_; }
  std::uint64_t received() const { return received_; }
  std::uint64_t quietSamples() const { return quiet_samples_; }
  double maxVelocity() const { return max_velocity_; }

private:
  std::vector<std::string> joints_;
  Time started_{}, last_received_{}, quiet_since_{};
  std::int64_t ros_started_=0, last_stamp_=0, quiet_stamp_=0;
  std::uint64_t received_=0, quiet_samples_=0;
  bool quiet_=false;
  double max_velocity_=0;
  std::string reason_="not_initialized";
};
} // namespace hb_robot_skills
