#pragma once
#include <hb_robot_interfaces/msg/scheduling_diagnostics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace hb_robot_skills {
// Own one executor for a group created with automatically_add_to_executor=false.
// The owner MUST stop it before destroying subscriptions or callback state.
class CallbackExecutor {
 public:
  void start(rclcpp::Node& node, const rclcpp::CallbackGroup::SharedPtr& group) {
    executor_.add_callback_group(group, node.get_node_base_interface());
    thread_ = std::thread([this] { executor_.spin(); });
  }
  void stop() {
    executor_.cancel();
    if (thread_.joinable()) thread_.join();
  }
  ~CallbackExecutor() { stop(); }
 private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

// No ROS publication or logging while holding this short statistics lock.
class SchedulingStats {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  static double ms(Time a, Time b) {
    return std::chrono::duration<double, std::milli>(b-a).count();
  }
  void arrival(size_t n) { arrivals_.fetch_add(n, std::memory_order_relaxed); }
  void receive(Time entry, double source_age, double lock_wait) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++value_.callbacks;
    if (last_rx_ != Time{}) set(value_.callback_gap_ms, value_.max_callback_gap_ms, ms(last_rx_,entry));
    last_rx_=entry;
    value_.source_age_at_entry_ms=source_age;
    set(value_.state_lock_wait_ms,value_.max_state_lock_wait_ms,lock_wait);
  }
  void reject() { std::lock_guard<std::mutex> lock(mutex_); ++value_.rejected; }
  void lockWait(Time entry) {
    const auto elapsed=ms(entry,Clock::now());
    std::lock_guard<std::mutex> lock(mutex_);
    set(value_.state_lock_wait_ms,value_.max_state_lock_wait_ms,elapsed);
  }
  void timer(Time entry, double period_ms=50.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++value_.timer_ticks;
    if (last_tick_!=Time{}) set(value_.timer_lateness_ms,value_.max_timer_lateness_ms,
                               std::max(0.0, ms(last_tick_,entry)-period_ms));
    last_tick_=entry;
  }
  void duration(Time entry) {
    const auto elapsed=ms(entry,Clock::now());
    std::lock_guard<std::mutex> lock(mutex_);
    set(value_.callback_duration_ms,value_.max_callback_duration_ms,elapsed);
  }
  void publishDuration(Time entry) {
    const auto elapsed=ms(entry,Clock::now());
    std::lock_guard<std::mutex> lock(mutex_);
    set(value_.publish_duration_ms,value_.max_publish_duration_ms,elapsed);
  }
  auto snapshot(const rclcpp::Time& now, const std::string& role) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto out=value_; out.header.stamp=now; out.role=role;
    out.middleware_arrivals=arrivals_.load(std::memory_order_relaxed); return out;
  }
 private:
  static void set(double& last,double& maximum,double v) {last=v;maximum=std::max(maximum,v);}
  std::mutex mutex_;
  std::atomic<uint64_t> arrivals_{0};
  Time last_rx_{},last_tick_{};
  hb_robot_interfaces::msg::SchedulingDiagnostics value_;
};
} // namespace hb_robot_skills
