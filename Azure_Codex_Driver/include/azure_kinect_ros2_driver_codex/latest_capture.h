// Copyright (c) 2026. Licensed under the MIT License.
#ifndef AZURE_KINECT_LATEST_CAPTURE_H
#define AZURE_KINECT_LATEST_CAPTURE_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

namespace azure_kinect_ros2_driver
{
// Single pending item: slow consumers never build an unbounded frame backlog.
template<class T>
class LatestCapture
{
public:
  bool push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) return false;
    const bool replaced = pending_.has_value();
    pending_ = std::move(value);
    ready_.notify_one();
    return replaced;
  }

  bool pop(T& value, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait_for(lock, timeout, [this] { return closed_ || pending_.has_value(); });
    if (closed_ || !pending_) return false;
    value = std::move(*pending_);
    pending_.reset();
    return true;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    pending_.reset();
    ready_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<T> pending_;
  bool closed_ = false;
};
}
#endif
