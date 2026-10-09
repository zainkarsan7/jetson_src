// Copyright (c) 2026. Licensed under the MIT License.
#ifndef AZURE_KINECT_MESSAGE_POOL_H
#define AZURE_KINECT_MESSAGE_POOL_H

#include <array>
#include <memory>

namespace azure_kinect_ros2_driver_codex
{
// One producer per pool. Never overwrite a message retained by a publisher.
// Exhaustion drops optional output instead of growing memory without a bound.
template<class T, size_t Capacity = 2>
class MessagePool
{
public:
  std::shared_ptr<T> acquire()
  {
    for (auto& slot : slots_)
    {
      if (!slot) slot = std::make_shared<T>();
      if (slot.use_count() == 1) return slot;
    }
    return {};
  }
private:
  std::array<std::shared_ptr<T>, Capacity> slots_{};
};
}
#endif
