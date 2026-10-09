// Copyright (c) 2026. Licensed under the MIT License.
#include "azure_kinect_ros2_driver_codex/latest_capture.h"
#include <gtest/gtest.h>
#include <future>
#include <memory>

using azure_kinect_ros2_driver_codex::LatestCapture;
using namespace std::chrono_literals;

TEST(LatestCapture, ReplacesOldFrameAndReleasesItsResources)
{
  LatestCapture<std::shared_ptr<int>> queue;
  auto first = std::make_shared<int>(1);
  std::weak_ptr<int> old = first;
  EXPECT_FALSE(queue.push(std::move(first)));
  EXPECT_TRUE(queue.push(std::make_shared<int>(2)));
  EXPECT_TRUE(old.expired());
  std::shared_ptr<int> result;
  ASSERT_TRUE(queue.pop(result, 0ms));
  EXPECT_EQ(*result, 2);
  EXPECT_FALSE(queue.pop(result, 0ms));
}

TEST(LatestCapture, CloseWakesBlockedConsumerAndDiscardsPendingData)
{
  LatestCapture<int> queue;
  std::promise<void> started;
  auto consumer = std::async(std::launch::async, [&] {
    started.set_value();
    int value;
    return queue.pop(value, 2s);
  });
  started.get_future().wait();
  queue.close();
  EXPECT_EQ(consumer.wait_for(500ms), std::future_status::ready);
  EXPECT_FALSE(consumer.get());
  EXPECT_FALSE(queue.push(42));
  int value;
  EXPECT_FALSE(queue.pop(value, 0ms));
}

TEST(LatestCapture, RecoveryClearsPendingFrame)
{
  LatestCapture<std::unique_ptr<int>> queue;
  queue.push(std::make_unique<int>(1));
  queue.clear();
  std::unique_ptr<int> value;
  EXPECT_FALSE(queue.pop(value, 0ms));
  EXPECT_FALSE(queue.push(std::make_unique<int>(2)));
  ASSERT_TRUE(queue.pop(value, 0ms));
  EXPECT_EQ(*value, 2);
}

TEST(LatestCapture, ConcurrentOverloadIsMonotonicAndRetainsFinalFrame)
{
  LatestCapture<int> queue;
  constexpr int frames = 10000;
  auto producer = std::async(std::launch::async, [&] {
    for (int i = 1; i <= frames; ++i) queue.push(i);
  });
  int previous = 0;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (previous < frames && std::chrono::steady_clock::now() < deadline)
  {
    int current;
    if (queue.pop(current, 100ms))
    {
      EXPECT_GT(current, previous);
      previous = current;
    }
  }
  producer.get();
  EXPECT_EQ(previous, frames);
}
