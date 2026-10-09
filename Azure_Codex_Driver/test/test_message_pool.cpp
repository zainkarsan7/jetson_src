// Copyright (c) 2026. Licensed under the MIT License.
#include "azure_kinect_ros2_driver_codex/message_pool.h"
#include <gtest/gtest.h>
#include <vector>

using azure_kinect_ros2_driver_codex::MessagePool;

TEST(MessagePool, RetainedMessageIsNeverOverwritten)
{
  MessagePool<std::vector<int>> pool;
  auto first = pool.acquire();
  first->assign(1024, 17);
  auto second = pool.acquire();
  second->assign(1024, 99);
  EXPECT_FALSE(pool.acquire());
  EXPECT_EQ(first->at(0), 17);
  EXPECT_EQ(second->at(0), 99);
  auto first_address = first.get();
  auto buffer = first->data();
  first.reset();
  auto reused = pool.acquire();
  EXPECT_EQ(reused.get(), first_address);
  EXPECT_EQ(reused->data(), buffer);
}

TEST(MessagePool, WarmSteadyStateRetainsBufferStorage)
{
  MessagePool<std::vector<uint8_t>> pool;
  auto first = pool.acquire();
  first->resize(1280 * 720 * 4);
  auto buffer = first->data();
  first.reset();
  for (int i = 0; i < 100; ++i)
  {
    auto frame = pool.acquire();
    frame->resize(1280 * 720 * 4);
    EXPECT_EQ(frame->data(), buffer);
  }
}
