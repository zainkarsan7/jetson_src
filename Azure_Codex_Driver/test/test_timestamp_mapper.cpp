// Copyright (c) 2026. Licensed under the MIT License.
#include "azure_kinect_ros2_driver_codex/timestamp_mapper.h"
#include <gtest/gtest.h>

using azure_kinect_ros2_driver_codex::TimestampMapper;
constexpr int64_t base = 10000000000LL;
constexpr int64_t wall = 1700000000000000000LL;
constexpr int64_t period = 33333000;

static TimestampMapper::Estimate sample(TimestampMapper& mapper, int64_t device,
                                        int64_t delay = 0, int64_t drift = 0, int64_t clock_step = 0)
{
  const auto arrival = base + device + delay + drift;
  const auto host = arrival + 1000000;
  return mapper.observe(device, arrival, host + wall + clock_step, host, host);
}

TEST(TimestampMapper, TenToHundredMillisecondArrivalJitterDoesNotResetClock)
{
  TimestampMapper mapper;
  const auto initial = sample(mapper, period);
  int64_t previous_stamp = period + initial.offset_ns;
  for (int i = 2; i < 600; ++i)
  {
    // A delayed frame followed by the next two catching up to nominal arrival.
    const int64_t delay = (i % 60 == 0) ? 100000000 : ((i % 60 == 1) ? 70000000 : ((i % 60 == 2) ? 40000000 : 0));
    const auto estimate = sample(mapper, i * period, delay);
    ASSERT_TRUE(estimate.ready);
    EXPECT_FALSE(estimate.device_reset);
    EXPECT_FALSE(estimate.ros_clock_jump);
    EXPECT_EQ(estimate.offset_ns, initial.offset_ns);
    EXPECT_GT(i * period + estimate.offset_ns, previous_stamp);
    previous_stamp = i * period + estimate.offset_ns;
  }
}

TEST(TimestampMapper, SustainedLatencyChangeIsSlewLimited)
{
  TimestampMapper mapper;
  auto previous = sample(mapper, period).offset_ns;
  for (int i = 2; i < 180; ++i)
  {
    auto estimate = sample(mapper, i * period, 100000000);
    EXPECT_LE(std::abs(estimate.offset_ns - previous), period * 500 / 1000000);
    EXPECT_FALSE(estimate.device_reset);
    previous = estimate.offset_ns;
  }
}

TEST(TimestampMapper, TracksPositiveAndNegativeOscillatorDrift)
{
  for (const int ppm : {-100, 100})
  {
    TimestampMapper mapper;
    TimestampMapper::Estimate estimate;
    for (int i = 1; i <= 1800; ++i)
      estimate = sample(mapper, i * period, 0, i * period * ppm / 1000000);
    const int64_t expected = base + wall + 1800 * period * ppm / 1000000;
    EXPECT_LT(std::abs(estimate.offset_ns - expected), 250000);
  }
}

TEST(TimestampMapper, ReanchorsDeviceClockRollback)
{
  TimestampMapper mapper;
  sample(mapper, 100 * period);
  auto estimate = sample(mapper, period);
  EXPECT_TRUE(estimate.device_reset);
  EXPECT_FALSE(estimate.ros_clock_jump);
  EXPECT_EQ(estimate.offset_ns, base + wall);
}

TEST(TimestampMapper, RealRosClockStepIsNotHiddenAsUsbJitter)
{
  TimestampMapper mapper;
  auto before = sample(mapper, period);
  auto after = sample(mapper, 2 * period, 100000000, 0, -1000000000);
  EXPECT_TRUE(after.ros_clock_jump);
  EXPECT_FALSE(after.device_reset);
  EXPECT_EQ(after.offset_ns - before.offset_ns, -1000000000);
}

TEST(TimestampMapper, RejectsPreemptedHostClockSamples)
{
  TimestampMapper mapper;
  auto before = sample(mapper, period);
  EXPECT_FALSE(mapper.observe(2 * period, base + 2 * period, wall, 0, 10000000).ready);
  auto after = sample(mapper, 3 * period);
  EXPECT_EQ(after.offset_ns, before.offset_ns);
  EXPECT_FALSE(after.ros_clock_jump);
}

TEST(TimestampMapper, StreamResetStartsFreshEpoch)
{
  TimestampMapper mapper;
  sample(mapper, 100 * period);
  mapper.reset();
  auto estimate = sample(mapper, period, 50000000);
  EXPECT_TRUE(estimate.ready);
  EXPECT_EQ(estimate.offset_ns, base + wall + 50000000);
}
