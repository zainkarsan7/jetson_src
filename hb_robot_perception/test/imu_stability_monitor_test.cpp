#include <gtest/gtest.h>
#include <hb_robot_perception/imu_stability_monitor.hpp>
#include <thread>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;
class ImuMonitorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0,nullptr);
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node=std::make_shared<rclcpp::Node>("imu_monitor_regression");
    hb_perception::ImuStabilityConfig config;
    config.window=100ms;config.max_sample_age=100ms;config.min_samples=5;
    monitor=std::make_unique<hb_perception::ImuStabilityMonitor>(*node,"/test/recovery_imu",config);
    pub=node->create_publisher<sensor_msgs::msg::Imu>("/test/recovery_imu",rclcpp::SensorDataQoS());
    executor->add_node(node);
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline) {
      executor->spin_some();std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(pub->get_subscription_count(),0u);
  }
  void TearDown() override {
    executor->remove_node(node);monitor.reset();pub.reset();node.reset();executor.reset();rclcpp::shutdown();
  }
  void stream(double gyro, std::chrono::milliseconds duration=350ms) {
    const auto end=std::chrono::steady_clock::now()+duration;
    while(std::chrono::steady_clock::now()<end) {
      sensor_msgs::msg::Imu message;message.angular_velocity.z=gyro;
      message.linear_acceleration.z=9.81;
      pub->publish(message);executor->spin_some();std::this_thread::sleep_for(5ms);executor->spin_some();
    }
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
  std::unique_ptr<hb_perception::ImuStabilityMonitor> monitor;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};
TEST_F(ImuMonitorTest, FreshStationaryWindowIsValidAndStable) {
  EXPECT_FALSE(monitor->metrics().valid);
  EXPECT_TRUE(std::isinf(monitor->metrics().sample_age_s));
  stream(0);
  const auto metrics=monitor->metrics();
  EXPECT_TRUE(metrics.valid);EXPECT_TRUE(metrics.stable);
  EXPECT_GE(metrics.samples_count,5u);EXPECT_LT(metrics.sample_age_s,0.1);
}
TEST_F(ImuMonitorTest, DelayedReadPreservesFreshStationaryWindow) {
  stream(0);
  const auto before=monitor->metrics();
  ASSERT_TRUE(before.valid);
  // With a 100 ms window and 90% coverage, a read 35 ms later used to
  // invalidate the pose even though the same samples were still fresh.
  std::this_thread::sleep_for(35ms);
  const auto after=monitor->metrics();
  ASSERT_LT(after.sample_age_s,0.1);
  EXPECT_TRUE(after.valid);
  EXPECT_TRUE(after.stable);
  EXPECT_EQ(after.samples_count,before.samples_count);
}
TEST_F(ImuMonitorTest, MotionIsValidButNotStable) {
  stream(0.2);
  const auto metrics=monitor->metrics();
  EXPECT_TRUE(metrics.valid);EXPECT_FALSE(metrics.stable);
  EXPECT_NEAR(metrics.latest_ang_vel_rad_s,0.2,1e-9);
}
TEST_F(ImuMonitorTest, StaleAndResetWindowsLoseValidity) {
  stream(0);ASSERT_TRUE(monitor->metrics().valid);
  std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(monitor->metrics().valid);EXPECT_FALSE(monitor->metrics().stable);
  EXPECT_GE(monitor->metrics().sample_age_s,0.1);
  monitor->reset();EXPECT_FALSE(monitor->metrics().valid);
  EXPECT_EQ(monitor->metrics().samples_count,0u);
}
TEST_F(ImuMonitorTest, NonfiniteSamplesCannotEstablishValidity) {
  stream(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(monitor->metrics().valid);EXPECT_FALSE(monitor->metrics().stable);
  EXPECT_EQ(monitor->metrics().samples_count,0u);
}
