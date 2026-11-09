#include <gtest/gtest.h>
#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <thread>

using namespace std::chrono_literals;

TEST(RGBDAcquisition, RecoversDelayedTFWithoutAnotherCapture)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("rgbd_acquisition_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    tf2_ros::Buffer tf(node->get_clock());
    hb_perception::RGBDAcquisition acquisition(node.get(), &tf, "world", "rgb", "depth", "info");
    const auto qos = rclcpp::SensorDataQoS();
    using Image = sensor_msgs::msg::Image;
    auto rgb = node->create_publisher<Image>("rgb", qos);
    auto depth = node->create_publisher<Image>("depth", qos);
    auto info = node->create_publisher<sensor_msgs::msg::CameraInfo>("info", qos);
    const auto spin_until = [&](auto condition, auto timeout) {
        const auto end = std::chrono::steady_clock::now() + timeout;
        do {
            executor.spin_some();
            if (condition()) return true;
            std::this_thread::sleep_for(1ms);
        } while (std::chrono::steady_clock::now() < end);
        return false;
    };
    ASSERT_TRUE(spin_until([&] { return rgb->get_subscription_count() &&
        depth->get_subscription_count() && info->get_subscription_count(); }, 2s));
    info->publish(sensor_msgs::msg::CameraInfo{});
    ASSERT_TRUE(spin_until([&] { return info->get_subscription_count() == 0; }, 2s));

    const auto t = node->now();
    Image image;
    image.header.frame_id = "optical";
    image.header.stamp = t;
    depth->publish(image);
    image.header.stamp = t + rclcpp::Duration(2ms);
    rgb->publish(image);

    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "world";
    pose.child_frame_id = "optical";
    pose.transform.rotation.w = 1;
    pose.header.stamp = t;
    ASSERT_TRUE(tf.setTransform(pose, "test"));
    EXPECT_FALSE(spin_until([&] { return acquisition.latest().has_value(); }, 80ms));
    pose.header.stamp = t + rclcpp::Duration(4ms);
    pose.transform.translation.x = 2;
    ASSERT_TRUE(tf.setTransform(pose, "test"));
    ASSERT_TRUE(spin_until([&] { return acquisition.latest().has_value(); }, 250ms));
    const auto ob = acquisition.acquireAfter(t - rclcpp::Duration(1ms), 0ms);
    ASSERT_TRUE(ob);
    EXPECT_DOUBLE_EQ(ob->camera_pose.transform.translation.x, 1);
    EXPECT_EQ(ob->stamp, t + rclcpp::Duration(2ms));
    EXPECT_FALSE(acquisition.acquireAfter(t, 0ms));  // depth is exactly at the boundary

    // A missing counterpart must not pair with the next capture.
    pose.header.stamp = t + rclcpp::Duration(150ms);
    ASSERT_TRUE(tf.setTransform(pose, "test"));
    image.header.stamp = t + rclcpp::Duration(50ms);
    depth->publish(image);
    image.header.stamp = t + rclcpp::Duration(100ms);
    rgb->publish(image);
    EXPECT_FALSE(spin_until([&] { return acquisition.latest()->stamp > ob->stamp; }, 80ms));
    depth->publish(image);
    ASSERT_TRUE(spin_until([&] { return acquisition.latest()->stamp > ob->stamp; }, 250ms));
    EXPECT_EQ(acquisition.latest()->depth_to_rgb->header.stamp, image.header.stamp);
    rclcpp::shutdown();
}

TEST(RGBDAcquisition, CaptureWindowReleasesSubscriptionsAndRejectsAdjacentFrames)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("rgbd_window_test");
    tf2_ros::Buffer tf(node->get_clock());
    using Capture = hb_perception::RGBDAcquisition;
    const auto qos = rclcpp::SensorDataQoS();
    auto rgb = node->create_publisher<Capture::Image>("rgb", qos);
    auto depth = node->create_publisher<Capture::Image>("depth", qos);
    auto info = node->create_publisher<Capture::CameraInfo>("info", qos);
    const auto matched = [&](size_t count) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        do {
            if (rgb->get_subscription_count() == count && depth->get_subscription_count() == count &&
                info->get_subscription_count() == count) return true;
            std::this_thread::sleep_for(5ms);
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
    };
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "world";
    pose.child_frame_id = "optical";
    pose.transform.rotation.w = 1;
    ASSERT_TRUE(tf.setTransform(pose, "test", true));
    for (int view = 0; view < 2; ++view) {
        ASSERT_TRUE(matched(0));
        Capture capture(node.get(), &tf, "world", Capture::Mode::CaptureWindow, "rgb", "depth", "info");
        ASSERT_TRUE(matched(1));
        info->publish(Capture::CameraInfo{});
        const auto boundary = node->now();
        Capture::Image image;
        image.header.frame_id = "optical";
        image.header.stamp = boundary;
        rgb->publish(image);
        depth->publish(image);
        EXPECT_FALSE(capture.acquireAfter(boundary, 100ms));
        image.header.stamp = boundary + rclcpp::Duration(50ms);
        depth->publish(image);
        image.header.stamp = boundary + rclcpp::Duration(83ms);
        rgb->publish(image);
        EXPECT_FALSE(capture.acquireAfter(boundary, 100ms));
        image.header.stamp = boundary + rclcpp::Duration(85ms);
        depth->publish(image);
        auto ob = capture.acquireAfter(boundary, 1s);
        ASSERT_TRUE(ob);
        EXPECT_EQ(ob->depth_to_rgb->header.stamp, image.header.stamp);
    }
    EXPECT_TRUE(matched(0));
    rclcpp::shutdown();
}
