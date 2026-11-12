
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "azure_kinect_ros2_driver_codex/msg/rgbd_frame.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    using RGBDFrame =
        azure_kinect_ros2_driver_codex::msg::RGBDFrame;

    MinimalSubscriber()
        : Node("rgbd_debug")
    {
        rgbd_frame_sub_ = create_subscription<RGBDFrame>(
            "/k4a/rgbd",
            rclcpp::SensorDataQoS().keep_last(1),
            [this](RGBDFrame::ConstSharedPtr msg)
            {
                (void)msg;

                received_.fetch_add(
                    1, std::memory_order_relaxed);
            });

        timer_ = create_wall_timer(
            1s,
            [this]()
            {
                const auto count = received_.exchange(
                    0, std::memory_order_relaxed);

                RCLCPP_INFO(
                    get_logger(),
                    "RGBD received: %llu frames/s",
                    static_cast<unsigned long long>(count));
            });

        RCLCPP_INFO(
            get_logger(),
            "Minimal RGBD subscriber started");
    }

private:
    std::atomic<uint64_t> received_{0};

    rclcpp::Subscription<RGBDFrame>::SharedPtr
        rgbd_frame_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MinimalSubscriber>());

    rclcpp::shutdown();

    return 0;
}
