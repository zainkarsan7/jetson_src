#include "hb_robot_perception/imu_stability_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "hb_robot_interfaces/msg/platform_imu_metrics.hpp"
#include "std_msgs/msg/u_int8.hpp"


#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <mutex>
#include <string>
#include <vector>


namespace hb_perception{

class PlatformTelemetryNode final : public rclcpp::Node{
    using Imu = sensor_msgs::msg::Imu;
    using Metrics = hb_robot_interfaces::msg::PlatformImuMetrics;
    using Clock = std::chrono::steady_clock;
    struct Sample{
        Clock::time_point receipt;
        rclcpp::Time stamp;
        std::array<double,3> w,a;
    };

    public:
        PlatformTelemetryNode(): Node("platform_telemetry_node"){

            const auto topic = declare_parameter<std::string>("imu_topic","/hb_robot_lidar/imu");
            window_s_ = declare_parameter<double>("window_s", 0.5);
            max_age_s_ = declare_parameter<double>("max_age_s_", 0.2);
            min_coverage_ = declare_parameter<double>("min_window_coverage", 0.9);
            cutoff_hz_ = declare_parameter<double>("filter_cutoff_hz", 12.0);
            max_gyro_rms_ = declare_parameter<double>("max_gyro_rms", 0.02);
            max_accel_rms_ = declare_parameter<double>("max_accel_rms", 0.2);
            max_mean_gyro_ = declare_parameter<double>("max_mean_gyro", 0.04);
            stable_dwell_s_ = declare_parameter<double>("stable_dwell_s", 0.5);
            min_samples_ = declare_parameter<int>("min_samples_", 20);

            const auto rate = declare_parameter<double>("metrics_rate_hz",10);
            if(window_s_ <= 0 || 
                max_age_s_ <= 0 || 
                min_samples_ <= 2 || 
                min_coverage_ <= 0 ||
                min_coverage_ > 1 ||
                cutoff_hz_ <= 0 ||
                rate <= 0 ||
                stable_dwell_s_<0
             ){
                throw std::invalid_argument("invalid parameters");
             }

             metrics_pub_ = create_publisher<Metrics>("/platform/imu/metrics",10);
             state_pub_ = create_publisher<std_msgs::msg::UInt8>("/platform/imu/motion_state",10);
             filtered_imu_pub_ = create_publisher<Imu>("/platform/imu/filtered",rclcpp::SensorDataQoS());
             imu_sub_ = create_subscription<Imu>(topic,rclcpp::SensorDataQoS(),
             [this](Imu::ConstSharedPtr msg){onImu(*msg);});
             timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate),
             [this](){publishMetrics();});
             RCLCPP_INFO(get_logger(),"Telemetry listening on %s", topic.c_str());
            }

    private:
        Imu onImu(Imu::ConstSharedPtr msg){

        }
        void publishMetrics();


        double window_s_, max_age_s_, min_coverage_, cutoff_hz_, max_gyro_rms_, max_accel_rms_,max_mean_gyro_,stable_dwell_s_;
        int min_samples_;
        std::mutex mutex_;
        std::deque<Sample> samples_;
        std::array<double,3> filtered_w_{},filtered_a_{};
        bool filter_ready_ = false;
        Clock::time_point last_receipt_{};
        std::optional<Clock::time_point> candidate_since_;
        rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<Metrics>::SharedPtr metrics_pub_;
        rclcpp::Publisher<Imu>::SharedPtr filtered_imu_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char** argv){
    return 0;
}