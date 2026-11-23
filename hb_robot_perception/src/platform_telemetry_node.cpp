
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace hb_perception {

class TiltFilter {
public:
    using Imu = sensor_msgs::msg::Imu;
    using Clock = std::chrono::steady_clock;

    struct Output {
        Imu imu;
        geometry_msgs::msg::Vector3Stamped rpy;
    };

    TiltFilter(
        double gyro_cutoff_hz,
        double accel_cutoff_hz,
        double correction_time_s,
        double gravity_gate_m_s2,
        std::array<double, 3> gyro_bias)
        : gyro_cutoff_(gyro_cutoff_hz),
          accel_cutoff_(accel_cutoff_hz),
          correction_time_(correction_time_s),
          gravity_gate_(gravity_gate_m_s2),
          bias_(gyro_bias)
    {}

    std::optional<Output> update(
        const Imu& raw,
        Clock::time_point received)
    {
        if (raw.angular_velocity_covariance[0] == -1.0 ||
            raw.linear_acceleration_covariance[0] == -1.0)
            return std::nullopt;

        const Eigen::Vector3d w_raw(
            raw.angular_velocity.x,
            raw.angular_velocity.y,
            raw.angular_velocity.z);

        const Eigen::Vector3d a_raw(
            raw.linear_acceleration.x,
            raw.linear_acceleration.y,
            raw.linear_acceleration.z);

        if (!w_raw.allFinite() || !a_raw.allFinite())
            return std::nullopt;

        const Eigen::Vector3d w =
            w_raw - Eigen::Vector3d(
                bias_[0], bias_[1], bias_[2]);

        const int64_t stamp_ns =
            static_cast<int64_t>(raw.header.stamp.sec) *
                1000000000LL +
            static_cast<int64_t>(
                raw.header.stamp.nanosec);

        double dt = 0.0;

        if (ready_) {
            if (stamp_ns > 0 && last_stamp_ns_ > 0) {
                dt = static_cast<double>(
                    stamp_ns - last_stamp_ns_) * 1e-9;
            } else {
                dt = std::chrono::duration<double>(
                    received - last_received_).count();
            }

            if (!std::isfinite(dt) ||
                dt <= 0.0 || dt > max_sample_gap_s_)
            {
                ready_ = false;
                last_stamp_ns_ = 0;
                return std::nullopt;
            }
        }

        if (!ready_) {
            // Initialize only when acceleration is
            // reasonably consistent with gravity.
            if (std::abs(a_raw.norm() - 9.80665) >
                gravity_gate_)
                return std::nullopt;

            w_filtered_ = w;
            a_filtered_ = a_raw;

            // Initial roll/pitch from gravity.
            // Absolute yaw remains unobservable.
            q_ = Eigen::Quaterniond::FromTwoVectors(
                a_raw.normalized(),
                Eigen::Vector3d::UnitZ());

            q_.normalize();
            ready_ = true;
        } else {
            const double aw = -std::expm1(
                -2.0 * M_PI * gyro_cutoff_ * dt);

            const double aa = -std::expm1(
                -2.0 * M_PI * accel_cutoff_ * dt);

            w_filtered_ +=
                aw * (w - w_filtered_);

            a_filtered_ +=
                aa * (a_raw - a_filtered_);

            Eigen::Vector3d omega = w_filtered_;

            // Accelerometer correction of roll/pitch.
            // Suppress correction during strong acceleration.
            const double accel_norm =
                a_filtered_.norm();

            if (accel_norm > 1e-6 &&
                std::abs(accel_norm - 9.80665) <=
                    gravity_gate_)
            {
                const Eigen::Vector3d predicted =
                    q_.conjugate() *
                    Eigen::Vector3d::UnitZ();

                const Eigen::Vector3d measured =
                    a_filtered_ / accel_norm;

                omega +=
                    (1.0 / correction_time_) *
                    measured.cross(predicted);
            }

            const double angle = omega.norm() * dt;

            if (angle > 1e-12) {
                q_ = q_ * Eigen::Quaterniond(
                    Eigen::AngleAxisd(
                        angle,
                        omega.normalized()));

                q_.normalize();
            }
        }

        last_stamp_ns_ = stamp_ns;
        last_received_ = received;

        Output out;
        out.imu = raw;

        out.imu.angular_velocity.x = w_filtered_.x();
        out.imu.angular_velocity.y = w_filtered_.y();
        out.imu.angular_velocity.z = w_filtered_.z();

        out.imu.linear_acceleration.x = a_filtered_.x();
        out.imu.linear_acceleration.y = a_filtered_.y();
        out.imu.linear_acceleration.z = a_filtered_.z();

        out.imu.orientation.x = q_.x();
        out.imu.orientation.y = q_.y();
        out.imu.orientation.z = q_.z();
        out.imu.orientation.w = q_.w();

        // Provisional diagnostic covariance only.
        out.imu.orientation_covariance.fill(0.0);
        out.imu.orientation_covariance[0] = 0.0025;
        out.imu.orientation_covariance[4] = 0.0025;
        out.imu.orientation_covariance[8] = 1e6;

        // Retain source gyro/acceleration covariances.
        // They must be calibrated for filtered output
        // before use in sensor fusion.

        const Eigen::Matrix3d R =
            q_.toRotationMatrix();

        out.rpy.header = raw.header;

        out.rpy.vector.x =
            std::atan2(R(2, 1), R(2, 2));

        out.rpy.vector.y =
            std::atan2(
                -R(2, 0),
                std::hypot(R(2, 1), R(2, 2)));

        out.rpy.vector.z =
            std::atan2(R(1, 0), R(0, 0));

        return out;
    }

private:
    double gyro_cutoff_;
    double accel_cutoff_;
    double correction_time_;
    double gravity_gate_;

    std::array<double, 3> bias_;
    const double max_sample_gap_s_ = 0.25;

    bool ready_ = false;
    int64_t last_stamp_ns_ = 0;
    Clock::time_point last_received_{};

    Eigen::Vector3d w_filtered_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d a_filtered_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond q_ =
        Eigen::Quaterniond::Identity();
};


class PlatformTelemetryNode final : public rclcpp::Node {
    using Imu = sensor_msgs::msg::Imu;
    using Rpy = geometry_msgs::msg::Vector3Stamped;
    using Clock = std::chrono::steady_clock;

public:
    PlatformTelemetryNode()
        : Node("platform_telemetry")
    {
        const auto primary_topic =
            declare_parameter<std::string>(
                "imu_topic",
                "/mpu6050/imu");

        const auto secondary_topic =
            declare_parameter<std::string>(
                "mpu6050_imu_topic", "/mpu6050_2/imu");

        const double gyro_cutoff =
            declare_parameter<double>(
                "attitude.gyro_cutoff_hz", 20.0);

        const double accel_cutoff =
            declare_parameter<double>(
                "attitude.accel_cutoff_hz", 5.0);

        const double correction_time =
            declare_parameter<double>(
                "attitude.correction_time_s", 1.5);

        const double gravity_gate =
            declare_parameter<double>(
                "attitude.gravity_gate_m_s2", 0.5);

        const auto primary_bias =
            declare_parameter<std::vector<double>>(
                "attitude.mpu6050_2_gyro_bias",
                {0.0, 0.0, 0.0});

        const auto secondary_bias =
            declare_parameter<std::vector<double>>(
                "attitude.mpu6050_gyro_bias",
                {0.0, 0.0, 0.0});

        if (!std::isfinite(gyro_cutoff) ||
            !std::isfinite(accel_cutoff) ||
            !std::isfinite(correction_time) ||
            !std::isfinite(gravity_gate) ||
            gyro_cutoff <= 0.0 ||
            accel_cutoff <= 0.0 ||
            correction_time <= 0.0 ||
            gravity_gate <= 0.0 ||
            primary_bias.size() != 3 ||
            secondary_bias.size() != 3 ||
            primary_topic.empty() ||
            (!secondary_topic.empty() &&
             secondary_topic == primary_topic))
        {
            throw std::invalid_argument(
                "Invalid platform telemetry parameters");
        }

        for (double v : primary_bias)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "Invalid mpu6050_2 gyro bias");

        for (double v : secondary_bias)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "Invalid MPU6050 gyro bias");

        primary_filter_ = std::make_unique<TiltFilter>(
            gyro_cutoff,
            accel_cutoff,
            correction_time,
            gravity_gate,
            std::array<double, 3>{
                primary_bias[0],
                primary_bias[1],
                primary_bias[2]});

        secondary_filter_ = std::make_unique<TiltFilter>(
            gyro_cutoff,
            accel_cutoff,
            correction_time,
            gravity_gate,
            std::array<double, 3>{
                secondary_bias[0],
                secondary_bias[1],
                secondary_bias[2]});

        const auto qos = rclcpp::SensorDataQoS();

        mpu6050_2_pub_ = create_publisher<Imu>(
            "/platform/imu/mpu6050_2/filtered", qos);

        mpu6050_pub_ = create_publisher<Imu>(
            "/platform/imu/mpu6050/filtered", qos);

        mpu6050_2_rpy_pub_ = create_publisher<Rpy>(
            "/platform/imu/mpu6050_2/rpy", qos);

        mpu6050_rpy_pub_ = create_publisher<Rpy>(
            "/platform/imu/mpu6050/rpy", qos);

        mpu6050_2_sub_ = create_subscription<Imu>(
            primary_topic,
            qos,
            [this](Imu::ConstSharedPtr msg) {
                const auto result =
                    primary_filter_->update(
                        *msg, Clock::now());

                if (!result)
                    return;

                mpu6050_2_pub_->publish(result->imu);
                mpu6050_2_rpy_pub_->publish(result->rpy);
            });

        if (!secondary_topic.empty()) {
            mpu6050_sub_ = create_subscription<Imu>(
                secondary_topic,
                qos,
                [this](Imu::ConstSharedPtr msg) {
                    const auto result =
                        secondary_filter_->update(
                            *msg, Clock::now());

                    if (!result)
                        return;

                    mpu6050_pub_->publish(result->imu);
                    mpu6050_rpy_pub_->publish(result->rpy);
                });
        }

        RCLCPP_INFO(
            get_logger(),
            "mpu6050_2 IMU: %s",
            primary_topic.c_str());

        RCLCPP_INFO(
            get_logger(),
            "MPU6050 IMU: %s",
            secondary_topic.empty()
                ? "(disabled)"
                : secondary_topic.c_str());
    }

private:
    std::unique_ptr<TiltFilter>
        primary_filter_, secondary_filter_;

    rclcpp::Subscription<Imu>::SharedPtr
        mpu6050_2_sub_, mpu6050_sub_;

    rclcpp::Publisher<Imu>::SharedPtr
        mpu6050_2_pub_, mpu6050_pub_;

    rclcpp::Publisher<Rpy>::SharedPtr
        mpu6050_2_rpy_pub_, mpu6050_rpy_pub_;
};

}  // namespace hb_perception


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(
            std::make_shared<
                hb_perception::PlatformTelemetryNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("platform_telemetry"),
            "%s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
