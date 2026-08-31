#pragma once

#include <memory>
#include <string>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "crane_controllers/crane_attitude_estimator.hpp"

namespace crane_controllers{
    class CraneAttitudeController: public controller_interface::ControllerInterface
    {
        public:
            controller_interface::CallbackReturn on_init() override;
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
            controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &);
        private:
        
            CraneAttitudeEstimator estimator_;
            realtime_tools::RealtimeBuffer<CraneAttitudeState> state_buffer_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
            std::string marker_pose_topic_;
            std::string imu_topic_;

    };

}