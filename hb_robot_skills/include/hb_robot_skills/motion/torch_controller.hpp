#pragma once

#include <rclcpp/rclcpp.hpp>
#include "hb_robot_interfaces/msg/torch_status.hpp"
#include <ur_msgs/msg/io_states.hpp>

#include <mutex>
#include <optional>
#include <cmath>


struct TorchIOConfig{

    std::string io_topic = "/io_and_status_controller/io_states";

    std::string set_io_service = "/io_and_status_controller/set_io";

    int torch_output_pin = 1;
    int transfer_input_pin = 0;
    int voltage_input_pin = 0;

    bool transfer_active_high = true;

    double voltage_gain = 1.0;
    double voltage_offset = 0.0;
    double io_timeout_s = 1.0;

};

struct TorchTelemetry{
    
    rclcpp::Time stamp;
    bool command_on = false;
    
    std::optional<bool> output_on;
    std::optional<bool> transfer_detected;

    std::optional<double> analog_value;
    std::optional<int> analog_domain;

    std::optional<double> arc_voltage_v;
    
};

class TorchController{
    public:
        TorchController(rclcpp::Node& node,
        TorchIOConfig config):
        node_(node),
        config_(std::move(config))
        {


            io_sub_ = node_.create_subscription<ur_msgs::msg::IOStates>(
                config_.io_topic,
                rclcpp::QoS(10).reliable(),
                [this](ur_msgs::msg::IOStates::ConstSharedPtr msg){
                    onIOStates(*msg);
                }
            );
        

        }

    bool setTorchEnabled(bool enabled);
    TorchTelemetry snapshot() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return telemetry_;
    }
    bool hasFreshIO() const;
    bool waitForArcTransfer(std::chrono::milliseconds timeout);
    void setActiveSegment(const std::string segment_name);
    

    
    
    private:

    void onIOStates(ur_msgs::msg::IOStates msg){
        TorchTelemetry next;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            next.command_on = telemetry_.command_on;
        }
        next.stamp = node_.now();
        for(const auto& digital: msg.digital_out_states){
            if(digital.pin == config_.transfer_input_pin){
                next.transfer_detected = (digital.state == config_.transfer_active_high);
            }
        }
        for(const auto& analog: msg.analog_in_states){
            if(analog.pin == config_.voltage_input_pin){
                next.analog_value = analog.state;
                next.analog_domain = analog.domain;

            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_ = std::move(next);

        }


    }

    mutable std::mutex mutex_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_sub_;
    rclcpp::Node node_;
    TorchIOConfig config_;
    TorchTelemetry telemetry_;


}

