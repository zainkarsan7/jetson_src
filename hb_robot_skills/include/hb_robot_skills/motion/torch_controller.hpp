#pragma once

#include <rclcpp/rclcpp.hpp>
#include "hb_robot_interfaces/msg/torch_status.hpp"
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <mutex>
#include <optional>
#include <cmath>
#include <future>
#include <cstdint>
#include <string>
#include <utility>
#include <chrono>



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
    
    rclcpp::Time stamp{0,0,RCL_ROS_TIME};
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

            status_pub_ = node_.create_publisher<hb_robot_interfaces::msg::TorchStatus>(
                "/cut_profile/torch_status",
                rclcpp::QoS(10)
            );

            io_client_ = node_.create_client<ur_msgs::srv::SetIO>(
                config_.set_io_service
            );

            status_timer_ = node_.create_wall_timer(std::chrono::milliseconds(100),
            [this](){
                publishStatus();
            }
        );

        

        }


    TorchTelemetry snapshot() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return telemetry_;
    }
    bool hasFreshIO() const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(!last_io_time_){
            return false;
        }

        const auto age= std::chrono::steady_clock::now() - *last_io_time_;
        return age < std::chrono::duration<double>(config_.io_timeout_s);
    }
    void setActiveSegment(const std::string& segment_name){
        std::lock_guard<std::mutex>lock(mutex_);
        active_segment_ = segment_name;
    }

    bool setTorchEnabled(bool enabled){
        using namespace std::chrono_literals;
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        if(!hasFreshIO()){
            return false;
        }
        if(!io_client_->wait_for_service(1s)){
            RCLCPP_ERROR(node_.get_logger(),"ur SetIO is unavailable");
            return false;
        }
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun =1;
        request->pin=config_.torch_output_pin;
        request->state=  enabled ? 1.0: 0.0;
        auto future = io_client_->async_send_request(request);
        if(future.wait_for(1s) != std::future_status::ready){
            RCLCPP_ERROR(node_.get_logger(),"UR set IO timed out");
            return false;
        }

        const auto response = future.get();
        if(!response || !response->success){
            RCLCPP_ERROR(node_.get_logger(),"couldnt turn torch on");
            return false;
        }
        {
            //inform the telemetry_
            std::lock_guard<std::mutex> lcok(mutex_);
            telemetry_.command_on = enabled;
        }
        return true;


    }
      
    
    private:

    
    void onIOStates(ur_msgs::msg::IOStates msg){
        TorchTelemetry next;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            next.command_on = telemetry_.command_on;
        }
        
        next.stamp = node_.now();
        
        for(const auto& digital: msg.digital_out_states){
            if(digital.pin == config_.torch_output_pin){
                next.output_on = digital.state;
            }
        }
        
        for(const auto& digital: msg.digital_in_states){
            if(digital.pin == config_.transfer_input_pin){
                next.transfer_detected = (digital.state == config_.transfer_active_high);
            }
        }
        for(const auto& analog: msg.analog_in_states){
            if(analog.pin == config_.voltage_input_pin){
                next.analog_value = analog.state;
                next.analog_domain = static_cast<int>(analog.domain);

            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_ = std::move(next);
            last_io_time_ = std::chrono::steady_clock::now();

        }


    }

    void publishStatus(){

        const auto data = snapshot();

        hb_robot_interfaces::msg::TorchStatus msg;
        msg.stamp = data.stamp;
        msg.segment_name = active_segment_;
        msg.io_connected = hasFreshIO();
        msg.command_on = data.command_on;
        msg.output_on = data.output_on.value_or(false);
        msg.transfer_detected = data.transfer_detected.value_or(false);
        msg.analog_valid = data.analog_value.has_value();
        msg.analog_input_value = data.analog_value.value_or(0.0);
        msg.analog_domain = static_cast<uint8_t>(data.analog_domain.value_or(0));        
        msg.arc_voltage_valid = data.arc_voltage_v.has_value();
        msg.arc_voltage_v = data.arc_voltage_v.value_or(0.0);
        msg.ready = false;
        msg.fault =false;
        msg.fault_reason = "";
        status_pub_->publish(msg);
    }
    std::string active_segment_;
    std::optional<std::chrono::steady_clock::time_point> last_io_time_;
    mutable std::mutex mutex_;
    std::mutex command_mutex_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_sub_;
    rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
    rclcpp::Node& node_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    TorchIOConfig config_;
    TorchTelemetry telemetry_;
};

