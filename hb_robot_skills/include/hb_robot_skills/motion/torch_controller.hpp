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
#include <functional>
#include <condition_variable>


using SteadyClock = std::chrono::steady_clock;
enum class TorchCommandResult{
    CONFIRMED,
    NOT_READY,
    SERVICE_UNAVAILABLE,
    REQUEST_FAILED,
    FEEDBACK_TIMEOUT,
    IO_LOST,
    CANCELED,
    FAULT

};

struct TorchIOConfig{

    std::string io_topic = "/io_and_status_controller/io_states";

    std::string set_io_service = "/io_and_status_controller/set_io";

    int torch_output_pin = 1;
    int transfer_input_pin = 0;
    int voltage_input_pin = 0;

    bool transfer_active_high = true;

    double transfer_timeout_s = 2.0;
    double output_timeout_s = 1.0;
    double transfer_loss_grace_s = 0.1;

    bool ignition_enabled = false;
    double voltage_gain = 1.0;
    double voltage_offset = 0.0;

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
    enum class State{
        IDLE,
        IGNITING,
        ESTABLISHED,
        CUTTING,
        SHUTTING_DOWN,
        FAULT
    };
    enum class WaitResult{
        CONFIRMED,
        TIMEOUT,
        IO_LOST,
        CANCELED,
        FAULT
    };

    using CancelCheck = std::function<bool()>;
    using FaultCallback = std::function<void (const std::string&)>;

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
            });

            monitor_timer_ = node_.create_wall_timer(std::chrono::milliseconds(20),
        [this](){
            monitor();
        });
        

        

        }

    bool isArcEstablished() const;
    bool hasFault() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return state_==State::FAULT;
    }
    
    std::string faultReason() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return fault_reason_;
    }

    TorchCommandResult requestIgnition(std::chrono::milliseconds timeout,
    const CancelCheck& canceled = {})
    {
        std::unique_lock<std::mutex>command_lock(command_mutex_);
        {
            std::lock_guard<std::mutex>lock(mutex_);
            if(!config_.ignition_enabled || 
            state_!=State::IDLE||
            !freshLocked() || 
            telemetry_.output_on!=std::optional<bool>(false)||
            telemetry_.transfer_detected!=std::optional<bool>(false)){
                return TorchCommandResult::NOT_READY;
            }
            state_ = State::IGNITING;
        }
            const auto result = setTorchEnabled(true, timeout, canceled);
            if(result!=TorchCommandResult::CONFIRMED){
                {
                    std::lock_guard<std::mutex>lock(mutex_);
                    latchFaultLocked("ignition not confirmed");
                }
            }
            return result;
        
    }
    TorchCommandResult requestShutdown(std::chrono::milliseconds timeout){
        
            std::unique_lock<std::mutex>command_lock(command_mutex_);
            {
                std::lock_guard<std::mutex>lock(mutex_);
                if(state_ != State::FAULT) state_= State::SHUTTING_DOWN;
            }
            
            const auto result = setTorchEnabled(false,timeout,{});
            if(result != TorchCommandResult::CONFIRMED){
                std::lock_guard<std::mutex>lock(mutex_);
                latchFaultLocked("shutdown not confirmed");
            }
            return result;
    }
    WaitResult waitForOutputOff(std::chrono::milliseconds timeout,
        CancelCheck canceled = {}){
            return waitFor(timeout,canceled,[](const TorchTelemetry& t){
            return t.output_on == std::optional<bool>(false);
        });

        }


    bool beginCut(const std::string& segment){
        std::lock_guard<std::mutex> lock(mutex_);
        if(!config_.ignition_enabled || 
            state_!=State::IDLE||
            !freshLocked() || 
            telemetry_.output_on!=std::optional<bool>(false)||
            telemetry_.transfer_detected!=std::optional<bool>(false)){
                return false;
            }
        active_segment_ = segment;
        state_ = State::IGNITING;
        return true;

    }

    bool finishCut(){
        std::lock_guard<std::mutex>lock(mutex_);
        if(state_ != State::SHUTTING_DOWN || !freshLocked() ||
        telemetry_.output_on != std::optional<bool>(false) ||
        telemetry_.transfer_detected != std::optional<bool>(false)
    ) { return false;}
    state_ = State::IDLE;
    active_segment_.clear();
    return true;
    }



    void setFaultCallback(std::function<void(const std::string&)> callback){
       std::lock_guard<std::mutex> lock(mutex_);
       fault_callback_ = std::move(callback);
       
    }
    

    

    WaitResult waitForArcTransfer(
        std::chrono::milliseconds timeout,
        CancelCheck canceled = {}
    ){
        const auto result = waitFor(timeout,canceled,[](const TorchTelemetry& t){
            return t.output_on.value_or(false)&&
            t.transfer_detected.value_or(false);
        });
        if(result == WaitResult::CONFIRMED){
            std::lock_guard<std::mutex>lock(mutex_);
            if(state_!=State::IGNITING ||
            !freshLocked()||
            telemetry_.output_on == std::optional<bool>(true)||
            telemetry_.transfer_detected != std::optional<bool>(true)){
                return WaitResult::FAULT;
            }

        state_ = State::ESTABLISHED;
        }
        return result;
    }

    bool markCutting(){
        std::lock_guard<std::mutex>lock(mutex_);
        if(state_!=State::ESTABLISHED ||
             !freshLocked() ||
             telemetry_.transfer_detected !=
             std::optional<bool>(true)){
                return false;
             }
        state_ = State::CUTTING;
        return true;

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
        return age < std::chrono::duration<double>(config_.output_timeout_s);
    }
    void setActiveSegment(const std::string& segment_name){
        std::lock_guard<std::mutex>lock(mutex_);
        active_segment_ = segment_name;
    }

    
    State state() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return state_;
    }
    
    bool clearFault(){
        std::lock_guard<std::mutex>lock(mutex_);
        if(state_ != State::FAULT || !freshLocked() ||
        telemetry_.output_on != std::optional<bool>(false) ||
        telemetry_.transfer_detected != std::optional<bool>(false)){
            return false;
        }
        fault_reason_.clear();
        fault_reported_ = false;
        transfer_lost_since_.reset();
        active_segment_.clear();
        state_ = State::IDLE;
        return true;


    }
      
    
    private:

    TorchCommandResult setTorchEnabled(bool enabled,
    std::chrono::milliseconds timeout,
    const CancelCheck& canceled
    ){
        using Clock = std::chrono::steady_clock;
        const auto deadline = Clock::now() + timeout;
        if(enabled && canceled && canceled()){
            return TorchCommandResult::CANCELED;
        }

        const auto remaining= [&](){
            const auto now = Clock::now();
            if(now>=deadline){
                return std::chrono::milliseconds(0);
            }
            return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        };
        if(!io_client_->wait_for_service(std::min(remaining(),std::chrono::milliseconds(200)))){
            RCLCPP_ERROR(node_.get_logger(),"ur SetIO is unavailable");
            return TorchCommandResult::SERVICE_UNAVAILABLE;
        }
             
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun =1;
        request->pin=config_.torch_output_pin;
        request->state=  enabled ? 1.0: 0.0;

        auto pending = io_client_->async_send_request(request);
        while(remaining().count()>0){
            if(enabled && canceled && canceled()){
                io_client_->remove_pending_request(pending);
                return TorchCommandResult::CANCELED;
            }
            if(pending.wait_for(std::chrono::milliseconds(20))== std::future_status::ready)
            {
                 try{
                    const auto response = pending.get();
                    if(!response || !response->success) return TorchCommandResult::REQUEST_FAILED;
                    
                }catch(const std::exception& e){
                    RCLCPP_ERROR(node_->get_logger(),"SetIO exception %s",e.what());
                    return TorchCommandResult::REQUEST_FAILED;
                }
                std::lock_guard<std::mutex>lock(mutex_);
                while(Clock::now() < deadline){
                    if(enabled && canceled && canceled()) return TorchCommandResult::CANCELED;
                    if(enabled && state_ == State::FAULT) return TorchCommandResult::FAULT; 
                    if(!freshLocked()) return TorchCommandResult::IO_LOST;
                    if(telemetry_.output_on == std::optional<bool>(enabled)){

                        telemetry_.command_on = enabled;
                        return TorchCommandResult::CONFIRMED;
                    }
                    cv_.wait_for(lock,std::chrono::milliseconds(20));

                }
                return TorchCommandResult::FEEDBACK_TIMEOUT;
            }
        }
        io_client_->remove_pending_request(pending);
        return TorchCommandResult::REQUEST_FAILED;
    }

       
    bool freshLocked()const{
        if(!last_io_time_){
            return false;
        }
        return std::chrono::steady_clock::now() - *last_io_time_ < std::chrono::duration<double>(config_.output_timeout_s);     
        
    }
    void latchFaultLocked(const std::string& reason){
        if(state_ == State::FAULT) return;

        state_ = State::FAULT;
        fault_reason_ = reason;
        cv_.notify_all();

    }

    
    template<typename Predicate> WaitResult waitFor(std::chrono::milliseconds timeout,
    const CancelCheck& canceled,
    Predicate predicate
){
    const auto deadline = SteadyClock::now() + timeout;
    std::unique_lock<std::mutex>lock(mutex_);
    while(SteadyClock::now() < deadline){
        if(canceled && canceled()){
            return WaitResult::CANCELED;
        }
        if(state_ == State::FAULT){
            return WaitResult::FAULT;
        }
        if(!freshLocked()){
            return WaitResult::IO_LOST;
        }
        if(predicate(telemetry_)){
            
            return WaitResult::CONFIRMED;
        }
        cv_.wait_until(
            lock, std::min(deadline, SteadyClock::now() + std::chrono::milliseconds(20))
        );
        
    }
    return WaitResult::TIMEOUT;

}

    void onIOStates(ur_msgs::msg::IOStates msg){
        TorchTelemetry next;
        
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
            next.command_on = telemetry_.command_on;
            telemetry_ = std::move(next);
            if(state_==State::CUTTING && telemetry_.transfer_detected == std::optional<bool>(true)){
                transfer_lost_since_.reset();
            }
            
        
            last_io_time_ = std::chrono::steady_clock::now();

        }
        cv_.notify_all();

    }

    void publishStatus(){

        const auto data = snapshot();

        hb_robot_interfaces::msg::TorchStatus msg;
        msg.stamp = data.stamp;
        {
        std::lock_guard<std::mutex>lock(mutex_);
        msg.segment_name = active_segment_;
        }
        
        
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
        msg.fault =hasFault();
        msg.fault_reason = faultReason();
        status_pub_->publish(msg);
    }

    void monitor(){
        FaultCallback callback;
        std::string reason;
        {
            std::lock_guard<std::mutex>lock(mutex_);
            const bool active = 
                state_ == State::IGNITING ||
                state_ == State::ESTABLISHED ||
                state_ == State::CUTTING ||
                state_ == State::SHUTTING_DOWN;
            if(active && !freshLocked()){
                latchFaultLocked("Torch IO lost feedback");
                        }
            if(state_==State::CUTTING){
                const bool transferred = telemetry_.transfer_detected == std::optional<bool>(true);
                if(!transferred){
                    const auto now = std::chrono::steady_clock::now();
                    if(!transfer_lost_since_){
                        transfer_lost_since_ = now;
                    }
                    if(now- *transfer_lost_since_ >= std::chrono::duration<double>(config_.transfer_loss_grace_s)){
                        latchFaultLocked("arc lost during cut");
                    }
                }
                else{
                    transfer_lost_since_.reset();
                }
            }
            if(state_==State::FAULT && !fault_reported_){
                fault_reported_ = true;
                callback = fault_callback_;
                reason = fault_reason_;
            }
            
        }
        if(callback){
                callback(reason);
            }

    }


    std::optional<std::chrono::steady_clock::time_point> last_io_time_;
    std::optional<std::chrono::steady_clock::time_point> transfer_lost_since_;
    mutable std::mutex mutex_;
    std::mutex command_mutex_;

    State state_{State::IDLE};
    std::string fault_reason_;
    std::string active_segment_;
    bool fault_reported_{false};

    std::condition_variable cv_;
    FaultCallback fault_callback_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_sub_;
    rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
    rclcpp::Node& node_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    
    TorchIOConfig config_;
    TorchTelemetry telemetry_;
};

