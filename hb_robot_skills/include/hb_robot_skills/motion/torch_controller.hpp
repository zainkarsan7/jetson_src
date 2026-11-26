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
#include <sstream>
#include <iomanip>
#include <algorithm>


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


            // Keep raw I/O and its status timer ordered, away from the node's
            // default action/supervision callbacks. Service replies use their
            // existing separate group.
            io_feedback_callback_group_ = node_.create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions io_options;
            io_options.callback_group = io_feedback_callback_group_;
            io_sub_ = node_.create_subscription<ur_msgs::msg::IOStates>(
                config_.io_topic,
                rclcpp::QoS(10).reliable(),
                [this](ur_msgs::msg::IOStates::ConstSharedPtr msg){
                    onIOStates(*msg);
                }, io_options
            );

            status_pub_ = node_.create_publisher<hb_robot_interfaces::msg::TorchStatus>(
                "/cut_profile/torch_status",
                rclcpp::QoS(10)
            );
             // A service response must not queue behind the node's default
            // mutually-exclusive I/O, action and supervision callbacks.
            io_reply_callback_group_ = node_.create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            io_client_ = node_.create_client<ur_msgs::srv::SetIO>(
                config_.set_io_service, rmw_qos_profile_services_default,
                io_reply_callback_group_
            );
            status_callback_group_ = io_feedback_callback_group_;

            status_timer_ = node_.create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    publishStatus();
                },
                status_callback_group_);
            // status_timer_ = node_.create_wall_timer(std::chrono::milliseconds(100),
            // [this](){
            //     publishStatus();
            // });

            monitor_timer_ = node_.create_wall_timer(std::chrono::milliseconds(20),
        [this](){
            monitor();
        });
        

        

        }

    // Nonblocking containment path, callable from supervision callbacks. This is
    // a process stop request, not a hardware emergency-stop guarantee. Feedback
    // must still confirm output OFF before a recovery motion is permitted.
    void emergencyShutdown(const std::string& reason) {
        { 
            std::lock_guard<std::mutex> lock(mutex_); 
                telemetry_.command_on = false;

            latchFaultLocked(reason); 
        }
        if (!io_client_->service_is_ready()) return;
        auto request=std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun=1; 
        request->pin=config_.torch_output_pin; 
        request->state=0.0;
        io_client_->async_send_request(request, [](rclcpp::Client<ur_msgs::srv::SetIO>::SharedFuture){});
    }

    bool isArcEstablished() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return state_==State::ESTABLISHED || state_==State::CUTTING;
    }
    bool hasFault() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return state_==State::FAULT;
    }
    
    std::string faultReason() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return fault_reason_;
    }

    static const char* resultName(TorchCommandResult result) {
        switch(result) {
        case TorchCommandResult::CONFIRMED: return "CONFIRMED";
        case TorchCommandResult::NOT_READY: return "NOT_READY";
        case TorchCommandResult::SERVICE_UNAVAILABLE: return "SERVICE_UNAVAILABLE";
        case TorchCommandResult::REQUEST_FAILED: return "REQUEST_FAILED";
        case TorchCommandResult::FEEDBACK_TIMEOUT: return "FEEDBACK_TIMEOUT";
        case TorchCommandResult::IO_LOST: return "IO_LOST";
        case TorchCommandResult::CANCELED: return "CANCELED";
        case TorchCommandResult::FAULT: return "FAULT";
        }
        return "UNKNOWN";
    }

    std::string diagnostic() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return diagnosticLocked();
    }

    TorchCommandResult requestIgnition(std::chrono::milliseconds timeout,
                                      const CancelCheck& canceled = {}) {
        std::unique_lock<std::mutex> command_lock(command_mutex_);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // A previous containment OFF may have completed before transfer
            // fell low. Finalize that clean shutdown, never a FAULT or active cut.
            if(state_ == State::SHUTTING_DOWN) (void)finishCutLocked(false);
            if(!config_.ignition_enabled || state_ != State::IDLE ||
               !freshLocked() || telemetry_.command_on ||
               telemetry_.output_on != std::optional<bool>(false) ||
               telemetry_.transfer_detected != std::optional<bool>(false)) {
                RCLCPP_ERROR(node_.get_logger(),
                    "[TORCH IGNITION REJECTED] result=NOT_READY %s",
                    diagnosticLocked().c_str());
                return TorchCommandResult::NOT_READY;
            }
            RCLCPP_INFO(node_.get_logger(),"[TORCH IGNITION REQUEST] %s",
                        diagnosticLocked().c_str());
            state_ = State::IGNITING;
            telemetry_.command_on = true;
        }
        const auto result = setTorchEnabled(true, timeout, canceled);
        if(result != TorchCommandResult::CONFIRMED) {
            std::lock_guard<std::mutex> lock(mutex_);
            latchFaultLocked(std::string("ignition not confirmed: ") + resultName(result));
        }
        return result;
    }

    TorchCommandResult requestShutdown(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> command_lock(command_mutex_);
        bool already_off = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Avoid an unnecessary SetIO round trip for an idle, confirmed OFF
            // torch (e.g. inspection planning failed before any motion).
            // A timed-out ON request may still execute at the server: in that
            // case always send OFF, including after operator fault clearance.
            already_off = !on_request_uncertain_ && state_ == State::IDLE &&
                !telemetry_.command_on && freshLocked() &&
                telemetry_.output_on == std::optional<bool>(false) &&
                telemetry_.transfer_detected == std::optional<bool>(false);
            telemetry_.command_on = false;
            if(!already_off && state_ != State::FAULT) state_ = State::SHUTTING_DOWN;
        }
        if (already_off) {
            RCLCPP_INFO(node_.get_logger(),
                "[TORCH IO RESULT] on=0 result=CONFIRMED stage=already_idle_off %s",
                diagnostic().c_str());
            return TorchCommandResult::CONFIRMED;
        }
        const auto result = setTorchEnabled(false, timeout, {});
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(result != TorchCommandResult::CONFIRMED)
                latchFaultLocked(std::string("shutdown not confirmed: ") + resultName(result));
            else
                (void)finishCutLocked(); // Fault-free, fresh OFF and transfer-low only.
        }
        return result;
    }
    WaitResult waitForOutputOff(std::chrono::milliseconds timeout,
        CancelCheck canceled = {}){
            return waitFor(timeout,canceled,[](const TorchTelemetry& t){
            return t.output_on == std::optional<bool>(false);
        });

        }


    // bool beginCut(const std::string& segment){
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     if(!config_.ignition_enabled || 
    //         state_!=State::IDLE||
    //         !freshLocked() || 
    //         telemetry_.output_on!=std::optional<bool>(false)||
    //         telemetry_.transfer_detected!=std::optional<bool>(false)){
    //             return false;
    //         }
    //     active_segment_ = segment;
    //     state_ = State::IGNITING;
    //     return true;

    // }

    bool finishCut() {
        std::lock_guard<std::mutex> lock(mutex_);
        return finishCutLocked();
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
            telemetry_.output_on != std::optional<bool>(true)||
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
        // clearFault already requires fresh, measured OFF and transfer-low.
        telemetry_.command_on = false;
        fault_reported_ = false;
        transfer_lost_since_.reset();
        active_segment_.clear();
        state_ = State::IDLE;
        return true;


    }


    void requestOutputOff()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_.command_on = false;
        }
        if (!io_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<ur_msgs::srv::SetIO::Request>();

        request->fun = 1;
        request->pin = config_.torch_output_pin;
        request->state = 0.0;

        io_client_->async_send_request(
            request,
            [](rclcpp::Client<
                ur_msgs::srv::SetIO>::SharedFuture) {});
    }

      
    
    private:

    static const char* stateName(State state) {
        switch(state) {
        case State::IDLE: return "IDLE";
        case State::IGNITING: return "IGNITING";
        case State::ESTABLISHED: return "ESTABLISHED";
        case State::CUTTING: return "CUTTING";
        case State::SHUTTING_DOWN: return "SHUTTING_DOWN";
        case State::FAULT: return "FAULT";
        }
        return "UNKNOWN";
    }

    std::string diagnosticLocked() const {
        const auto bit=[](const std::optional<bool>& value) {
            return value ? int(*value) : -1;
        };
        const double age=last_io_time_ ? std::chrono::duration<double,std::milli>(
            SteadyClock::now()-*last_io_time_).count() : -1.0;
        std::ostringstream text;
        text<<std::fixed<<std::setprecision(1)
            <<"state="<<stateName(state_)<<" enabled="<<config_.ignition_enabled
            <<" command_on="<<telemetry_.command_on<<" output="<<bit(telemetry_.output_on)
            <<" transfer="<<bit(telemetry_.transfer_detected)<<" io_fresh="<<freshLocked()
            <<" callback_age_ms="<<age<<" io_limit_ms="<<config_.output_timeout_s*1000.0
            <<" rx_count="<<io_rx_count_<<" max_callback_gap_ms="<<max_io_gap_ms_
            <<" output_pin="<<config_.torch_output_pin<<" transfer_pin="<<config_.transfer_input_pin
            <<" segment='"<<active_segment_<<"' fault='"<<fault_reason_<<"'";
        return text.str();
    }

    bool finishCutLocked(bool clear_segment = true) {
        if((state_ != State::SHUTTING_DOWN && state_ != State::IDLE) ||
           telemetry_.command_on || !freshLocked() ||
           telemetry_.output_on != std::optional<bool>(false) ||
           telemetry_.transfer_detected != std::optional<bool>(false)) return false;
        if(state_ == State::SHUTTING_DOWN)
            RCLCPP_INFO(node_.get_logger(),
                "[TORCH STATE] SHUTTING_DOWN -> IDLE: fresh OFF and transfer-low confirmed");
        state_ = State::IDLE;
        transfer_lost_since_.reset();
        if(clear_segment) active_segment_.clear();
        return true;
    }

    TorchCommandResult setTorchEnabled(bool enabled, std::chrono::milliseconds timeout,
                                      const CancelCheck& canceled) {
        const auto started = SteadyClock::now();
        const auto deadline = started + timeout;
        int64_t request_id = -1;
        const auto done = [&](TorchCommandResult result, const char* stage) {
            const double elapsed = std::chrono::duration<double,std::milli>(
                SteadyClock::now()-started).count();
            const auto detail=diagnostic();
            if(result == TorchCommandResult::CONFIRMED)
                RCLCPP_INFO(node_.get_logger(),
                   "[TORCH IO RESULT] on=%d request=%lld result=%s stage=%s elapsed_ms=%.1f %s",
                    int(enabled),static_cast<long long>(request_id),resultName(result),stage,elapsed,detail.c_str());
            else
                RCLCPP_ERROR(node_.get_logger(),
                    "[TORCH IO RESULT] on=%d request=%lld result=%s stage=%s elapsed_ms=%.1f %s",
                    int(enabled),static_cast<long long>(request_id),resultName(result),stage,elapsed,detail.c_str());
            return result;
        };
        const auto canceled_on = [&] { return enabled && canceled && canceled(); };
        const auto remaining = [&] {
            const auto now=SteadyClock::now();
            return now>=deadline ? std::chrono::milliseconds(0) :
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline-now);
        };
        while(rclcpp::ok() && remaining().count()>0) {
            if(canceled_on()) return done(TorchCommandResult::CANCELED,"service_wait");
            if(io_client_->wait_for_service(std::min(remaining(),std::chrono::milliseconds(200)))) break;
        }
        if(!io_client_->service_is_ready())
            return done(TorchCommandResult::SERVICE_UNAVAILABLE,"service_wait");
        if(canceled_on() || !rclcpp::ok()) return done(TorchCommandResult::CANCELED,"before_send");
        if(enabled && hasFault()) return done(TorchCommandResult::FAULT,"before_send");
        if(remaining().count()<=0) return done(TorchCommandResult::REQUEST_FAILED,"deadline_before_send");

        auto request=std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun=1; request->pin=config_.torch_output_pin; request->state=enabled ? 1.0 : 0.0;
        bool prior_on_uncertainty = false;
        if (enabled) {
            std::lock_guard<std::mutex> lock(mutex_);
            prior_on_uncertainty = on_request_uncertain_;
            on_request_uncertain_ = true;
        }
        auto pending=io_client_->async_send_request(request);
        request_id=pending.request_id;
        RCLCPP_INFO(node_.get_logger(),"[TORCH IO REQUEST] on=%d pin=%d request=%lld service=%s",
            int(enabled),config_.torch_output_pin,static_cast<long long>(pending.request_id),
            config_.set_io_service.c_str());
        while(rclcpp::ok() && remaining().count()>0) {
            if(canceled_on()) {
                io_client_->remove_pending_request(pending);
                return done(TorchCommandResult::CANCELED,"service_reply");
            }
            if(pending.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) continue;
            try {
                const auto response=pending.get();
                if (enabled) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    // This ON call finished, but an earlier missing response
                    // must remain uncertain even after another successful ON.
                    on_request_uncertain_ = prior_on_uncertainty;
                }
                RCLCPP_INFO(node_.get_logger(),
                    "[TORCH IO ACK] request=%lld success=%d elapsed_ms=%.1f callback_group=dedicated",
                    static_cast<long long>(request_id),int(response && response->success),
                    std::chrono::duration<double,std::milli>(SteadyClock::now()-started).count());
                if(!response || !response->success)
                    return done(TorchCommandResult::REQUEST_FAILED,"service_rejected");
            } catch(const std::exception& e) {
                RCLCPP_ERROR(node_.get_logger(),"SetIO exception %s",e.what());
                return done(TorchCommandResult::REQUEST_FAILED,"service_exception");
            }
            while(rclcpp::ok() && SteadyClock::now()<deadline) {
                // External predicates may themselves read torch state.
                if(canceled_on()) return done(TorchCommandResult::CANCELED,"output_feedback");
                std::unique_lock<std::mutex> lock(mutex_);
                TorchCommandResult result=TorchCommandResult::FEEDBACK_TIMEOUT;
                bool finished=true;
                if(enabled && state_==State::FAULT) result=TorchCommandResult::FAULT;
                else if(!freshLocked()) result=TorchCommandResult::IO_LOST;
                else if(telemetry_.output_on==std::optional<bool>(enabled)) result=TorchCommandResult::CONFIRMED;
                else finished=false;
                if(finished) { lock.unlock(); return done(result,"output_feedback"); }
                cv_.wait_until(lock,std::min(deadline,SteadyClock::now()+std::chrono::milliseconds(20)));
            }
            return done(TorchCommandResult::FEEDBACK_TIMEOUT,"output_feedback");
        }
        io_client_->remove_pending_request(pending);
        return done(TorchCommandResult::REQUEST_FAILED,"service_reply_timeout");
    }

    bool freshLocked()const{
        if(!last_io_time_){
            return false;
        }
        return std::chrono::steady_clock::now() - *last_io_time_ < std::chrono::duration<double>(config_.output_timeout_s);     
        
    }
    void latchFaultLocked(const std::string& reason){
        if(state_ == State::FAULT) return;

        RCLCPP_ERROR(node_.get_logger(),"[TORCH FAULT FIRST] reason=%s %s",
            reason.c_str(),diagnosticLocked().c_str());
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

    void onIOStates(const ur_msgs::msg::IOStates& msg){
        bool publish_now = false;
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
            // Preserve raw feedback while ignition/cutting is active: reducing
            // that stream to a timer could hide transfer-low pulses and make
            // the bridge incorrectly infer a continuous arc. Only steady idle
            // OFF telemetry is reduced to the existing 10 Hz timer.
            publish_now = state_ == State::IGNITING || state_ == State::ESTABLISHED ||
                state_ == State::CUTTING || state_ == State::SHUTTING_DOWN || next.command_on ||
                next.output_on != std::optional<bool>(false) ||
                next.transfer_detected != std::optional<bool>(false) ||
                next.output_on != telemetry_.output_on ||
                next.transfer_detected != telemetry_.transfer_detected;
            telemetry_ = std::move(next);
            if(state_==State::CUTTING && telemetry_.transfer_detected == std::optional<bool>(true)){
                transfer_lost_since_.reset();
            }
            
        
            const auto received=SteadyClock::now();
            if(last_io_time_)
                max_io_gap_ms_=std::max(max_io_gap_ms_,
                    std::chrono::duration<double,std::milli>(received-*last_io_time_).count());
            ++io_rx_count_;
            last_io_time_ = received;

        }
        cv_.notify_all();
        if (publish_now) publishStatus();
    }

    void publishStatus(){
        hb_robot_interfaces::msg::TorchStatus msg;
        {
            // One coherent snapshot. The I/O callback and timer share a group,
            // so an older timer snapshot cannot overtake a newer I/O snapshot.
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& data = telemetry_;
            msg.stamp = data.stamp;
            msg.segment_name = active_segment_;
            msg.io_connected = freshLocked();
            msg.command_on = data.command_on;
            msg.output_on = data.output_on.value_or(false);
            msg.transfer_detected = data.transfer_detected.value_or(false);
            msg.analog_valid = data.analog_value.has_value();
            msg.analog_input_value = data.analog_value.value_or(0.0);
            msg.analog_domain = static_cast<uint8_t>(data.analog_domain.value_or(0));
            msg.arc_voltage_valid = data.arc_voltage_v.has_value();
            msg.arc_voltage_v = data.arc_voltage_v.value_or(0.0);
            msg.ready = config_.ignition_enabled && state_ == State::IDLE && freshLocked() &&
                !data.command_on && data.output_on == std::optional<bool>(false) &&
                data.transfer_detected == std::optional<bool>(false);
            msg.fault = state_ == State::FAULT;
            msg.fault_reason = fault_reason_;
        }
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


    uint64_t io_rx_count_=0; // mutex_
    double max_io_gap_ms_=0.0; // mutex_
    std::optional<std::chrono::steady_clock::time_point> last_io_time_;
    std::optional<std::chrono::steady_clock::time_point> transfer_lost_since_;
    mutable std::mutex mutex_;
    std::mutex command_mutex_;

    State state_{State::IDLE};
    std::string fault_reason_;
    std::string active_segment_;
    bool fault_reported_{false};
    bool on_request_uncertain_{false}; // mutex_; never cleared by clearFault()
    rclcpp::CallbackGroup::SharedPtr io_feedback_callback_group_;

    std::condition_variable cv_;
    FaultCallback fault_callback_;
    rclcpp::CallbackGroup::SharedPtr io_reply_callback_group_;

    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_sub_;
    rclcpp::Publisher<hb_robot_interfaces::msg::TorchStatus>::SharedPtr status_pub_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
    rclcpp::Node& node_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    // Add as a TorchController member:
    rclcpp::CallbackGroup::SharedPtr status_callback_group_;
    
    TorchIOConfig config_;
    TorchTelemetry telemetry_;
};

