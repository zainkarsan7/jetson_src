#include "hb_robot_perception/imu_stability_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

namespace hb_perception{

ImuStabilityMonitor::ImuStabilityMonitor(rclcpp::Node& node, const std::string& topic,
                                         ImuStabilityConfig config)
    : ImuStabilityMonitor(node, topic, config, nullptr) {}

ImuStabilityMonitor::ImuStabilityMonitor(
            rclcpp::Node& node,
            const std::string& imu_topic,
            ImuStabilityConfig config,
            rclcpp::CallbackGroup::SharedPtr callback_group):
            config_(config){
            auto qos = rclcpp::SensorDataQoS();
            rclcpp::SubscriptionOptions options;
            options.callback_group = callback_group;
            sub_ = node.create_subscription<sensor_msgs::msg::Imu>(imu_topic,qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg){
                onImu(msg, false);
            }, options);
            if (!config_.auxiliary_accel_topic.empty()) {
                accel_sub_ = node.create_subscription<sensor_msgs::msg::Imu>(
                    config_.auxiliary_accel_topic, qos,
                    [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg, true); }, options);
            }
            }


    void ImuStabilityMonitor::onImu(const sensor_msgs::msg::Imu::SharedPtr msg, bool acceleration_only){
                // ROS Imu uses -1 for an unavailable measurement. An accelerometer-only
                // message cannot establish the gyro part of this six-axis stability test.
                if ((!acceleration_only && msg->angular_velocity_covariance[0] == -1.0) ||
                    msg->linear_acceleration_covariance[0] == -1.0) return;
                Sample s;
                s.recieved_at = Clock::now();
                s.wx = acceleration_only ? 0.0 : msg->angular_velocity.x;
                s.wy = acceleration_only ? 0.0 : msg->angular_velocity.y;
                s.wz = acceleration_only ? 0.0 : msg->angular_velocity.z;
                s.ax = msg->linear_acceleration.x;
                s.ay = msg->linear_acceleration.y;
                s.az = msg->linear_acceleration.z;

                if(!std::isfinite(s.wx)||
                   !std::isfinite(s.wy)||
                   !std::isfinite(s.wz)||
                   !std::isfinite(s.ax)||
                   !std::isfinite(s.ay)||
                   !std::isfinite(s.az)){
                    return;
                   }
                {
                std::lock_guard<std::mutex>lock(mutex_);
                    auto& samples = acceleration_only ? accel_samples_ : samples_;
                    // After a stale interval, require a new observation window before
                    // declaring stability again, even if no timer ran during the gap.
                    if (!samples.empty() &&
                        s.recieved_at - samples.back().recieved_at > config_.max_sample_age) {
                        ++reception_gaps_;
                        samples.clear();
                    }
                    samples.push_back(s);
                    const auto cutoff = s.recieved_at - config_.window;
                    while(!samples.empty() && 
                    samples.front().recieved_at < cutoff
                ){
                    samples.pop_front();
                }
                }
                cv_.notify_all();
                }
    ImuStabilityMetrics ImuStabilityMonitor::evaluateLocked(Clock::time_point now) const{
        auto result = evaluateWindow(now, samples_);
        if (accel_sub_) {
            const auto accel = evaluateWindow(now, accel_samples_);
            result.valid = result.valid && accel.valid;
            result.stable = result.stable && accel.stable;
            result.samples_count = std::min(result.samples_count, accel.samples_count);
            result.sample_age_s = std::max(result.sample_age_s, accel.sample_age_s);
            result.lin_acc_rms_m_s2 = std::max(result.lin_acc_rms_m_s2, accel.lin_acc_rms_m_s2);
            result.latest_acc_deviation_m_s2 = std::max(result.latest_acc_deviation_m_s2, accel.latest_acc_deviation_m_s2);
            result.peak_acc_deviation_m_s2 = std::max(result.peak_acc_deviation_m_s2, accel.peak_acc_deviation_m_s2);
        }
        return result;
    }

    ImuStabilityMetrics ImuStabilityMonitor::evaluateWindow(
            Clock::time_point now, const std::deque<Sample>& samples) const {
        ImuStabilityMetrics result_;
        result_.reception_gaps = reception_gaps_;
        if(samples.empty()){
            return result_;

        }
        result_.sample_age_s = std::chrono::duration<double>(
            now - samples.back().recieved_at).count();
        if(now - samples.back().recieved_at > config_.max_sample_age){
            return result_;
        }
        // Freshness is measured from the reader to the latest received sample above.
        // Coverage and motion statistics describe the window ending at that sample.
        // A delayed timer read must not discard the start of an otherwise fresh window.
        const auto cutoff = samples.back().recieved_at - config_.window;

        double sum_wx = 0.0;
        double sum_wy = 0.0;
        double sum_wz = 0.0;
        double sum_ax = 0.0;
        double sum_ay = 0.0;
        double sum_az = 0.0;

        const Sample* first = nullptr;


        double peak_omega = 0.0;
        double peak_acc_deviation = 0.0;
        for(const auto& s :samples){
            if(s.recieved_at < cutoff){
                continue;
            }
            if(!first){
                first = &s;
            }

            sum_wx += s.wx;
            sum_wy += s.wy;
            sum_wz += s.wz;
            sum_ax += s.ax;
            sum_ay += s.ay;
            sum_az += s.az;
            ++result_.samples_count;            
        }

        if(result_.samples_count<config_.min_samples || !first){
            return result_;
        }
        // some span of time should be within threshold
        const auto span = samples.back().recieved_at - first->recieved_at;
        if(span < 0.9 * config_.window){
            return result_;
        }
        // Validity means enough finite, fresh samples spanning the observation
        // window. It does NOT imply stability: valid motion must remain observable.
        // Recovery guards require both flags; leaving valid at its default false
        // silently rejects every workpiece reference regardless of IMU quality.
        result_.valid = true;
        const double n = static_cast<double>(result_.samples_count);
        const double m_wx = sum_wx/n;
        const double m_wy = sum_wy/n;
        const double m_wz = sum_wz/n;
        const double m_ax = sum_ax/n;
        const double m_ay = sum_ay/n;
        const double m_az = sum_az/n;

        const double norm_ang_vel = m_wx * m_wx + 
                                    m_wy * m_wy + 
                                    m_wz * m_wz;

        const double norm_lin_acc = m_ax * m_ax + 
                                    m_ay * m_ay + 
                                    m_az * m_az;

        double var_ang_vel = 0.0;
        double var_lin_acc = 0.0;                                    

        
        
        // covariance calc;
        for(const auto & s: samples){
            if(s.recieved_at < cutoff){
                continue;
            }

            const double dwx = s.wx-m_wx;
            const double dwy = s.wy-m_wy;
            const double dwz = s.wz-m_wz;
            const double dax = s.ax-m_ax;
            const double day = s.ay-m_ay;
            const double daz = s.az-m_az;


            var_ang_vel += dwx * dwx + 
                            dwy * dwy + 
                            dwz * dwz;

            var_lin_acc += dax * dax + 
                            day * day + 
                            daz * daz;

            const double omega = std::sqrt(s.wx*s.wx + s.wy * s.wy + s.wz*s.wz);
            const double acc_deviation = std::sqrt(dax*dax + day*day + daz * daz );
            peak_omega = std::max(peak_omega,omega);
            peak_acc_deviation = std::max(peak_acc_deviation,acc_deviation);
            
            
            
        }
        result_.ang_vel_rms_rad_s = std::sqrt(var_ang_vel/n);
        result_.lin_acc_rms_m_s2 = std::sqrt(var_lin_acc/n);
        result_.mean_ang_vel_rad_s = std::sqrt(norm_ang_vel);
        const auto& latest = samples.back();
        result_.latest_ang_vel_rad_s = std::sqrt(
            latest.wx*latest.wx + latest.wy*latest.wy + latest.wz*latest.wz);
        result_.latest_acc_deviation_m_s2 = std::sqrt(
            (latest.ax-m_ax)*(latest.ax-m_ax) +
            (latest.ay-m_ay)*(latest.ay-m_ay) +
            (latest.az-m_az)*(latest.az-m_az));
        result_.peak_acc_deviation_m_s2 = peak_acc_deviation;
        result_.peak_ang_vel_rad_s = peak_omega;
        result_.stable = 
                result_.ang_vel_rms_rad_s< config_.max_ang_vel_rms && 
                result_.lin_acc_rms_m_s2< config_.max_lin_acc_rms  &&
                result_.mean_ang_vel_rad_s< config_.max_mean_ang_vel;

        return result_;
        

    }

    ImuStabilityMetrics ImuStabilityMonitor::metrics() const{
            std::lock_guard<std::mutex>lock(mutex_);

            return evaluateLocked(Clock::now());
        }
    
    void ImuStabilityMonitor::reset(){

        std::lock_guard<std::mutex>lock(mutex_);

        samples_.clear();
        accel_samples_.clear();

        
    }

    bool ImuStabilityMonitor::waitForStable(
        std::chrono::milliseconds timeout,
        const std::function<bool()>& canceled
    ){
        const auto deadline = Clock::now() + timeout;
        std::unique_lock<std::mutex>lock(mutex_);
        while(rclcpp::ok()){
            if(canceled && canceled()){
                return false;
            }
            if(evaluateLocked(Clock::now()).stable){
                return true;
            }
            if(Clock::now() >=deadline){
                return false;
            }
            cv_.wait_for(lock,std::chrono::milliseconds(25));
        }
        return false;

    }


}

