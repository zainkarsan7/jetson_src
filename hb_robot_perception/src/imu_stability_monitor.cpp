#include "hb_robot_perception/imu_stability_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

namespace hb_perception{

ImuStabilityMonitor::ImuStabilityMonitor(
            rclcpp::Node& node,
            const std::string& imu_topic,
            ImuStabilityConfig config):
            config_(config){
            
            sub_ = node.create_subscription<sensor_msgs::msg::Imu>(imu_topic,10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg){
                onImu(msg);
            });
            }


    void ImuStabilityMonitor::onImu(const sensor_msgs::msg::Imu::SharedPtr msg){
                Sample s;
                s.recieved_at = Clock::now();
                s.wx = msg->angular_velocity.x;
                s.wy = msg->angular_velocity.y;
                s.wz = msg->angular_velocity.z;
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
                    samples_.push_back(s);
                    const auto cutoff = s.recieved_at - config_.window;
                    while(!samples_.empty() && 
                    samples_.front().recieved_at < cutoff
                ){
                    samples_.pop_front();
                }
                }
                cv_.notify_all();
                }
    ImuStabilityMetrics ImuStabilityMonitor::evaluateLocked(Clock::time_point now) const{
        ImuStabilityMetrics result_;
        if(samples_.empty()){
            return result_;

        }
        if(now - samples_.back().recieved_at > config_.max_sample_age){
            return result_;
        }
        const auto cutoff = now - config_.window;

        double sum_wx = 0.0;
        double sum_wy = 0.0;
        double sum_wz = 0.0;
        double sum_ax = 0.0;
        double sum_ay = 0.0;
        double sum_az = 0.0;

        const Sample* first = nullptr;

        for(const auto& s :samples_){
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
            ++result_.samples;            
        }

        if(result_.samples<config_.min_samples || !first){
            return result_;
        }
        // some span of time should be within threshold
        const auto span = samples_.back().recieved_at - first->recieved_at;
        if(span < 0.9 * config_.window){
            return result_;
        }
        const double n = static_cast<double>(result_.samples);
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
        for(const auto & s: samples_){
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
            
            
            
        }
        result_.ang_vel_rms_rad_s = std::sqrt(var_ang_vel/n);
        result_.lin_acc_rms_m_s2 = std::sqrt(var_lin_acc/n);
        result_.mean_ang_vel_rad_s = std::sqrt(norm_ang_vel);
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
            cv_.wait_for(lock,std::chrono::microseconds(25));
        }
        return false;

    }


}

