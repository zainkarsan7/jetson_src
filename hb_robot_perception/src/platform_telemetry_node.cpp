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

        static double norm(const std::array<double,3> v){
            return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        }

        static geometry_msgs::msg::Vector3 vectorMsg(const std::array<double,3> v){
            geometry_msgs::msg::Vector3 out;
            out.x = v[0];
            out.y = v[1];
            out.z = v[2];
            return out;
        }
        Imu onImu(Imu&msg){
            Sample s;
            s.receipt = Clock::now();
            s.stamp = rclcpp::Time(msg.header.stamp,get_clock()->get_clock_type());
            s.w = {msg.angular_velocity.x, msg.angular_velocity.y,msg.angular_velocity.z};
            s.a = {msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z};
            for(const auto v: s.w) if(!std::isfinite(v)) return;
            for(const auto v: s.a) if(!std::isfinite(v)) return;
            Imu filtered =msg;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!filter_ready_){

                    filtered_w_ = s.w;
                    filtered_a_ = s.a;
                    filter_ready_ = true;

                }else{
                    auto elapsed = s.receipt - last_receipt_;
                    const double dt_seconds = std::chrono::duration<double>(elapsed).count();
                    

                    // low pass signal filter:alpha =  1-e^-2pi * f * dt
                    if(dt_seconds > 0.0 && dt_seconds < 1.0){
                         const double alpha = 1 - std::exp(-2.0 * M_PI * dt_seconds * cutoff_hz_);

                    // filter both acceleration and omega
                        for (size_t k=0; k<3; k++){
                            filtered_w_[k] += alpha * (s.w[k] - filtered_w_[k]);
                            filtered_a_[k] += alpha * (s.a[k] - filtered_a_[k]);
                        }
                    }
                    else{
                        filtered_a_ = s.a;
                        filtered_w_ = s.w;
                    }
                    
                }
                last_receipt_ = s.receipt;
                samples_.push_back(s);
               
                const auto cutoff = s.receipt - std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(window_s_));
                
                while(!samples_.empty() && 
                samples_.front().receipt < cutoff
            ){
                samples_.pop_front();
            }
            filtered.angular_velocity = vectorMsg(filtered_w_);
            filtered.linear_acceleration = vectorMsg(filtered_a_);

              
            }// decon the mutex
        filtered.angular_velocity_covariance[0] = -1.0;
        filtered.linear_acceleration_covariance[0] = -1.0;
        filtered_imu_pub_->publish(filtered);
        }
        void publishMetrics(){
            std::vector<Sample> samples;

            const auto now = Clock::now();
            {
                std::lock_guard<std::mutex>lock(mutex_);
                samples.assign(samples_.begin(),samples_.end());
            }
            Metrics m;
            m.header.stamp = this->now();
            m.sample_age = std::numeric_limits<double>::infinity();
            m.sample_count = static_cast < size_t> (samples.size());
            if(!samples.empty()){
                m.header.stamp = samples.back().stamp;
                m.sample_age = std::chrono::duration<double>(now - samples.back().receipt).count();
                
            }
            if(samples.size() >= static_cast<size_t>(min_samples_) && 
        m.sample_age <= max_age_s_ &&
            std::chrono::duration<double>(samples.back().receipt - samples.front().receipt).count() >= min_coverage_ * window_s_){
        m.valid = true;
        std::array<double, 3> mw{}, ma{}, vw{}, va{};
            //clacl mean
        for(const auto& s: samples){
            for (size_t k=0;k<3;k++){
                mw[k] +=s.w[k];
                ma[k] +=s.a[k];
                }
        }
        const double n = static_cast<double>(samples.size());
        for (size_t k=0;k<3;k++){
                mw[k]/=n;
                ma[k]/=n;
                }
            
        //calc var
        for (const auto&s :samples){
            std::array<double,3> da{};
            for (size_t k=0;k<3;k++){
                const double dw = s.w[k] -mw[k];

                vw[k] += dw* dw;

                da[k] += s.a[k] - ma[k];
                va[k] += da[k]*da[k];

            }
            m.gyro_peak_rad_s = std::max(m.gyro_peak_rad_s,norm(s.w));
            m.accel_deviation_peak_m_s2 = std::max(m.accel_deviation_peak_m_s2, norm(da));

        }
        // these are for a sqrt norm
        double sw = 0; double sa=0;
        for(size_t k=0;k<3;k++){
            vw[k]/=n;
            va[k]/=n;
            sw += vw[k];
            sa += va[k];
            vw[k] = std::sqrt(vw[k]);
            va[k] = std::sqrt(va[k]);

            }
        m.gyro_mean_rad_s = vectorMsg(mw);
        m.accel_mean_m_s2 = vectorMsg(ma);
        m.gyro_stddev_rad_s = vectorMsg(vw);
        m.accel_stddev_m_s2 = vectorMsg(va);
        m.gyro_residual_rms_rad_s = std::sqrt(sw);
        m.accel_residual_rms_m_s2 = std::sqrt(sa);
        m.accel_mean_norm_m_s2 = norm(ma);
        m.gyro_latest_rad_s = norm(samples.back().w);
        std::array<double,3> latest_da{};
        for(size_t k=0; k<3;k++){
            latest_da[k] = samples.back().a[k] - ma[k];
            }
            m.accel_deviation_latest_m_s2 = norm(latest_da);
            const bool below = m.gyro_residual_rms_rad_s < max_gyro_rms_ &&
            m.accel_residual_rms_m_s2 < max_accel_rms_ &&
            norm(mw) < max_mean_gyro_;

        if(below){
            if(!candidate_since_) candidate_since_ = now;
            m.stable = std::chrono::duration<double>(now- *candidate_since_).count() >=stable_dwell_s_;
            else candidate_since_.reset();
        }
        else  candidate_since_.reset();

        metrics_pub_->publish(m);
        std_msgs::msg::UInt8 state;
        //unknown = 0, stable = 1, disturbed = 2
        state.data = !m.valid ? 0 : (m.stable? 1 : 2);
        state_pub_->publish(state);
        }
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
   
    rclcpp::init(argc,argv);
    try{
        rclcpp::spin(std::make_shared<hb_perception::PlatformTelemetryNode>());
    }catch(const std::exception& e){
        RCLCPP_FATAL(rclcpp::get_logger(),"%s",e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown()
    return 0;

}