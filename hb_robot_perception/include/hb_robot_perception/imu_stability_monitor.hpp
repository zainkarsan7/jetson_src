#pragma once 
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <deque>
namespace hb_perception{


struct ImuStabilityConfig{
    double max_ang_vel_rms=0.01;
    double max_lin_acc_rms=0.1;
    double max_mean_ang_vel=0.03;
    std::size_t min_samples=20;
    std::chrono::milliseconds window{500};
    std::chrono::milliseconds max_sample_age{200};
};

struct ImuStabilityMetrics{
    // magnitudes:
    double ang_vel_rms_rad_s=0.0;
    double lin_acc_rms_m_s2=0.0;
    double mean_ang_vel_rad_s=0.0;
    
    bool stable=false;
    std::size_t samples_count=0;

    bool valid = false;
    double sample_age_s;
    double peak_ang_vel_rad_s = 0.0;
    double peak_acc_deviation_m_s2 = 0.0;

    double latest_ang_vel_rad_s = 0.0;
    double latest_acc_deviation_m_s2 = 0.0;




};

class ImuStabilityMonitor{
    public:

        ImuStabilityMonitor(
            rclcpp::Node& node,
            const std::string& imu_topic,
            ImuStabilityConfig config = {}

        );
        bool waitForStable(
            std::chrono::milliseconds timeout,
            const std::function<bool()>& canceled = {}
        );
        
       
        ImuStabilityMetrics metrics() const;
        
        void reset();

        const ImuStabilityConfig& config(){
            return config_;
        }

    private:
        using Clock = std::chrono::steady_clock;
        struct Sample{
            Clock::time_point recieved_at;
            double wx,wy,wz;
            double ax,ay,az;
        };
        

        void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
        ImuStabilityMetrics evaluateLocked(Clock::time_point now) const;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
        ImuStabilityConfig config_;
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::deque<Sample> samples_;

        ImuStabilityMetrics metric_;
};


enum class PlatformMotionState{

    UNKNOWN,
    STABLE,
    DISTURBED,
    LATCHED
};

struct PlatformSupervisorConfig{
    double max_peak_acc_deviation_m_s2 = 0.0;
    double max_peak_ang_vel_rad_s = 0.0;
    double max_ang_vel_rms_rad_s= 0.0; 
    double max_lin_acc_rms_m_s2 = 0.0;
    std::chrono::milliseconds stable_dwell;
    
};

class PlatformMotionSupervisor{
    public:
    explicit  PlatformMotionSupervisor(PlatformSupervisorConfig config);

    PlatformMotionState update(const ImuStabilityMetrics& metrics);
    bool executionPermitted();
    bool disturbanceLatched();
    void resetLatch();


    private:
        PlatformMotionState state_ = PlatformMotionState::UNKNOWN;
        PlatformSupervisorConfig config_;
};

}
