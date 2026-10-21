#pragma once 
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cstddef>
#include <chrono>
namespace hb_perception{


struct ImuStabilityMetrics{
    // magnitudes:
    double ang_vel_rms_rad_s;
    double lin_acc_rms_m_s2;
    double orn_rms_rad_s;
    
    bool stable;
    double window_size_s;

};

class ImuSettlingMonitor{
    public:
        void addSample(const sensor_msgs::msg::Imu& msg );
        ImuStabilityMetrics evaluate() const;
        bool waitUntilStable(
            std::chrono::milliseconds timeout
        );

    private:
        ImuStabilityMetrics metric_;
};



}
