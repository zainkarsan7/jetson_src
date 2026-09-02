#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace crane_controllers{

    struct CraneAttitudeState{
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();

        Eigen::Quaterniond orn = Eigen::Quaterniond::Identity();

        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();

        Eigen::Vector3d omega = Eigen::Vector3d::Zero();

        bool pose_valid= false;
        bool imu_valid= false;

        double pose_stamp= 0.0;
        double imu_stamp = 0.0;


    };

    class CraneAttitudeEstimator{
        public:
         CraneAttitudeEstimator() = default;

        void set_marker_chassis_t(const Eigen::Isometry3d &marker_to_chassis);

        void set_rs_imu_chassis_t(const Eigen::Isometry3d &rs_imu_to_chassis);

        void update_marker_pose(const geometry_msgs::msg::TransformStamped & marker_pose);

        void update_imu(const sensor_msgs::msg::Imu &imu_msg);
        const CraneAttitudeState & state() const;



        private:
        Eigen::Isometry3d marker_to_chassis_ = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d rs_imu_to_chassis_ = Eigen::Isometry3d::Identity();

        CraneAttitudeState state_;
        
    };

}