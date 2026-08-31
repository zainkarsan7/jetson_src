#include "crane_controllers/crane_attitude_estimator.hpp"
#include "crane_controllers/crane_attitude_utils.hpp"
#include "tf2_eigen/tf2_eigen.h"

namespace crane_controllers
{
    
    void CraneAttitudeEstimator::set_marker_chassis_t(const Eigen::Isometry3d &marker_to_chassis){
        marker_to_chassis_ = marker_to_chassis;
       
    }

    void CraneAttitudeEstimator::set_rs_imu_chassis_t(const Eigen::Isometry3d &rs_imu_to_chassis){
        rs_imu_to_chassis_ = rs_imu_to_chassis;
    }

    void CraneAttitudeEstimator::update_marker_pose(const geometry_msgs::msg::PoseStamped &marker_pose){
        
        const Eigen::Isometry3d cam_to_marker;
        tf2::fromMsg(marker_pose,cam_to_marker);

        const Eigen::Isometry3d cam_to_chassis = cam_to_marker * marker_to_chassis_;

        state_.pos = cam_to_chassis.translation();
        state_.orn = cam_to_chassis.rotation();
        state_.orn.normalize();
        state_.rpy = attitude::quaternion_to_rpy(state_.orn);

        state_.pose_valid = true;
        state_.pose_stamp = static_cast<double>(marker_pose.header.stamp.sec) + static_cast<double>(marker_pose.header.stamp.nanosec)*1e-9;

    }

    void CraneAttitudeEstimator::update_imu(const sensor_msgs::msg::Imu &imu_msg){
        Eigen::Vector3d omega_imu(imu_msg.angular_velocity.x,
                                  imu_msg.angular_velocity.y,
                                  imu_msg.angular_velocity.z);
        state_.omega = rs_imu_to_chassis_.rotation().normalized() * omega_imu;
        state_.imu_stamp = static_cast<double>(imu_msg.header.stamp.sec) + static_cast<double>(imu_msg.header.stamp.nanosec)*1e-9;
        state_.imu_valid = true;
    }
    const CraneAttitudeState & CraneAttitudeEstimator::state() const{
        return state_;
    }



} // namespace crane_controllers
