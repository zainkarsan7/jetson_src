#include "crane_controllers/crane_attitude_estimator.hpp"
#include "crane_controllers/crane_attitude_utils.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace crane_controllers
{
    
    void CraneAttitudeEstimator::set_marker_chassis_t(const Eigen::Isometry3d &marker_to_chassis){
        marker_to_chassis_ = marker_to_chassis;
       
    }

    void CraneAttitudeEstimator::set_rs_imu_chassis_t(const Eigen::Isometry3d &rs_imu_to_chassis){
        rs_imu_to_chassis_ = rs_imu_to_chassis;
    }

    void update_marker_pose(const geometry_msgs::msg::PoseStamped &marker_pose){
        
        const Eigen::Isometry3d ref_;
    }



} // namespace crane_controllers
