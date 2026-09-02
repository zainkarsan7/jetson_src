#include "crane_estimation/crane_attitude_utils.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <algorithm>

namespace crane_estimation{
        double wrap_angle(double angle){
            return std::remainder(angle,2.0*M_PI);
        }
        double get_angle_error(double target, double measured){
            return wrap_angle(target-measured);
        }


        Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q){

            const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
            const double roll = std::atan2(
                R(2,1),R(2,2)
            );
            const double pitch = std::asin(std::clamp(-R(2,0),-1.0,1.0));
            const double yaw = std::atan2(R(1,0),R(0,0));

            return Eigen::Vector3d (wrap_angle(roll),wrap_angle(pitch),wrap_angle(yaw));
        }


    
}