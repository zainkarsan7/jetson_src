#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace crane_estimation
{

        double wrap_angle(double angle);
        double get_angle_error(double target, double measured);
        Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);
        
    
    
}