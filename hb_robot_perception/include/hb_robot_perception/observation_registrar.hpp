#pragma once
#include <Eigen/Geometry>
#include <pcl/registration/icp.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstddef>


    

namespace hb_perception{


    using PointT = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT>;

    struct RegistrationConfig{
        double max_correspondence_m = 0.025;
        int max_iters = 40;
        double max_translation_m = 0.02;
        double max_rotation_rad = 0.035;
        double max_fitness_m2 = 0.000025;
        std::size_t min_points = 100;
    };
    struct RegistrationResult{
        bool accepted= false;
        bool converged = false;
        Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
        double fitness_m2 = 0.0;
        double translation_m = 0.0;
        double rotation_rad = 0.0;
        std::string reason;
    };


    class ObservationRegistrar{
        public:
            ObservationRegistrar(const RegistrationConfig config={}):
            config_(config){

            }

            RegistrationResult registerCloud(
                const PointCloud& source, 
                const PointCloud& target
            ) const;


        
        private:

            RegistrationConfig config_;
    }

}