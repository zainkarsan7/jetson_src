#include "hb_robot_perception/observation_registrar.hpp"
#include <algorithm>
#include <cmath>

namespace hb_perception{

    RegistrationResult ObservationRegistrar::registerCloud(
                const PointCloud& source, 
                const PointCloud& target
            ) const{

                RegistrationResult result;

                if(source.size() < config_.min_points ||
                    target.size() < config_.min_points){
                        result.reason = "insufficient ponts";
                        return result;
                    }
                

                    pcl::IterativeClosestPoint<PointT,PointT> icp;
                    icp.setInputSource(std::make_shared<PointCloud>(source));
                    icp.setInputTarget(std::make_shared<PointCloud>(target));
                    icp.setMaxCorrespondenceDistance(config_.max_correspondence_m);
                    icp.setMaximumIterations(config_.max_iters);
                    icp.setTransformationEpsilon(1e-8);
                    icp.setEuclideanFitnessEpsilon(1e-7);
                    PointCloud aligned;
                    icp.align(aligned);
                    result.converged = icp.hasConverged();
                    if(!result.converged){
                        result.reason = "icp failed to converge";
                        return result;
                    }
                    result.correction = icp.getFinalTransformation();
                    result.fitness_m2 = icp.getFitnessScore(config_.max_correspondence_m);
                    const Eigen::Vector3f translation = result.correction.block<3,1>(0,3);
                    const Eigen::Matrix3f rotation = result.correction.block<3,3>(0,0);
                    result.translation_m = translation.norm();
                    result.rotation_rad = Eigen::AngleAxisf(rotation).angle();
                    if(!std::isfinite(result.fitness_m2) ||
                    !std::isfinite(result.translation_m) ||
                    !std::isfinite(result.rotation_rad)){
                        result.reason = "non finite registration result";
                        return result;
                    } 
                    if(result.translation_m>config_.max_translation_m){
                        result.reason = "ICP translation too large";
                        return result;

                    }
                    if(result.rotation_rad  > config_.max_rotation_rad){
                        result.reason= "ICP rotation too large";
                        return result;
                    }
                    if (result.fitness_m2 > config_.max_fitness_m2){
                        result.reason = "fitness too poor";
                        return result;
                    }
                    result.accepted = true;
                    result.reason = "accepted";
                    return result;

            }
        }


