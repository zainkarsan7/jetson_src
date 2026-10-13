#include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_perception/profile_observation_model.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace hb_perception{

    namespace{
        constexpr float kPi = 3.1415926535f;
        float clamp01(float x){
            return std::clamp(x,0.0f,1.0f);
        }

    }

    ProfileObservationModel::ProfileObservationModel(ProfileCoverageConfig config):
    config_(config){
        if(config_.sample_spacing < 0.0f || config_.support_radius <=0.0f || config_.target_neighbors == 0){
            throw std::invalid_argument("config is messed up");
        }

    }


    void ProfileObservationModel::reset(){
        coverage_samples_.clear();
        num_obs_ = 0;
        initialized_ = false;
        current_match_ = ProfileMatch{};
    }

    void ProfileObservationModel::setMatch(const ProfileMatch& match){
        current_match_ = match;
        sampleBoundary();
        initialized_ = true;
        num_obs_ = 0;
    }

    void ProfileObservationModel::sampleBoundary(){
        coverage_samples_.clear();
        const float spacing = config_.sample_spacing;
        auto addSample=[&](const Eigen::Vector2f& p, const Eigen::Vector2f& n){
            ProfileCoverageSample sample;
            sample.position = p;
            sample.normal = n;
            coverage_samples_.push_back(sample);
        };

        for (const auto& prim: current_match_.profile.boundary){
         
            if (const auto* line = std::get_if<LineSegment2D>(&prim)){
                const Eigen::Vector2f delta = line->b - line->a;
                const float length = delta.norm();
                if(length<1e-8){
                    return;
                }
                const Eigen::Vector2f tangent = delta/length;
                const Eigen::Vector2f normal(-tangent.y(),tangent.x());

                const int steps = std::max(1,static_cast<int>(std::ceil(length/spacing)));

                for (int i =0; i<=steps; i++){
                    const float t = static_cast<float>(i)/steps;
                    addSample(line->a + delta * t,normal);
                }


            }
        }

    }


    bool ProfileObservationModel::integrate(
        const SectionModel& section
    ){

        if(!initialized_ || coverage_samples_.empty() || !section.cloud || section.cloud->empty()){
            std::cerr<<"emtpy section model or no samples or not initialized"<<std::endl;
            return false;
        }
        using Point = pcl::PointXYZ;
        auto cloud2d = pcl::make_shared<pcl::PointCloud<Point>>();
        cloud2d->reserve(section.cloud->size());
        for (const auto& sp : section.cloud->points){
            if(!std::isfinite(sp.x)|| !std::isfinite(sp.y)){
                continue;
            }
            Point q;
            q.x = sp.x;
            q.y = sp.y;
            q.z = 0.0f;
            cloud2d->push_back(q);
        }
        if(cloud2d->empty()){
            std::cerr<<"cloud2d is emtpy"<<std::endl;
            return false;
        }

        pcl::KdTreeFLANN<Point> tree;
        tree.setInputCloud(cloud2d);
        const float rad = config_.support_radius;
        const float rad_sq = rad * rad;


        





    }


















}
