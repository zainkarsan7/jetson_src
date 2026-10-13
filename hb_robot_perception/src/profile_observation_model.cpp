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

        for (auto& sample: coverage_samples_){

            const Eigen::Vector2f p_section = current_match_.transform * sample.position;
            Point query;
            query.x = p_section.x();
            query.y = p_section.y();
            query.z = 0.0f;
            std::vector<int> indices;
            std::vector<float> sq_dists;

            const int found = tree.radiusSearch(
                query,
                rad,
                indices,
                sq_dists
            );
            if (found<=0){
                continue;
            }

            unsigned int valid_neighbors = 0;
            double residual_sum = 0.0;
            for(const float d2 : sq_dists){
                if(!std::isfinite(d2)){
                    continue;
                }
                valid_neighbors++;
                residual_sum+=std::sqrt(d2);
            }
            if (valid_neighbors == 0){
                continue;
            }

            const float frame_support =  clamp01(static_cast<float>(valid_neighbors)/static_cast<float>(config_.target_neighbors));
            sample.support = std::max(sample.support, frame_support);
            const float frame_residual = static_cast<float>(residual_sum)/static_cast<float>(valid_neighbors);
            
            const uint32_t prev_count = sample.num_obs;
            sample.mean_residual = (sample.mean_residual  * prev_count + frame_residual)/(prev_count+1);
            sample.num_obs +=1;
            return true;

        }

        
    }
    float ProfileObservationModel::coverageFraction()const{
            if(coverage_samples_.empty()){
                return 0.0f;
            }
            std::size_t supported = 0;

            for (const auto& sample: coverage_samples_){
                if(sample.support>=0.75f){
                    ++supported;
                }

            }
            return static_cast<float>(supported)/static_cast<float>(coverage_samples_.size());
        }

}
