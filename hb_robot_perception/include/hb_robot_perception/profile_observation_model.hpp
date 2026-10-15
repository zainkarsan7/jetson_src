#pragma once
#include "hb_robot_perception/profile_types.hpp"
#include "hb_robot_perception/perception_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>



namespace hb_perception{

    struct ProfileCoverageSample{
        Eigen::Vector2f position = Eigen::Vector2f::Zero();
        Eigen::Vector2f normal= Eigen::Vector2f::Zero();
        float support = 0.0f;
        float mean_residual = 0.0f;
        uint32_t num_obs = 0;

    };

    struct ProfileCoverageConfig{
        float sample_spacing = 0.003f;
        float support_radius = 0.006f;
        unsigned int target_neighbors = 4;

    };


class ProfileObservationModel{
        public:

            explicit ProfileObservationModel(ProfileCoverageConfig config = {});

            void reset();

            bool integrate(
                const SectionModel& section
                // const Eigen::Isometry3f& world_from_section,
                // const rclcpp::Time& stamp
            );

            void setMatch(const ProfileMatch& match);

            void updateMatch(const ProfileMatch& match);
            const std::vector<ProfileCoverageSample>& coverage() const{
                return coverage_samples_;
            }
            float coverageFraction() const;

            
            const ProfileMatch& match() const{
                return current_match_;
            }
            uint32_t observationCount() const{
                return num_obs_;
            }

        private:

            void sampleBoundary();
            
            ProfileCoverageConfig config_;
            bool initialized_ = false;

            std::vector<ProfileCoverageSample> coverage_samples_;
            ProfileMatch current_match_;
            uint32_t num_obs_=0;
    };

}
