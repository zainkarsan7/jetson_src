#pragma once
#include "hb_robot_interfaces/msg/profile_estimate.hpp"
#include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <vector>

namespace hb_perception{

    class ProfileMatcher{
        public:

            struct Parameters{
                float pi = 3.141592653589f;
                float inlier_tolerance = 0.005f;
                // i dont now what truncation dist does, like a threshold error
                float truncation_distance = 0.02f;
                float max_translation = 0.03f;
                float max_rotation = 10.0f * pi / 180.0f;
                float translation_step = 0.005f;
                float rotation_step = 2.0f * pi/180.0f;
            };
            ProfileMatcher();
            explicit ProfileMatcher(const Parameters& params);

            std::vector<ProfileMatch> match(
                const SectionModel& section,
                const std::vector<ProfileModel>& candidates
            ) const;

            visualization_msgs::msg::MarkerArray getVisualization(
    const SectionModel& section,
    const std::vector<ProfileMatch>& matches,
    const std::string& frame_id,
    std::size_t max_matches) const;
                hb_robot_interfaces::msg::ProfileEstimate getProfileEstimateMsg(const std::string& planning_frame,const SectionModel& section, const ProfileMatch& match);
        private:

          std::vector<Eigen::Isometry2f> canonicalTransforms(
        const SectionModel& section) const;

        float transformCost(
    const SectionModel& section,
    const ProfileModel& profile,
    const Eigen::Isometry2f& transform) const;

            ProfileMatch fitCand(const SectionModel& section, 
            const ProfileModel& profile) const;

            ProfileMatch evalTransform(const SectionModel& section,
            const ProfileModel& profile,
        const Eigen::Isometry2f& transform) const;

            float pointToProfileDistance(
                const Eigen::Vector2f& point,
                const ProfileModel& profile,
                const Eigen::Isometry2f& profile_to_section) const;
            
            static float pointToLineSegmentDistance(
                const Eigen::Vector2f& point,
                const LineSegment2D& segment
            );
            // later add arcs

            static float pointToLineArcDistance(
                const Eigen::Vector2f& point,
                const ArcSegment2D& segment
            );

            static std::vector<Eigen::Vector2f>
            sampleProfileBoundary(
                const ProfileModel& profile,
                float arc_resolution=0.002f);

            Eigen::Isometry3f makeProfilePose(const SectionModel& section, const ProfileMatch& match);

            

            Parameters params_;
    };





}