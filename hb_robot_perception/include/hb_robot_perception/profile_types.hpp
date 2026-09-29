#pragma once


#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>
#include <variant>

namespace hb_perception{

    enum class ProfileFamily{
        Unknown,
        IPN,
        IPE,
        Circular,
        Rectangular
    };

    struct LineSegment2D{
        Eigen::Vector2f a = Eigen::Vector2f::Zero();
        Eigen::Vector2f b = Eigen::Vector2f::Zero();
    };
    struct ArcSegment2D{
        Eigen::Vector2f center = Eigen::Vector2f::Zero();
        float start  = 0.0f;
        float end    = 0.0f;
        float radius = 0.0f;

    };


    using ProfilePrimitive = std::variant<LineSegment2D, ArcSegment2D>;

    struct ProfileModel{

        std::string name;
        ProfileFamily family = ProfileFamily::Unknown;
        std::vector<ProfilePrimitive> boundary;
        float height = 0.0;
        float width = 0.0;
    };


    struct ProfileMatch{
        ProfileModel profile;
        Eigen::Isometry2f transform  = Eigen::Isometry2f::Identity();
        std::vector<float> residuals;
        float rms_dist = std::numeric_limits<float>::infinity();
        // points that are close enough to the profile as fraction of total points
        float inlier_fraction = 0.0f;
        
        float score = 0.0f;
    };






    
}