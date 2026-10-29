#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "hb_robot_perception/profile_mesh_builder.hpp"

// Compare the actual marker path with the serialized collision-mesh path.
// Include reflections: tiny changes in observations can change the winning
// candidate's handedness for a symmetric IPN section.
TEST(ProfilePose, SerializedExtrusionAgreesWithMarkerForEveryHandedness)
{
    hb_perception::ProfileMatcher matcher;
    hb_perception::ProfileMeshBuilder builder;
    hb_perception::SectionModel section;
    section.frame = Eigen::Isometry3f::Identity();
    section.frame.linear() = Eigen::AngleAxisf(
        1.1f, Eigen::Vector3f(1.0f, 2.0f, -0.5f).normalized()).toRotationMatrix();
    section.frame.translation() = Eigen::Vector3f(0.4f, -0.7f, 1.2f);
    hb_perception::ProfileMatch match;
    match.profile = hb_perception::ProfileLibrary::ipnProfiles().front();
    const auto mesh = builder.extrude(match.profile, -0.3, 0.3);
    ASSERT_TRUE(mesh.has_value());

    for (const float handedness : {1.0f, -1.0f, 1.0f, -1.0f})
    {
        for (const float angle : {0.0f, 0.5f, 1.5707963f, 3.1415927f, 4.712389f})
        {
            SCOPED_TRACE(::testing::Message() << "handedness=" << handedness << " angle=" << angle);
            match.transform.setIdentity();
            match.transform.linear() = Eigen::Rotation2Df(angle).toRotationMatrix();
            match.transform.linear().col(0) *= handedness;
            match.transform.translation() = Eigen::Vector2f(0.013f, -0.021f);

            const Eigen::Isometry3d pose = matcher.makeProfilePose(section, match).cast<double>();
            EXPECT_NEAR(pose.linear().determinant(), 1.0, 1e-5);
            Eigen::Isometry3d wire_pose;
            tf2::fromMsg(matcher.getProfileEstimateMsg("world", section, match).pose, wire_pose);
            EXPECT_TRUE(wire_pose.matrix().isApprox(pose.matrix(), 1e-5));

            const auto markers = matcher.getVisualization(section, {match}, "world", 1);
            bool found_boundary = false;
            for (const auto& marker : markers.markers)
            {
                if (marker.ns != "profile_boundary") continue;
                found_boundary = true;
                for (const auto& point : marker.points)
                {
                    const Eigen::Vector3d expected(point.x, point.y, point.z);
                    bool found_vertex = false;
                    for (const auto& vertex : mesh->vertices)
                    {
                        // Project either extrusion cap back onto the section.
                        const Eigen::Vector3d actual = wire_pose * Eigen::Vector3d(vertex.x, vertex.y, 0.0);
                        if ((actual - expected).norm() < 1e-5) found_vertex = true;
                    }
                    EXPECT_TRUE(found_vertex);
                }
            }
            EXPECT_TRUE(found_boundary);
            const Eigen::Vector3d expected_z = handedness * section.frame.linear().col(2).cast<double>();
            EXPECT_TRUE(wire_pose.linear().col(2).isApprox(expected_z, 1e-5));
        }
    }
}
