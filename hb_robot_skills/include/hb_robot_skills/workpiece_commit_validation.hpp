
#pragma once

#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <rclcpp/time.hpp>
#include <cstdint>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <string>

namespace hb_robot_skills::workpiece {

using Reference =
    hb_robot_interfaces::msg::WorkpieceReference;

using CollisionObject =
    moveit_msgs::msg::CollisionObject;

inline constexpr const char* kObjectId =
    "wk_workpiece_collision";

inline bool rigid(const Eigen::Isometry3d& t)
{
    if (!t.matrix().allFinite())
        return false;

    const auto& r = t.linear();

    return std::abs(r.determinant() - 1.0) < 1e-4 &&
           (r.transpose() * r).isApprox(
               Eigen::Matrix3d::Identity(), 1e-4);
}

inline bool validMesh(const CollisionObject& object)
{
    return object.id == kObjectId &&
           object.meshes.size() == 1 &&
           object.mesh_poses.size() == 1 &&
           !object.meshes.front().vertices.empty() &&
           !object.meshes.front().triangles.empty() &&
           object.primitives.empty() &&
           object.primitive_poses.empty();
}

inline bool sameGeometry(
    const CollisionObject& a,
    const CollisionObject& b)
{
    if (!validMesh(a) || !validMesh(b))
        return false;

    const auto& ma = a.meshes.front();
    const auto& mb = b.meshes.front();

    if (ma.vertices.size() != mb.vertices.size() ||
        ma.triangles.size() != mb.triangles.size())
        return false;

    for (std::size_t i = 0;
         i < ma.vertices.size(); ++i)
    {
        const auto& x = ma.vertices[i];
        const auto& y = mb.vertices[i];

        if (x.x != y.x || x.y != y.y || x.z != y.z)
            return false;
    }

    for (std::size_t i = 0;
         i < ma.triangles.size(); ++i)
    {
        if (ma.triangles[i].vertex_indices !=
            mb.triangles[i].vertex_indices)
            return false;
    }

    return true;
}

inline Eigen::Isometry3d meshPose(
    const CollisionObject& object)
{
    Eigen::Isometry3d outer =
        Eigen::Isometry3d::Identity();

    Eigen::Isometry3d inner =
        Eigen::Isometry3d::Identity();

    tf2::fromMsg(object.pose, outer);
    tf2::fromMsg(object.mesh_poses.front(), inner);

    return outer * inner;
}

inline bool poseMatches(
    const Eigen::Isometry3d& a,
    const Eigen::Isometry3d& b,
    double position_tolerance = 1e-4,
    double rotation_tolerance = 1e-4)
{
    if (!rigid(a) || !rigid(b))
        return false;

    const double position_error =
        (a.translation() - b.translation()).norm();

    const double angle =
        std::abs(Eigen::AngleAxisd(
            a.linear() * b.linear().transpose()).angle());

    return position_error <= position_tolerance &&
           angle <= rotation_tolerance;
}

inline bool consistent(
    const CollisionObject& object,
    const Reference& reference,
    std::string& error)
{
    if (!validMesh(object))
    {
        error = "Invalid workpiece extrusion mesh";
        return false;
    }

    if (object.header.frame_id.empty() ||
        object.header.frame_id !=
            reference.estimate.header.frame_id ||
        reference.cloud.header.frame_id !=
            reference.estimate.header.frame_id)
    {
        error = "Workpiece reference frame mismatch";
        return false;
    }

    Eigen::Isometry3d estimate =
        Eigen::Isometry3d::Identity();

    tf2::fromMsg(reference.estimate.pose, estimate);

    if (!poseMatches(meshPose(object), estimate))
    {
        error = "Reference estimate and mesh pose disagree";
        return false;
    }

    return true;
}

inline std::string referenceId(
    const Reference& reference)
{
    return std::to_string(
        rclcpp::Time(
            reference.estimate.header.stamp)
            .nanoseconds());
}

}  // namespace hb_robot_skills::workpiece
