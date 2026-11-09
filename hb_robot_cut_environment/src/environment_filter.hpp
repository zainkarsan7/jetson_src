#pragma once
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <shape_msgs/msg/mesh.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <map>

namespace cut_environment {
using Vec = Eigen::Vector3d;
using Pose = Eigen::Isometry3d;
using Cell = std::array<int, 3>;
using Votes = std::map<Cell, unsigned>;
using Cloud = sensor_msgs::msg::PointCloud2;
// Arm joints in chain order, followed by bracket, torch, camera and sensor.
using Poses = std::array<Pose, 12>;
class Surface {
 public:
  explicit Surface(const std::string& mesh);
  bool contains(const Vec& point, double margin) const;
 private:
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  Eigen::AlignedBox3d bounds_;
};
class Profile {
 public:
  Profile(const shape_msgs::msg::Mesh& mesh, double margin);
  bool contains(const Vec& point) const;
 private:
  double margin_, low_, high_;
  std::vector<std::array<Eigen::Vector2d, 3>> cap_;
};
class Filter {
 public:
  void add(const Cloud&, const Poses&, const Pose& world_to_mesh, const Profile&,
           double jacket, double size, Votes&) const;
 private:
  Surface mount_{"base_mount.stl"}, bracket_{"bracket.stl"};
};
}
