#include "environment_filter.hpp"
#include <geometric_shapes/mesh_operations.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

namespace cut_environment {
namespace {
constexpr double spacing = 0.012;
template<class V>
double segmentDistance2(const V& p, const V& a, const V& b) {
  const V axis = b-a;
  const double t = std::clamp((p-a).dot(axis) / std::max(1e-16, axis.squaredNorm()), 0., 1.);
  return (p-a-t*axis).squaredNorm();
}
std::array<unsigned, 3> xyzOffsets(const Cloud& cloud) {
  std::array<unsigned, 3> offsets{};
  for (unsigned i=0; i<3; ++i) {
    const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&](const auto& f){ return f.name == std::string(1, "xyz"[i]); });
    if (field == cloud.fields.end() || field->datatype != field->FLOAT32 || field->count != 1 ||
        cloud.point_step < 4 || field->offset > cloud.point_step-4)
      throw std::runtime_error("Expected FLOAT32 XYZ");
    offsets[i] = field->offset;
  }
  if (uint64_t(cloud.point_step)*cloud.width > cloud.row_step ||
      uint64_t(cloud.row_step)*cloud.height > cloud.data.size())
    throw std::runtime_error("Invalid PointCloud2 strides");
  return offsets;
}
}

Surface::Surface(const std::string& file) {
  const std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
      "package://hb_robot_description/meshes/" + file));
  if (!mesh || !mesh->triangle_count) throw std::runtime_error("Missing mask surface " + file);
  auto samples = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (unsigned t=0; t<mesh->triangle_count; ++t) {
    auto vertex = [&](unsigned corner) -> Vec {
      return Eigen::Map<const Vec>(mesh->vertices + 3*mesh->triangles[3*t+corner]);
    };
    const Vec a=vertex(0), b=vertex(1), c=vertex(2);
    bounds_.extend(a); bounds_.extend(b); bounds_.extend(c);
    const int n=std::max(1, int(std::ceil(std::max({(a-b).norm(), (b-c).norm(), (a-c).norm()})/spacing)));
    for (int i=0; i<=n; ++i)
      for (int j=0; j<=n-i; ++j) {
        const Vec p = a + (double(i)/n)*(b-a) + (double(j)/n)*(c-a);
        samples->emplace_back(p.x(), p.y(), p.z());
      }
  }
  tree_.setInputCloud(samples);
}
bool Surface::contains(const Vec& p, double padding) const {
  const double radius = padding + spacing;
  if (bounds_.squaredExteriorDistance(p) > radius*radius) return false;
  pcl::Indices ids;
  std::vector<float> distances;
  return tree_.radiusSearch(pcl::PointXYZ(p.x(),p.y(),p.z()), radius, ids, distances, 1) > 0;
}

Profile::Profile(const shape_msgs::msg::Mesh& mesh, double margin)
    : margin_(margin), low_(INFINITY), high_(-INFINITY) {
  for (const auto& p : mesh.vertices) {
    if (!Vec(p.x,p.y,p.z).allFinite()) throw std::runtime_error("Non-finite workpiece");
    low_=std::min(low_,p.z); high_=std::max(high_,p.z);
  }
  if (!(low_<high_)) throw std::runtime_error("Invalid workpiece extrusion");
  for (const auto& p : mesh.vertices)
    if (std::min(p.z-low_,high_-p.z)>1e-6) throw std::runtime_error("Expected local-Z extrusion");
  for (const auto& tri : mesh.triangles) {
    std::array<Eigen::Vector2d,3> xy;
    bool lower=true;
    for (unsigned i=0; i<3; ++i) {
      const auto& p=mesh.vertices.at(tri.vertex_indices[i]);
      xy[i]={p.x,p.y};
      lower &= (p.z-low_ < 1e-6);
    }
    if (lower) cap_.push_back(xy);
  }
  if (cap_.empty()) throw std::runtime_error("Missing workpiece end cap");
}
bool Profile::contains(const Vec& p) const {
  const double dz=std::max({0.,low_-p.z(),p.z()-high_});
  if (dz>margin_) return false;
  const Eigen::Vector2d xy=p.head<2>();
  for (const auto& tri : cap_) {
    bool positive=false, negative=false;
    for (unsigned i=0; i<3; ++i) {
      const auto& a=tri[i]; const auto& b=tri[(i+1)%3];
      if (segmentDistance2(xy,a,b) <= margin_*margin_-dz*dz) return true;
      const double cross=(b.x()-a.x())*(xy.y()-a.y())-(b.y()-a.y())*(xy.x()-a.x());
      positive |= cross>0; negative |= cross<0;
    }
    if (!positive || !negative) return true;
  }
  return false;
}

void Filter::add(const Cloud& cloud, const Poses& links, const Pose& world_to_mesh,
                 const Profile& profile, double jacket, double size, Votes& votes) const {
  const auto offsets=xyzOffsets(cloud);
  const uint16_t one=1;
  const bool swap=cloud.is_bigendian != (*reinterpret_cast<const uint8_t*>(&one)==0);
  const Pose mount=links[0].inverse(), bracket=links[8].inverse(),
             torch=links[9].inverse(), camera=links[10].inverse();
  const std::array<double,7> radii{.10,.11,.11,.09,.08,.08,.07};
  std::set<Cell> observed;
  bool has_returns=false;
  for (unsigned y=0; y<cloud.height; ++y) {
    for (unsigned x=0; x<cloud.width; ++x) {
      const auto* data=cloud.data.data()+size_t(y)*cloud.row_step+size_t(x)*cloud.point_step;
      Vec sensor;
      for (unsigned i=0; i<3; ++i) {
        uint32_t bits;
        std::memcpy(&bits,data+offsets[i],4);
        if (swap) bits=__builtin_bswap32(bits);
        float value;
        std::memcpy(&value,&bits,4);
        sensor[i]=value;
      }
      if (!sensor.allFinite() || sensor.squaredNorm()==0.) continue;
      has_returns=true;
      if (sensor.squaredNorm()>4.) continue;
      const Vec p=links.back()*sensor;
      bool robot=false;
      for (unsigned i=0; i<radii.size(); ++i) {
        const double r=radii[i]+jacket;
        if (segmentDistance2<Vec>(p,links[i].translation(),links[i+1].translation())<=r*r) {
          robot=true;
          break;
        }
      }
      if (robot) continue;
      const Vec m=mount*p;
      if (mount_.contains({m.x(),m.z(),-m.y()},.015) || bracket_.contains(bracket*p,jacket)) continue;
      if (segmentDistance2<Vec>(torch*p,Vec::Zero(),Vec(0,0,.40))<=std::pow(.018+jacket,2)) continue;
      if (((camera*p).array().abs() <= Vec(.12,.09,.07).array()+jacket).all()) continue;
      if (profile.contains(world_to_mesh*p)) continue;
      const Vec cell=(p/size).array().floor();
      if (cell.minCoeff()<INT32_MIN || cell.maxCoeff()>INT32_MAX) throw std::runtime_error("Invalid voxel coordinates");
      observed.insert({int(cell.x()),int(cell.y()),int(cell.z())});
    }
  }
  if (!has_returns) throw std::runtime_error("Cloud contains no valid returns");
  for (const auto& cell : observed) ++votes[cell];
}
}
