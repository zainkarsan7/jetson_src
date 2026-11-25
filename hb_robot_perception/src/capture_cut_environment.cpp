#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <hb_robot_interfaces/msg/task_state.hpp>
#include <hb_robot_interfaces/srv/capture_cut_environment.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace
{

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Transform = Eigen::Isometry3d;

using Cloud = sensor_msgs::msg::PointCloud2;
using TaskState = hb_robot_interfaces::msg::TaskState;
using Capture = hb_robot_interfaces::srv::CaptureCutEnvironment;
using Object = moveit_msgs::msg::CollisionObject;
using GetScene = moveit_msgs::srv::GetPlanningScene;
using ApplyScene = moveit_msgs::srv::ApplyPlanningScene;
using GetParameters = rcl_interfaces::srv::GetParameters;
using Trigger = std_srvs::srv::Trigger;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct ScopeExit
{
  std::function<void()> action;

  ~ScopeExit()
  {
    action();
  }
};

bool environmentObject(const Object & object)
{
  return object.id.rfind("cut_environment_", 0) == 0;
}

Transform makeTransform(
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  const Vec3 translation(x, y, z);
  Eigen::Quaterniond rotation(qw, qx, qy, qz);

  if (!translation.allFinite() ||
      !rotation.coeffs().allFinite() ||
      rotation.squaredNorm() < 1e-16)
  {
    throw std::runtime_error("Invalid pose or transform");
  }

  rotation.normalize();

  Transform result = Transform::Identity();
  result.linear() = rotation.toRotationMatrix();
  result.translation() = translation;
  return result;
}

Transform poseTransform(const geometry_msgs::msg::Pose & pose)
{
  return makeTransform(
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
}

geometry_msgs::msg::Pose makePose(
  const Vec3 & position,
  const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity())
{
  geometry_msgs::msg::Pose result;
  result.position.x = position.x();
  result.position.y = position.y();
  result.position.z = position.z();
  result.orientation.x = rotation.x();
  result.orientation.y = rotation.y();
  result.orientation.z = rotation.z();
  result.orientation.w = rotation.w();
  return result;
}

template<typename Vector>
double segmentDistanceSquared(
  const Vector & point,
  const Vector & start,
  const Vector & end)
{
  const Vector axis = end - start;
  const double denominator = axis.squaredNorm();
  const double fraction = denominator > 1e-16 ?
    std::clamp((point - start).dot(axis) / denominator, 0.0, 1.0) :
    0.0;

  return (point - start - fraction * axis).squaredNorm();
}

struct Capsule
{
  Vec3 start;
  Vec3 end;
  double radius;
};

struct CapsuleMask
{
  Vec3 start;
  Vec3 axis;
  Vec3 lower;
  Vec3 upper;
  double inverse_length_squared;
  double radius_squared;

  explicit CapsuleMask(const Capsule & capsule)
  : start(capsule.start),
    axis(capsule.end - capsule.start),
    lower(capsule.start.cwiseMin(capsule.end) -
      Vec3::Constant(capsule.radius)),
    upper(capsule.start.cwiseMax(capsule.end) +
      Vec3::Constant(capsule.radius)),
    inverse_length_squared(
      axis.squaredNorm() > 1e-16 ? 1.0 / axis.squaredNorm() : 0.0),
    radius_squared(capsule.radius * capsule.radius)
  {
  }

  bool contains(const Vec3 & point) const
  {
    if ((point.array() < lower.array()).any() ||
        (point.array() > upper.array()).any())
    {
      return false;
    }

    const Vec3 offset = point - start;
    const double fraction = std::clamp(
      offset.dot(axis) * inverse_length_squared, 0.0, 1.0);

    return (offset - fraction * axis).squaredNorm() <= radius_squared;
  }
};

Capsule fitCapsule(const std::vector<Vec3> & vertices)
{
  if (vertices.size() < 3) {
    throw std::runtime_error("Collision mesh has fewer than three vertices");
  }

  Vec3 center = Vec3::Zero();
  for (const auto & vertex : vertices) {
    if (!vertex.allFinite()) {
      throw std::runtime_error("Non-finite collision mesh vertex");
    }
    center += vertex;
  }
  center /= static_cast<double>(vertices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & vertex : vertices) {
    const Vec3 offset = vertex - center;
    covariance.noalias() += offset * offset.transpose();
  }
  covariance /= static_cast<double>(vertices.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Collision mesh PCA failed");
  }

  const Vec3 axis = solver.eigenvectors().col(2);
  double low = std::numeric_limits<double>::infinity();
  double high = -low;

  for (const auto & vertex : vertices) {
    const double projection = (vertex - center).dot(axis);
    low = std::min(low, projection);
    high = std::max(high, projection);
  }

  Capsule result{center + low * axis, center + high * axis, 0.0};
  double radius_squared = 0.0;

  for (const auto & vertex : vertices) {
    radius_squared = std::max(
      radius_squared,
      segmentDistanceSquared(vertex, result.start, result.end));
  }

  result.radius = std::sqrt(radius_squared);
  return result;
}

using LinkCapsules = std::map<std::string, std::vector<Capsule>>;

LinkCapsules loadCapsules(const std::string & description)
{
  urdf::Model model;
  if (!model.initString(description)) {
    throw std::runtime_error("Cannot parse robot_description");
  }

  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);

  LinkCapsules result;

  for (const auto & link : links) {
    if (link->name != "robot_mount" &&
        link->name.rfind("ur10e_", 0) != 0)
    {
      continue;
    }

    auto collisions = link->collision_array;
    if (collisions.empty() && link->collision) {
      collisions.push_back(link->collision);
    }

    for (const auto & collision : collisions) {
      if (!collision || !collision->geometry ||
          collision->geometry->type != urdf::Geometry::MESH)
      {
        throw std::runtime_error(
          "Expected collision mesh on link " + link->name);
      }

      const auto * mesh =
        dynamic_cast<const urdf::Mesh *>(collision->geometry.get());

      if (!mesh || mesh->filename.empty()) {
        throw std::runtime_error("Missing mesh on link " + link->name);
      }

      const Vec3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      if (!scale.allFinite() || (scale.array() == 0.0).any()) {
        throw std::runtime_error("Invalid mesh scale on " + link->name);
      }

      std::string resource = mesh->filename;
      if (!resource.empty() && resource.front() == '/') {
        resource = "file://" + resource;
      }

      // geometric_shapes supports package:// and file:// resources,
      // including binary and ASCII STL.
      std::unique_ptr<shapes::Mesh> loaded(
        shapes::createMeshFromResource(resource, scale));

      if (!loaded || loaded->vertex_count < 3) {
        throw std::runtime_error("Cannot load collision mesh " + resource);
      }

      const auto & origin = collision->origin;
      const Transform link_from_mesh = makeTransform(
        origin.position.x, origin.position.y, origin.position.z,
        origin.rotation.x, origin.rotation.y,
        origin.rotation.z, origin.rotation.w);

      std::vector<Vec3> vertices;
      vertices.reserve(loaded->vertex_count);

      for (unsigned int i = 0; i < loaded->vertex_count; ++i) {
        const Vec3 vertex(
          loaded->vertices[3 * i],
          loaded->vertices[3 * i + 1],
          loaded->vertices[3 * i + 2]);

        vertices.push_back(link_from_mesh * vertex);
      }

      result[link->name].push_back(fitCapsule(vertices));
    }
  }

  const std::array<const char *, 9> required{
    "robot_mount",
    "ur10e_shoulder_link",
    "ur10e_upper_arm_link",
    "ur10e_forearm_link",
    "ur10e_wrist_1_link",
    "ur10e_wrist_2_link",
    "ur10e_wrist_3_link",
    "ur10e_bracket_link",
    "ur10e_torch_link"
  };

  for (const auto * name : required) {
    if (result.find(name) == result.end()) {
      throw std::runtime_error(
          std::string("Missing robot collision mesh: ") + name);
    }
  }

  return result;
}

double cross2(const Vec2 & a, const Vec2 & b)
{
  return a.x() * b.y() - a.y() * b.x();
}

class Extrusion
{
public:
  Extrusion(const shape_msgs::msg::Mesh & mesh, double margin)
  : margin_(margin)
  {
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
      throw std::runtime_error("Empty workpiece mesh");
    }

    lower_ = Vec3::Constant(std::numeric_limits<double>::infinity());
    upper_ = -lower_;

    std::vector<Vec3> vertices;
    vertices.reserve(mesh.vertices.size());

    for (const auto & point : mesh.vertices) {
      const Vec3 vertex(point.x, point.y, point.z);
      if (!vertex.allFinite()) {
        throw std::runtime_error("Non-finite workpiece vertex");
      }
      vertices.push_back(vertex);
      lower_ = lower_.cwiseMin(vertex);
      upper_ = upper_.cwiseMax(vertex);
    }

    if (upper_.z() <= lower_.z()) {
      throw std::runtime_error("Workpiece has no extrusion depth");
    }

    for (const auto & vertex : vertices) {
      if (std::abs(vertex.z() - lower_.z()) > 1e-6 &&
          std::abs(vertex.z() - upper_.z()) > 1e-6)
      {
        throw std::runtime_error("Expected local-Z workpiece extrusion");
      }
    }

    for (const auto & face : mesh.triangles) {
      std::array<Vec2, 3> triangle;
      bool lower_cap = true;

      for (std::size_t i = 0; i < 3; ++i) {
        const auto index = face.vertex_indices[i];
        if (index >= vertices.size()) {
          throw std::runtime_error("Invalid workpiece triangle index");
        }

        const auto & vertex = vertices[index];
        lower_cap &= std::abs(vertex.z() - lower_.z()) <= 1e-6;
        triangle[i] = vertex.head<2>();
      }

      if (lower_cap &&
          std::abs(cross2(
            triangle[1] - triangle[0],
            triangle[2] - triangle[0])) > 1e-16)
      {
        cap_.push_back(triangle);
      }
    }

    if (cap_.empty()) {
      throw std::runtime_error("Workpiece has no valid lower cap");
    }
  }

  bool contains(const Vec3 & point) const
  {
    if ((point.array() < (lower_.array() - margin_)).any() ||
        (point.array() > (upper_.array() + margin_)).any())
    {
      return false;
    }

    const double axial = std::max({
      lower_.z() - point.z(), point.z() - upper_.z(), 0.0});

    const double radius_squared =
      std::max(0.0, margin_ * margin_ - axial * axial);

    const Vec2 xy = point.head<2>();

    for (const auto & triangle : cap_) {
      bool all_positive = true;
      bool all_negative = true;

      for (std::size_t i = 0; i < 3; ++i) {
        const Vec2 & a = triangle[i];
        const Vec2 & b = triangle[(i + 1) % 3];

        if (segmentDistanceSquared(xy, a, b) <= radius_squared) {
          return true;
        }

        const double side = cross2(b - a, xy - a);
        all_positive &= side >= 0.0;
        all_negative &= side <= 0.0;
      }

      if (all_positive || all_negative) {
        return true;
      }
    }

    return false;
  }

private:
  double margin_;
  Vec3 lower_;
  Vec3 upper_;
  std::vector<std::array<Vec2, 3>> cap_;
};

struct Cell
{
  int32_t x;
  int32_t y;
  int32_t z;

  bool operator==(const Cell & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator<(const Cell & other) const
  {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct CellHash
{
  std::size_t operator()(const Cell & cell) const noexcept
  {
    std::size_t hash = std::hash<int32_t>{}(cell.x);
    hash ^= std::hash<int32_t>{}(cell.y) +
      0x9e3779b9U + (hash << 6U) + (hash >> 2U);
    hash ^= std::hash<int32_t>{}(cell.z) +
      0x9e3779b9U + (hash << 6U) + (hash >> 2U);
    return hash;
  }
};

Cell cellForPoint(const Vec3 & point, double size)
{
  const Vec3 index = (point / size).array().floor().matrix();

  if (!index.allFinite() ||
      (index.array() < std::numeric_limits<int32_t>::min()).any() ||
      (index.array() > std::numeric_limits<int32_t>::max()).any())
  {
    throw std::runtime_error("Voxel coordinates exceed supported range");
  }

  return {
    static_cast<int32_t>(index.x()),
    static_cast<int32_t>(index.y()),
    static_cast<int32_t>(index.z())
  };
}

struct Vote
{
  uint32_t last_scan = std::numeric_limits<uint32_t>::max();
  uint32_t count = 0;
};

using Votes = std::unordered_map<Cell, Vote, CellHash>;
using Poses = std::map<std::string, Transform>;

uint32_t fieldOffset(const Cloud & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }

    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field.count != 1 ||
        static_cast<uint64_t>(field.offset) + 4 > cloud.point_step)
    {
      throw std::runtime_error("Expected valid FLOAT32 field " + name);
    }

    return field.offset;
  }

  throw std::runtime_error("Missing point cloud field " + name);
}

float readFloat(const uint8_t * data, bool big_endian)
{
  uint32_t bits;

  if (big_endian) {
    bits =
      (static_cast<uint32_t>(data[0]) << 24U) |
      (static_cast<uint32_t>(data[1]) << 16U) |
      (static_cast<uint32_t>(data[2]) << 8U) |
      static_cast<uint32_t>(data[3]);
  } else {
    bits =
      static_cast<uint32_t>(data[0]) |
      (static_cast<uint32_t>(data[1]) << 8U) |
      (static_cast<uint32_t>(data[2]) << 16U) |
      (static_cast<uint32_t>(data[3]) << 24U);
  }

  float result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

}  // namespace

class CaptureNode : public rclcpp::Node
{
public:
  CaptureNode()
  : Node("capture_cut_environment")
  {
    range_ = declare_parameter("lidar_max_range_m", 2.0);
    padding_ = declare_parameter("jacket_padding_m", 0.045);
    margin_ = declare_parameter("workpiece_margin_m", 0.025);
    voxel_size_ = declare_parameter("voxel_size_m", 0.04);
    duration_ = declare_parameter("capture_duration_s", 0.5);
    max_state_age_ = declare_parameter("max_state_age_s", 0.5);
    min_scans_ = declare_parameter("min_voxel_scans", 2);
    scan_lenience_ = declare_parameter("scan_lenience", 2);

    debug_invalid_scene_ =
      declare_parameter("debug_allow_invalid_scene", false);
    debug_only_ = declare_parameter("debug_publish_only", false);
    publish_markers_ = declare_parameter("publish_debug_markers", true);

    const auto description_node = declare_parameter<std::string>(
      "robot_description_node", "/robot_state_publisher");

    const int max_clouds = declare_parameter("max_capture_clouds", 512);
    const int max_megabytes = declare_parameter("max_capture_megabytes", 256);
    const int max_voxels = declare_parameter("max_candidate_voxels", 500000);

    const auto bounded = [](double value, double low, double high) {
      return std::isfinite(value) && value >= low && value <= high;
    };

    if (!bounded(range_, 0.1, 50.0) ||
        !bounded(padding_, 0.0, 0.25) ||
        !bounded(margin_, 0.0, 2.0) ||
        !bounded(voxel_size_, 0.01, 0.2) ||
        !bounded(duration_, 0.5, 30.0) ||
        !bounded(max_state_age_, 0.01, 30.0) ||
        min_scans_ < 1 || min_scans_ > max_clouds ||
        max_clouds < 5 || max_clouds > 4096 ||
        max_megabytes < 1 || max_megabytes > 2048 ||
        max_voxels < 1)
    {
      throw std::runtime_error("Invalid capture parameters");
    }

    max_clouds_ = static_cast<std::size_t>(max_clouds);
    max_bytes_ = static_cast<std::size_t>(max_megabytes) * 1024U * 1024U;
    max_voxels_ = static_cast<std::size_t>(max_voxels);

    // Client responses must remain executable while a service callback waits.
    rpc_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    get_scene_ = create_client<GetScene>(
      "/get_planning_scene", rmw_qos_profile_services_default, rpc_group_);
    apply_scene_ = create_client<ApplyScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, rpc_group_);

    std::string description_service = description_node;
    while (!description_service.empty() &&
           description_service.back() == '/')
    {
      description_service.pop_back();
    }
    description_service += "/get_parameters";

    description_ = create_client<GetParameters>(
      description_service, rmw_qos_profile_services_default, rpc_group_);

    // Retain TF for the longest supported capture plus processing time.
    tf_ = std::make_shared<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(60.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    state_sub_ = create_subscription<TaskState>(
      "/task_state",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](TaskState::ConstSharedPtr state) {
        std::lock_guard<std::mutex> guard(mutex_);
        state_ = std::move(state);

        if (active_ && !validLocked(request_)) {
          invalid_ = true;
          condition_.notify_all();
        }
      });

    marker_pub_ = create_publisher<MarkerArray>(
      "/cut_environment/debug_robot_masks",
      rclcpp::QoS(1).reliable().transient_local());

    capture_service_ = create_service<Capture>(
      "capture_cut_environment",
      [this](
        const std::shared_ptr<Capture::Request> request,
        std::shared_ptr<Capture::Response> response)
      {
        capture(*request, *response);
      },
      rmw_qos_profile_services_default,
      service_group_);

    clear_service_ = create_service<Trigger>(
      "clear_cut_environment",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
      {
        clear(*response);
      },
      rmw_qos_profile_services_default,
      service_group_);

    std::random_device random;
    std::ostringstream identifier;
    identifier << std::hex << random() << random() << random() << random();
    instance_id_ = identifier.str();

    RCLCPP_INFO(
      get_logger(),
      "Capture ready: %.2fs, %.2fm range, %.3fm voxels, %d scan votes",
      duration_, range_, voxel_size_, min_scans_);
  }

private:
  template<typename Service>
  typename Service::Response::SharedPtr rpc(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const typename Service::Request::SharedPtr & request)
  {
    if (!client->wait_for_service(1s)) {
      throw std::runtime_error(
          std::string("Service unavailable: ") + client->get_service_name());
    }

    auto future = client->async_send_request(request);

    if (future.wait_for(3s) != std::future_status::ready) {
      client->remove_pending_request(future);
      throw std::runtime_error(
          std::string("Service timed out; remote outcome may be unknown: ") +
          client->get_service_name());
    }

    auto response = future.get();
    if (!response) {
      throw std::runtime_error("Service returned no response");
    }
    return response;
  }

  bool validLocked(const Capture::Request & request) const
  {
    if (!state_) {
      return false;
    }

    const int64_t age = now().nanoseconds() -
      rclcpp::Time(state_->header.stamp, get_clock()->get_clock_type())
      .nanoseconds();

    return
      age >= 0 &&
      age <= static_cast<int64_t>(max_state_age_ * 1e9) &&
      (state_->scene_valid || debug_invalid_scene_) &&
      state_->imu_stable &&
      state_->supervision_valid &&
      !state_->motion_fault_active &&
      !state_->process_fault_active &&
      state_->geometry_version == request.geometry_version &&
      state_->pose_version == request.pose_version &&
      state_->scene_version == request.scene_version;
  }

  void requireValid()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (invalid_ || !validLocked(request_)) {
      throw std::runtime_error(
          "Capture invalidated by stale state, changed versions or instability");
    }
  }

  void initializeCapsules()
  {
    if (!capsules_.empty()) {
      return;
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names = {"robot_description"};
    const auto response = rpc<GetParameters>(description_, request);

    if (response->values.size() != 1 ||
        response->values.front().type !=
          rcl_interfaces::msg::ParameterType::PARAMETER_STRING ||
        response->values.front().string_value.empty())
    {
      throw std::runtime_error("robot_description is unavailable");
    }

    // Assign only after every required collision mesh has been validated.
    auto loaded = loadCapsules(response->values.front().string_value);

    std::size_t count = 0;
    for (const auto & entry : loaded) {
      count += entry.second.size();
    }

    capsules_ = std::move(loaded);
    RCLCPP_INFO(
      get_logger(), "Cached %zu mesh capsules across %zu links",
      count, capsules_.size());
  }

  std::vector<Object> sceneObjects()
  {
    auto request = std::make_shared<GetScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    return rpc<GetScene>(get_scene_, request)->scene.world.collision_objects;
  }

  void cloud(Cloud::ConstSharedPtr message)
  {
    const int64_t stamp =
      rclcpp::Time(message->header.stamp, get_clock()->get_clock_type())
      .nanoseconds();

    std::lock_guard<std::mutex> guard(mutex_);
    if (!active_) {
      return;
    }

    // Wait for actual fresh delivery before opening the timed window.
    if (!collecting_) {
      const int64_t age = now().nanoseconds() - stamp;
      if (age >= 0 && age <= 500000000LL) {
        cloud_ready_ = true;
        condition_.notify_all();
      }
      return;
    }

    if (stamp < start_ns_ || stamp >= end_ns_ ||
        scans_.find(stamp) != scans_.end())
    {
      return;
    }

    if (scans_.size() >= max_clouds_ ||
        message->data.size() > max_bytes_ - captured_bytes_)
    {
      overflow_ = true;
      condition_.notify_all();
      return;
    }

    captured_bytes_ += message->data.size();
    scans_.emplace(stamp, std::move(message));
  }

  std::map<int64_t, Cloud::ConstSharedPtr> acquire(
    const Capture::Request & request)
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);

      if (!validLocked(request)) {
        throw std::runtime_error("Invalid task state before acquisition");
      }

      request_ = request;
      active_ = true;
      collecting_ = false;
      invalid_ = false;
      overflow_ = false;
      cloud_ready_ = false;
      captured_bytes_ = 0;
      scans_.clear();
    }

    cloud_sub_ = create_subscription<Cloud>(
      "/hb_robot_lidar/cloud",
      rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr message) {
        cloud(std::move(message));
      });

    std::map<int64_t, Cloud::ConstSharedPtr> result;

    {
      std::unique_lock<std::mutex> guard(mutex_);

      condition_.wait_for(guard, 2s, [this]() {
        return cloud_ready_ || invalid_;
      });

      if (!cloud_ready_ || invalid_ || !validLocked(request)) {
        throw std::runtime_error(
            "No fresh LiDAR stream or invalid task state");
      }

      start_ns_ = now().nanoseconds();
      end_ns_ = start_ns_ + static_cast<int64_t>(duration_ * 1e9);
      collecting_ = true;

      // Timestamp admission remains limited to capture_duration_s.
      condition_.wait_for(
        guard,
        std::chrono::duration<double>(duration_ + 0.15),
        [this]() { return invalid_ || overflow_; });

      collecting_ = false;

      if (overflow_) {
        throw std::runtime_error("Capture exceeded its memory/cloud limit");
      }
      if (invalid_ || !validLocked(request)) {
        throw std::runtime_error("Task state invalidated during acquisition");
      }

      result.swap(scans_);
    }

    // Stop cloud delivery and deserialization during filtering and idle time.
    cloud_sub_.reset();

    const double span = result.size() > 1 ?
      static_cast<double>(result.rbegin()->first - result.begin()->first) / 1e9 :
      0.0;

    // At 12 Hz this permits approximately two missing scans.
    const std::size_t required_count = static_cast<std::size_t>(
      std::max({5, min_scans_, static_cast<int>(std::floor(duration_ * 12.0)) - scan_lenience_}));

    const double required_span = std::max(0.33, duration_ - 0.20);

    if (result.size() < required_count || span < required_span) {
      throw std::runtime_error(
          "Insufficient LiDAR coverage: " + std::to_string(result.size()) +
          " scans, span " + std::to_string(span) + " seconds");
    }

    return result;
  }

  void finishCapture()
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      active_ = false;
      collecting_ = false;
      scans_.clear();
      captured_bytes_ = 0;
    }
    cloud_sub_.reset();
  }

  Poses scanPoses(
    const std::string & frame,
    const Cloud & scan)
  {
    if (scan.header.frame_id.empty()) {
      throw std::runtime_error("LiDAR cloud has no frame");
    }

    std::vector<std::string> links;
    links.reserve(capsules_.size() + 2);
    for (const auto & entry : capsules_) {
      links.push_back(entry.first);
    }
    links.push_back("camera_base");
    links.push_back(scan.header.frame_id);

    std::sort(links.begin(), links.end());
    links.erase(std::unique(links.begin(), links.end()), links.end());

    const rclcpp::Time stamp(
      scan.header.stamp, get_clock()->get_clock_type());

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    std::string last_error;

    while (rclcpp::ok()) {
      requireValid();

      Poses result;
      try {
        for (const auto & link : links) {
          if (link == frame) {
            result.emplace(link, Transform::Identity());
            continue;
          }

          const auto message = tf_->lookupTransform(
            frame, link, stamp, rclcpp::Duration::from_seconds(0.0));

          const auto & transform = message.transform;
          result.emplace(link, makeTransform(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w));
        }
        return result;
      } catch (const tf2::TransformException & error) {
        last_error = error.what();
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(25ms);
    }

    throw std::runtime_error("TF unavailable for captured scan: " + last_error);
  }

  void checkStability(const Poses & poses, Poses & initial)
  {
    if (initial.empty()) {
      initial = poses;
      return;
    }

    if (poses.size() != initial.size()) {
      throw std::runtime_error("Frame set changed during acquisition");
    }

    for (const auto & entry : poses) {
      const auto previous = initial.find(entry.first);
      if (previous == initial.end()) {
        throw std::runtime_error("LiDAR frame changed during acquisition");
      }

      const auto & first = previous->second;
      const auto & current = entry.second;
      const double translation =
        (first.translation() - current.translation()).norm();
      const double angle = Eigen::AngleAxisd(
        first.linear().transpose() * current.linear()).angle();

      if (translation > 0.01 || angle > 0.02) {
        throw std::runtime_error(
            "Robot moved during capture: " + entry.first);
      }
    }
  }

  void filterScan(
    const Cloud & scan,
    uint32_t scan_index,
    const Poses & poses,
    const Transform & mesh_from_world,
    const Extrusion & profile,
    Votes & votes)
  {
    const uint32_t x_offset = fieldOffset(scan, "x");
    const uint32_t y_offset = fieldOffset(scan, "y");
    const uint32_t z_offset = fieldOffset(scan, "z");

    const uint64_t row_bytes =
      static_cast<uint64_t>(scan.width) * scan.point_step;

    const uint64_t required_bytes = scan.height == 0 ? 0 :
      static_cast<uint64_t>(scan.height - 1) * scan.row_step + row_bytes;

    if (scan.point_step == 0 || row_bytes > scan.row_step ||
        required_bytes > scan.data.size())
    {
      throw std::runtime_error("Malformed PointCloud2 layout");
    }

    std::vector<CapsuleMask> masks;
    for (const auto & entry : capsules_) {
      const Transform & world_from_link = poses.at(entry.first);
      for (const auto & capsule : entry.second) {
        masks.emplace_back(Capsule{
          world_from_link * capsule.start,
          world_from_link * capsule.end,
          capsule.radius + padding_
        });
      }
    }

    const Transform & world_from_lidar = poses.at(scan.header.frame_id);
    const Transform camera_from_world = poses.at("camera_base").inverse();
    const Vec3 camera_half_extents =
      Vec3(0.12, 0.09, 0.07) + Vec3::Constant(padding_);
    const double range_squared = range_ * range_;

    bool any_valid_return = false;

    for (uint32_t row = 0; row < scan.height; ++row) {
      const uint8_t * row_data =
        scan.data.data() + static_cast<std::size_t>(row) * scan.row_step;

      for (uint32_t column = 0; column < scan.width; ++column) {
        const uint8_t * data =
          row_data + static_cast<std::size_t>(column) * scan.point_step;

        const Vec3 local(
          readFloat(data + x_offset, scan.is_bigendian),
          readFloat(data + y_offset, scan.is_bigendian),
          readFloat(data + z_offset, scan.is_bigendian));

        if (!local.allFinite()) {
          continue;
        }

        const double distance_squared = local.squaredNorm();
        if (distance_squared <= 0.0) {
          continue;
        }

        any_valid_return = true;
        if (distance_squared > range_squared) {
          continue;
        }

        const Vec3 point = world_from_lidar * local;

        bool masked = false;
        for (const auto & mask : masks) {
          if (mask.contains(point)) {
            masked = true;
            break;
          }
        }
        if (masked) {
          continue;
        }

        const Vec3 camera_point = camera_from_world * point;
        if ((camera_point.cwiseAbs().array() <=
             camera_half_extents.array()).all())
        {
          continue;
        }

        if (profile.contains(mesh_from_world * point)) {
          continue;
        }

        const Cell cell = cellForPoint(point, voxel_size_);
        auto found = votes.find(cell);

        if (found == votes.end()) {
          if (votes.size() >= max_voxels_) {
            throw std::runtime_error("Candidate voxel limit exceeded");
          }
          found = votes.emplace(cell, Vote{}).first;
        }

        auto & vote = found->second;
        if (vote.last_scan != scan_index) {
          vote.last_scan = scan_index;
          ++vote.count;
        }
      }
    }

    if (!any_valid_return) {
      throw std::runtime_error("Cloud contains no valid nonzero returns");
    }
  }

  void publishMarkers(
    const Poses & poses,
    const std::string & frame,
    const std::vector<Cell> & cells)
  {
    if (!publish_markers_) {
      return;
    }

    MarkerArray array;
    Marker erase;
    erase.action = Marker::DELETEALL;
    array.markers.push_back(erase);

    int32_t identifier = 0;

    const auto add = [&](
      int32_t type,
      const std::string & name,
      const Vec3 & position,
      const Eigen::Quaterniond & rotation,
      const Vec3 & scale)
    {
      Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = now();
      marker.ns = name;
      marker.id = identifier++;
      marker.type = type;
      marker.action = Marker::ADD;
      marker.pose = makePose(position, rotation);
      marker.scale.x = scale.x();
      marker.scale.y = scale.y();
      marker.scale.z = scale.z();
      marker.color.g = 0.8F;
      marker.color.b = 1.0F;
      marker.color.a = 0.25F;
      array.markers.push_back(std::move(marker));
    };

    for (const auto & entry : capsules_) {
      const Transform & world_from_link = poses.at(entry.first);

      for (const auto & capsule : entry.second) {
        const Vec3 start = world_from_link * capsule.start;
        const Vec3 end = world_from_link * capsule.end;
        const Vec3 axis = end - start;
        const double length = axis.norm();
        const double diameter = 2.0 * (capsule.radius + padding_);

        if (length > 1e-8) {
          const Eigen::Quaterniond rotation =
            Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), axis);

          add(
            Marker::CYLINDER, entry.first, (start + end) * 0.5,
            rotation, Vec3(diameter, diameter, length));
        }

        add(
          Marker::SPHERE, entry.first, start,
          Eigen::Quaterniond::Identity(), Vec3::Constant(diameter));
        add(
          Marker::SPHERE, entry.first, end,
          Eigen::Quaterniond::Identity(), Vec3::Constant(diameter));
      }
    }

    if (!cells.empty()) {
      Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = now();
      marker.ns = "environment_voxels";
      marker.id = 0;
      marker.type = Marker::CUBE_LIST;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = voxel_size_;
      marker.scale.y = voxel_size_;
      marker.scale.z = voxel_size_;
      marker.color.r = 1.0F;
      marker.color.g = 0.5F;
      marker.color.a = 0.5F;
      marker.points.reserve(cells.size());

      for (const auto & cell : cells) {
        geometry_msgs::msg::Point point;
        point.x = (static_cast<double>(cell.x) + 0.5) * voxel_size_;
        point.y = (static_cast<double>(cell.y) + 0.5) * voxel_size_;
        point.z = (static_cast<double>(cell.z) + 0.5) * voxel_size_;
        marker.points.push_back(point);
      }

      array.markers.push_back(std::move(marker));
    }

    marker_pub_->publish(array);
  }

  Object voxelObject(
    const std::vector<Cell> & cells,
    const std::string & frame)
  {
    Object object;
    object.id = "cut_environment_" + instance_id_ + "_" +
      std::to_string(++sequence_);
    object.header.frame_id = frame;
    object.header.stamp = now();
    object.pose.orientation.w = 1.0;
    object.operation = Object::ADD;

    for (std::size_t begin = 0; begin < cells.size();) {
      std::size_t end = begin + 1;

      while (end < cells.size() &&
             cells[end].x == cells[begin].x &&
             cells[end].y == cells[begin].y &&
             static_cast<int64_t>(cells[end].z) ==
               static_cast<int64_t>(cells[end - 1].z) + 1)
      {
        ++end;
      }

      const double length = static_cast<double>(end - begin);

      const Vec3 center(
        (static_cast<double>(cells[begin].x) + 0.5) * voxel_size_,
        (static_cast<double>(cells[begin].y) + 0.5) * voxel_size_,
        (static_cast<double>(cells[begin].z) + length * 0.5) * voxel_size_);

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;

      // BOX dimensions are full side lengths.
      box.dimensions = {
        voxel_size_, voxel_size_, voxel_size_ * length
      };

      object.primitives.push_back(std::move(box));
      object.primitive_poses.push_back(makePose(center));
      begin = end;
    }

    return object;
  }

  std::shared_ptr<ApplyScene::Request> removalDiff(
    const std::vector<Object> & objects)
  {
    auto request = std::make_shared<ApplyScene::Request>();
    request->scene.is_diff = true;
    request->scene.robot_state.is_diff = true;

    for (const auto & object : objects) {
      if (!environmentObject(object)) {
        continue;
      }

      Object remove;
      remove.id = object.id;
      remove.header = object.header;
      remove.operation = Object::REMOVE;
      request->scene.world.collision_objects.push_back(std::move(remove));
    }

    return request;
  }

  void capture(
    const Capture::Request & request,
    Capture::Response & response)
  {
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock()) {
      response.message = "Environment capture/clear is busy";
      return;
    }

    ScopeExit cleanup{[this]() { finishCapture(); }};

    try {
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!validLocked(request)) {
          throw std::runtime_error("Invalid task state");
        }
      }

      initializeCapsules();
      const auto objects = sceneObjects();

      const auto work = std::find_if(
        objects.begin(), objects.end(),
        [](const Object & object) {
          return object.id == "wk_workpiece_collision";
        });

      if (work == objects.end() ||
          work->header.frame_id.empty() ||
          work->meshes.size() != 1 ||
          work->mesh_poses.size() != 1)
      {
        throw std::runtime_error("Missing accepted workpiece extrusion");
      }

      const std::string frame = work->header.frame_id;
      const Extrusion profile(work->meshes.front(), margin_);
      const Transform mesh_from_world =
        (poseTransform(work->pose) *
         poseTransform(work->mesh_poses.front())).inverse();

      auto scans = acquire(request);
      const std::size_t scan_count = scans.size();

      const auto processing_start = std::chrono::steady_clock::now();

      Poses initial;
      Votes votes;
      votes.reserve(std::min<std::size_t>(max_voxels_, 32768));

      uint32_t scan_index = 0;
      for (auto & entry : scans) {
        requireValid();

        const auto poses = scanPoses(frame, *entry.second);
        checkStability(poses, initial);
        filterScan(
          *entry.second, scan_index++, poses,
          mesh_from_world, profile, votes);

        // Release each raw cloud as soon as processing completes.
        entry.second.reset();
      }

      std::vector<Cell> occupied;
      occupied.reserve(votes.size());

      for (const auto & entry : votes) {
        if (entry.second.count >= static_cast<uint32_t>(min_scans_)) {
          occupied.push_back(entry.first);
        }
      }

      // Only accepted voxel keys need sorting for Z-run coalescing.
      std::sort(occupied.begin(), occupied.end());

      requireValid();
      publishMarkers(initial, frame, occupied);

      response.occupied_voxels = static_cast<uint32_t>(occupied.size());

      if (debug_only_) {
        response.success = false;
        response.scene_changed = false;
        response.message =
          "Debug preview published; capture was not applied to MoveIt";
        return;
      }

      auto apply = removalDiff(objects);
      std::string environment_id;

      if (!occupied.empty()) {
        auto object = voxelObject(occupied, frame);
        environment_id = object.id;
        apply->scene.world.collision_objects.push_back(std::move(object));
      }

      requireValid();

      const bool changed = !apply->scene.world.collision_objects.empty();
      if (changed && !rpc<ApplyScene>(apply_scene_, apply)->success) {
        throw std::runtime_error("MoveIt rejected environment update");
      }

      response.environment_id = environment_id;
      response.scene_changed = changed;
      response.success = true;
      response.message =
        std::to_string(scan_count) + " scans, " +
        std::to_string(occupied.size()) + " occupied voxels";

      const double processing_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - processing_start).count();

      RCLCPP_INFO(
        get_logger(), "%s; processing/apply %.3fs",
        response.message.c_str(), processing_seconds);
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
      RCLCPP_ERROR(get_logger(), "Capture failed: %s", error.what());
    }
  }

  void clear(Trigger::Response & response)
  {
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock()) {
      response.message = "Environment capture/clear is busy";
      return;
    }

    try {
      const auto objects = sceneObjects();
      auto apply = removalDiff(objects);
      const auto count = apply->scene.world.collision_objects.size();

      if (count != 0 && !rpc<ApplyScene>(apply_scene_, apply)->success) {
        throw std::runtime_error("MoveIt rejected environment removal");
      }

      MarkerArray markers;
      Marker erase;
      erase.action = Marker::DELETEALL;
      markers.markers.push_back(erase);
      marker_pub_->publish(markers);

      response.success = true;
      response.message =
        "Removed " + std::to_string(count) + " environment objects";
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
    }
  }

  double range_;
  double padding_;
  double margin_;
  double voxel_size_;
  double duration_;
  double max_state_age_;
  int min_scans_;
  int scan_lenience_;
  bool debug_invalid_scene_;
  bool debug_only_;
  bool publish_markers_;

  std::size_t max_clouds_;
  std::size_t max_bytes_;
  std::size_t max_voxels_;

  std::mutex operation_mutex_;
  std::mutex mutex_;
  std::condition_variable condition_;

  TaskState::ConstSharedPtr state_;
  Capture::Request request_;

  bool active_ = false;
  bool collecting_ = false;
  bool invalid_ = false;
  bool overflow_ = false;
  bool cloud_ready_ = false;

  int64_t start_ns_ = 0;
  int64_t end_ns_ = 0;
  std::size_t captured_bytes_ = 0;

  std::map<int64_t, Cloud::ConstSharedPtr> scans_;
  LinkCapsules capsules_;
  std::string instance_id_;
  uint64_t sequence_ = 0;

  rclcpp::CallbackGroup::SharedPtr rpc_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::Client<GetScene>::SharedPtr get_scene_;
  rclcpp::Client<ApplyScene>::SharedPtr apply_scene_;
  rclcpp::Client<GetParameters>::SharedPtr description_;

  rclcpp::Subscription<TaskState>::SharedPtr state_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Service<Capture>::SharedPtr capture_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CaptureNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3);

    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("capture_cut_environment"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}