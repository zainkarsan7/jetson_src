
#include <hb_robot_interfaces/action/revalidate_workpiece.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <hb_robot_perception/observation_registrar.hpp>
#include <hb_robot_perception/perception_types.hpp>

#include "hb_robot_skills/platform_supervision_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

using Clock = std::chrono::steady_clock;
using Cloud = hb_perception::PointCloud;
using Point = hb_perception::PointT;
using CollisionObject = moveit_msgs::msg::CollisionObject;
using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kWorkpieceId = "wk_workpiece_collision";

double toDegrees(double radians) {
    return radians * 180.0 / kPi;
}

double rmsMm(double fitness) {
    if (!std::isfinite(fitness) || fitness < 0.0)
        return std::numeric_limits<double>::infinity();

    return 1000.0 * std::sqrt(fitness);
}

bool finiteRigid(const Eigen::Isometry3d& pose) {
    if (!pose.matrix().allFinite())
        return false;

    const auto& rotation = pose.linear();

    return std::abs(rotation.determinant() - 1.0) < 1e-3 &&
           (rotation.transpose() * rotation)
               .isApprox(Eigen::Matrix3d::Identity(), 1e-3);
}

double rotationDifference(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b) {

    return std::abs(
        Eigen::AngleAxisd(a * b.transpose()).angle());
}

Cloud voxelize(const Cloud& input, float resolution) {
    Cloud output;

    Cloud::Ptr finite(new Cloud);

    for (const auto& point : input) {
        if (pcl::isFinite(point))
            finite->push_back(point);
    }

    if (finite->empty())
        return output;

    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(
        resolution, resolution, resolution);
    filter.setInputCloud(finite);
    filter.filter(output);

    return output;
}

struct StoredWorkpiece {
    CollisionObject object;

    Eigen::Isometry3d object_pose =
        Eigen::Isometry3d::Identity();

    Eigen::Isometry3d mesh_pose =
        Eigen::Isometry3d::Identity();

    Eigen::Isometry3d world_from_mesh =
        Eigen::Isometry3d::Identity();
};

std::optional<StoredWorkpiece> lookupWorkpiece(
    PlanningSceneInterface& scene,
    const std::string& frame) {

    const auto objects =
        scene.getObjects({kWorkpieceId});

    const auto it = objects.find(kWorkpieceId);

    if (it == objects.end())
        return std::nullopt;

    const auto& object = it->second;

    if (object.header.frame_id != frame ||
        object.meshes.size() != 1 ||
        object.mesh_poses.size() != 1 ||
        object.meshes.front().vertices.empty() ||
        !object.primitives.empty()) {
        return std::nullopt;
    }

    StoredWorkpiece stored;
    stored.object = object;

    tf2::fromMsg(
        object.pose, stored.object_pose);

    tf2::fromMsg(
        object.mesh_poses.front(),
        stored.mesh_pose);

    stored.world_from_mesh =
        stored.object_pose * stored.mesh_pose;

    if (!finiteRigid(stored.object_pose) ||
        !finiteRigid(stored.mesh_pose) ||
        !finiteRigid(stored.world_from_mesh)) {
        return std::nullopt;
    }

    return stored;
}


// Similar to InspectProfileServer's authoritative
// pose readback: compare the full object*mesh pose,
// not only the CollisionObject's outer pose.
bool poseMatches(
    const Eigen::Isometry3d& actual,
    const Eigen::Isometry3d& expected,
    double translation_tolerance,
    double rotation_tolerance) {

    return finiteRigid(actual) &&
           finiteRigid(expected) &&
           (actual.translation() -
            expected.translation()).norm() <=
               translation_tolerance &&
           rotationDifference(
               actual.linear(),
               expected.linear()) <=
               rotation_tolerance;
}

// Compare geometry as well as pose. A concurrent
// replacement with the same object ID is not safe
// to treat as the same workpiece.
bool sameMeshGeometry(
    const CollisionObject& a,
    const CollisionObject& b) {

    if (a.meshes.size() != 1 ||
        b.meshes.size() != 1)
        return false;

    const auto& am = a.meshes.front();
    const auto& bm = b.meshes.front();

    if (am.vertices.size() != bm.vertices.size() ||
        am.triangles.size() != bm.triangles.size())
        return false;

    for (std::size_t i = 0;
         i < am.vertices.size(); ++i) {

        const auto& av = am.vertices[i];
        const auto& bv = bm.vertices[i];

        if (av.x != bv.x ||
            av.y != bv.y ||
            av.z != bv.z)
            return false;
    }

    for (std::size_t i = 0;
         i < am.triangles.size(); ++i) {

        if (am.triangles[i].vertex_indices !=
            bm.triangles[i].vertex_indices)
            return false;
    }

    return true;
}

// Maximum displacement of mesh vertices under
// a proposed world-frame correction.
double maximumMeshDisplacement(
    const StoredWorkpiece& stored,
    const Eigen::Isometry3d& correction) {

    double maximum = 0.0;

    for (const auto& vertex :
         stored.object.meshes.front().vertices) {

        const Eigen::Vector3d local(
            vertex.x, vertex.y, vertex.z);

        const Eigen::Vector3d world =
            stored.world_from_mesh * local;

        const double displacement =
            (correction * world - world).norm();

        maximum = std::max(
            maximum, displacement);
    }

    return maximum;
}

// A projected reference point is supported if
// a fresh observed point lies sufficiently close.
double overlapFraction(
    const Cloud& source,
    const Cloud& target,
    const Eigen::Matrix4f& correction,
    double distance_m) {

    if (source.empty() || target.empty())
        return 0.0;

    Cloud projected;

    pcl::transformPointCloud(
        source, projected, correction);

    auto target_ptr = target.makeShared();

    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target_ptr);

    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);

    const double threshold2 =
        distance_m * distance_m;

    std::size_t valid = 0;
    std::size_t supported = 0;

    for (const auto& point : projected) {
        if (!pcl::isFinite(point))
            continue;

        ++valid;

        if (tree.nearestKSearch(
                point, 1,
                indices, squared_distances) > 0 &&
            squared_distances.front() <= threshold2) {
            ++supported;
        }
    }

    return valid > 0
        ? static_cast<double>(supported) / valid
        : 0.0;
}

struct Seed {
    Eigen::Matrix4f transform =
        Eigen::Matrix4f::Identity();

    std::string label;
};

// Rotation seeds are constructed around the
// workpiece center, not around world origin.
std::vector<Seed> makeSeeds(
    const Eigen::Isometry3d& workpiece_pose,
    double translation_step,
    double rotation_step) {

    std::vector<Seed> seeds;

    const Eigen::Vector3f axis =
        workpiece_pose.linear()
            .col(2).cast<float>().normalized();

    Eigen::Vector3f x =
        axis.cross(Eigen::Vector3f::UnitZ());

    if (x.norm() < 0.1f)
        x = axis.cross(Eigen::Vector3f::UnitX());

    x.normalize();

    const Eigen::Vector3f y =
        axis.cross(x).normalized();

    const Eigen::Vector3f center =
        workpiece_pose.translation().cast<float>();

    const float step =
        static_cast<float>(translation_step);

    const auto add = [&](
        const Eigen::Vector3f& translation,
        const Eigen::Vector3f& rotation_axis,
        double angle,
        const std::string& label) {

        Seed seed;
        seed.label = label;

        Eigen::Matrix3f rotation =
            Eigen::Matrix3f::Identity();

        if (std::abs(angle) > 0.0) {
            rotation =
                Eigen::AngleAxisf(
                    static_cast<float>(angle),
                    rotation_axis.normalized())
                    .toRotationMatrix();
        }

        seed.transform.block<3, 3>(0, 0) =
            rotation;

        seed.transform.block<3, 1>(0, 3) =
            center - rotation * center + translation;

        seeds.push_back(std::move(seed));
    };

    add(Eigen::Vector3f::Zero(),
        axis, 0.0, "identity");

    add( step * x, axis, 0.0, "+x");
    add(-step * x, axis, 0.0, "-x");

    add( step * y, axis, 0.0, "+y");
    add(-step * y, axis, 0.0, "-y");

    add( step * axis, axis, 0.0, "+axis");
    add(-step * axis, axis, 0.0, "-axis");

    for (const auto& rotation_axis : {x, y}) {
        for (double sign : {-1.0, 1.0}) {
            add(
                Eigen::Vector3f::Zero(),
                rotation_axis,
                sign * rotation_step,
                "tilt");
        }
    }

    return seeds;
}

struct Candidate {
    Eigen::Isometry3d correction =
        Eigen::Isometry3d::Identity();

    double fitness_m2 =
        std::numeric_limits<double>::infinity();

    double overlap = 0.0;
    double translation_m = 0.0;
    double rotation_rad = 0.0;

    std::size_t seed_index = 0;
};

}  // namespace

class RevalidateServer : public rclcpp::Node {
    using Action =
        hb_robot_interfaces::action::RevalidateWorkpiece;

    using Handle =
        rclcpp_action::ServerGoalHandle<Action>;

    using Reference =
        hb_robot_interfaces::msg::WorkpieceReference;

    using State =
        hb_robot::recovery::State;

public:
    RevalidateServer()
        : Node("revalidate_workpiece_server") {

        frame_ = declare_parameter<std::string>(
            "scene_base_frame", "world");

        range_ = declare_parameter<double>(
            "depth_range", 1.5);

        search_margin_m_ =
            declare_parameter<double>(
                "revalidation.search_margin_m", 0.10);

        seed_translation_m_ =
            declare_parameter<double>(
                "revalidation.seed_translation_m", 0.015);

        seed_rotation_rad_ =
            declare_parameter<double>(
                "revalidation.seed_rotation_rad", 0.035);

        min_overlap_ =
            declare_parameter<double>(
                "revalidation.min_overlap", 0.60);

        overlap_distance_m_ =
            declare_parameter<double>(
                "revalidation.overlap_distance_m", 0.010);

        ambiguity_translation_m_ =
            declare_parameter<double>(
                "revalidation.ambiguity_translation_m", 0.005);

        ambiguity_rotation_rad_ =
            declare_parameter<double>(
                "revalidation.ambiguity_rotation_rad", 0.02);

        ambiguity_fitness_ratio_ =
            declare_parameter<double>(
                "revalidation.ambiguity_fitness_ratio", 1.20);

        unchanged_tolerance_m_ =
            declare_parameter<double>(
                "revalidation.unchanged_tolerance_m", 0.0001);

        readback_position_tolerance_m_ =
            declare_parameter<double>(
                "revalidation.readback_position_tolerance_m",
                0.0001);

        readback_rotation_tolerance_rad_ =
            declare_parameter<double>(
                "revalidation.readback_rotation_tolerance_rad",
                0.0001);

        capture_timeout_s_ =
            declare_parameter<double>(
                "revalidation.capture_timeout_s", 15.0);

        confirmation_timeout_s_ =
            declare_parameter<double>(
                "revalidation.confirmation_timeout_s", 5.0);

        // Defaults preserve the previous registration
        // acceptance limits.
        config_.max_correspondence_m = 0.025;
        config_.max_translation_m = 0.05;
        config_.max_rotation_rad = 0.10;
        config_.max_fitness_m2 = 0.000025;
        config_.min_points = 100;

        if (!std::isfinite(range_) ||
            !std::isfinite(search_margin_m_) ||
            !std::isfinite(seed_translation_m_) ||
            !std::isfinite(seed_rotation_rad_) ||
            !std::isfinite(min_overlap_) ||
            !std::isfinite(overlap_distance_m_) ||
            !std::isfinite(ambiguity_translation_m_) ||
            !std::isfinite(ambiguity_rotation_rad_) ||
            !std::isfinite(ambiguity_fitness_ratio_) ||
            !std::isfinite(unchanged_tolerance_m_) ||
            !std::isfinite(capture_timeout_s_) ||
            !std::isfinite(confirmation_timeout_s_) ||
            range_ <= 0.0 ||
            search_margin_m_ <= 0.0 ||
            seed_translation_m_ <= 0.0 ||
            seed_rotation_rad_ <= 0.0 ||
            min_overlap_ <= 0.0 ||
            min_overlap_ > 1.0 ||
            overlap_distance_m_ <= 0.0 ||
            ambiguity_translation_m_ <= 0.0 ||
            ambiguity_rotation_rad_ <= 0.0 ||
            ambiguity_fitness_ratio_ < 1.0 ||
            unchanged_tolerance_m_ <= 0.0 ||
            capture_timeout_s_ <= 0.0 ||
            confirmation_timeout_s_ <= 0.0) {
            throw std::invalid_argument(
                "Invalid revalidation configuration");
        }

        tf_ = std::make_unique<tf2_ros::Buffer>(
            get_clock());

        listener_ =
            std::make_unique<tf2_ros::TransformListener>(
                *tf_);

        rgbd_ =
            std::make_unique<
                hb_perception::RGBDAcquisition>(
                this,
                tf_.get(),
                frame_,
                hb_perception::RGBDAcquisition::
                    Mode::CaptureWindow,
                "/k4a/rgb/image_raw",
                "/k4a/depth_to_rgb/image_raw",
                "/k4a/depth_to_rgb/camera_info");

        supervision_ =
            std::make_unique<
                hb_robot_skills::PlatformSupervisionClient>(
                *this);

        ref_sub_ = create_subscription<Reference>(
            "/workpiece_reference",
            rclcpp::QoS(1).transient_local(),
            [this](Reference::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                reference_ = *msg;
            });

        state_sub_ = create_subscription<State>(
            "/task_state",
            rclcpp::QoS(1).transient_local(),
            [this](State::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = *msg;
                state_received_ = Clock::now();
            });

        ref_pub_ = create_publisher<Reference>(
            "/workpiece_reference",
            rclcpp::QoS(1).transient_local());

        estimate_pub_ = create_publisher<
            hb_robot_interfaces::msg::ProfileEstimate>(
            "/profile_inspection/estimate",
            rclcpp::QoS(1).transient_local());

        const auto cloud_qos =
            rclcpp::QoS(1).reliable().transient_local();

        acquisition_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/acquisition_cloud",
                cloud_qos);

        target_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/target_cloud",
                cloud_qos);

        reference_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/reference_cloud",
                cloud_qos);

        aligned_reference_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/aligned_reference_cloud",
                cloud_qos);

        server_ = rclcpp_action::create_server<Action>(
            this,
            "revalidate_workpiece",

            [this](const auto&, auto) {
                bool idle = false;

                return busy_.compare_exchange_strong(
                    idle, true)
                    ? rclcpp_action::GoalResponse::
                          ACCEPT_AND_EXECUTE
                    : rclcpp_action::GoalResponse::REJECT;
            },

            [](auto) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },

            [this](auto handle) {
                if (worker_.joinable())
                    worker_.join();

                worker_ = std::thread(
                    [this, handle] {
                        run(handle);
                        busy__storeFalse();
                    });
            });

        RCLCPP_INFO(
            get_logger(),
            "RevalidateWorkpiece ready: "
            "object=%s max_correction=%.1f mm / %.2f deg "
            "fitness_limit=%.3f mm RMS "
            "min_overlap=%.2f",
            kWorkpieceId,
            config_.max_translation_m * 1000.0,
            toDegrees(config_.max_rotation_rad),
            rmsMm(config_.max_fitness_m2),
            min_overlap_);
    }

    ~RevalidateServer() override {
        if (worker_.joinable())
            worker_.join();
    }

    
void publishCloud(
    const Cloud& cloud,
    const rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = frame_;
    msg.header.stamp = stamp;

    publisher->publish(msg);
}

private:
    void busy__storeFalse() {
        busy_.store(false);
    }

    State snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    bool stateFresh() {
        std::lock_guard<std::mutex> lock(mutex_);

        return Clock::now() - state_received_ < 500ms;
    }

    bool referenceCurrent(
        uint64_t expected_pose,
        const std::string& expected_id,
        uint64_t expected_geometry) {

        const State state = snapshot();

        return stateFresh() &&
               state.scene_valid &&
               state.geometry_version == expected_geometry &&
               state.pose_version == expected_pose &&
               state.scene_id == expected_id &&
               !state.motion_fault_active &&
               !state.process_fault_active;
    }

    bool authorized(
        uint64_t expected_pose,
        const std::string& expected_id,
        uint64_t expected_geometry) {

        const State state = snapshot();

        return referenceCurrent(
                   expected_pose,
                   expected_id,
                   expected_geometry) &&
               state.supervision_valid &&
               state.imu_stable &&
               supervision_->stable();
    }

    bool waitForTaskConfirmation(
        const std::string& new_reference_id,
        uint64_t expected_geometry,
        uint64_t expected_pose,
        uint64_t old_scene_version,
        bool scene_changed,
        const std::function<bool()>& canceled) {

        const auto deadline =
            Clock::now() +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(
                    confirmation_timeout_s_));

        while (!canceled() &&
               Clock::now() < deadline) {

            const State state = snapshot();

            if (stateFresh() &&
                state.scene_id == new_reference_id &&
                state.geometry_version == expected_geometry &&
                state.pose_version >= expected_pose &&
                state.workpiece_pose_valid &&
                state.supervision_valid &&
                state.imu_stable &&
                !state.motion_fault_active &&
                !state.process_fault_active &&
                (!scene_changed ||
                 state.scene_version > old_scene_version) &&
                supervision_->stable()) {

                RCLCPP_INFO(
                    get_logger(),
                    "TaskState confirmed recovery: "
                    "geometry=%llu pose=%llu scene=%llu",
                    static_cast<unsigned long long>(
                        state.geometry_version),
                    static_cast<unsigned long long>(
                        state.pose_version),
                    static_cast<unsigned long long>(
                        state.scene_version));

                return true;
            }

            std::this_thread::sleep_for(50ms);
        }

        const State state = snapshot();

        RCLCPP_ERROR(
            get_logger(),
            "Recovery acknowledgment timeout: "
            "reference=%s expected=%s "
            "geometry=%llu/%llu pose=%llu/%llu "
            "scene=%llu previous=%llu "
            "pose_valid=%d status=%s",
            state.scene_id.c_str(),
            new_reference_id.c_str(),
            static_cast<unsigned long long>(
                state.geometry_version),
            static_cast<unsigned long long>(
                expected_geometry),
            static_cast<unsigned long long>(
                state.pose_version),
            static_cast<unsigned long long>(
                expected_pose),
            static_cast<unsigned long long>(
                state.scene_version),
            static_cast<unsigned long long>(
                old_scene_version),
            state.workpiece_pose_valid,
            state.status_message.c_str());

        return false;
    }

    // Applies only a pose update to the existing
    // extrusion mesh. Its vertices and object ID
    // remain unchanged.
    //
    // The function checks the pre-update object,
    // applies the new pose, and reads it back
    // from the authoritative MoveIt scene.
    bool updateWorkpieceMesh(
        PlanningSceneInterface& scene,
        const StoredWorkpiece& before,
        const Eigen::Isometry3d& correction,
        std::string& error) {

        if (!finiteRigid(correction)) {
            error = "Invalid correction transform";
            return false;
        }

        // Detect unexpected replacement or editing
        // before committing the recovered pose.
        const auto current =
            lookupWorkpiece(scene, frame_);

        if (!current) {
            error = "Workpiece missing before commit";
            return false;
        }

        if (!sameMeshGeometry(
                before.object,
                current->object) ||
            !poseMatches(
                current->world_from_mesh,
                before.world_from_mesh,
                readback_position_tolerance_m_,
                readback_rotation_tolerance_rad_)) {

            error = "Workpiece changed before commit";
            return false;
        }

        auto updated = current->object;

        const Eigen::Isometry3d expected_world_pose =
            correction * current->world_from_mesh;

        // world_from_mesh =
        //     world_from_object * object_from_mesh
        //
        // Keep world_from_object unchanged and update
        // only object_from_mesh.
        const Eigen::Isometry3d new_mesh_pose =
            current->object_pose.inverse() *
            expected_world_pose;

        if (!finiteRigid(new_mesh_pose)) {
            error = "Invalid updated mesh pose";
            return false;
        }

        updated.mesh_poses.front() =
            tf2::toMsg(new_mesh_pose);

        updated.header.stamp = now();
        updated.operation = CollisionObject::ADD;

        if (!scene.applyCollisionObject(updated)) {
            error = "MoveIt rejected workpiece pose update";
            return false;
        }

        // Authoritative readback, as performed by
        // InspectProfileServer::updateCuttingCollision().
        const auto readback =
            lookupWorkpiece(scene, frame_);

        if (!readback) {
            error = "Workpiece missing after update";
            return false;
        }

        if (!sameMeshGeometry(
                updated, readback->object)) {
            error = "Mesh geometry changed on readback";
            return false;
        }

        const double position_error =
            (readback->world_from_mesh.translation() -
             expected_world_pose.translation()).norm();

        const double rotation_error =
            rotationDifference(
                readback->world_from_mesh.linear(),
                expected_world_pose.linear());

        RCLCPP_INFO(
            get_logger(),
            "Workpiece mesh readback: "
            "position_error=%.6f m "
            "rotation_error=%.6f rad "
            "vertices=%zu",
            position_error,
            rotation_error,
            readback->object.meshes.front()
                .vertices.size());

        if (!poseMatches(
                readback->world_from_mesh,
                expected_world_pose,
                readback_position_tolerance_m_,
                readback_rotation_tolerance_rad_)) {

            error = "Workpiece pose readback mismatch";
            return false;
        }

        return true;
    }

    void run(const std::shared_ptr<Handle>& handle) {
        auto result =
            std::make_shared<Action::Result>();

        result->localized = false;
        result->poorly_constrained = false;

        const auto canceled = [&] {
            return handle->is_canceling() ||
                   !rclcpp::ok();
        };

        bool completed = false;

        const auto finish =
            [&](const std::string& message) {

            if (completed)
                return;

            completed = true;
            result->message = message;

            RCLCPP_INFO(
                get_logger(),
                "RevalidateWorkpiece: localized=%d "
                "poorly_constrained=%d "
                "fitness_m2=%.9f rms_mm=%.3f "
                "translation_mm=%.3f "
                "rotation_deg=%.3f "
                "message=%s",
                result->localized,
                result->poorly_constrained,
                result->registration_fitness,
                rmsMm(result->registration_fitness),
                result->translation_delta * 1000.0,
                toDegrees(result->rotation_delta),
                message.c_str());

            if (handle->is_canceling()) {
                result->localized = false;
                handle->canceled(result);
            } else if (result->localized) {
                handle->succeed(result);
            } else {
                handle->abort(result);
            }
        };

        try {
            const auto goal = handle->get_goal();

            Reference reference;
            State initial_state;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                reference = reference_;
                initial_state = state_;
            }

            if (!stateFresh() ||
                reference.cloud.data.empty() ||
                reference.cloud.header.frame_id != frame_ ||
                reference.estimate.header.frame_id != frame_) {

                finish("No fresh matching workpiece reference");
                return;
            }

            const uint64_t expected_geometry =
                initial_state.geometry_version;

            const uint64_t expected_pose =
                goal->expected_pose_version;

            const std::string expected_id =
                std::to_string(
                    rclcpp::Time(
                        reference.estimate.header.stamp)
                        .nanoseconds());

            if (!referenceCurrent(
                    expected_pose,
                    expected_id,
                    expected_geometry)) {

                finish("Reference superseded before recovery");
                return;
            }

            // 1. Look up the authoritative workpiece
            // before acquiring the observation.
            PlanningSceneInterface scene;

            const auto stored =
                lookupWorkpiece(scene, frame_);

            if (!stored) {
                finish(
                    "Authoritative wk_workpiece_collision "
                    "mesh missing or malformed");
                return;
            }

            Eigen::Isometry3d original_estimate;
            tf2::fromMsg(
                reference.estimate.pose,
                original_estimate);

            if (!poseMatches(
                    stored->world_from_mesh,
                    original_estimate,
                    readback_position_tolerance_m_,
                    readback_rotation_tolerance_rad_)) {

                finish(
                    "Stored workpiece mesh pose disagrees "
                    "with the reference estimate");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Recovery reference verified: "
                "geometry=%llu pose=%llu scene=%llu "
                "mesh_vertices=%zu",
                static_cast<unsigned long long>(
                    expected_geometry),
                static_cast<unsigned long long>(
                    expected_pose),
                static_cast<unsigned long long>(
                    initial_state.scene_version),
                stored->object.meshes.front()
                    .vertices.size());

            // 2. Wait for trusted platform supervision.
            const auto supervision_deadline =
                Clock::now() + 10s;

            while (!canceled() &&
                   Clock::now() < supervision_deadline) {

                if (!referenceCurrent(
                        expected_pose,
                        expected_id,
                        expected_geometry)) {
                    finish("Reference changed while waiting");
                    return;
                }

                if (authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry))
                    break;

                std::this_thread::sleep_for(50ms);
            }

            if (canceled()) {
                finish("Recovery canceled");
                return;
            }

            if (!authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry) ||
                !supervision_->waitForStable(
                    10s, canceled)) {

                finish("Stable platform supervision unavailable");
                return;
            }

            // 3. Capture fresh RGB-D after stability.
            const rclcpp::Time after = now();

            const auto acquisition_deadline =
                Clock::now() +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(
                        capture_timeout_s_));

            std::optional<hb_perception::Observation>
                observation;

            std::uint64_t acquisition_attempts = 0;

            while (!canceled() &&
                   Clock::now() < acquisition_deadline) {

                if (!authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry)) {

                    finish("Supervision lost during acquisition");
                    return;
                }

                ++acquisition_attempts;

                observation =
                    rgbd_->acquireAfter(after, 500ms);

                if (observation)
                    break;
            }

            if (canceled()) {
                finish("Recovery canceled");
                return;
            }

            if (!observation) {
                finish("Fresh RGB-D acquisition timed out");
                return;
            }

            if (!authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry)) {

                finish("Supervision lost after acquisition");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Fresh RGB-D: attempts=%llu stamp=%.6f",
                static_cast<unsigned long long>(
                    acquisition_attempts),
                rclcpp::Time(
                    observation->stamp).seconds());

            // 4. Convert to world-frame clouds.
            auto raw =
                hb_perception::observationToCloud(
                    *observation, range_);

            if (!raw || raw->empty()) {
                finish("Empty RGB-D observation");
                return;
            }

            Cloud world;
            Cloud reference_cloud;

            const Eigen::Isometry3d camera_pose =
                tf2::transformToEigen(
                    observation->camera_pose);

            pcl::transformPointCloud(
                *raw,
                world,
                camera_pose.matrix().cast<float>());

            pcl::fromROSMsg(
                reference.cloud,
                reference_cloud);

            // Publish the complete acquired scene, transformed
            // into the world frame using the observation camera pose.
            publishCloud(
                world,
                acquisition_cloud_pub_,
                observation->stamp);

            // Publish the existing reference before correction.
            publishCloud(
                reference_cloud,
                reference_cloud_pub_,
                observation->stamp);

            RCLCPP_INFO(
                get_logger(),
                "Published acquisition cloud: "
                "points=%zu frame=%s stamp=%.6f",
                world.size(),
                frame_.c_str(),
                rclcpp::Time(observation->stamp).seconds());    

            if (reference_cloud.empty()) {
                finish("Reference cloud empty");
                return;
            }

            // 5. Crop around the existing workpiece,
            // allowing bounded pose recovery.
            Eigen::Vector4f lo, hi;

            pcl::getMinMax3D(
                reference_cloud, lo, hi);

            Cloud cropped;

            const float margin =
                static_cast<float>(search_margin_m_);

            for (const auto& point : world) {
                if (!pcl::isFinite(point))
                    continue;

                const auto xyz =
                    point.getVector3fMap().array();

                if ((xyz >=
                     lo.head<3>().array() -
                         margin).all() &&
                    (xyz <=
                     hi.head<3>().array() +
                         margin).all()) {

                    cropped.push_back(point);
                }
            }

            const Cloud source =
                voxelize(reference_cloud, 0.003f);

            const Cloud target =
                voxelize(cropped, 0.003f);


            publishCloud(
              target,
              target_cloud_pub_,
              observation->stamp);

            RCLCPP_INFO(
                get_logger(),
                "Recovery point counts: "
                "raw=%zu world=%zu reference=%zu "
                "crop=%zu source=%zu target=%zu",
                raw->size(),
                world.size(),
                reference_cloud.size(),
                cropped.size(),
                source.size(),
                target.size());

            if (source.size() < config_.min_points ||
                target.size() < config_.min_points) {

                finish("Insufficient points for registration");
                return;
            }

            // 6. Multi-hypothesis registration.
            const auto seeds =
                makeSeeds(
                    original_estimate,
                    seed_translation_m_,
                    seed_rotation_rad_);

            hb_perception::ObservationRegistrar
                registrar(config_);

            std::vector<Candidate> candidates;

            double best_failed_fitness =
                std::numeric_limits<double>::infinity();

            std::string best_failed_reason =
                "No candidate";

            for (std::size_t i = 0;
                 i < seeds.size(); ++i) {

                if (canceled() ||
                    !authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry)) {

                    finish("Interrupted during registration");
                    return;
                }

                Cloud projected;

                pcl::transformPointCloud(
                    source,
                    projected,
                    seeds[i].transform);

                const auto registration =
                    registrar.registerCloud(
                        projected, target);

                if (std::isfinite(
                        registration.fitness_m2) &&
                    registration.fitness_m2 <
                        best_failed_fitness) {

                    best_failed_fitness =
                        registration.fitness_m2;

                    best_failed_reason =
                        registration.reason;
                }

                if (!registration.accepted)
                    continue;

                const Eigen::Matrix4f total =
                    registration.correction *
                    seeds[i].transform;

                Eigen::Isometry3d correction =
                    Eigen::Isometry3d::Identity();

                correction.matrix() =
                    total.cast<double>();

                if (!finiteRigid(correction))
                    continue;

                const double rotation =
                    std::abs(
                        Eigen::AngleAxisd(
                            correction.linear()).angle());

                // Measure displacement at the actual
                // workpiece, not only at world origin.
                const double maximum_displacement =
                    maximumMeshDisplacement(
                        *stored, correction);

                if (maximum_displacement >
                        config_.max_translation_m ||
                    rotation >
                        config_.max_rotation_rad)
                    continue;

                const double overlap =
                    overlapFraction(
                        source,
                        target,
                        total,
                        overlap_distance_m_);

                if (overlap < min_overlap_)
                    continue;

                Candidate candidate;

                candidate.correction = correction;
                candidate.fitness_m2 =
                    registration.fitness_m2;
                candidate.overlap = overlap;
                candidate.translation_m =
                    maximum_displacement;
                candidate.rotation_rad = rotation;
                candidate.seed_index = i;

                candidates.push_back(
                    std::move(candidate));

                RCLCPP_INFO(
                    get_logger(),
                    "Candidate %zu: "
                    "fitness=%.9f rms=%.3f mm "
                    "overlap=%.3f "
                    "max_displacement=%.3f mm "
                    "rotation=%.3f deg",
                    i,
                    registration.fitness_m2,
                    rmsMm(registration.fitness_m2),
                    overlap,
                    maximum_displacement * 1000.0,
                    toDegrees(rotation));
            }

            if (candidates.empty()) {
                result->registration_fitness =
                    best_failed_fitness;

                finish(
                    "No recovery candidate passed: " +
                    best_failed_reason);
                return;
            }

            std::sort(
                candidates.begin(),
                candidates.end(),
                [](const Candidate& a,
                   const Candidate& b) {

                    if (std::abs(
                            a.fitness_m2 -
                            b.fitness_m2) > 1e-9)
                        return a.fitness_m2 <
                               b.fitness_m2;

                    return a.overlap > b.overlap;
                });

            const Candidate& best =
                candidates.front();


            Cloud aligned_reference;

            pcl::transformPointCloud(
                source,
                aligned_reference,
                best.correction.matrix().cast<float>());

            publishCloud(
                aligned_reference,
                aligned_reference_cloud_pub_,
                observation->stamp);

            RCLCPP_INFO(
                get_logger(),
                "Published aligned reference: "
                "points=%zu fitness_m2=%.9f "
                "overlap=%.3f",
                aligned_reference.size(),
                best.fitness_m2,
                best.overlap);

            result->registration_fitness =
                best.fitness_m2;

            result->translation_delta =
                best.translation_m;

            result->rotation_delta =
                best.rotation_rad;

            // 7. Reject competing plausible poses.
            bool ambiguous = false;

            const Eigen::Vector3d beam_axis =
                original_estimate.linear()
                    .col(2).normalized();

            for (std::size_t i = 1;
                 i < candidates.size(); ++i) {

                const auto& alternative =
                    candidates[i];

                if (alternative.fitness_m2 >
                    best.fitness_m2 *
                        ambiguity_fitness_ratio_ +
                        1e-7)
                    continue;

                const Eigen::Isometry3d relative =
                    alternative.correction *
                    best.correction.inverse();

                // Compare displacement at the
                // workpiece, not at world origin.
                const Eigen::Vector3d center =
                    stored->world_from_mesh.translation();

                const Eigen::Vector3d displacement =
                    relative * center - center;

                const double axial =
                    std::abs(
                        displacement.dot(beam_axis));

                const double transverse =
                    (displacement -
                     displacement.dot(beam_axis) *
                         beam_axis).norm();

                const double rotation =
                    std::abs(
                        Eigen::AngleAxisd(
                            relative.linear()).angle());

                if (axial >
                        ambiguity_translation_m_ ||
                    transverse >
                        ambiguity_translation_m_ ||
                    rotation >
                        ambiguity_rotation_rad_) {

                    ambiguous = true;

                    RCLCPP_WARN(
                        get_logger(),
                        "Competing recovery pose: "
                        "seed=%zu axial=%.3f mm "
                        "transverse=%.3f mm "
                        "rotation=%.3f deg",
                        alternative.seed_index,
                        axial * 1000.0,
                        transverse * 1000.0,
                        toDegrees(rotation));
                }
            }

            // Explicit axial perturbation test.
            // This checks a known ambiguity of
            // extruded profiles even if the ordinary
            // candidate set converges to one solution.
            for (double shift : {-0.015, 0.015}) {
                if (canceled() ||
                    !authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry)) {
                    finish("Interrupted during ambiguity test");
                    return;
                }

                Eigen::Matrix4f offset =
                    Eigen::Matrix4f::Identity();

                offset.block<3, 1>(0, 3) =
                    (beam_axis * shift).cast<float>();

                Cloud shifted;
                pcl::transformPointCloud(
                    source, shifted, offset);

                const auto alternative =
                    registrar.registerCloud(
                        shifted, target);

                if (!alternative.accepted)
                    continue;

                const Eigen::Matrix4f total =
                    alternative.correction * offset;

                Eigen::Isometry3d alternative_delta =
                    Eigen::Isometry3d::Identity();

                alternative_delta.matrix() =
                    total.cast<double>();

                if (!finiteRigid(alternative_delta))
                    continue;

                if (alternative.fitness_m2 >
                    best.fitness_m2 *
                        ambiguity_fitness_ratio_ +
                        1e-7)
                    continue;

                const Eigen::Vector3d center =
                    stored->world_from_mesh.translation();

                const Eigen::Vector3d difference =
                    alternative_delta * center -
                    best.correction * center;

                const double axial =
                    std::abs(
                        difference.dot(beam_axis));

                const double transverse =
                    (difference -
                     difference.dot(beam_axis) *
                         beam_axis).norm();

                if (axial >
                        ambiguity_translation_m_ &&
                    transverse <
                        ambiguity_translation_m_) {
                    ambiguous = true;
                }
            }

            if (ambiguous) {
                result->poorly_constrained = true;

                finish(
                    "Registration is geometrically "
                    "ambiguous; no scene update");
                return;
            }

            // 8. Determine whether the mesh actually
            // needs a new pose.
            const double maximum_displacement =
                maximumMeshDisplacement(
                    *stored, best.correction);

            const bool changed =
                maximum_displacement >
                    unchanged_tolerance_m_;

            RCLCPP_INFO(
                get_logger(),
                "Recovery decision: "
                "changed=%d max_displacement=%.3f mm "
                "tolerance=%.3f mm",
                changed,
                maximum_displacement * 1000.0,
                unchanged_tolerance_m_ * 1000.0);

            if (canceled() ||
                !authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry)) {

                finish("State changed before scene commit");
                return;
            }

            // 9. Capture current scene version.
            const State before_commit = snapshot();

            // Reject an unrelated scene update while
            // registration was running.
            if (before_commit.scene_version !=
                initial_state.scene_version) {

                finish(
                    "Planning scene changed during "
                    "revalidation; retry with new snapshot");
                return;
            }

            // 10. Apply and verify the mesh pose.
            if (changed) {
                std::string error;

                if (!updateWorkpieceMesh(
                        scene,
                        *stored,
                        best.correction,
                        error)) {

                    finish(
                        "Workpiece mesh commit failed: " +
                        error);
                    return;
                }
            } else {
                // Even without a change, verify that
                // the authoritative object is still
                // the one used for registration.
                const auto readback =
                    lookupWorkpiece(scene, frame_);

                if (!readback ||
                    !sameMeshGeometry(
                        stored->object,
                        readback->object) ||
                    !poseMatches(
                        stored->world_from_mesh,
                        readback->world_from_mesh,
                        readback_position_tolerance_m_,
                        readback_rotation_tolerance_rad_)) {

                    finish(
                        "Workpiece changed during "
                        "unchanged-pose confirmation");
                    return;
                }
            }

            // 11. Publish the corresponding estimate
            // and transformed reference model.
            //
            // This must be the SAME correction as
            // the one applied to the mesh.
            if (changed) {
                reference.estimate.pose =
                    tf2::toMsg(
                        best.correction *
                        original_estimate);

                Cloud corrected_cloud;

                pcl::transformPointCloud(
                    reference_cloud,
                    corrected_cloud,
                    best.correction.matrix()
                        .cast<float>());

                pcl::toROSMsg(
                    corrected_cloud,
                    reference.cloud);
            }

            // Preserve the observation time as the
            // evidence timestamp.
            //
            // The cloud payload is still the reference
            // model, not the raw camera observation.
            reference.estimate.header.stamp = now();

            reference.cloud.header =
                reference.estimate.header;

            reference.cloud.header.stamp =
                observation->stamp;

            reference.relocalized = true;
            reference.expected_pose_version =
                expected_pose;

            const std::string new_reference_id =
                std::to_string(
                    rclcpp::Time(
                        reference.estimate.header.stamp)
                        .nanoseconds());

            result->estimate =
                reference.estimate;

            estimate_pub_->publish(
                reference.estimate);

            ref_pub_->publish(reference);

            RCLCPP_INFO(
                get_logger(),
                "Recovery reference published: "
                "id=%s mesh_updated=%d "
                "old_scene_version=%llu",
                new_reference_id.c_str(),
                changed,
                static_cast<unsigned long long>(
                    before_commit.scene_version));

            // 12. Wait for the task-state authority.
            //
            // When the mesh changes, require a scene
            // revision as well as pose confirmation.
            if (!waitForTaskConfirmation(
                    new_reference_id,
                    expected_geometry,
                    expected_pose,
                    before_commit.scene_version,
                    changed,
                    canceled)) {

                finish(
                    "Workpiece recovery published but "
                    "task-state acknowledgment failed; "
                    "do not execute a stored plan");
                return;
            }

            result->localized = true;
            result->poorly_constrained = false;

            finish(
                changed
                    ? "Workpiece pose recovered; mesh "
                      "readback verified and task-state "
                      "confirmed"
                    : "Fresh observation confirmed "
                      "existing workpiece pose; "
                      "scene unchanged");

        } catch (const std::exception& error) {
            finish(
                std::string(
                    "Revalidation exception: ") +
                error.what());
        }
    }

    std::string frame_;
    double range_ = 1.5;

    double search_margin_m_ = 0.10;
    double seed_translation_m_ = 0.015;
    double seed_rotation_rad_ = 0.035;

    double min_overlap_ = 0.60;
    double overlap_distance_m_ = 0.010;

    double ambiguity_translation_m_ = 0.005;
    double ambiguity_rotation_rad_ = 0.02;
    double ambiguity_fitness_ratio_ = 1.20;

    double unchanged_tolerance_m_ = 0.0001;

    double readback_position_tolerance_m_ = 0.0001;
    double readback_rotation_tolerance_rad_ = 0.0001;

    double capture_timeout_s_ = 15.0;
    double confirmation_timeout_s_ = 5.0;

    hb_perception::RegistrationConfig config_;

    std::mutex mutex_;
    Reference reference_;
    State state_;

    Clock::time_point state_received_{};

    std::atomic<bool> busy_{false};
    std::thread worker_;

    std::unique_ptr<tf2_ros::Buffer> tf_;

    std::unique_ptr<
        tf2_ros::TransformListener> listener_;

    std::unique_ptr<
        hb_perception::RGBDAcquisition> rgbd_;

    std::unique_ptr<
        hb_robot_skills::PlatformSupervisionClient>
        supervision_;

    rclcpp::Subscription<Reference>::SharedPtr
        ref_sub_;

    rclcpp::Subscription<State>::SharedPtr
        state_sub_;

    rclcpp::Publisher<Reference>::SharedPtr
        ref_pub_;

    rclcpp::Publisher<
        hb_robot_interfaces::msg::ProfileEstimate>::
        SharedPtr estimate_pub_;

    rclcpp_action::Server<Action>::SharedPtr
        server_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    acquisition_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        target_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        reference_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        aligned_reference_cloud_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RevalidateServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
