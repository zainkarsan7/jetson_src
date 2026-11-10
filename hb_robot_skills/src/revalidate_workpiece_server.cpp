
#include <hb_robot_interfaces/action/revalidate_workpiece.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/srv/commit_workpiece.hpp>

#include "hb_robot_skills/workpiece_commit_validation.hpp"
#include "hb_robot_skills/platform_supervision_client.hpp"

#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <hb_robot_perception/observation_registrar.hpp>
#include <hb_robot_perception/perception_types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
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
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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

using CollisionObject =
    moveit_msgs::msg::CollisionObject;

using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;

namespace wp = hb_robot_skills::workpiece;

constexpr double kPi =
    3.14159265358979323846;

constexpr const char* kWorkpieceId =
    "wk_workpiece_collision";

double degrees(double radians)
{
    return radians * 180.0 / kPi;
}

double rmsMm(double fitness)
{
    if (!std::isfinite(fitness) ||
        fitness < 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    return 1000.0 * std::sqrt(fitness);
}

bool finiteRigid(const Eigen::Isometry3d& pose)
{
    if (!pose.matrix().allFinite())
        return false;

    const auto& r = pose.linear();

    return std::abs(r.determinant() - 1.0) < 1e-3 &&
           (r.transpose() * r).isApprox(
               Eigen::Matrix3d::Identity(), 1e-3);
}

double rotationDifference(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b)
{
    return std::abs(
        Eigen::AngleAxisd(
            a * b.transpose()).angle());
}

bool poseMatches(
    const Eigen::Isometry3d& actual,
    const Eigen::Isometry3d& expected,
    double translation_tolerance,
    double rotation_tolerance)
{
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

Cloud voxelize(
    const Cloud& input,
    float resolution)
{
    Cloud output;

    Cloud::Ptr finite(new Cloud);

    for (const auto& point : input)
    {
        if (pcl::isFinite(point))
            finite->push_back(point);
    }

    if (finite->empty())
        return output;

    pcl::VoxelGrid<Point> filter;

    filter.setLeafSize(
        resolution,
        resolution,
        resolution);

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
    const std::string& frame)
{
    const auto objects =
        scene.getObjects({kWorkpieceId});

    const auto it =
        objects.find(kWorkpieceId);

    if (it == objects.end())
        return std::nullopt;

    const auto& object = it->second;

    if (object.header.frame_id != frame ||
        !wp::validMesh(object))
    {
        return std::nullopt;
    }

    StoredWorkpiece stored;
    stored.object = object;

    tf2::fromMsg(
        object.pose,
        stored.object_pose);

    tf2::fromMsg(
        object.mesh_poses.front(),
        stored.mesh_pose);

    stored.world_from_mesh =
        stored.object_pose * stored.mesh_pose;

    if (!finiteRigid(stored.object_pose) ||
        !finiteRigid(stored.mesh_pose) ||
        !finiteRigid(stored.world_from_mesh))
    {
        return std::nullopt;
    }

    return stored;
}

double maximumMeshDisplacement(
    const StoredWorkpiece& stored,
    const Eigen::Isometry3d& correction)
{
    double maximum = 0.0;

    for (const auto& vertex :
         stored.object.meshes.front().vertices)
    {
        const Eigen::Vector3d local(
            vertex.x,
            vertex.y,
            vertex.z);

        const Eigen::Vector3d world =
            stored.world_from_mesh * local;

        maximum = std::max(
            maximum,
            (correction * world - world).norm());
    }

    return maximum;
}

double overlapFraction(
    const Cloud& source,
    const Cloud& target,
    const Eigen::Matrix4f& correction,
    double distance_m)
{
    if (source.empty() || target.empty())
        return 0.0;

    Cloud projected;

    pcl::transformPointCloud(
        source,
        projected,
        correction);

    pcl::KdTreeFLANN<Point> tree;

    tree.setInputCloud(target.makeShared());

    std::vector<int> indices(1);
    std::vector<float> distances(1);

    const double threshold2 =
        distance_m * distance_m;

    std::size_t valid = 0;
    std::size_t supported = 0;

    for (const auto& point : projected)
    {
        if (!pcl::isFinite(point))
            continue;

        ++valid;

        if (tree.nearestKSearch(
                point,
                1,
                indices,
                distances) > 0 &&
            distances.front() <= threshold2)
        {
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

std::vector<Seed> makeSeeds(
    const Eigen::Isometry3d& workpiece_pose,
    double translation_step,
    double rotation_step)
{
    std::vector<Seed> seeds;

    const Eigen::Vector3f axis =
        workpiece_pose.linear()
            .col(2).cast<float>().normalized();

    Eigen::Vector3f x =
        axis.cross(Eigen::Vector3f::UnitZ());

    if (x.norm() < 0.1f)
    {
        x = axis.cross(
            Eigen::Vector3f::UnitX());
    }

    x.normalize();

    const Eigen::Vector3f y =
        axis.cross(x).normalized();

    const Eigen::Vector3f center =
        workpiece_pose.translation().cast<float>();

    const float step =
        static_cast<float>(translation_step);

    const auto add =
        [&](const Eigen::Vector3f& translation,
            const Eigen::Vector3f& rotation_axis,
            double angle,
            const std::string& label)
    {
        Seed seed;
        seed.label = label;

        Eigen::Matrix3f rotation =
            Eigen::Matrix3f::Identity();

        if (std::abs(angle) > 0.0)
        {
            rotation =
                Eigen::AngleAxisf(
                    static_cast<float>(angle),
                    rotation_axis.normalized())
                    .toRotationMatrix();
        }

        seed.transform.block<3, 3>(0, 0) =
            rotation;

        seed.transform.block<3, 1>(0, 3) =
            center -
            rotation * center +
            translation;

        seeds.push_back(std::move(seed));
    };

    add(
        Eigen::Vector3f::Zero(),
        axis, 0.0, "identity");

    add( step * x, axis, 0.0, "+x");
    add(-step * x, axis, 0.0, "-x");

    add( step * y, axis, 0.0, "+y");
    add(-step * y, axis, 0.0, "-y");

    add( step * axis, axis, 0.0, "+axis");
    add(-step * axis, axis, 0.0, "-axis");

    for (const auto& rotation_axis : {x, y})
    {
        for (double sign : {-1.0, 1.0})
        {
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

    using Commit =
        hb_robot_interfaces::srv::CommitWorkpiece;

public:
    RevalidateServer()
        : Node("revalidate_workpiece_server")
    {
        frame_ =
            declare_parameter<std::string>(
                "scene_base_frame", "world");

        range_ =
            declare_parameter<double>(
                "depth_range", 1.5);

        search_margin_m_ =
            declare_parameter<double>(
                "revalidation.search_margin_m",
                0.10);

        seed_translation_m_ =
            declare_parameter<double>(
                "revalidation.seed_translation_m",
                0.015);

        seed_rotation_rad_ =
            declare_parameter<double>(
                "revalidation.seed_rotation_rad",
                0.035);

        min_overlap_ =
            declare_parameter<double>(
                "revalidation.min_overlap",
                0.60);

        overlap_distance_m_ =
            declare_parameter<double>(
                "revalidation.overlap_distance_m",
                0.010);

        ambiguity_translation_m_ =
            declare_parameter<double>(
                "revalidation.ambiguity_translation_m",
                0.005);

        ambiguity_rotation_rad_ =
            declare_parameter<double>(
                "revalidation.ambiguity_rotation_rad",
                0.02);

        ambiguity_fitness_ratio_ =
            declare_parameter<double>(
                "revalidation.ambiguity_fitness_ratio",
                1.20);

        unchanged_tolerance_m_ =
            declare_parameter<double>(
                "revalidation.unchanged_tolerance_m",
                0.0001);

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
                "revalidation.capture_timeout_s",
                15.0);

        confirmation_timeout_s_ =
            declare_parameter<double>(
                "revalidation.confirmation_timeout_s",
                5.0);

        commit_timeout_s_ =
            declare_parameter<double>(
                "revalidation.commit_timeout_s",
                10.0);

        config_.max_correspondence_m = 0.025;
        config_.max_translation_m = 0.05;
        config_.max_rotation_rad = 0.10;
        config_.max_fitness_m2 = 0.000025;
        config_.min_points = 100;

        const std::vector<double> positive{
            range_,
            search_margin_m_,
            seed_translation_m_,
            seed_rotation_rad_,
            overlap_distance_m_,
            ambiguity_translation_m_,
            ambiguity_rotation_rad_,
            ambiguity_fitness_ratio_,
            unchanged_tolerance_m_,
            readback_position_tolerance_m_,
            readback_rotation_tolerance_rad_,
            capture_timeout_s_,
            confirmation_timeout_s_,
            commit_timeout_s_
        };

        if (frame_.empty() ||
            !std::all_of(
                positive.begin(),
                positive.end(),
                [](double x)
                {
                    return std::isfinite(x) &&
                           x > 0.0;
                }) ||
            !std::isfinite(min_overlap_) ||
            min_overlap_ <= 0.0 ||
            min_overlap_ > 1.0 ||
            ambiguity_fitness_ratio_ < 1.0)
        {
            throw std::invalid_argument(
                "Invalid revalidation configuration");
        }

        tf_ =
            std::make_unique<tf2_ros::Buffer>(
                get_clock());

        listener_ =
            std::make_unique<
                tf2_ros::TransformListener>(*tf_);

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

        commit_client_ =
            create_client<Commit>(
                "/task_state/commit_workpiece");

        // Subscribe ONLY to the reference accepted
        // by TaskStateServer.
        const auto reference_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        ref_sub_ =
            create_subscription<Reference>(
                "/workpiece_reference/accepted",
                reference_qos,
                [this](Reference::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        mutex_);

                    reference_ = *msg;
                    reference_received_ = true;
                });

        state_sub_ =
            create_subscription<State>(
                "/task_state",
                reference_qos,
                [this](State::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        mutex_);

                    state_ = *msg;
                    state_received_ = Clock::now();
                });

        const auto cloud_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        acquisition_cloud_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/acquisition_cloud",
                cloud_qos);

        target_cloud_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/target_cloud",
                cloud_qos);

        reference_cloud_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/reference_cloud",
                cloud_qos);

        aligned_reference_cloud_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/revalidate_workpiece/aligned_reference_cloud",
                cloud_qos);

        server_ =
            rclcpp_action::create_server<Action>(
                this,
                "revalidate_workpiece",
                [this](const auto&, auto)
                {
                    bool idle = false;

                    return busy_.compare_exchange_strong(
                        idle, true)
                        ? rclcpp_action::GoalResponse::
                            ACCEPT_AND_EXECUTE
                        : rclcpp_action::GoalResponse::
                            REJECT;
                },
                [](auto)
                {
                    return rclcpp_action::CancelResponse::
                        ACCEPT;
                },
                [this](auto handle)
                {
                    if (worker_.joinable())
                        worker_.join();

                    worker_ = std::thread(
                        [this, handle]()
                        {
                            run(handle);
                            busy_.store(false);
                        });
                });

        RCLCPP_INFO(
            get_logger(),
            "RevalidateWorkpiece ready: "
            "object=%s correction=%.1f mm / %.2f deg "
            "fitness_limit=%.3f mm RMS "
            "min_overlap=%.2f "
            "commit=/task_state/commit_workpiece",
            kWorkpieceId,
            config_.max_translation_m * 1000.0,
            degrees(config_.max_rotation_rad),
            rmsMm(config_.max_fitness_m2),
            min_overlap_);
    }

    ~RevalidateServer() override
    {
        if (worker_.joinable())
            worker_.join();
    }

private:
    void publishCloud(
        const Cloud& cloud,
        const rclcpp::Publisher<
            sensor_msgs::msg::PointCloud2>::SharedPtr&
            publisher,
        const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;

        pcl::toROSMsg(cloud, msg);

        msg.header.frame_id = frame_;
        msg.header.stamp = stamp;

        publisher->publish(msg);
    }

    bool getSnapshot(
        Reference& reference,
        State& state)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!reference_received_ ||
            Clock::now() - state_received_ >= 500ms)
        {
            return false;
        }

        reference = reference_;
        state = state_;

        return true;
    }

    bool referenceCurrent(
        uint64_t expected_pose,
        const std::string& expected_id,
        uint64_t expected_geometry,
        uint64_t expected_scene)
    {
        Reference reference;
        State state;

        if (!getSnapshot(reference, state))
            return false;

        const std::string reference_id =
            wp::referenceId(reference);

        return state.scene_valid &&
               state.scene_id == expected_id &&
               reference_id == expected_id &&
               state.geometry_version ==
                   expected_geometry &&
               state.pose_version ==
                   expected_pose &&
               state.scene_version ==
                   expected_scene &&
               !state.motion_fault_active &&
               !state.process_fault_active;
    }

    // Recovery is allowed when workpiece_pose_valid
    // is false. That is precisely what this action
    // is intended to repair.
    bool authorized(
        uint64_t expected_pose,
        const std::string& expected_id,
        uint64_t expected_geometry,
        uint64_t expected_scene)
    {
        Reference reference;
        State state;

        if (!getSnapshot(reference, state))
            return false;

        return referenceCurrent(
                   expected_pose,
                   expected_id,
                   expected_geometry,
                   expected_scene) &&
               state.supervision_valid &&
               state.imu_stable &&
               supervision_->stable();
    }

    void run(
        const std::shared_ptr<Handle>& handle)
    {
        auto result =
            std::make_shared<Action::Result>();

        result->localized = false;
        result->poorly_constrained = false;

        bool completed = false;

        const auto canceled = [&]()
        {
            return handle->is_canceling() ||
                   !rclcpp::ok();
        };

        const auto finish =
            [&](const std::string& message)
        {
            if (completed)
                return;

            completed = true;
            result->message = message;

            RCLCPP_INFO(
                get_logger(),
                "Revalidation: localized=%d "
                "ambiguous=%d rms=%.3f mm "
                "translation=%.3f mm "
                "rotation=%.3f deg: %s",
                result->localized,
                result->poorly_constrained,
                rmsMm(result->registration_fitness),
                result->translation_delta * 1000.0,
                degrees(result->rotation_delta),
                message.c_str());

            if (handle->is_canceling())
            {
                result->localized = false;
                handle->canceled(result);
            }
            else if (result->localized)
            {
                handle->succeed(result);
            }
            else
            {
                handle->abort(result);
            }
        };

        try
        {
            const auto goal =
                handle->get_goal();

            Reference reference;
            State initial_state;

            if (!getSnapshot(
                    reference,
                    initial_state))
            {
                finish(
                    "No fresh TaskState or accepted "
                    "workpiece reference");
                return;
            }

            if (reference.cloud.data.empty() ||
                reference.cloud.header.frame_id !=
                    frame_ ||
                reference.estimate.header.frame_id !=
                    frame_)
            {
                finish(
                    "Accepted reference is empty "
                    "or uses an unexpected frame");
                return;
            }

            const uint64_t expected_geometry =
                initial_state.geometry_version;

            const uint64_t expected_pose =
                goal->expected_pose_version;

            const uint64_t expected_scene =
                initial_state.scene_version;

            const std::string expected_id =
                wp::referenceId(reference);

            if (!referenceCurrent(
                    expected_pose,
                    expected_id,
                    expected_geometry,
                    expected_scene))
            {
                finish(
                    "Accepted reference or expected "
                    "pose version is stale");
                return;
            }

            PlanningSceneInterface scene;

            const auto stored =
                lookupWorkpiece(scene, frame_);

            if (!stored)
            {
                finish(
                    "Authoritative workpiece mesh "
                    "missing or malformed");
                return;
            }

            Eigen::Isometry3d original_estimate =
                Eigen::Isometry3d::Identity();

            tf2::fromMsg(
                reference.estimate.pose,
                original_estimate);

            if (!poseMatches(
                    stored->world_from_mesh,
                    original_estimate,
                    readback_position_tolerance_m_,
                    readback_rotation_tolerance_rad_))
            {
                finish(
                    "MoveIt workpiece pose disagrees "
                    "with accepted reference");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Recovery reference: "
                "geometry=%llu pose=%llu scene=%llu "
                "vertices=%zu",
                static_cast<unsigned long long>(
                    expected_geometry),
                static_cast<unsigned long long>(
                    expected_pose),
                static_cast<unsigned long long>(
                    expected_scene),
                stored->object.meshes.front()
                    .vertices.size());

            // Wait for stable supervision, but do not
            // require the old workpiece pose to be valid.
            const auto supervision_deadline =
                Clock::now() + 10s;

            while (!canceled() &&
                   Clock::now() <
                       supervision_deadline)
            {
                if (!referenceCurrent(
                        expected_pose,
                        expected_id,
                        expected_geometry,
                        expected_scene))
                {
                    finish(
                        "Reference or scene changed "
                        "while waiting for stability");
                    return;
                }

                if (authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry,
                        expected_scene))
                {
                    break;
                }

                std::this_thread::sleep_for(50ms);
            }

            if (canceled())
            {
                finish("Recovery canceled");
                return;
            }

            if (!authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry,
                    expected_scene))
            {
                finish(
                    "Stable platform supervision "
                    "unavailable");
                return;
            }

            // Acquire after the platform has settled.
            const rclcpp::Time after = now();

            const auto acquisition_deadline =
                Clock::now() +
                std::chrono::duration_cast<
                    Clock::duration>(
                    std::chrono::duration<double>(
                        capture_timeout_s_));

            std::optional<hb_perception::Observation>
                observation;

            uint64_t attempts = 0;

            while (!canceled() &&
                   Clock::now() <
                       acquisition_deadline)
            {
                if (!authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry,
                        expected_scene))
                {
                    finish(
                        "Authorization lost during "
                        "RGB-D acquisition");
                    return;
                }

                ++attempts;

                observation =
                    rgbd_->acquireAfter(
                        after, 500ms);

                if (observation)
                    break;
            }

            if (canceled())
            {
                finish("Recovery canceled");
                return;
            }

            if (!observation)
            {
                finish(
                    "Fresh RGB-D acquisition timed out");
                return;
            }

            if (!authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry,
                    expected_scene))
            {
                finish(
                    "Authorization lost after acquisition");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Fresh RGB-D acquired: "
                "attempts=%llu stamp=%.6f",
                static_cast<unsigned long long>(
                    attempts),
                rclcpp::Time(
                    observation->stamp).seconds());

            // Convert acquired observation into
            // the configured world frame.
            auto raw =
                hb_perception::observationToCloud(
                    *observation,
                    range_);

            if (!raw || raw->empty())
            {
                finish("Empty RGB-D observation");
                return;
            }

            const Eigen::Isometry3d camera_pose =
                tf2::transformToEigen(
                    observation->camera_pose);

            Cloud world;
            Cloud reference_cloud;

            pcl::transformPointCloud(
                *raw,
                world,
                camera_pose.matrix().cast<float>());

            pcl::fromROSMsg(
                reference.cloud,
                reference_cloud);

            publishCloud(
                world,
                acquisition_cloud_pub_,
                observation->stamp);

            publishCloud(
                reference_cloud,
                reference_cloud_pub_,
                observation->stamp);

            if (reference_cloud.empty())
            {
                finish("Accepted reference cloud empty");
                return;
            }

            // Crop the observation around the
            // accepted reference with recovery margin.
            Eigen::Vector4f lo, hi;

            pcl::getMinMax3D(
                reference_cloud,
                lo, hi);

            Cloud cropped;

            const float margin =
                static_cast<float>(
                    search_margin_m_);

            for (const auto& point : world)
            {
                if (!pcl::isFinite(point))
                    continue;

                const auto xyz =
                    point.getVector3fMap().array();

                if ((xyz >=
                     lo.head<3>().array() -
                         margin).all() &&
                    (xyz <=
                     hi.head<3>().array() +
                         margin).all())
                {
                    cropped.push_back(point);
                }
            }

            const Cloud source =
                voxelize(
                    reference_cloud, 0.003f);

            const Cloud target =
                voxelize(
                    cropped, 0.003f);

            publishCloud(
                target,
                target_cloud_pub_,
                observation->stamp);

            RCLCPP_INFO(
                get_logger(),
                "Registration clouds: "
                "raw=%zu world=%zu reference=%zu "
                "crop=%zu source=%zu target=%zu",
                raw->size(),
                world.size(),
                reference_cloud.size(),
                cropped.size(),
                source.size(),
                target.size());

            if (source.size() <
                    config_.min_points ||
                target.size() <
                    config_.min_points)
            {
                finish(
                    "Insufficient points for registration");
                return;
            }

            // Multi-seed registration.
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
                "No accepted registration candidate";

            for (std::size_t i = 0;
                 i < seeds.size();
                 ++i)
            {
                if (canceled() ||
                    !authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry,
                        expected_scene))
                {
                    finish(
                        "Interrupted during registration");
                    return;
                }

                Cloud projected;

                pcl::transformPointCloud(
                    source,
                    projected,
                    seeds[i].transform);

                const auto registration =
                    registrar.registerCloud(
                        projected,
                        target);

                if (std::isfinite(
                        registration.fitness_m2) &&
                    registration.fitness_m2 <
                        best_failed_fitness)
                {
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
                            correction.linear())
                            .angle());

                const double displacement =
                    maximumMeshDisplacement(
                        *stored,
                        correction);

                if (displacement >
                        config_.max_translation_m ||
                    rotation >
                        config_.max_rotation_rad)
                {
                    continue;
                }

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
                    displacement;

                candidate.rotation_rad = rotation;
                candidate.seed_index = i;

                candidates.push_back(
                    std::move(candidate));

                RCLCPP_INFO(
                    get_logger(),
                    "Candidate %zu: RMS=%.3f mm "
                    "overlap=%.3f "
                    "displacement=%.3f mm "
                    "rotation=%.3f deg",
                    i,
                    rmsMm(registration.fitness_m2),
                    overlap,
                    displacement * 1000.0,
                    degrees(rotation));
            }

            if (candidates.empty())
            {
                result->registration_fitness =
                    best_failed_fitness;

                finish(
                    "No registration candidate passed: " +
                    best_failed_reason);
                return;
            }

            std::sort(
                candidates.begin(),
                candidates.end(),
                [](const Candidate& a,
                   const Candidate& b)
                {
                    if (std::abs(
                            a.fitness_m2 -
                            b.fitness_m2) > 1e-9)
                    {
                        return a.fitness_m2 <
                               b.fitness_m2;
                    }

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

            result->registration_fitness =
                best.fitness_m2;

            result->translation_delta =
                best.translation_m;

            result->rotation_delta =
                best.rotation_rad;

            // Reject competing plausible corrections.
            bool ambiguous = false;

            const Eigen::Vector3d beam_axis =
                original_estimate.linear()
                    .col(2).normalized();

            for (std::size_t i = 1;
                 i < candidates.size();
                 ++i)
            {
                const auto& alternative =
                    candidates[i];

                if (alternative.fitness_m2 >
                    best.fitness_m2 *
                        ambiguity_fitness_ratio_ +
                    1e-7)
                {
                    continue;
                }

                const Eigen::Isometry3d relative =
                    alternative.correction *
                    best.correction.inverse();

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
                        ambiguity_rotation_rad_)
                {
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
                        degrees(rotation));
                }
            }

            // Explicit axial ambiguity probe for
            // extruded profiles.
            for (double shift : {-0.015, 0.015})
            {
                if (canceled() ||
                    !authorized(
                        expected_pose,
                        expected_id,
                        expected_geometry,
                        expected_scene))
                {
                    finish(
                        "Interrupted during axial "
                        "ambiguity test");
                    return;
                }

                Eigen::Matrix4f offset =
                    Eigen::Matrix4f::Identity();

                offset.block<3, 1>(0, 3) =
                    (beam_axis * shift).cast<float>();

                Cloud shifted;

                pcl::transformPointCloud(
                    source,
                    shifted,
                    offset);

                const auto alternative =
                    registrar.registerCloud(
                        shifted,
                        target);

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
                {
                    continue;
                }

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
                        ambiguity_translation_m_)
                {
                    ambiguous = true;
                }
            }

            if (ambiguous)
            {
                result->poorly_constrained = true;

                finish(
                    "Registration is geometrically "
                    "ambiguous; workpiece unchanged");
                return;
            }

            // Small corrections are treated as
            // unchanged geometry. Do not serialize
            // a sub-tolerance pose correction while
            // claiming the scene was unchanged.
            const double maximum_displacement =
                maximumMeshDisplacement(
                    *stored,
                    best.correction);

            const bool changed =
                maximum_displacement >
                    unchanged_tolerance_m_;

            const Eigen::Isometry3d applied_correction =
                changed
                    ? best.correction
                    : Eigen::Isometry3d::Identity();

            RCLCPP_INFO(
                get_logger(),
                "Recovery decision: changed=%d "
                "max_displacement=%.3f mm "
                "tolerance=%.3f mm",
                changed,
                maximum_displacement * 1000.0,
                unchanged_tolerance_m_ * 1000.0);

            if (canceled() ||
                !authorized(
                    expected_pose,
                    expected_id,
                    expected_geometry,
                    expected_scene))
            {
                finish(
                    "Authorization lost before commit");
                return;
            }

            // Recheck the physical MoveIt object
            // immediately before proposing a commit.
            const auto current_workpiece =
                lookupWorkpiece(scene, frame_);

            if (!current_workpiece ||
                !wp::sameGeometry(
                    current_workpiece->object,
                    stored->object) ||
                !poseMatches(
                    current_workpiece->world_from_mesh,
                    stored->world_from_mesh,
                    readback_position_tolerance_m_,
                    readback_rotation_tolerance_rad_))
            {
                finish(
                    "Workpiece changed during registration");
                return;
            }

            Reference latest_reference;
            State before_commit;

            if (!getSnapshot(
                    latest_reference,
                    before_commit) ||
                before_commit.scene_version !=
                    initial_state.scene_version ||
                wp::referenceId(latest_reference) !=
                    expected_id)
            {
                finish(
                    "Scene or reference changed "
                    "during revalidation");
                return;
            }

            // Build the proposed object. Revalidation
            // does not write MoveIt directly.
            CollisionObject proposed =
                current_workpiece->object;

            const Eigen::Isometry3d corrected_world =
                applied_correction *
                current_workpiece->world_from_mesh;

            const Eigen::Isometry3d corrected_local =
                current_workpiece->object_pose.inverse() *
                corrected_world;

            if (!finiteRigid(corrected_local))
            {
                finish("Invalid recovered mesh transform");
                return;
            }

            proposed.mesh_poses.front() =
                tf2::toMsg(corrected_local);

            proposed.operation =
                CollisionObject::ADD;

            // Transform the reference cloud by
            // exactly the correction proposed to
            // the authoritative collision object.
            Cloud corrected_cloud;

            pcl::transformPointCloud(
                reference_cloud,
                corrected_cloud,
                applied_correction.matrix().cast<float>());

            pcl::toROSMsg(
                corrected_cloud,
                reference.cloud);

            reference.estimate.pose =
                tf2::toMsg(
                    applied_correction *
                    original_estimate);

            reference.estimate.header.stamp = now();

            reference.cloud.header =
                reference.estimate.header;

            // This timestamp is evidence of the
            // new RGB-D acquisition, not the original
            // reference cloud's acquisition.
            reference.cloud.header.stamp =
                observation->stamp;

            reference.relocalized = true;

            reference.expected_pose_version =
                expected_pose;

            std::string validation_error;

            if (!wp::consistent(
                    proposed,
                    reference,
                    validation_error))
            {
                finish(
                    "Recovered mesh/reference mismatch: " +
                    validation_error);
                return;
            }

            auto request =
                std::make_shared<Commit::Request>();

            request->mode =
                Commit::Request::RECOVERY;

            request->expected_geometry_version =
                initial_state.geometry_version;

            request->expected_pose_version =
                initial_state.pose_version;

            request->expected_scene_version =
                initial_state.scene_version;

            request->reference = reference;
            request->collision_object = proposed;

            if (!commit_client_->wait_for_service(3s))
            {
                finish(
                    "TaskState commit service unavailable");
                return;
            }

            auto future =
                commit_client_->async_send_request(
                    request);

            const auto deadline =
                Clock::now() +
                std::chrono::duration_cast<
                    Clock::duration>(
                    std::chrono::duration<double>(
                        commit_timeout_s_));

            bool response_ready = false;

            while (rclcpp::ok() &&
                   Clock::now() < deadline)
            {
                if (future.wait_for(50ms) ==
                    std::future_status::ready)
                {
                    response_ready = true;
                    break;
                }
            }

            if (!response_ready)
            {
                finish(
                    "Recovery commit timed out; "
                    "outcome unknown. Reconcile "
                    "TaskState before retrying");
                return;
            }

            const auto response =
                future.get();

            if (!response->accepted)
            {
                finish(
                    "Recovery commit rejected: " +
                    response->message);
                return;
            }

            const std::string expected_new_id =
                wp::referenceId(reference);

            if (response->reference_id !=
                expected_new_id)
            {
                finish(
                    "Commit returned unexpected "
                    "reference identity");
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Recovery committed: "
                "reference=%s geometry=%llu "
                "pose=%llu scene=%llu changed=%d",
                response->reference_id.c_str(),
                static_cast<unsigned long long>(
                    response->geometry_version),
                static_cast<unsigned long long>(
                    response->pose_version),
                static_cast<unsigned long long>(
                    response->scene_version),
                changed);

            result->estimate =
                reference.estimate;

            result->localized = true;
            result->poorly_constrained = false;

            finish(
                changed
                    ? "Workpiece pose recovered and "
                      "committed by TaskStateServer"
                    : "Fresh observation confirmed "
                      "existing workpiece pose; "
                      "reference committed");
        }
        catch (const std::exception& error)
        {
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
    double commit_timeout_s_ = 10.0;

    hb_perception::RegistrationConfig config_;

    std::mutex mutex_;

    Reference reference_;
    State state_;

    bool reference_received_ = false;

    Clock::time_point state_received_{};

    std::atomic<bool> busy_{false};
    std::thread worker_;

    std::unique_ptr<tf2_ros::Buffer> tf_;

    std::unique_ptr<tf2_ros::TransformListener>
        listener_;

    std::unique_ptr<hb_perception::RGBDAcquisition>
        rgbd_;

    std::unique_ptr<
        hb_robot_skills::PlatformSupervisionClient>
        supervision_;

    rclcpp::Client<Commit>::SharedPtr
        commit_client_;

    rclcpp::Subscription<Reference>::SharedPtr
        ref_sub_;

    rclcpp::Subscription<State>::SharedPtr
        state_sub_;

    rclcpp_action::Server<Action>::SharedPtr
        server_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        acquisition_cloud_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        target_cloud_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        reference_cloud_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr
        aligned_reference_cloud_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RevalidateServer>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
