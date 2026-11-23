
#include "hb_robot_skills/scene_revision_tracker.hpp"
#include "hb_robot_skills/workpiece_commit_validation.hpp"
#include <rmw/types.h>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/msg/profile_estimate.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_interfaces/msg/platform_imu_metrics.hpp>
#include <hb_robot_interfaces/msg/platform_supervision.hpp>
#include <hb_robot_interfaces/srv/commit_workpiece.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hb_robot_perception/imu_stability_monitor.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <condition_variable>

using namespace std::chrono_literals;

using State =
    hb_robot_interfaces::msg::TaskState;

using Reference =
    hb_robot_interfaces::msg::WorkpieceReference;

using Supervision =
    hb_robot_interfaces::msg::PlatformSupervision;

using RosImuMetrics =
    hb_robot_interfaces::msg::PlatformImuMetrics;

using CommitWorkpiece =
    hb_robot_interfaces::srv::CommitWorkpiece;

using CollisionObject =
    moveit_msgs::msg::CollisionObject;

using PlanningScene =
    moveit_msgs::msg::PlanningScene;

using Clock =
    std::chrono::steady_clock;

namespace wp = hb_robot_skills::workpiece;

namespace {

RosImuMetrics toRosMetrics(
    const hb_perception::ImuChannelMetrics& m,
    const std_msgs::msg::Header& header,
    const std::string& name,
    double confidence,
    double coverage)
{
    RosImuMetrics out;

    out.header = header;
    out.sensor_name = name;

    out.valid = m.valid;
    out.stable = m.stable;

    out.sample_count =
        static_cast<uint32_t>(
            std::min<std::size_t>(
                m.samples_count,
                std::numeric_limits<uint32_t>::max()));

    out.sample_age = m.sample_age_s;
    out.reception_gaps = m.reception_gaps;

    const auto copyVector =
        [](
            geometry_msgs::msg::Vector3& dst,
            const std::array<double, 3>& src)
    {
        dst.x = src[0];
        dst.y = src[1];
        dst.z = src[2];
    };

    copyVector(
        out.gyro_mean_rad_s,
        m.gyro_mean);

    copyVector(
        out.gyro_stddev_rad_s,
        m.gyro_stddev);

    copyVector(
        out.accel_mean_m_s2,
        m.accel_mean);

    copyVector(
        out.accel_stddev_m_s2,
        m.accel_stddev);

    out.gyro_residual_rms_rad_s =
        m.ang_vel_rms_rad_s;

    out.accel_residual_rms_m_s2 =
        m.lin_acc_rms_m_s2;

    out.gyro_peak_rad_s =
        m.peak_ang_vel_rad_s;

    out.accel_deviation_peak_m_s2 =
        m.peak_acc_deviation_m_s2;

    out.gyro_latest_rad_s =
        m.latest_ang_vel_rad_s;

    out.accel_deviation_latest_m_s2 =
        m.latest_acc_deviation_m_s2;

    out.accel_mean_norm_m_s2 =
        m.accel_mean_norm_m_s2;

    out.stable_confidence = confidence;
    out.confidence_coverage_s = coverage;

    out.gyro_rms_pass = m.gyro_rms_pass;
    out.accel_rms_pass = m.accel_rms_pass;
    out.mean_gyro_pass = m.mean_gyro_pass;

    return out;
}

}  // namespace

class TaskStateServer : public rclcpp::Node {
public:
    TaskStateServer()
        : rclcpp::Node("task_state_server")
    {
        const uint64_t initial_version =
            static_cast<uint64_t>(
                now().nanoseconds());

        state_.geometry_version =
            initial_version;

        state_.pose_version =
            initial_version;

        state_.scene_valid = false;
        state_.workpiece_pose_valid = false;

        state_.supervision_valid = false;
        state_.imu_stable = false;

        state_.path_valid = false;

        state_.kerf_status =
            State::INVALID_OBSERVATION;

        last_untrusted_ =
            now().nanoseconds();

        rcl_interfaces::msg::ParameterDescriptor
            read_only;

        read_only.read_only = true;

        const auto positive =
            [this, &read_only](
                const std::string& name,
                double fallback)
        {
            const double value =
                declare_parameter<double>(
                    name,
                    fallback,
                    read_only);

            if (!std::isfinite(value) ||
                value <= 0.0)
            {
                throw std::invalid_argument(
                    "Invalid parameter: " + name);
            }

            return value;
        };

        hb_perception::ImuStabilityConfig config;

        config.max_ang_vel_rms =
            positive(
                "platform_imu.max_gyro_rms",
                0.01);

        config.max_lin_acc_rms =
            positive(
                "platform_imu.max_accel_rms",
                0.1);

        config.max_mean_ang_vel =
            positive(
                "platform_imu.max_mean_gyro",
                0.03);

        const double confidence_window_s =
            positive(
                "platform_imu.confidence_window_s",
                2.0);

        config.confidence_window =
            std::chrono::milliseconds(
                static_cast<int64_t>(
                    std::lround(
                        confidence_window_s *
                        1000.0)));

        grace_s_ =
            positive(
                "platform_imu.supervision_grace_s",
                0.8);

        acquire_confidence_ =
            positive(
                "platform_imu.confidence_acquire",
                0.90);

        release_confidence_ =
            positive(
                "platform_imu.confidence_release",
                0.65);

        hard_peak_gyro_ =
            positive(
                "platform_imu.hard_peak_gyro_rad_s",
                0.20);

        hard_peak_accel_ =
            positive(
                "platform_imu.hard_peak_accel_m_s2",
                0.80);

        const auto mpu_bias =
            declare_parameter<std::vector<double>>(
            "platform_imu.mpu6050_gyro_bias",
            {-0.04716386, -0.00346401, 0.05875498},
            read_only);

        const auto mpu_2_bias =
            declare_parameter<std::vector<double>>(
                "platform_imu.mpu6050_2_gyro_bias",
                {-0.07190013, 0.02460354, -0.01190212},
                read_only);

        const auto valid_bias =
            [](const std::vector<double>& bias)
        {
            return bias.size() == 3 &&
                std::all_of(
                    bias.begin(),
                    bias.end(),
                    [](double value) {
                        return std::isfinite(value);
                    });
        };

        if (!valid_bias(mpu_bias) ||
            !valid_bias(mpu_2_bias))
        {
            throw std::invalid_argument(
                "Invalid MPU6050 gyro biases");
        }

        std::copy(
            mpu_bias.begin(),
            mpu_bias.end(),
            config.mpu6050_gyro_bias.begin());

        std::copy(
            mpu_2_bias.begin(),
            mpu_2_bias.end(),
            config.mpu6050_2_gyro_bias.begin());


        if (acquire_confidence_ > 1.0 ||
            release_confidence_ >=
                acquire_confidence_)
        {
            throw std::invalid_argument(
                "Invalid IMU confidence thresholds");
        }

        const auto primary_topic =
            declare_parameter<std::string>(
                "platform_imu_topic",
                "/mpu6050/imu");

        config.auxiliary_accel_topic =
            declare_parameter<std::string>(
                "platform_imu_2_topic",
                "/mpu6050_2/imu",
                read_only);

        if (config.auxiliary_accel_topic.empty())
        {
            throw std::invalid_argument(
                "Two-channel IMU supervision "
                "requires platform_imu_2_topic");
        }

        imu_callbacks_ =
            create_callback_group(
                rclcpp::CallbackGroupType::
                    MutuallyExclusive);

        imu_ =
            std::make_unique<
                hb_perception::ImuStabilityMonitor>(
                *this,
                primary_topic,
                config,
                imu_callbacks_);

        // A separate callback group lets the commit
        // service call MoveIt without blocking the
        // TaskState scene and supervision callbacks.
        commit_callbacks_ =
            create_callback_group(
                rclcpp::CallbackGroupType::
                    MutuallyExclusive);

        const auto transient_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        pub_ =
            create_publisher<State>(
                "/task_state",
                transient_qos);

        supervision_pub_ =
            create_publisher<Supervision>(
                "/platform_supervision",
                rclcpp::QoS(10));

        accepted_reference_pub_ =
            create_publisher<Reference>(
                "/workpiece_reference/accepted",
                transient_qos);

        accepted_estimate_pub_ =
            create_publisher<
                hb_robot_interfaces::msg::ProfileEstimate>(
                "/profile_inspection/accepted_estimate",
                transient_qos);

        // No legacy /workpiece_reference subscription.
        // New workpieces are accepted only through
        // /task_state/commit_workpiece.

        scene_ =
            create_subscription<PlanningScene>(
                "/monitored_planning_scene",
                rclcpp::QoS(10),
                [this](
                    PlanningScene::ConstSharedPtr msg,
                    const rclcpp::MessageInfo& info)
        {
           if (!isAuthoritativeScenePublisher(info))
        return;
           
            onPlanningScene(*msg);
        });

        torch_ =
            create_subscription<
                hb_robot_interfaces::msg::TorchStatus>(
                "/cut_profile/torch_status",
                rclcpp::QoS(10),
                [this](
                    hb_robot_interfaces::msg::
                        TorchStatus::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        mutex_);


                    torch_status_received_ = Clock::now();
                    torch_fault_reported_ = msg->fault;
                    if (msg->fault) {
                        torch_fault_cleared_observed_ = {};

                        const bool was_faulted =
                            state_.process_fault_active;
                        
                        state_.process_fault_active = true;
                        state_.path_valid = false;

                        if (!was_faulted) {
                            RCLCPP_ERROR(
                                get_logger(),
                                "Torch process fault latched; "
                                "cutting disabled, pose retained");
                        }
                    }
                    else if(torch_fault_cleared_observed_ ==
                            Clock::time_point{}) {
                        torch_fault_cleared_observed_ = Clock::now();
                    } 
                });

        fault_ =
            create_subscription<std_msgs::msg::Bool>(
                "/motion_fault",
                rclcpp::QoS(10),
                [this](
                    std_msgs::msg::Bool::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        mutex_);

                    state_.motion_fault_active =
                        msg->data;

                    if (msg->data)
                    {
                        invalidateLocked(
                            "/motion_fault asserted");
                    }
                });

        invalidation_ =
            create_subscription<std_msgs::msg::Bool>(
                "/workpiece_pose_invalidated",
                rclcpp::QoS(10),
                [this](
                    std_msgs::msg::Bool::ConstSharedPtr msg)
                {
                    if (!msg->data)
                        return;

                    std::lock_guard<std::mutex> lock(
                        mutex_);

                    invalidateLocked(
                        "/workpiece_pose_invalidated asserted");
                });

        execution_ =
            create_subscription<
                std_msgs::msg::UInt64>(
                "/cut_profile/execution_version",
                transient_qos,
                [this](
                    std_msgs::msg::UInt64::ConstSharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(
                        mutex_);

                    state_.execution_version =
                        msg->data;

                    state_.kerf_status =
                        State::INVALID_OBSERVATION;
                });

        commit_service_ =
            create_service<CommitWorkpiece>(
                "/task_state/commit_workpiece",
                [this](
                    const std::shared_ptr<
                        CommitWorkpiece::Request> request,
                    std::shared_ptr<
                        CommitWorkpiece::Response> response)
                {
                    commitWorkpiece(request, response);
                },
                rmw_qos_profile_services_default,
                commit_callbacks_);


        
        clear_process_fault_service_ =
            create_service<std_srvs::srv::Trigger>(
                "/task_state/clear_process_fault",
                [this](
                    const std::shared_ptr<
                        std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<
                        std_srvs::srv::Trigger::Response> response)
                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    const auto now_steady = Clock::now();

                    const bool torch_fresh =
                        torch_status_received_ != Clock::time_point{} &&
                        now_steady - torch_status_received_ < 500ms;

                    const bool clear_observed =
                        torch_fault_cleared_observed_ != Clock::time_point{} &&
                        now_steady - torch_fault_cleared_observed_ >= 200ms;

                    if (!torch_fresh ||
                        torch_fault_reported_ ||
                        !clear_observed) {
                        response->success = false;
                        response->message =
                            "Fresh, continuously fault-free torch "
                            "status required before clearing latch";
                        return;
                    }

                    if (state_.motion_fault_active ||
                        !state_.supervision_valid ||
                        !state_.imu_stable ||
                        !state_.workpiece_pose_valid) {
                        response->success = false;
                        response->message =
                            "Motion fault, platform supervision, "
                            "or pose validation prevents clearance";
                        return;
                    }

                    if (!state_.process_fault_active) {
                        response->success = true;
                        response->message =
                            "Process fault already clear";
                        return;
                    }

                    state_.process_fault_active = false;

                    // Do not automatically restore path_valid,
                    // change geometry versions, or authorize motion.
                    publishStateLocked();

                    response->success = true;
                    response->message =
                        "TaskState process fault latch cleared";

                    RCLCPP_WARN(
                        get_logger(),
                        "[PROCESS CLEAR] Process fault latch cleared");
                });


        timer_ =
            create_wall_timer(
                50ms,
                [this]()
                {
                    updateSupervision();
                });

        RCLCPP_INFO(
            get_logger(),
            "TaskStateServer initialized: "
            "IMU=%s + %s "
            "confidence_window=%.2f "
            "acquire=%.2f release=%.2f "
            "grace=%.2f",
            primary_topic.c_str(),
            config.auxiliary_accel_topic.c_str(),
            confidence_window_s,
            acquire_confidence_,
            release_confidence_,
            grace_s_);
    }

private:
    void invalidateLocked(
        const std::string& reason)
    {
        last_untrusted_ =
            now().nanoseconds();

        invalidation_reason_ = reason;
        // state_.process_fault_active = true;
        state_.workpiece_pose_valid = false;
        state_.path_valid = false;

        state_.kerf_status =
            State::INVALID_OBSERVATION;

        RCLCPP_WARN(
            get_logger(),
            "Workpiece pose invalidated: %s",
            reason.c_str());
    }

    bool supervisionTrustedLocked(
        const hb_perception::ImuStabilityMetrics& metrics)
        const
    {

        return metrics.valid &&
               metrics.stable &&
            //    metrics.stable_confidence >=
            //        acquire_confidence_ &&
               trusted_stable_ &&
               !uncertainty_active_ &&
               !supervision_latched_ &&
               state_.supervision_valid &&
               state_.imu_stable &&
               !state_.motion_fault_active &&
               !state_.process_fault_active;
    }

    void publishStateLocked()
    {
        state_.header.stamp = now();
        pub_->publish(state_);
    }

    bool isAuthoritativeScenePublisher(
        const rclcpp::MessageInfo& info)
        {
            const auto& source =
                info.get_rmw_message_info().publisher_gid;

            const auto publishers =
                get_publishers_info_by_topic(
                    "/monitored_planning_scene");

            for (const auto& publisher : publishers)
            {
                const auto& name = publisher.node_name();

                if (name != "move_group" &&
                    name.rfind("move_group_private_", 0) != 0)
                    continue;

                const auto& gid = publisher.endpoint_gid();

                if (gid.size() == RMW_GID_STORAGE_SIZE &&
                    std::equal(
                        gid.begin(),
                        gid.end(),
                        source.data))
                {
                    return true;
                }
            }

            return false;
        }
    void onPlanningScene(
        const PlanningScene& scene
    )
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!scene.is_diff)
        {
            state_.cut_environment_id.clear();
        }

        for (const auto& object :
             scene.world.collision_objects)
        {
            if (object.id.rfind(
                    "cut_environment_", 0) != 0)
                continue;

            if (object.operation ==
                CollisionObject::REMOVE)
            {
                if (state_.cut_environment_id ==
                    object.id)
                {
                    state_.cut_environment_id.clear();
                }
            }
            else
            {
                state_.cut_environment_id =
                    object.id;
            }
        }
        std::string revision_reason;
        if (scene_revisions_.changed(scene, &revision_reason))
        {
            ++state_.scene_version;
            state_.path_valid = false;
            RCLCPP_WARN(
                get_logger(),
                "[SCENE REVISION] revision=%llu "
                "is_diff=%d reason='%s' "
                "environment='%s'",
                static_cast<unsigned long long>(
                    state_.scene_version),
                static_cast<int>(scene.is_diff),
                revision_reason.c_str(),
                state_.cut_environment_id.c_str());

           
         }


        


        if (commit_scene_pending_)
        {
            const auto& objects =
                scene.world.collision_objects;

            const auto it = std::find_if(
                objects.begin(),
                objects.end(),
                [](const CollisionObject& object)
                {
                    return object.id == wp::kObjectId &&
                        object.operation !=
                            CollisionObject::REMOVE;
                });

            if (it != objects.end())
            {
                std::string error;

                if (wp::consistent(
                        *it,
                        pending_reference_,
                        error))
                {
                    commit_scene_observed_ = true;
                    scene_cv_.notify_all();
                }
            }
        }



    }

    void commitWorkpiece(
        const std::shared_ptr<
            CommitWorkpiece::Request> request,
        std::shared_ptr<
            CommitWorkpiece::Response> response)
    {
        std::lock_guard<std::mutex> transaction_lock(
            commit_mutex_);

        response->accepted = false;

        const bool initial =
            request->mode ==
            CommitWorkpiece::Request::INITIAL;

        const bool recovery =
            request->mode ==
            CommitWorkpiece::Request::RECOVERY;

        auto reject =
            [&](const std::string& reason)
        {
            response->message = reason;

            RCLCPP_ERROR(
                get_logger(),
                "Workpiece commit rejected: %s",
                reason.c_str());
        };

        if (!initial && !recovery)
        {
            reject("Unknown workpiece commit mode");
            return;
        }

        const auto& reference =
            request->reference;

        const auto& proposed =
            request->collision_object;

        const int64_t published =
            rclcpp::Time(
                reference.estimate.header.stamp)
                .nanoseconds();

        const int64_t observed =
            rclcpp::Time(
                reference.cloud.header.stamp)
                .nanoseconds();

        const double age =
            (now() -
             rclcpp::Time(
                 reference.estimate.header.stamp))
                .seconds();

        if (published <= 0 ||
            observed <= 0 ||
            observed > published ||
            age < 0.0 ||
            age > 5.0 ||
            reference.cloud.data.empty() ||
            reference.estimate.profile_name.empty())
        {
            reject("Malformed or expired reference");
            return;
        }

        std::string geometry_error;

        if (!wp::consistent(
                proposed,
                reference,
                geometry_error))
        {
            reject(geometry_error);
            return;
        }

        State before;
        std::optional<Reference> old_reference;
        int64_t supervision_epoch = 0;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (published <= last_publication_)
            {
                reject("Superseded reference");
                return;
            }

            before = state_;
            old_reference = accepted_reference_;
            supervision_epoch = last_untrusted_;

            if (recovery)
            {
                if (!old_reference ||
                    !state_.scene_valid ||
                    request->expected_geometry_version !=
                        state_.geometry_version ||
                    request->expected_pose_version !=
                        state_.pose_version ||
                    request->expected_scene_version !=
                        state_.scene_version)
                {
                    reject(
                        "Recovery expected versions "
                        "do not match TaskState");
                    return;
                }

                if (!reference.relocalized ||
                    reference.expected_pose_version !=
                        state_.pose_version)
                {
                    reject(
                        "Invalid recovery reference lineage");
                    return;
                }

                if (reference.estimate.profile_name !=
                    old_reference->estimate.profile_name)
                {
                    reject(
                        "Recovery changed profile identity");
                    return;
                }
            }
            else if (reference.relocalized)
            {
                reject(
                    "Initial inspection marked as recovery");
                return;
            }
        }

        const auto metrics =
            imu_->metrics();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!supervisionTrustedLocked(metrics))
            {
                RCLCPP_ERROR(
                        get_logger(),
                        "Commit raw IMU rejection: "
                        "valid=%d stable=%d confidence=%.3f "
                        "acquire=%.3f coverage=%.3f "
                        "trusted=%d uncertainty=%d latched=%d "
                        "supervision=%d imu=%d, workpiece valid=%d," 
                        "motion fault=%d," 
                        "process fault=%d,"
                        "last_trusted=%lld",
                        static_cast<int>(metrics.valid),
                        static_cast<int>(metrics.stable),
                        metrics.stable_confidence,
                        acquire_confidence_,
                        metrics.confidence_coverage_s,
                        static_cast<int>(trusted_stable_),
                        static_cast<int>(uncertainty_active_),
                        static_cast<int>(supervision_latched_),
                        static_cast<int>(state_.supervision_valid),
                        static_cast<int>(state_.imu_stable),                        
                        static_cast<int>(state_.workpiece_pose_valid),
                        static_cast<int>(state_.motion_fault_active),
                        static_cast<int>(state_.process_fault_active),
                        static_cast<long long>(last_untrusted_));
                
                reject(
                    "Platform supervision not trusted "
                    "at workpiece commit");
                return;
            }

            // INITIAL inspection may contain several
            // individually stable viewpoints separated
            // by intentional robot movement.
            //
            // RECOVERY requires a fresh observation
            // from the current trusted interval.
            if (recovery &&
                observed <= last_untrusted_)
            {
                reject(
                    "Recovery observation predates "
                    "the current trusted interval");
                return;
            }

            if (last_untrusted_ != supervision_epoch)
            {
                reject(
                    "Supervision changed before commit");
                return;
            }
        }

        // Read the existing authoritative object.
        const auto objects =
            planning_scene_interface_.getObjects({
                wp::kObjectId
            });

        const auto old_it =
            objects.find(wp::kObjectId);

        if (recovery)
        {
            if (old_it == objects.end())
            {
                reject("Existing workpiece mesh missing");
                return;
            }

            if (!wp::sameGeometry(
                    old_it->second,
                    proposed))
            {
                reject(
                    "Recovery attempted to change "
                    "workpiece mesh geometry");
                return;
            }

            if (old_it->second.header.frame_id !=
                proposed.header.frame_id)
            {
                reject("Recovery changed mesh frame");
                return;
            }

            std::string old_error;

            if (!wp::consistent(
                    old_it->second,
                    *old_reference,
                    old_error))
            {
                reject(
                    "Existing mesh/reference mismatch: " +
                    old_error);
                return;
            }
        }

        bool pose_changed = true;

        if (old_it != objects.end() &&
            wp::sameGeometry(
                old_it->second,
                proposed))
        {
            pose_changed =
                !wp::poseMatches(
                    wp::meshPose(old_it->second),
                    wp::meshPose(proposed));
        }

        // Prevent cutting while the scene is updated.
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (state_.geometry_version !=
                    before.geometry_version ||
                state_.pose_version !=
                    before.pose_version ||
                state_.scene_version !=
                    before.scene_version ||
                last_untrusted_ != supervision_epoch ||
                !state_.supervision_valid ||
                !state_.imu_stable ||
                state_.motion_fault_active ||
                state_.process_fault_active)
            {
                reject("TaskState changed before scene update");
                return;
            }

            state_.workpiece_pose_valid = false;
            state_.path_valid = false;

            invalidation_reason_ =
                "Workpiece commit in progress";

            publishStateLocked();
        }

        std::vector<CollisionObject> updates;

        if (initial)
        {
            CollisionObject remove_temporary;

            remove_temporary.header.frame_id =
                proposed.header.frame_id;

            remove_temporary.id =
                "inspection_workpiece";

            remove_temporary.operation =
                CollisionObject::REMOVE;

            updates.push_back(
                std::move(remove_temporary));
        }

        CollisionObject updated = proposed;
        updated.operation = CollisionObject::ADD;

        updates.push_back(
            std::move(updated));

        {
            std::lock_guard<std::mutex> lock(mutex_);

            commit_scene_pending_ = true;
            commit_scene_observed_ = false;
            pending_reference_ = reference;
        }

        if (!planning_scene_interface_
                 .applyCollisionObjects(updates))
        {
            reject("MoveIt rejected collision update");

            {
            std::unique_lock<std::mutex> lock(mutex_);

            commit_scene_pending_ = false;
            commit_scene_observed_ = false;

            }

            return;
        }

        // Verify the stored MoveIt object, not
        // the locally proposed object.
        const auto stored =
            planning_scene_interface_.getObjects({
                wp::kObjectId,
                "inspection_workpiece"
            });

        const auto verified =
            stored.find(wp::kObjectId);

        std::string readback_error;

        const bool mesh_verified =
            verified != stored.end() &&
            wp::sameGeometry(
                verified->second,
                proposed) &&
            wp::consistent(
                verified->second,
                reference,
                readback_error);

        const bool temporary_removed =
            !initial ||
            stored.find("inspection_workpiece") ==
                stored.end();

        if (!mesh_verified || !temporary_removed)
        {
            reject(
                !temporary_removed
                    ? "Temporary inspection object "
                      "still present"
                    : "MoveIt workpiece readback failed: " +
                      readback_error);
            {
            std::unique_lock<std::mutex> lock(mutex_);

            commit_scene_pending_ = false;
            commit_scene_observed_ = false;

            }

            // Execution stays invalidated.
            return;
        }

        {
            std::unique_lock<std::mutex> lock(mutex_);

            const bool synchronized =
                scene_cv_.wait_for(
                    lock,
                    3s,
                    [&]()
                    {
                        return commit_scene_observed_;
                    });

            commit_scene_pending_ = false;
            commit_scene_observed_ = false;

            if (!synchronized)
            {
                reject(
                    "MoveIt readback succeeded, but "
                    "monitored scene synchronization "
                    "was not confirmed");
                return;
            }
        }


        // Recheck supervision after MoveIt update.
        const auto final_metrics =
            imu_->metrics();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!supervisionTrustedLocked(
                    final_metrics) ||
                last_untrusted_ != supervision_epoch ||
                state_.geometry_version !=
                    before.geometry_version ||
                state_.pose_version !=
                    before.pose_version ||
                state_.motion_fault_active ||
                state_.process_fault_active)
            {
                reject(
                    "Supervision or workpiece versions "
                    "changed during commit");

                // The scene may have changed.
                // Do not restore execution permission.
                return;
            }

            // Only TaskState advances these versions.
            if (initial)
                ++state_.geometry_version;

            if (initial || pose_changed)
                ++state_.pose_version;

            accepted_reference_ = reference;

            measured_pose_ =
                reference.estimate.pose;

            last_publication_ = published;

            state_.scene_id =
                wp::referenceId(reference);

            state_.scene_valid = true;
            state_.workpiece_pose_valid = true;
            state_.path_valid = false;

            invalidation_reason_.clear();

            accepted_reference_pub_->publish(
                reference);

            accepted_estimate_pub_->publish(
                reference.estimate);

            publishStateLocked();

            response->accepted = true;

            response->message =
                "Workpiece committed and verified";

            response->reference_id =
                state_.scene_id;

            response->geometry_version =
                state_.geometry_version;

            response->pose_version =
                state_.pose_version;

            response->scene_version =
                state_.scene_version;

            RCLCPP_INFO(
                get_logger(),
                "Workpiece committed: "
                "id=%s geometry=%llu pose=%llu "
                "scene=%llu initial=%d",
                state_.scene_id.c_str(),
                static_cast<unsigned long long>(
                    state_.geometry_version),
                static_cast<unsigned long long>(
                    state_.pose_version),
                static_cast<unsigned long long>(
                    state_.scene_version),
                initial);
        }
    }

    
void updateSupervision()
{
    using Seconds = std::chrono::duration<double>;

    const auto metrics = imu_->metrics();
    const auto tick = Clock::now();

    // ---------------------------------------------------------
    // 1. Evaluate sensor evidence.
    // ---------------------------------------------------------

    const bool mpu6050_2_good =
        metrics.mpu6050_2.valid &&
        metrics.mpu6050_2.stable;

    const bool mpu6050_good =
        metrics.mpu6050.valid &&
        metrics.mpu6050.stable;

    const bool both_good =
        mpu6050_2_good && mpu6050_good;

    const bool one_good =
        mpu6050_2_good != mpu6050_good;

    const bool neither_good =
        !mpu6050_2_good && !mpu6050_good;

    const bool mpu6050_2_hard =
        metrics.mpu6050_2.valid &&
        (metrics.mpu6050_2.peak_ang_vel_rad_s >= hard_peak_gyro_ ||
         metrics.mpu6050_2.peak_acc_deviation_m_s2 >= hard_peak_accel_);

    const bool mpu6050_hard =
        metrics.mpu6050.valid &&
        (metrics.mpu6050.peak_ang_vel_rad_s >= hard_peak_gyro_ ||
         metrics.mpu6050.peak_acc_deviation_m_s2 >= hard_peak_accel_);

    // Preserve existing dual-confirmation hard-motion policy.

    const bool both_gyro_hard =
    metrics.mpu6050.valid &&
    metrics.mpu6050_2.valid &&
    metrics.mpu6050.peak_ang_vel_rad_s >= hard_peak_gyro_ &&
    metrics.mpu6050_2.peak_ang_vel_rad_s >= hard_peak_gyro_;

    const bool both_accel_hard =
    metrics.mpu6050.valid &&
    metrics.mpu6050_2.valid &&
    metrics.mpu6050.peak_acc_deviation_m_s2 >= hard_peak_accel_ &&
    metrics.mpu6050_2.peak_acc_deviation_m_s2 >= hard_peak_accel_;

    const bool hard_motion =
        both_gyro_hard || both_accel_hard;
    // const bool hard_motion =
    //     mpu6050_2_hard && mpu6050_hard;

    const bool both_stale =
        metrics.mpu6050_2.sample_age_s > grace_s_ &&
        metrics.mpu6050.sample_age_s > grace_s_;

    const bool confidence_finite =
        std::isfinite(metrics.stable_confidence);

    const bool confidence_acquired =
        confidence_finite &&
        metrics.stable_confidence >= acquire_confidence_;

    const bool confidence_lost =
        !confidence_finite ||
        metrics.stable_confidence < release_confidence_;

    State published;
    Supervision diagnostic;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // -----------------------------------------------------
        // 2. Capture previous state BEFORE changing anything.
        // -----------------------------------------------------

        const bool was_supervised =
            state_.supervision_valid;

        const bool was_trusted =
            trusted_stable_;

        const bool was_latched =
            supervision_latched_;

        std::string transition_reason = "unchanged";

        // -----------------------------------------------------
        // 3. Update the internal supervision state machine.
        //
        // Priority:
        //   A. Confirmed hard motion
        //   B. Both sensors healthy
        //   C. Exactly one sensor healthy
        //   D. Neither sensor healthy
        //
        // Authorization is calculated AFTER this entire block.
        // -----------------------------------------------------

        
// -----------------------------------------------------
// 3. Update supervision state.
// -----------------------------------------------------

if (hard_motion)
{
    RCLCPP_WARN(
            get_logger(),
            "[IMU HARD] "
            "mpu1 gyro=%.4f accel=%.4f "
            "mpu2 gyro=%.4f accel=%.4f",
            metrics.mpu6050.peak_ang_vel_rad_s,
            metrics.mpu6050.peak_acc_deviation_m_s2,
            metrics.mpu6050_2.peak_ang_vel_rad_s,
            metrics.mpu6050_2.peak_acc_deviation_m_s2);
    if (!supervision_latched_)
    {
        invalidateLocked(
            "Hard IMU motion threshold exceeded");
    }

    trusted_stable_ = false;
    supervision_latched_ = true;
    uncertainty_active_ = false;
    single_channel_active_ = false;

        transition_reason = "confirmed_hard_motion";
        }
        else if (both_stale)
        {
            trusted_stable_ = false;
            supervision_latched_ = true;
            uncertainty_active_ = false;
            single_channel_active_ = false;

            transition_reason = "both_channels_stale";
        }
        else if (both_good)
        {
            single_channel_active_ = false;
            uncertainty_active_ = false;

            if (confidence_acquired)
            {
                // Initial authorization or recovery after a latch
                // still requires the existing confidence threshold.
                trusted_stable_ = true;
                supervision_latched_ = false;

                transition_reason = "confidence_acquired";
            }
            else
            {
                // Both channels are good NOW.
                // A historical confidence dip must not revoke
                // previously acquired authorization.
                //
                // Do not clear an existing latch here.
                transition_reason =
                    "both_good_preserve_authorization";
            }
        }
        else if (one_good)
        {
            // One valid, stable IMU is sufficient to preserve
            // authorization already established previously.
            //
            // No timeout here: this is genuine redundancy,
            // rather than a short grace interval.

            uncertainty_active_ = false;
            single_channel_active_ = true;

            if (trusted_stable_ && !supervision_latched_)
            {
                transition_reason =
                    mpu6050_2_good
                        ? "mpu6050_2_maintaining_supervision"
                        : "mpu6050_maintaining_supervision";
            }
            else
            {
                // A single channel cannot establish NEW
                // authorization or clear a previous latch.
                transition_reason =
                    "single_channel_cannot_acquire";
            }
        }
        else
        {
            // Neither channel currently provides acceptable
            // stability evidence.

            single_channel_active_ = false;

            if (!uncertainty_active_)
            {
                uncertainty_active_ = true;
                uncertainty_since_ = tick;
            }

            const double uncertain_for =
                Seconds(tick - uncertainty_since_).count();

            if (uncertain_for >= grace_s_)
            {
                trusted_stable_ = false;
                supervision_latched_ = true;
                uncertainty_active_ = false;

                transition_reason =
                    "no_stable_channels_grace_expired";
            }
            else
            {
                transition_reason =
                    "no_stable_channels_grace_active";
            }
        }

        // -----------------------------------------------------
        // 4. Final authorization.
        // -----------------------------------------------------

        state_.supervision_valid =
            trusted_stable_ &&
            !supervision_latched_ &&
            !hard_motion &&
            !both_stale;

        state_.imu_stable = state_.supervision_valid;


        // -----------------------------------------------------
        // 5. Log actual authorization transitions.
        // -----------------------------------------------------

        if (was_supervised != state_.supervision_valid ||
            was_trusted != trusted_stable_ ||
            was_latched != supervision_latched_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[IMU AUTH TRANSITION] "
                "authorized=%d->%d "
                "trusted=%d->%d "
                "latched=%d->%d "
                "reason=%s "
                "mpu6050_2_good=%d mpu6050_good=%d "
                "mpu6050_2_hard=%d mpu6050_hard=%d "
                "confidence=%.3f "
                "acquire=%.3f release=%.3f "
                "coverage_s=%.3f "
                "both_stale=%d "
                "single_channel=%d uncertainty=%d",
                static_cast<int>(was_supervised),
                static_cast<int>(state_.supervision_valid),
                static_cast<int>(was_trusted),
                static_cast<int>(trusted_stable_),
                static_cast<int>(was_latched),
                static_cast<int>(supervision_latched_),
                transition_reason.c_str(),
                static_cast<int>(mpu6050_2_good),
                static_cast<int>(mpu6050_good),
                static_cast<int>(mpu6050_2_hard),
                static_cast<int>(mpu6050_hard),
                metrics.stable_confidence,
                acquire_confidence_,
                release_confidence_,
                metrics.confidence_coverage_s,
                static_cast<int>(both_stale),
                static_cast<int>(single_channel_active_),
                static_cast<int>(uncertainty_active_));
        }

        // -----------------------------------------------------
        // 6. Build status from the FINAL state.
        // -----------------------------------------------------

        std::ostringstream status;

        status << std::fixed
               << std::setprecision(3)
               << "confidence="
               << metrics.stable_confidence
               << " coverage_s="
               << metrics.confidence_coverage_s
               << " raw_valid="
               << metrics.valid
               << " raw_stable="
               << metrics.stable
               << " mpu6050_2_good="
               << mpu6050_2_good
               << " mpu6050_good="
               << mpu6050_good
               << " authorized="
               << state_.supervision_valid
               << " reason="
               << transition_reason;

        if (hard_motion)
        {
            status << " HARD_MOTION";
        }
        else if (supervision_latched_)
        {
            status << " UNTRUSTED";
        }
        else if (uncertainty_active_)
        {
            status << " UNCERTAIN";
        }
        else if (state_.supervision_valid)
        {
            status << " STABLE";
        }
        else
        {
            status << " NOT_AUTHORIZED";
        }

        if (!state_.workpiece_pose_valid &&
            !invalidation_reason_.empty())
        {
            status << "; pose invalidated: "
                   << invalidation_reason_;
        }

        state_.status_message = status.str();

        state_.header.stamp = now();

        // -----------------------------------------------------
        // 7. Construct consistent TaskState and diagnostics.
        // -----------------------------------------------------

        published = state_;

        diagnostic.header = published.header;

        diagnostic.mpu6050_2 =
            toRosMetrics(
                metrics.mpu6050_2,
                diagnostic.header,
                "mpu6050_2",
                metrics.stable_confidence,
                metrics.confidence_coverage_s);

        diagnostic.mpu6050 =
            toRosMetrics(
                metrics.mpu6050,
                diagnostic.header,
                "mpu6050",
                metrics.stable_confidence,
                metrics.confidence_coverage_s);

        diagnostic.combined_confidence =
            metrics.stable_confidence;

        diagnostic.combined_coverage_s =
            metrics.confidence_coverage_s;

        diagnostic.both_channels_valid =
            metrics.valid;

        diagnostic.both_channels_stable =
            metrics.stable;

        diagnostic.supervision_valid =
            state_.supervision_valid;

        diagnostic.imu_stable =
            state_.imu_stable;

        diagnostic.hard_motion_detected =
            hard_motion;

        if (hard_motion || supervision_latched_)
        {
            diagnostic.state = Supervision::UNTRUSTED;
        }
        else if (uncertainty_active_)
        {
            diagnostic.state = Supervision::UNCERTAIN;
        }
        else if (state_.supervision_valid)
        {
            diagnostic.state = Supervision::STABLE;
        }
        else
        {
            diagnostic.state = Supervision::UNKNOWN;
        }

        diagnostic.reason =
            state_.status_message;
    }

        // ---------------------------------------------------------
        // 8. Publish completed snapshots.
        // ---------------------------------------------------------

        pub_->publish(published);
        supervision_pub_->publish(diagnostic);
    }


    // Shared authoritative state.
    std::mutex mutex_;
    std::mutex commit_mutex_;

    std::condition_variable scene_cv_;
    bool commit_scene_pending_ = false;
    bool commit_scene_observed_ = false;
    Reference pending_reference_;

    State state_;

    std::optional<Reference> accepted_reference_;

    hb_robot_skills::SceneRevisionTracker
        scene_revisions_;

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface_;

    // IMU supervision.
    rclcpp::CallbackGroup::SharedPtr
        imu_callbacks_;

    rclcpp::CallbackGroup::SharedPtr
        commit_callbacks_;

    std::unique_ptr<
        hb_perception::ImuStabilityMonitor> imu_;

    double acquire_confidence_ = 0.90;
    double release_confidence_ = 0.5;
    double grace_s_ = 1.20;

    double hard_peak_gyro_ = 0.20;
    double hard_peak_accel_ = 0.80;

    bool trusted_stable_ = false;
    bool uncertainty_active_ = false;
    bool supervision_latched_ = false;

    Clock::time_point uncertainty_since_{};

    // Workpiece identity.
    int64_t last_publication_ = 0;
    int64_t last_untrusted_ = 0;

    Clock::time_point single_channel_since_{};
    bool single_channel_active_{false};

    geometry_msgs::msg::Pose measured_pose_;
    std::string invalidation_reason_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    clear_process_fault_service_;

    Clock::time_point torch_status_received_{};
    Clock::time_point torch_fault_cleared_observed_{};

    bool torch_fault_reported_ = true;



    // Publishers.
    rclcpp::Publisher<State>::SharedPtr pub_;

    rclcpp::Publisher<Supervision>::SharedPtr
        supervision_pub_;

    rclcpp::Publisher<Reference>::SharedPtr
        accepted_reference_pub_;

    rclcpp::Publisher<
        hb_robot_interfaces::msg::ProfileEstimate>::
        SharedPtr accepted_estimate_pub_;

    // Subscriptions.
    rclcpp::Subscription<PlanningScene>::SharedPtr
        scene_;

    rclcpp::Subscription<
        hb_robot_interfaces::msg::TorchStatus>::
        SharedPtr torch_;

    rclcpp::Subscription<std_msgs::msg::Bool>::
        SharedPtr fault_, invalidation_;

    rclcpp::Subscription<std_msgs::msg::UInt64>::
        SharedPtr execution_;

    // Service and timer.
    rclcpp::Service<CommitWorkpiece>::SharedPtr
        commit_service_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TaskStateServer>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 3);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
