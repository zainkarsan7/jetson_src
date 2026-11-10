
#include "hb_robot_skills/scene_revision_tracker.hpp"

#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_interfaces/msg/platform_imu_metrics.hpp>
#include <hb_robot_interfaces/msg/platform_supervision.hpp>

#include <hb_robot_perception/imu_stability_monitor.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using State = hb_robot_interfaces::msg::TaskState;
using Supervision =
    hb_robot_interfaces::msg::PlatformSupervision;
using RosImuMetrics =
    hb_robot_interfaces::msg::PlatformImuMetrics;
using ChannelMetrics =
    hb_perception::ImuChannelMetrics;

using Clock = std::chrono::steady_clock;

namespace {

RosImuMetrics toRosMetrics(
    const ChannelMetrics& m,
    const std_msgs::msg::Header& header,
    const std::string& name,
    double confidence,
    double coverage) {

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

  auto copyVector = [](
      geometry_msgs::msg::Vector3& dst,
      const std::array<double, 3>& src) {
    dst.x = src[0];
    dst.y = src[1];
    dst.z = src[2];
  };

  copyVector(out.gyro_mean_rad_s, m.gyro_mean);
  copyVector(out.gyro_stddev_rad_s, m.gyro_stddev);
  copyVector(out.accel_mean_m_s2, m.accel_mean);
  copyVector(out.accel_stddev_m_s2, m.accel_stddev);

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
      : Node("task_state_server") {

    state_.geometry_version =
        static_cast<uint64_t>(
            now().nanoseconds());

    state_.pose_version =
        state_.geometry_version;

    // Initial state must not authorize execution.
    state_.scene_valid = false;
    state_.workpiece_pose_valid = false;
    state_.supervision_valid = false;
    state_.imu_stable = false;
    state_.path_valid = false;

    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;

    auto positive = [&](const std::string& name,
                        double fallback) {
      const double value =
          declare_parameter<double>(
              name, fallback, read_only);

      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(
            "Invalid parameter: " + name);

      return value;
    };

    hb_perception::ImuStabilityConfig config;

    config.max_ang_vel_rms =
        positive("platform_imu.max_gyro_rms", 0.01);

    config.max_lin_acc_rms =
        positive("platform_imu.max_accel_rms", 0.1);

    config.max_mean_ang_vel =
        positive("platform_imu.max_mean_gyro", 0.03);

    const double confidence_window_s =
        positive(
            "platform_imu.confidence_window_s", 2.0);

    config.confidence_window =
        std::chrono::milliseconds(
            static_cast<int64_t>(
                std::lround(
                    confidence_window_s * 1000.0)));

    grace_s_ = positive(
        "platform_imu.supervision_grace_s", 0.8);

    acquire_confidence_ = positive(
        "platform_imu.confidence_acquire", 0.90);

    release_confidence_ = positive(
        "platform_imu.confidence_release", 0.65);

    hard_peak_gyro_ = positive(
        "platform_imu.hard_peak_gyro_rad_s", 0.20);

    hard_peak_accel_ = positive(
        "platform_imu.hard_peak_accel_m_s2", 0.80);

    const auto bias =
        declare_parameter<std::vector<double>>(
            "platform_imu.mpu6050_gyro_bias",
            {-0.0787, 0.0088, -0.0031});

    if (bias.size() != 3 ||
        !std::all_of(
            bias.begin(),
            bias.end(),
            [](double v) {
              return std::isfinite(v);
            })) {
      throw std::invalid_argument(
          "Invalid MPU6050 gyro bias");
    }

    std::copy(
        bias.begin(),
        bias.end(),
        config.mpu6050_gyro_bias.begin());

    if (acquire_confidence_ > 1.0 ||
        release_confidence_ >= acquire_confidence_) {
      throw std::invalid_argument(
          "Invalid confidence thresholds");
    }

    const auto primary_topic =
        declare_parameter<std::string>(
            "platform_imu_topic",
            "/hb_robot_lidar/imu");

    config.auxiliary_accel_topic =
        declare_parameter<std::string>(
            "platform_accel_topic",
            "/mpu6050/imu",
            read_only);

    if (config.auxiliary_accel_topic.empty())
      throw std::invalid_argument(
          "Two-channel supervision requires "
          "platform_accel_topic");

    imu_callbacks_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    imu_ = std::make_unique<
        hb_perception::ImuStabilityMonitor>(
        *this,
        primary_topic,
        config,
        imu_callbacks_);

    pub_ = create_publisher<State>(
        "/task_state",
        rclcpp::QoS(1).transient_local());

    supervision_pub_ =
        create_publisher<Supervision>(
            "/platform_supervision",
            rclcpp::QoS(10));

    RCLCPP_INFO(
        get_logger(),
        "Platform IMU: %s + %s; "
        "confidence_window=%.2f acquire=%.2f "
        "release=%.2f grace=%.2f",
        primary_topic.c_str(),
        config.auxiliary_accel_topic.c_str(),
        confidence_window_s,
        acquire_confidence_,
        release_confidence_,
        grace_s_);

    ref_ = create_subscription<
        hb_robot_interfaces::msg::WorkpieceReference>(
        "/workpiece_reference",
        rclcpp::QoS(1).transient_local(),
        [this](
            hb_robot_interfaces::msg::
                WorkpieceReference::ConstSharedPtr m) {

          const auto metrics = imu_->metrics();

          const auto stamp =
              rclcpp::Time(
                  m->estimate.header.stamp);

          const auto cloud_stamp =
              rclcpp::Time(
                  m->cloud.header.stamp);

          const double age =
              (now() - stamp).seconds();

          const int64_t published =
              stamp.nanoseconds();

          const int64_t observed =
              cloud_stamp.nanoseconds();

          if (age < 0.0 ||
              age > 5.0 ||
              published <= last_publication_ ||
              observed <= 0 ||
              observed > published ||
              m->cloud.data.empty() ||
              m->estimate.profile_name.empty() ||
              m->estimate.header.frame_id.empty() ||
              m->estimate.header.frame_id !=
                  m->cloud.header.frame_id) {

            RCLCPP_WARN(
                get_logger(),
                "Rejected malformed, expired or "
                "superseded workpiece reference");
            return;
          }

          if (m->relocalized &&
              (!state_.scene_valid ||
               m->expected_pose_version !=
                   state_.pose_version)) {

            RCLCPP_WARN(
                get_logger(),
                "Rejected revalidation: expected pose "
                "version mismatch");
            return;
          }

          // Require the observation to belong to the
          // current trusted interval.
          //
          // The raw metrics are checked here because
          // this node is the supervision authority.
          // Downstream skills consume PlatformSupervision.
          //
          // Reception gap counters are not compared with
          // previous 50 ms ticks: that comparison does not
          // prove observation-window continuity.
          const bool observed_after_untrusted =
              observed > last_untrusted_;

          const bool platform_confirmed =
              metrics.valid &&
              metrics.stable &&
              metrics.stable_confidence >=
                  acquire_confidence_ &&
              trusted_stable_ &&
              !uncertainty_active_ &&
              !supervision_latched_ &&
              !state_.motion_fault_active &&
              !state_.process_fault_active &&
              observed_after_untrusted;

          if (!platform_confirmed) {
            RCLCPP_WARN(
                get_logger(),
                "Reference not confirmed: "
                "valid=%d stable=%d confidence=%.3f "
                "trusted=%d uncertain=%d latched=%d "
                "observed_after_untrusted=%d "
                "uni_gaps=%llu mpu_gaps=%llu",
                metrics.valid,
                metrics.stable,
                metrics.stable_confidence,
                trusted_stable_,
                uncertainty_active_,
                supervision_latched_,
                observed_after_untrusted,
                static_cast<unsigned long long>(
                    metrics.unilidar.reception_gaps),
                static_cast<unsigned long long>(
                    metrics.mpu6050.reception_gaps));
          }

          if (!m->relocalized)
            ++state_.geometry_version;

          if (!m->relocalized ||
              m->estimate.pose != measured_pose_)
            ++state_.pose_version;

          measured_pose_ = m->estimate.pose;
          last_publication_ = published;

          state_.scene_valid = true;
          state_.scene_id =
              std::to_string(published);

          state_.workpiece_pose_valid =
              platform_confirmed;

          state_.path_valid = false;

          invalidation_reason_ =
              platform_confirmed
                  ? ""
                  : "reference requires a fresh "
                    "supervised observation";

          RCLCPP_INFO(
              get_logger(),
              "Reference accepted: id=%s "
              "confirmed=%d geometry=%llu pose=%llu",
              state_.scene_id.c_str(),
              platform_confirmed,
              static_cast<unsigned long long>(
                  state_.geometry_version),
              static_cast<unsigned long long>(
                  state_.pose_version));
        });

    scene_ = create_subscription<
        moveit_msgs::msg::PlanningScene>(
        "/monitored_planning_scene",
        10,
        [this](
            moveit_msgs::msg::
                PlanningScene::ConstSharedPtr m) {

          if (!m->is_diff)
            state_.cut_environment_id.clear();

          for (const auto& object :
               m->world.collision_objects) {

            if (object.id.rfind(
                    "cut_environment_", 0) != 0)
              continue;

            if (object.operation ==
                object.REMOVE) {

              if (state_.cut_environment_id ==
                  object.id)
                state_.cut_environment_id.clear();

            } else {
              state_.cut_environment_id = object.id;
            }
          }

          if (scene_revisions_.changed(*m)) {
            ++state_.scene_version;
            state_.path_valid = false;

            std::ostringstream objects;

            for (const auto& object :
                 m->world.collision_objects) {
              objects << " " << object.id
                      << "(op="
                      << static_cast<int>(
                             object.operation)
                      << ")";
            }

            RCLCPP_INFO(
                get_logger(),
                "Scene revision=%llu "
                "is_diff=%d environment=%s "
                "objects=[%s] attached=%zu "
                "octomap_bytes=%zu fixed_tf=%zu "
                "acm_entries=%zu",
                static_cast<unsigned long long>(
                    state_.scene_version),
                m->is_diff,
                state_.cut_environment_id.c_str(),
                objects.str().c_str(),
                m->robot_state
                    .attached_collision_objects.size(),
                m->world.octomap.octomap.data.size(),
                m->fixed_frame_transforms.size(),
                m->allowed_collision_matrix
                    .entry_names.size());
          }
        });

    torch_ = create_subscription<
        hb_robot_interfaces::msg::TorchStatus>(
        "/cut_profile/torch_status",
        10,
        [this](
            hb_robot_interfaces::msg::
                TorchStatus::ConstSharedPtr m) {
          state_.process_fault_active = m->fault;

          if (m->fault)
            invalidate("Torch process fault");
        });

    fault_ =
        create_subscription<std_msgs::msg::Bool>(
            "/motion_fault",
            10,
            [this](
                std_msgs::msg::Bool::ConstSharedPtr m) {
              state_.motion_fault_active = m->data;

              if (m->data)
                invalidate("/motion_fault asserted");
            });

    invalidation_ =
        create_subscription<std_msgs::msg::Bool>(
            "/workpiece_pose_invalidated",
            10,
            [this](
                std_msgs::msg::Bool::ConstSharedPtr m) {
              if (m->data)
                invalidate(
                    "/workpiece_pose_invalidated asserted");
            });

    execution_ =
        create_subscription<std_msgs::msg::UInt64>(
            "/cut_profile/execution_version",
            rclcpp::QoS(1).transient_local(),
            [this](
                std_msgs::msg::UInt64::ConstSharedPtr m) {
              state_.execution_version = m->data;
              state_.kerf_status =
                  State::INVALID_OBSERVATION;
            });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this] {
          updateSupervision();
        });
  }

private:
  void invalidate(const std::string& reason) {
    last_untrusted_ = now().nanoseconds();

    if (state_.workpiece_pose_valid) {
      RCLCPP_WARN(
          get_logger(),
          "Workpiece pose invalidated "
          "(pose_version=%llu): %s",
          static_cast<unsigned long long>(
              state_.pose_version),
          reason.c_str());
    }

    invalidation_reason_ = reason;

    state_.workpiece_pose_valid = false;
    state_.path_valid = false;
    state_.kerf_status =
        State::INVALID_OBSERVATION;
  }

  void updateSupervision() {
    const auto m = imu_->metrics();
    const auto tick = Clock::now();

    const bool raw_good =
        m.valid && m.stable;

    const bool unilidar_hard =
        m.unilidar.valid &&
        (m.unilidar.peak_ang_vel_rad_s >=
             hard_peak_gyro_ ||
         m.unilidar.peak_acc_deviation_m_s2 >=
             hard_peak_accel_);

    const bool mpu6050_hard =
        m.mpu6050.valid &&
        (m.mpu6050.peak_ang_vel_rad_s >=
             hard_peak_gyro_ ||
         m.mpu6050.peak_acc_deviation_m_s2 >=
             hard_peak_accel_);

    const bool hard_motion =
        unilidar_hard || mpu6050_hard;

    const bool hard_stale =
        m.unilidar.sample_age_s > grace_s_ ||
        m.mpu6050.sample_age_s > grace_s_;

    const bool confidence_acquired =
        raw_good &&
        m.stable_confidence >=
            acquire_confidence_;

    const bool confidence_lost =
        m.stable_confidence <
            release_confidence_;

    if (hard_motion) {
      if (!supervision_latched_)
        invalidate(
            "Hard IMU motion threshold exceeded");

      trusted_stable_ = false;
      uncertainty_active_ = false;
      supervision_latched_ = true;

    } else if (!trusted_stable_) {
      if (confidence_acquired && !hard_stale) {
        trusted_stable_ = true;
        uncertainty_active_ = false;
        supervision_latched_ = false;

        RCLCPP_INFO(
            get_logger(),
            "Platform supervision acquired: "
            "confidence=%.3f",
            m.stable_confidence);
      }

    } else if (!raw_good ||
               confidence_lost ||
               hard_stale) {

      if (!uncertainty_active_) {
        uncertainty_active_ = true;
        uncertainty_since_ = tick;

        RCLCPP_WARN_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Platform supervision UNCERTAIN: "
    "confidence=%.3f raw_good=%d "
    "uni_valid=%d uni_stable=%d "
    "mpu_valid=%d mpu_stable=%d",
    m.stable_confidence,
    raw_good,
    m.unilidar.valid,
    m.unilidar.stable,
    m.mpu6050.valid,
    m.mpu6050.stable);
      }

      const double uncertain_for =
          std::chrono::duration<double>(
              tick - uncertainty_since_).count();

      if (hard_stale ||
          uncertain_for >= grace_s_) {

        if (!supervision_latched_) {
          invalidate(
              hard_stale
                  ? "Prolonged IMU data loss"
                  : "Persistent IMU instability "
                    "or low confidence");
        }

        supervision_latched_ = true;
        trusted_stable_ = false;
        uncertainty_active_ = false;
      }

    } else {
      if (uncertainty_active_) {
       RCLCPP_DEBUG(
    get_logger(),
    "Platform supervision recovered within grace interval");
      }

      uncertainty_active_ = false;
      supervision_latched_ = false;
    }

    // Debounced task-state compatibility flags.
    // During UNCERTAIN these remain true, but the
    // PlatformSupervision state is not STABLE.
    state_.supervision_valid = trusted_stable_;
    state_.imu_stable = trusted_stable_;

    std::ostringstream status;

    status << std::fixed << std::setprecision(3)
           << "confidence=" << m.stable_confidence
           << " coverage_s=" << m.confidence_coverage_s
           << " raw_valid=" << m.valid
           << " raw_stable=" << m.stable
           << " uni_valid=" << m.unilidar.valid
           << " uni_stable=" << m.unilidar.stable
           << " mpu_valid=" << m.mpu6050.valid
           << " mpu_stable=" << m.mpu6050.stable;

    if (hard_motion) {
      status << " HARD_MOTION";
    } else if (uncertainty_active_) {
      status << " UNCERTAIN";
    } else if (trusted_stable_) {
      status << " STABLE";
    } else {
      status << " NOT_AUTHORIZED";
    }

    if (!state_.workpiece_pose_valid &&
        !invalidation_reason_.empty()) {
      status << "; pose invalidated: "
             << invalidation_reason_;
    }

    state_.status_message = status.str();
    state_.header.stamp = now();

    pub_->publish(state_);

    Supervision diagnostic;
    diagnostic.header = state_.header;

    diagnostic.unilidar = toRosMetrics(
        m.unilidar,
        diagnostic.header,
        "unilidar",
        m.stable_confidence,
        m.confidence_coverage_s);

    diagnostic.mpu6050 = toRosMetrics(
        m.mpu6050,
        diagnostic.header,
        "mpu6050",
        m.stable_confidence,
        m.confidence_coverage_s);

    diagnostic.combined_confidence =
        m.stable_confidence;

    diagnostic.combined_coverage_s =
        m.confidence_coverage_s;

    diagnostic.both_channels_valid = m.valid;
    diagnostic.both_channels_stable = m.stable;

    diagnostic.supervision_valid =
        state_.supervision_valid;

    diagnostic.imu_stable =
        state_.imu_stable;

    diagnostic.hard_motion_detected =
        hard_motion;

    if (hard_motion || supervision_latched_) {
      diagnostic.state =
          Supervision::UNTRUSTED;
    } else if (uncertainty_active_) {
      diagnostic.state =
          Supervision::UNCERTAIN;
    } else if (trusted_stable_) {
      diagnostic.state =
          Supervision::STABLE;
    } else {
      diagnostic.state =
          Supervision::UNKNOWN;
    }

    diagnostic.reason =
        state_.status_message;

    supervision_pub_->publish(diagnostic);
  }

  hb_robot_skills::SceneRevisionTracker
      scene_revisions_;

  State state_;

  rclcpp::CallbackGroup::SharedPtr
      imu_callbacks_;

  std::unique_ptr<
      hb_perception::ImuStabilityMonitor> imu_;

  rclcpp::Publisher<State>::SharedPtr pub_;

  rclcpp::Publisher<Supervision>::SharedPtr
      supervision_pub_;

  rclcpp::Subscription<
      hb_robot_interfaces::msg::WorkpieceReference>::
      SharedPtr ref_;

  rclcpp::Subscription<
      hb_robot_interfaces::msg::TorchStatus>::
      SharedPtr torch_;

  rclcpp::Subscription<
      moveit_msgs::msg::PlanningScene>::
      SharedPtr scene_;

  rclcpp::Subscription<std_msgs::msg::Bool>::
      SharedPtr fault_, invalidation_;

  rclcpp::Subscription<std_msgs::msg::UInt64>::
      SharedPtr execution_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string invalidation_reason_;

  int64_t last_publication_ = 0;
  int64_t last_untrusted_ = now().nanoseconds();

  geometry_msgs::msg::Pose measured_pose_;

  double acquire_confidence_ = 0.90;
  double release_confidence_ = 0.65;
  double grace_s_ = 0.80;

  double hard_peak_gyro_ = 0.20;
  double hard_peak_accel_ = 0.80;

  bool trusted_stable_ = false;
  bool uncertainty_active_ = false;
  bool supervision_latched_ = false;

  Clock::time_point uncertainty_since_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<TaskStateServer>();

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 2);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
