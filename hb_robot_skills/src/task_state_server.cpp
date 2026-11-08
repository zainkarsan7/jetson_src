#include "hb_robot_skills/scene_revision_tracker.hpp"
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_perception/imu_stability_monitor.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

using State=hb_robot_interfaces::msg::TaskState;
// All state writers stay in the default mutually exclusive callback group.
// Only IMU reception runs concurrently; the monitor protects its own sample buffer.
class TaskStateServer : public rclcpp::Node {
 public:
  TaskStateServer():Node("task_state_server") {
    state_.geometry_version=static_cast<uint64_t>(now().nanoseconds());
    state_.pose_version=state_.geometry_version;
    // Platform noise limits are distinct from camera-settling and telemetry
    // settings. Keep existing defaults; calibration must be an explicit choice.
    // Read-only parameters avoid accepting ros2 param set without updating the
    // already constructed monitor. Set overrides at node startup and restart.
    hb_perception::ImuStabilityConfig config;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    auto limit=[&](const std::string& name,double fallback) {
      const double value=declare_parameter<double>(name,fallback,descriptor);
      if(!std::isfinite(value) || value<=0)
        throw std::invalid_argument(name+" must be finite and positive");
      return value;
    };
    config.max_ang_vel_rms=limit("platform_imu.max_gyro_rms",config.max_ang_vel_rms);
    config.max_lin_acc_rms=limit("platform_imu.max_accel_rms",config.max_lin_acc_rms);
    config.max_mean_ang_vel=limit("platform_imu.max_mean_gyro",config.max_mean_ang_vel);
    const auto topic=declare_parameter<std::string>("platform_imu_topic","/hb_robot_lidar/imu");
    config.auxiliary_accel_topic=declare_parameter<std::string>("platform_accel_topic","/mpu6050/imu",descriptor);
    imu_callbacks_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_=std::make_unique<hb_perception::ImuStabilityMonitor>(*this,topic,config,imu_callbacks_);
    RCLCPP_INFO(get_logger(),
      "Platform supervision: %s; gyro RMS < %.6f rad/s, accel RMS < %.6f m/s^2, "
      "mean gyro < %.6f rad/s; independent of platform_telemetry settings",
      topic.c_str(),config.max_ang_vel_rms,config.max_lin_acc_rms,config.max_mean_ang_vel);
    pub_=create_publisher<State>("/task_state",rclcpp::QoS(1).transient_local());
    ref_=create_subscription<hb_robot_interfaces::msg::WorkpieceReference>(
      "/workpiece_reference",rclcpp::QoS(1).transient_local(),[this](hb_robot_interfaces::msg::WorkpieceReference::ConstSharedPtr m){
        const auto metrics=imu_->metrics();
        const double age=(now()-rclcpp::Time(m->estimate.header.stamp)).seconds();
        const auto published=rclcpp::Time(m->estimate.header.stamp).nanoseconds();
        const auto observed=rclcpp::Time(m->cloud.header.stamp).nanoseconds();
        if(age<0 || age>5 || published<=last_publication_ || observed<=0 ||
           observed>published || m->cloud.data.empty() || m->estimate.profile_name.empty() ||
           m->estimate.header.frame_id.empty() || m->estimate.header.frame_id!=m->cloud.header.frame_id) {
          RCLCPP_WARN(get_logger(),"Rejected malformed, expired or superseded workpiece reference");
          return;
        }
        if(m->relocalized && (!state_.scene_valid || m->expected_pose_version!=state_.pose_version)) return;
        // A measured fit can support a candidate even when its observation interval
        // cannot support execution. Never promote it just because the IMU recovers.
        const bool confirmed=metrics.valid && metrics.stable && observed>last_untrusted_ &&
            metrics.reception_gaps==last_seen_gaps_ && !state_.motion_fault_active;
        if(!m->relocalized) ++state_.geometry_version;
        if(!m->relocalized || m->estimate.pose!=measured_pose_) ++state_.pose_version;
        measured_pose_=m->estimate.pose;
        last_publication_=published;
        state_.scene_valid=true;
        state_.scene_id=std::to_string(published);
        state_.workpiece_pose_valid=confirmed;
        invalidation_reason_=confirmed ? "" : "reference requires a fresh supervised observation";
        RCLCPP_INFO(get_logger(),"Reference %s accepted for planning; execution pose confirmed=%d",
                    state_.scene_id.c_str(),confirmed);
        state_.path_valid=false;
      });
    scene_=create_subscription<moveit_msgs::msg::PlanningScene>("/monitored_planning_scene",10,[this](moveit_msgs::msg::PlanningScene::ConstSharedPtr m){
      if(scene_revisions_.changed(*m)) {
        ++state_.scene_version;
        state_.path_valid=false;
        // Each revision can retire a plan. Keep both messages if an edit and a
        // snapshot arrive back-to-back; throttling hides that sequence.
        std::ostringstream objects;
        for(const auto& object:m->world.collision_objects)
          objects << " " << object.id << "(op=" << static_cast<int>(object.operation) << ")";
        RCLCPP_INFO(get_logger(),
          "Scene revision=%lu is_diff=%d objects=[%s] attached=%zu octomap_bytes=%zu "
          "fixed_tf=%zu acm_entries=%zu",
          static_cast<unsigned long>(state_.scene_version),m->is_diff,objects.str().c_str(),
          m->robot_state.attached_collision_objects.size(),m->world.octomap.octomap.data.size(),
          m->fixed_frame_transforms.size(),m->allowed_collision_matrix.entry_names.size());
      }
    });
    torch_=create_subscription<hb_robot_interfaces::msg::TorchStatus>("/cut_profile/torch_status",10,[this](hb_robot_interfaces::msg::TorchStatus::ConstSharedPtr m){
      state_.process_fault_active=m->fault;
    });
    fault_=create_subscription<std_msgs::msg::Bool>("/motion_fault",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      state_.motion_fault_active=m->data;
      if(m->data) invalidate("/motion_fault asserted");
    });
    invalidation_=create_subscription<std_msgs::msg::Bool>("/workpiece_pose_invalidated",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data) invalidate("/workpiece_pose_invalidated asserted");});
    execution_=create_subscription<std_msgs::msg::UInt64>("/cut_profile/execution_version",
      rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::UInt64::ConstSharedPtr m){
        state_.execution_version=m->data; state_.kerf_status=State::INVALID_OBSERVATION;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const auto m=imu_->metrics();
      state_.supervision_valid=m.valid; state_.imu_stable=m.valid && m.stable;

      if(!m.valid)
        state_.status_message="Platform IMU invalid: samples=" + std::to_string(m.samples_count) +
          ", age_s=" + std::to_string(m.sample_age_s) +
          "; requires fresh samples spanning the configured window";
      else if(!m.stable) state_.status_message=stabilitySummary(m);
      else if(!state_.workpiece_pose_valid)
        state_.status_message="Platform stable; waiting for a new accepted reference or revalidation";
      else if(state_.process_fault_active || state_.motion_fault_active)
        state_.status_message="Geometry valid; execution blocked by active fault";
      else state_.status_message="Geometry valid; platform supervision stable";
      // Loss of observation latches execution permission off, even when packets
      // are fresh but the sample window is incomplete. The planning reference
      // and its measured pose version survive. Only a supervised observation
      // acquired after the last untrusted interval can clear this latch.
      if(m.reception_gaps != last_seen_gaps_)
        invalidate("platform IMU reception gap exceeded max_sample_age");
      else if(!m.valid || !m.stable)
        invalidate(state_.status_message);
      last_seen_gaps_=m.reception_gaps;
      if(state_.imu_stable && !state_.workpiece_pose_valid && !invalidation_reason_.empty())
        state_.status_message += ". Pose invalidated by " + invalidation_reason_;
      state_.header.stamp=now(); pub_->publish(state_);
    });
  }
 private:
  std::string stabilitySummary(const hb_perception::ImuStabilityMetrics& m) {
    if(!m.valid) return "Platform IMU window invalid; stability statistics are not actionable";
    const auto& config=imu_->config();
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "Platform stability: ";
    auto check=[&](const char* name,double value,double limit,const char* unit) {
      out << name << "=" << value << " < " << limit << " " << unit
          << (value<limit ? " [PASS]; " : " [FAIL]; ");
    };
    // Both RMS values are vector RMS about the window mean, not absolute
    // acceleration and not the latest sample. A constant sensor bias cancels.
    check("gyro_rms",m.ang_vel_rms_rad_s,config.max_ang_vel_rms,"rad/s");
    check("accel_rms",m.lin_acc_rms_m_s2,config.max_lin_acc_rms,"m/s^2");
    check("mean_gyro",m.mean_ang_vel_rad_s,config.max_mean_ang_vel,"rad/s");
    return out.str();
  }
  // Preserve the first cause until a fresh reference is accepted. The platform
  // can settle again before PlanCut checks the state at the end of planning.
  std::string invalidation_reason_;
  std::uint64_t last_seen_gaps_=0;
  int64_t last_publication_=0, last_untrusted_=now().nanoseconds();
  geometry_msgs::msg::Pose measured_pose_;
  void invalidate(const std::string& reason) {
    last_untrusted_=now().nanoseconds();
    if(state_.workpiece_pose_valid) {
      invalidation_reason_=reason;
      RCLCPP_WARN(get_logger(),"Workpiece pose invalidated (pose_version=%llu): %s",
        static_cast<unsigned long long>(state_.pose_version),reason.c_str());
    }
    state_.workpiece_pose_valid=false; state_.path_valid=false;
    state_.kerf_status=State::INVALID_OBSERVATION;
  }
  hb_robot_skills::SceneRevisionTracker scene_revisions_;
  State state_;
  rclcpp::CallbackGroup::SharedPtr imu_callbacks_;
  std::unique_ptr<hb_perception::ImuStabilityMonitor> imu_;
  rclcpp::Publisher<State>::SharedPtr pub_;
  rclcpp::Subscription<hb_robot_interfaces::msg::WorkpieceReference>::SharedPtr ref_;
  rclcpp::Subscription<hb_robot_interfaces::msg::TorchStatus>::SharedPtr torch_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_,invalidation_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr execution_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<TaskStateServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
