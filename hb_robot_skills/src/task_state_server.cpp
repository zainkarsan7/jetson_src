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
// Single writer for versions. The executor is single threaded, so reference commits,
// disturbance invalidation and publication cannot interleave half an update.
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
    imu_=std::make_unique<hb_perception::ImuStabilityMonitor>(*this,topic,config);
    RCLCPP_INFO(get_logger(),
      "Platform supervision: %s; gyro RMS < %.6f rad/s, accel RMS < %.6f m/s^2, "
      "mean gyro < %.6f rad/s; independent of platform_telemetry settings",
      topic.c_str(),config.max_ang_vel_rms,config.max_lin_acc_rms,config.max_mean_ang_vel);
    pub_=create_publisher<State>("/task_state",rclcpp::QoS(1).transient_local());
    ref_=create_subscription<hb_robot_interfaces::msg::WorkpieceReference>(
      "/workpiece_reference",rclcpp::QoS(1).transient_local(),[this](hb_robot_interfaces::msg::WorkpieceReference::ConstSharedPtr m){
        const auto metrics=imu_->metrics();
        const double age=(now()-rclcpp::Time(m->estimate.header.stamp)).seconds();
        if(age<0 || age>5 || !metrics.valid || !metrics.stable || m->cloud.data.empty() ||
           m->estimate.header.frame_id.empty()) {
          RCLCPP_WARN(get_logger(),
            "Reference rejected: age=%.3fs (allowed 0..5), imu_valid=%d imu_stable=%d "
            "samples=%zu sample_age=%.3fs cloud_empty=%d frame_empty=%d. "
            "Restore supervision and acquire a fresh reference; rejected references are not queued.",
            age, metrics.valid, metrics.stable, metrics.samples_count, metrics.sample_age_s,
            m->cloud.data.empty(), m->estimate.header.frame_id.empty());
          RCLCPP_WARN(get_logger(),"Reference stability diagnostics: %s",
            stabilitySummary(metrics).c_str());
          return;
        }
        if(m->relocalized && m->expected_pose_version!=state_.pose_version) return;
        if(!m->relocalized) ++state_.geometry_version;
        ++state_.pose_version;
        state_.workpiece_pose_valid=true;
        state_.path_valid=false;
      });
    scene_=create_subscription<moveit_msgs::msg::PlanningScene>("/monitored_planning_scene",10,[this](moveit_msgs::msg::PlanningScene::ConstSharedPtr m){
      // Robot state updates are not environment changes. Collision and ACM changes are.
      if(!m->is_diff || !m->world.collision_objects.empty() || !m->world.octomap.octomap.data.empty() ||
         !m->robot_state.attached_collision_objects.empty() ||
         !m->allowed_collision_matrix.entry_names.empty()) {
        ++state_.scene_version; state_.path_valid=false;
      }
    });
    torch_=create_subscription<hb_robot_interfaces::msg::TorchStatus>("/cut_profile/torch_status",10,[this](hb_robot_interfaces::msg::TorchStatus::ConstSharedPtr m){
      state_.process_fault_active=m->fault;
    });
    fault_=create_subscription<std_msgs::msg::Bool>("/motion_fault",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      state_.motion_fault_active=m->data;
      if(m->data) invalidate();
    });
    invalidation_=create_subscription<std_msgs::msg::Bool>("/workpiece_pose_invalidated",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data) invalidate();});
    execution_=create_subscription<std_msgs::msg::UInt64>("/cut_profile/execution_version",
      rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::UInt64::ConstSharedPtr m){
        state_.execution_version=m->data; state_.kerf_status=State::INVALID_OBSERVATION;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const auto m=imu_->metrics();
      state_.supervision_valid=m.valid; state_.imu_stable=m.valid && m.stable;
      if(!state_.imu_stable) invalidate();
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
  void invalidate() {
    if(state_.workpiece_pose_valid) ++state_.pose_version;
    state_.workpiece_pose_valid=false; state_.path_valid=false;
    state_.kerf_status=State::INVALID_OBSERVATION;
  }
  State state_;
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
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<TaskStateServer>());rclcpp::shutdown();
}
