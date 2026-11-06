#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/msg/torch_status.hpp>
#include <hb_robot_perception/imu_stability_monitor.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <rclcpp/rclcpp.hpp>

using State=hb_robot_interfaces::msg::TaskState;
// Single writer for versions. The executor is single threaded, so reference commits,
// disturbance invalidation and publication cannot interleave half an update.
class TaskStateServer : public rclcpp::Node {
 public:
  TaskStateServer():Node("task_state_server") {
    state_.geometry_version=static_cast<uint64_t>(now().nanoseconds());
    state_.pose_version=state_.geometry_version;
    imu_=std::make_unique<hb_perception::ImuStabilityMonitor>(*this,
      declare_parameter<std::string>("platform_imu_topic","/hb_robot_lidar/imu"));
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
      else if(!m.stable) state_.status_message="Platform IMU valid but motion exceeds stability thresholds";
      else if(!state_.workpiece_pose_valid)
        state_.status_message="Platform stable; waiting for a new accepted reference or revalidation";
      else if(state_.process_fault_active || state_.motion_fault_active)
        state_.status_message="Geometry valid; execution blocked by active fault";
      else state_.status_message="Geometry valid; platform supervision stable";
      state_.header.stamp=now(); pub_->publish(state_);
    });
  }
 private:
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
