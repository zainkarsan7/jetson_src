#include "cut_profile_server.hpp"
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <limits>

namespace hb_robot_skills {
CutProfileServer::~CutProfileServer() {
  if(worker_.joinable()) worker_.join();
  if(escape_worker_.joinable()) escape_worker_.join();
}
bool CutProfileServer::readFreshTaskState(TaskState& snapshot) const {
  std::lock_guard<std::mutex> lock(task_mutex_);
  snapshot=task_state_;
  return std::chrono::steady_clock::now()-task_received_<std::chrono::milliseconds(500);
}
bool CutProfileServer::taskPermitsExecution(TaskState* snapshot) const {
  TaskState state;
  const bool fresh=readFreshTaskState(state);
  if(snapshot) *snapshot=state;
  return fresh && hb_robot::recovery::geometryPermitted(state);
}
void CutProfileServer::executeStored(const std::string& id,
    const std::shared_ptr<GoalHandleCutProfile>& h) {
  auto result=std::make_shared<CutProfile::Result>();
  result->plan_id=id;result->cut_id=h->get_goal()->cut_id;
  auto finish=[&](const std::string& message,bool success=false) {
    result->success=success;result->message=message;
    result->completed_segments=completed_segments_;result->active_segment=active_segment_;
    result->execution_version=execution_version_;
    motion_active_=false;
    if(h->is_canceling()) h->canceled(result);
    else if(success) h->succeed(result);else h->abort(result);
  };
  completed_segments_=0;active_segment_.clear();
  {std::lock_guard<std::mutex> lock(approval_mutex_);
    motion_approved_=false;motion_rejected_=false;}
  auto it=stored_plans_.find(id);
  if(!h->get_goal()->execute || skip_motion_ || it==stored_plans_.end() ||
     it->second.cut_id!=h->get_goal()->cut_id || it->second.consumed) {
    finish("Stored plan missing, consumed, or execution disabled");return;
  }
  auto& stored=it->second;result->parent_plan_id=stored.parent_id;
  TaskState current;
  if(!taskPermitsExecution(&current) || !hb_robot::recovery::sameVersions(current,stored.versions)) {
    finish("Stored plan is stale");return;
  }
  if(escape_busy_) {finish("Escape has priority over execution");return;}
  cut_stop_requested_=false;
  if(require_plan_approval_) {
    const auto approved=waitForPlanApproval(h);
    if(approved!=MotionOutcome::SUCCESS) {finish("Execution approval not granted");return;}
  }
  // Approval may take minutes; recheck immediately before execution.
  if(h->is_canceling() || !taskPermitsExecution(&current) ||
     !hb_robot::recovery::sameVersions(current,stored.versions)) {finish("State changed awaiting approval");return;}
  auto actual=move_group_->getCurrentState(2.0);
  if(!actual || stored.executable.cuts.empty()) {finish("Missing current robot state");return;}
  const auto& start=stored.executable.cuts.front().transit_in.trajectory_.joint_trajectory;
  if(start.joint_names.empty() || start.points.empty() || start.points.front().positions.size()!=start.joint_names.size()) {
    finish("Invalid trajectory start");return;
  }
  for(size_t j=0;j<start.joint_names.size();++j)
    if(!(std::abs(actual->getVariablePosition(start.joint_names[j])-start.points.front().positions[j])<=0.02)) {
      finish("Robot moved since planning; request a new plan");return;
    }
  // Current scene validation is independent of the optimizer and stored versions.
  {
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
  auto validate=[&](const auto& plan) {
    robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(),planning_group_);
    trajectory.setRobotTrajectoryMsg(*actual,plan.trajectory_);
    if(trajectory.empty()) return false;
    for(size_t i=0;i<trajectory.getWayPointCount();++i)
      if(!trajectory.getWayPoint(i).satisfiesBounds() ||
         scene->isStateColliding(trajectory.getWayPoint(i),planning_group_)) return false;
    return true;
  };
  for(const auto& c:stored.executable.cuts)
    if(!validate(c.transit_in) || !validate(c.process.approach) || !validate(c.process.cut) ||
       !validate(c.process.retract) || (c.transit_out && !validate(*c.transit_out))) {
      finish("Stored trajectory failed current scene validation");return;
    }
  }
  if(process_enabled_ && (!torch_controller_->hasFreshIO() || torch_controller_->hasFault())) {
    finish("Torch feedback unavailable or fault latched");return;
  }
  for(const auto& interval:stored.intervals)
    for(const auto& attempted:attempted_[stored.cut_id])
      if(hb_robot::recovery::overlaps(interval,attempted)) {finish("Recut region already attempted");return;}
  TaskState immediately_before;
  if(h->is_canceling() || escape_busy_ || !taskPermitsExecution(&immediately_before) ||
     !hb_robot::recovery::sameVersions(current,immediately_before)) {
    finish("State changed during final trajectory validation");return;
  }
  // Consume BEFORE movement. Cancellation and faults never make a plan reusable.
  stored.consumed=true;
  for(const auto& interval:stored.intervals) attempted_[stored.cut_id].push_back(interval);
  execution_state_=current;cut_stop_requested_=false;motion_active_=true;
  std_msgs::msg::UInt64 event;event.data=++execution_version_;execution_pub_->publish(event);
  result->executed=true;
  const auto outcome=executeCuttingPlan(stored.executable,h);
  if(outcome!=MotionOutcome::SUCCESS) {
    result->interrupted=true;requestCutStop();
    finish("Execution interrupted; diagnose evidence and request a newly validated plan");return;
  }
  finish(process_enabled_?"Cut executed; separation still requires verification":
    "Motion dry run complete (torch disabled); separation not verified",true);
}

void CutProfileServer::escape(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    hb_robot_interfaces::action::Escape>>& h) {
  using Action=hb_robot_interfaces::action::Escape;
  auto result=std::make_shared<Action::Result>();
  result->status=Action::Result::STOPPED_IN_PLACE;
  result->pose_requires_revalidation=true;
  auto finish=[&](const std::string& message) {
    motion_active_=false;retreat_active_=false;
    // Commit the result's revalidation requirement to shared state too. Publish
    // after the bounded retreat so invalidation cannot authorize further motion.
    std_msgs::msg::Bool invalid;invalid.data=true;invalidate_pose_pub_->publish(invalid);
    const auto telemetry=torch_controller_->snapshot();
    result->torch_confirmed_off=torch_controller_->hasFreshIO() &&
      telemetry.output_on==std::optional<bool>(false) &&
      telemetry.transfer_detected==std::optional<bool>(false);
    const auto robot=move_group_->getCurrentState(1.0);
    result->robot_stationary=robot && robot->hasVelocities();
    if(result->robot_stationary)
      for(const auto& joint:robot->getRobotModel()->getVariableNames())
        if(!std::isfinite(robot->getVariableVelocity(joint)) ||
           std::abs(robot->getVariableVelocity(joint))>0.01) result->robot_stationary=false;
    result->message=message;
    if(h->is_canceling()) h->canceled(result);else h->succeed(result);
  };
  try {
    requestCutStop();
    std::lock_guard<std::mutex> execution_lock(execution_mutex_);
    // The independent callback has already requested OFF and stop. Now feedback
    // may be awaited without delaying the first containment request.
    const auto off=torch_controller_->requestShutdown(std::chrono::milliseconds(1500));
    TaskState state;
    {std::lock_guard<std::mutex> lock(task_mutex_);state=task_state_;
      if(std::chrono::steady_clock::now()-task_received_>std::chrono::milliseconds(500)) state.supervision_valid=false;}
    if(!h->get_goal()->allow_retreat || h->is_canceling()) {finish("Stop requested; no retreat requested");return;}
    if(off!=TorchCommandResult::CONFIRMED || !state.supervision_valid || !state.imu_stable ||
       !state.workpiece_pose_valid || state.motion_fault_active) {
      result->status=Action::Result::MOTION_NOT_PERMITTED;
      finish("Retreat denied: off confirmation, valid geometry and motion permission required");return;
    }
    const auto estimate=latestProfileEstimate();
    auto robot=move_group_->getCurrentState(2.0);
    const auto model=estimate?hb_perception::ProfileLibrary::find(estimate->profile_name):std::nullopt;
    if(!robot || !estimate || !model) {result->status=Action::Result::NO_SAFE_PATH;finish("Missing retreat geometry");return;}
    bool stationary=robot->hasVelocities();
    if(stationary) for(const auto& joint:robot->getRobotModel()->getVariableNames())
      if(!std::isfinite(robot->getVariableVelocity(joint)) ||
         std::abs(robot->getVariableVelocity(joint))>0.01) stationary=false;
    const auto telemetry=torch_controller_->snapshot();
    if(!stationary || telemetry.transfer_detected!=std::optional<bool>(false)) {
      result->status=Action::Result::MOTION_NOT_PERMITTED;
      finish("Stationary robot and extinguished arc must be confirmed before retreat");return;
    }
    {planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      if(scene->isStateColliding(*robot,planning_group_)) {
        result->status=Action::Result::MOTION_NOT_PERMITTED;
        finish("Contact/collision state requires intervention before retreat");return;
      }}
    Eigen::Isometry3d pose;tf2::fromMsg(estimate->pose,pose);
    const auto tcp=robot->getGlobalLinkTransform(plasma_link_);
    Eigen::Vector3d normal=Eigen::Vector3d::Zero();double nearest=std::numeric_limits<double>::infinity();
    for(const auto& f:model->cut_features) {
      const Eigen::Vector3d a=pose*Eigen::Vector3d(f.start.x(),f.start.y(),0);
      const Eigen::Vector3d b=pose*Eigen::Vector3d(f.end.x(),f.end.y(),0);
      const Eigen::Vector3d d=b-a;
      if(d.squaredNorm()<1e-12) continue;
      const double t=std::clamp((tcp.translation()-a).dot(d)/d.squaredNorm(),0.0,1.0);
      const double distance=(tcp.translation()-(a+t*d)).norm();
      if(distance<nearest) {nearest=distance;normal=pose.linear()*Eigen::Vector3d(f.outward_normal.x(),f.outward_normal.y(),0);}
    }
    if(normal.norm()<0.9) {result->status=Action::Result::NO_SAFE_PATH;finish("No outward normal");return;}
    const double distance=h->get_goal()->retreat_distance>0?h->get_goal()->retreat_distance:0.03;
    for(double lateral:{0.0,0.25,-0.25}) {
      Eigen::Isometry3d target=tcp;
      target.translation()+=distance*(normal+lateral*pose.linear().col(2)).normalized();
      move_group_->setStartState(*robot);move_group_->setEndEffectorLink(plasma_link_);
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction=move_group_->computeCartesianPath({tf2::toMsg(target)},0.002,0.1,trajectory,true);
      if(fraction<0.999 || trajectory.joint_trajectory.points.empty()) continue;
      TaskState latest;
      {std::lock_guard<std::mutex> lock(task_mutex_);latest=task_state_;}
      if(h->is_canceling() || !latest.imu_stable || !latest.workpiece_pose_valid ||
         !hb_robot::recovery::sameVersions(state,latest)) break;
      // Process faults remain latched; geometric retreat is permitted only with
      // the output confirmed OFF. No normal is treated as inherently collision-free.
      moveit::planning_interface::MoveGroupInterface::Plan plan;plan.trajectory_=trajectory;
      execution_state_=state;cut_stop_requested_=false;retreat_active_=true;
      const auto outcome=move_group_->execute(plan);
      retreat_active_=false;
      result->status=outcome==moveit::core::MoveItErrorCode::SUCCESS && !cut_stop_requested_?
        Action::Result::RETRACTED:Action::Result::EXECUTION_FAILED;
      finish("Bounded collision-checked retreat finished");return;
    }
    result->status=Action::Result::NO_SAFE_PATH;finish("No validated retreat; holding position");
  } catch(const std::exception& e) {
    requestCutStop();result->status=Action::Result::EXECUTION_FAILED;finish(e.what());
  }
}
} // namespace hb_robot_skills
