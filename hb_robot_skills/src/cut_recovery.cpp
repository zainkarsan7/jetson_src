
#include <hb_robot_skills/cut_profile_server.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace hb_robot_skills {
CutProfileServer::~CutProfileServer() {
  if(worker_.joinable()) worker_.join();
  if(escape_worker_.joinable()) escape_worker_.join();
  if(recovery_worker_.joinable()) recovery_worker_.join();
}



CutProfileServer::TaskStateFreshness
CutProfileServer::taskStateFreshness() const
{
    TaskStateFreshness result;

    std::lock_guard<std::mutex> lock(task_mutex_);

    result.state = task_state_;

    result.received =
        task_received_ !=
        std::chrono::steady_clock::time_point{};

    if (!result.received)
        return result;

    result.age_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            task_received_).count();

    result.fresh = result.age_ms < task_state_timeout_ms_;

    return result;
}

bool CutProfileServer::readFreshTaskState(TaskState& snapshot) const {
  std::lock_guard<std::mutex> lock(task_mutex_);
  snapshot = task_state_;
  return task_received_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - task_received_ <
             std::chrono::milliseconds(task_state_timeout_ms_);
}

bool CutProfileServer::taskPermitsExecution(TaskState* snapshot) const {
  TaskState state;
  const bool fresh = readFreshTaskState(state);
  if(snapshot) *snapshot = state;
  return fresh && hb_robot::recovery::geometryPermitted(state);
}

void CutProfileServer::executeStored(
    const std::string& id,
    const std::shared_ptr<GoalHandleCutProfile>& h) {
  using Clock = std::chrono::steady_clock;
  const auto goal = h->get_goal();
  const auto entered = Clock::now();
  RCLCPP_INFO(get_logger(),
      "[CUT PREFLIGHT] enter plan=%s cut=%s execute=%d skip=%d process=%d approval=%d",
      id.c_str(), goal->cut_id.c_str(), int(goal->execute), int(skip_motion_),
      int(process_enabled_), int(require_plan_approval_));

  auto result = std::make_shared<CutProfile::Result>();
  result->plan_id = id;
  result->cut_id = goal->cut_id;
  std::string stage = "admission";
  auto finish = [&](const std::string& reason, bool success = false) {
    result->success = success;
    result->message = "[" + stage + "] " + reason;
    result->completed_segments = completed_segments_;
    result->active_segment = active_segment_;
    result->execution_version = execution_version_;
    motion_active_ = false;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - entered).count();
    if(success) {
      RCLCPP_INFO(get_logger(),
          "[CUT TERMINAL] plan=%s stage=%s elapsed_ms=%.1f executed=%d "
          "completed=%u active=%s reason=%s",
          id.c_str(), stage.c_str(), elapsed_ms, int(result->executed),
          static_cast<unsigned>(completed_segments_), active_segment_.c_str(),
          reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(),
          "[CUT TERMINAL] plan=%s stage=%s elapsed_ms=%.1f executed=%d "
          "interrupted=%d completed=%u active=%s reason=%s",
          id.c_str(), stage.c_str(), elapsed_ms, int(result->executed),
          int(result->interrupted), static_cast<unsigned>(completed_segments_),
          active_segment_.c_str(), reason.c_str());
    }
    releaseGoalAdmission();
    if(h->is_canceling()) h->canceled(result);
    else if(success) h->succeed(result);
    else h->abort(result);
  };

  // Snapshot the TaskState and its reception timestamp under one mutex.
  auto stateDetails = [&](const TaskState& s, const TaskState& expected) {
    double age_ms = -1.0;
    {
      std::lock_guard<std::mutex> lock(task_mutex_);
      if(task_received_ != Clock::time_point{})
        age_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - task_received_).count();
    }
    std::ostringstream os;
    os << "task_age_ms=" << age_ms
       << " supervision=" << s.supervision_valid
       << " imu=" << s.imu_stable
       << " scene_valid=" << s.scene_valid
       << " pose_valid=" << s.workpiece_pose_valid
       << " motion_fault=" << s.motion_fault_active
       << " process_fault=" << s.process_fault_active
       << " geometry=" << s.geometry_version << "/" << expected.geometry_version
       << " pose=" << s.pose_version << "/" << expected.pose_version
       << " scene=" << s.scene_version << "/" << expected.scene_version
       << " reference='" << s.scene_id << "'/'" << expected.scene_id << "'"
       << " environment='" << s.cut_environment_id << "'/'"
       << expected.cut_environment_id << "'";
    return os.str();
  };

  completed_segments_ = 0;
  active_segment_.clear();
  {
    std::lock_guard<std::mutex> lock(approval_mutex_);
    motion_approved_ = false;
    motion_rejected_ = false;
  }
  auto it = stored_plans_.find(id);
  if(!goal->execute) { finish("execute flag false"); return; }
  if(skip_motion_) { finish("skip_cutting_motion is enabled"); return; }
  if(it == stored_plans_.end()) { finish("plan ID not found in stored_plans_"); return; }
  auto& stored = it->second;
  result->parent_plan_id = stored.parent_id;
  if(stored.cut_id != goal->cut_id) {
    finish("cut_id mismatch: stored='" + stored.cut_id + "' requested='" + goal->cut_id + "'");
    return;
  }
  if(stored.consumed) { finish("stored plan was already consumed"); return; }

  stage = "initial_task_state";
  TaskState current;
  const bool permitted = taskPermitsExecution(&current);
  const bool versions_match = hb_robot::recovery::sameVersions(current, stored.versions);
  if(!permitted || !versions_match) {
    finish("task permission=" + std::to_string(permitted) +
           " versions_match=" + std::to_string(versions_match) + " " +
           stateDetails(current, stored.versions));
    return;
  }
  if(escape_busy_) { finish("escape_busy=1"); return; }
  cut_stop_requested_ = false;

  stage = "approval";
  if(require_plan_approval_) {
    RCLCPP_INFO(get_logger(), "[CUT PREFLIGHT] waiting for approval plan=%s", id.c_str());
    const auto approved = waitForPlanApproval(h);
      if (approved != MotionOutcome::SUCCESS) {
          result->approval_rejected =
              approved == MotionOutcome::REJECTED;

        finish(
              result->approval_rejected
                  ? "Operator rejected cut feature"
                  : "Execution approval not granted");
          return;
      }
    
    //   bool accepted = false, rejected = false;
    //   {
    //     std::lock_guard<std::mutex> lock(approval_mutex_);
    //     accepted = motion_approved_;
    //     rejected = motion_rejected_;
    //   }
    //   TaskState s;
    //   const bool ok = taskPermitsExecution(&s);
    //   finish("approval outcome=" + std::to_string(static_cast<int>(approved)) +
    //          " approved=" + std::to_string(accepted) +
    //          " rejected=" + std::to_string(rejected) +
    //          " canceled=" + std::to_string(h->is_canceling()) +
    //          " cut_stop=" + std::to_string(cut_stop_requested_.load()) +
    //          " escape=" + std::to_string(escape_busy_.load()) +
    //          " task_permitted=" + std::to_string(ok) + " " +
    //          stateDetails(s, stored.versions));
    //   return;
    // }
  }

  stage = "post_approval_task_state";
  const bool post_permitted = taskPermitsExecution(&current);
  const bool post_versions = hb_robot::recovery::sameVersions(current, stored.versions);
  if(h->is_canceling() || !post_permitted || !post_versions) {
    finish("canceled=" + std::to_string(h->is_canceling()) +
           " permitted=" + std::to_string(post_permitted) +
           " versions_match=" + std::to_string(post_versions) + " " +
           stateDetails(current, stored.versions));
    return;
  }

  stage = "robot_start";
  auto actual = move_group_->getCurrentState(2.0);
  if(!actual) { finish("getCurrentState(2.0) returned null"); return; }
  if(stored.executable.cuts.empty()) {
     finish("stored executable has zero cuts"); 
     return; 
    }
    


  const auto& start = stored.executable.cuts.front().transit_in.trajectory_.joint_trajectory;
  
  if (cut_planning_mode_ == CutPlanningMode::Independent) {
    if (stored.executable.cuts.size() != 1 ||
        !stored.executable.cuts.front().transit_out) {
        finish(
            "Independent stored plan must contain exactly "
            "one complete home-to-home cut");
        return;
    }
}
  
  
  
  
  
  if(start.joint_names.empty()) { finish("first transit has no joint names"); return; }
  if(start.points.empty()) { finish("first transit has no waypoints"); return; }
  if(start.points.front().positions.size() != start.joint_names.size()) {
    finish("first transit positions=" + std::to_string(start.points.front().positions.size()) +
           " joint_names=" + std::to_string(start.joint_names.size()));
    return;
  }
  for(size_t j = 0; j < start.joint_names.size(); ++j) {
    const auto& joint = start.joint_names[j];
    const double actual_q = actual->getVariablePosition(joint);
    const double planned_q = start.points.front().positions[j];
    const double error = std::abs(actual_q - planned_q);
    if(!(error <= 0.02)) {
      std::ostringstream os;
      os << "joint='" << joint << "' actual=" << actual_q
         << " planned=" << planned_q << " abs_error=" << error
         << " tolerance=0.02";
      finish(os.str());
      return;
    }
  }

  stage = "trajectory_validation";
{
    planning_scene::PlanningScenePtr validation_scene;

    {
        planning_scene_monitor::LockedPlanningSceneRO locked(
            planning_scene_monitor_);

        if (!locked) {
            finish("planning_scene_monitor lock returned null");
            return;
        }

        validation_scene = locked->diff();
        validation_scene->decoupleParent();
    }

    // Apply exactly the same normalization as the planner.
    if (!prepareCuttingScene(validation_scene)) {
        finish(
            "Execution cutting scene normalization failed");
        return;
    }

    const auto ids =
        validation_scene->getWorld()->getObjectIds();

    {
        std::ostringstream os;

        for (const auto& object_id : ids)
            os << " '" << object_id << "'";

        RCLCPP_INFO(
            get_logger(),
            "[CUT SCENE] plan=%s world_objects=%zu ids=%s",
            id.c_str(),
            ids.size(),
            os.str().c_str());
    }

    // Keep your existing validate lambda and loop below.
    auto validate = [&](const auto& plan, const std::string& segment,
                        const char* phase) -> bool {
      robot_trajectory::RobotTrajectory trajectory(
          move_group_->getRobotModel(), planning_group_);
      trajectory.setRobotTrajectoryMsg(*actual, plan.trajectory_);
      const auto count = trajectory.getWayPointCount();
      if(trajectory.empty()) {
        finish("segment=" + segment + " phase=" + phase + " empty trajectory");
        return false;
      }
      RCLCPP_INFO(get_logger(),
          "[CUT VALIDATE] segment=%s phase=%s waypoints=%zu",
          segment.c_str(), phase, count);
      for(size_t i = 0; i < count; ++i) {
        const auto& waypoint = trajectory.getWayPoint(i);
        if(!waypoint.satisfiesBounds()) {
          std::ostringstream os;
          os << "segment=" << segment << " phase=" << phase
             << " waypoint=" << i << "/" << count << " predicate=joint_bounds";
          finish(os.str());
          return false;
        }
        if(validation_scene->isStateColliding(waypoint, planning_group_)) {
          collision_detection::CollisionRequest request;
          collision_detection::CollisionResult collision;
          request.group_name = planning_group_;
          request.contacts = true;
          request.max_contacts = 32;
          request.max_contacts_per_pair = 4;
          validation_scene->checkCollision(request, collision, waypoint);
          std::ostringstream os;
          os << "segment=" << segment << " phase=" << phase
             << " waypoint=" << i << "/" << count
             << " predicate=collision contact_count=" << collision.contact_count;
          for(const auto& entry : collision.contacts) {
            os << " pair=('" << entry.first.first << "','" << entry.first.second << "')";
            if(!entry.second.empty())
              os << " depth_m=" << entry.second.front().depth;
          }
          if(collision.contacts.empty())
            os << " [contact details unavailable from diagnostic query]";
          finish(os.str());
          return false;
        }
      }
      return true;
    };
    for(const auto& cut : stored.executable.cuts) {
      if(!validate(cut.transit_in, cut.name, "transit_in")) return;
      if(!validate(cut.process.approach, cut.name, "approach")) return;
      if(!validate(cut.process.cut, cut.name, "cut")) return;
      if(!validate(cut.process.retract, cut.name, "retract")) return;
      if(cut.transit_out && !validate(*cut.transit_out, cut.name, "return_home")) return;
    }
  }

  stage = "torch_preflight";
  if(process_enabled_) {
    const bool io_fresh = torch_controller_->hasFreshIO();
    const bool fault = torch_controller_->hasFault();
    if(!io_fresh || fault) {
      finish("process_enabled=1 torch_io_fresh=" + std::to_string(io_fresh) +
             " torch_fault=" + std::to_string(fault));
      return;
    }
  }

  stage = "recut_overlap";
  for(const auto& interval : stored.intervals)
    for(const auto& attempted : attempted_[stored.cut_id])
      if(hb_robot::recovery::overlaps(interval, attempted)) {
        std::ostringstream os;
        os << "segment=" << interval.segment_id
           << " requested=[" << interval.s0 << "," << interval.s1 << "]"
           << " previous=[" << attempted.s0 << "," << attempted.s1 << "]";
        finish(os.str());
        return;
      }

  stage = "final_task_state";
  TaskState immediately_before;
  const bool final_permitted = taskPermitsExecution(&immediately_before);
  const bool final_versions =
      hb_robot::recovery::sameVersions(current, immediately_before);
  if(h->is_canceling() || escape_busy_ || !final_permitted || !final_versions) {
    finish("canceled=" + std::to_string(h->is_canceling()) +
           " escape=" + std::to_string(escape_busy_.load()) +
           " permitted=" + std::to_string(final_permitted) +
           " versions_match=" + std::to_string(final_versions) + " " +
           stateDetails(immediately_before, current));
    return;
  }

  // Preserve consume-before-motion and all existing supervision/interlocks.
  stage = "motion_execution";
  stored.consumed = true;
  for(const auto& interval : stored.intervals)
    attempted_[stored.cut_id].push_back(interval);
  execution_state_ = current;
  cut_stop_requested_ = false;
  motion_active_ = true;
  std_msgs::msg::UInt64 event;
  event.data = ++execution_version_;
  execution_pub_->publish(event);
  result->executed = true;
  const auto outcome = executeCuttingPlan(stored.executable, h);
  if(outcome != MotionOutcome::SUCCESS) {
    result->interrupted = true;
    requestCutStop();
    TaskState latest;
    const bool still_permitted = taskPermitsExecution(&latest);
    finish("executeCuttingPlan outcome=" +
           std::to_string(static_cast<int>(outcome)) +
           " active_segment='" + active_segment_ + "'" +
           " stop_requested=" + std::to_string(cut_stop_requested_.load()) +
           " task_permitted=" + std::to_string(still_permitted) + " " +
           stateDetails(latest, execution_state_) +
           " (check preceding Motion execution failed / MoveIt controller logs)");
    return;
  }
  stage = "complete";
  finish(process_enabled_ ?
      "cut motion completed; separation requires verification" :
      "motion dry run completed; torch disabled; separation not verified", true);
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
