#include "hb_robot_skills/motion/cut_inspection.hpp"
#include "hb_robot_skills/kerf_observation.hpp"
#include "hb_robot_skills/kerf_evidence.hpp"
#include "hb_robot_skills/motion/cut_candidate_graph.hpp"
#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/transit_planner.hpp"
#include "hb_robot_skills/motion/planning_budget.hpp"
#include "hb_robot_skills/motion/named_cut_posture.hpp"
#include "hb_robot_skills/motion/inspection_sweep_order.hpp"
#include "hb_robot_skills/motion/cut_inspection_seed_cache.hpp"
#include "hb_robot_interfaces/action/inspect_cut.hpp"
#include "hb_robot_interfaces/msg/kerf_observation.hpp"
#include "hb_robot_interfaces/msg/kerf_ray_sample.hpp"
#include "hb_robot_interfaces/srv/evaluate_kerf.hpp"
#include "hb_robot_interfaces/srv/approve_motion.hpp"
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <condition_variable>
#include <type_traits>
#include "hb_robot_interfaces/srv/get_cut_definition.hpp"
#include "hb_robot_interfaces/srv/replay_kerf.hpp"
#include <iomanip>
#include <sstream>
#include "hb_robot_perception/profile_library.hpp"
#include "hb_robot_perception/cropped_rgbd_acquisition.hpp"
#include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_skills/platform_supervision_client.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <atomic>
#include <thread>
#include <map>
#include <limits>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hb_robot_skills {
class InspectCutServer : public rclcpp::Node {
  using Action = hb_robot_interfaces::action::InspectCut;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using MoveGroup = moveit::planning_interface::MoveGroupInterface;
  using Plan = MoveGroup::Plan;

public:
  InspectCutServer() : Node("inspect_cut_server") {
    group_ = declare_parameter<std::string>("planning_group", "manipulator");
    // IK link in the robot model. The calibrated optical frame comes from RGB-D.
    camera_ = declare_parameter<std::string>("camera_link", "camera_visor");
    frame_ = declare_parameter<std::string>("scene_base_frame", "world");
    infer_evidence_ = declare_parameter<bool>("enable_geometric_kerf_evidence", false);
    ray_config_.allow_sampled_open_verdict =
        declare_parameter<bool>("allow_sampled_open_verdict", false);
    ray_config_.depth_uncertainty = positive("kerf_depth_uncertainty", 0.002);
    ray_config_.half_kerf_width = positive("kerf_half_width", 0.001);
    ray_config_.bin_width = positive("kerf_bin_width", 0.002);
    spacing_ = positive("sample_spacing", 0.025);
    standoff_ = positive("camera_standoff", 0.30);
    band_ = positive("kerf_half_band", 0.015);
    margin_ = positive("profile_margin", 0.02);
    range_ = positive("depth_range", 1.0);
    timeout_ = positive("acquisition_timeout", 5.0);
    stability_timeout_ = positive("stability_timeout", 10.0);
    rgbd_topic_ = declare_parameter<std::string>("perception_rgbd_topic", "/k4a/rgbd");
    view_request_.pos_tol = positive("view_position_tolerance_m", 0.025);
    view_request_.roll_tol = positive("view_roll_tolerance_rad", 1.57);
    view_request_.ik_timeout = positive("view_ik_timeout_s", 0.02);
    view_request_.joint_delta = positive("view_joint_consistency_rad", 6.28);
    view_request_.num_candidates = boundedCount("view_samples", 32, 256);
    max_view_solutions_ = boundedCount("view_solutions", 6, 16);
    seed_trials_ = boundedCount("view_seed_trials", 16, 64);
    local_seed_radius_ = positive("view_local_seed_radius_rad", 0.35);
    local_consistency_ = positive("view_local_ik_consistency_rad", 0.75);
    station_joint_step_ = positive("view_station_joint_step_rad", 0.35);
    direction_seed_names_ = declare_parameter<std::vector<std::string>>(
      "cut_geometry.direction_seed_states", {"cut_forward", "cut_down", "cut_up"});
    planning_timeout_ = positive("inspection_planning_timeout_s", 120.0);
    transit_budget_ = positive("inspection_transit_budget_s", 15.0);    
    local_transit_budget_ = positive("inspection_station_ptp_budget_s", 2.0);
    transit_options_.guided_ptp_time_s = positive("inspection_ptp_call_timeout_s", 2.0);
    transit_options_.guide_link = camera_;
    task_state_timeout_ms_ = positive("task_state_timeout_ms", 500.0);

    planning_time_ = positive("planning_time", 5.0);
    transit_options_.velocity_scaling = positive("transit.velocity_scaling", 0.05);
    transit_options_.acceleration_scaling = positive("transit.acceleration_scaling", 0.01);
    if (transit_options_.velocity_scaling > 1.0 || transit_options_.acceleration_scaling > 1.0)
      throw std::invalid_argument("Transit scaling must be in (0, 1]");
    require_plan_approval_ = declare_parameter<bool>("require_plan_approval", true);
    skip_motion_ = declare_parameter<bool>("skip_inspection_motion", true);
    // Preview is deliberately separate from successful acquisition.
    preview_ = declare_parameter<bool>("preview_only", true);
    declare_parameter<std::string>("perception_rgb_topic", "/k4a/rgb/image_raw");
    declare_parameter<std::string>("perception_depth_topic", "/k4a/depth_to_rgb/image_raw");
    declare_parameter<std::string>("perception_camera_info_topic", "/k4a/depth_to_rgb/camera_info");
  }
  ~InspectCutServer() override {
    shutting_down_.store(true);
    interrupt("server shutdown");
    if (worker_.joinable())
      worker_.join();
    rgbd_acquisition_.reset(); // Join receiver before destroying its TF buffer.
  }
  void initialize() {
    move_ = std::make_unique<MoveGroup>(shared_from_this(), group_);
    move_->setPlanningTime(planning_time_);
    move_->setPoseReferenceFrame(frame_);
    move_->setMaxVelocityScalingFactor(transit_options_.velocity_scaling);
    move_->setMaxAccelerationScalingFactor(transit_options_.acceleration_scaling);
    move_->setGoalPositionTolerance(0.002);
    move_->setGoalOrientationTolerance(0.02);
    if (move_->getPlanningFrame() != frame_ || !move_->getRobotModel()->hasLinkModel(camera_))
      throw std::runtime_error("Planning frame mismatch or camera IK link absent from robot model");
    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), "robot_description");
    if (!scene_monitor_->getPlanningScene())
      throw std::runtime_error("Planning scene unavailable");
    scene_monitor_->startSceneMonitor("/monitored_planning_scene");

    scene_monitor_->startWorldGeometryMonitor();

    scene_monitor_->startStateMonitor();
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    rgbd_acquisition_ = std::make_unique<hb_perception::CroppedRGBDAcquisition>(
        this, tf_.get(), frame_, hb_perception::CroppedRGBDAcquisition::Mode::CaptureWindow,
        rgbd_topic_);
    RCLCPP_INFO(get_logger(),
                "[KERF RGBD] persistent receiver topic=%s; embedded CameraInfo; "
                "roll_reference=inspection_home_optical",
                rgbd_topic_.c_str());
    stability_ = std::make_unique<hb_robot_skills::PlatformSupervisionClient>(*this);

    display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", rclcpp::QoS(1).reliable().transient_local());
    route_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "~/display_planned_path", rclcpp::QoS(1).reliable().transient_local());
    // InspectProfile remains the sole owner of approve_profile_motion.
    // Use the same service type and GUI pattern with an unambiguous endpoint.
    approval_service_ = create_service<hb_robot_interfaces::srv::ApproveMotion>(
        "approve_inspect_cut_motion",
        [this](hb_robot_interfaces::srv::ApproveMotion::Request::SharedPtr request,
               hb_robot_interfaces::srv::ApproveMotion::Response::SharedPtr response) {
          const bool live = !shutting_down_.load() && taskValid();
          std::string message;
          {
            std::lock_guard<std::mutex> lock(approval_mutex_);
            response->accepted = live && approval_pending_ && !approval_decided_;
            if (response->accepted) {
              approval_decided_ = true;
              motion_approved_ = request->approve;
              message = std::string(request->approve ? "Approved: " : "Rejected: ") + approval_description_;
            } else {
              message = "No undecided, live kerf inspection motion awaiting approval";
            }
          }
          setApprovalMessage(*response, response->accepted, message);
          RCLCPP_INFO(get_logger(), "[KERF APPROVAL] %s", message.c_str());
          approval_cv_.notify_all();
        });
    RCLCPP_INFO(get_logger(),
        "[KERF MOTION CONFIG] planner=profile_transit_portfolio velocity=%.3f acceleration=%.3f "
        "approval=%d skip_motion=%d service=%s route_preview=%s",
        transit_options_.velocity_scaling, transit_options_.acceleration_scaling,
        int(require_plan_approval_), int(skip_motion_), approval_service_->get_service_name(),
        route_traj_pub_->get_topic_name());

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/kerf_cloud",
                                                                 rclcpp::QoS(1).transient_local());
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/viewpoints",
                                                                 rclcpp::QoS(1).transient_local());
    link_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "~/camera_link_viewpoints", rclcpp::QoS(1).transient_local());
    observation_pub_ =
        create_publisher<hb_robot_interfaces::msg::KerfObservation>("~/observations", 10);
    rays_pub_ = create_publisher<hb_robot_interfaces::msg::KerfRaySample>(
        "~/ray_samples", rclcpp::QoS(100).best_effort());
      cut_seeds_sub_ = create_subscription<hb_robot_interfaces::msg::CutInspectionSeeds>(
      "/cut_profile/inspection_seeds", rclcpp::QoS(1).reliable().transient_local(),
      [this](hb_robot_interfaces::msg::CutInspectionSeeds::ConstSharedPtr seeds) {
        if (seeds->features.size()!=seeds->states.size() || seeds->states.size()>4096) return;
        std::lock_guard<std::mutex> lock(seed_mutex_);
        cut_seeds_ = seeds;
      });
    using Evaluate = hb_robot_interfaces::srv::EvaluateKerf;
    using Replay = hb_robot_interfaces::srv::ReplayKerf;
    replay_service_ = create_service<Replay>(
        "~/replay_observation",
        [this](Replay::Request::SharedPtr request, Replay::Response::SharedPtr response) {
          if (busy_) {
            response->message = "Live inspection is active";
            return;
          }
          try {
            const auto model = hb_perception::ProfileLibrary::find(request->profile_name);
            const auto &record = request->observation;
            if (!model || record.cut_id.empty() || record.feature_name.empty())
              throw std::invalid_argument("Missing replay identity");
            auto evidence = request->prior.required_segments.empty()
                                ? initializeEvidence(*model, {record.feature_name}, ray_config_)
                                : request->prior;
            if (!request->prior.required_segments.empty() &&
                (evidence.required_segments != std::vector<std::string>{record.feature_name} ||
                 evidence.cut_id != record.cut_id ||
                 evidence.geometry_version != record.geometry_version ||
                 evidence.pose_version != record.pose_version ||
                 evidence.execution_version != record.execution_version))
              throw std::invalid_argument("Mixed replay feature or epoch");
            hb_perception::Observation ob;
            ob.rgb = std::make_shared<sensor_msgs::msg::Image>(record.rgb);
            ob.depth_to_rgb = std::make_shared<sensor_msgs::msg::Image>(record.depth);
            ob.rgb_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(record.camera_info);
            ob.camera_pose = record.camera_pose;
            ob.stamp = rclcpp::Time(record.header.stamp);
            if (record.header.frame_id != record.camera_pose.header.frame_id ||
                record.camera_pose.child_frame_id != record.camera_info.header.frame_id ||
                std::abs((rclcpp::Time(record.rgb.header.stamp) -
                          rclcpp::Time(record.depth.header.stamp))
                             .seconds()) > 0.03)
              throw std::invalid_argument("Replay frames or RGB-D timestamps disagree");
            (void)kerfObservationCloud(
                ob, range_); // Validate encoding, dimensions, intrinsics and buffers.
            Eigen::Isometry3d world_from_cut;
            tf2::fromMsg(record.cut_pose, world_from_cut);
            const auto camera = tf2::transformToEigen(record.camera_pose);
            if (!world_from_cut.matrix().allFinite() || !camera.matrix().allFinite() ||
                !world_from_cut.linear().transpose().isApprox(world_from_cut.linear().inverse(),
                                                              1e-4) ||
                !camera.linear().transpose().isApprox(camera.linear().inverse(), 1e-4))
              throw std::invalid_argument("Invalid replay transforms");
            accumulateKerfEvidence(evidence, *model, ob, world_from_cut, ray_config_);
            evidence.header = record.header;
            evidence.cut_id = record.cut_id;
            evidence.geometry_version = record.geometry_version;
            evidence.pose_version = record.pose_version;
            evidence.execution_version = record.execution_version;
            evidence.registration_valid = true;
            response->evidence = std::move(evidence);
            response->success = true;
            response->message = "Offline TF-registered inference only";
          } catch (const std::exception &e) {
            response->message = e.what();
          }
        });
    using Definition = hb_robot_interfaces::srv::GetCutDefinition;
    definition_service_ = create_service<Definition>(
        "~/cut_definition",
        [](Definition::Request::SharedPtr request, Definition::Response::SharedPtr response) {
          const auto model = hb_perception::ProfileLibrary::find(request->profile_name);
          if (!model)
            return;
          std::ostringstream out;
          out << std::setprecision(9) << model->name << ' ' << model->height << ' ' << model->width;
          for (const auto &f : model->cut_features)
            out << "\nfeature " << f.name << ' ' << int(f.type) << ' ' << f.start.x() << ' '
                << f.start.y() << ' ' << f.end.x() << ' ' << f.end.y() << ' '
                << f.outward_normal.x() << ' ' << f.outward_normal.y();
          for (const auto &primitive : model->boundary) {
            if (const auto *line = std::get_if<hb_perception::LineSegment2D>(&primitive))
              out << "\nline " << line->a.x() << ' ' << line->a.y() << ' ' << line->b.x() << ' '
                  << line->b.y();
            else {
              const auto &arc = std::get<hb_perception::ArcSegment2D>(primitive);
              out << "\narc " << arc.center.x() << ' ' << arc.center.y() << ' ' << arc.start << ' '
                  << arc.end << ' ' << arc.radius;
            }
          }
          response->found = true;
          response->definition = out.str();
        });
    evaluate_service_ =
        create_service<Evaluate>("~/evaluate_evidence", [](Evaluate::Request::SharedPtr request,
                                                           Evaluate::Response::SharedPtr response) {
          const auto verdict = hb_robot::recovery::evaluateFeature(
              request->evidence, request->state, request->cut_id, request->feature_id,
              request->last_motion_epoch);
          response->status = verdict.status;
          response->residuals = verdict.residuals;
        });
    state_sub_ = create_subscription<hb_robot::recovery::State>(
        "/task_state", rclcpp::QoS(1).transient_local(),
        [this](hb_robot::recovery::State::ConstSharedPtr state) {
          if (state->header.stamp.sec < 0 || state->header.stamp.nanosec >= 1000000000u) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[KERF TASK RX] rejected malformed TaskState timestamp");
            return;
          }
          const double source_age = (now() - rclcpp::Time(state->header.stamp)).seconds();
          if (!std::isfinite(source_age) || source_age < -0.005 || source_age > 5.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[KERF TASK RX] rejected source_age_ms=%.1f", source_age * 1000.0);
            return;
          }
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = *state;
            received_ = Clock::now();
            source_age_at_rx_ms_ = std::max(0.0, source_age * 1000.0);
          }
          (void)taskValid();
        });
    state_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (busy_.load())
        (void)taskValid();
    });
    server_ = rclcpp_action::create_server<Action>(
        this, "inspect_cut",
        [this](const auto &, std::shared_ptr<const Action::Goal> goal) {
          const auto &p = goal->estimate.pose;
          const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                                     p.orientation.z);
          if (goal->cut_id.empty() || goal->estimate.header.frame_id != frame_ ||
              !hb_perception::ProfileLibrary::find(goal->estimate.profile_name) ||
              !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
              !std::isfinite(p.position.z) || !q.coeffs().allFinite() ||
              std::abs(q.norm() - 1.0) > 1e-3 || !std::isfinite(goal->sample_spacing) ||
              goal->sample_spacing < 0 || !std::isfinite(goal->camera_standoff) ||
              goal->camera_standoff < 0 || goal->samples_per_viewpoint > 100)
            return rclcpp_action::GoalResponse::REJECT;
          const std::set<std::string> scope(goal->required_segments.begin(),
                                            goal->required_segments.end());
          if (scope.empty() || scope.size() != goal->required_segments.size() || scope.count(""))
            return rclcpp_action::GoalResponse::REJECT;
          // Never take the platform mutex while holding state_mutex_: the
          // platform wait invokes our cancellation predicate under its own lock.
          if (!stability_->stable()) {
            RCLCPP_WARN(get_logger(), "[KERF ADMISSION] platform not stable: %s",
                        stability_->reason().c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (busy_.load() || shutting_down_.load())
            return rclcpp_action::GoalResponse::REJECT;
          const auto check = taskCheckLocked(false);
          if (!check.valid || goal->geometry_version != state_.geometry_version ||
              goal->pose_version != state_.pose_version ||
              goal->execution_version != state_.execution_version) {
            RCLCPP_WARN(get_logger(), "[KERF ADMISSION] %s goal_versions=%llu/%llu/%llu",
                        check.detail.c_str(),
                        static_cast<unsigned long long>(goal->geometry_version),
                        static_cast<unsigned long long>(goal->pose_version),
                        static_cast<unsigned long long>(goal->execution_version));
            return rclcpp_action::GoalResponse::REJECT;
          }
          expected_ = state_;
          interrupted_ = false;
          interruption_reason_.clear();
          motion_active_ = false;
          busy_.store(true);
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const auto &) {
          interrupt("operator canceled inspection");
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const auto &handle) {
          if (worker_.joinable())
            worker_.join();
          worker_ = std::thread([this, handle] {
            try {
              run(handle);
            } catch (const std::exception &error) {
              RCLCPP_ERROR(get_logger(), "[KERF WORKER] terminal reporting failed: %s",
                           error.what());
              if (rclcpp::ok() && handle->is_active()) {
                auto failed = std::make_shared<Action::Result>();
                failed->success = false;
                failed->result_code = Action::Result::MOTION_FAILED;
                failed->evidence.registration_valid = false;
                failed->message = error.what();
                try {
                  if (handle->is_canceling())
                    handle->canceled(failed);
                  else
                    handle->abort(failed);
                } catch (const std::exception &) {
                }
              }
            }
            std::lock_guard<std::mutex> commands(motion_command_mutex_);
            std::lock_guard<std::mutex> lock(state_mutex_);
            busy_.store(false);
          });
        });
  }

private:
  using Clock = std::chrono::steady_clock;
  std::size_t boundedCount(const std::string &name, int fallback, int maximum) {
    const int value = declare_parameter<int>(name, fallback);
    if (value < 1 || value > maximum)
      throw std::invalid_argument(name + " out of range");
    return static_cast<std::size_t>(value);
  }
  double positive(const std::string &name, double fallback) {
    const double v = declare_parameter<double>(name, fallback);
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument(name + " must be positive");
    return v;
  }
  std::optional<hb_perception::Observation>
  acquireFresh(hb_perception::CroppedRGBDAcquisition &capture, const rclcpp::Time &after,
               const std::function<bool()> &canceled) {
    const auto capture_window = std::chrono::milliseconds(static_cast<int64_t>(timeout_ * 1000.0));

    uint32_t timeouts = 0;

    while (rclcpp::ok() && !canceled()) {
      const auto before = capture.diagnostics();
      auto observation = capture.acquireAfter(after, capture_window);

      // Authorization may have changed while
      // acquireAfter() was blocking.
      if (!rclcpp::ok() || canceled())
        return std::nullopt;

      if (observation)
        return observation;

      ++timeouts;
      const auto d = capture.diagnostics();
      RCLCPP_WARN(get_logger(),
          "[KERF RGBD WAIT] topic=%s timeouts=%u matched=%zu arrivals=%zu callbacks=%zu "
          "invalid=%zu resolved=%zu tf_failures=%zu executor_ticks=%zu",
          d.topic.c_str(), timeouts, d.matched_publishers,
          d.middleware_arrivals-before.middleware_arrivals, d.received-before.received,
          d.invalid-before.invalid, d.resolved-before.resolved,
          d.tf_failures-before.tf_failures, d.executor_ticks-before.executor_ticks);

      capture.logCaptureFailure(after);

      // Avoid a busy loop if the receiver returns
      // immediately rather than waiting for its window.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return std::nullopt;
  }
  struct TaskCheck {
    bool valid;
    std::string detail;
  };

  TaskCheck taskCheckLocked(bool check_versions) const {
    const double callback_age_ms =
        received_ == Clock::time_point{}
            ? std::numeric_limits<double>::infinity()
            : std::chrono::duration<double, std::milli>(Clock::now() - received_).count();
    const double effective_age_ms = callback_age_ms + source_age_at_rx_ms_;
    const bool fresh = std::isfinite(effective_age_ms) && effective_age_ms >= 0 &&
                       effective_age_ms < task_state_timeout_ms_;
    const bool geometry = hb_robot::recovery::geometryPermitted(state_);
    const bool versions =
        !check_versions || (state_.geometry_version == expected_.geometry_version &&
                            state_.pose_version == expected_.pose_version &&
                            state_.scene_version == expected_.scene_version &&
                            state_.execution_version == expected_.execution_version &&
                            state_.cut_environment_id == expected_.cut_environment_id);
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << "task_age_ms=" << effective_age_ms
        << " limit_ms=" << task_state_timeout_ms_ << " callback_age_ms=" << callback_age_ms
        << " source_age_at_rx_ms=" << source_age_at_rx_ms_ << " fresh=" << fresh
        << " geometry_ok=" << geometry << " versions_ok=" << versions
        << " supervision=" << state_.supervision_valid << " imu=" << state_.imu_stable
        << " scene_valid=" << state_.scene_valid << " pose_valid=" << state_.workpiece_pose_valid
        << " motion_fault=" << state_.motion_fault_active
        << " process_fault=" << state_.process_fault_active
        << " geometry=" << state_.geometry_version << '/' << expected_.geometry_version
        << " pose=" << state_.pose_version << '/' << expected_.pose_version
        << " scene=" << state_.scene_version << '/' << expected_.scene_version
        << " execution=" << state_.execution_version << '/' << expected_.execution_version;
    return {fresh && geometry && versions, out.str()};
  }

  void stopInterruptedMotion() {
    // Keep STOP active while execute() is outstanding, including late goal
    // acceptance. A single STOP before goal acceptance can miss that motion.
    // The existing 50 ms timer retries only during interrupted execution.
    std::lock_guard<std::mutex> commands(motion_command_mutex_);
    bool stop = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stop = busy_.load() && interrupted_ && motion_active_;
    }
    if (stop && move_)
      move_->stop();
  }

  void interrupt(const std::string &reason) {
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (busy_.load() && !interrupted_) {
        interrupted_ = true;
        interruption_reason_ = reason;
        first = true;
      }
    }
    if (first)
      RCLCPP_WARN(get_logger(), "[KERF INTERRUPT] %s", reason.c_str());
    stopInterruptedMotion();
  }

  bool taskValid() {
    bool valid = false, first = false;
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto check = taskCheckLocked(busy_.load());
      if (busy_.load() && !check.valid && !interrupted_) {
        interrupted_ = true;
        interruption_reason_ = check.detail;
        first = true;
      }
      valid = check.valid && (!busy_.load() || !interrupted_);
      reason = interruption_reason_;
    }
    if (first)
      RCLCPP_WARN(get_logger(), "[KERF INTERRUPT] %s", reason.c_str());
    if (!valid)
      stopInterruptedMotion();
    return valid;
  }

  // ApproveMotion.message is bool in the supplied interface; some deployed
  // versions use string. Keep both source-compatible without changing messages.
  template<class Response>
  static void setApprovalMessage(Response& response, bool accepted, const std::string& text) {
    if constexpr (std::is_assignable_v<decltype(response.message)&, std::string>)
      response.message = text;
    else
      response.message = accepted;
  }

  void clearApproval() {
    std::lock_guard<std::mutex> lock(approval_mutex_);
    approval_pending_ = approval_decided_ = motion_approved_ = false;
    approval_description_.clear();
    approval_cv_.notify_all();
  }

  enum class ApprovalOutcome { Approved, Rejected, Canceled };
  ApprovalOutcome waitForInspectionApproval(const moveit_msgs::msg::DisplayTrajectory& display,
      const std::string& description, const std::function<bool()>& canceled) {
    if (canceled()) return ApprovalOutcome::Canceled;
    {
      std::lock_guard<std::mutex> lock(approval_mutex_);
      approval_pending_ = approval_decided_ = motion_approved_ = false;
      approval_description_ = description;
      // Publish the complete, fixed route including return home before accepting
      // a decision. No planning occurs after this approval.
      display_traj_pub_->publish(display);
      approval_pending_ = require_plan_approval_;
    }
    if (!require_plan_approval_)
      return canceled() ? ApprovalOutcome::Canceled : ApprovalOutcome::Approved;
    RCLCPP_INFO(get_logger(), "Awaiting inspection motion approval: %s (service=%s)",
        description.c_str(), approval_service_->get_service_name());
    for (;;) {
      // The planning deadline does not include human review time. Live
      // authorization and cancellation still apply throughout the wait.
      if (canceled()) {
        clearApproval();
        return ApprovalOutcome::Canceled;
      }
      std::unique_lock<std::mutex> lock(approval_mutex_);
      if (approval_decided_) {
        const auto result = motion_approved_ ? ApprovalOutcome::Approved : ApprovalOutcome::Rejected;
        approval_pending_ = approval_decided_ = motion_approved_ = false;
        approval_description_.clear();
        return result;
      }
      approval_cv_.wait_for(lock, std::chrono::milliseconds(50));
    }
  }

  bool executeInspectionMotion(const Plan &plan, const std::function<bool()> &canceled) {
    if (canceled())
      return false;
    {
      std::lock_guard<std::mutex> commands(motion_command_mutex_);
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (interrupted_ || shutting_down_.load())
        return false;
      motion_active_ = true;
    }
    const auto clear_motion = [this] {
      std::lock_guard<std::mutex> commands(motion_command_mutex_);
      std::lock_guard<std::mutex> lock(state_mutex_);
      motion_active_ = false;
    };
    try {
      // MoveGroup and its action clients live for the lifetime of this server.
      const auto result = move_->execute(plan);
      clear_motion();
      return result == moveit::core::MoveItErrorCode::SUCCESS && !canceled();
    } catch (...) {
      interrupt("MoveGroup execution threw an exception");
      clear_motion();
      throw;
    }
  }

  static bool validRigidTransform(const Eigen::Isometry3d &pose) {
    if (!pose.matrix().allFinite())
      return false;
    const Eigen::Matrix3d rotation = pose.linear();
    return std::abs(rotation.determinant() - 1.0) < 1e-6 &&
           (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6);
  }

  // acquireAfter() applies one lower bound to BOTH image stamps. Keep separate
  // cursors so a small RGB/depth offset does not make the slower image compare
  // against the previous faster image. Never reuse either image as evidence.
  struct ObservationCursor {
    int64_t settled, observation, rgb, depth;
    explicit ObservationCursor(int64_t after)
        : settled(after), observation(after), rgb(after), depth(after) {}
    int64_t lowerBound() const { return std::max(settled, std::min(rgb, depth)); }
    bool advance(int64_t obs, int64_t color, int64_t aligned_depth) {
      const bool fresh = obs > observation && color > rgb && aligned_depth > depth &&
                         obs > settled && color > settled && aligned_depth > settled;
      observation = std::max(observation, obs);
      rgb = std::max(rgb, color);
      depth = std::max(depth, aligned_depth);
      return fresh;
    }
  };

  void publishCameraPoses(const geometry_msgs::msg::PoseArray &optical,
                          const Eigen::Isometry3d &link_from_optical) {
    poses_pub_->publish(optical);
    geometry_msgs::msg::PoseArray links;
    links.header = optical.header;
    for (const auto &pose : optical.poses) {
      Eigen::Isometry3d world_from_optical;
      tf2::fromMsg(pose, world_from_optical);
      const Eigen::Isometry3d world_from_link = world_from_optical * link_from_optical.inverse();
      links.poses.push_back(tf2::toMsg(world_from_link));
    }
    link_poses_pub_->publish(links);
  }

  void run(const std::shared_ptr<Handle> &handle) {
    auto result = std::make_shared<Action::Result>();
    uint8_t exception_code = Action::Result::PLANNING_FAILED;
    hb_perception::PointCloud cloud;
    auto finish = [&](uint8_t code, std::string message) {
      clearApproval();
      if (code == Action::Result::SUCCESS)
        (void)taskValid();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (interrupted_) {
          code = Action::Result::CANCELLED;
          message += "; interruption: " + interruption_reason_;
        }
      }
      if (handle->is_canceling())
        code = Action::Result::CANCELLED;
      result->success = code == Action::Result::SUCCESS;
      result->result_code = code;
      result->interpretation = Action::Result::INCONCLUSIVE;
      result->cut_complete = false;
      result->confidence = 0;
      result->message = message;
      const auto goal = handle->get_goal();
      result->evidence.header.stamp = now();
      result->evidence.header.frame_id = "workpiece";
      result->evidence.cut_id = goal->cut_id;
      result->evidence.geometry_version = goal->geometry_version;
      result->evidence.pose_version = goal->pose_version;
      result->evidence.execution_version = goal->execution_version;
      result->evidence.registration_valid = result->success;
      if (!infer_evidence_)
        result->evidence.unknown_fraction = 1.0;
      pcl::toROSMsg(cloud, result->kerf_cloud);
      result->kerf_cloud.header.frame_id = frame_;
      result->kerf_cloud.header.stamp = now();
      cloud_pub_->publish(result->kerf_cloud);
      // A late cancellation/supervision loss must invalidate even partially
      // collected evidence. Partial observations remain available for diagnosis.
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (interrupted_ || handle->is_canceling()) {
        result->success = false;
        result->result_code = Action::Result::CANCELLED;
        result->evidence.registration_valid = false;
        if (interrupted_ && result->message.find(interruption_reason_) == std::string::npos)
          result->message += "; interruption: " + interruption_reason_;
      }
      if (result->success)
        RCLCPP_INFO(get_logger(), "[KERF RESULT] %s", result->message.c_str());
      else
        RCLCPP_ERROR(get_logger(), "[KERF RESULT] code=%u views=%u/%u points=%zu: %s",
                     static_cast<unsigned>(result->result_code), result->viewpoints_captured,
                     result->viewpoints_requested, cloud.size(), result->message.c_str());
      if (handle->is_canceling())
        handle->canceled(result);
      else if (result->success)
        handle->succeed(result);
      else
        handle->abort(result);
    };
    auto canceled = [&] {
      if (handle->is_canceling())
        interrupt("operator canceled inspection");
      if (!rclcpp::ok() || shutting_down_.load())
        interrupt("server shutdown");
      return handle->is_canceling() || !rclcpp::ok() || shutting_down_.load() || !taskValid();
    };
    try {
      const auto goal = handle->get_goal();
      auto profile = *hb_perception::ProfileLibrary::find(goal->estimate.profile_name);
      result->evidence = initializeEvidence(profile, goal->required_segments, ray_config_);
      // Keep the full boundary for ray intersections, but route only the requested features.
      profile.cut_features.erase(
          std::remove_if(profile.cut_features.begin(), profile.cut_features.end(),
                         [&](const auto &f) {
                           return std::find(goal->required_segments.begin(),
                                            goal->required_segments.end(),
                                            f.name) == goal->required_segments.end();
                         }),
          profile.cut_features.end());
      Eigen::Isometry3d world_from_cut;
      tf2::fromMsg(goal->estimate.pose, world_from_cut);
      const auto cut_from_world = world_from_cut.inverse();
      // Clear old visualization/evidence before calibration or planning.
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = frame_;
      poses.header.stamp = now();
      poses_pub_->publish(poses);
      link_poses_pub_->publish(poses);
      sensor_msgs::msg::PointCloud2 empty;
      pcl::toROSMsg(cloud, empty);
      empty.header = poses.header;
      cloud_pub_->publish(empty);
      if (profile.cut_features.empty()) {
        finish(Action::Result::PLANNING_FAILED, "No requested material features");
        return;
      }

      if (!rgbd_acquisition_) {
        finish(Action::Result::ACQUISITION_FAILED, "RGB-D acquisition receiver not initialized");
        return;
      }

      auto &capture = *rgbd_acquisition_;

      exception_code = Action::Result::STABILITY_TIMEOUT;

      if (!stability_->waitForStable(
              std::chrono::milliseconds(static_cast<int64_t>(stability_timeout_ * 1000.0)),
              canceled)) {
        finish(canceled() ? Action::Result::CANCELLED : Action::Result::STABILITY_TIMEOUT,
               "Platform not stable before RGB-D calibration: " + stability_->reason());

        return;
      }

      exception_code = Action::Result::ACQUISITION_FAILED;

      auto calibration = acquireFresh(capture, now(), canceled);

      if (!calibration) {
        finish(canceled() ? Action::Result::CANCELLED : Action::Result::ACQUISITION_FAILED,
               "Initial RGB-D acquisition interrupted");

        return;
      }

      if (!calibration->rgb || !calibration->depth_to_rgb || !calibration->rgb_camera_info) {
        finish(Action::Result::ACQUISITION_FAILED, "Incomplete initial RGB-D observation");
        return;
      }
      const auto &info = *calibration->rgb_camera_info;
      if (info.width == 0 || info.height == 0 || !std::isfinite(info.k[0]) ||
          !std::isfinite(info.k[4]) || !std::isfinite(info.k[2]) || !std::isfinite(info.k[5]) ||
          info.k[0] <= 0 || info.k[4] <= 0) {
        finish(Action::Result::REGISTRATION_FAILED, "Invalid RGB optical camera intrinsics");
        return;
      }
      if (canceled() || !stability_->stable()) {
        finish(Action::Result::STABILITY_TIMEOUT,
               "Calibration interrupted or platform no longer stable");
        return;
      }
      const std::string optical_frame = info.header.frame_id;
      if (optical_frame.empty() || calibration->camera_pose.child_frame_id != optical_frame ||
          calibration->camera_pose.header.frame_id != frame_) {
        finish(Action::Result::REGISTRATION_FAILED,
               "RGB-D calibration and optical TF frames disagree");
        return;
      }
      // A body link (+X forward) and its RGB optical child (+Z forward) are not
      // interchangeable. Use the driver's calibrated rotation AND translation.
      exception_code = Action::Result::REGISTRATION_FAILED;
      const Eigen::Isometry3d link_from_optical = tf2::transformToEigen(tf_->lookupTransform(
          camera_, optical_frame, calibration->stamp, rclcpp::Duration::from_seconds(1.0)));
      const Eigen::Isometry3d world_from_optical = tf2::transformToEigen(calibration->camera_pose);
      if (!validRigidTransform(link_from_optical) || !validRigidTransform(world_from_optical)) {
        finish(Action::Result::REGISTRATION_FAILED, "Invalid calibrated camera transformation");
        return;
      }

      if (canceled()) {
        finish(Action::Result::CANCELLED, "Interrupted before route planning");
        return;
      }
      auto start = move_->getCurrentState(5.0);
      if (!start) {
        finish(Action::Result::PLANNING_FAILED, "No current robot state");
        return;
      }
      start->update();
      const auto *jmg = start->getJointModelGroup(group_);
      if (!jmg)
        throw std::runtime_error("Inspection joint group is absent");
      const auto &home_names = jmg->getDefaultStateNames();
      if (std::find(home_names.begin(), home_names.end(), "inspection_home") == home_names.end()) {
        finish(Action::Result::PLANNING_FAILED, "SRDF inspection_home is missing");
        return;
      }
      auto home = std::make_shared<moveit::core::RobotState>(*start);
      home->setToDefaultValues(jmg, "inspection_home");
      home->update();
      // Use a deterministic calibrated optical roll reference from inspection_home.
      const Eigen::Isometry3d reference_optical =
          home->getGlobalLinkTransform(camera_) * link_from_optical;
      const auto stations = motion::inspectionStations(
          profile, world_from_cut, goal->sample_spacing > 0 ? goal->sample_spacing : spacing_,
          goal->camera_standoff > 0 ? goal->camera_standoff : standoff_,
          reference_optical.linear());
      result->viewpoints_requested = static_cast<uint32_t>(stations.size());
      poses.header.stamp = now();
      for (const auto &station : stations)
        poses.poses.push_back(tf2::toMsg(station.camera_poses.front()));
      publishCameraPoses(poses, link_from_optical);
      const Eigen::Quaterniond reference_rotation(reference_optical.linear());
      RCLCPP_INFO(get_logger(),
                  "[KERF STATIONS] count=%zu reference=inspection_home_optical frame=%s q_xyzw=[%.6f %.6f "
                  "%.6f %.6f] preview=%d",
                  stations.size(), optical_frame.c_str(), reference_rotation.x(),
                  reference_rotation.y(), reference_rotation.z(), reference_rotation.w(),
                  int(preview_));
      if (stations.empty()) {
        finish(Action::Result::PLANNING_FAILED, "No material features");
        return;
      }
      if (preview_) {
        finish(Action::Result::PLANNING_FAILED, "Preview only: calibrated optical/body viewpoints "
                                                "published; no motion or kerf acquisition");
        return;
      }
      motion::ExplorationRequest view_request = view_request_;
      view_request.roll_axis = Eigen::Vector3d::UnitZ();
      view_request.link_from_view = link_from_optical;
      const auto seesTarget = [&](const Eigen::Isometry3d &optical_pose,
                                  const Eigen::Vector3d &target) {
        const Eigen::Vector3d p = optical_pose.inverse() * target;
        if (!p.allFinite() || p.z() <= 0.02 || p.z() > range_)
          return false;
        const double u = info.k[0] * p.x() / p.z() + info.k[2];
        const double v = info.k[4] * p.y() / p.z() + info.k[5];
        return std::isfinite(u) && std::isfinite(v) && u >= 2 && v >= 2 && u + 2 < info.width &&
               v + 2 < info.height;
      };
      RCLCPP_INFO(get_logger(),
                  "[KERF CAMERA] ik_link=%s optical_frame=%s rgbd=%s position_tol=%.3f "
                  "roll_tol=%.3f samples=%zu solutions=%zu",
                  camera_.c_str(), optical_frame.c_str(), rgbd_topic_.c_str(), view_request.pos_tol,
                  view_request.roll_tol, view_request.num_candidates, max_view_solutions_);
      exception_code = Action::Result::PLANNING_FAILED;
      if (!scene_monitor_->requestPlanningSceneState("/get_planning_scene")) {
        finish(Action::Result::PLANNING_FAILED, "Could not synchronize the MoveIt planning scene");
        return;
      }
      planning_scene::PlanningScenePtr scene;
      {
        planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
        scene = locked->diff();
        scene->decoupleParent();
      }
      motion::CutCandidateGraph graph;
      std::vector<moveit::core::RobotStatePtr> states{start};
      const auto no_capture = stations.size();
      std::vector<std::size_t> station_ids{no_capture};
      const auto root = graph.addNode(0, 0);
      std::vector<motion::NodeId> previous{root};
      std::set<motion::EdgeId> local_edges;
      std::map<motion::EdgeId, Plan> plans;
      motion::GraphRoute route;
      motion::NodeId sink;
      // Scope ends before execution/acquisition. Nested edge budgets inherit
      // both this overall deadline and cancellation/supervision checks.
      {
        motion::PlanningBudget planning_budget(planning_timeout_, canceled);
        motion::PlanningBudgetScope planning_scope(planning_budget);
        motion::ExplorationPlanner explorer(move_->getRobotModel(), group_, camera_);
        hb_robot_interfaces::msg::CutInspectionSeeds::ConstSharedPtr cached;
        { std::lock_guard<std::mutex> lock(seed_mutex_); cached = cut_seeds_; }
        if (cached && !motion::inspectionSeedsMatch(*cached, goal->cut_id,
            goal->estimate.profile_name, group_, frame_, goal->geometry_version, goal->pose_version))
          cached.reset();
        std::size_t layer_base = 1;
        for (std::size_t begin=0; begin<stations.size();) {
          motion::PlanningBudgetScope::check("inspection feature sampling");
          std::size_t end=begin+1;
          while (end<stations.size() && stations[end].feature==stations[begin].feature) ++end;
          std::vector<moveit::core::RobotState> bases;
          const auto add_seed = [&](const moveit::core::RobotState& state) {
            if (std::none_of(bases.begin(), bases.end(), [&](const auto& old) {
                return old.distance(state,jmg)<0.02;
            })) bases.push_back(state);
          };
          // Prefer this feature's cut branches, then allow other features as
          // hints (e.g. the opposite web face omitted from the cutting route).
          std::size_t reused = 0;
          if (cached) for (bool same_feature : {true, false}) {
            for (std::size_t i=0; i<cached->states.size() && reused<8; ++i) {
              if ((cached->features[i]==stations[begin].feature)!=same_feature) continue;
              auto state = motion::inspectionSeedState(cached->states[i], *start, jmg);
              if (!state) continue;
              const auto old_size = bases.size();
              add_seed(*state);
              if (bases.size()!=old_size) ++reused;
            }
          }
          add_seed(*start); add_seed(*home);
          for (const auto& name : direction_seed_names_)
            if (auto posture=motion::namedCutPosture(*start,jmg,name)) add_seed(*posture);
          for (const auto from : previous) add_seed(*states[from]);
          std::vector<motion::IKSeed> seeds;
          for (const auto& base : bases) seeds.push_back({base, true});
          seeds.push_back({*start, false});
          seeds.push_back({*home, false});
          // Use the same bounded perturbation/global-branch generator as cutting.
          std::mt19937 rng(0);
          for (std::size_t i=2; i<seed_trials_; ++i) {
            auto seed=motion::sampleIKSeed(*start, &bases[(i-2)%bases.size()], jmg,
              i, 2, local_seed_radius_, rng);
            // As in cutting, a global branch gets a chance before local
            // warm starts can fill the retention quota.
            if (i==2) seeds.insert(seeds.begin()+std::min<std::size_t>(2,seeds.size()),std::move(seed));
            else seeds.push_back(std::move(seed));
          }
          std::vector<Eigen::Isometry3d> nominal;
          for (std::size_t i=begin; i<end; ++i) nominal.push_back(stations[i].camera_poses.front());
          const auto sampled=explorer.sampleSweep(seeds, nominal, view_request, scene,
            max_view_solutions_, local_consistency_, station_joint_step_,
            [&](std::size_t i, const auto& pose) { return seesTarget(pose,stations[begin+i].target); }, canceled);
          const auto& stats=sampled.stats;
          if (stats.stopped) {
            finish(Action::Result::CANCELLED,"Inspection sweep canceled or live supervision lost"); return;
          }
          RCLCPP_INFO(get_logger(),
            "[KERF SWEEP] feature=%s stations=%zu middle=%zu seeds=%zu reused_cut=%zu poses=%zu "
            "ik=%zu ik_failed=%zu collision=%zu framing=%zu continuation_failed=%zu branches=%zu",
            stations[begin].feature.c_str(),end-begin,begin+(end-begin)/2+1,seeds.size(),reused,
            stats.candidates,stats.ik_attempts,stats.ik_failed,stats.collision_rejected,
            stats.view_rejected,sampled.continuation_failed,sampled.sweeps.size());
          motion::PlanningBudgetScope::check("inspection feature sampled");
          if (sampled.sweeps.empty()) {
            finish(Action::Result::PLANNING_FAILED,
              "No continuous inspection sweep for feature="+stations[begin].feature); return;
          }
          const auto visits=motion::inspectionSweepOrder(end-begin);
          std::vector<motion::NodeId> exits;
          for (const auto& sweep : sampled.sweeps) {
            motion::NodeId last=root;
            for (std::size_t k=0; k<visits.size(); ++k) {
              const auto& visit=visits[k];
              auto state=std::make_shared<moveit::core::RobotState>(sweep[visit.station].robot_state);
              const auto id=graph.addNode(layer_base+k,0);
              states.push_back(state);
              station_ids.push_back(visit.capture ? begin+visit.station : no_capture);
              if (k==0) {
                for (const auto from : previous) graph.addEdge(from,id,states[from]->distance(*state,jmg));
              } else {
                local_edges.insert(graph.addEdge(last,id,states[last]->distance(*state,jmg)));
              }
              last=id;
            }
            exits.push_back(last);
          }
          previous=std::move(exits);
          layer_base+=visits.size();
          begin=end;
        }
        // The return home remains mandatory and is checked before any execution.
        const auto home_id=graph.addNode(layer_base++,0);
        states.push_back(home);station_ids.push_back(no_capture);
        for (const auto from : previous) graph.addEdge(from,home_id,states[from]->distance(*home,jmg));
        sink=graph.addNode(layer_base,0);
        const auto final_edge=graph.addEdge(home_id,sink,0);
        graph.edge(final_edge).status=motion::TransitStatus::Valid;
        auto transit_options=transit_options_;
        transit_options.scene=scene;
        home->copyJointGroupPositions(jmg,transit_options.home_joints);
        for (;;) {
          motion::PlanningBudgetScope::check("inspection route search");
          route=graph.cheapestRoute(root,sink);
          if (!route.found()) {
            finish(Action::Result::PLANNING_FAILED,"No checked PTP route through every inspection sweep and home"); return;
          }
          bool complete=true;
          // Reject a bad local sweep before spending a long entry/exit budget.
          // Keep validating this route after success; only re-search on failure.
          auto validation_order=route.edges;
          std::stable_sort(validation_order.begin(),validation_order.end(),[&](auto a,auto b) {
            return local_edges.count(a)>local_edges.count(b);
          });
          for (const auto edge_id : validation_order) {
            auto& edge=graph.edge(edge_id);
            if (edge.status==motion::TransitStatus::Valid) continue;
            motion::PlanningBudgetScope::check("inspection transit");
            const bool local=local_edges.count(edge_id)>0;
            auto options=transit_options;
            const double edge_budget_s =
              local ? local_transit_budget_ : transit_budget_;
            // Reuse InspectProfile's transit portfolio with the same conservative
            // scaling. The nested budget still bounds every local/entry edge.
            if (local) {
              options.guided_max_ptp_calls=1;
              options.home_joints.clear();
            }
            std::optional<Plan> plan;
            try {
              motion::PlanningBudget edge_budget(edge_budget_s);
              motion::PlanningBudgetScope edge_scope(edge_budget);
              plan=motion::planTransit(*move_,get_logger(),*states[edge.from],*states[edge.to],options);
            } catch (const motion::PlanningInterrupted& stopped) {
              if (stopped.reason==motion::PlanningInterrupted::Reason::Canceled) throw;
              motion::PlanningBudgetScope::check("inspection transit deadline");
              RCLCPP_WARN(get_logger(),"[KERF TRANSIT] edge=%zu exhausted %.1fs budget",edge_id,edge_budget_s);
            }
            if (!plan || !scene->isPathValid(plan->start_state_,plan->trajectory_,group_)) {
              edge.status=motion::TransitStatus::Invalid;
              complete=false;
              RCLCPP_WARN(get_logger(),"[KERF TRANSIT] edge=%zu local=%d failed; trying another middle branch",
                edge_id,local);
              break;
            }
            edge.status=motion::TransitStatus::Valid;
            edge.actual_cost=edge.estimated_cost;
            plans.emplace(edge_id,std::move(*plan));
          }
          if (complete) break;
        }
        motion::PlanningBudgetScope::check("inspection route complete");
      }
      poses.poses.clear();
      poses.header.stamp = now();
      for (const auto id : route.nodes) {
        if (id != root && id != sink && station_ids[id] < stations.size())
          poses.poses.push_back(
              tf2::toMsg(states[id]->getGlobalLinkTransform(camera_) * link_from_optical));
      }
      publishCameraPoses(poses, link_from_optical);
      RCLCPP_INFO(get_logger(),
                  "[KERF ROUTE] stations=%zu validated_transits=%zu; return_home included",
                  stations.size(), route.edges.size() - 1);
      moveit_msgs::msg::DisplayTrajectory display;
      display.model_id = move_->getRobotModel()->getName();
      for (const auto edge_id : route.edges) {
        if (graph.edge(edge_id).to == sink) continue;
        const auto& plan = plans.at(edge_id);
        if (display.trajectory.empty()) display.trajectory_start = plan.start_state_;
        display.trajectory.push_back(plan.trajectory_);
      }
      route_traj_pub_->publish(display);
      if (skip_motion_) {
        display_traj_pub_->publish(display);
        finish(Action::Result::PLANNING_FAILED,
               "skip_inspection_motion: complete route published; no motion or acquisition");
        return;
      }
      const auto approval = waitForInspectionApproval(display,
          "complete kerf inspection route, including return to inspection_home", canceled);
      if (approval != ApprovalOutcome::Approved) {
        finish(Action::Result::CANCELLED, approval == ApprovalOutcome::Rejected
            ? "Operator rejected kerf inspection route" : "Inspection approval invalidated or canceled");
        return;
      }
      // The approved route is fixed. Each leg still checks live authorization,
      // the actual start and the latest collision scene before execution.
      exception_code = Action::Result::MOTION_FAILED;
      const auto samples = goal->samples_per_viewpoint ? goal->samples_per_viewpoint : 3;
      std::set<std::size_t> captured_stations;
      for (const auto edge_id : route.edges) {
        const auto &edge = graph.edge(edge_id);
        if (edge.to == sink)
          continue;
        if (canceled()) {
          finish(Action::Result::CANCELLED, "Canceled");
          return;
        }
        // Do not execute a stale planned start after external robot movement.
        auto current = move_->getCurrentState(2.0);
        if (!current || current->distance(*states[edge.from], jmg) > 0.05) {
          finish(Action::Result::MOTION_FAILED, "Robot moved since route planning");
          return;
        }
        {
          planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
          if (!locked->isPathValid(plans.at(edge_id).start_state_, plans.at(edge_id).trajectory_,
                                   group_)) {
            finish(Action::Result::MOTION_FAILED,
                   "Inspection trajectory invalid in latest planning scene");
            return;
          }
        }
        if (canceled()) {
          finish(Action::Result::CANCELLED, "Canceled before inspection motion");
          return;
        }
        exception_code = Action::Result::MOTION_FAILED;
        if (!executeInspectionMotion(plans.at(edge_id), canceled)) {
          finish(Action::Result::MOTION_FAILED, "Inspection motion failed");
          return;
        }
        if (station_ids[edge.to] == no_capture)
          continue;

        if (!stability_->waitForStable(
                std::chrono::milliseconds(static_cast<int>(stability_timeout_ * 1000)), canceled)) {
          if (canceled()) {
            finish(Action::Result::CANCELLED, "Canceled while waiting for stability");
            return;
          }
          finish(Action::Result::STABILITY_TIMEOUT,
                 "Platform did not settle: " + stability_->reason());
          return;
        }

        ObservationCursor cursor(now().nanoseconds());
        auto last_sample_stamp = now();

        const auto index = station_ids[edge.to];

        exception_code = Action::Result::ACQUISITION_FAILED;

        const auto station_points_before = cloud.size();

        uint32_t sample = 0;
        uint32_t received = 0;
        uint32_t rejected = 0;

        while (sample < samples && rclcpp::ok() && !canceled()) {
          // Re-establish stability before accepting a new
          // observation if the platform is temporarily unsettled.
          if (!stability_->stable()) {
            if (!stability_->waitForStable(
                    std::chrono::milliseconds(static_cast<int64_t>(stability_timeout_ * 1000.0)),
                    canceled)) {
              finish(canceled() ? Action::Result::CANCELLED : Action::Result::STABILITY_TIMEOUT,
                     "Platform did not settle during acquisition: " + stability_->reason());

              return;
            }

            // Never reuse observations from before settling.
            cursor = ObservationCursor(now().nanoseconds());
          }

          const rclcpp::Time boundary(cursor.lowerBound(), get_clock()->get_clock_type());
          auto ob = acquireFresh(capture, boundary, canceled);
          if (!ob)
            break;
          ++received;
          if (canceled())
            break;
          if (!ob->rgb || !ob->depth_to_rgb || !ob->rgb_camera_info) {
            ++rejected;
            // A malformed cached observation must not be retried forever.
            (void)cursor.advance(ob->stamp.nanoseconds(), ob->stamp.nanoseconds(),
                                 ob->stamp.nanoseconds());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[KERF RETRY] Incomplete RGB-D observation");
            continue;
          }
          const rclcpp::Time depth_stamp(ob->depth_to_rgb->header.stamp);
          const rclcpp::Time rgb_stamp(ob->rgb->header.stamp);
          const bool fresh = cursor.advance(ob->stamp.nanoseconds(), rgb_stamp.nanoseconds(),
                                            depth_stamp.nanoseconds());
          const double depth_skew = std::abs((depth_stamp - ob->stamp).seconds());
          const double rgb_depth_skew = std::abs((rgb_stamp - depth_stamp).seconds());
          if (!fresh || depth_skew > 0.03 || rgb_depth_skew > 0.03) {
            ++rejected;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[KERF RETRY] RGB-D timing: new_images=%d depth_skew=%.4f rgb_depth_skew=%.4f",
                int(fresh), depth_skew, rgb_depth_skew);
            continue;
          }
          if (!stability_->stable()) {
            ++rejected;
            cursor = ObservationCursor(now().nanoseconds());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[KERF RETRY] Platform unstable at acquisition");
            continue;
          }

          // A changed optical frame or calibration is not
          // simply a dropped camera frame. The planned route
          // depends on the original camera calibration.
          if (ob->camera_pose.header.frame_id != frame_ ||
              ob->camera_pose.child_frame_id != optical_frame ||
              ob->rgb_camera_info->header.frame_id != optical_frame ||
              ob->rgb_camera_info->k != info.k || ob->rgb_camera_info->width != info.width ||
              ob->rgb_camera_info->height != info.height) {
            finish(Action::Result::REGISTRATION_FAILED,
                   "Optical frame or camera calibration changed");

            return;
          }

          const auto camera_pose = tf2::transformToEigen(ob->camera_pose);

          const Eigen::Isometry3d expected =
              states[edge.to]->getGlobalLinkTransform(camera_) * link_from_optical;

          if (!validRigidTransform(camera_pose)) {
            finish(Action::Result::REGISTRATION_FAILED, "Invalid measured optical transformation");
            return;
          }

          const double position_error = (camera_pose.translation() - expected.translation()).norm();

          const double angle_error =
              Eigen::AngleAxisd(expected.linear().transpose() * camera_pose.linear()).angle();

          const bool target_visible = seesTarget(camera_pose, stations[index].target);
          RCLCPP_INFO(get_logger(),
                      "[KERF REGISTRATION] station=%zu position_error_mm=%.3f angle_error_deg=%.3f "
                      "target_visible=%d observation_age_ms=%.1f",
                      index + 1, position_error * 1000.0, angle_error * 180.0 / 3.141592653589793,
                      int(target_visible), (now() - ob->stamp).seconds() * 1000.0);
          if (!std::isfinite(position_error) || !std::isfinite(angle_error) ||
              position_error > 0.02 || angle_error > 0.05 || !target_visible) {
            finish(Action::Result::REGISTRATION_FAILED,
                   "Measured optical view differs from plan: "
                   "position_m=" +
                       std::to_string(position_error) +
                       " angle_rad=" + std::to_string(angle_error));

            return;
          }
          auto raw = kerfObservationCloud(*ob, range_);
          if (!raw || raw->empty()) {
            ++rejected;

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[KERF RETRY] Empty valid depth cloud");

            continue;
          }

          if (canceled())
            break;

          for (auto point : raw->points) {
            const Eigen::Vector3d world = camera_pose * point.getVector3fMap().cast<double>();
            if (!motion::inKerfBand(cut_from_world * world, profile, band_, margin_))
              continue;
            if (cloud.size() >= 5000000) {
              finish(Action::Result::ACQUISITION_FAILED,
                     "Five million point budget exceeded; reduce samples or band width");
              return;
            }
            point.x = world.x();
            point.y = world.y();
            point.z = world.z();
            cloud.push_back(point);
          }
          if (infer_evidence_)
            accumulateKerfEvidence(result->evidence, profile, *ob, world_from_cut, ray_config_,
                                   [&](const std::string &feature, double s, RayEvidence kind,
                                       double measured, double entry, double exit, bool within) {
                                     hb_robot_interfaces::msg::KerfRaySample ray;
                                     ray.header.frame_id = frame_;
                                     ray.header.stamp = ob->stamp;
                                     ray.cut_id = goal->cut_id;
                                     ray.feature_id = feature;
                                     ray.execution_version = goal->execution_version;
                                     ray.s = s;
                                     ray.classification = static_cast<uint8_t>(kind);
                                     ray.measured_distance = measured;
                                     ray.material_entry = entry;
                                     ray.material_exit = exit;
                                     ray.uncertainty = ray_config_.depth_uncertainty;
                                     ray.within_kerf = within;
                                     rays_pub_->publish(ray);
                                   });
          hb_robot_interfaces::msg::KerfObservation record;
          record.header.frame_id = frame_;
          record.header.stamp = ob->stamp;
          record.geometry_version = goal->geometry_version;
          record.pose_version = goal->pose_version;
          record.execution_version = goal->execution_version;
          record.cut_id = goal->cut_id;
          record.feature_name = stations[index].feature;
          record.viewpoint_index = index;
          record.cut_pose = goal->estimate.pose;
          record.camera_pose = ob->camera_pose;
          record.rgb = *ob->rgb;
          record.depth = *ob->depth_to_rgb;
          record.camera_info = *ob->rgb_camera_info;
          observation_pub_->publish(record);
          auto feedback = std::make_shared<Action::Feedback>();
          feedback->current_viewpoint = result->viewpoints_captured;
          feedback->total_viewpoints = stations.size();
          feedback->feature_name = stations[index].feature;
          feedback->current_pose = tf2::toMsg(camera_pose);
          feedback->samples_acquired = sample + 1;
          handle->publish_feedback(feedback);

          last_sample_stamp = ob->stamp;
          ++sample;

          RCLCPP_INFO(get_logger(),
                      "[KERF SAMPLE] feature=%s station=%zu/%zu "
                      "accepted=%u/%u received=%u rejected=%u",
                      stations[index].feature.c_str(), index + 1, stations.size(), sample,
                      static_cast<unsigned>(samples), received, rejected);
        }

        if (canceled() || !rclcpp::ok()) {
          finish(Action::Result::CANCELLED, "Inspection interrupted during acquisition");

          return;
        }

        if (sample != samples) {
          finish(Action::Result::ACQUISITION_FAILED, "Station acquisition incomplete");

          return;
        }

        if (!captured_stations.insert(index).second) {
          finish(Action::Result::ACQUISITION_FAILED, "Duplicate capture station in route");
          return;
        }
        ++result->viewpoints_captured;
        RCLCPP_INFO(get_logger(),
                    "[KERF CAPTURE] feature=%s station=%zu/%zu samples=%u added_points=%zu "
                    "total_points=%zu",
                    stations[index].feature.c_str(), index + 1, stations.size(), samples,
                    cloud.size() - station_points_before, cloud.size());
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = frame_;
        msg.header.stamp = last_sample_stamp;
        cloud_pub_->publish(msg);
      }
      if (canceled()) {
        finish(Action::Result::CANCELLED, "Canceled");
        return;
      }
      if (!stability_->waitForStable(
              std::chrono::milliseconds(static_cast<int64_t>(stability_timeout_ * 1000.0)), canceled)) {
        finish(canceled() ? Action::Result::CANCELLED : Action::Result::STABILITY_TIMEOUT,
               "Platform not stable after return home");
        return;
      }
      auto final_state = move_->getCurrentState(2.0);
      if (!final_state || final_state->distance(*home, jmg) > 0.05) {
        finish(Action::Result::MOTION_FAILED, "Return to inspection_home not confirmed");
        return;
      }
      if (result->viewpoints_captured != result->viewpoints_requested) {
        finish(Action::Result::ACQUISITION_FAILED, "Not all required stations were captured");
        return;
      }
      if (cloud.empty()) {
        finish(Action::Result::ACQUISITION_FAILED,
               "No measured points in kerf band; missing depth is not free space");
        return;
      }
      finish(Action::Result::SUCCESS,
             "Post-cut acquisition complete; separation and bridging have not been evaluated");
    } catch (const motion::PlanningInterrupted& e) {
      finish(canceled() ? Action::Result::CANCELLED : Action::Result::PLANNING_FAILED, e.what());
    } catch (const std::exception &e) {
      finish(exception_code, e.what());
    }
  }
  double task_state_timeout_ms_;
  double source_age_at_rx_ms_ = 0.0;
  std::mutex state_mutex_;
  std::mutex motion_command_mutex_;
  bool interrupted_ = false;   // state_mutex_
  bool motion_active_ = false; // state_mutex_, including execute() goal-response wait
  std::string interruption_reason_;
  std::atomic<bool> shutting_down_{false};

  hb_robot::recovery::State state_;
  hb_robot::recovery::State expected_;

  rclcpp::Publisher<hb_robot_interfaces::msg::KerfRaySample>::SharedPtr rays_pub_;
  rclcpp::Service<hb_robot_interfaces::srv::EvaluateKerf>::SharedPtr evaluate_service_;
  rclcpp::Service<hb_robot_interfaces::srv::GetCutDefinition>::SharedPtr definition_service_;
  rclcpp::Service<hb_robot_interfaces::srv::ReplayKerf>::SharedPtr replay_service_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<hb_robot::recovery::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::string group_, camera_, frame_, rgbd_topic_;
  double spacing_, standoff_, band_, margin_, range_, timeout_, stability_timeout_,
      planning_timeout_;
  std::size_t max_view_solutions_;
  std::size_t seed_trials_;
  double local_seed_radius_,local_consistency_,station_joint_step_,local_transit_budget_;
  double transit_budget_ = 15.0;
  std::vector<std::string> direction_seed_names_;
  std::mutex seed_mutex_;
  hb_robot_interfaces::msg::CutInspectionSeeds::ConstSharedPtr cut_seeds_;
  rclcpp::Subscription<hb_robot_interfaces::msg::CutInspectionSeeds>::SharedPtr cut_seeds_sub_;
  motion::ExplorationRequest view_request_;
  motion::TransitPlannerOptions transit_options_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  bool preview_, infer_evidence_;
  double planning_time_ = 5.0;
  bool require_plan_approval_ = true, skip_motion_ = false;
  std::mutex approval_mutex_;
  std::condition_variable approval_cv_;
  bool approval_pending_ = false, approval_decided_ = false, motion_approved_ = false;
  std::string approval_description_;
  rclcpp::Service<hb_robot_interfaces::srv::ApproveMotion>::SharedPtr approval_service_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_, route_traj_pub_;
  KerfRayConfig ray_config_;
  std::atomic<bool> busy_{false};
  std::thread worker_;
  std::unique_ptr<MoveGroup> move_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<hb_robot_skills::PlatformSupervisionClient> stability_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::unique_ptr<hb_perception::CroppedRGBDAcquisition> rgbd_acquisition_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr link_poses_pub_;
  rclcpp::Publisher<hb_robot_interfaces::msg::KerfObservation>::SharedPtr observation_pub_;
};
} // namespace hb_robot_skills
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hb_robot_skills::InspectCutServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  try {
    node->initialize();
    spin.join();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    if (spin.joinable())
      spin.join();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
