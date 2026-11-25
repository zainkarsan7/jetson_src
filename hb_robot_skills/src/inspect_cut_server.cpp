#include "hb_robot_skills/motion/cut_inspection.hpp"
#include "hb_robot_skills/kerf_observation.hpp"
#include "hb_robot_skills/kerf_evidence.hpp"
#include "hb_robot_skills/motion/cut_candidate_graph.hpp"
#include "hb_robot_skills/motion/exploration_planner.hpp"
#include "hb_robot_skills/motion/transit_planner.hpp"
#include "hb_robot_interfaces/action/inspect_cut.hpp"
#include "hb_robot_interfaces/msg/kerf_observation.hpp"
#include "hb_robot_interfaces/msg/torch_status.hpp"
#include "hb_robot_interfaces/msg/kerf_ray_sample.hpp"
#include "hb_robot_interfaces/srv/evaluate_kerf.hpp"
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
    infer_evidence_=declare_parameter<bool>("enable_geometric_kerf_evidence",false);
    ray_config_.allow_sampled_open_verdict=declare_parameter<bool>("allow_sampled_open_verdict",false);
    ray_config_.depth_uncertainty=positive("kerf_depth_uncertainty",0.002);
    ray_config_.half_kerf_width=positive("kerf_half_width",0.001);
    ray_config_.bin_width=positive("kerf_bin_width",0.002);
    spacing_ = positive("sample_spacing", 0.025);
    standoff_ = positive("camera_standoff", 0.30);
    band_ = positive("kerf_half_band", 0.015);
    margin_ = positive("profile_margin", 0.02);
    range_ = positive("depth_range", 1.0);
    timeout_ = positive("acquisition_timeout", 3.0);
    stability_timeout_ = positive("stability_timeout", 10.0);
    rgbd_topic_ = declare_parameter<std::string>("perception_rgbd_topic", "/k4a/rgbd");
    view_request_.pos_tol = positive("view_position_tolerance_m", 0.025);
    view_request_.roll_tol = positive("view_roll_tolerance_rad", 1.57);
    view_request_.ik_timeout = positive("view_ik_timeout_s", 0.02);
    view_request_.joint_delta = positive("view_joint_consistency_rad", 6.28);
    view_request_.num_candidates = boundedCount("view_samples", 32, 256);
    max_view_solutions_ = boundedCount("view_solutions", 6, 16);
    planning_timeout_ = positive("inspection_planning_timeout_s", 60.0);
    transit_options_.guide_link = camera_;
    
    // Preview is deliberately separate from successful acquisition.
    preview_ = declare_parameter<bool>("preview_only", true);
    declare_parameter<std::string>("perception_rgb_topic", "/k4a/rgb/image_raw");
    declare_parameter<std::string>("perception_depth_topic", "/k4a/depth_to_rgb/image_raw");
    declare_parameter<std::string>("perception_camera_info_topic", "/k4a/depth_to_rgb/camera_info");
  }
  ~InspectCutServer() override { if (worker_.joinable()) worker_.join(); }
  void initialize() {
    move_ = std::make_unique<MoveGroup>(shared_from_this(), group_);
    move_->setPlanningTime(5.0);
    move_->setPoseReferenceFrame(frame_);
    move_->setMaxVelocityScalingFactor(0.1);
    move_->setMaxAccelerationScalingFactor(0.1);
    move_->setGoalPositionTolerance(0.002);
    move_->setGoalOrientationTolerance(0.02);
    if (move_->getPlanningFrame() != frame_ || !move_->getRobotModel()->hasLinkModel(camera_))
      throw std::runtime_error("Planning frame mismatch or camera IK link absent from robot model");
    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    if (!scene_monitor_->getPlanningScene()) throw std::runtime_error("Planning scene unavailable");
    scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
     
    stability_ =
    std::make_unique<hb_robot_skills::PlatformSupervisionClient>(
        *this);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/kerf_cloud", rclcpp::QoS(1).transient_local());
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/viewpoints", rclcpp::QoS(1).transient_local());
    observation_pub_ = create_publisher<hb_robot_interfaces::msg::KerfObservation>("~/observations", 10);
    rays_pub_ = create_publisher<hb_robot_interfaces::msg::KerfRaySample>("~/ray_samples", rclcpp::QoS(100).best_effort());
    using Evaluate = hb_robot_interfaces::srv::EvaluateKerf;
    using Replay = hb_robot_interfaces::srv::ReplayKerf;
    replay_service_ = create_service<Replay>("~/replay_observation",
      [this](Replay::Request::SharedPtr request, Replay::Response::SharedPtr response) {
        if(busy_) {response->message="Live inspection is active";return;}
        try {
          const auto model=hb_perception::ProfileLibrary::find(request->profile_name);
          const auto& record=request->observation;
          if(!model || record.cut_id.empty() || record.feature_name.empty()) throw std::invalid_argument("Missing replay identity");
          auto evidence=request->prior.required_segments.empty()?initializeEvidence(*model,{record.feature_name},ray_config_):request->prior;
          if(!request->prior.required_segments.empty() &&
             (evidence.required_segments!=std::vector<std::string>{record.feature_name} || evidence.cut_id!=record.cut_id ||
              evidence.geometry_version!=record.geometry_version || evidence.pose_version!=record.pose_version ||
              evidence.execution_version!=record.execution_version)) throw std::invalid_argument("Mixed replay feature or epoch");
          hb_perception::Observation ob;
          ob.rgb=std::make_shared<sensor_msgs::msg::Image>(record.rgb);
          ob.depth_to_rgb=std::make_shared<sensor_msgs::msg::Image>(record.depth);
          ob.rgb_camera_info=std::make_shared<sensor_msgs::msg::CameraInfo>(record.camera_info);
          ob.camera_pose=record.camera_pose;ob.stamp=rclcpp::Time(record.header.stamp);
          if(record.header.frame_id!=record.camera_pose.header.frame_id ||
             record.camera_pose.child_frame_id!=record.camera_info.header.frame_id ||
             std::abs((rclcpp::Time(record.rgb.header.stamp)-rclcpp::Time(record.depth.header.stamp)).seconds())>0.03)
            throw std::invalid_argument("Replay frames or RGB-D timestamps disagree");
          (void)kerfObservationCloud(ob,range_); // Validate encoding, dimensions, intrinsics and buffers.
          Eigen::Isometry3d world_from_cut;tf2::fromMsg(record.cut_pose,world_from_cut);
          const auto camera=tf2::transformToEigen(record.camera_pose);
          if(!world_from_cut.matrix().allFinite() || !camera.matrix().allFinite() ||
             !world_from_cut.linear().transpose().isApprox(world_from_cut.linear().inverse(),1e-4) ||
             !camera.linear().transpose().isApprox(camera.linear().inverse(),1e-4))
            throw std::invalid_argument("Invalid replay transforms");
          accumulateKerfEvidence(evidence,*model,ob,world_from_cut,ray_config_);
          evidence.header=record.header;evidence.cut_id=record.cut_id;evidence.geometry_version=record.geometry_version;
          evidence.pose_version=record.pose_version;evidence.execution_version=record.execution_version;evidence.registration_valid=true;
          response->evidence=std::move(evidence);response->success=true;response->message="Offline TF-registered inference only";
        } catch(const std::exception& e) {response->message=e.what();}
      });
    using Definition = hb_robot_interfaces::srv::GetCutDefinition;
    definition_service_ = create_service<Definition>("~/cut_definition",
      [](Definition::Request::SharedPtr request, Definition::Response::SharedPtr response) {
        const auto model=hb_perception::ProfileLibrary::find(request->profile_name);
        if(!model) return;
        std::ostringstream out;out<<std::setprecision(9)<<model->name<<' '<<model->height<<' '<<model->width;
        for(const auto& f:model->cut_features)
          out<<"\nfeature "<<f.name<<' '<<int(f.type)<<' '<<f.start.x()<<' '<<f.start.y()<<' '
             <<f.end.x()<<' '<<f.end.y()<<' '<<f.outward_normal.x()<<' '<<f.outward_normal.y();
        for(const auto& primitive:model->boundary) {
          if(const auto* line=std::get_if<hb_perception::LineSegment2D>(&primitive))
            out<<"\nline "<<line->a.x()<<' '<<line->a.y()<<' '<<line->b.x()<<' '<<line->b.y();
          else {
            const auto& arc=std::get<hb_perception::ArcSegment2D>(primitive);
            out<<"\narc "<<arc.center.x()<<' '<<arc.center.y()<<' '<<arc.start<<' '<<arc.end<<' '<<arc.radius;
          }
        }
        response->found=true;response->definition=out.str();
      });
    evaluate_service_ = create_service<Evaluate>("~/evaluate_evidence",
      [](Evaluate::Request::SharedPtr request, Evaluate::Response::SharedPtr response) {
        const auto verdict=hb_robot::recovery::evaluateFeature(request->evidence,request->state,
          request->cut_id,request->feature_id,request->last_motion_epoch);
        response->status=verdict.status;response->residuals=verdict.residuals;
      });
    state_sub_=create_subscription<hb_robot::recovery::State>(
      "/task_state",rclcpp::QoS(1).transient_local(),
      [this](hb_robot::recovery::State::ConstSharedPtr state){
        if(state->header.stamp.sec<0 || state->header.stamp.nanosec>=1000000000u) return;
        const double age=(now()-rclcpp::Time(state->header.stamp)).seconds();
        if(age < -0.005 || age > 5.0) return;
        {std::lock_guard<std::mutex> lock(state_mutex_);state_=*state;
          received_=std::chrono::steady_clock::now()-std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.0,age)));}
        if(busy_ && !taskValid()) move_->stop();
      });
    torch_sub_=create_subscription<hb_robot_interfaces::msg::TorchStatus>("/cut_profile/torch_status",rclcpp::QoS(1).best_effort(),
      [this](hb_robot_interfaces::msg::TorchStatus::ConstSharedPtr status) {
        if(status->stamp.sec<0 || status->stamp.nanosec>=1000000000u) return;
        const double age=(now()-rclcpp::Time(status->stamp)).seconds();
        if(age < -0.005 || age > 5.0) return;
        {std::lock_guard<std::mutex> lock(state_mutex_);
          torch_off_=status->io_connected && !status->command_on && !status->output_on && !status->transfer_detected;
          torch_received_=std::chrono::steady_clock::now()-std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.0,age)));}
        if(busy_ && !taskValid()) move_->stop();
      });
    state_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      if(busy_ && !taskValid()) move_->stop();
    });
    server_ = rclcpp_action::create_server<Action>(this, "inspect_cut",
      [this](const auto&, std::shared_ptr<const Action::Goal> goal) {
        const auto& p = goal->estimate.pose;
        const Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        if (goal->cut_id.empty() || goal->estimate.header.frame_id != frame_ ||
            !hb_perception::ProfileLibrary::find(goal->estimate.profile_name) ||
            !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z) ||
            !q.coeffs().allFinite() || std::abs(q.norm()-1.0)>1e-3 ||
            !std::isfinite(goal->sample_spacing) || goal->sample_spacing<0 ||
            !std::isfinite(goal->camera_standoff) || goal->camera_standoff<0 ||
            goal->samples_per_viewpoint>100)
          return rclcpp_action::GoalResponse::REJECT;
        const std::set<std::string> scope(goal->required_segments.begin(),goal->required_segments.end());
        if(scope.empty() || scope.size()!=goal->required_segments.size() || scope.count(""))
          return rclcpp_action::GoalResponse::REJECT;
        std::lock_guard<std::mutex> lock(state_mutex_);
        if(std::chrono::steady_clock::now()-received_>=std::chrono::milliseconds(500) ||
           (!preview_ && (!torch_off_ || std::chrono::steady_clock::now()-torch_received_>=std::chrono::milliseconds(500))) ||
           !hb_robot::recovery::geometryPermitted(state_) || !stability_->stable() ||
           goal->geometry_version!=state_.geometry_version || goal->pose_version!=state_.pose_version ||
           goal->execution_version!=state_.execution_version)
          return rclcpp_action::GoalResponse::REJECT;
        bool idle = false;
        if (!busy_.compare_exchange_strong(idle, true)) return rclcpp_action::GoalResponse::REJECT;
        expected_=state_;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto&) { move_->stop(); return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& handle) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, handle] { run(handle); busy_ = false; });
      });
  }
 private:
  using Clock = std::chrono::steady_clock;
  std::size_t boundedCount(const std::string& name, int fallback, int maximum) {
    const int value = declare_parameter<int>(name, fallback);
    if (value < 1 || value > maximum) throw std::invalid_argument(name + " out of range");
    return static_cast<std::size_t>(value);
  }
  double positive(const std::string& name, double fallback) {
    const double v = declare_parameter<double>(name, fallback);
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument(name + " must be positive");
    return v;
  }
  std::optional<hb_perception::Observation> acquireFresh(
      hb_perception::CroppedRGBDAcquisition& capture, const rclcpp::Time& after,
      Clock::time_point deadline, const std::function<bool()>& canceled) {
    while (!canceled() && Clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now());
      if (remaining.count() <= 0) break;
      auto ob = capture.acquireAfter(after, std::min(remaining, std::chrono::milliseconds(500)));
      if (ob) return ob;
    }
    capture.logCaptureFailure(after);
    return std::nullopt;
  }
  bool taskValid() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::chrono::steady_clock::now()-received_<std::chrono::milliseconds(500) &&
      (preview_ || (torch_off_ && std::chrono::steady_clock::now()-torch_received_<std::chrono::milliseconds(500))) &&
      hb_robot::recovery::geometryPermitted(state_) &&
      (!busy_ || (state_.geometry_version==expected_.geometry_version &&
        state_.pose_version==expected_.pose_version && state_.scene_version==expected_.scene_version &&
        state_.execution_version==expected_.execution_version));
  }
  void run(const std::shared_ptr<Handle>& handle) {
    auto result = std::make_shared<Action::Result>();
    uint8_t exception_code = Action::Result::PLANNING_FAILED;
    hb_perception::PointCloud cloud;
    auto finish = [&](uint8_t code, const std::string& message) {
      if (handle->is_canceling()) code=Action::Result::CANCELLED;
      result->success = code == Action::Result::SUCCESS;
      result->result_code = code;
      result->interpretation = Action::Result::INCONCLUSIVE;
      result->cut_complete = false;
      result->confidence = 0;
      result->message = message;
      if (result->success) RCLCPP_INFO(get_logger(), "[KERF RESULT] %s", message.c_str());
      else RCLCPP_ERROR(get_logger(), "[KERF RESULT] code=%u views=%u/%u points=%zu: %s",
        static_cast<unsigned>(code), result->viewpoints_captured, result->viewpoints_requested,
        cloud.size(), message.c_str());
      const auto goal=handle->get_goal();
      result->evidence.header.stamp=now();
      result->evidence.header.frame_id="workpiece";
      result->evidence.cut_id=goal->cut_id;
      result->evidence.geometry_version=goal->geometry_version;
      result->evidence.pose_version=goal->pose_version;
      result->evidence.execution_version=goal->execution_version;
      result->evidence.registration_valid=result->success;
      if(!infer_evidence_) result->evidence.unknown_fraction=1.0;
      pcl::toROSMsg(cloud, result->kerf_cloud);
      result->kerf_cloud.header.frame_id = frame_;
      result->kerf_cloud.header.stamp = now();
      cloud_pub_->publish(result->kerf_cloud);
      if (handle->is_canceling()) { result->success=false; result->result_code=Action::Result::CANCELLED; handle->canceled(result); }
      else if (result->success) handle->succeed(result);
      else handle->abort(result);
    };
    auto canceled = [&] { return handle->is_canceling() || !rclcpp::ok() || !taskValid(); };
    try {
      const auto goal = handle->get_goal();
      auto profile = *hb_perception::ProfileLibrary::find(goal->estimate.profile_name);
      result->evidence=initializeEvidence(profile,goal->required_segments,ray_config_);
      // Keep the full boundary for ray intersections, but route only the requested features.
      profile.cut_features.erase(std::remove_if(profile.cut_features.begin(),profile.cut_features.end(),
        [&](const auto& f){return std::find(goal->required_segments.begin(),goal->required_segments.end(),f.name)==goal->required_segments.end();}),profile.cut_features.end());
      Eigen::Isometry3d world_from_cut;
      tf2::fromMsg(goal->estimate.pose, world_from_cut);
      const auto cut_from_world = world_from_cut.inverse();
      const auto stations = motion::inspectionStations(profile, world_from_cut,
        goal->sample_spacing>0 ? goal->sample_spacing : spacing_,
        goal->camera_standoff>0 ? goal->camera_standoff : standoff_);
      result->viewpoints_requested = stations.size();
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = frame_; poses.header.stamp = now();
      for (const auto& station : stations) poses.poses.push_back(tf2::toMsg(station.camera_poses.front()));
      poses_pub_->publish(poses);
      // Clear latched evidence from any previous cut before doing work.
      sensor_msgs::msg::PointCloud2 empty; pcl::toROSMsg(cloud, empty);
      empty.header = poses.header; cloud_pub_->publish(empty);
      if (stations.empty()) { finish(Action::Result::PLANNING_FAILED, "No material features"); return; }
      if (preview_) { finish(Action::Result::PLANNING_FAILED, "Preview only: viewpoints published; no acquisition performed"); return; }
      auto start = move_->getCurrentState(5.0);
      if (!start) { finish(Action::Result::PLANNING_FAILED, "No current robot state"); return; }

      // Use the profile-inspection receiver once for the entire action. Its own
      // executor handles RGB-D/TF while this worker plans or executes motion.
      hb_perception::CroppedRGBDAcquisition capture(this, tf_.get(), frame_,
        hb_perception::CroppedRGBDAcquisition::Mode::CaptureWindow, rgbd_topic_);
      const auto capture_duration = std::chrono::milliseconds(static_cast<int>(timeout_*1000));
      auto calibration = acquireFresh(capture, now(), Clock::now()+capture_duration, canceled);
      if (!calibration) {
        finish(canceled()?Action::Result::CANCELLED:Action::Result::ACQUISITION_FAILED,
          "No fresh RGB-D calibration/TF on " + rgbd_topic_ + " before planning"); return;
      }
      const std::string optical_frame = calibration->rgb_camera_info->header.frame_id;
      if (optical_frame.empty() || calibration->camera_pose.child_frame_id!=optical_frame ||
          calibration->camera_pose.header.frame_id!=frame_) {
        finish(Action::Result::REGISTRATION_FAILED, "RGB-D calibration and optical TF frames disagree"); return;
      }
      // A body link (+X forward) and its RGB optical child (+Z forward) are not
      // interchangeable. Use the driver's calibrated rotation AND translation.
      exception_code=Action::Result::REGISTRATION_FAILED;
      const Eigen::Isometry3d link_from_optical = tf2::transformToEigen(
        tf_->lookupTransform(camera_, optical_frame, calibration->stamp, rclcpp::Duration::from_seconds(1.0)));
      motion::ExplorationRequest view_request = view_request_;
      view_request.roll_axis = Eigen::Vector3d::UnitZ();
      view_request.link_from_view = link_from_optical;
      const auto& info = *calibration->rgb_camera_info;
      const auto seesTarget = [&](const Eigen::Isometry3d& optical_pose, const Eigen::Vector3d& target) {
        const Eigen::Vector3d p = optical_pose.inverse()*target;
        if (!p.allFinite() || p.z()<=0.02 || p.z()>range_) return false;
        const double u=info.k[0]*p.x()/p.z()+info.k[2];
        const double v=info.k[4]*p.y()/p.z()+info.k[5];
        return std::isfinite(u) && std::isfinite(v) && u>=2 && v>=2 &&
          u+2<info.width && v+2<info.height;
      };
      RCLCPP_INFO(get_logger(),
        "[KERF CAMERA] ik_link=%s optical_frame=%s rgbd=%s position_tol=%.3f roll_tol=%.3f samples=%zu solutions=%zu",
        camera_.c_str(), optical_frame.c_str(), rgbd_topic_.c_str(), view_request.pos_tol,
        view_request.roll_tol, view_request.num_candidates, max_view_solutions_);
      exception_code=Action::Result::PLANNING_FAILED;
      if (!scene_monitor_->requestPlanningSceneState("/get_planning_scene")) {
        finish(Action::Result::PLANNING_FAILED, "Could not synchronize the MoveIt planning scene"); return;
      }
      planning_scene::PlanningScenePtr scene;
      {
        planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
        scene=locked->diff(); scene->decoupleParent();
      }
      const auto planning_deadline = Clock::now() +
        std::chrono::milliseconds(static_cast<int>(planning_timeout_*1000));
      auto planning_stopped = [&] { return canceled() || Clock::now()>=planning_deadline; };
      motion::ExplorationPlanner explorer(move_->getRobotModel(), group_, camera_);

      // Reuse the cut planner's layered candidate graph and lazy edge validation.
      // Each layer is a REQUIRED observation station, each node an exploration solution.
      motion::CutCandidateGraph graph;
      std::vector<moveit::core::RobotStatePtr> states{start};
      std::vector<std::size_t> station_ids{0};
      const auto root = graph.addNode(0, 0);
      std::vector<motion::NodeId> previous{root};
      const auto* jmg = start->getJointModelGroup(group_);
      if (!jmg) throw std::runtime_error("Inspection joint group is absent");
      const auto& home_names=jmg->getDefaultStateNames();
      if(std::find(home_names.begin(),home_names.end(),"inspection_home")==home_names.end()) {
        finish(Action::Result::PLANNING_FAILED,"SRDF inspection_home is missing");return;
      }
      auto home=std::make_shared<moveit::core::RobotState>(*start);
      home->setToDefaultValues(jmg,"inspection_home");home->update();
      for (std::size_t i=0; i<stations.size(); ++i) {
        std::vector<motion::NodeId> layer;
        std::vector<moveit::core::RobotState> seeds;
        const auto add_seed = [&](const moveit::core::RobotState& candidate) {
          if (std::none_of(seeds.begin(),seeds.end(),[&](const auto& old) {
                return old.distance(candidate,jmg)<0.02;
              })) seeds.push_back(candidate);
        };
        for (const auto from : previous) add_seed(*states[from]);
        add_seed(*start); add_seed(*home);
        // A second deterministic branch even when start == inspection_home.
        auto defaults=*start; defaults.setToDefaultValues();
        std::vector<double> default_joints; defaults.copyJointGroupPositions(jmg,default_joints);
        auto neutral=*start; neutral.setJointGroupPositions(jmg,default_joints); neutral.update(); add_seed(neutral);
        const auto sampled=explorer.sampleViews(seeds, stations[i].camera_poses.front(), view_request,
          scene, max_view_solutions_, [&](const auto& pose) { return seesTarget(pose,stations[i].target); },
          planning_stopped);
        std::ostringstream diagnostics;
        diagnostics << "feature=" << stations[i].feature << " station=" << i+1 << '/' << stations.size()
          << " target=[" << stations[i].target.transpose() << "] seeds=" << seeds.size()
          << " candidates=" << sampled.candidates << " ik_attempts=" << sampled.ik_attempts
          << " ik_or_bounds_failed=" << sampled.ik_failed << " collision_rejected=" << sampled.collision_rejected
          << " framing_rejected=" << sampled.view_rejected << " duplicates=" << sampled.duplicates
          << " accepted=" << sampled.views.size();
        RCLCPP_INFO(get_logger(),"[KERF IK] %s",diagnostics.str().c_str());
        if (planning_stopped()) {
          finish(canceled()?Action::Result::CANCELLED:Action::Result::PLANNING_FAILED,
            "Inspection planning stopped (cancel, supervision or planning deadline): " + diagnostics.str()); return;
        }
        if (sampled.views.empty()) {
          finish(Action::Result::PLANNING_FAILED, "No usable inspection view: " + diagnostics.str()); return;
        }
        for (const auto& solution : sampled.views) {
          auto state = std::make_shared<moveit::core::RobotState>(solution.robot_state);
          const auto id = graph.addNode(i+1, 0);
          states.push_back(state); station_ids.push_back(i); layer.push_back(id);
          for (const auto from : previous) graph.addEdge(from, id, states[from]->distance(*state,jmg));
        }
        previous = std::move(layer);
      }
      // Independent cutting starts at inspection_home. Include the return leg
      // in the checked route before moving, under the same task supervision.
      const auto home_id=graph.addNode(stations.size()+1,0);
      states.push_back(home);station_ids.push_back(stations.size());
      for(const auto from:previous) graph.addEdge(from,home_id,states[from]->distance(*home,jmg));
      previous={home_id};
      const auto sink = graph.addNode(stations.size()+2, 0);
      for (const auto from : previous) {
        auto id=graph.addEdge(from, sink, 0); graph.edge(id).status=motion::TransitStatus::Valid;
      }
      std::map<motion::EdgeId, Plan> plans;
      motion::GraphRoute route;
      auto transit_options=transit_options_;
      transit_options.scene=scene;
      home->copyJointGroupPositions(jmg,transit_options.home_joints);
      bool route_complete=false;
      while (!planning_stopped()) {
        route = graph.cheapestRoute(root, sink);
        if (!route.found()) { finish(Action::Result::PLANNING_FAILED, "No collision-checked route through every station"); return; }
        bool complete = true;
        for (const auto edge_id : route.edges) {
          auto& edge = graph.edge(edge_id);
          if (edge.status == motion::TransitStatus::Valid) continue;
          complete = false;
          if (planning_stopped()) break;

          auto plan = motion::planTransit(
              *move_, get_logger(),
              *states[edge.from], *states[edge.to],
              transit_options);

          if (planning_stopped()) break;

          if (!plan || !scene->isPathValid(plan->start_state_,plan->trajectory_,group_)) {
            edge.status=motion::TransitStatus::Invalid;
            RCLCPP_WARN(get_logger(),"[KERF TRANSIT] edge=%zu from=%zu to=%zu failed; trying another candidate route",
              edge_id,edge.from,edge.to);
          } else {
            edge.status=motion::TransitStatus::Valid;
            const auto& points=plan->trajectory_.joint_trajectory.points;
            double cost=0;
            for (std::size_t k=1;k<points.size();++k)
              for (std::size_t j=0;j<points[k].positions.size();++j)
                cost+=std::abs(points[k].positions[j]-points[k-1].positions[j]);
            edge.actual_cost=cost; plans.emplace(edge_id,std::move(*plan));
          }
          break;
        }
        if (complete) {route_complete=true;break;}
      }
      if (!route_complete || planning_stopped()) {
        finish(canceled()?Action::Result::CANCELLED:Action::Result::PLANNING_FAILED,
          "Inspection route incomplete: canceled, supervision lost or planning deadline reached");return;
      }
      poses.poses.clear(); poses.header.stamp=now();
      for (const auto id : route.nodes) {
        if(id!=root && id!=sink && station_ids[id]<stations.size())
          poses.poses.push_back(tf2::toMsg(states[id]->getGlobalLinkTransform(camera_)*link_from_optical));
      }
      poses_pub_->publish(poses);
      RCLCPP_INFO(get_logger(),"[KERF ROUTE] stations=%zu validated_transits=%zu; return_home included",
        stations.size(),route.edges.size()-1);
      exception_code=Action::Result::MOTION_FAILED;
      const auto samples=goal->samples_per_viewpoint ? goal->samples_per_viewpoint : 3;
      for (const auto edge_id : route.edges) {
        const auto& edge=graph.edge(edge_id);
        if (edge.to==sink) continue;
        if (canceled()) { finish(Action::Result::CANCELLED, "Canceled"); return; }
        // Do not execute a stale planned start after external robot movement.
        auto current=move_->getCurrentState(2.0);
        if (!current || current->distance(*states[edge.from], jmg)>0.05) {
          finish(Action::Result::MOTION_FAILED, "Robot moved since route planning"); return;
        }
        {
          planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
          if (!locked->isPathValid(plans.at(edge_id).start_state_,plans.at(edge_id).trajectory_,group_)) {
            finish(Action::Result::MOTION_FAILED, "Inspection trajectory invalid in latest planning scene"); return;
          }
        }
        if (canceled()) { finish(Action::Result::CANCELLED, "Canceled before inspection motion"); return; }
        exception_code=Action::Result::MOTION_FAILED;
        if (move_->execute(plans.at(edge_id)) != moveit::core::MoveItErrorCode::SUCCESS) {
          finish(Action::Result::MOTION_FAILED, "Inspection motion failed"); return;
        }
        if(station_ids[edge.to]==stations.size()) continue;
        
        if (!stability_->waitForStable(
              std::chrono::milliseconds(static_cast<int>(stability_timeout_ * 1000)), canceled)) {
          if (canceled()) {
            finish(Action::Result::CANCELLED, "Canceled while waiting for stability"); return;
          }
          finish(Action::Result::STABILITY_TIMEOUT, "Platform did not settle: " + stability_->reason());
          return;
        }

        auto stamp=now(); // Strictly post-motion and post-stability observations only.
        const auto index=station_ids[edge.to];
        exception_code=Action::Result::ACQUISITION_FAILED;
        const auto station_points_before=cloud.size();
        for (uint32_t sample=0;sample<samples;++sample) {
          if (canceled()) { finish(Action::Result::CANCELLED, "Canceled"); return; }
          const auto ob=acquireFresh(capture,stamp,Clock::now()+capture_duration,canceled);
          
         if (!ob || !stability_->stable()) {
          
            finish(canceled()?Action::Result::CANCELLED:Action::Result::ACQUISITION_FAILED,
              "Missing fresh RGB-D or stability lost at " + stations[index].feature +
              " station=" + std::to_string(index+1) + " sample=" + std::to_string(sample+1)); return;
          }
          if (rclcpp::Time(ob->depth_to_rgb->header.stamp)<=stamp ||
              std::abs((rclcpp::Time(ob->depth_to_rgb->header.stamp)-ob->stamp).seconds())>0.03 ||
              ob->camera_pose.header.frame_id!=frame_ || ob->camera_pose.child_frame_id!=optical_frame ||
              ob->rgb_camera_info->header.frame_id!=optical_frame ||
              ob->rgb_camera_info->k!=info.k || ob->rgb_camera_info->width!=info.width ||
              ob->rgb_camera_info->height!=info.height) {
            finish(Action::Result::ACQUISITION_FAILED, "RGB-D timing or optical frame mismatch"); return;
          }
          stamp=ob->stamp;
          const auto camera_pose=tf2::transformToEigen(ob->camera_pose);
          const Eigen::Isometry3d expected=states[edge.to]->getGlobalLinkTransform(camera_)*link_from_optical;
          const double position_error=(camera_pose.translation()-expected.translation()).norm();
          const double angle_error=Eigen::AngleAxisd(expected.linear().transpose()*camera_pose.linear()).angle();
          if (position_error>0.02 || angle_error>0.05 || !seesTarget(camera_pose,stations[index].target)) {
            finish(Action::Result::REGISTRATION_FAILED, "Measured optical view differs from plan or loses target: position_m=" +
              std::to_string(position_error) + " angle_rad=" + std::to_string(angle_error)); return;
          }
          auto raw=kerfObservationCloud(*ob, range_);

          if (!raw || raw->empty()) { finish(Action::Result::ACQUISITION_FAILED, "Empty valid depth cloud"); return; }
          for (auto point : raw->points) {
            const Eigen::Vector3d world=camera_pose*point.getVector3fMap().cast<double>();
            if (!motion::inKerfBand(cut_from_world*world,profile,band_,margin_)) continue;
            if (cloud.size()>=5000000) {
              finish(Action::Result::ACQUISITION_FAILED, "Five million point budget exceeded; reduce samples or band width"); return;
            }
            point.x=world.x(); point.y=world.y(); point.z=world.z(); cloud.push_back(point);
          }
          if(infer_evidence_)
            accumulateKerfEvidence(result->evidence,profile,*ob,world_from_cut,ray_config_,
              [&](const std::string& feature,double s,RayEvidence kind,double measured,double entry,double exit,bool within) {
                hb_robot_interfaces::msg::KerfRaySample ray;
                ray.header.frame_id=frame_;ray.header.stamp=ob->stamp;ray.cut_id=goal->cut_id;
                ray.feature_id=feature;ray.execution_version=goal->execution_version;ray.s=s;
                ray.classification=static_cast<uint8_t>(kind);ray.measured_distance=measured;
                ray.material_entry=entry;ray.material_exit=exit;ray.uncertainty=ray_config_.depth_uncertainty;
                ray.within_kerf=within;rays_pub_->publish(ray);
              });
          hb_robot_interfaces::msg::KerfObservation record;
          record.header.frame_id=frame_; record.header.stamp=ob->stamp;
          record.geometry_version=goal->geometry_version;
          record.pose_version=goal->pose_version;
          record.execution_version=goal->execution_version;
          record.cut_id=goal->cut_id; record.feature_name=stations[index].feature;
          record.viewpoint_index=index; record.cut_pose=goal->estimate.pose;
          record.camera_pose=ob->camera_pose; record.rgb=*ob->rgb;
          record.depth=*ob->depth_to_rgb; record.camera_info=*ob->rgb_camera_info;
          observation_pub_->publish(record);
          auto feedback=std::make_shared<Action::Feedback>();
          feedback->current_viewpoint=index; feedback->total_viewpoints=stations.size();
          feedback->feature_name=stations[index].feature; feedback->current_pose=tf2::toMsg(camera_pose);
          feedback->samples_acquired=sample+1; handle->publish_feedback(feedback);
        }
        ++result->viewpoints_captured;
        RCLCPP_INFO(get_logger(),"[KERF CAPTURE] feature=%s station=%zu/%zu samples=%u added_points=%zu total_points=%zu",
          stations[index].feature.c_str(),index+1,stations.size(),samples,
          cloud.size()-station_points_before,cloud.size());
        sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(cloud,msg);
        msg.header.frame_id=frame_; msg.header.stamp=stamp; cloud_pub_->publish(msg);
      }
      if (canceled()) { finish(Action::Result::CANCELLED, "Canceled"); return; }
      if (cloud.empty()) { 
        finish(Action::Result::ACQUISITION_FAILED, "No measured points in kerf band; missing depth is not free space"); 
        return; 
      }
      finish(Action::Result::SUCCESS, "Post-cut acquisition complete; separation and bridging have not been evaluated");
    } catch (const std::exception& e) { 
      finish(exception_code,e.what());
    }
  }
  std::mutex state_mutex_;
  hb_robot::recovery::State state_;
  hb_robot::recovery::State expected_;
  bool torch_off_=false;
  std::chrono::steady_clock::time_point torch_received_{};
  rclcpp::Subscription<hb_robot_interfaces::msg::TorchStatus>::SharedPtr torch_sub_;
  rclcpp::Publisher<hb_robot_interfaces::msg::KerfRaySample>::SharedPtr rays_pub_;
  rclcpp::Service<hb_robot_interfaces::srv::EvaluateKerf>::SharedPtr evaluate_service_;
  rclcpp::Service<hb_robot_interfaces::srv::GetCutDefinition>::SharedPtr definition_service_;
  rclcpp::Service<hb_robot_interfaces::srv::ReplayKerf>::SharedPtr replay_service_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<hb_robot::recovery::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::string group_,camera_,frame_,rgbd_topic_;
  double spacing_,standoff_,band_,margin_,range_,timeout_,stability_timeout_,planning_timeout_;
  std::size_t max_view_solutions_;
  motion::ExplorationRequest view_request_;
  motion::TransitPlannerOptions transit_options_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  bool preview_,infer_evidence_;
  KerfRayConfig ray_config_;
  std::atomic<bool> busy_{false};
  std::thread worker_;
  std::unique_ptr<MoveGroup> move_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<hb_robot_skills::PlatformSupervisionClient>
    stability_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<hb_robot_interfaces::msg::KerfObservation>::SharedPtr observation_pub_;
};
} // namespace hb_robot_skills
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<hb_robot_skills::InspectCutServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  try { node->initialize(); spin.join(); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"%s",e.what());
    rclcpp::shutdown(); if(spin.joinable()) spin.join(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
