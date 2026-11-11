#include "hb_robot_skills/motion/cut_inspection.hpp"
#include "hb_robot_skills/kerf_observation.hpp"
#include "hb_robot_skills/kerf_evidence.hpp"
#include "hb_robot_skills/motion/cut_candidate_graph.hpp"
#include "hb_robot_interfaces/action/inspect_cut.hpp"
#include "hb_robot_interfaces/msg/kerf_observation.hpp"
#include "hb_robot_perception/profile_library.hpp"
#include "hb_robot_perception/rgbd_acquisition.hpp"
#include "hb_robot_perception/perception_types.hpp"
#include "hb_robot_skills/platform_supervision_client.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
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
    // Must be an optical frame in the robot model (or fixed child of the camera).
    camera_ = declare_parameter<std::string>("camera_link", "camera_visor");
    frame_ = declare_parameter<std::string>("scene_base_frame", "world");
    infer_evidence_=declare_parameter<bool>("enable_geometric_kerf_evidence",false);
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
      throw std::runtime_error("Planning frame mismatch or camera optical link absent from robot model");
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
     
    stability_ =
    std::make_unique<hb_robot_skills::PlatformSupervisionClient>(
        *this);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/kerf_cloud", rclcpp::QoS(1).transient_local());
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/viewpoints", rclcpp::QoS(1).transient_local());
    observation_pub_ = create_publisher<hb_robot_interfaces::msg::KerfObservation>("~/observations", 10);
    state_sub_=create_subscription<hb_robot::recovery::State>(
      "/task_state",rclcpp::QoS(1).transient_local(),
      [this](hb_robot::recovery::State::ConstSharedPtr state){
        {std::lock_guard<std::mutex> lock(state_mutex_);state_=*state;received_=std::chrono::steady_clock::now();}
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
        if(!taskValid()) return rclcpp_action::GoalResponse::REJECT;
        bool idle = false;
        if (!busy_.compare_exchange_strong(idle, true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto&) { move_->stop(); return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& handle) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, handle] { run(handle); busy_ = false; });
      });
  }
 private:
  double positive(const std::string& name, double fallback) {
    const double v = declare_parameter<double>(name, fallback);
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument(name + " must be positive");
    return v;
  }
  bool taskValid() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::chrono::steady_clock::now()-received_<std::chrono::milliseconds(500) &&
      hb_robot::recovery::geometryPermitted(state_);
  }
  void run(const std::shared_ptr<Handle>& handle) {
    auto result = std::make_shared<Action::Result>();
    hb_perception::PointCloud cloud;
    auto finish = [&](uint8_t code, const std::string& message) {
      result->success = code == Action::Result::SUCCESS;
      result->result_code = code;
      result->interpretation = Action::Result::INCONCLUSIVE;
      result->cut_complete = false;
      result->confidence = 0;
      result->message = message;
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
      const auto profile = *hb_perception::ProfileLibrary::find(goal->estimate.profile_name);
      result->evidence=initializeEvidence(profile,goal->required_segments,ray_config_);
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

      // Reuse the cut planner's layered candidate graph and lazy edge validation.
      // Each layer is a required observation station, each node a camera roll.
      motion::CutCandidateGraph graph;
      std::vector<moveit::core::RobotStatePtr> states{start};
      std::vector<std::size_t> station_ids{0};
      const auto root = graph.addNode(0, 0);
      std::vector<motion::NodeId> previous{root};
      const auto* jmg = start->getJointModelGroup(group_);
      for (std::size_t i=0; i<stations.size(); ++i) {
        std::vector<motion::NodeId> layer;
        for (const auto& pose : stations[i].camera_poses) {
          auto state = std::make_shared<moveit::core::RobotState>(*states[previous.front()]);
          if (!state->setFromIK(jmg, pose, camera_, 0.1) || !state->satisfiesBounds(jmg)) continue;
          state->update();
          const auto id = graph.addNode(i+1, 0);
          states.push_back(state); station_ids.push_back(i); layer.push_back(id);
          for (const auto from : previous) graph.addEdge(from, id, 0.0);
        }
        if (canceled()) { finish(Action::Result::CANCELLED, "Canceled during IK"); return; }
        if (layer.empty()) { finish(Action::Result::PLANNING_FAILED, "No IK at " + stations[i].feature); return; }
        previous = std::move(layer);
      }
      const auto sink = graph.addNode(stations.size()+1, 0);
      for (const auto from : previous) {
        auto id=graph.addEdge(from, sink, 0); graph.edge(id).status=motion::TransitStatus::Valid;
      }
      std::map<motion::EdgeId, Plan> plans;
      motion::GraphRoute route;
      while (!canceled()) {
        route = graph.cheapestRoute(root, sink);
        if (!route.found()) { finish(Action::Result::PLANNING_FAILED, "No collision-checked route through every station"); return; }
        bool complete = true;
        for (const auto edge_id : route.edges) {
          auto& edge = graph.edge(edge_id);
          if (edge.status == motion::TransitStatus::Valid) continue;
          complete = false;
          move_->setStartState(*states[edge.from]);
          move_->setJointValueTarget(*states[edge.to]);
          Plan plan;
          if (move_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS || plan.trajectory_.joint_trajectory.points.empty()) {
            edge.status=motion::TransitStatus::Invalid;
          } else {
            edge.status=motion::TransitStatus::Valid;
            const auto& points=plan.trajectory_.joint_trajectory.points;
            double cost=0;
            for (std::size_t k=1;k<points.size();++k)
              for (std::size_t j=0;j<points[k].positions.size();++j)
                cost+=std::abs(points[k].positions[j]-points[k-1].positions[j]);
            edge.actual_cost=cost; plans.emplace(edge_id,std::move(plan));
          }
          break;
        }
        if (complete) break;
      }
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
        if (move_->execute(plans.at(edge_id)) != moveit::core::MoveItErrorCode::SUCCESS) {
          finish(Action::Result::MOTION_FAILED, "Inspection motion failed"); return;
        }
        
        if (!stability_->waitForStable(
        std::chrono::milliseconds(
            static_cast<int>(stability_timeout_ * 1000)),
        canceled)) {
          if (canceled()) {
        finish(Action::Result::CANCELLED, "Canceled while waiting for stability");
        return;
    }

    finish(
        Action::Result::STABILITY_TIMEOUT,
        "Platform did not settle: " + stability_->reason());
    return;
}





        auto stamp=now(); // Strictly post-motion and post-stability observations only.
        const auto index=station_ids[edge.to];
        hb_perception::RGBDAcquisition capture(this, tf_.get(), frame_,
          hb_perception::RGBDAcquisition::Mode::CaptureWindow,
          get_parameter("perception_rgb_topic").as_string(),
          get_parameter("perception_depth_topic").as_string(),
          get_parameter("perception_camera_info_topic").as_string());
        for (uint32_t sample=0;sample<samples;++sample) {
          if (canceled()) { finish(Action::Result::CANCELLED, "Canceled"); return; }
          const auto ob=capture.acquireAfter(stamp, std::chrono::milliseconds(static_cast<int>(timeout_*1000)));
          
         if (!ob || !stability_->stable()) {
          
            finish(Action::Result::ACQUISITION_FAILED, "Missing fresh RGB-D or stability lost"); return;
          }
          if (rclcpp::Time(ob->depth_to_rgb->header.stamp)<=stamp ||
              std::abs((rclcpp::Time(ob->depth_to_rgb->header.stamp)-ob->stamp).seconds())>0.03 ||
              ob->camera_pose.header.frame_id!=frame_ || ob->camera_pose.child_frame_id!=camera_) {
            finish(Action::Result::ACQUISITION_FAILED, "RGB-D timing or optical frame mismatch"); return;
          }
          stamp=ob->stamp;
          const auto camera_pose=tf2::transformToEigen(ob->camera_pose);
          const auto expected=states[edge.to]->getGlobalLinkTransform(camera_);
          if ((camera_pose.translation()-expected.translation()).norm()>0.02 ||
              Eigen::AngleAxisd(expected.linear().transpose()*camera_pose.linear()).angle()>0.05) {
            finish(Action::Result::REGISTRATION_FAILED, "Measured optical pose differs from planned view"); return;
          }
          auto raw=hb_perception::observationToCloud(*ob, range_);

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
            accumulateKerfEvidence(result->evidence,profile,*ob,world_from_cut,ray_config_);
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
      finish(Action::Result::ACQUISITION_FAILED,e.what()); 
    }
  }
  std::mutex state_mutex_;
  hb_robot::recovery::State state_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<hb_robot::recovery::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::string group_,camera_,frame_;
  double spacing_,standoff_,band_,margin_,range_,timeout_,stability_timeout_;
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  try { node->initialize(); spin.join(); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"%s",e.what());
    rclcpp::shutdown(); if(spin.joinable()) spin.join(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
