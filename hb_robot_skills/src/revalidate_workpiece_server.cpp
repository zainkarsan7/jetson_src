#include <hb_robot_interfaces/action/revalidate_workpiece.hpp>
#include <hb_robot_interfaces/msg/workpiece_reference.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_perception/rgbd_acquisition.hpp>
#include <hb_robot_perception/imu_stability_monitor.hpp>
#include <hb_robot_perception/observation_registrar.hpp>
#include <hb_robot_skills/kerf_observation.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <atomic>
#include <thread>

class RevalidateServer : public rclcpp::Node {
  using Action=hb_robot_interfaces::action::RevalidateWorkpiece;
  using Handle=rclcpp_action::ServerGoalHandle<Action>;
  using Reference=hb_robot_interfaces::msg::WorkpieceReference;
 public:
  RevalidateServer():Node("revalidate_workpiece_server") {
    frame_=declare_parameter<std::string>("scene_base_frame","world");
    range_=declare_parameter<double>("depth_range",1.5);
    config_.max_correspondence_m=0.025;config_.max_translation_m=0.05;
    config_.max_rotation_rad=0.10;config_.max_fitness_m2=0.000025;
    config_.min_points=100;
    tf_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*tf_);
    acquisition_=std::make_unique<hb_perception::RGBDAcquisition>(this,tf_.get(),frame_);
    imu_=std::make_unique<hb_perception::ImuStabilityMonitor>(*this,"/k4a/imu");
    ref_sub_=create_subscription<Reference>("/workpiece_reference",rclcpp::QoS(1).transient_local(),
      [this](Reference::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);reference_=*m;});
    state_sub_=create_subscription<hb_robot::recovery::State>("/task_state",rclcpp::QoS(1).transient_local(),
      [this](hb_robot::recovery::State::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);state_=*m;received_=std::chrono::steady_clock::now();});
    ref_pub_=create_publisher<Reference>("/workpiece_reference",rclcpp::QoS(1).transient_local());
    estimate_pub_=create_publisher<hb_robot_interfaces::msg::ProfileEstimate>(
      "/profile_inspection/estimate",rclcpp::QoS(1).transient_local());
    server_=rclcpp_action::create_server<Action>(this,"revalidate_workpiece",
      [this](const auto&,auto){bool idle=false;return busy_.compare_exchange_strong(idle,true)?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;},
      [](auto){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h){if(worker_.joinable()) worker_.join();worker_=std::thread([this,h]{run(h);busy_=false;});});
  }
  ~RevalidateServer() override {if(worker_.joinable()) worker_.join();}
 private:
  void run(std::shared_ptr<Handle> h) {
    auto result=std::make_shared<Action::Result>();
    auto canceled=[&]{return h->is_canceling() || !rclcpp::ok();};
    auto finish=[&](std::string reason){
      result->message=reason;
      if(h->is_canceling()) {result->localized=false;h->canceled(result);}
      else if(result->localized) h->succeed(result);else h->abort(result);
    };
    try {
      Reference reference;
      {std::lock_guard<std::mutex> lock(mutex_);reference=reference_;}
      if(reference.cloud.data.empty() || reference.cloud.header.frame_id!=frame_ ||
         reference.estimate.header.frame_id!=frame_) {finish("No matching stored reference");return;}
      // Wait briefly for LIVE platform supervision before taking a new image.
      // Settling alone never confirms a pose. The image and registration below do.
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
      while(!canceled()) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if(state_.pose_version!=h->get_goal()->expected_pose_version ||
             state_.scene_id!=std::to_string(rclcpp::Time(reference.estimate.header.stamp).nanoseconds()) ||
             state_.motion_fault_active || state_.process_fault_active) {
            finish("Reference superseded or fault active before localization");return;
          }
          if(state_.supervision_valid && state_.imu_stable &&
             std::chrono::steady_clock::now()-received_<std::chrono::milliseconds(500)) break;
        }
        if(std::chrono::steady_clock::now()>=deadline) {finish("Platform supervision unavailable");return;}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if(canceled()) {finish("Localization canceled");return;}
      if(!imu_->waitForStable(std::chrono::seconds(10),canceled)) {finish("No stable observation window");return;}
      const auto observation=acquisition_->acquireAfter(now(),std::chrono::seconds(3));
      if(!observation || canceled()) {finish("Fresh acquisition failed");return;}
      auto raw=hb_robot_skills::kerfObservationCloud(*observation,range_);
      hb_perception::PointCloud world,reference_cloud;
      pcl::transformPointCloud(*raw,world,tf2::transformToEigen(observation->camera_pose).matrix().cast<float>());
      pcl::fromROSMsg(reference.cloud,reference_cloud);
      Eigen::Vector4f lo,hi;pcl::getMinMax3D(reference_cloud,lo,hi);
      auto target=std::make_shared<hb_perception::PointCloud>();
      for(const auto& p:world) if((p.getVector3fMap().array()>=lo.head<3>().array()-0.06f).all() &&
         (p.getVector3fMap().array()<=hi.head<3>().array()+0.06f).all()) target->push_back(p);
      auto filter=[](hb_perception::PointCloud::Ptr cloud){
        hb_perception::PointCloud out;pcl::VoxelGrid<hb_perception::PointT> voxel;
        voxel.setLeafSize(0.003f,0.003f,0.003f);voxel.setInputCloud(cloud);voxel.filter(out);return out;};
      auto source=filter(reference_cloud.makeShared());auto current=filter(target);
      hb_perception::ObservationRegistrar registrar(config_);
      const auto registration=registrar.registerCloud(source,current);
      result->registration_fitness=registration.fitness_m2;
      result->translation_delta=registration.translation_m;result->rotation_delta=registration.rotation_rad;
      if(!registration.accepted) {finish(registration.reason);return;}
      Eigen::Isometry3d original;tf2::fromMsg(reference.estimate.pose,original);
      // Low residual on an extruded section does not establish longitudinal pose.
      // Restart ICP on either side of the axial solution. Equally good but displaced
      // solutions reject localization instead of selecting an arbitrary beam position.
      for(double shift:{-0.015,0.015}) {
        Eigen::Matrix4f offset=Eigen::Matrix4f::Identity();
        offset.block<3,1>(0,3)=(original.linear().col(2)*shift).cast<float>();
        hb_perception::PointCloud shifted;pcl::transformPointCloud(source,shifted,offset);
        const auto alternative=registrar.registerCloud(shifted,current);
        const Eigen::Matrix4f combined=alternative.correction*offset;
        if(alternative.accepted && alternative.fitness_m2<=registration.fitness_m2*1.2+1e-7 &&
           (combined.block<3,1>(0,3)-registration.correction.block<3,1>(0,3)).norm()>0.005) {
          result->poorly_constrained=true;finish("Axially ambiguous registration; acquire distinctive reference geometry");return;
        }
      }
      const auto m=imu_->metrics();
      {std::lock_guard<std::mutex> lock(mutex_);
        if(state_.pose_version!=h->get_goal()->expected_pose_version ||
           std::chrono::steady_clock::now()-received_>std::chrono::milliseconds(500) ||
           !state_.supervision_valid || !state_.imu_stable ||
           !m.valid || !m.stable || canceled()) {finish("State changed during localization");return;}}
      Eigen::Isometry3d delta=Eigen::Isometry3d::Identity();
      delta.matrix()=registration.correction.cast<double>();
      moveit::planning_interface::PlanningSceneInterface scene;
      auto objects=scene.getObjects({"wk_workpiece_collision","wk_section_collision"});
      if(!objects.count("wk_workpiece_collision")) {finish("Stored workpiece collision missing");return;}
      // Bound displacement over the entire planned extrusion, including rotation
      // about a distant origin. Use the existing 0.1 mm mesh consistency scale.
      // Unknown geometry always takes the conservative update-and-replan path.
      const auto& beam=objects.at("wk_workpiece_collision");
      bool unchanged=beam.meshes.size()==1 && beam.mesh_poses.size()==1 &&
                     !beam.meshes.front().vertices.empty() && beam.header.frame_id==frame_;
      if(unchanged) {
        Eigen::Isometry3d object_pose,mesh_pose;
        tf2::fromMsg(beam.pose,object_pose);tf2::fromMsg(beam.mesh_poses.front(),mesh_pose);
        const Eigen::Isometry3d world_from_mesh=object_pose*mesh_pose;
        for(const auto& vertex:beam.meshes.front().vertices) {
          const Eigen::Vector3d point=world_from_mesh*Eigen::Vector3d(vertex.x,vertex.y,vertex.z);
          if(!((delta*point-point).norm()<=1e-4)) {unchanged=false;break;}
        }
      }
      std::vector<moveit_msgs::msg::CollisionObject> updates;
      for(auto& pair:objects) {
        auto object=pair.second;
        if(object.header.frame_id!=frame_) {finish("Collision frame mismatch");return;}
        Eigen::Isometry3d pose;tf2::fromMsg(object.pose,pose);object.pose=tf2::toMsg(delta*pose);
        object.operation=moveit_msgs::msg::CollisionObject::ADD;updates.push_back(object);
      }
      if(!unchanged && !scene.applyCollisionObjects(updates)) {finish("Collision scene update failed");return;}
      if(canceled() || !imu_->metrics().stable) {finish("Interrupted before pose commit");return;}
      if(!unchanged) reference.estimate.pose=tf2::toMsg(delta*original);
      reference.estimate.header.stamp=now();
      // Keep the baseline cloud as well as its pose when confirming; repeated
      // sub-tolerance registrations must never accumulate into reference drift.
      if(!unchanged) {
        pcl::transformPointCloud(reference_cloud,world,registration.correction);
        pcl::toROSMsg(world,reference.cloud);
      }
      reference.cloud.header=reference.estimate.header;
      reference.cloud.header.stamp=observation->stamp;
      reference.relocalized=true;reference.expected_pose_version=h->get_goal()->expected_pose_version;
      result->estimate=reference.estimate;result->localized=true;
      estimate_pub_->publish(reference.estimate);ref_pub_->publish(reference);
      finish("Fresh localization published for task-state confirmation");
    } catch(const std::exception& e){finish(e.what());}
  }
  std::string frame_;double range_;
  hb_perception::RegistrationConfig config_;
  std::mutex mutex_;Reference reference_;hb_robot::recovery::State state_;
  std::chrono::steady_clock::time_point received_{};
  std::atomic<bool> busy_{false};std::thread worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_;std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<hb_perception::RGBDAcquisition> acquisition_;
  std::unique_ptr<hb_perception::ImuStabilityMonitor> imu_;
  rclcpp::Subscription<Reference>::SharedPtr ref_sub_;
  rclcpp::Subscription<hb_robot::recovery::State>::SharedPtr state_sub_;
  rclcpp::Publisher<Reference>::SharedPtr ref_pub_;
  rclcpp::Publisher<hb_robot_interfaces::msg::ProfileEstimate>::SharedPtr estimate_pub_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<RevalidateServer>();
  rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);executor.spin();rclcpp::shutdown();
}
