#include "hb_robot_skills/scene_revision_tracker.hpp"
#include <gtest/gtest.h>
#include <hb_robot_interfaces/recovery.hpp>
#include <hb_robot_skills/kerf_evidence.hpp>
#include <limits>
using namespace hb_robot::recovery;
State validState() {
  State s;s.imu_stable=s.supervision_valid=s.workpiece_pose_valid=true;
  s.geometry_version=7;s.pose_version=3;s.execution_version=9;return s;
}
Evidence complete() {
  Evidence e;e.cut_id="cut";e.registration_valid=true;
  e.geometry_version=7;e.pose_version=3;e.execution_version=9;e.required_segments={"flange","web"};
  for(const auto& name:e.required_segments) {
    hb_robot_interfaces::msg::KerfSegmentEvidence b;
    b.interval.segment_id=name;b.interval.s0=0;b.interval.s1=1;
    b.observed_fraction=b.open_fraction=b.confidence=1;b.through_thickness_observed=true;
    e.segments.push_back(b);
  }
  return e;
}
TEST(Kerf, CompleteRequiresAllFeatures) {
  auto e=complete();EXPECT_EQ(evaluate(e,validState(),"cut").status,State::VERIFIED);
  e.segments.pop_back();EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INCONCLUSIVE);
}
TEST(Kerf, TinyBridgeOverridesGlobalScore) {
  auto e=complete();e.segments[0].interval.s1=0.999;
  auto bridge=e.segments[0];bridge.interval.s0=0.999;bridge.interval.s1=1;
  bridge.open_fraction=0;bridge.residual_fraction=1;e.segments.push_back(bridge);
  auto result=evaluate(e,validState(),"cut");
  ASSERT_EQ(result.status,State::RESIDUAL_MATERIAL);ASSERT_EQ(result.residuals.size(),1u);
  EXPECT_DOUBLE_EQ(result.residuals[0].s0,0.999);
}
TEST(Kerf, StaleEpochOrPoseIsInvalid) {
  auto e=complete();e.pose_version++;
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INVALID_OBSERVATION);
  e=complete();e.execution_version--;
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INVALID_OBSERVATION);
}
TEST(Kerf, MissingDepthIsUnknown) {
  auto e=complete();e.segments[0].open_fraction=0;e.segments[0].observed_fraction=0;
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INCONCLUSIVE);
}
TEST(Kerf, GapsOverlapNanAndWrongIdentity) {
  auto e=complete();e.segments[0].interval.s0=0.01;
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INCONCLUSIVE);
  e=complete();e.segments.push_back(e.segments[0]);
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INVALID_OBSERVATION);
  e=complete();e.segments[0].confidence=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluate(e,validState(),"cut").status,State::INVALID_OBSERVATION);
  EXPECT_EQ(evaluate(complete(),validState(),"other").status,State::INVALID_OBSERVATION);
}
TEST(State, DisturbanceAndFaultDenyExecution) {
  auto s=validState();EXPECT_TRUE(geometryPermitted(s));
  s.imu_stable=false;EXPECT_FALSE(geometryPermitted(s));
  s=validState();s.process_fault_active=true;EXPECT_FALSE(geometryPermitted(s));
  s=validState();s.supervision_valid=false;EXPECT_FALSE(geometryPermitted(s));
}
TEST(Recut, AttemptsOverlapOnlyWithinOriginalFeature) {
  Interval a;a.segment_id="web";a.s0=.2;a.s1=.4;
  auto b=a;b.s0=.3;b.s1=.6;EXPECT_TRUE(overlaps(a,b));
  b.segment_id="flange";EXPECT_FALSE(overlaps(a,b));
  b=a;b.s0=.4;b.s1=.5;EXPECT_FALSE(overlaps(a,b));
  b.s0=.7;EXPECT_FALSE(validInterval(b));
}
TEST(Rays, NoReturnAndOcclusionNeverVerify) {
  using namespace hb_robot_skills;
  EXPECT_EQ(classifyKerfRay({.3,.32},.5,.002,true),RayEvidence::OPEN);
  EXPECT_EQ(classifyKerfRay({.3,.32},.31,.002,true),RayEvidence::MATERIAL);
  EXPECT_EQ(classifyKerfRay({.3,.32},0,.002,true),RayEvidence::UNKNOWN);
  EXPECT_EQ(classifyKerfRay({.3,.32},.2,.002,true),RayEvidence::UNKNOWN);
  EXPECT_EQ(classifyKerfRay({.3,.32},.5,.002,false),RayEvidence::UNKNOWN);
  EXPECT_EQ(classifyKerfRay({.3,.32},.3,.002,true),RayEvidence::UNKNOWN);
}
TEST(Rays, PolygonThicknessIntersection) {
  hb_perception::ProfileModel model;
  for(const auto& ab:std::vector<std::pair<Eigen::Vector2f,Eigen::Vector2f>>{
      {{0,0},{1,0}},{{1,0},{1,1}},{{1,1},{0,1}},{{0,1},{0,0}}})
    model.boundary.push_back(hb_perception::LineSegment2D{ab.first,ab.second});
  const auto interval=hb_robot_skills::materialRayInterval(model,{-1,.5,0},{1,0,0});
  ASSERT_TRUE(interval);EXPECT_DOUBLE_EQ(interval->first,1);EXPECT_DOUBLE_EQ(interval->second,2);
}

TEST(SceneRevision, JointFeedbackAndRepublishedSnapshotPreservePlan) {
  hb_robot_skills::SceneRevisionTracker tracker;
  moveit_msgs::msg::PlanningScene scene;
  scene.world.collision_objects.resize(1);
  scene.world.collision_objects.front().id = "workpiece";
  scene.fixed_frame_transforms.resize(1);
  ASSERT_TRUE(tracker.changed(scene));
  scene.robot_state.joint_state.position = {0.4};
  ++scene.fixed_frame_transforms.front().header.stamp.sec;
  EXPECT_FALSE(tracker.changed(scene));
  moveit_msgs::msg::PlanningScene joints;
  joints.is_diff = true;
  joints.robot_state.is_diff = true;
  joints.robot_state.joint_state.name = {"joint"};
  EXPECT_FALSE(tracker.changed(joints));
  EXPECT_FALSE(tracker.changed(scene));
  scene.world.collision_objects.front().pose.position.x += 0.001;
  EXPECT_TRUE(tracker.changed(scene));
  scene.world.collision_objects.clear();
  EXPECT_TRUE(tracker.changed(scene));
}

TEST(SceneRevision, RelativeEditsAndChangedCollisionPolicyInvalidate) {
  hb_robot_skills::SceneRevisionTracker tracker;
  moveit_msgs::msg::PlanningScene edit;
  edit.is_diff = true;
  edit.world.collision_objects.resize(1);
  edit.world.collision_objects.front().operation = moveit_msgs::msg::CollisionObject::APPEND;
  EXPECT_TRUE(tracker.changed(edit));
  EXPECT_TRUE(tracker.changed(edit));
  edit.world.collision_objects.clear();
  edit.allowed_collision_matrix.default_entry_names = {"torch"};
  edit.allowed_collision_matrix.default_entry_values = {false};
  EXPECT_TRUE(tracker.changed(edit));
  EXPECT_FALSE(tracker.changed(edit));
  edit.allowed_collision_matrix.default_entry_values = {true};
  EXPECT_TRUE(tracker.changed(edit));
}

TEST(SceneRevision, ReplacingAttachedBodyStateInvalidates) {
  hb_robot_skills::SceneRevisionTracker tracker;
  moveit_msgs::msg::PlanningScene scene;
  scene.robot_state.attached_collision_objects.resize(1);
  ASSERT_TRUE(tracker.changed(scene));
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = false;
  diff.robot_state.joint_state.name = {"joint"};
  EXPECT_TRUE(tracker.changed(diff));
  EXPECT_FALSE(tracker.changed(diff));
}
