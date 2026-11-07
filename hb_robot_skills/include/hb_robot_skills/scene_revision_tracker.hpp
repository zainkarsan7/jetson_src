#pragma once

#include <moveit_msgs/msg/planning_scene.hpp>
#include <optional>
#include <utility>

namespace hb_robot_skills {

// The monitored topic contains both complete snapshots and edit operations.
// Joint feedback can republish the same collision environment many times during
// one plan. Compare full environments by value, but never discard a relative
// collision-object edit such as APPEND just because its payload was seen before.
class SceneRevisionTracker {
  using Scene = moveit_msgs::msg::PlanningScene;
 public:
  bool changed(const Scene& msg) {
    const auto& robot = msg.robot_state;
    const auto& acm = msg.allowed_collision_matrix;
    const bool object_edit = !msg.world.collision_objects.empty() ||
        !msg.world.octomap.octomap.id.empty() || !msg.world.octomap.octomap.data.empty() ||
        !robot.attached_collision_objects.empty();
    // A replacement robot state can also remove all previously attached bodies.
    const bool replaces_attachments = !robot.is_diff && may_have_attachments_ &&
        (!robot.joint_state.name.empty() || !robot.multi_dof_joint_state.joint_names.empty());
    const bool config_edit = !msg.fixed_frame_transforms.empty() || !msg.link_padding.empty() ||
        !msg.link_scale.empty() || !acm.entry_names.empty() || !acm.default_entry_names.empty();
    if (msg.is_diff && !object_edit && !config_edit && !replaces_attachments) return false;

    if (!msg.is_diff || replaces_attachments)
      may_have_attachments_ = !robot.attached_collision_objects.empty();
    else if (!robot.attached_collision_objects.empty())
      may_have_attachments_ = true;

    if (msg.is_diff && (object_edit || replaces_attachments)) {
      previous_.reset();
      return true;
    }
    Scene environment;
    environment.is_diff = msg.is_diff;
    environment.robot_model_name = msg.robot_model_name;
    environment.world = msg.world;
    environment.robot_state.attached_collision_objects = robot.attached_collision_objects;
    environment.allowed_collision_matrix = acm;
    environment.link_padding = msg.link_padding;
    environment.link_scale = msg.link_scale;
    environment.fixed_frame_transforms = msg.fixed_frame_transforms;
    // The transform values, parent and child are explicit. Only the time at
    // which an identical transform was republished is irrelevant here.
    for (auto& tf : environment.fixed_frame_transforms) tf.header.stamp = {};

    if (previous_ && *previous_ == environment) return false;
    previous_ = std::move(environment);
    return true;
  }
 private:
  std::optional<Scene> previous_;
  bool may_have_attachments_ = false;
};

}  // namespace hb_robot_skills
