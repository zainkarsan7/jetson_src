#pragma once

#include <moveit_msgs/msg/planning_scene.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>

namespace hb_robot_skills {

// MoveIt interleaves full scenes and sparse diffs on monitored_planning_scene.
// An omitted field in a diff means "keep the previous value". Comparing adjacent
// messages would therefore turn ordinary monitor traffic into scene revisions.
class SceneRevisionTracker {
  using Scene = moveit_msgs::msg::PlanningScene;
  using Object = moveit_msgs::msg::CollisionObject;

 public:
  bool changed(const Scene& msg) {
    const bool full = !msg.is_diff;
    bool dirty = full && !have_world_;
    const auto& world = msg.world;
    if (full) {
      auto objects = world.collision_objects;
      std::sort(objects.begin(), objects.end(),
                [](const Object& a, const Object& b) { return a.id < b.id; });
      // MoveIt can reconstruct the same rigid pose with different last bits.
      // Retain the accepted baseline on equivalent samples so drift accumulates.
      if (objects.size() == objects_.size()) {
        for (size_t i = 0; i < objects.size(); ++i) {
          if (objects[i].id == objects_[i].id && samePose(objects[i].pose, objects_[i].pose))
            objects[i].pose = objects_[i].pose;
        }
      }
      for (auto& object : objects) object.header.stamp = builtin_interfaces::msg::Time{};
      dirty |= replace(objects_, objects);
      have_world_ = true;
    } else if (!world.collision_objects.empty()) {
      // These are operations (including relative APPEND), not a world snapshot.
      // Invalidate immediately and resynchronize at the next full scene.
      have_world_ = false;
      dirty = true;
    }

    if (full || !world.octomap.octomap.id.empty() || !world.octomap.octomap.data.empty()) {
      auto map = world.octomap;
      map.header.stamp = builtin_interfaces::msg::Time{};
      map.octomap.header.stamp = builtin_interfaces::msg::Time{};
      dirty |= replace(octomap_, map);
    }
    const auto& policy = msg.allowed_collision_matrix;
    if (full || !policy.entry_names.empty() || !policy.default_entry_names.empty())
      dirty |= replace(policy_, policy);
    dirty |= updateLinks(padding_, msg.link_padding, full);
    dirty |= updateLinks(scale_, msg.link_scale, full);
    if (full || !msg.robot_model_name.empty())
      dirty |= replace(model_, msg.robot_model_name);

    // No attached tool changes occur in the cutting cell. Ordinary joint
    // feedback is irrelevant. Still invalidate on an explicit payload edit.
    const auto& robot = msg.robot_state;
    if (full || (!robot.is_diff && !attachments_.empty() &&
                 (!robot.joint_state.name.empty() || !robot.multi_dof_joint_state.joint_names.empty())))
      dirty |= replace(attachments_, robot.attached_collision_objects);
    else if (!robot.attached_collision_objects.empty()) {
      attachments_ = robot.attached_collision_objects;
      dirty = true;
    }

    // The cell's collision meshes are expressed in world. Camera, IMU and
    // tool TF updates cannot move them. If geometry uses another frame (or
    // the world is unknown), conservatively watch the whole transform chain.
    const auto external = [](const std::string& frame) {
      return !frame.empty() && frame != "world";
    };
    const bool uses_tf = !have_world_ ||
        (!octomap_.octomap.data.empty() && external(octomap_.header.frame_id)) ||
        std::any_of(objects_.begin(), objects_.end(),
                    [&](const Object& obj) { return external(obj.header.frame_id); });
    if (full || !msg.fixed_frame_transforms.empty()) {
      auto next = full ? decltype(transforms_){} : transforms_;
      for (const auto& tf : msg.fixed_frame_transforms)
        next[{tf.header.frame_id, tf.child_frame_id}] = tf.transform;
      dirty |= replace(transforms_, next) && uses_tf;
    }
    return dirty;
  }

 private:
  template <typename T>
  static bool replace(T& previous, const T& value) {
    if (previous == value) return false;
    previous = value;
    return true;
  }

  template <typename Links>
  static bool updateLinks(std::map<std::string, typename Links::value_type>& saved,
                          const Links& links, bool full) {
    if (!full && links.empty()) return false;
    std::map<std::string, typename Links::value_type> next;
    if (!full) next = saved;
    for (const auto& link : links) next[link.link_name] = link;
    return replace(saved, next);
  }

  static bool samePose(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    const auto near = [](double x, double y) { return std::abs(x - y) <= 1e-12; };
    const auto& p = a.position;
    const auto& q = a.orientation;
    const auto& r = b.orientation;
    // q and -q encode the same orientation.
    const auto same_rotation = [&](double sign) {
      return near(q.x, sign*r.x) && near(q.y, sign*r.y) &&
             near(q.z, sign*r.z) && near(q.w, sign*r.w);
    };
    return near(p.x, b.position.x) && near(p.y, b.position.y) && near(p.z, b.position.z) &&
           (same_rotation(1) || same_rotation(-1));
  }

  bool have_world_ = false;
  Scene::_world_type::_collision_objects_type objects_;
  Scene::_world_type::_octomap_type octomap_;
  Scene::_robot_state_type::_attached_collision_objects_type attachments_;
  Scene::_allowed_collision_matrix_type policy_;
  std::map<std::string, moveit_msgs::msg::LinkPadding> padding_;
  std::map<std::string, moveit_msgs::msg::LinkScale> scale_;
  std::map<std::pair<std::string, std::string>, geometry_msgs::msg::Transform> transforms_;
  std::string model_;
};

}  // namespace hb_robot_skills
