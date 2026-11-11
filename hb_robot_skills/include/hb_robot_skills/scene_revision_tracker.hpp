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

    bool changed(
        const Scene& msg,
        std::string* reason = nullptr)
    {
        const bool full = !msg.is_diff;
        bool dirty = full && !have_world_;

        std::string changes;

        auto record = [&](bool did_change, const char* component) {
            if (!did_change)
                return;

            if (!changes.empty())
                changes += ", ";

            changes += component;
            dirty = true;
        };

        if (dirty)
            changes = "initial_world";

        const auto& world = msg.world;

        // --------------------------------------------------
        // 1. World collision objects
        // --------------------------------------------------

        if (full)
        {
            auto objects = world.collision_objects;

            std::sort(
                objects.begin(),
                objects.end(),
                [](const Object& a, const Object& b) {
                    return a.id < b.id;
                });

            // Preserve the baseline for numerically equivalent poses.
            if (objects.size() == objects_.size())
            {
                for (size_t i = 0; i < objects.size(); ++i)
                {
                    if (objects[i].id == objects_[i].id &&
                        samePose(objects[i].pose, objects_[i].pose))
                    {
                        objects[i].pose = objects_[i].pose;
                    }
                }
            }

            for (auto& object : objects)
            {
                object.header.stamp =
                    builtin_interfaces::msg::Time{};
            }

            record(
                replace(objects_, objects),
                "collision_objects");

            have_world_ = true;
        }
        else
        {
            for (const auto& edit : world.collision_objects)
            {
                const bool object_changed =
                    applyObjectEdit(edit);

                if (object_changed)
                {
                    if (!changes.empty())
                        changes += ", ";

                    changes += "object_edit:";
                    changes += edit.id.empty()
                        ? "<unnamed>"
                        : edit.id;

                    dirty = true;
                }
            }
        }

        // --------------------------------------------------
        // 2. Octomap
        // --------------------------------------------------

        if (full ||
            !world.octomap.octomap.id.empty() ||
            !world.octomap.octomap.data.empty())
        {
            auto map = world.octomap;

            map.header.stamp =
                builtin_interfaces::msg::Time{};

            map.octomap.header.stamp =
                builtin_interfaces::msg::Time{};

            record(
                replace(octomap_, map),
                "octomap");
        }

        // --------------------------------------------------
        // 3. Allowed collision matrix
        // --------------------------------------------------

        const auto& policy = msg.allowed_collision_matrix;

        if (full ||
            !policy.entry_names.empty() ||
            !policy.default_entry_names.empty())
        {
            record(
                replace(policy_, policy),
                "allowed_collision_matrix");
        }

        // --------------------------------------------------
        // 4. Link padding and scaling
        // --------------------------------------------------

        record(
            updateLinks(padding_, msg.link_padding, full),
            "link_padding");

        record(
            updateLinks(scale_, msg.link_scale, full),
            "link_scale");

        // --------------------------------------------------
        // 5. Robot model
        // --------------------------------------------------

        if (full || !msg.robot_model_name.empty())
        {
            record(
                replace(model_, msg.robot_model_name),
                "robot_model_name");
        }

        // --------------------------------------------------
        // 6. Attached collision objects
        // --------------------------------------------------

        const auto& robot = msg.robot_state;

        if (full ||
            (!robot.is_diff &&
            !attachments_.empty() &&
            (!robot.joint_state.name.empty() ||
              !robot.multi_dof_joint_state.joint_names.empty())))
        {
            record(
                replace(
                    attachments_,
                    robot.attached_collision_objects),
                "attached_collision_objects");
        }
        else if (!robot.attached_collision_objects.empty())
        {
            // Preserve original tracker behavior:
            // an explicit attachment payload invalidates
            // even if its contents match the baseline.
            attachments_ = robot.attached_collision_objects;

            record(
                true,
                "attached_collision_objects_explicit");
        }

        // --------------------------------------------------
        // 7. Fixed-frame transforms
        // --------------------------------------------------

        const auto external = [](const std::string& frame) {
            return !frame.empty() && frame != "world";
        };

        const bool uses_tf =
            !have_world_ ||
            (!octomap_.octomap.data.empty() &&
            external(octomap_.header.frame_id)) ||
            std::any_of(
                objects_.begin(),
                objects_.end(),
                [&](const Object& obj) {
                    return external(obj.header.frame_id);
                });

        if (full || !msg.fixed_frame_transforms.empty())
        {
            auto next =
                full
                    ? decltype(transforms_){}
                    : transforms_;

            for (const auto& tf : msg.fixed_frame_transforms)
            {
                next[{
                    tf.header.frame_id,
                    tf.child_frame_id
                }] = tf.transform;
            }

            const bool transforms_changed =
                replace(transforms_, next);

            // Preserve the existing behavior:
            // update the baseline even when these transforms
            // do not affect collision geometry.
            record(
                transforms_changed && uses_tf,
                "fixed_frame_transforms");
        }

        // --------------------------------------------------
        // 8. Diagnostic output
        // --------------------------------------------------

        if (reason)
        {
            *reason = changes.empty()
                ? "no_collision_relevant_change"
                : changes;
        }

        return dirty;
}

  

 private:
  bool applyObjectEdit(Object edit) {
    // For a known world, absolute ADD/REMOVE operations have a predictable
    // result. Updating the baseline also makes the confirming full scene a no-op.
    // Relative edits and bulk operations still require a full resynchronization.
    if (!have_world_ || edit.id.empty() ||
        (edit.operation != Object::ADD && edit.operation != Object::REMOVE)) {
      have_world_ = false;
      return true;
    }
    auto pos = std::lower_bound(objects_.begin(), objects_.end(), edit.id,
        [](const Object& obj, const std::string& id) { return obj.id < id; });
    const bool exists = pos != objects_.end() && pos->id == edit.id;
    if (edit.operation == Object::REMOVE) {
      if (exists) objects_.erase(pos);
      return exists;
    }
    edit.header.stamp = builtin_interfaces::msg::Time{};
    if (exists) {
      if (samePose(edit.pose, pos->pose)) edit.pose = pos->pose;
      return replace(*pos, edit);
    }
    objects_.insert(pos, std::move(edit));
    return true;
  }

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
