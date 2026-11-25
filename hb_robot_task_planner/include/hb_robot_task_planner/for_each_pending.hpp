#pragma once
#include <behaviortree_cpp_v3/control_node.h>
#include <functional>
#include <string>
#include <utility>

namespace hb_robot_task_planner {
// A failure retains the child's checkpoint. Only a recorded completion resets it.
class ForEachPending : public BT::ControlNode {
public:
  ForEachPending(const std::string& name, const BT::NodeConfiguration& config,
                 std::function<bool()> pending)
      : BT::ControlNode(name, config), pending_(std::move(pending)) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (childrenCount() != 1) throw BT::RuntimeError("ForEachPending needs one child");
    setStatus(BT::NodeStatus::RUNNING);
    if (!pending_()) return BT::NodeStatus::SUCCESS;
    const auto result = children_nodes_[0]->executeTick();
    if (result == BT::NodeStatus::SUCCESS) {
      haltChild(0);
      return BT::NodeStatus::RUNNING;
    }
    return result;
  }
private:
  std::function<bool()> pending_;
};
}  // namespace hb_robot_task_planner