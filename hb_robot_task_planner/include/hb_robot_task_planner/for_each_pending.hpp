#pragma once
#include <behaviortree_cpp_v3/control_node.h>
#include <functional>
#include <string>
#include <utility>

namespace hb_robot_task_planner {
// A failure retains the child's checkpoint unless the caller proves that this
// iteration ended through a deliberate, nonexecuted operator rejection.
class ForEachPending : public BT::ControlNode {
public:
  ForEachPending(const std::string& name, const BT::NodeConfiguration& config,
                std::function<bool()> pending,
                 std::function<bool()> advance_rejected = {})
      : BT::ControlNode(name, config), pending_(std::move(pending)),
        advance_rejected_(std::move(advance_rejected)) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (childrenCount() != 1) throw BT::RuntimeError("ForEachPending needs one child");
    setStatus(BT::NodeStatus::RUNNING);
     if (advance_rejected_ && advance_rejected_()) {
      // haltChild alone resets status but does not call halt() on a failed
      // SequenceStar; explicitly reset its retained child index.
      children_nodes_[0]->halt();
      haltChild(0);
      return BT::NodeStatus::RUNNING;
    }
    if (!pending_()) return BT::NodeStatus::SUCCESS;
    const auto result = children_nodes_[0]->executeTick();
     if (result == BT::NodeStatus::FAILURE && advance_rejected_ && advance_rejected_()) {
      children_nodes_[0]->halt();
      haltChild(0);
      return BT::NodeStatus::RUNNING;
    }
    if (result == BT::NodeStatus::SUCCESS) {
      haltChild(0);
      return BT::NodeStatus::RUNNING;
    }
    return result;
  }
private:
  std::function<bool()> pending_;
  std::function<bool()> advance_rejected_;

};
}  // namespace hb_robot_task_planner