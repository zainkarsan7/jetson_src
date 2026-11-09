#include <atomic>
#include <cmath>
#include <thread>

#include <hb_robot_interfaces/action/return_to_inspection_home.hpp>
#include <hb_robot_skills/motion/transit_planner.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp_action/rclcpp_action.hpp>

namespace {
using Action = hb_robot_interfaces::action::ReturnToInspectionHome;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using hb_robot_skills::motion::MoveGroup;

class HomeServer {
public:
  explicit HomeServer(const rclcpp::Node::SharedPtr& node)
      : node_(node), group_(node, node->get_parameter_or<std::string>("planning_group", "manipulator")),
        monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description")) {
    if (!monitor_->getPlanningScene())
      throw std::runtime_error("Cannot initialize the inspection home planning scene");
    server_ = rclcpp_action::create_server<Action>(
        node, "return_to_inspection_home",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
          return busy_.exchange(true) ? rclcpp_action::GoalResponse::REJECT :
                                       rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::REJECT; },
        [this](std::shared_ptr<Handle> goal) {
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, goal] {
            auto result = std::make_shared<Action::Result>();
            try {
              run(goal);
              result->success = true;
              result->message = "At inspection home";
            } catch (const std::exception& error) {
              result->message = error.what();
            }
            if (rclcpp::ok()) {
              if (result->success) goal->succeed(result);
              else goal->abort(result);
            }
            busy_ = false;
          });
        });
  }

  ~HomeServer() {
    if (worker_.joinable()) worker_.join();
  }

private:
  void syncScene() {
    if (!monitor_->requestPlanningSceneState("/get_planning_scene"))
      throw std::runtime_error("Could not obtain the current planning scene");
  }

  bool valid(const MoveGroup::Plan& plan) const {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    const auto& path = plan.trajectory_.joint_trajectory;
    auto state = scene->getCurrentState();
    const auto* joints = state.getJointModelGroup(group_.getName());
    if (path.points.empty() || path.joint_names.size() != joints->getActiveJointModels().size())
      return false;
    for (const auto* joint : joints->getActiveJointModels()) {
      if (std::count(path.joint_names.begin(), path.joint_names.end(), joint->getName()) != 1)
        return false;
    }
    // Check the commanded joint coordinates, including between sparse OMPL waypoints.
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto& q = path.points[i].positions;
      const auto& prev = path.points[i ? i - 1 : 0].positions;
      double max_delta = 0;
      for (size_t j = 0; j < q.size(); ++j)
        max_delta = std::max(max_delta, std::abs(q[j] - prev[j]));
      const int steps = std::max(1, static_cast<int>(std::ceil(max_delta / 0.01)));
      for (int k = 1; k <= steps; ++k) {
        for (size_t j = 0; j < q.size(); ++j)
          state.setVariablePosition(path.joint_names[j], prev[j] + (q[j] - prev[j]) * k / steps);
        state.update();
        if (!state.satisfiesBounds(joints) || scene->isStateColliding(state, group_.getName()))
          return false;
      }
    }
    return true;
  }

  void run(const std::shared_ptr<Handle>& goal) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = "Planning";
    goal->publish_feedback(feedback);
    auto start = group_.getCurrentState(2.0);
    if (!start) throw std::runtime_error("Current robot state unavailable");
    auto target = *start;
    if (!target.setToDefaultValues(group_.getName(), "inspection_home"))
      throw std::runtime_error("Named state inspection_home unavailable");
    syncScene();
    auto plan = hb_robot_skills::motion::planTransit(
        group_, node_->get_logger(), *start, target,
        [this](const MoveGroup::Plan& candidate) { return valid(candidate); }, true);
    if (!plan) throw std::runtime_error("No collision-free transit to inspection home");
    syncScene();
    if (!valid(*plan)) throw std::runtime_error("Selected path failed final scene validation");
    feedback->phase = "Executing";
    goal->publish_feedback(feedback);
    if (group_.execute(*plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Inspection home execution failed");
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroup group_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::atomic_bool busy_{false};
  std::thread worker_;
};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "return_to_inspection_home_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  HomeServer server(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
