#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace hb_robot_skills::motion {

// One wall-clock budget follows a synchronous planning call stack. Thread-local
// scope keeps concurrent action/recovery workers independent. It never applies
// to execution or to control/supervision callbacks.
class PlanningInterrupted : public std::runtime_error {
public:
  enum class Reason { Deadline, Canceled };
  PlanningInterrupted(Reason reason, const std::string& stage)
      : std::runtime_error((reason == Reason::Canceled ? "PLANNING_CANCELED: " :
                            "PLANNING_BUDGET_EXHAUSTED: ") + stage), reason(reason) {}
  Reason reason;
};

class PlanningBudget {
public:
  using Clock = std::chrono::steady_clock;
  explicit PlanningBudget(double seconds, std::function<bool()> canceled = {})
      : canceled_(std::move(canceled)) {
    if (!std::isfinite(seconds) || seconds <= 0)
      throw std::invalid_argument("Planning budget must be finite and positive");
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(seconds));
  }
  void check(const char* stage) const {
    if (canceled_ && canceled_())
      throw PlanningInterrupted(PlanningInterrupted::Reason::Canceled, stage);
    if (Clock::now() >= deadline_)
      throw PlanningInterrupted(PlanningInterrupted::Reason::Deadline, stage);
  }
  double remaining() const {
    return std::max(0.0, std::chrono::duration<double>(deadline_ - Clock::now()).count());
  }
private:
  Clock::time_point deadline_;
  std::function<bool()> canceled_;
};

class PlanningBudgetScope {
public:
  explicit PlanningBudgetScope(const PlanningBudget& budget)
      : budget_(budget), parent_(current_) { current_ = this; }
  ~PlanningBudgetScope() { current_ = parent_; }
  PlanningBudgetScope(const PlanningBudgetScope&) = delete;
  PlanningBudgetScope& operator=(const PlanningBudgetScope&) = delete;
  static void check(const char* stage) {
    for (auto scope = current_; scope; scope = scope->parent_)
      scope->budget_.check(stage);
  }
  static double slice(double requested, const char* stage) {
    check(stage);
    for (auto scope = current_; scope; scope = scope->parent_)
      requested = std::min(requested, scope->budget_.remaining());
    // Never pass zero to IK/MoveIt: some plugins interpret zero as a default.
    if (requested < 0.0001)
      throw PlanningInterrupted(PlanningInterrupted::Reason::Deadline, stage);
    return requested;
  }
private:
  const PlanningBudget& budget_;
  PlanningBudgetScope* parent_;
  inline static thread_local PlanningBudgetScope* current_ = nullptr;
};
}  // namespace hb_robot_skills::motion
