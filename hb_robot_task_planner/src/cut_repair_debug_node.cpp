#include <hb_robot_interfaces/action/cut_profile.hpp>
#include <hb_robot_interfaces/action/inspect_cut.hpp>
#include <hb_robot_interfaces/msg/platform_supervision.hpp>
#include <hb_robot_interfaces/msg/profile_estimate.hpp>
#include <hb_robot_interfaces/recovery.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace recovery = hb_robot::recovery;
using Clock = std::chrono::steady_clock;
using Cut = hb_robot_interfaces::action::CutProfile;
using Inspect = hb_robot_interfaces::action::InspectCut;
using Platform = hb_robot_interfaces::msg::PlatformSupervision;
using Estimate = hb_robot_interfaces::msg::ProfileEstimate;
using Trigger = std_srvs::srv::Trigger;
using namespace std::chrono_literals;

// A manually stepped client of the real skills. It owns no motion, torch I/O,
// task authority, or BT session. All callbacks use one mutually exclusive group;
// service callbacks queue work and never wait for actions or parameter services.
class CutRepairDebugNode : public rclcpp::Node {
 public:
  CutRepairDebugNode() : Node("cut_repair_debug") {
    cut_id_ = config<std::string>("cut_id", "");
    parent_id_ = config<std::string>("parent_plan_id", "");
    geometry_version_ = config<int64_t>("geometry_version", 0);
    expected_execution_ = config<int64_t>("execution_version", -1);
    required_ = config<std::vector<std::string>>("required_segments", {});
    source_ = config<std::string>("interval_source", "evidence");
    if (source_ != "evidence" && source_ != "manual")
      throw std::invalid_argument("interval_source must be evidence or manual");
    const auto names = config<std::vector<std::string>>("residual_segments", {});
    const auto starts = config<std::vector<double>>("residual_starts", {});
    const auto ends = config<std::vector<double>>("residual_ends", {});
    if (names.size() != starts.size() || names.size() != ends.size())
      throw std::invalid_argument("Residual arrays must have equal lengths");
    for (size_t i = 0; i < names.size(); ++i) {
      recovery::Interval interval;
      interval.segment_id = names[i]; interval.s0 = starts[i]; interval.s1 = ends[i];
      if (!recovery::validInterval(interval))
        throw std::invalid_argument("Invalid residual interval");
      manual_.push_back(interval);
    }
    allow_inspection_ = config<bool>("allow_inspection_motion", false);
    allow_execute_ = config<bool>("allow_recut_execution", false);
    allow_live_ = config<bool>("allow_live_cutting", false);
    standoff_ = positive("standoff", 0.01);
    pos_tol_ = positive("pos_tol", 0.01);
    ang_tol_ = positive("ang_tol", 0.3);
    camera_standoff_ = positive("camera_standoff", 0.30);
    sample_spacing_ = positive("sample_spacing", 0.025);
    samples_ = config<int64_t>("samples_per_viewpoint", 3);
    if (samples_ < 1 || samples_ > 100)
      throw std::invalid_argument("samples_per_viewpoint must be in [1,100]");
    cut_endpoint_ = config<std::string>("cut_action", "/cut_profile");
    inspect_endpoint_ = config<std::string>("inspect_action", "/inspect_cut");
    cut_node_ = config<std::string>("cut_server_node", "/cut_profile_server");
    inspect_node_ = config<std::string>("inspect_server_node", "/inspect_cut_server");

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = create_subscription<recovery::State>("/task_state", latched,
      [this](recovery::State::ConstSharedPtr m) {
        Clock::time_point at;
        if (sampleTime(m->header.stamp, at)) { state_ = *m; state_at_ = at; }
      });
    platform_sub_ = create_subscription<Platform>("/platform_supervision", rclcpp::QoS(1).reliable(),
      [this](Platform::ConstSharedPtr m) {
        Clock::time_point at;
        if (sampleTime(m->header.stamp, at)) { platform_ = *m; platform_at_ = at; }
      });
    estimate_sub_ = create_subscription<Estimate>("/profile_inspection/accepted_estimate", latched,
      [this](Estimate::ConstSharedPtr m) { estimate_ = *m; have_estimate_ = true; });
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched);
    evidence_pub_ = create_publisher<recovery::Evidence>("~/evidence", latched);
    for (const std::string name : {"status", "bind", "inspect", "plan_recut", "execute_recut", "cancel"}) {
      services_.push_back(create_service<Trigger>("~/" + name,
        [this, name](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
          try { command(name, *response); }
          catch (const std::exception& e) { response->success = false; response->message = e.what(); }
          publishStatus();
        }));
    }
  }

  void initialize() {
    cut_ = rclcpp_action::create_client<Cut>(shared_from_this(), cut_endpoint_);
    inspect_ = rclcpp_action::create_client<Inspect>(shared_from_this(), inspect_endpoint_);
    cut_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), cut_node_);
    inspect_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), inspect_node_);
    timer_ = create_wall_timer(100ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Debug services ready: ~/bind, ~/inspect, ~/plan_recut, "
      "~/execute_recut, ~/cancel, ~/status. Production servers retain all interlocks.");
  }

 private:
  template<class T> T config(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  double positive(const std::string& name, double value) {
    value = config<double>(name, value);
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be positive");
    return value;
  }
  bool sampleTime(const builtin_interfaces::msg::Time& stamp, Clock::time_point& at) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u || (stamp.sec == 0 && stamp.nanosec == 0))
      return false;
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    if (!std::isfinite(age) || age < -0.005 || age > 5.0) return false;
    at = Clock::now() - std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(std::max(0.0, age)));
    return true;
  }
  static double ageMs(Clock::time_point at) {
    return at == Clock::time_point{} ? -1.0 :
      std::chrono::duration<double, std::milli>(Clock::now() - at).count();
  }
  std::string gate(bool starting, bool require_binding = true, bool check_execution = true) {
    // There is no cross-server robot-motion lease. A live BT, including a paused
    // one, can resume independently; do not share command ownership with it.
    if (count_publishers("/task_session/status") != 0) return "bt_executor still owns a task session";
    if (ageMs(state_at_) < 0 || ageMs(state_at_) >= 500) return "TaskState older than 500 ms or missing";
    if (ageMs(platform_at_) < 0 || ageMs(platform_at_) >= 500) return "PlatformSupervision older than 500 ms or missing";
    if (!recovery::geometryPermitted(state_)) return "TaskState denies motion: " + state_.status_message;
    if (!platform_.supervision_valid || !platform_.imu_stable || platform_.hard_motion_detected)
      return "Platform denies motion: " + platform_.reason;
    if (starting && (state_.supervision_uncertain || platform_.state == Platform::UNCERTAIN))
      return "Platform uncertain; new operation must wait";
    if (!have_estimate_ || estimate_.header.stamp.sec < 0 || estimate_.header.stamp.nanosec >= 1000000000u ||
        state_.scene_id != std::to_string(rclcpp::Time(estimate_.header.stamp).nanoseconds()))
      return "Accepted estimate does not match TaskState.scene_id";
    if (require_binding) {
      if (!bound_) return "Call ~/bind with the original cut provenance first";
      if (!recovery::sameVersions(state_, bound_state_) || state_.scene_id != bound_state_.scene_id ||
          state_.cut_environment_id != bound_state_.cut_environment_id)
        return "Bound geometry/pose/scene/environment changed; rebind and replan";
      if (check_execution && state_.execution_version != static_cast<uint64_t>(expected_execution_))
        return "Execution version changed; previous evidence/candidate is no longer applicable";
    }
    return {};
  }
  void command(const std::string& name, Trigger::Response& response) {
    if (name == "status") { response.success = true; response.message = status(); return; }
    if (name == "cancel") {
      response.success = !operation_.empty();
      response.message = response.success ? "Cancellation requested; remain busy until terminal result" : "No owned action";
      if (response.success) cancel("operator request");
      return;
    }
    if (!operation_.empty()) { response.message = "Busy: " + operation_ + "; use ~/cancel"; return; }
    if (name == "bind") {
      const auto reason = gate(true, false);
      if (!reason.empty()) { response.message = reason; return; }
      const std::set<std::string> unique(required_.begin(), required_.end());
      if (cut_id_.empty() || parent_id_.empty() || geometry_version_ <= 0 || expected_execution_ < 0 ||
          required_.empty() || unique.size() != required_.size() || unique.count("")) {
        response.message = "Set cut_id, parent_plan_id, geometry_version, execution_version and unique required_segments";
        return;
      }
      if (state_.geometry_version != static_cast<uint64_t>(geometry_version_) ||
          state_.execution_version != static_cast<uint64_t>(expected_execution_)) {
        response.message = "Configured original cut geometry/execution does not match TaskState"; return;
      }
      if (state_.cut_environment_id.empty()) { response.message = "Capture the cut environment through the normal workflow first"; return; }
      bound_state_ = state_; bound_estimate_ = estimate_; bound_ = true;
      candidate_.clear(); evidence_ = {}; last_ = "Bound operator-supplied cut provenance; parent validity checked by cut server when planning";
      response.success = true; response.message = last_; return;
    }
    const auto reason = gate(true);
    if (!reason.empty()) { response.message = reason; return; }
    if (name == "execute_recut" && (!allow_execute_ || candidate_.empty())) {
      response.message = "Need allow_recut_execution=true and a successfully planned recut"; return;
    }
    if (name == "plan_recut") {
      intervals_ = residuals(); // Validate before discarding an existing candidate.
      candidate_.clear();
    }
    const bool inspection = name == "inspect";
    if (inspection) { evidence_ = {}; candidate_.clear(); }
    const auto parameters = inspection ? inspect_parameters_ : cut_parameters_;
    if (!parameters->service_is_ready() || (inspection ? !inspect_->action_server_is_ready() : !cut_->action_server_is_ready())) {
      response.message = "Action or parameter server unavailable"; return;
    }
    operation_ = name; configuring_ = true; cancel_requested_ = false; action_requested_ = false;
    last_feedback_ = {};
    started_ = Clock::now(); const auto ticket = ++ticket_;
    const std::vector<std::string> names = inspection ? std::vector<std::string>{"preview_only"} :
      std::vector<std::string>{"cut_planning.mode", "require_plan_approval", "skip_cutting_motion", "enable_cutting_process"};
    try {
      parameters->get_parameters(names, [this, ticket](auto future) {
        if (ticket != ticket_ || !configuring_) return;
        try {
          const auto values = future.get();
          const auto denied = gate(true);
          if (!denied.empty()) throw std::runtime_error(denied);
          if (operation_ == "inspect") {
            if (values.size() != 1) throw std::runtime_error("Missing inspection parameters");
            if (!values[0].as_bool() && !allow_inspection_)
              throw std::runtime_error("Inspection server can move: set allow_inspection_motion=true explicitly");
          } else {
            if (values.size() != 4 || values[0].as_string() != "monolithic")
              throw std::runtime_error("Recut debug supports monolithic parent plans only; independent lineage is not implemented in BT");
            if (operation_ == "execute_recut") {
              if (!values[1].as_bool()) throw std::runtime_error("Production require_plan_approval must remain true");
              if (values[2].as_bool()) throw std::runtime_error("skip_cutting_motion prevents stored-plan execution");
              if (values[3].as_bool() && !allow_live_)
                throw std::runtime_error("Server enables plasma: allow_live_cutting is false");
              RCLCPP_WARN(get_logger(), "Recut execution requested: enable_cutting_process=%d; awaiting production approval", values[3].as_bool());
            }
          }
          configuring_ = false; started_ = Clock::now();
          dispatch();
          publishStatus();
        } catch (const std::exception& e) {
          if (action_requested_) {
            // A transport exception after submission is not proof the remote
            // server never received the goal. Do not release ownership.
            cancel(std::string("Dispatch outcome uncertain: ") + e.what());
          } else {
            finish(std::string("NOT SENT: ") + e.what());
          }
        }
      });
    } catch (...) { operation_.clear(); configuring_ = false; throw; }
    response.success = true; response.message = "Queued " + name + "; result and blockers appear on ~/status";
  }
  std::vector<recovery::Interval> residuals() {
    std::vector<recovery::Interval> input;
    if (source_ == "manual") {
      input = manual_;
      RCLCPP_WARN(get_logger(), "Using operator-selected diagnostic intervals, not inspection-derived residual evidence");
    } else {
      if (std::set<std::string>(evidence_.required_segments.begin(), evidence_.required_segments.end()) !=
          std::set<std::string>(required_.begin(), required_.end()))
        throw std::runtime_error("Inspection evidence does not cover the bound required feature set");
      const auto verdict = recovery::evaluate(evidence_, state_, cut_id_);
      if (verdict.status != recovery::State::RESIDUAL_MATERIAL)
        throw std::runtime_error("No valid residual evidence; preview/acquisition success does not imply a recut target");
      input = verdict.residuals;
    }
    std::map<std::string, recovery::Interval> merged;
    for (const auto& interval : input) {
      if (!recovery::validInterval(interval) || std::find(required_.begin(), required_.end(), interval.segment_id) == required_.end())
        throw std::runtime_error("Residual is invalid or outside required_segments");
      auto inserted = merged.emplace(interval.segment_id, interval);
      if (!inserted.second) {
        inserted.first->second.s0 = std::min(inserted.first->second.s0, interval.s0);
        inserted.first->second.s1 = std::max(inserted.first->second.s1, interval.s1);
      }
    }
    if (merged.empty()) throw std::runtime_error("No residual intervals configured");
    std::vector<recovery::Interval> result;
    for (const auto& entry : merged) result.push_back(entry.second);
    return result;
  }
  template<class Action, class Done>
  void send(const typename rclcpp_action::Client<Action>::SharedPtr& client,
            const typename Action::Goal& goal, Done done) {
    const auto ticket = ticket_;
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, client, ticket](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle) {
      if (ticket != ticket_) return;
      if (!handle) { finish("REJECTED by action server; inspect its terminal log"); return; }
      cancel_goal_ = [this, client, handle] {
        client->async_cancel_goal(handle, [this](auto response) {
          RCLCPP_WARN(get_logger(), "Cancel response=%d; awaiting terminal action result (not proof of containment)",
            static_cast<int>(response->return_code));
        });
      };
      if (cancel_requested_) sendCancel();
      else last_ = operation_ + " accepted";
    };
    options.feedback_callback = [this, ticket](auto, const auto&) {
      if (ticket == ticket_) last_feedback_ = Clock::now();
    };
    options.result_callback = [this, ticket, done](const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult& wrapped) {
      if (ticket != ticket_) return;
      if (!wrapped.result) { finish("No action result; outcome unknown"); return; }
      done(*wrapped.result, wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && !cancel_requested_);
    };
    action_requested_ = true;
    client->async_send_goal(goal, options);
  }
  void dispatch() {
    if (operation_ == "inspect") {
      Inspect::Goal goal;
      goal.estimate = bound_estimate_; goal.cut_id = cut_id_; goal.required_segments = required_;
      goal.geometry_version = bound_state_.geometry_version; goal.pose_version = bound_state_.pose_version;
      goal.execution_version = static_cast<uint64_t>(expected_execution_);
      goal.sample_spacing = sample_spacing_; goal.camera_standoff = camera_standoff_;
      goal.samples_per_viewpoint = static_cast<uint32_t>(samples_);
      send<Inspect>(inspect_, goal, [this](const Inspect::Result& result, bool succeeded) {
        // Publish the real evidence, including partial/invalid results, for recording.
        evidence_pub_->publish(result.evidence);
        if (succeeded && result.success && gate(false).empty()) evidence_ = result.evidence;
        const auto verdict = recovery::evaluate(evidence_, state_, cut_id_);
        finish("inspection success=" + std::to_string(result.success) + " code=" + std::to_string(result.result_code) +
          " views=" + std::to_string(result.viewpoints_captured) + "/" + std::to_string(result.viewpoints_requested) +
          " evidence_verdict=" + std::to_string(verdict.status) + " message=" + result.message);
      });
      return;
    }
    Cut::Goal goal;
    goal.standoff = static_cast<float>(standoff_); goal.pos_tol = static_cast<float>(pos_tol_);
    goal.ang_tol = static_cast<float>(ang_tol_); goal.cut_id = cut_id_;
    goal.geometry_version = bound_state_.geometry_version; goal.pose_version = bound_state_.pose_version;
    goal.scene_version = bound_state_.scene_version;
    goal.execute = operation_ == "execute_recut";
    if (goal.execute) {
      goal.plan_id = candidate_; candidate_.clear(); evidence_ = {}; // Never replay an uncertain/consumed plan.
    } else { goal.parent_plan_id = parent_id_; goal.intervals = intervals_; }
    send<Cut>(cut_, goal, [this, executing = goal.execute](const Cut::Result& result, bool succeeded) {
      if (!executing && succeeded && result.success && !result.executed && !result.interrupted &&
          !result.plan_id.empty() && result.feature_plans.empty() && result.batch_id.empty() &&
          result.cut_id == cut_id_ && result.parent_plan_id == parent_id_ && gate(false).empty())
        candidate_ = result.plan_id;
      if (executing && result.executed && result.execution_version > static_cast<uint64_t>(expected_execution_))
        expected_execution_ = static_cast<int64_t>(result.execution_version);
      finish("cut success=" + std::to_string(result.success) + " executed=" + std::to_string(result.executed) +
        " interrupted=" + std::to_string(result.interrupted) + " plan=" + result.plan_id +
        " execution=" + std::to_string(result.execution_version) + " candidate=" + candidate_ + " message=" + result.message);
    });
  }
  void cancel(const std::string& reason) {
    if (cancel_requested_) return;
    cancel_requested_ = true; candidate_.clear(); evidence_ = {};
    last_ = "Cancellation requested: " + reason;
    RCLCPP_WARN(get_logger(), "%s", last_.c_str());
    if (configuring_) { finish("Canceled before sending any action"); return; }
    sendCancel();
    // No goal handle yet: goal_response_callback cancels it when it arrives.
    // Keep ownership until a terminal result, including after a timeout.
  }
  void sendCancel() {
    if (!cancel_goal_) return;
    try { cancel_goal_(); }
    catch (const std::exception& e) { last_ += std::string("; cancel error: ") + e.what(); }
  }
  void finish(const std::string& message) {
    last_ = operation_ + ": " + message;
    RCLCPP_INFO(get_logger(), "%s", last_.c_str());
    operation_.clear(); configuring_ = false; cancel_goal_ = {}; publishStatus();
  }
  void tick() {
    if (!operation_.empty()) {
      const auto denied = gate(false, true, operation_ != "execute_recut");
      if (!denied.empty()) cancel(denied);
      else if (Clock::now() - started_ > (configuring_ ? 10s : operation_ == "plan_recut" ? 90s : 600s))
        cancel("operation deadline exceeded");
    }
    if (Clock::now() - published_ >= 1s) publishStatus();
  }
  std::string status() {
    std::ostringstream out;
    out << "operation=" << (operation_.empty() ? "idle" : operation_) << " checking_parameters=" << configuring_
        << " cancel_pending=" << (!operation_.empty() && cancel_requested_) << " bound=" << bound_
        << " task_age_ms=" << ageMs(state_at_) << " platform_age_ms=" << ageMs(platform_at_)
        << " feedback_age_ms=" << ageMs(last_feedback_) << " cut=" << cut_id_ << " parent=" << parent_id_
        << " candidate=" << candidate_ << " geometry=" << state_.geometry_version << '/' << geometry_version_
        << " pose=" << state_.pose_version << " scene=" << state_.scene_version
        << " execution=" << state_.execution_version << '/' << expected_execution_
        << " environment=" << state_.cut_environment_id << " interval_source=" << source_
        << " start_gate='" << gate(true) << "' last='" << last_ << "'";
    return out.str();
  }
  void publishStatus() {
    std_msgs::msg::String msg; msg.data = status(); status_pub_->publish(msg); published_ = Clock::now();
  }

  recovery::State state_, bound_state_;
  Platform platform_;
  Estimate estimate_, bound_estimate_;
  recovery::Evidence evidence_;
  Clock::time_point state_at_{}, platform_at_{}, started_{}, published_{}, last_feedback_{};
  bool have_estimate_ = false, bound_ = false, configuring_ = false, cancel_requested_ = false;
  bool action_requested_ = false;
  bool allow_inspection_, allow_execute_, allow_live_;
  int64_t geometry_version_, expected_execution_, samples_;
  uint64_t ticket_ = 0;
  double standoff_, pos_tol_, ang_tol_, camera_standoff_, sample_spacing_;
  std::string cut_id_, parent_id_, candidate_, source_, operation_, last_;
  std::string cut_endpoint_, inspect_endpoint_, cut_node_, inspect_node_;
  std::vector<std::string> required_;
  std::vector<recovery::Interval> manual_, intervals_;
  std::function<void()> cancel_goal_;
  rclcpp::Subscription<recovery::State>::SharedPtr state_sub_;
  rclcpp::Subscription<Platform>::SharedPtr platform_sub_;
  rclcpp::Subscription<Estimate>::SharedPtr estimate_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<recovery::Evidence>::SharedPtr evidence_pub_;
  rclcpp_action::Client<Cut>::SharedPtr cut_;
  rclcpp_action::Client<Inspect>::SharedPtr inspect_;
  rclcpp::AsyncParametersClient::SharedPtr cut_parameters_, inspect_parameters_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<CutRepairDebugNode>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("cut_repair_debug"), "%s", e.what()); code = 1;
  }
  rclcpp::shutdown();
  return code;
}
