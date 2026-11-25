#pragma once
#include <hb_robot_interfaces/msg/kerf_evidence.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include <hb_robot_interfaces/supervision_policy.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace hb_robot::recovery {
using State = hb_robot_interfaces::msg::TaskState;
using Interval = hb_robot_interfaces::msg::CutInterval;
using Evidence = hb_robot_interfaces::msg::KerfEvidence;
inline bool validInterval(const Interval& i) {
  return !i.segment_id.empty() && std::isfinite(i.s0) && std::isfinite(i.s1) &&
    i.s0 >= 0 && i.s1 <= 1 && i.s0 < i.s1;
}
inline bool overlaps(const Interval& a, const Interval& b) {
  return a.segment_id == b.segment_id && a.s0 < b.s1 && b.s0 < a.s1;
}
inline bool planningPermitted(const State& s) {
  supervision::Evidence e;
  e.scene_valid = s.scene_valid;
  e.fault = s.process_fault_active || s.motion_fault_active;
  return supervision::decide(e, supervision::Operation::Plan) == supervision::Decision::Allow;
}
inline bool geometryPermitted(const State& s) {
  supervision::Evidence e;
  e.fresh = true; // The caller must additionally check transport age.
  e.authorized = s.supervision_valid && s.imu_stable;
  // Continuation uses TaskState authorization. New operations additionally
  // require fresh, stable PlatformSupervision at their admission gate.
  e.fault = s.process_fault_active || s.motion_fault_active;
  e.pose_valid = s.workpiece_pose_valid;
  e.scene_valid = s.scene_valid;
  return supervision::decide(e, supervision::Operation::ContinueMotion) == supervision::Decision::Allow;
}
inline bool sameVersions(const State& a, const State& b) {
  return a.geometry_version == b.geometry_version && a.pose_version == b.pose_version &&
    a.scene_version == b.scene_version;
}
struct Verdict {
  uint8_t status = State::INVALID_OBSERVATION;
  std::vector<Interval> residuals;
};
// Pure evaluator: SUCCESS of a BT evaluator means evaluation ran, not separation.
// Verification requires complete interval coverage of EVERY original feature.
// A global average can never hide a short bridge or an unobserved gap.
inline Verdict evaluate(const Evidence& e, const State& s, const std::string& cut_id) {
  Verdict v;
  auto fraction=[](double x){ return std::isfinite(x) && x>=0 && x<=1; };
  if (!geometryPermitted(s) || !e.registration_valid || cut_id.empty() ||
      e.cut_id!=cut_id || e.geometry_version!=s.geometry_version ||
      e.pose_version!=s.pose_version || e.execution_version!=s.execution_version ||
      e.required_segments.empty() || !fraction(e.observed_fraction) ||
      !fraction(e.open_fraction) || !fraction(e.unknown_fraction)) return v;
  std::set<std::string> required(e.required_segments.begin(),e.required_segments.end());
  if (required.size()!=e.required_segments.size() || required.count("")) return v;
  std::map<std::string,std::vector<hb_robot_interfaces::msg::KerfSegmentEvidence>> bins;
  bool all_open=true;
  for (const auto& b:e.segments) {
    if (!validInterval(b.interval) || !required.count(b.interval.segment_id) ||
        !fraction(b.observed_fraction) || !fraction(b.open_fraction) ||
        !fraction(b.residual_fraction) || !fraction(b.confidence) ||
        b.open_fraction+b.residual_fraction > b.observed_fraction+1e-9) return v;
    bins[b.interval.segment_id].push_back(b);
    if (b.residual_fraction>0 && b.confidence>=0.95) v.residuals.push_back(b.interval);
    if (!b.through_thickness_observed || b.observed_fraction<1-1e-9 ||
        b.open_fraction<1-1e-9 || b.residual_fraction>0 || b.confidence<0.95)
      all_open=false;
  }
  for (const auto& name:required) {
    auto& list=bins[name];
    std::sort(list.begin(),list.end(),[](const auto& a,const auto& b){return a.interval.s0<b.interval.s0;});
    double end=0;
    for (const auto& b:list) {
      // Overlapping bins are malformed: do not count duplicated views as coverage.
      if(b.interval.s0<end-1e-9) return Verdict{};
      if(b.interval.s0>end+1e-9) all_open=false;
      end=b.interval.s1;
    }
    if(end<1-1e-9) all_open=false;
  }
  v.status=!v.residuals.empty()?State::RESIDUAL_MATERIAL:
    (all_open?State::VERIFIED:State::INCONCLUSIVE);
  return v;
}
// Scope is supplied by the caller, never inferred from the returned evidence.
inline Verdict evaluateFeature(const Evidence& e, const State& s,
    const std::string& cut, const std::string& feature, uint64_t last_motion_epoch = 0) {
  if (feature.empty() || e.required_segments != std::vector<std::string>{feature} ||
      e.execution_version < last_motion_epoch) return {};
  return evaluate(e, s, cut);
}

// Normalized feature coordinates. Preserve gaps; one resulting interval is
// submitted per action because CutProfile accepts one interval per feature.
inline std::vector<Interval> repairIntervals(std::vector<Interval> input,
    double margin = 0.0, double adjacency = 1e-9) {
  if (!std::isfinite(margin) || margin < 0 || margin > 0.02 ||
      !std::isfinite(adjacency) || adjacency < 0 || adjacency > 0.001)
    throw std::invalid_argument("Invalid normalized repair margin/tolerance");
  for (auto& i : input) {
    if (!validInterval(i)) throw std::invalid_argument("Invalid residual interval");
    i.s0 = std::max(0.0, i.s0 - margin); i.s1 = std::min(1.0, i.s1 + margin);
  }
  std::sort(input.begin(), input.end(), [](const auto& a, const auto& b) {
    return a.segment_id == b.segment_id ? a.s0 < b.s0 : a.segment_id < b.segment_id;
  });
  std::vector<Interval> result;
  for (const auto& i : input) {
    if (!result.empty() && result.back().segment_id == i.segment_id &&
        i.s0 <= result.back().s1 + adjacency) result.back().s1 = std::max(result.back().s1, i.s1);
    else result.push_back(i);
  }
  return result;
}

struct FeatureRecoveryState {
  std::string original_plan_id, recut_plan_id;
  uint64_t last_motion_epoch = 0;
  Evidence evidence;
  Verdict verdict;
  unsigned inspection_attempts = 0, recut_attempts = 0;
  bool original_executed = false, interrupted = false;
  Interval planned_interval;
  // Write ahead of submission: an uncertain result must never enable replay.
  std::vector<Interval> attempted;
  bool verified(const State& s, const std::string& cut, const std::string& feature) const {
    return evaluateFeature(evidence, s, cut, feature, last_motion_epoch).status == State::VERIFIED;
  }
};
} // namespace hb_robot::recovery