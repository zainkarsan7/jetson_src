#pragma once
#include <hb_robot_interfaces/msg/kerf_evidence.hpp>
#include <hb_robot_interfaces/msg/task_state.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

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
inline bool geometryPermitted(const State& s) {
  return s.supervision_valid && s.imu_stable && s.workpiece_pose_valid &&
    !s.process_fault_active && !s.motion_fault_active;
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
} // namespace hb_robot::recovery
