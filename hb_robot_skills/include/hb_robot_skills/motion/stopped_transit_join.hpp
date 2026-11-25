#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace hb_robot_skills::motion {
// Join independently timed, stopped arm motions without blending through the
// staging posture. Both endpoint velocities must explicitly be zero.
template<class Trajectory>
bool appendStoppedJointTrajectory(Trajectory& target,const Trajectory& source,
                                  double tolerance,std::string& error) {
  const auto fail=[&](const char* why){error=why;return false;};
  const auto stamp=[](const auto& t)->std::int64_t {
    return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
  };
  const auto valid=[&](const auto& t) {
    if(t.joint_names.empty() || t.points.size()<2 || stamp(t.points.front().time_from_start)!=0) return false;
    for(std::size_t i=0;i<t.joint_names.size();++i)
      if(t.joint_names[i].empty() || std::count(t.joint_names.begin(),t.joint_names.end(),t.joint_names[i])!=1) return false;
    std::int64_t previous=-1;
    for(const auto& p:t.points) {
      if(p.positions.size()!=t.joint_names.size() ||
         (!p.velocities.empty() && p.velocities.size()!=t.joint_names.size()) ||
         (!p.accelerations.empty() && p.accelerations.size()!=t.joint_names.size()) ||
         p.time_from_start.sec<0 || p.time_from_start.nanosec>=1000000000u) return false;
      for(const auto* values:{&p.positions,&p.velocities,&p.accelerations})
        for(double x:*values) if(!std::isfinite(x)) return false;
      const auto time=stamp(p.time_from_start);
      if(time<=previous) return false;
      previous=time;
    }
    return true;
  };
  if(!std::isfinite(tolerance) || tolerance<0 || !valid(source)) return fail("invalid source trajectory/timing");
  if(target.points.empty()) {target=source;return true;}
  if(!valid(target) || target.joint_names!=source.joint_names) return fail("incompatible joint layout/timing");
  const auto& a=target.points.back();const auto& b=source.points.front();
  const auto stopped=[&](const auto& p) {
    return p.velocities.size()==target.joint_names.size() &&
      std::all_of(p.velocities.begin(),p.velocities.end(),[](double v){return std::abs(v)<=1e-6;});
  };
  if(!stopped(a) || !stopped(b)) return fail("staging boundary is not explicitly stopped");
  for(std::size_t i=0;i<a.positions.size();++i)
    if(std::abs(a.positions[i]-b.positions[i])>tolerance) return fail("staging boundary position mismatch");
  const auto offset=stamp(a.time_from_start);
  if(offset+stamp(source.points.back().time_from_start)>
      static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())*1000000000LL)
    return fail("combined trajectory duration overflow");
  // Drop exactly the duplicate boundary; retain the first leg's zero velocity.
  for(std::size_t i=1;i<source.points.size();++i) {
    auto p=source.points[i];const auto time=offset+stamp(p.time_from_start);
    p.time_from_start.sec=static_cast<std::int32_t>(time/1000000000LL);
    p.time_from_start.nanosec=static_cast<std::uint32_t>(time%1000000000LL);
    target.points.push_back(std::move(p));
  }
  return true;
}
} // namespace hb_robot_skills::motion
