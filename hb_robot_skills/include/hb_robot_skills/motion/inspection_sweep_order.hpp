#pragma once
#include <cstddef>
#include <vector>

namespace hb_robot_skills::motion {
struct InspectionVisit {
  std::size_t station;
  bool capture;
};
// Enter the middle, walk to one extremity, then retrace adjacent stations to
// reach the other. Transit-only revisits never count as acquired stations.
inline std::vector<InspectionVisit> inspectionSweepOrder(std::size_t count) {
  std::vector<InspectionVisit> visits;
  if (!count) return visits;
  const auto middle = count/2;
  for (std::size_t i=middle+1; i>0; --i) visits.push_back({i-1, true});
  for (std::size_t i=1; i<count; ++i) visits.push_back({i, i>middle});
  return visits;
}
} // namespace hb_robot_skills::motion
