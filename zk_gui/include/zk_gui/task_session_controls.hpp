#pragma once
#include <string>

namespace zk_gui {
// UI availability only. The executor checks live authorization on every command.
inline bool taskCommandAvailable(const std::string& command, const std::string& outcome,
                                 bool fresh, bool contained) {
  if (!fresh) return false;
  if (command=="cancel") return outcome!="CANCELED_SAFE";
  if (command=="retry_containment") return outcome=="FAULT_UNCONTAINED";
  if (!contained) return false;
  if (command=="resume") return outcome=="PAUSED";
  if (command=="recover") return outcome=="RECOVERY_REQUIRED";
  if (command=="inspect" || command=="defer_feature") return outcome=="NEEDS_INSPECTION";
  return false;
}
} // namespace zk_gui
