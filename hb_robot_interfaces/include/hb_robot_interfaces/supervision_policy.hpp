#pragma once

namespace hb_robot::supervision {
enum class Operation { Plan, AcquirePose, StartMotion, ContinueMotion };
enum class Decision { Allow, Wait, Revalidate, Stop };

struct Evidence {
  bool fresh = false;
  bool authorized = false;
  bool uncertain = false;
  bool hard_motion = false;
  bool fault = false;
  bool pose_valid = false;
  bool scene_valid = false;
};

// The authority owns sensor fusion/grace. Consumers must not reconstruct it.
// New motion/observations wait during uncertainty; already authorized motion
// follows the authority's bounded continuation policy.
inline Decision decide(const Evidence& e, Operation operation) {
  if (e.fault || e.hard_motion) return Decision::Stop;
  if (operation == Operation::Plan)
    return e.scene_valid ? Decision::Allow : Decision::Revalidate;
  if (!e.fresh || !e.authorized)
    return operation == Operation::ContinueMotion ? Decision::Stop : Decision::Wait;
  if (e.uncertain && operation != Operation::ContinueMotion) return Decision::Wait;
  if (operation == Operation::AcquirePose) return Decision::Allow;
  if (!e.pose_valid || !e.scene_valid) return Decision::Revalidate;
  return Decision::Allow;
}
}  // namespace hb_robot::supervision