# Cut candidate and transit tuning

The active server settings are in `hb_robot_bringup/launch/skills.launch.py`.
Parameters are read when the server initializes; restart it after changing them.

The geometric constraint planner still chooses the cut geometry and finds nearby
feasible IK states. The process generator now uses its approach state as the first
seed (retract state for backward solving), includes its solved local-Z roll, and
also includes zero roll. These are hints, not executable paths: every Cartesian
sample still passes IK, bounds, collision, TCP error and branch-continuity checks.
The nominal Cartesian path is preserved; small position/tilt deviations accepted
by the geometric constraints are not copied into the cutting path.

## Sampling

Default attempts per roll:

- One geometric seed, then one original robot-state seed.
- Twelve bounded perturbations around the geometric seed.
- Two globally random seeds for alternative branches.

`cut_sampling.global_seeds_per_roll` controls the last category. Set it to zero
to disable explicit full-range random seeding; the IK solver may still perform
its own search for the unconstrained reference attempt. Global attempts must be
fewer than `cut_sampling.seeds_per_roll`.

`cut_sampling.local_seed_radius_rad` defaults to 0.35 radians per variable.
`cut_sampling.local_ik_consistency_rad` defaults to 0.75 radians around the initial
local/geometric seed. Later IK solves use the existing 0.20-radian continuation
limit around the preceding state. Bounded jitter is reproducible; globally random
seeds and the IK solver are not made deterministic by this change.

`cut_sampling.roll_samples` is the grid size; zero and the geometric roll can add
samples, and equivalent roll orientations are deduplicated.

The generator and graph share `cut_sampling.max_candidates` (default 32).
Retention ranks process motion plus estimated round-trip transit cost. The best
24 candidates occupy the first slots; up to eight remaining slots retain distinct
entry/exit configurations (joint-group distance over 0.75). If fewer distinct
configurations exist, the remaining cheapest candidates fill those slots.

## Transit objective

For each planned transit, let `total` be accumulated travel summed across arm
joints and `maximum` the largest accumulated travel of any one joint. Cost is:

```
transit_weight * (total + 2 * maximum
  + excess_travel_weight * max(0, maximum - preferred_joint_travel_rad)^2)
```

This retains the existing total/maximum travel objective and adds a soft budget.
The default preferred budget is 1.57 radians and excess weight is 2.0. The budget
is accumulated travel, not endpoint difference or a velocity limit. Estimates use
the same objective on endpoint displacement; actual costs include detours.

Increase `cut_cost.excess_travel_weight` to discourage long individual-joint moves.
Increase `cut_cost.transit_weight` relative to `cut_cost.process_weight` to favor
shorter transit at the expense of more process motion.

`cut_cost.max_transit_joint_travel_rad` optionally rejects transits exceeding an
absolute accumulated per-joint travel limit. Zero (default) disables rejection,
preserving expensive alternatives when they are the only feasible plans. A positive
limit is never automatically relaxed: the graph tries other candidates or fails.
The graph logs actual travel and selected costs using its existing diagnostics.

Unplanned process nodes now use endpoint-distance lower bounds instead of sampled
path scores, so an overestimated sampled score does not hide a cheaper regenerated
process path. Process scores remain useful for candidate retention.

The graph still returns to the original robot state after each cut. It does not
optimize cut order or direct transitions between cuts. Pilz still regenerates
process paths and validates their endpoint connections. The geometric planner's
runtime is unchanged; the benefit is reuse of its work in candidate generation.

## Validation on the ROS host

```
colcon build --packages-select hb_robot_skills --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hb_robot_skills --event-handlers console_direct+
colcon test-result --verbose
```

The graph regression tests cover pessimistic sampled scores, actual transit
detours, an alternative route under a hard travel limit, and no admissible route.
Compare representative profile plans with the existing approval preview enabled:
record success rate, planning time, total travel and maximum per-joint travel.
These defaults are tuning starting points, not robot-validated thresholds.
