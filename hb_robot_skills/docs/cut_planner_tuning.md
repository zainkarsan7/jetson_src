# Cut sampling, position bounds, and inter-cut transits

## Position and collision policy

The action's `pos_tol` is a hard radial bound about the **original nominal TCP
position**, not an allowance to accumulate displacement at every correction.
`ang_tol` constrains tilt; local-Z roll remains free within the orientation
constraint. A positive finite `pos_tol` is required. No tolerance is relaxed on
failure.

Geometric sampling keeps the exact nominal translation. It first tries the previous
solution's orientation and nominal orientation, then restores MoveIt's native
`IKConstraintSampler::samplePose` distribution across the complete orientation
constraint. The sampled translation is discarded: the position target is never
shifted. This replaces the narrower hand-written half-tilt/golden-roll distribution,
which could miss top-flange solutions.

Most IK attempts use the reference/best feasible seed with consistency limits.
Every fourth attempt (when enabled) explores another bounded robot configuration
without the local consistency restriction, so elbow/wrist branches are not all
excluded by the same reference seed. Bounds, position/orientation constraints and
collision checks are passed INTO the IK validity callback, allowing a supporting
solver to try another solution for the same orientation. Results are independently
rechecked afterward. The shallowest admissible colliding solution is retained for
repair if no collision-free IK result is found. Repeated equivalent solutions do
not count toward `feasible_samples`; that setting is an early-stop target, not a
minimum required for success.

A waypoint beam retains up to three distinct configurations through approach,
start, end and retract instead of irreversibly selecting the cheapest approach.
The total pose-attempt budget at each phase is split across parent branches, not
multiplied by beam width. A dead-end parent does not discard the other parents.
This is still a bounded beam heuristic, not exhaustive backtracking or a guarantee
of recovering every solution the old positional sampler found.

Body-contact correction uses a bounded joint step
in the TCP translation Jacobian's null space; IK projects its orientation proposal
back onto the original position. Roll/tilt proposals provide an alternative for
torch contacts. Every accepted step must satisfy the original position/orientation
constraints and joint bounds, and reduce contact merit. No progress means failure
of that sample, not a larger positional allowance. The last repair iteration is
now checked for success.

This is a bounded local heuristic, not a globally optimal collision solver. It
can reject a reachable solution if escaping contact requires first worsening the
contact merit. Increase orientation sampling before increasing repair iterations.
Self/environment collisions still fail unless the final planning-scene collision
check accepts the state; no allowed-collision entries were changed.

Process generation tries the full geometric orientation (including tilt) before
the roll grid, always with the original path positions. Each sampled IK solution
must meet both the generator accuracy limit and `pos_tol`, as well as orientation,
collision, bounds, and branch-continuity checks. Regenerated Pilz trajectories
are checked at their waypoints against the original finite approach/cut/retract
lines and endpoint positions, plus orientation constraints. This is sampled path
validation, not a continuous collision/geometry proof between waypoints. MoveIt's
existing planning-scene collision checking remains required.

The alternative box path constraint now derives its cross-section from `pos_tol`
and includes the orientation constraint. It no longer adds 10 mm to the cut length
or silently substitutes a fixed 0.5 mm cross-section.

## Transit ownership and return policy

Example direct route, with cut order unchanged:

```
initial state -> A approach -> A cut -> A retract
              -> B approach -> B cut -> B retract -> initial state
```

By default the graph retains BOTH candidate-to-candidate edges and a home node
at every boundary, with edges from every previous candidate and to every next
candidate. It lazily plans the competing routes using actual regenerated process
endpoints. Direct edges compete with the SUM of both home legs under the same
travel objective; endpoint-distance estimates do not override actual planned
detours. It can choose home at one boundary and direct at the next. A failed edge removes that
candidate pair, not either entire candidate. Alternative pairs remain available.
All transits still go through the existing collision-checked planning callback and
boundary validation; a direct transit is not joint interpolation or a straight
Cartesian shortcut.

**Ownership:** `cuts[i].transit_in` owns the motion from the previous cut's
retract endpoint (or the initial state for the first cut) to this cut's approach
endpoint. `transit_out` is optional and exists only for an explicit return to the
initial state. Direct inter-cut motion is never duplicated as both A's outbound
and B's inbound trajectory. Preview and execution both follow this representation.
Torch/process sequencing and approval remain unchanged: approach, cut, retract,
then transit. Home means the captured initial robot state, not a named SRDF pose.

Launch controls:

```bash
# Automatic direct-versus-home choice, final return enabled (defaults):
ros2 launch hb_robot_bringup skills.launch.py
# Legacy route with a home return between every cut:
ros2 launch hb_robot_bringup skills.launch.py return_home_between_cuts:=true
# End at the final retract position:
ros2 launch hb_robot_bringup skills.launch.py return_home_after_last_cut:=false
```

Corresponding node parameters are `cut_transit.return_home_between_cuts` and
`cut_transit.return_home_after_last_cut`. Options are read at server initialization.
A failed or expensive direct route can select a validated home alternative.
`return_home_between_cuts:=false` now means automatic choice, not direct-only;
`true` still forces home for compatibility. The final-return option is independent.
The home state is an actual optional waypoint, not merely a seed supplied to OMPL.
Both legs are collision-planned and charged separately; hard travel limits apply
to each leg, matching the existing per-transit semantics. No limit is relaxed.
`[CUT GRAPH]` logs show selected incoming routes and home-return costs. The graph optimizes across the retained candidates and fixed cut order,
not all possible IK solutions, cut orders, or stochastic transit-planner outcomes.

## Sampling controls and recommended starting values

Edit the cut server parameter dictionary in `hb_robot_bringup/launch/skills.launch.py`
and restart the server after rebuilding/installing launch changes.

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `cut_geometry.max_pose_samples` | 48 | Total pose-attempt budget per waypoint, shared across beam parents |
| `cut_geometry.feasible_samples` | 4 | Early-stop target of distinct feasible configurations per parent; fewer can succeed |
| `cut_geometry.refinement_iterations` | 8 | Maximum monotone collision-repair iterations per sample |
| `cut_geometry.ik_timeout_s` | 0.05 | Restore the original per-attempt IK timeout, allowing callback branch search |
| `cut_geometry.waypoint_beam_width` | 3 | Distinct waypoint configurations retained for continuation |
| `cut_geometry.explore_ik_branches` | true | Reserve every fourth attempt for an alternate unrestricted IK branch |
| `cut_geometry.ik_consistency_rad` | 2.0 | Initial per-variable IK search bound |
| `cut_geometry.repair_joint_step_rad` | 0.03 | Maximum joint correction per body-contact proposal |
| `cut_geometry.repair_roll_step_rad` | 0.15 | Roll increment per contact-repair proposal |
| `cut_sampling.roll_samples` | 12 | Process roll grid; geometric orientation and zero are also tried |
| `cut_sampling.seeds_per_roll` | 16 | Maximum seed attempts per orientation |
| `cut_sampling.global_seeds_per_roll` | 2 | Last seed attempts are global if the distinct-candidate cap has not been reached |
| `cut_sampling.max_candidates_per_roll` | 2 | Stop after this many distinct process candidates per orientation; 0 disables early stopping |
| `cut_sampling.local_seed_radius_rad` | 0.35 | Bounded seed perturbation |
| `cut_sampling.local_ik_consistency_rad` | 0.75 | First IK consistency bound for geometric/local process seeds |
| `cut_sampling.max_candidates` | 32 | Retained candidates per segment |

Keep an existing 64-attempt / 8-feasible setup for comparison; increasing either is
not required to enable this strategy. Use beam width 3, branch exploration enabled,
and a 0.05-second IK timeout. `[CUT GEOMETRY]` logs identify the segment/phase and
report pose sampling failures, IK failures, constraint/collision callback
rejections, repairs, duplicates and distinct feasible results. Callback rejection
counts can exceed the number of pose attempts because one IK call can test several
solutions. Inspect those counts before tuning:

- Many IK failures with few constraint/collision callbacks: inspect reachability,
  solver timeout and seed/branch access.
- Many collision rejections: inspect collision geometry and accepted contacts;
  do not relax position bounds to hide incorrect geometry.
- Feasible approach but no complete chain: preserve continuation diversity; try
  beam width 4 within the same stage budget (each parent then gets fewer attempts).
- Many duplicates: more feasible samples may simply repeat the same branch.

`pos_tol` is not a performance knob. Exact-position solving remains stricter than
the older random sampling inside its sphere; a task that actually needs a shifted
cut position must fail under the fixed-position policy. Global seed exploration
and native orientation sampling are stochastic, not repeatably identical.

Direct mode ranks retained candidates by process cost with endpoint diversity,
rather than penalizing every candidate for a fictitious round trip home. Legacy
mode keeps the home-based ranking. The best 3/4 of slots use cost; remaining slots
favor distinct entry/exit states before filling with the next cheapest candidates.
The two-candidate cap is per orientation, including the geometric tilt hint.

Transit cost still combines total travel, twice maximum per-joint travel, and the
quadratic excess-travel penalty (`cut_cost.preferred_joint_travel_rad=1.57`,
`cut_cost.excess_travel_weight=2.0`). The optional hard accumulated per-joint transit
budget `cut_cost.max_transit_joint_travel_rad` remains disabled at 0 and is never
relaxed. Unknown-edge estimates allow for endpoint tolerances; actual planned
costs replace them. The graph now indexes outgoing edges instead of scanning all
edges at every node.

## Validation

```bash
cd ~/ros2_ws
colcon build --packages-select hb_robot_skills hb_robot_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hb_robot_skills --ctest-args -R cut_graph_planner_test --output-on-failure
```

Regression tests also cover a cheap-approach dead end rescued by the beam, shared
stage budgets, endpoint deduplication and rejection of incomplete chains. They do
not prove top-flange success without the actual robot model and collision scene.
Other regression tests cover direct ownership, legacy returns, optional final return,
failed and discontinuous connections, alternative candidate pairs, position-bound
rejection, and transit cost/budget behavior. They use a small synthetic robot and
stub motion planners; they do not validate UR IK, hardware motion, or machining.
Before execution, inspect the full approval preview and compare planning time,
IK attempts, candidate counts, and transit travel on representative profiles.
This change does not fix the separate UR reverse-interface connection failure.
