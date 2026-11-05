# Recovery architecture

## Scope and sensor ownership

The implementation composes geometric recovery, process containment and separation
verification using the existing profile acquisition, cut geometry, process candidate
generator and cut graph planner. It does not introduce five independent servers.

The platform-fixed IMU is **/hb_robot_lidar/imu**, confirmed by the operator.
The robot-mounted **/k4a/imu** only determines when the camera has settled for RGB-D
acquisition. Camera movement must not invalidate the workpiece pose.
The platform_imu_topic launch argument is configurable.

MoveIt stop and UR SetIO are software process-containment requests, not replacements
for controller safety functions or guarantees of hardware emergency-stop latency.
No hardware motion or ignition was performed during development.
enable_cutting_process defaults to false. Motion-only execution never proves separation.

## Components and contracts

| Component | Responsibility |
| --- | --- |
| task_state_server | Authoritative geometry, pose, scene and execution epochs; platform supervision |
| inspect_profile_server | Full inspection; publishes an atomic reference after successful collision update and settling |
| revalidate_workpiece_server | Stored-cloud localization using RGBDAcquisition and ObservationRegistrar |
| cut_profile_server | Retained plans, recut attempts, execution, stop requests and Escape |
| inspect_cut_server | Existing view planning and RGB-D capture, plus versioned spatial evidence |
| bt_executor | Evaluation, outcome conditions and bounded recovery composition |

WorkpieceReference binds an estimate and its extracted cloud in the planning frame.
Intermediate estimates remain available for visualization but never restore validity.
TaskState extends the original phase interface with explicit validity and epochs.

The state server publishes every 50 ms. Clients deny execution if delivery is absent
for 500 ms. Platform disturbance or missing IMU data invalidates a previously valid
pose and advances its epoch once. Returning stable does not restore pose validity.

A new full reference advances geometry and pose. Relocalization preserves geometry
identity and advances pose. Environment changes advance scene; robot-state-only updates
do not. Full scene snapshots and ACM updates conservatively invalidate plans even when
they might be equivalent. This may cause unnecessary replanning.

State-manager restart begins a new timestamp-based epoch. A reference more than five
seconds old cannot establish validity on restart; a latched old reference can still
serve as registration input. Execution epochs advance before plan movement, preventing
evidence from an earlier attempt from validating a later attempt.

## ValidateKerf and evidence

CutInterval contains segment_id and local normalized Cartesian arc length [s0,s1].
This is neither beam extrusion distance nor joint-trajectory time. Original web end
clearance is applied before normalization and is not reapplied to a recut interval.

KerfEvidence identifies the cut, geometry, pose and execution epochs. Its required
feature manifest comes from the original planned segments, including the selected web
side. Each bin reports observed, open and residual fractions, quality, and whether
through-thickness evidence exists.

The pure evaluator in hb_robot_interfaces/recovery.hpp:

- Rejects stale identities/epochs, invalid registration, NaN/out-of-range scores,
  malformed intervals, duplicate names and overlapping bins.
- Requires every original feature to cover [0,1] without gaps.
- Lets a supported residual interval override otherwise open evidence.
- Keeps unknown, occluded and low-quality evidence inconclusive.
- Requires through-thickness evidence for every verified bin.

ValidateKerf stores its verdict in the BT's shared context. SUCCESS means evaluation
completed, not separation. KerfVerified, ResidualDetected and KerfInconclusive test
different outcomes. INVALID_OBSERVATION cannot accidentally trigger recutting.

### Optional geometric inference

enable_geometric_kerf_evidence defaults to false: raw RGB-D capture alone stays
inconclusive. Activation is explicit because the research model requires calibration.

When enabled, three depth rays across each bin are projected into the aligned image.
The original closed, straight-sided profile supplies the first material entry/exit
interval. A finite measured return beyond the back surface plus uncertainty supports
OPEN. A return whose entire uncertainty interval lies inside material supports RESIDUAL.
Missing depth, foreground occlusion, ambiguous boundary hits, curved profiles, or rays
leaving the transverse kerf slab remain UNKNOWN. Residual evidence is sticky across
viewpoints and cannot be averaged away by open observations.

| Parameter | Default | Meaning |
| --- | --- | --- |
| enable_geometric_kerf_evidence | false | Enable research inference |
| kerf_depth_uncertainty | 0.002 m | Combined sensor and registration error bound |
| kerf_half_width | 0.001 m | Permitted axial half-width of the kerf |
| kerf_bin_width | 0.002 m | Maximum longitudinal bin width |

Quality is a deterministic evidence gate, not a calibrated probability.
Verification is resolution limited: sub-bin bridges, depth multipath, model error and
uncertainty bounds require specimen validation. It does not certify mechanical release.
Conservative boundary ambiguity may leave an uncut face inconclusive.
A second inspection changes standoff from 0.30 m to 0.35 m; camera rolls remain selected
by the existing inspection planner.

## RevalidateWorkpiece

The action takes an expected pose epoch and requires a stored reference.

1. Wait for camera stability and acquire a strictly newer RGB-D observation.
2. Convert/transform with existing acquisition TF and crop to reference bounds +60 mm.
3. Downsample at 3 mm and use ObservationRegistrar.
4. Reject inadequate points, nonconvergence, fitness above 25e-6 m², translation above
   50 mm or rotation above 0.10 rad.
5. Restart registration from ±15 mm axial offsets. Comparably good solutions that remain
   displaced by over 5 mm report poorly_constrained.
6. Recheck platform/camera stability, heartbeat and expected epoch. Update existing
   collision objects with the rigid correction, then publish the estimate/reference.

The axial restart test is an ambiguity heuristic, not a full observability proof.
Uniform sections may correctly refuse localization and require distinctive geometry.
No nearest cluster is selected and no profile family is refitted.
AwaitGeometry waits for the authoritative state commit.
Failed revalidation does not silently fall through to full inspection of another object.

## Planning and executing recuts

PlanCut, PlanRecut, ExecuteCut and ExecuteRecut reuse the CutProfile action/server.

| Request | Behavior |
| --- | --- |
| Empty plan_id, execute=false | Generate and retain a plan |
| Nonempty plan_id, execute=true | Execute exactly that retained plan |
| parent_plan_id and intervals | Regenerate a recovery plan from original geometry |

Planning requires cut identity and matching geometry/pose/scene epochs.
Plans are in-memory and unavailable after restart. A trajectory slice is never executed.

Approach, process and retract are regenerated using CutPlanner, the existing process
candidate generator and CutGraphPlanner. Flange-only and web-only recovery cuts are
supported. The BT merges residual bins into one envelope per feature.

Old process-trajectory joint states add optional warm-start candidates. Their normalized
point index is only a seed heuristic, not the geometric definition of s.
Fresh candidates remain available. Every seeded path is solved and checked again;
the actual robot state remains the transit-planning reference. This uses the repository's
candidate/graph backend and does not introduce a new STOMP/CHOMP optimizer.

Immediately before execution the server checks task freshness and epochs, operator
approval, current joints against the stored start (0.02 rad), retained waypoints against
current collision scene/bounds, torch feedback when enabled, and prior recut attempts.
State is checked again after approval. Original planners supply full path feasibility;
the extra stored-plan check does not replace their continuous/path validation.

The plan and recut regions are consumed BEFORE movement. Abort/cancel cannot unconsume
them. Results report executed/interrupted, completed segment count, active segment and
execution epoch. Progress is segment-level, not an inferred precise cut location.

An interrupted process is not automatically resumed: the BT contains the fault and
exits unverified. Diagnosis, fresh geometry, evidence and a new request are required.
Reticking cannot execute an already consumed plan.

## Escape and fault handling

The cutting server's 20 ms callback independently requests OFF and MoveIt stop on stale
or invalid supervision. Torch faults use the same nonblocking containment path.
The first motion-stop request does not wait for IO feedback or a BT tick.

Escape shares the cutting server's torch and MoveGroup ownership. It requests stop,
waits for cutting execution to leave its mutex, then confirms OFF.
Automatic BT containment sets allow_retreat=false.

An explicit Escape request may enable bounded retreat: default 30 mm, maximum 100 mm.
It requires valid/fresh/stable pose, no motion fault and confirmed OFF.
The nearest original feature supplies an outward normal and two longitudinal variants.
Cartesian planning must reach the full target with collision checking enabled.
Supervision continues during retreat; a normal is never presumed collision-free.

The result separately reports status, confirmed OFF, stationary state and recommended
revalidation. Missing velocity feedback is not stationary confirmation.
A successful ROS action means the report is available, not necessarily RETRACTED.
Faults are not cleared by Escape.

The /motion_fault Bool input needs a controller-specific collision/protective-stop
adapter; the repository does not currently supply one. Connect it for controller-fault
experiments. Torch feedback and platform supervision already use their existing inputs.

## BT behavior and verification

The XML composes EnsureGeometryValid, ObserveAndEvaluate and EnsureSeparationVerified.
Only KerfVerified finishes successfully. Inconclusive evidence permits a bounded second
inspection; residual evidence permits one plan/execute/reinspect sequence.
Exhausted or faulted paths request Escape and exit unverified.

Tests cover stale evidence, tiny bridges, missing coverage, malformed bins, task
permission, interval overlap, ray classifications and profile intersections.
Mocked XML tests exercise verified, inconclusive, residual and interrupted execution
paths. These tests do not establish hardware stop latency or physical ICP accuracy.

Suggested experiments, with torch disabled first:

1. Move only the camera: settling changes, platform pose validity remains valid.
2. Disturb the platform: pose becomes invalid and stability alone cannot restore it.
3. Compare complete, bridge and occluded specimens at the configured bin resolution.
4. Replay a consumed plan: expect rejection.
5. Change scene/pose while approval is pending: expect rejection before movement.
6. Inject arc loss, controller fault and platform-signal loss independently; measure
   first OFF/stop requests, confirmed OFF, actual stationary state and result.
7. Interrupt a cut then move the workpiece: contain, revalidate, inspect and request
   a newly validated residual plan. No old trajectory resumes automatically.

Build/test in WSL:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    MAKEFLAGS=-j2 colcon build --packages-select hb_robot_interfaces hb_robot_skills hb_robot_task_planner hb_robot_bringup --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
    colcon test --packages-select hb_robot_skills hb_robot_task_planner --event-handlers console_direct+
    colcon test-result --verbose

Launch exposes platform_imu_topic and inspect_cut_preview_only.
Post-cut inspection remains preview-only by default; enable its motion deliberately.
Configure geometric evidence and uncertainty on the inspection server after calibration.

## Validation performed

The changed ROS interfaces, skills, BT executor and bringup package compile on ROS 2
Humble in this workspace. The selected regression run passed 40 assertions/tests:
9 recovery/evidence tests, 4 XML recovery-path tests, 5 existing inspection tests and
22 existing cut-graph tests. Launch syntax and git diff whitespace checks also passed.
These are software checks; physical registration accuracy, ray-model calibration,
controller-fault wiring and measured stop latency remain hardware experiments.
