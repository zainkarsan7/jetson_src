# Close cut inspection: acquisition stage

`inspect_cut_server` implements `hb_robot_interfaces/action/InspectCut`. Its first
stage acquires a dense post-cut band around a known transverse cut. It does not
classify material or claim separation. `success` means all required stations
were captured; `interpretation=NOT_EVALUATED`, `cut_complete=false` and
`confidence=0` are intentional, including on successful acquisition.

## Geometry and motion

Supply a `ProfileEstimate` for the actual cut plane, not an arbitrary fitted
section elsewhere along the member. Its pose maps profile XY into the planning
frame; local Z is the extrusion axis. Supply a nonempty `cut_id` for provenance.
The server requires that estimate's frame to equal its configured planning frame.

The planner shares `ProfileLibrary::cut_features` with `CutPlanner`. It samples
both flange spans and both web faces, including endpoints, at no more than
`sample_spacing` (default 25 mm). Unlike the cutting planner it does not discard
one web face as an alternative cut. Each station has three optical camera roll
candidates, with +Z looking toward the feature and a default 300 mm standoff.
The camera standoff must be calibrated to the sensor's useful near-depth range.
It is not the torch standoff.

The existing `CutCandidateGraph` provides layered route search. IK candidates
are joined by lazily planned MoveIt transitions; failed transitions are removed
and the route is searched again. Every station is mandatory. Collision checks
use MoveIt's current scene and retain the workpiece collision geometry. The
whole route is planned before execution. This does not assume torch reachability
implies camera reachability or optical visibility. It does not command the torch.

`preview_only=true` by default publishes nominal viewpoints without moving or
acquiring and returns a non-success result explaining preview mode. To execute,
set `preview_only=false`. Load the same robot description, semantic description,
kinematics and planning configuration as the other skills, with MoveIt running
and the workpiece collision object installed. `camera_link` must name the RGB
optical frame in the robot model and RGB headers (default
`camera_rgb_optical_frame`); configure it to the actual installed camera.

## Acquisition and outputs

At each station: execute the planned transition, reset and wait for a fresh IMU
stability window, then collect strictly newer synchronized RGB-D samples (three
by default). It rejects stale depth, RGB/depth skew over 30 ms, invalid intrinsics,
mismatched dimensions, optical frame mismatch, and camera poses differing from
the planned pose by more than 20 mm or 0.05 rad. IMU stability is checked again
at every sample. Cancellation stops MoveIt and returns partial evidence.

Aligned depth supports `16UC1` millimeters and `32FC1` meters; color is converted
to RGB. Every finite valid depth pixel is transformed with its timestamped TF.
Only points within `kerf_half_band` (default +/-15 mm) along cut Z and the nominal
profile bounding box plus `profile_margin` (20 mm) are accumulated. Axial depth
is preserved. There is no 5 mm voxel filter or statistical edge removal, and no
pre-cut cloud is merged into this cloud. Duplicate observations are deliberately
retained for later evidence fusion. The band includes nearby background and is
not yet a material mask. The five-million-point budget aborts with partial data
rather than silently dropping detail; reduce the band or samples when needed.

Outputs:

- `/inspect_cut_server/viewpoints`: nominal station poses for preview.
- `/inspect_cut_server/kerf_cloud`: accumulated colored band in the planning frame,
  published after each station and on completion/failure; cleared per goal.
- `/inspect_cut_server/observations`: `KerfObservation` records containing cut ID,
  feature, station index, cut pose, original RGB/depth/intrinsics and camera pose.
  Record this topic in a rosbag for replay and future free-space inference.
- Action feedback: station/feature, measured camera pose and accepted sample count.
- Action result: partial/full cloud, requested and completed station counts, and
  acquisition status. No points in the band is acquisition failure, not proof of
  an open cut. Station completion is not material evidence coverage.

The new goal replaces the unused placeholder `viewpoints`/`max_ang_vel` goal.
Rebuild clients with the interfaces. There is no existing behavior-tree
`InspectCut` client to migrate. This executable is installed by `hb_robot_skills`;
robot-specific launch integration should supply the same MoveIt parameters as
`inspect_profile_server`.

## Stage two: inference and telemetry

This stage intentionally uses TF registration only (`registration_refined=false`).
It does not use `SceneModel`'s ICP or its coarse filtering. Before free-space
inference, validate registration accuracy against unchanged material outside the
cut band. If introducing ICP, apply the accepted correction to BOTH point clouds
and stored camera poses; registration residuals and uncertainty must be retained.

The 2.5D evidence model can index nominal material cells in cut XY while retaining
measured axial depth and viewing rays. Keep OPEN, OCCUPIED, UNKNOWN and UNRELIABLE
separate. Missing depth is UNKNOWN, not OPEN. Occluded web/flange interiors cannot
be declared severed just because all stations were visited. The initial camera
poses do not yet include ray visibility scoring or inner-flange-face coverage;
add complementary views when the evidence model identifies unresolved cells.

Record `/cut_profile/torch_status` during the preceding cut, along with robot
joint states/TF, cut identity and executed segment timing. The existing
`TorchStatus` has timestamps, segment name, transfer, voltage, I/O validity and
faults, but no cut ID or executed path coordinate. Bind it explicitly to the
cut execution before fusing it: never associate the latest live torch sample
with a previous cut implicitly. Transfer dropout, abnormal voltage or speed
can motivate bridging hypotheses and denser resampling; they are not visual
proof of a bridge. Telemetry association and hypothesis inference are stage two,
not implemented by this acquisition server.

## Validation

`cut_inspection_test` checks complete feature/end-point sampling, optical axes
under a rotated/translated cut frame, invalid/budgeted requests, band boundaries,
and millimeter depth/color conversion. Run this and the existing graph tests:

```sh
colcon test --packages-select hb_robot_skills --ctest-args -R 'cut_(inspection|graph_planner)_test'
colcon test-result --verbose
```

Hardware commissioning still needs collision-scene validation, sensor near-range
and calibration checks, recorded-data replay and comparison with known cut
specimens. The unit tests do not establish physical coverage or separation.
