# Standalone hand–eye calibration for ROS 2 Humble

One `ament_python` package for Ubuntu 22.04 / ROS 2 Humble. It provides a
standalone Qt calibration window, headless ROS services, ArUco/ChArUco target
generation and detection, timestamped robot-pose sampling, five OpenCV hand-eye
solvers, saved datasets, and transform export.

This adapts the [MoveIt calibration workflow](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.rst).
It is a new standalone implementation, not a port of the original RViz panel.
MoveIt can be used to move the robot, but is not a dependency. The package never
commands robot motion. It works with any robot publishing the required TF and
any calibrated camera publishing `sensor_msgs/Image` plus `CameraInfo`.

**Validation status:** solver, target detection, file export and Qt tests were
run on Windows with OpenCV 4.5.5 and 4.6.0. A live Humble build, DDS integration
tests and hardware calibration have not been run in the development session.
The ROS tests and a Humble Docker build/test recipe are included. See
[validation details](docs/VALIDATION.md).

## 1. Build in your ROS workspace

Extract/copy this directory as `YOUR_WORKSPACE/src/handeye_calibration_ros2`.
Do not install the old ROS 1 MoveIt calibration repository for this package.
Use the system Python supplied with Humble; do not build from a Conda shell.

```bash
source /opt/ros/humble/setup.bash
cd YOUR_WORKSPACE
rosdep install --from-paths src/handeye_calibration_ros2 --ignore-src --rosdistro humble -y
colcon build --packages-select handeye_calibration_ros2 --symlink-install
source install/setup.bash
colcon test --packages-select handeye_calibration_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

If rosdep has never been initialized, first run `sudo rosdep init` and
`rosdep update`. Main dependencies resolve to Humble's `rclpy`, `tf2_ros`,
`cv_bridge`, messages/services, and Ubuntu's `python3-opencv`, `python3-numpy`,
`python3-yaml`, and `python3-pyqt5`. Ubuntu's OpenCV includes ArUco. Avoid replacing
its NumPy/OpenCV packages with pip wheels in the ROS Python environment; that can
break the binary ABI used by `cv_bridge`.

Optional isolated build/test on a Docker host:

```bash
cd src/handeye_calibration_ros2
docker build -t handeye-humble-test .
```

This builds the package and executes its tests, including ROS-specific tests.
The Dockerfile targets both amd64 and arm64 where the base image and apt packages
are available; it has not been run on a Jetson here.

## 2. Configure frames and camera topics

Start your robot driver / joint-state publisher / robot_state_publisher and
camera driver. Copy the provided configuration somewhere writable:

```bash
cp "$(ros2 pkg prefix --share handeye_calibration_ros2)/config/calibration.yaml" /tmp/handeye.yaml
```

Edit `/tmp/handeye.yaml`. Keep the top-level node key `handeye`. Important fields:

For your robot there is also `config/ur10e_azure_example.yaml`, based on the
checked-in link names and unprefixed driver topic/frame names. Verify these
against the running drivers before using that example.

| Parameter | Meaning |
|---|---|
| `mode` | `eye_in_hand` (camera on robot, target fixed in base) or `eye_to_hand` (camera fixed, target on robot) |
| `base_frame` | Robot reference frame in which the calibration target/camera stays fixed |
| `effector_frame` | Robot link rigidly attached to the moving camera/target, usually tool0 |
| `camera_frame` | **Optical frame exactly matching the Image and CameraInfo headers** |
| `mount_frame` | Optional camera housing/root frame, for exporting a replacement mount joint |
| `image_topic` | Uncompressed color or monochrome `sensor_msgs/Image` topic |
| `camera_info_topic` | Corresponding calibrated `CameraInfo` topic, same resolution and optical frame |
| `image_is_rectified` | `false`: use K+D; `true`: use P with zero distortion and account for R |
| `output_directory` | A dedicated directory for this calibration session; choose a new one per setup |
| `board` | Target type, dictionary, counts and **measured physical lengths in metres** |

All calibration parameters are read-only while running; edit the YAML and
restart to change them. This prevents mixing different models in one session.
Only static intrinsics are supported; do not change camera mode/resolution or
focus during collection. The node reuses CameraInfo only if its geometry and
frame match the image, and rejects intrinsic changes after sampling starts.

For your checked-in UR10e description, `ur10e_base_link` and `ur10e_tool0` are
the robot frames. Do **not** use `camera_visor` as `camera_frame` merely because
it is your inspection planning link. Camera measurements are expressed in the
image optical frame. Read the actual header:

```bash
ros2 topic list -t
ros2 topic echo --once /YOUR_RGB_CAMERA_INFO_TOPIC
ros2 run tf2_ros tf2_echo ur10e_base_link ur10e_tool0
```

The checked-in Azure Kinect driver publishes relative `rgb/image_raw` and
`rgb/camera_info` topics, but namespaces and frame prefixes depend on how you
launch it. Use the names actually reported by your running driver. If the driver
publishes JPEG `CompressedImage` only, enable its raw image output or use
`image_transport` republish first. This package subscribes to raw `Image`.

For a camera mounted on the tool, the robot's **base→tool** TF must be accurate;
no initial robot→camera extrinsic estimate is used for solving. Do not derive
that base→tool TF through the camera mount being calibrated. Both computers must
use aligned clocks. For simulation/bag replay set `use_sim_time:=true` and provide
`/clock` for all relevant nodes.

## 3. Print a target

The default is a 5×7 ChArUco board, 40 mm squares, 30 mm markers, dictionary
`DICT_5X5_250`. These counts refer to chessboard squares. For `kind: aruco`,
counts refer to markers and `marker_separation_m` is the white gap between them.

```bash
ros2 run handeye_calibration_ros2 handeye target \
  --config /tmp/handeye.yaml --output /tmp/handeye_target.png
```

A PNG and YAML sidecar are produced. Print without stretching, mount the target
flat and rigid, and **measure the actual printed dimensions**. Update the YAML
lengths if printing scaled the board. Image pixel size is not a guarantee of
physical print scale. The default board region is 200×280 mm, excluding margins;
use paper large enough or choose smaller physical squares. Never change the
pattern's dictionary/counts without printing the corresponding target.

The GUI's **Save target PNG** button generates the active configuration's board.
Use a target generated by the same OpenCV environment used for detection. Sessions
record the detector version because board coordinate conventions changed around
OpenCV 4.6. The board frame itself need not be surveyed in world coordinates.

## 4. Collect and solve

```bash
ros2 launch handeye_calibration_ros2 calibration.launch.py config:=/tmp/handeye.yaml
```

The window shows annotated images, capture readiness, samples and results.

1. For **eye-in-hand**, fix the target in the robot base frame and keep the camera
   rigidly attached to the tool. For **eye-to-hand**, fix the camera and mount the
   target rigidly on the moving tool.
2. Position the robot using MoveIt or its pendant so the target is well visible.
3. Stop the robot and wait for **Ready to capture**. Click **Capture sample**.
4. Repeat for roughly 15–25 varied poses. Include appreciable rotations about at
   least two different axes, as well as varied translations. Avoid collecting
   only almost-identical poses or rotating around just one axis.
5. Click **Solve**. The minimum is five poses; five alone is not a quality guarantee.
6. Inspect per-sample closure errors. **Undo last** removes a bad last sample;
   **Clear** starts over. To remove an arbitrary sample offline, edit a copy of
   `session.yaml` and re-solve that copy using the CLI.
7. Click **Export calibration** after the configured RMS thresholds pass.

Robot TF is interpolated at each image timestamp. There is no latest-TF fallback.
The node requires fresh observations and a stable interval, and rejects
near-duplicate poses. Missing targets reset the settling interval. Images use
sensor-data QoS so best-effort camera publishers are supported.

Each successful Capture/Undo/Clear atomically updates `session.yaml`. **Load
session** resumes the existing file after a valid current camera frame is seen
and its intrinsics/configuration are checked. Keep the physical target mounting
unchanged when resuming. Capture in a directory with an existing session replaces
that saved session unless you Load it first; use separate output directories for
separate calibrations. Exported calibration files remain the last explicitly
exported result until you solve and export again.

The default solver is Park. Set `solver` to `park`, `tsai`, `horaud`, `andreff`,
or `daniilidis` to compare methods against the **same** recorded dataset.
Quality checks use closure consistency, not absolute metrology accuracy. The
default export thresholds are 10 mm translation RMS and 2 degrees rotation RMS;
tighten them to your application after checking expected measurement noise.
No samples are silently removed or automatically classified as outliers.

## 5. Use the result

Outputs in `output_directory`:

- `session.yaml`: paired transforms, image timestamps, target reprojection errors,
  frame/board/intrinsics context.
- `calibration.yaml`: solved transform, method, diagnostics, per-sample residuals.
- `calibration.launch.py`: ROS 2 static transform publisher for a **new preview
  child**, `<camera_frame>_calibrated`.
- `mount_joint.xml`: optional fixed-joint snippet when `mount_frame` is configured
  and its rigid transform to the optical frame is known through TF.

Transform convention: `T_A_B` maps a point expressed in B into A.

| Mode | `parent_from_camera` result |
|---|---|
| Eye in hand | `T_effector_cameraOptical` |
| Eye to hand | `T_base_cameraOptical` |

Preview the optical transform without changing your camera's live TF tree:

```bash
ros2 launch /ABSOLUTE/SESSION/DIRECTORY/calibration.launch.py
```

For permanent integration, update the camera's existing fixed mounting joint.
If the robot mounts a housing frame `M` and the optical frame is `C`, then
`T_parent_M = T_parent_C * T_C_M`. Setting `mount_frame` enables this conversion
and writes the joint snippet. `T_C_M` must describe only the known rigid camera
assembly, not a transform that depends on the unknown robot mount.

In your existing description the mounting joint is `tool0_to_kinect` in
`hb_robot_description/urdf/hb_sensors_macro.xacro`, with child `camera_base`.
If your driver supplies the correct fixed optical↔camera_base transform, use
`mount_frame: camera_base`, then replace that joint's origin with the exported
values. Preserve the existing joint name and link structure as needed.

Do not add a second publisher/parent for the same existing camera link. Updating
the URDF mount also updates the geometry MoveIt uses. Publishing the optical
calibration as a new TF does not by itself correct your MoveIt robot model.

## Headless operation and offline validation

```bash
ros2 launch handeye_calibration_ros2 calibration.launch.py config:=/tmp/handeye.yaml gui:=false
ros2 service call /handeye/capture std_srvs/srv/Trigger '{}'
ros2 service call /handeye/undo std_srvs/srv/Trigger '{}'
ros2 service call /handeye/solve std_srvs/srv/Trigger '{}'
ros2 service call /handeye/save std_srvs/srv/Trigger '{}'
```

Additional Trigger services: `/handeye/load`, `/handeye/clear`. Each response
contains `success` and a diagnostic message. `/handeye/status` is JSON in
`std_msgs/String`; `/handeye/target_detection` is a debug Image;
`/handeye/target_pose` is the observed target pose in the physical optical frame.
The target pose is **not** a robot calibration result. To attach the GUI later:

```bash
ros2 run handeye_calibration_ros2 calibration_gui --backend /handeye
```

Offline solving and held-out validation:

```bash
ros2 run handeye_calibration_ros2 handeye solve /path/to/session.yaml \
  --method park --output-directory /path/to/offline_result
ros2 run handeye_calibration_ros2 handeye validate \
  /path/to/offline_result/calibration.yaml /path/to/held_out_session.yaml
```

Collect the held-out session in a different output directory, at new poses,
without moving the fixed target/camera or changing the target's tool attachment.
The validation command evaluates those samples against the original fitted
constant target transform; it does not refit the camera extrinsics or recenter
the residuals. It reports errors for review rather than deciding application
acceptance. Offline solving does not recreate an optional mount transform because
the saved dataset does not contain the intra-camera mount TF.

## Troubleshooting and limits

| Symptom | Check |
|---|---|
| No image | Exact raw Image topic/type, ROS domain/discovery, matching QoS |
| Header/frame mismatch | Use the Image optical `frame_id`, not the visor/housing frame |
| Stale image / waiting for TF | Clock alignment, `use_sim_time`, TF buffer and image latency |
| Never settles | Actual vibration, exposure/motion blur, pose-estimation noise; tune settling tolerances after inspecting images |
| Ambiguous planar pose | Tilt the board/camera, increase apparent target size, improve lighting |
| Solver rejects diversity | Add rotations about a second axis and a wider orientation range |
| High residuals | Wrong print dimensions/intrinsics, moving target, flexing mount, bad robot kinematics, timestamp errors |
| GUI unavailable on Jetson | Run headless services, or run the GUI on a ROS-connected desktop |

Only pinhole `plumb_bob`/`rational_polynomial` raw models and ordinary rectified
images are supported. Fisheye distortion, ROI/binning, translated stereo
projection matrices, camera intrinsic calibration, depth calibration, automated
robot motions and original RViz-plugin compatibility are outside this package.
Robot kinematics and camera intrinsics are assumed already calibrated.

See [NOTICE.md](NOTICE.md) for upstream references and implementation provenance.
