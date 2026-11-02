# WSL supervisor baseline

## Graphics correction, 2026-11-02

The user reported that GPU rendering produced a blank viewport, including the
grid. Loading D3D12 libraries and low CPU usage did not establish correct
rendering. The launcher now defaults to `LIBGL_ALWAYS_SOFTWARE=1` and restores
`QT_X11_NO_MITSHM=1`, matching the previous graphics fallback while retaining
Qt XCB and the lighter 15 FPS supervisor profile. GPU mode is an explicit
experiment (`software_rendering:=0`), not the verified default. Restart RViz
to apply these process environment settings. Visual confirmation is still needed.

The measurements below describe the previous GPU experiment, not the corrected
fallback's performance.

## Verified on the laptop, 2026-10-31

The previous WSL launcher forced `LIBGL_ALWAYS_SOFTWARE=1`. Its RViz process
had llvmpipe threads and reported about 415% CPU. WSL itself supports accelerated
OpenGL through D3D12 on the Intel Iris Xe GPU.

The initial launcher defaulted to GPU rendering and Qt XCB. The supervisor profile
limits rendering to 15 FPS and disables Image, DepthCloud, PointCloud2 and the
nvblox TSDF marker display. It preserves the existing review panels, trajectory
display, and other markers. This is a reduced-traffic baseline, not a bandwidth
firewall: other subscribers and planning-scene geometry can still carry data.

After restarting RViz, the process loaded D3D12 libraries, had no llvmpipe
threads, and reported approximately 6% CPU. These process-lifetime averages
are not a controlled benchmark; both graphics and display configuration changed.
The Jetson nodes were not discoverable at verification time, so this does not
establish camera or robot performance under load.

## Run in WSL

The new files have been copied into both the existing WSL source and install
directories. Previous launcher files have timestamped `.backup-*` copies beside
them. Future package builds install the source versions normally.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hb_robot_moveit_config wsl_moveit_rviz.launch.py
```

Close an existing RViz instance before launching another. The launcher starts
only RViz; it does not start hardware, MoveIt, or skill servers. It inherits
the shell's ROS domain and middleware settings, which must match the intended
robot deployment. The session inspected used domain 0 and rmw_fastrtps_cpp.

For a deliberate comparison with the full display configuration:

```bash
ros2 launch hb_robot_moveit_config wsl_moveit_rviz.launch.py \
  rviz_config:=$(ros2 pkg prefix hb_robot_moveit_config)/share/hb_robot_moveit_config/config/moveit.rviz
```

Do not use that full profile as the initial wireless baseline. Add visualization
streams one at a time while monitoring the Jetson. The frame-rate setting limits
rendering, not the rate of subscribed ROS messages.

## Verify the camera executable on the Jetson

The Windows source checkout's Kinect launch now names `azure_kinect_node_codex`.
This change has not been deployed to the Jetson. A stale executable can remain
installed after a CMake target is renamed; rebuilding does not necessarily remove
the previous executable. Another sourced overlay can also select another build.

Run in the same Jetson shell used to launch the camera:

```bash
ros2 pkg prefix azure_kinect_ros2_driver_codex
ros2 pkg executables azure_kinect_ros2_driver_codex
pgrep -af azure_kinect
```

For the actual camera PID, inspect `/proc/PID/exe` with `readlink -f`.
After deploying the corrected source, build the driver and bringup package in
the Jetson's existing workspace, then source that workspace's install setup.
Do not delete a whole install tree to remove one stale executable.

## Remaining validation

1. Confirm GUI interaction with the new profile.
2. Confirm Jetson ROS nodes, TF and `/get_planning_scene` are discoverable.
3. Keep UR control, camera acquisition and perception on the Jetson initially.
4. Add native `/k4a/rgb/image_raw/compressed` preview with best-effort QoS.
   Do not replace local perception's raw input with lossy preview data.
5. Add reduced-rate/downsampled scene visualization, measuring CPU, network load
   and the camera node's private diagnostics topic on the Jetson.
6. Compare wireless with a wired laptop link before relocating planning/skills.

Keep existing motion approval requirements. GUI responsiveness alone does not
validate UR control deadlines or resolve the previously observed reverse-interface
disconnect. No robot commands or motion approvals were issued during this work.
