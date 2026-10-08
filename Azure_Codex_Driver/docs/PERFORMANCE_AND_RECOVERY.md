# Capture performance and recovery patch

The supplied launch file `hb_robot_bringup/launch/kinect.launch.xml` uses
720p color at 30 Hz, BGRA output, 50 Hz IMU output, and no point clouds.
The original code already used best-effort, volatile QoS with depth 1 and
subscriber checks. Those settings alone do not address the failure in the log.

## What changed

- **Decode outside the SDK capture callback.** With `color_format: bgra` and
  `driver_color_decode: true` (the default), the driver requests the camera's
  MJPEG stream from the SDK. A separate worker uses TurboJPEG to decode directly
  into a reusable BGRA buffer, only when a consumer needs color pixels. ROS
  topics, BGRA encoding, image dimensions, and frame rate configuration remain
  unchanged. There is no JPEG re-encoding and no GPU dependency. Frames replaced
  before processing are never decoded. This bypasses the SDK's
  `DecodeMJPEGtoBGRA32()` path implicated by the supplied log.
- **Bound backlog.** A dedicated capture thread drains the SDK into a single
  pending slot. If processing falls behind, it replaces the pending capture.
  One capture can be processing while one waits. Live acquisition no longer
  waits for publication or an extra frame-rate sleep.
- **Keep depth useful when RGB is bad.** Native depth and uncolored clouds are
  published before optional color processing. A failed JPEG decode drops that
  capture's color output and colored cloud, retaining its depth/IR outputs.
  It does not reuse the previous image's pixels or timestamp.
- **Reduce memory traffic and duplicated work.** Depth conversion writes metres
  directly into the ROS `32FC1` message, removing one full float-image allocation
  and copy. SDK row stride is respected. Each registration direction is computed
  at most once per processed capture, shared between registered image and cloud
  outputs. ROS messages keep their own pixel storage; published buffers are not
  mutated by the next capture. Calibration templates are cached.
- **Recover boundedly.** Capture reads use 100 ms polls. An SDK exception, an IMU
  read failure, or an extended absence of captures triggers coordinated camera
  and IMU stop/start, with backoff. Pending captures are cleared; captures from an
  earlier stream generation are rejected before further publication. Repeated
  failure stops workers and leaves the node alive to publish ERROR diagnostics.
- **Fix lifecycle and correctness defects.** Initialized atomic run state;
  guarded joins after partial startup; exception boundaries on worker threads;
  protected timestamp/debug state; no logging through a reset node pointer;
  allocated JPEG messages; SDK `CUSTOM` point-cloud buffers; point-cloud iterators
  constructed after sizing; invalid depth/cloud configurations rejected.
- **Build optimization.** Unspecified single-configuration builds default to
  Release. Explicit Debug/RelWithDebInfo selections are respected. The recording
  library uses its exported CMake target.

This is still CPU JPEG decoding. It does not repair USB corruption, firmware
failures, or an SDK call that hangs internally. Stream recovery uses the existing
device handle: physical unplug/replug may require restarting the node. SDK
synchronized captures still couple capture availability to both enabled cameras.
The worker keeps draining captures under ROS load, but CPU starvation can still
affect the SDK's own threads. No Jetson throughput improvement is claimed without
measurement on that device.

## Build on the Jetson

Keep the existing ARM64 Kinect SDK installation. The x86 libraries used for local
validation are not part of the source archive and must not be copied to the Jetson.

```bash
sudo apt-get install libturbojpeg0-dev ros-humble-diagnostic-msgs
source /opt/ros/humble/setup.bash
cd ~/ros2_pac_ws
colcon build --symlink-install --packages-select azure_kinect_ros2_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Replace the driver package's sources with the supplied package, including the
new headers and `test` directory; copying only `k4a_ros_device.cpp` is insufficient.
Stop the old camera node before launching the rebuilt one. Existing launch files
pick up the new default pipeline without changing resolution or ROS encoding.

## Parameters

New parameters are read-only after startup. Existing sensor configuration also
requires a node restart to take effect reliably.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `driver_color_decode` | `true` | Decode live BGRA output in the driver worker; `false` restores SDK BGRA decoding for comparison. JPEG output and playback keep their existing format paths. |
| `capture_timeout_ms` | `1000` | Continuous absence of captures before recovery; minimum 100 ms. Allow more time for externally triggered/subordinate setups if needed. |
| `recovery_max_attempts` | `3` | Maximum start attempts within recovery, and maximum successive unhealthy stream restarts; 0 disables recovery. The unhealthy-stream counter resets after one configured second's worth of successful captures. |
| `recovery_backoff_ms` | `500` | Base delay between restart attempts, multiplied by attempt number within recovery. |
| `max_capture_age_ms` | `250` | Reject old live captures/output before publication; 0 disables. Compared with the SDK host monotonic arrival timestamp, not a calibrated physical exposure-to-actuation latency. Playback is exempt. |

For example, these can be added inside the existing Kinect launch `<node>`:

```xml
<param name="driver_color_decode" value="true"/>
<param name="capture_timeout_ms" value="1000"/>
<param name="recovery_max_attempts" value="3"/>
<param name="recovery_backoff_ms" value="500"/>
<param name="max_capture_age_ms" value="250"/>
```

The age threshold is a configurable drop policy, not a collision-detection
deadline guarantee. Downstream consumers must also reject missing/stale messages:
DDS queues, transport, and downstream computation occur after the driver's gate.
Native depth is prioritized within each processing cycle; slow RGB processing can
still reduce the cadence of subsequent cycles. There is no independent depth
worker or hard real-time scheduling in this patch.

## Diagnostics and comparison

The node publishes `diagnostic_msgs/DiagnosticArray` at 1 Hz on `~/diagnostics`,
normally `/k4a_ros2_node/diagnostics`:

```bash
ros2 topic echo /k4a_ros2_node/diagnostics
```

Counters include captures received, overwritten pending captures, stale-output
drops, decode errors, processing errors, capture failures, and stream restarts.
Timing values include the last processing duration, the last checked SDK-host
frame age, and time since the last received/processed capture. Stale-output drops
can count multiple suppressed outputs of one capture. Replaced captures measure
driver backlog pressure, not USB/SDK-internal losses. OK describes capture and
processing activity; it is not a per-topic freshness guarantee.

Terminal failures remain visible as ERROR while the process is alive. A supervisor
that only checks process exit will not restart it automatically; it must monitor
diagnostics or message freshness. The patch does not add an infinite respawn loop.

Compare the original driver and this build with identical camera modes, consumers,
and CPU/power settings. Suggested runs:

1. Depth-only subscribers while keeping color enabled. Confirm BGRA decoding is
   skipped until a raw/registered RGB or colored-cloud subscriber connects.
2. The actual crane workload, then add the dashboard/recording consumers that
   previously triggered the failure. Record per-process CPU, RSS, received rates,
   message age percentiles, drops, and recovery events for at least 10 minutes.
3. Bounded additional CPU load in a stationary test. Check that backlog becomes
   frame replacement rather than an increasing message-age tail.
4. Registered RGB plus colored clouds, checking geometry against the original
   SDK transformation output. Shared transforms should reduce duplicated work.
5. Camera disconnect and shutdown tests. Check explicit loss-of-data indication,
   bounded recovery attempts, no stale frame replay, and clean Ctrl-C.

Keep the RGB/depth acquisition settings fixed during comparison. Report exact
Jetson model, JetPack/L4T and SDK versions with results; they determine whether a
later hardware JPEG decoder implementation is appropriate.

## Automated validation

```bash
colcon test --packages-select azure_kinect_ros2_driver \
  --ctest-args -R '^test_' --output-on-failure
colcon test-result --verbose
```

The tests cover latest-frame replacement/resource release, queue wakeup and
shutdown, concurrent overload, real TurboJPEG decoding, corrupt input, image
metadata, and buffer reuse. A Linux-only test interposes a simulated SDK device
and runs the real ROS node through capture failure/recovery, bad JPEGs, timeout,
stale frames, demand-driven decoding, compressed publication, and partial startup.
The fake SDK library is test-only and is never installed. It bypasses physical
device I/O and SDK registration, so these tests do not validate Jetson USB timing,
hardware recovery, calibration accuracy, or actual camera throughput.
