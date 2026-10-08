# Validation record

Validated on 2026-10-08 using Ubuntu 22.04.5 under WSL, GCC 11.4,
ROS 2 Humble (rclcpp 16.0.10), Microsoft Kinect SDK 1.4.2 x86_64,
and Ubuntu TurboJPEG 2.1.2. The SDK and JPEG packages were extracted
under `.validation` for the build, not installed system-wide.

- Release build of the complete driver: passed.
- Latest-capture queue: 4 Google Tests passed (replacement and resource lifetime,
  shutdown wakeup, recovery clear, concurrent producer/consumer overload).
- JPEG decoder: 3 Google Tests passed (valid pixels and metadata,
  dimension validation, corrupt frame followed by successful buffer reuse).
- ROS node with simulated SDK I/O: 8 scenarios passed (recovery, corrupt JPEG,
  persistent capture failure, timeout, stale captures, no color demand, JPEG
  publication, startup failure). Each running-node scenario also checked clean
  SIGINT shutdown. The test verified depth remained metre-valued `32FC1` and raw
  color remained `bgra8`.
- Focused C++ static analysis: passed with warning/performance/portability checks;
  missing external includes and the existing constructor-initializer style
  suggestion were suppressed. This does not replace compiler or hardware tests.
- Python integration-test syntax compilation: passed.

Final CTest run: 3/3 test targets passed in 30.73 seconds. This covers 7 unit
cases and 8 simulated-node scenarios. Repository-wide formatting/lint checks
were not included in that run.

Not validated: ARM64 compilation, the real Azure Kinect, Jetson CPU/USB behavior,
hardware disconnect/reconnect, actual SDK registration geometry, and recorded
file playback. The new threaded handoff applies only to live acquisition;
playback keeps its sequential, rate-limited capture path. Follow the on-device
comparison procedure in `PERFORMANCE_AND_RECOVERY.md` before evaluating the
patch as a collision-detection or mapping input.
