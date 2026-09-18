# Validation record

Development host: Windows, Python 3.11.4. ROS 2, WSL and Docker execution were
not available to this task. No robot was connected or commanded.

## Executed checks

- Synthetic ground-truth recovery for both eye-in-hand and eye-to-hand using
  all five OpenCV methods, including transform-direction checks.
- Noisy samples and held-out closure validation; inverted extrinsics are rejected
  by the test's error criterion.
- Rejection of insufficient/single-axis excitation and invalid transforms.
- Capture checks for stale data, movement, duplicate poses and observation gaps.
- Synthetic image rendering, detection and pose recovery for ArUco and ChArUco.
- Raw versus rectified CameraInfo handling and invalid-model rejection.
- Session roundtrip, context mismatch, atomic-write failure preservation.
- Generated launch syntax, calibrated preview child frame, and mount-joint
  translation/RPY conversion.
- Qt window state transitions and Capture button dispatch offscreen.

The final suite passed **26 tests** on OpenCV **4.5.5** and **4.6.0**;
the ROS-specific test module was skipped because `rclpy` is unavailable here.
The final test adds the complete offline target → solve → held-out validation
workflow. Python source distribution and wheel builds also completed locally;
these are packaging checks, not a ROS workspace build.

## Included but not executed here

`test/test_ros_adapter.py` checks real ROS message conversion, TF interpolation
at the image timestamp, capture invalidation on target loss, solve/export
service callbacks and a Trigger call through the ROS graph. Run these with
`colcon test` in Humble. The Dockerfile runs build/test in a Humble Jammy image.

Hardware acceptance still requires varied real samples, held-out validation,
inspection of the exported TF/URDF mount and confirmation of application-specific
accuracy. Low closure residuals alone do not establish absolute accuracy.
