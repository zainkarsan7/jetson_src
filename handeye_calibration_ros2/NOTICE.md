# Origin and scope

This is an independently implemented ROS 2 Humble adaptation of the calibration
workflow described by the MoveIt hand-eye tutorial. It is not an official MoveIt
release, a source-to-source port of its RViz plugin, or binary-compatible with
`moveit_calibration`. It uses a standalone Qt window and standard ROS 2 services.
It does not contain the original repository's source files or require its
Python/crigroup solvers. OpenCV supplies the calibration algorithms and target
detector; those installed dependencies retain their own licenses.

References inspected during development:

- [MoveIt tutorial](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.rst)
- [Original ROS 1 implementation](https://github.com/moveit/moveit_calibration)
- [Andrej Orsula's ROS 2 port, PR 118](https://github.com/moveit/moveit_calibration/pull/118),
  inspected at commit `0263c8c8aa8f1b548bf152969900d74b3fbf9a75` in
  `AndrejOrsula/moveit2_calibration`, branch `ros2_port`.
- [OpenCV 4.5 calibration APIs](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html)
- [Humble TF buffer API source](https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/buffer.py)

The original tutorial and port establish the workflow; package code, tests,
session format, capture checks and GUI here are newly implemented. No claim of
hardware validation or official ROS distribution release is made.
