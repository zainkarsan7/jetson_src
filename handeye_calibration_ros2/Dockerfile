FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive QT_QPA_PLATFORM=offscreen ROS_LOCALHOST_ONLY=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-pyqt5 python3-opencv python3-numpy python3-yaml \
    ros-humble-cv-bridge ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . src/handeye_calibration_ros2
RUN source /opt/ros/humble/setup.bash && \
    rosdep update --rosdistro humble && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -y && \
    colcon build --packages-select handeye_calibration_ros2 && \
    source install/setup.bash && \
    python3 -c "import rclpy, cv2; assert hasattr(cv2, 'aruco')" && \
    colcon test --packages-select handeye_calibration_ros2 --event-handlers console_direct+ && \
    colcon test-result --verbose
CMD ["bash"]
