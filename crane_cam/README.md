# InnoMaker USB camera from WSL

Plug the UVC camera into the laptop and leave it available to Windows. The WSL publisher starts Windows FFmpeg, captures DirectShow MJPEG, and timestamps each frame in WSL when it receives it. The detector uses the crane's `DICT_6X6_250` dictionary.

From Ubuntu with ROS 2 Humble and Windows `ffmpeg.exe` on the WSL path:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select crane_cam
source install/crane_cam/share/crane_cam/local_setup.bash
ffmpeg.exe -hide_banner -list_devices true -f dshow -i dummy
read -rp 'Windows camera name: ' camera
ffmpeg.exe -hide_banner -list_options true -f dshow -i "video=$camera"
ros2 launch crane_cam innomaker.launch.xml device:="$camera"
```

Select the external camera's exact name. The default capture request is 1920×1080 MJPEG at 30 fps. If the camera lists a different MJPEG mode, supply its `width:=`, `height:=`, and `fps:=` in the launch command. View `/innomaker/image_aruco` in `rqt_image_view` to see marker outlines and IDs.

## Metric poses

Use a monocular ROS calibration YAML for this lens, focus and capture resolution. To create one, keep the camera launch running and use the ROS `camera_calibration` package from a second sourced WSL terminal. For a checkerboard with **8×6 internal corners** and **25 mm squares**, run:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --no-service-check --ros-args -r image:=/innomaker/image_raw -r camera:=/innomaker
```

Adjust both board measurements to your board, collect varied angles across the image, then select **Calibrate** and **Save**. Extract `ost.yaml` from the saved calibration archive. Restart the camera launch with the YAML's absolute WSL path and the measured outer black square width in metres. All visible markers should have that size and distinct IDs.

```bash
ros2 launch crane_cam innomaker.launch.xml device:="$camera" \
  calibration:="$HOME/innomaker.yaml" marker_size_m:=0.10
```

Each visible marker publishes a transform from `innomaker_optical_frame` to `innomaker_aruco_<id>` with the image timestamp. Camera axes are x right, y down, z forward. To read marker 7, run `ros2 run tf2_ros tf2_echo innomaker_optical_frame innomaker_aruco_7`. The annotated image also shows the estimated axes.
