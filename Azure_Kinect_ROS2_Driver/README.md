# Azure Kinect ROS2 Driver

This project is a ROS2 node which publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/) to the [Robot Operating System (ROS)](https://docs.ros.org/). Developers working with ROS2 can use this node to connect an Azure Kinect Developer Kit to an existing ROS2 installation.

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect DK. It supports Linux installations of ROS2.

This is a port of the ROS1 drivers:

- [github.com/matlabbe/Azure_Kinect_ROS_Driver](https://github.com/matlabbe/Azure_Kinect_ROS_Driver)
- [github.com/microsoft/Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color, depth and infrared Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream
- A TF2 model representing the extrinsic calibration of the camera

The camera is fully configurable using a variety of options which can be specified in ROS2 launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect Developer Kit hardware. It does not provide access to:

- Microphone array
- Body tracking

## Status

This is a basic port to ROS2 and not thoroughly tested at this point. Limited testing has been performed with 
ROS2 Humble on Ubuntu 22.04.

Community additions and fixes are welcome.

## Building

### Azure Kinect Sensor SDK

Follow the [installation instructions](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation) in the Azure Kinect Sensor SDK repo to install the sensor SDK for your platform.

### Compiling

Clone the repo into the `src` directory of your [ROS2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

`
colcon build --symlink-install --packages-select azure_kinect_ros2_driver
`

## Running
Source your workspace: `source <ROS2 ws>/install/setup.bash`

`ros2 run azure_kinect_ros2_driver azure_kinect_node`

or from a file recorded via `k4arecorder`:

`ros2 run azure_kinect_ros2_driver azure_kinect_node --ros-args -p recording_file:=/path/to/myrecording.mkv`

A simple Python-based subscriber is included to visualize some of the data being published. An `image_proc` node is used
to undistort the images. Install via `sudo apt install ros-<$ROS2_DISTRO>-image-pipeline`

Launch the nodes:

`ros2 launch azure_kinect_ros2_driver k4a_test_record_launch.py`

## License

[MIT License](LICENSE)
