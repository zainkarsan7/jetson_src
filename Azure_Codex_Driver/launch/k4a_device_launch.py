import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    defaults = os.path.join(
        get_package_share_directory('azure_kinect_ros2_driver_codex'), 'config', 'codex.yaml')
    ld = LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=defaults),
        DeclareLaunchArgument('namespace', default_value=''),
    ])


    k4a_node = Node(
        package="azure_kinect_ros2_driver_codex",
        executable="azure_kinect_node_codex",
        name="k4a_ros2_node_codex",
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        output="screen",
        emulate_tty=True
    )



    ld.add_action(k4a_node)
    return ld
