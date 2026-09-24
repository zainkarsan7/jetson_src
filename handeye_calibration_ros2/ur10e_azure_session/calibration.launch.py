"""Calibration preview. Integrate the physical mount into your URDF separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('child_frame', default_value='rgb_camera_link_calibrated'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='handeye_calibrated_camera',
             arguments=['--x', '-0.05800833689554896', '--y', '0.034248384111423315', '--z', '0.17167886376745778', '--qx', '-0.2722726343821056', '--qy', '-0.27059146004150725', '--qz', '-0.6468726811846693', '--qw', '0.6589412786100068', '--frame-id', 'ur10e_tool0'] + ['--child-frame-id', LaunchConfiguration('child_frame')]),
    ])
