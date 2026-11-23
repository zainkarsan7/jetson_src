import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('hb_robot_hardware'),
        'config',
        'hardware.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'platform_port',
            default_value='/dev/ttyACM0',
        ),
        DeclareLaunchArgument(
            'torch_height_port',
            default_value='/dev/ttyACM2',
        ),

        Node(
            package='hb_robot_hardware',
            executable='platform_bridge',
            name='platform_bridge',
            output='screen',
            parameters=[
                cfg,
                {'port': LaunchConfiguration('platform_port')},
            ],
        ),

        Node(
            package='hb_robot_hardware',
            executable='torch_height_bridge',
            name='torch_height_bridge',
            output='screen',
            parameters=[
                cfg,
                {'port': LaunchConfiguration('torch_height_port')},
            ],
        ),
    ])