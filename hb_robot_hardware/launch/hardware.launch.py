from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('hb_robot_hardware'), 'config', 'hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM1'),
        Node(package='hb_robot_hardware', executable='nano_bridge', name='nano_bridge',
             output='screen', parameters=[cfg, {'port': LaunchConfiguration('port')}])
    ])
