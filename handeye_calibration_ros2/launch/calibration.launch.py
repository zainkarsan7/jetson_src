from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('handeye_calibration_ros2'), 'config', 'calibration.yaml'])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='handeye_calibration_ros2', executable='calibration_node', name='handeye',
             output='screen', parameters=[LaunchConfiguration('config'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='handeye_calibration_ros2', executable='calibration_gui', name='handeye_gui',
             output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
    ])
