from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hb_robot").to_moveit_configs()
    return LaunchDescription([
        Node(package='zain_pkg',
             executable='example_1',
            #  prefix=['gdbserver localhost:3000'],
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics]),
       
    ])