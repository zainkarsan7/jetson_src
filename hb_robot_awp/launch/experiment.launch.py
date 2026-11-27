from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('cut', 'generic'):
        raise ValueError('mode must be cut or generic')
    target = 'cut_pose_experiment' if mode == 'cut' else 'awp_excitation_server'
    share = Path(get_package_share_directory('hb_robot_awp'))
    moveit = (MoveItConfigsBuilder('hb_robot', package_name='hb_robot_moveit_config')
              .robot_description(file_path='config/hb_robot.urdf.xacro')
              .robot_description_semantic(file_path='config/hb_robot.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .to_moveit_configs())
    config = share/'config'/('cut_pose_experiment.yaml' if mode == 'cut' else 'awp_excitation.yaml')
    return [Node(package='hb_robot_awp', executable=target, output='screen', parameters=[
        moveit.robot_description, moveit.robot_description_semantic, moveit.robot_description_kinematics,
        str(config), {'execute_motion': LaunchConfiguration('execute_motion').perform(context).lower() == 'true'}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='cut'),
                              DeclareLaunchArgument('execute_motion', default_value='false'),
                              OpaqueFunction(function=setup)])
