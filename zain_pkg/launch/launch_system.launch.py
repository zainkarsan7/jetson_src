from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction, ExecuteProcess
def get_launch(context):

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hb_robot_control"),"launch","start_hb_robot.launch.py"]
            )
        ),
            launch_arguments={"launch_rviz": 'True','use_fake_hardware': 'true'}.items()
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("hb_robot_moveit_config"),"launch","move_group.launch.py"])
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("hb_robot_moveit_config"),"launch","moveit_rviz.launch.py"])
        )
    )

    

    activate_sjtc = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
                    cmd=["ros2", "control", "set_controller_state", 
                         "scaled_joint_trajectory_controller","active"],
                    shell=False
                )])
    
    nodes = [control_launch,move_group,rviz,activate_sjtc]

    if LaunchConfiguration('include_sensors').perform(context).lower() in ['true','1']:
        k4a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("azure_kinect_ros2_driver"),"launch","k4a_device_launch.py"]
            )
            )
        )
    
        lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("unitree_lidar_ros2"),"launch.py"]
                )
            )
        )
        nodes.append(lidar)
        nodes.append(k4a)

    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(
        'include_sensors',
        default_value='false',
        description='launch the lidar or k4a sensors'
    ),OpaqueFunction(function=get_launch)])

    


