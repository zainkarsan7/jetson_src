from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    calibration_yaml = os.path.join(get_package_share_directory("hb_robot_description"), "urdf","my_robot_calibration.yaml")
    with open(calibration_yaml,'r') as f:
        calib = yaml.safe_load(f)
    
    kin_hash = calib["kinematics"]["hash"]

    control_package = FindPackageShare("hb_robot_control")
    
    control_file = PathJoinSubstitution(
        [control_package, "urdf", "hb_robot_controlled.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([FindPackageShare("hb_robot_description"), "rviz", "urdf.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", control_file, " ", "ur_type:=", ur_type," ", "kinematics_hash:=",kin_hash]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10e",
        )
    ]

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )