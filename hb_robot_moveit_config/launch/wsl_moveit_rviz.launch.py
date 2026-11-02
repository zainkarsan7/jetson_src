"""Laptop supervisor with a compatible WSL graphics fallback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("hb_robot", package_name="hb_robot_moveit_config")
        .robot_description(file_path="config/hb_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/hb_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(config.package_path / "config/supervisor.rviz"),
        ),
        DeclareLaunchArgument(
            "software_rendering", default_value="1",
            description="Software rendering is the working WSL fallback; 0 tests GPU rendering.",
        ),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", LaunchConfiguration("software_rendering")),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                config.planning_pipelines,
                {"use_sim_time": False},
            ],
        ),
    ])
