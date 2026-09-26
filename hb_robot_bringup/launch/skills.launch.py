from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config=  (MoveItConfigsBuilder(
        "hb_robot",
        package_name="hb_robot_moveit_config"
    ).robot_description(file_path="config/hb_robot.urdf.xacro")
    .robot_description_semantic(file_path="config/hb_robot.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .planning_pipelines(pipelines=[
        "ompl","chomp","pilz_industrial_motion_planner",
    ]).to_moveit_configs()
    )
    inspect_scene_server= Node(
        package= "hb_robot_skills",
        executable = "inspect_scene_server",
        name = "inspect_scene_server",
        output="screen",
        # prefix = ["gdb -ex run --args"],
        parameters = [moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {"skip_inspection_motion": False},
        ],
    )
    return LaunchDescription([
        inspect_scene_server,
    ])



