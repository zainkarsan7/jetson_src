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
    ])
    # .sensors_3d(file_path="config/sensors_3d.yaml")
    .to_moveit_configs()
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

    inspect_profile_server= Node(
        package= "hb_robot_skills",
        executable = "inspect_profile_server",
        name = "inspect_profile_server",
        output="screen",
        # prefix = ["gdb -ex run --args"],
        parameters = [moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {"skip_inspection_motion": False,
        "require_plan_approval": True },
        ],
    )

    cut_profile_server= Node(
        package= "hb_robot_skills",
        executable = "cut_profile_server",
        name = "cut_profile_server",
        output="screen",
        # prefix = ["gdb -ex run --args"],
        # parameters = [moveit_config.robot_description,
        # moveit_config.robot_description_semantic,
        # moveit_config.robot_description_kinematics,
        parameters = [moveit_config.to_dict(),{"skip_cutting_motion": False,
        "require_plan_approval": True,
        # Geometric seed + reference seed + bounded local perturbations;
        # only the final two attempts per roll explore globally.
        "cut_sampling.roll_samples": 12,
        "cut_sampling.seeds_per_roll": 16,
        "cut_sampling.global_seeds_per_roll": 2,
        "cut_sampling.local_seed_radius_rad": 0.35,
        "cut_sampling.local_ik_consistency_rad": 0.75,
        "cut_sampling.max_candidates": 32,
        "cut_cost.transit_weight": 1.0,
        "cut_cost.process_weight": 1.0,
        "cut_cost.preferred_joint_travel_rad": 1.57,
        "cut_cost.excess_travel_weight": 2.0,
        # Optional hard limit on accumulated travel of any joint per transit.
        # Zero keeps large but necessary transits available at a higher cost.
        "cut_cost.max_transit_joint_travel_rad": 0.0,
        },]
        ,
    )
    return LaunchDescription([
        inspect_scene_server,
        cut_profile_server,
        inspect_profile_server
    ])



