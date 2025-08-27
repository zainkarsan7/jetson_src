from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hb_robot", package_name="hb_robot_moveit_config").robot_description(file_path="config/hb_robot.urdf.xacro").robot_description_semantic(file_path="config/hb_robot.srdf").robot_description_kinematics(file_path="config/kinematics.yaml").planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)
