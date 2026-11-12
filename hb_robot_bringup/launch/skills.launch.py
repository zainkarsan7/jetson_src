from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
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
    # inspect_scene_server= Node(
    #     package= "hb_robot_skills",
    #     executable = "inspect_scene_server",
    #     name = "inspect_scene_server",
    #     output="screen",
    #     # prefix = ["gdb -ex run --args"],
    #     parameters = [moveit_config.robot_description,
    #     moveit_config.robot_description_semantic,
    #     moveit_config.robot_description_kinematics,
    #     {"skip_inspection_motion": False},
    #     ],
    # )

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
        "platform_imu_topic": LaunchConfiguration("platform_imu_topic"),
        "platform_accel_topic": ParameterValue(LaunchConfiguration("platform_accel_topic"), value_type=str),
        "require_plan_approval": True,
        "exploration_weight_": 100.0,
        "motion_weight_": 0.001,
        "inspection_pos_tol": .03,
        "inspection_ang_tol": 0.1     
        
         },
        ],
    )

    cut_profile_server= Node(
        package= "hb_robot_skills",
        executable = "cut_profile_server",
        name = "cut_profile_server",
        output="screen",
        ros_arguments=[
        "--log-level",
        "move_group_interface:=warn",
    ],
        # prefix = ["gdb -ex run --args"],
        # parameters = [moveit_config.robot_description,
        # moveit_config.robot_description_semantic,
        # moveit_config.robot_description_kinematics,
        parameters = [moveit_config.to_dict(),{
        "skip_cutting_motion": False,
        "enable_cutting_process": False,
        "require_plan_approval": True,
        "transit.velocity_scaling": 0.05,
        "transit.acceleration_scaling": 0.01,
        "cut_transit.return_home_between_cuts": ParameterValue(LaunchConfiguration("return_home_between_cuts"), value_type=bool),
        "cut_transit.return_home_after_last_cut": ParameterValue(LaunchConfiguration("return_home_after_last_cut"), value_type=bool),
        "cut_geometry.max_pose_samples": 64,
        "cut_geometry.feasible_samples": 8,
        "cut_geometry.refinement_iterations": 8,
        "cut_geometry.ik_timeout_s": 0.05,
        "cut_geometry.waypoint_beam_width": 3,
        "cut_geometry.explore_ik_branches": True,
        "cut_sampling.max_candidates_per_roll": 3,
        # Geometric seed + reference seed + bounded local perturbations;
        # only the final two attempts per roll explore globally.
        "cut_sampling.roll_samples": 8,
        "cut_sampling.seeds_per_roll": 12,
        "cut_sampling.global_seeds_per_roll": 3,
        "cut_sampling.local_seed_radius_rad": 0.35,
        "cut_sampling.local_ik_consistency_rad": 0.75,
        "cut_sampling.max_candidates": 32,
        "cut_cost.transit_weight": 3.0,
        "cut_cost.process_weight": 1.0,
        "cut_cost.preferred_joint_travel_rad": 1.0,
        "cut_cost.excess_travel_weight": 8.0,
        # Optional hard limit on accumulated travel of any joint per transit.
        # Zero keeps large but necessary transits available at a higher cost.
        "cut_cost.max_transit_joint_travel_rad": 0.0,
        },]
        ,
    )
    return LaunchDescription([
        DeclareLaunchArgument("return_home_between_cuts", default_value="false",
            description="Force home between cuts; false compares direct and home routes by planned travel cost."),
        DeclareLaunchArgument("return_home_after_last_cut", default_value="true",
            description="Return to the initial robot state after the final retract."),
        DeclareLaunchArgument("inspect_cut_preview_only", default_value="true",
            description="Preview inspection views; false permits post-cut camera motion."),
        DeclareLaunchArgument("platform_accel_topic", default_value="/mpu6050/imu",
            description="Additional platform accelerometer. Set empty for lidar-only operation."),
        DeclareLaunchArgument("platform_imu_topic", default_value="/hb_robot_lidar/imu",
            description="Independent platform-fixed IMU; never use robot-mounted /k4a/imu here."),
        # Lidar IMU noise is higher than camera-settling noise. This initial
        # platform-only allowance matches platform_telemetry's existing default;
        # validate it against stationary AND disturbed recordings before cutting.
        DeclareLaunchArgument("platform_imu_max_accel_rms", default_value="0.2",
            description="Platform acceleration vector RMS about the window mean, in m/s^2; startup calibration setting."),
        Node(package="hb_robot_skills", executable="task_state_server", output="screen",
             parameters=[{"platform_imu_topic": "/hb_robot_lidar/imu",
                            "platform_accel_topic": "/mpu6050/imu",
                            "platform_imu.mpu6050_gyro_bias": [
                                                                -0.0787,
                                                                0.0088,
                                                                -0.0031,
                                                            ],
                            "platform_imu.max_gyro_rms": 0.01,
                            "platform_imu.max_accel_rms": 0.15,
                            "platform_imu.max_mean_gyro": 0.03,

                            "platform_imu.confidence_window_s": 2.0,
                            "platform_imu.confidence_acquire": 0.90,
                            "platform_imu.confidence_release": 0.65,
                            "platform_imu.supervision_grace_s": 0.80,

                            "platform_imu.hard_peak_gyro_rad_s": 0.20,
                            "platform_imu.hard_peak_accel_m_s2": 0.80,
                        }]),
        Node(package="hb_robot_skills", executable="capture_cut_environment", output="screen",
             additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
             parameters=[{"jacket_padding_m": 0.075, 
                        "workpiece_margin_m": 0.1,
                          "voxel_size_m": 0.05, 
                          "min_voxel_scans": 3,
                          "capture_duration_s": 1.0,
                          "debug_allow_invalid_scene": True,
                          "max_state_age_s": 1.0,
                          "lidar_max_range_m": 3.0,
                          "robot_description_node": "/robot_state_publisher"
                          }]),


        Node(package="hb_robot_skills", executable="revalidate_workpiece_server", output="screen"),
        Node(package="hb_robot_skills", executable="inspect_cut_server", output="screen",
             parameters=[moveit_config.to_dict(), {
                 "preview_only": ParameterValue(LaunchConfiguration("inspect_cut_preview_only"), value_type=bool)}]),
        Node(package="hb_robot_skills", executable="return_to_inspection_home_server",
             output="screen", parameters=[moveit_config.to_dict()]),
        cut_profile_server,
        inspect_profile_server
    ])



