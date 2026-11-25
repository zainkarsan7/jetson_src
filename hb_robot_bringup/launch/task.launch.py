from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():

return LaunchDescription([DeclareLaunchArgument("platform_imu_max_accel_rms", default_value="0.2",
            description="Platform acceleration vector RMS about the window mean, in m/s^2; startup calibration setting."),
        Node(package="hb_robot_skills", executable="task_state_server", output="screen",
             parameters=[{"platform_imu_topic": "/mpu6050/imu",
                            "platform_imu_2_topic": "/mpu6050_2/imu",
                            "platform_imu.mpu6050_gyro_bias": [
                                                                -0.0787,
                                                                0.0088,
                                                                -0.0031,
                                                            ],
                            "platform_imu.mpu6050_2_gyro_bias": [
                                -0.0787,
                                0.0088,
                                -0.0031,
                            ],
                            "platform_imu.max_gyro_rms": 10.01,
                            "platform_imu.max_accel_rms":10.15,
                            "platform_imu.max_mean_gyro": 10.03,

                            "platform_imu.confidence_window_s": 2.0,
                            "platform_imu.confidence_acquire": 0.90,
                            "platform_imu.confidence_release": 0.65,
                            "platform_imu.supervision_grace_s": 0.80,

                            "platform_imu.hard_peak_gyro_rad_s": 10.20,
                            "platform_imu.hard_peak_accel_m_s2": 10.80,
                        }]), ])