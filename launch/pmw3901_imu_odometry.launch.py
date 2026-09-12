import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    enable_xy_log = LaunchConfiguration("enable_xy_log")
    enable_debug_csv_log = LaunchConfiguration("enable_debug_csv_log")
    imu_yaw_weight = LaunchConfiguration("imu_yaw_weight")
    imu_gyro_z_sign = LaunchConfiguration("imu_gyro_z_sign")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("serial_baud", default_value="115200"),
            DeclareLaunchArgument("enable_xy_log", default_value="false"),
            DeclareLaunchArgument("enable_debug_csv_log", default_value="false"),
            DeclareLaunchArgument("imu_yaw_weight", default_value="0.5"),
            DeclareLaunchArgument("imu_gyro_z_sign", default_value="1.0"),
            Node(
                package="mouse_odometry",
                executable="pmw3901_serial_bridge",
                name="pmw3901_serial_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baud": ParameterValue(serial_baud, value_type=int),
                    }
                ],
            ),
            Node(
                package="mouse_odometry",
                executable="mouse_odom_node",
                name="mouse_odom_node",
                output="screen",
                parameters=[
                    {
                        "left_sensor_yaw": 0.0,
                        "right_sensor_yaw": math.pi,
                        "yaw_scale": 0.976,
                        "enable_xy_log": ParameterValue(
                            enable_xy_log, value_type=bool
                        ),
                        "enable_debug_csv_log": ParameterValue(
                            enable_debug_csv_log, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="mouse_odometry",
                executable="mouse_odom_imu_fusion_node",
                name="mouse_odom_imu_fusion_node",
                output="screen",
                parameters=[
                    {
                        "imu_topic": "/aiformula_sensing/zed_node/imu",
                        "imu_yaw_weight": ParameterValue(
                            imu_yaw_weight, value_type=float
                        ),
                        "imu_gyro_z_sign": ParameterValue(
                            imu_gyro_z_sign, value_type=float
                        ),
                        "enable_debug_csv_log": ParameterValue(
                            enable_debug_csv_log, value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
