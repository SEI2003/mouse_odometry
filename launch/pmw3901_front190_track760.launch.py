"""Start PMW3901 odometry with sensors 190 mm ahead of wheel center, 760 mm apart."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    publish_tf = LaunchConfiguration('publish_tf')
    enable_xy_log = LaunchConfiguration('enable_xy_log')
    enable_debug_csv_log = LaunchConfiguration('enable_debug_csv_log')

    # Distance calibration parameters
    left_x_meter_per_count = LaunchConfiguration('left_x_meter_per_count')
    right_x_meter_per_count = LaunchConfiguration('right_x_meter_per_count')

    # Y-axis calibration
    left_y_meter_per_count = LaunchConfiguration('left_y_meter_per_count')
    right_y_meter_per_count = LaunchConfiguration('right_y_meter_per_count')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='ESP32 / PMW3901 serial port',
        ),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('enable_xy_log', default_value='false'),
        DeclareLaunchArgument('enable_debug_csv_log', default_value='false'),

        # Calibrated X-axis distance per count
        DeclareLaunchArgument(
            'left_x_meter_per_count',
            default_value='0.00074195',
        ),
        DeclareLaunchArgument(
            'right_x_meter_per_count',
            default_value='0.00094224',
        ),
        DeclareLaunchArgument(
            'left_y_meter_per_count',
            default_value='0.00103584',
        ),
        DeclareLaunchArgument(
            'right_y_meter_per_count',
            default_value='0.00083645',
        ),

        Node(
            package='mouse_odometry',
            executable='pmw3901_serial_bridge',
            name='pmw3901_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baud': ParameterValue(
                    serial_baud, value_type=int
                ),
                'image_debug_only': False,
            }],
        ),

        Node(
            package='mouse_odometry',
            executable='mouse_odom_node',
            name='mouse_odom_node',
            output='screen',
            parameters=[{
                # Sensor geometry
                'left_sensor_x': 0.190,
                'left_sensor_y': 0.380,
                'right_sensor_x': 0.190,
                'right_sensor_y': -0.380,

                # Sensor orientation
                'left_sensor_yaw': 0.0,
                'right_sensor_yaw': math.pi,

                # X-axis calibration
                'left_x_meter_per_count': ParameterValue(
                    left_x_meter_per_count, value_type=float
                ),
                'right_x_meter_per_count': ParameterValue(
                    right_x_meter_per_count, value_type=float
                ),

                # Y-axis calibration
                'left_y_meter_per_count': ParameterValue(
                    left_y_meter_per_count, value_type=float
                ),
                'right_y_meter_per_count': ParameterValue(
                    right_y_meter_per_count, value_type=float
                ),

                # Yaw correction
                'yaw_scale': 0.976,

                # ROS and logging
                'publish_tf': ParameterValue(
                    publish_tf, value_type=bool
                ),
                'enable_xy_log': ParameterValue(
                    enable_xy_log, value_type=bool
                ),
                'enable_debug_csv_log': ParameterValue(
                    enable_debug_csv_log, value_type=bool
                ),
            }],
        ),
    ])