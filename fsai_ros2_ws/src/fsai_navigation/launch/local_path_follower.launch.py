"""Standalone local path follower launch.

Start this separately after verifying perceived_path is producing valid output.
Subscribes to /nav/perceived_path and publishes /carmaker/VehicleControl.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_topic', default_value='/nav/perceived_path'),
        DeclareLaunchArgument('control_topic', default_value='/carmaker/VehicleControl'),
        DeclareLaunchArgument('debug_topic', default_value='/nav/local_path_follower_markers'),
        DeclareLaunchArgument('odom_topic', default_value='/carmaker/odom'),
        DeclareLaunchArgument('control_frame', default_value='Fr1A'),
        DeclareLaunchArgument('max_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('cruise_gas', default_value='0.08'),
        DeclareLaunchArgument('min_gas', default_value='0.03'),
        DeclareLaunchArgument('max_gas', default_value='0.18'),
        DeclareLaunchArgument('lookahead_distance', default_value='3.0'),
        DeclareLaunchArgument('curvature_slowdown_gain', default_value='3.0'),
        DeclareLaunchArgument('speed_brake_gain', default_value='0.5'),
        DeclareLaunchArgument('max_speed_brake', default_value='0.3'),
        # Plant-inverse gain. Default 1.0 = identity (REAL VEHICLE, do-no-harm).
        # The CarMaker (CMRosIF) sim realises only ~0.146x of the commanded
        # road-wheel angle, so SIM runs must opt in with steer_command_gain:=6.85
        # (see steer_calibration.py). The follower warns whenever this is != 1.0.
        DeclareLaunchArgument('steer_command_gain', default_value='1.0'),

        Node(
            package='fsai_navigation',
            executable='local_path_follower',
            name='local_path_follower',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': LaunchConfiguration('path_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
                'debug_topic': LaunchConfiguration('debug_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'control_frame': LaunchConfiguration('control_frame'),
                'enabled': True,
                'max_speed_mps': LaunchConfiguration('max_speed_mps'),
                'cruise_gas': LaunchConfiguration('cruise_gas'),
                'min_gas': LaunchConfiguration('min_gas'),
                'max_gas': LaunchConfiguration('max_gas'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'curvature_slowdown_gain': LaunchConfiguration('curvature_slowdown_gain'),
                'speed_brake_gain': LaunchConfiguration('speed_brake_gain'),
                'max_speed_brake': LaunchConfiguration('max_speed_brake'),
                'steer_command_gain': LaunchConfiguration('steer_command_gain'),
            }],
        ),
    ])
