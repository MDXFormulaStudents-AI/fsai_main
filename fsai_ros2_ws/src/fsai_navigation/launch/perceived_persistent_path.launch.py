"""Launch perceived path generation and fixed-frame path memory."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cones_topic', default_value='/cones'),
        DeclareLaunchArgument('perceived_path_topic', default_value='/nav/perceived_path'),
        DeclareLaunchArgument('persistent_path_topic', default_value='/nav/persistent_path'),
        DeclareLaunchArgument('perceived_debug_topic', default_value='/nav/perceived_path_markers'),
        DeclareLaunchArgument('persistent_debug_topic', default_value='/nav/persistent_path_markers'),
        DeclareLaunchArgument('odom_topic', default_value='/carmaker/odom'),
        DeclareLaunchArgument('midline_method', default_value='forward_pairs'),
        DeclareLaunchArgument('assign_unknown_by_side', default_value='true'),
        DeclareLaunchArgument('unknown_cone_min_abs_y', default_value='0.5'),
        DeclareLaunchArgument('max_track_age_sec', default_value='1.5'),
        DeclareLaunchArgument('memory_frame', default_value='home'),
        DeclareLaunchArgument('control_frame', default_value='Fr1A'),
        DeclareLaunchArgument('max_hold_sec', default_value='3.0'),
        DeclareLaunchArgument('odom_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('require_odom', default_value='true'),
        DeclareLaunchArgument('min_store_path_length', default_value='3.0'),
        DeclareLaunchArgument('min_remaining_path_length', default_value='2.0'),
        DeclareLaunchArgument('max_cross_track_error', default_value='2.5'),
        DeclareLaunchArgument('start_pure_pursuit', default_value='false'),
        DeclareLaunchArgument('control_topic', default_value='/carmaker/VehicleControl'),
        DeclareLaunchArgument('pure_pursuit_debug_topic', default_value='/nav/pure_pursuit_path_follower_markers'),
        DeclareLaunchArgument('cruise_gas', default_value='0.08'),
        DeclareLaunchArgument('min_gas', default_value='0.03'),
        DeclareLaunchArgument('max_gas', default_value='0.18'),
        DeclareLaunchArgument('lookahead_distance', default_value='3.0'),
        DeclareLaunchArgument('curvature_slowdown_gain', default_value='3.0'),

        Node(
            package='fsai_navigation',
            executable='perceived_path',
            name='perceived_path',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('cones_topic'),
                'path_topic': LaunchConfiguration('perceived_path_topic'),
                'debug_topic': LaunchConfiguration('perceived_debug_topic'),
                'midline_method': LaunchConfiguration('midline_method'),
                'assign_unknown_by_side': LaunchConfiguration('assign_unknown_by_side'),
                'unknown_cone_min_abs_y': LaunchConfiguration('unknown_cone_min_abs_y'),
                'max_track_age_sec': LaunchConfiguration('max_track_age_sec'),
            }],
        ),

        Node(
            package='fsai_navigation',
            executable='persistent_path',
            name='persistent_path',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_path_topic': LaunchConfiguration('perceived_path_topic'),
                'output_path_topic': LaunchConfiguration('persistent_path_topic'),
                'debug_topic': LaunchConfiguration('persistent_debug_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'memory_frame': LaunchConfiguration('memory_frame'),
                'control_frame': LaunchConfiguration('control_frame'),
                'max_hold_sec': LaunchConfiguration('max_hold_sec'),
                'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                'require_odom': LaunchConfiguration('require_odom'),
                'min_store_path_length': LaunchConfiguration('min_store_path_length'),
                'min_remaining_path_length': LaunchConfiguration('min_remaining_path_length'),
                'max_cross_track_error': LaunchConfiguration('max_cross_track_error'),
            }],
        ),

        Node(
            package='fsai_navigation',
            executable='pure_pursuit_path_follower',
            name='pure_pursuit_path_follower',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_pure_pursuit')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': LaunchConfiguration('persistent_path_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
                'debug_topic': LaunchConfiguration('pure_pursuit_debug_topic'),
                'control_frame': LaunchConfiguration('control_frame'),
                'cruise_gas': LaunchConfiguration('cruise_gas'),
                'min_gas': LaunchConfiguration('min_gas'),
                'max_gas': LaunchConfiguration('max_gas'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'curvature_slowdown_gain': LaunchConfiguration('curvature_slowdown_gain'),
            }],
        ),
    ])
