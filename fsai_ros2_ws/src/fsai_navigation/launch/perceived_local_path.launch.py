"""Launch perceived path generation only.

The local path follower is started separately via local_path_follower.launch.py
so that perception can be verified before the vehicle is commanded to move.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cones_topic', default_value='/cones'),
        DeclareLaunchArgument('path_topic', default_value='/nav/perceived_path'),
        DeclareLaunchArgument('perceived_debug_topic', default_value='/nav/perceived_path_markers'),
        DeclareLaunchArgument('midline_method', default_value='forward_pairs'),
        DeclareLaunchArgument('assign_unknown_by_side', default_value='true'),
        DeclareLaunchArgument('unknown_cone_min_abs_y', default_value='0.5'),
        DeclareLaunchArgument('max_track_age_sec', default_value='1.5'),

        Node(
            package='fsai_navigation',
            executable='perceived_path',
            name='perceived_path',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('cones_topic'),
                'path_topic': LaunchConfiguration('path_topic'),
                'debug_topic': LaunchConfiguration('perceived_debug_topic'),
                'midline_method': LaunchConfiguration('midline_method'),
                'assign_unknown_by_side': LaunchConfiguration('assign_unknown_by_side'),
                'unknown_cone_min_abs_y': LaunchConfiguration('unknown_cone_min_abs_y'),
                'max_track_age_sec': LaunchConfiguration('max_track_age_sec'),
            }],
        ),
    ])
