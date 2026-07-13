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
        # Cone-tracker jitter knobs: lower smoothing_alpha = heavier EMA (steadier
        # cone positions); higher confirm_hits = a cone must be seen more times
        # before it feeds the planner.
        DeclareLaunchArgument('smoothing_alpha', default_value='0.35'),
        DeclareLaunchArgument('confirm_hits', default_value='2'),
        # Truncate the midline at the first consecutive-midpoint jump longer than
        # this (metres, absolute). Lower it if the path hops to opposite-side cones.
        DeclareLaunchArgument('max_midpoint_gap', default_value='4.0'),
        # Cost-based Delaunay midline (pass midline_method:=delaunay). Weights tune
        # the paper's cost terms; raise w_side/w_angle if the path picks wrong corridors.
        DeclareLaunchArgument('delaunay_target_length_m', default_value='10.0'),
        DeclareLaunchArgument('delaunay_seed_max_x', default_value='8.0'),
        DeclareLaunchArgument('delaunay_beam_width', default_value='5'),
        DeclareLaunchArgument('delaunay_max_depth', default_value='15'),
        DeclareLaunchArgument('delaunay_w_angle', default_value='1.0'),
        DeclareLaunchArgument('delaunay_w_width', default_value='1.0'),
        DeclareLaunchArgument('delaunay_w_length', default_value='1.0'),
        DeclareLaunchArgument('delaunay_w_side', default_value='1.5'),
        # Corner-cutting passes on the chosen centre-line (0=off, 2-3=smooth).
        DeclareLaunchArgument('delaunay_smoothing_iterations', default_value='2'),

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
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'confirm_hits': LaunchConfiguration('confirm_hits'),
                'max_midpoint_gap': LaunchConfiguration('max_midpoint_gap'),
                'delaunay_target_length_m': LaunchConfiguration('delaunay_target_length_m'),
                'delaunay_seed_max_x': LaunchConfiguration('delaunay_seed_max_x'),
                'delaunay_beam_width': LaunchConfiguration('delaunay_beam_width'),
                'delaunay_max_depth': LaunchConfiguration('delaunay_max_depth'),
                'delaunay_w_angle': LaunchConfiguration('delaunay_w_angle'),
                'delaunay_w_width': LaunchConfiguration('delaunay_w_width'),
                'delaunay_w_length': LaunchConfiguration('delaunay_w_length'),
                'delaunay_w_side': LaunchConfiguration('delaunay_w_side'),
                'delaunay_smoothing_iterations': LaunchConfiguration('delaunay_smoothing_iterations'),
            }],
        ),
    ])
