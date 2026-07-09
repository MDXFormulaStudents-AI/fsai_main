"""Skidpad navigation launch.

Starts two nodes:
  skidpad_path        — geometric path generator + state machine + in-loop correction
  local_path_follower — pure-pursuit follower with odom speed cap

Prerequisites (start separately before this launch):
  ros2 launch fsai_visualisation odom_forward_debug.launch.py
  (provides the odom → home → base_link → Fr1A TF chain)

Typical usage:
  # Default (3 m/s, 2 loops each side)
  ros2 launch fsai_navigation skidpad.launch.py

  # Tune speed
  ros2 launch fsai_navigation skidpad.launch.py max_speed_mps:=1.0

  # Adjust detection gate if staging distance changes
  ros2 launch fsai_navigation skidpad.launch.py min_approach_dist:=6.0

  # Disable Delaunay and use pairing only
  ros2 launch fsai_navigation skidpad.launch.py correction_use_delaunay:=false

  # Reduce correction aggressiveness
  ros2 launch fsai_navigation skidpad.launch.py correction_gain:=0.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Common ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/carmaker/odom'),
        DeclareLaunchArgument('cones_topic', default_value='/cones'),
        DeclareLaunchArgument('control_frame', default_value='Fr1A'),

        # ── Skidpad geometry ───────────────────────────────────────────────
        DeclareLaunchArgument('circle_radius', default_value='9.125'),
        DeclareLaunchArgument('loop_count_target', default_value='2'),

        # ── Skidpad path generation ────────────────────────────────────────
        DeclareLaunchArgument('lookahead_horizon', default_value='20.0'),

        # ── Orange-cone crossing detection ─────────────────────────────────
        DeclareLaunchArgument('min_approach_dist', default_value='20.0'),
        DeclareLaunchArgument('max_approach_dist', default_value='60.0'),
        DeclareLaunchArgument('orange_max_range', default_value='15.0'),
        DeclareLaunchArgument('orange_max_angle_deg', default_value='45.0'),
        DeclareLaunchArgument('min_orange_cluster_size', default_value='2'),
        DeclareLaunchArgument('crossing_hold_m', default_value='0.5'),

        # ── In-loop circle centre correction ───────────────────────────────
        DeclareLaunchArgument('correction_use_delaunay', default_value='true'),
        DeclareLaunchArgument('correction_use_pairing', default_value='true'),
        DeclareLaunchArgument('correction_min_ahead', default_value='0.5'),
        DeclareLaunchArgument('correction_max_ahead', default_value='10.0'),
        DeclareLaunchArgument('correction_track_width_min', default_value='1.5'),
        DeclareLaunchArgument('correction_track_width_max', default_value='5.0'),
        DeclareLaunchArgument('correction_min_midpoints', default_value='2'),
        DeclareLaunchArgument('correction_gain', default_value='0.2'),
        DeclareLaunchArgument('correction_deadband', default_value='0.15'),
        DeclareLaunchArgument('correction_max_step', default_value='0.5'),

        # ── Speed / exit ───────────────────────────────────────────────────
        DeclareLaunchArgument('max_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('exit_distance', default_value='20.0'),

        # ── Pure-pursuit tuning ────────────────────────────────────────────
        DeclareLaunchArgument('wheelbase', default_value='1.53'),
        DeclareLaunchArgument('lookahead_distance', default_value='3.0'),
        DeclareLaunchArgument('steer_gain', default_value='1.0'),
        DeclareLaunchArgument('max_steer', default_value='0.6'),
        DeclareLaunchArgument('max_steer_rate', default_value='4.0'),
        DeclareLaunchArgument('curvature_slowdown_gain', default_value='1.0'),
        DeclareLaunchArgument('cruise_gas', default_value='0.08'),
        DeclareLaunchArgument('min_gas', default_value='0.03'),
        DeclareLaunchArgument('max_gas', default_value='0.18'),
        DeclareLaunchArgument('speed_brake_gain', default_value='0.5'),
        DeclareLaunchArgument('max_speed_brake', default_value='0.3'),

        # ── skidpad_path node ──────────────────────────────────────────────
        Node(
            package='fsai_navigation',
            executable='skidpad_path',
            name='skidpad_path',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cones_topic': LaunchConfiguration('cones_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'path_topic': '/nav/skidpad_path',
                'debug_topic': '/nav/skidpad_path_markers',
                'circle_radius': LaunchConfiguration('circle_radius'),
                'loop_count_target': LaunchConfiguration('loop_count_target'),
                'lookahead_horizon': LaunchConfiguration('lookahead_horizon'),
                'min_approach_dist': LaunchConfiguration('min_approach_dist'),
                'max_approach_dist': LaunchConfiguration('max_approach_dist'),
                'orange_max_range': LaunchConfiguration('orange_max_range'),
                'orange_max_angle_deg': LaunchConfiguration('orange_max_angle_deg'),
                'min_orange_cluster_size': LaunchConfiguration('min_orange_cluster_size'),
                'crossing_hold_m': LaunchConfiguration('crossing_hold_m'),
                'exit_distance': LaunchConfiguration('exit_distance'),
                'correction_use_delaunay': LaunchConfiguration('correction_use_delaunay'),
                'correction_use_pairing': LaunchConfiguration('correction_use_pairing'),
                'correction_min_ahead': LaunchConfiguration('correction_min_ahead'),
                'correction_max_ahead': LaunchConfiguration('correction_max_ahead'),
                'correction_track_width_min': LaunchConfiguration('correction_track_width_min'),
                'correction_track_width_max': LaunchConfiguration('correction_track_width_max'),
                'correction_min_midpoints': LaunchConfiguration('correction_min_midpoints'),
                'correction_gain': LaunchConfiguration('correction_gain'),
                'correction_deadband': LaunchConfiguration('correction_deadband'),
                'correction_max_step': LaunchConfiguration('correction_max_step'),
            }],
        ),

        # ── local_path_follower node ───────────────────────────────────────
        Node(
            package='fsai_navigation',
            executable='local_path_follower',
            name='local_path_follower',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': '/nav/skidpad_path',
                'control_topic': '/carmaker/VehicleControl',
                'debug_topic': '/nav/local_path_follower_markers',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'control_frame': LaunchConfiguration('control_frame'),
                'enabled': True,
                'max_speed_mps': LaunchConfiguration('max_speed_mps'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'steer_gain': LaunchConfiguration('steer_gain'),
                'max_steer': LaunchConfiguration('max_steer'),
                'max_steer_rate': LaunchConfiguration('max_steer_rate'),
                'curvature_slowdown_gain': LaunchConfiguration('curvature_slowdown_gain'),
                'cruise_gas': LaunchConfiguration('cruise_gas'),
                'min_gas': LaunchConfiguration('min_gas'),
                'max_gas': LaunchConfiguration('max_gas'),
                'speed_brake_gain': LaunchConfiguration('speed_brake_gain'),
                'max_speed_brake': LaunchConfiguration('max_speed_brake'),
            }],
        ),
    ])
