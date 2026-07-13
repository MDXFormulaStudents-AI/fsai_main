"""Skidpad navigation launch.

Starts two nodes:
  skidpad_path        — geometric path generator + state machine
  local_path_follower — pure-pursuit follower with odom speed cap

Prerequisites (start separately before this launch):
  ros2 launch fsai_visualisation odom_forward_debug.launch.py
  (provides the odom → home → base_link → Fr1A TF chain)

SIMULATION NOTE:
  steer_command_gain defaults to 1.0 (real vehicle). In the CarMaker sim you
  MUST add steer_command_gain:=6.85 or the car under-steers ~7x. All sim
  examples below include it.

Typical usage:
  # Default (3 m/s, 2 loops each side) — SIM
  ros2 launch fsai_navigation skidpad.launch.py steer_command_gain:=6.85

  # Tune speed
  ros2 launch fsai_navigation skidpad.launch.py max_speed_mps:=1.0

  # Adjust detection gate if staging distance changes
  ros2 launch fsai_navigation skidpad.launch.py min_approach_dist:=6.0

  # Use pairing instead of Delaunay for the mid-loop detector
  ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=pairing

  # Use only single-sided cone offsets; Delaunay/pairing are disabled
  ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=single_side

  # Disable mid-loop correction entirely (pure geometric path).
  # NOTE: use 'none', not 'off' — YAML 1.1 parses the bare word 'off' as the
  # boolean False, which fails the string parameter type check.
  ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=none

  # Narrow the mid-loop perception window if entry/exit cones leak in
  ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_start_deg:=90.0 mid_loop_detection_end_deg:=260.0

  # Steering law A/B: open-loop fixed-angle lock vs feed-forward + correction
  ros2 launch fsai_navigation skidpad.launch.py steering_mode:=lock steer_command_gain:=6.85
  ros2 launch fsai_navigation skidpad.launch.py steering_mode:=feedforward steer_command_gain:=6.85
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

        # ── Mid-loop detection/correction ──────────────────────────────────
        DeclareLaunchArgument('mid_loop_detection_method', default_value='delaunay'),
        DeclareLaunchArgument('mid_loop_detection_start_deg', default_value='75.0'),
        DeclareLaunchArgument('mid_loop_detection_end_deg', default_value='285.0'),
        DeclareLaunchArgument('mid_loop_detection_min_ahead', default_value='0.5'),
        DeclareLaunchArgument('mid_loop_detection_max_ahead', default_value='10.0'),
        DeclareLaunchArgument('mid_loop_detection_track_width_min', default_value='1.5'),
        DeclareLaunchArgument('mid_loop_detection_track_width_max', default_value='5.0'),
        DeclareLaunchArgument('mid_loop_detection_radial_margin', default_value='1.2'),
        DeclareLaunchArgument('mid_loop_detection_min_midpoints', default_value='2'),
        DeclareLaunchArgument('mid_loop_detection_gain', default_value='0.15'),
        DeclareLaunchArgument('mid_loop_detection_deadband', default_value='0.20'),
        DeclareLaunchArgument('mid_loop_detection_max_step', default_value='0.35'),
        DeclareLaunchArgument('mid_loop_single_side_track_width', default_value='3.0'),
        DeclareLaunchArgument('mid_loop_single_side_radial_tolerance', default_value='1.5'),
        DeclareLaunchArgument('mid_loop_single_side_min_cones', default_value='1'),

        # ── Speed / exit ───────────────────────────────────────────────────
        DeclareLaunchArgument('max_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('exit_distance', default_value='20.0'),

        # ── Pure-pursuit tuning ────────────────────────────────────────────
        DeclareLaunchArgument('wheelbase', default_value='1.53'),
        DeclareLaunchArgument('lookahead_distance', default_value='3.0'),
        DeclareLaunchArgument('steer_gain', default_value='1.0'),
        DeclareLaunchArgument('max_steer', default_value='0.6'),
        DeclareLaunchArgument('max_steer_rate', default_value='4.0'),
        # Plant-inverse gain. Default 1.0 = identity (REAL VEHICLE, do-no-harm).
        # The CarMaker (CMRosIF) sim realises only ~0.146x of the commanded
        # road-wheel angle, so SIM runs must opt in with steer_command_gain:=6.85
        # (see steer_calibration.py). The follower logs a warning whenever this
        # is != 1.0 so an accidental sim gain on hardware cannot pass unnoticed.
        DeclareLaunchArgument('steer_command_gain', default_value='1.0'),
        # Steering law: 'pursuit' (pure pursuit), 'lock' (open-loop fixed angle
        # for the skidpad radius), or 'feedforward' (path-curvature feed-forward
        # + light cross-track/heading correction).
        DeclareLaunchArgument('steering_mode', default_value='pursuit'),
        DeclareLaunchArgument('path_straight_curvature', default_value='0.03'),
        DeclareLaunchArgument('crosstrack_gain', default_value='0.15'),
        DeclareLaunchArgument('heading_gain', default_value='0.5'),
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
                'mid_loop_detection_method': LaunchConfiguration('mid_loop_detection_method'),
                'mid_loop_detection_start_deg': LaunchConfiguration('mid_loop_detection_start_deg'),
                'mid_loop_detection_end_deg': LaunchConfiguration('mid_loop_detection_end_deg'),
                'mid_loop_detection_min_ahead': LaunchConfiguration('mid_loop_detection_min_ahead'),
                'mid_loop_detection_max_ahead': LaunchConfiguration('mid_loop_detection_max_ahead'),
                'mid_loop_detection_track_width_min': LaunchConfiguration('mid_loop_detection_track_width_min'),
                'mid_loop_detection_track_width_max': LaunchConfiguration('mid_loop_detection_track_width_max'),
                'mid_loop_detection_radial_margin': LaunchConfiguration('mid_loop_detection_radial_margin'),
                'mid_loop_detection_min_midpoints': LaunchConfiguration('mid_loop_detection_min_midpoints'),
                'mid_loop_detection_gain': LaunchConfiguration('mid_loop_detection_gain'),
                'mid_loop_detection_deadband': LaunchConfiguration('mid_loop_detection_deadband'),
                'mid_loop_detection_max_step': LaunchConfiguration('mid_loop_detection_max_step'),
                'mid_loop_single_side_track_width': LaunchConfiguration('mid_loop_single_side_track_width'),
                'mid_loop_single_side_radial_tolerance': LaunchConfiguration('mid_loop_single_side_radial_tolerance'),
                'mid_loop_single_side_min_cones': LaunchConfiguration('mid_loop_single_side_min_cones'),
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
                'steer_command_gain': LaunchConfiguration('steer_command_gain'),
                'steering_mode': LaunchConfiguration('steering_mode'),
                'skidpad_radius': LaunchConfiguration('circle_radius'),
                'path_straight_curvature': LaunchConfiguration('path_straight_curvature'),
                'crosstrack_gain': LaunchConfiguration('crosstrack_gain'),
                'heading_gain': LaunchConfiguration('heading_gain'),
                'curvature_slowdown_gain': LaunchConfiguration('curvature_slowdown_gain'),
                'cruise_gas': LaunchConfiguration('cruise_gas'),
                'min_gas': LaunchConfiguration('min_gas'),
                'max_gas': LaunchConfiguration('max_gas'),
                'speed_brake_gain': LaunchConfiguration('speed_brake_gain'),
                'max_speed_brake': LaunchConfiguration('max_speed_brake'),
            }],
        ),
    ])
