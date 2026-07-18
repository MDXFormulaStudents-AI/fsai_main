"""Single bring-up for ALL seven missions (AMI-driven track logic selection).

One mission_manager routes whatever the operator selects on the ADS-DV touchscreen:

  static_inspection_a/b, autonomous_demo -> static_profile_executor (YAML profiles)
  acceleration -> forward_distance_controller (75 m)
  skidpad      -> skidpad_path   -> /nav/active_path -> local_path_follower
  autocross    -> perceived_path -> /nav/active_path -> local_path_follower   (1 lap)
  trackdrive   -> perceived_path -> /nav/active_path -> local_path_follower   (10 laps)

mission_manager subscribes to both /static_drive_command and /dynamic_drive_command
and forwards the matching candidate to /vehicle/drive_command only while the
interface is DRIVING. Run this ONCE — do not also run mission_control.launch.py
(that would spawn a second mission_manager).

Bring these up SEPARATELY (not started here):
  - fsai_vehicle_interface  (real car / vcan HiL)   OR   vcu_simulator.py (bench)
  - perception producing /cones, and the odom source (/carmaker/odom in sim,
    /odometry/vehicle from fsai_localization on the real car).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cones_topic = LaunchConfiguration('cones_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    active_path_topic = LaunchConfiguration('active_path_topic')
    drive_command_topic = LaunchConfiguration('drive_command_topic')
    wheel_circumference_m = LaunchConfiguration('wheel_circumference_m')

    return LaunchDescription([
        # Defaults are set for the real-car / CAN-HiL run (wall-clock time, the
        # DriveCommand path through mission_manager, real odom source). Override
        # on the command line for the pure CMRosIF sim if needed.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cones_topic', default_value='/cones'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/vehicle'),
        DeclareLaunchArgument('active_path_topic', default_value='/nav/active_path'),
        DeclareLaunchArgument('drive_command_topic', default_value='/dynamic_drive_command'),
        # Followers active by default so they generate DriveCommand candidates.
        DeclareLaunchArgument('follower_enabled', default_value='true'),
        DeclareLaunchArgument('steer_command_gain', default_value='1.0'),
        DeclareLaunchArgument('max_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('midline_method', default_value='delaunay'),
        DeclareLaunchArgument('acceleration_distance', default_value='75.0'),
        DeclareLaunchArgument('acceleration_max_speed', default_value='4.0'),
        DeclareLaunchArgument('acceleration_min_crawl_speed', default_value='2.0'),
        DeclareLaunchArgument('wheel_circumference_m', default_value='1.674'),
        DeclareLaunchArgument('drive_torque_nm', default_value='500.0'),
        DeclareLaunchArgument('max_axle_rpm', default_value='500.0'),
        DeclareLaunchArgument('autocross_laps', default_value='1'),
        DeclareLaunchArgument('trackdrive_laps', default_value='10'),

        # ── Mission control (one manager for all 7 missions) ───────────────
        Node(
            package='fsai_mission_control', executable='mission_manager',
            name='mission_manager', output='screen', emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='fsai_mission_control', executable='static_profile_executor',
            name='static_profile_executor', output='screen', emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'wheel_circumference_m': wheel_circumference_m,
            }],
        ),
        Node(
            package='fsai_mission_control', executable='dynamic_mission_executor',
            name='dynamic_mission_executor', output='screen', emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'cones_topic': cones_topic,
                'odom_topic': odom_topic,
                'autocross_laps': LaunchConfiguration('autocross_laps'),
                'trackdrive_laps': LaunchConfiguration('trackdrive_laps'),
            }],
        ),

        # ── Path generators (self-gated → shared active path) ──────────────
        Node(
            package='fsai_navigation', executable='skidpad_path',
            name='skidpad_path', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cones_topic': cones_topic,
                'odom_topic': odom_topic,
                'path_topic': active_path_topic,
                'mission_gates': 'skidpad',
            }],
        ),
        Node(
            package='fsai_navigation', executable='perceived_path',
            name='perceived_path', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': cones_topic,
                'path_topic': active_path_topic,
                'mission_gates': 'acceleration,autocross,trackdrive',
                'midline_method': LaunchConfiguration('midline_method'),
            }],
        ),

        # ── Followers (publish DriveCommand candidate, self-gated) ─────────
        Node(
            package='fsai_navigation', executable='local_path_follower',
            name='local_path_follower', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'path_topic': active_path_topic,
                'odom_topic': odom_topic,
                'enabled': LaunchConfiguration('follower_enabled'),
                'publish_drive_command': True,
                'drive_command_topic': drive_command_topic,
                # Acceleration now runs through the follower (Option A): it steers to
                # the perceived-path midline AND stops itself at acceleration_distance.
                'mission_gates': ['acceleration', 'skidpad', 'autocross', 'trackdrive'],
                'accel_target_distance': LaunchConfiguration('acceleration_distance'),
                'steer_command_gain': LaunchConfiguration('steer_command_gain'),
                'max_speed_mps': LaunchConfiguration('max_speed_mps'),
                'wheel_circumference_m': wheel_circumference_m,
                'drive_torque_nm': LaunchConfiguration('drive_torque_nm'),
                'max_axle_rpm': LaunchConfiguration('max_axle_rpm'),
            }],
        ),
        # forward_distance_controller — RETIRED from acceleration (Option A). The
        # follower now steers to the cone midline AND runs the distance-stop, so this
        # blind-straight controller is left dormant (not driving, not commanding). To
        # revert to blind-straight acceleration: set enabled:=true + publish_drive_command:=true
        # here, and remove 'acceleration' from local_path_follower's mission_gates above.
        Node(
            package='fsai_navigation', executable='forward_distance_controller',
            name='forward_distance_controller', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'enabled': False,
                'publish_drive_command': False,
                'drive_command_topic': drive_command_topic,
                'mission_gate': 'acceleration',
                'target_distance': LaunchConfiguration('acceleration_distance'),
                'max_speed': LaunchConfiguration('acceleration_max_speed'),
                'min_crawl_speed': LaunchConfiguration('acceleration_min_crawl_speed'),
                'wheel_circumference_m': wheel_circumference_m,
                'drive_torque_nm': LaunchConfiguration('drive_torque_nm'),
                'max_axle_rpm': LaunchConfiguration('max_axle_rpm'),
            }],
        ),
    ])
