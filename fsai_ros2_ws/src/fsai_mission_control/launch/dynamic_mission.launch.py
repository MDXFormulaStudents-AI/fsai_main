"""Bring up the dynamic-mission stack (AMI-driven track logic selection).

Starts mission_manager + dynamic_mission_executor and the four dynamic pipelines,
all self-gated on /mission/selected:

  acceleration -> forward_distance_controller (75 m)
  skidpad      -> skidpad_path       -> /nav/active_path -> local_path_follower
  autocross    -> perceived_path     -> /nav/active_path -> local_path_follower  (1 lap)
  trackdrive   -> perceived_path     -> /nav/active_path -> local_path_follower  (10 laps)

The followers publish DriveCommand on /dynamic_drive_command; mission_manager forwards
it to /vehicle/drive_command only for the selected mission while the interface is DRIVING.

Bring these up SEPARATELY (not started here):
  - fsai_vehicle_interface  (real car / vcan HiL)   OR   vcu_simulator.py (bench)
  - perception producing /cones, and the odom source (/carmaker/odom).
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cones_topic', default_value='/cones'),
        DeclareLaunchArgument('odom_topic', default_value='/carmaker/odom'),
        DeclareLaunchArgument('active_path_topic', default_value='/nav/active_path'),
        DeclareLaunchArgument('drive_command_topic', default_value='/dynamic_drive_command'),
        # Sim VehicleControl output from the followers (CMRosIF). Real/HiL runs the
        # DriveCommand path via mission_manager, so leave this off there.
        DeclareLaunchArgument('follower_enabled', default_value='false'),
        DeclareLaunchArgument('steer_command_gain', default_value='1.0'),
        DeclareLaunchArgument('max_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('midline_method', default_value='delaunay'),
        DeclareLaunchArgument('acceleration_distance', default_value='75.0'),
        DeclareLaunchArgument('wheel_circumference_m', default_value='1.674'),
        DeclareLaunchArgument('drive_torque_nm', default_value='50.0'),
        DeclareLaunchArgument('max_axle_rpm', default_value='500.0'),
        DeclareLaunchArgument('autocross_laps', default_value='1'),
        DeclareLaunchArgument('trackdrive_laps', default_value='10'),

        # ── Mission control ────────────────────────────────────────────────
        Node(
            package='fsai_mission_control', executable='mission_manager',
            name='mission_manager', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='fsai_mission_control', executable='dynamic_mission_executor',
            name='dynamic_mission_executor', output='screen',
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
                'mission_gates': 'autocross,trackdrive',
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
                'mission_gates': ['skidpad', 'autocross', 'trackdrive'],
                'steer_command_gain': LaunchConfiguration('steer_command_gain'),
                'max_speed_mps': LaunchConfiguration('max_speed_mps'),
                'wheel_circumference_m': LaunchConfiguration('wheel_circumference_m'),
                'drive_torque_nm': LaunchConfiguration('drive_torque_nm'),
                'max_axle_rpm': LaunchConfiguration('max_axle_rpm'),
            }],
        ),
        Node(
            package='fsai_navigation', executable='forward_distance_controller',
            name='forward_distance_controller', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'enabled': LaunchConfiguration('follower_enabled'),
                'publish_drive_command': True,
                'drive_command_topic': drive_command_topic,
                'mission_gate': 'acceleration',
                'target_distance': LaunchConfiguration('acceleration_distance'),
                'wheel_circumference_m': LaunchConfiguration('wheel_circumference_m'),
                'drive_torque_nm': LaunchConfiguration('drive_torque_nm'),
                'max_axle_rpm': LaunchConfiguration('max_axle_rpm'),
            }],
        ),
    ])
