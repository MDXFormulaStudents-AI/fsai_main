"""Real-car localization: wheel+IMU odometry + the fixed-frame (home/Fr1A) TF.

vehicle_odometry synthesizes nav_msgs/Odometry from /vcu/wheel_speeds + /vcu/imu;
carmaker_odom_tf (reused, despite the name) turns that into the
odom -> home -> base_link -> Fr1A chain the nav stack already uses. Point the nav
nodes' odom_topic at odom_topic below (default /odometry/vehicle).

Prereq: fsai_vehicle_interface running (publishes /vcu/*). Not started here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/vehicle'),
        DeclareLaunchArgument('wheel_circumference_m', default_value='1.674'),
        DeclareLaunchArgument('aligned_frame_id', default_value='home'),
        DeclareLaunchArgument('fr1a_frame_id', default_value='Fr1A'),
        # base_link -> Fr1A static offset (ADS-DV: rear axle sits at x=+0.5637 m in
        # Fr1A). Only x is non-zero; y/z/rpy use the node defaults (0.0).
        DeclareLaunchArgument('base_link_to_fr1a_x', default_value='0.5637'),

        Node(
            package='fsai_localization', executable='vehicle_odometry',
            name='vehicle_odometry', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'wheel_circumference_m': LaunchConfiguration('wheel_circumference_m'),
            }],
        ),

        # NOTE: base_link -> Fr1A is NO LONGER published here. It is fixed vehicle
        # geometry, so it now lives as a static transform in fsai_sensors_bringup
        # sensors.launch.py (alongside the sensor mounts) — that decouples perception
        # /fusion from odometry entirely. carmaker_odom_tf's own odom-gated Fr1A
        # publish stays disabled below so there is exactly one publisher.

        # Fixed-frame builder (reused from fsai_visualisation). Consumes the
        # synthesized odom and emits odom -> home -> base_link only.
        Node(
            package='fsai_visualisation', executable='carmaker_odom_tf',
            name='odom_tf', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'aligned_frame_id': LaunchConfiguration('aligned_frame_id'),
                'fr1a_frame_id': LaunchConfiguration('fr1a_frame_id'),
                'base_link_to_fr1a_x': LaunchConfiguration('base_link_to_fr1a_x'),
                'publish_base_link_to_fr1a_tf': False,
            }],
        ),
    ])
