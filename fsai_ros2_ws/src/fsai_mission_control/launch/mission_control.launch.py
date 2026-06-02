"""Launches the mission manager and static profile executor."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_manager = Node(
        package='fsai_mission_control',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        emulate_tty=True,
    )

    static_profile_executor = Node(
        package='fsai_mission_control',
        executable='static_profile_executor',
        name='static_profile_executor',
        output='screen',
        emulate_tty=True,
        parameters=[{'wheel_circumference_m': 1.674}],
    )

    return LaunchDescription([
        mission_manager,
        static_profile_executor,
    ])
