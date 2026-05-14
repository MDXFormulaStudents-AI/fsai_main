"""
sensors.launch.py — launches only the bridge node.
Useful for inspecting raw sensor data on /perception/* topics without
running the full detection pipeline.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fsai_perception')

    return LaunchDescription([
        DeclareLaunchArgument('visualize', default_value='false',
                              description='Launch RViz'),

        Node(
            package='fsai_perception',
            executable='bridge',
            name='bridge',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('visualize')),
        ),
    ])
