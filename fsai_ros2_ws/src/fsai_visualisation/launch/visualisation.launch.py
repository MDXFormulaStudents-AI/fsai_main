"""
visualisation.launch.py — cone markers + RViz for the perception pipeline.

Run this alongside perception.launch.py to get live visualisation:
  ros2 launch fsai_visualisation visualisation.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fsai_visualisation')
    rviz_cfg = PathJoinSubstitution([pkg, 'config', 'fsai.rviz'])

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('show_unknown', default_value='true',
                              description='Show LiDAR-only (uncoloured) cones'),

        Node(
            package='fsai_visualisation',
            executable='cone_visualizer',
            name='cone_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time':  True,
                'show_unknown':  LaunchConfiguration('show_unknown'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
