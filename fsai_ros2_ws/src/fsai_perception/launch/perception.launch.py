"""
perception.launch.py — launches the full perception pipeline:
  bridge → lidar_detector → camera_detector → fusion → /cones
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

        DeclareLaunchArgument('env', default_value='sim',
                              description='Config environment: sim (CarMaker) or real (VLP-16 + ZED2). '
                                          'Selects the section of perception.yaml each node loads.'),
        DeclareLaunchArgument('visualize', default_value='false',
                              description='Launch RViz'),
        DeclareLaunchArgument('device', default_value='cuda:0',
                              description='YOLO inference device: cpu or cuda:0'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock (true for CarMaker, false for real hardware)'),

        # ── Bridge ──────────────────────────────────────────────────────
        Node(
            package='fsai_perception',
            executable='bridge',
            name='bridge',
            output='screen',
            parameters=[{
                'env': LaunchConfiguration('env'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # ── LiDAR Detector ──────────────────────────────────────────────
        Node(
            package='fsai_perception',
            executable='lidar_detector',
            name='lidar_detector',
            output='screen',
            parameters=[{
                'env': LaunchConfiguration('env'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # ── Camera Detector ─────────────────────────────────────────────
        Node(
            package='fsai_perception',
            executable='camera_detector',
            name='camera_detector',
            output='screen',
            parameters=[{
                'env': LaunchConfiguration('env'),
                'device': LaunchConfiguration('device'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # ── Fusion ──────────────────────────────────────────────────────
        Node(
            package='fsai_perception',
            executable='fusion',
            name='fusion',
            output='screen',
            parameters=[{
                'env': LaunchConfiguration('env'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # ── RViz (optional) ─────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('visualize')),
        ),
    ])
