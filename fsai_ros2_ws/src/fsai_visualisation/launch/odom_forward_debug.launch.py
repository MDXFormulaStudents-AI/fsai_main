"""Launch CarMaker odom TF alignment and the odom-forward RViz profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rviz_script = PathJoinSubstitution([
        LaunchConfiguration('workspace_dir'),
        'Start_RViz.sh',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('workspace_dir', default_value='/home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws'),
        DeclareLaunchArgument('rviz_profile', default_value='odom_forward'),
        DeclareLaunchArgument('odom_topic', default_value='/carmaker/odom'),
        DeclareLaunchArgument('aligned_frame_id', default_value='home'),
        DeclareLaunchArgument('fr1a_frame_id', default_value='Fr1A'),
        DeclareLaunchArgument('alignment_mode', default_value='initial_pose'),
        DeclareLaunchArgument('base_link_to_fr1a_x', default_value='0.0'),
        DeclareLaunchArgument('base_link_to_fr1a_y', default_value='0.0'),
        DeclareLaunchArgument('base_link_to_fr1a_z', default_value='0.0'),
        DeclareLaunchArgument('base_link_to_fr1a_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('base_link_to_fr1a_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('base_link_to_fr1a_yaw_deg', default_value='0.0'),

        Node(
            package='fsai_visualisation',
            executable='carmaker_odom_tf',
            name='carmaker_odom_tf',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': True,
                'publish_aligned_frame': True,
                'aligned_frame_id': LaunchConfiguration('aligned_frame_id'),
                'alignment_mode': LaunchConfiguration('alignment_mode'),
                'publish_parent_to_aligned_frame': True,
                'publish_base_link_to_fr1a_tf': True,
                'fr1a_frame_id': LaunchConfiguration('fr1a_frame_id'),
                'base_link_to_fr1a_x': LaunchConfiguration('base_link_to_fr1a_x'),
                'base_link_to_fr1a_y': LaunchConfiguration('base_link_to_fr1a_y'),
                'base_link_to_fr1a_z': LaunchConfiguration('base_link_to_fr1a_z'),
                'base_link_to_fr1a_roll_deg': LaunchConfiguration('base_link_to_fr1a_roll_deg'),
                'base_link_to_fr1a_pitch_deg': LaunchConfiguration('base_link_to_fr1a_pitch_deg'),
                'base_link_to_fr1a_yaw_deg': LaunchConfiguration('base_link_to_fr1a_yaw_deg'),
                'publish_debug_markers': True,
                'publish_trace_path': True,
            }],
        ),

        ExecuteProcess(
            cmd=['bash', rviz_script, LaunchConfiguration('rviz_profile')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
