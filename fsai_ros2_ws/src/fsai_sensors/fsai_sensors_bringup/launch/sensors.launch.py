"""MDX FSAI sensor bringup — launches all active sensors with a single command."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Launch ZED2 stereo camera (set false when camera not connected)',
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Launch Velodyne VLP-16 LiDAR (set false when LiDAR not connected)',
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2',
        description='ZED camera model (zed, zedm, zed2, zed2i, zedx, zedxm)',
    )

    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.1.201',
        description='IP address of the VLP-16 LiDAR',
    )

    zed_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('zed_wrapper'),
                        'launch',
                        'zed_camera.launch.py',
                    )
                ),
                launch_arguments={
                    'camera_model': LaunchConfiguration('camera_model'),
                }.items(),
            )
        ],
    )

    velodyne_driver = Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        parameters=[{
            'device_ip': LaunchConfiguration('lidar_ip'),
            'port': 2368,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'velodyne',
        }],
    )

    velodyne_pointcloud = Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_pointcloud',
        parameters=[{
            'model': 'VLP16',
            'calibration': os.path.join(
                get_package_share_directory('velodyne_pointcloud'),
                'params',
                'VLP16db.yaml',
            ),
            'min_range': 0.4,
            'max_range': 100.0,
        }],
    )

    return LaunchDescription([
        enable_camera_arg,
        enable_lidar_arg,
        camera_model_arg,
        lidar_ip_arg,
        zed_launch,
        velodyne_driver,
        velodyne_pointcloud,
    ])
