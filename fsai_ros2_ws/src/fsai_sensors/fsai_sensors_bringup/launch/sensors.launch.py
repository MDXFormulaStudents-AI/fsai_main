"""MDX FSAI sensor bringup — launches all active sensors with a single command."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_mounts(config_dir: str) -> dict:
    with open(os.path.join(config_dir, 'sensor_mounts.yaml')) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = get_package_share_directory('fsai_sensors_bringup')
    config_dir = os.path.join(pkg, 'config')
    mounts = _load_mounts(config_dir)

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

    # ── ZED2 camera ───────────────────────────────────────────────────────────
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

    zed_tf = Node(
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed2_tf',
        arguments=[
            '--x', str(mounts['zed2']['x']),
            '--y', str(mounts['zed2']['y']),
            '--z', str(mounts['zed2']['z']),
            '--yaw', str(mounts['zed2']['yaw']),
            '--pitch', str(mounts['zed2']['pitch']),
            '--roll', str(mounts['zed2']['roll']),
            '--frame-id', mounts['zed2']['parent_frame'],
            '--child-frame-id', mounts['zed2']['child_frame'],
        ],
    )

    # ── Velodyne VLP-16 LiDAR ─────────────────────────────────────────────────
    vlp16_params = os.path.join(config_dir, 'vlp16.yaml')

    velodyne_driver = Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        parameters=[vlp16_params],
    )

    velodyne_pointcloud = Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_pointcloud',
        parameters=[vlp16_params],
    )

    velodyne_tf = Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_tf',
        arguments=[
            '--x', str(mounts['velodyne']['x']),
            '--y', str(mounts['velodyne']['y']),
            '--z', str(mounts['velodyne']['z']),
            '--yaw', str(mounts['velodyne']['yaw']),
            '--pitch', str(mounts['velodyne']['pitch']),
            '--roll', str(mounts['velodyne']['roll']),
            '--frame-id', mounts['velodyne']['parent_frame'],
            '--child-frame-id', mounts['velodyne']['child_frame'],
        ],
    )

    return LaunchDescription([
        enable_camera_arg,
        enable_lidar_arg,
        camera_model_arg,
        zed_launch,
        zed_tf,
        velodyne_driver,
        velodyne_pointcloud,
        velodyne_tf,
    ])
