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

    enable_fr1a_arg = DeclareLaunchArgument(
        'enable_fr1a',
        default_value='true',
        description='Publish the static base_link->Fr1A transform. Set false on the '
                    'camera-only sensors.launch instance (the ZED supervisor passes '
                    'this) so Fr1A has exactly one owner — the LiDAR instance — and a '
                    'ZED restart never touches the frame fusion depends on.',
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
                    # Kill the ZED's own TF broadcast. It publishes map->odom->zed_camera_link
                    # (VIO) at ~26Hz, which roots the whole ZED frame tree at a detached odom
                    # and overrides our base_link->zed_camera_link — so fusion could never
                    # resolve Fr1A->zed_camera_center. These are LAUNCH args (they override the
                    # yaml). With them off, zed_camera_link is free for sensors' zed2_tf to parent
                    # to base_link. We use wheel+IMU localization, not ZED VIO.
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
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

    # ── Fr1A vehicle-reference frame (static, NOT a sensor) ───────────────────
    # base_link -> Fr1A is fixed vehicle geometry, so it belongs with the mounts and
    # is published here — independent of odometry/localization — so perception/fusion
    # can transform into Fr1A from just sensors + perception. (vehicle_odometry.launch
    # no longer publishes Fr1A; this is its single home.)
    #
    # Gated on enable_fr1a so it has exactly ONE owner: start_fsai.sh runs sensors.launch
    # twice (lidar-only + camera-only), and the camera-only instance passes
    # enable_fr1a:=false. That keeps Fr1A on the LiDAR instance ONLY, so a ZED restart
    # (which bounces the camera instance) can never disturb the frame fusion depends on.
    fr1a_tf = Node(
        condition=IfCondition(LaunchConfiguration('enable_fr1a')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_fr1a',
        arguments=[
            '--x', str(mounts['fr1a']['x']),
            '--y', str(mounts['fr1a']['y']),
            '--z', str(mounts['fr1a']['z']),
            '--yaw', str(mounts['fr1a']['yaw']),
            '--pitch', str(mounts['fr1a']['pitch']),
            '--roll', str(mounts['fr1a']['roll']),
            '--frame-id', mounts['fr1a']['parent_frame'],
            '--child-frame-id', mounts['fr1a']['child_frame'],
        ],
    )

    return LaunchDescription([
        enable_camera_arg,
        enable_lidar_arg,
        enable_fr1a_arg,
        camera_model_arg,
        zed_launch,
        zed_tf,
        velodyne_driver,
        velodyne_pointcloud,
        velodyne_tf,
        fr1a_tf,
    ])
