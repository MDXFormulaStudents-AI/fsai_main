"""MDX FSAI sensor bringup — launches all active sensors with a single command."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2',
        description='ZED camera model (zed, zedm, zed2, zed2i, zedx, zedxm)',
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items(),
    )

    # ── LiDAR (not yet fitted) ────────────────────────────────────────────────
    # Uncomment and configure when LiDAR driver package is available:
    #
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('<lidar_package>'),
    #             'launch',
    #             '<lidar_launch_file>.launch.py',
    #         )
    #     ),
    # )

    return LaunchDescription([
        camera_model_arg,
        zed_launch,
        # lidar_launch,
    ])
