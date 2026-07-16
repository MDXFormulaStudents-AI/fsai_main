"""
bag_replay.launch.py — dev helper for replaying a recorded bag through perception.

Two things this fixes for the recovered Bag_2:
  1. /velodyne_points in the bag is decimated (DB corruption kept ~46 of ~1351
     frames) → blinking. The raw /velodyne_packets survived intact, so we run the
     velodyne transform node to regenerate a FULL-RATE /velodyne_points from them.
  2. The bag's baked-in base_link->velodyne TF has a wrong 180 deg roll (the sensor
     is actually upright). We republish the CORRECTED extrinsics from
     sensor_mounts.yaml (now roll 0), plus base_link->Fr1A.

Play the bag WITHOUT its bad /velodyne_points and /tf_static so this launch's
outputs win. rosbag2 0.15.16 has no exclude flag, so use the inclusive --topics
complement:

  ros2 launch fsai_sensors_bringup bag_replay.launch.py

  BAG=/home/mdxfsai/fsai_main/Bag_2_fixed
  TOPICS=$(ros2 bag info "$BAG" | grep -oP 'Topic: \K\S+' \
           | grep -vE '^/velodyne_points$|^/tf_static$' | tr '\n' ' ')
  ros2 bag play "$BAG" --clock --topics $TOPICS

Then run perception (env:=real, use_sim_time:=true) + visualisation as usual.

NOTE: excluding /tf_static drops the deep ZED frames too, so camera projection on
the bag is limited to base_link->zed_camera_center until those are republished.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _st(name, m, parent=None, child=None):
    """static_transform_publisher Node from a sensor_mounts entry (or explicit)."""
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=[
            '--x', str(m['x']), '--y', str(m['y']), '--z', str(m['z']),
            '--roll', str(m.get('roll', 0.0)), '--pitch', str(m.get('pitch', 0.0)),
            '--yaw', str(m.get('yaw', 0.0)),
            '--frame-id', parent or m['parent_frame'],
            '--child-frame-id', child or m['child_frame'],
        ],
    )


def generate_launch_description():
    pkg = get_package_share_directory('fsai_sensors_bringup')
    cfg = os.path.join(pkg, 'config')
    mounts = yaml.safe_load(open(os.path.join(cfg, 'sensor_mounts.yaml')))
    vlp16_params = os.path.join(cfg, 'vlp16.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true for bag playback with --clock'),
        DeclareLaunchArgument('base_link_to_fr1a_x', default_value='0.5637',
                              description='rear-axle offset base_link->Fr1A (m)'),
        DeclareLaunchArgument('regen_cloud', default_value='true',
                              description='true: regenerate /velodyne_points from /velodyne_packets. '
                                          'false: TF-correction only (use the bag\'s own /velodyne_points).'),

        # Regenerate a full-rate /velodyne_points from the intact /velodyne_packets.
        # Skip with regen_cloud:=false when replaying the bag's own /velodyne_points
        # (otherwise this would double-publish and clash with the bag).
        Node(
            package='velodyne_pointcloud', executable='velodyne_transform_node',
            name='velodyne_pointcloud', output='screen',
            condition=IfCondition(LaunchConfiguration('regen_cloud')),
            parameters=[vlp16_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Corrected extrinsics (override the bag's bad /tf_static, which is excluded on play)
        _st('velodyne_tf', mounts['velodyne']),
        _st('zed2_tf',     mounts['zed2']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_link_to_fr1a',
            arguments=['--x', LaunchConfiguration('base_link_to_fr1a_x'),
                       '--frame-id', 'base_link', '--child-frame-id', 'Fr1A'],
        ),
    ])
