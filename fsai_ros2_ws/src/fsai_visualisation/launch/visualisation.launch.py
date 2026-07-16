"""
visualisation.launch.py — env-aware RViz + marker converters for the perception
pipeline.

Run it on the compute, OR on any laptop on the same ROS_DOMAIN_ID (42) and DDS
network to see everything the compute publishes:

  ros2 launch fsai_visualisation visualisation.launch.py env:=real           # real car / bag
  ros2 launch fsai_visualisation visualisation.launch.py env:=sim            # CarMaker
  ros2 launch fsai_visualisation visualisation.launch.py env:=real use_sim_time:=false   # LIVE real car

RViz cannot render the custom Cone3DArray message, so the marker converters
(cone_visualizer) run HERE — they subscribe to the compute's cone topics over the
network and republish visualization_msgs/MarkerArray locally for RViz. Always on:
  /cones                   -> /cone_markers         (fused, coloured cones)
  /perception/lidar/cones  -> /lidar_cone_markers   (raw LiDAR detections — debug, all grey)

RViz config selection: uses config/fsai_<env>.rviz if it exists, else config/fsai.rviz.
Override with rviz_config:=/abs/path.rviz. Add env-specific debug views in launch_setup().
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _cone_viz(name, input_topic, output_topic, use_sim_time, show_unknown, drop_car=False):
    """A cone_visualizer instance (Cone3DArray -> MarkerArray)."""
    # Each instance also publishes /car_marker; on secondary (debug) instances we
    # remap it away so it does not fight the primary instance's car marker.
    remaps = [('/car_marker', '/_unused/%s_car_marker' % name)] if drop_car else []
    return Node(
        package='fsai_visualisation', executable='cone_visualizer',
        name=name, output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic':  input_topic,
            'output_topic': output_topic,
            'show_unknown': show_unknown,
        }],
        remappings=remaps,
    )


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration('env').perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    show_unknown = ParameterValue(LaunchConfiguration('show_unknown'), value_type=bool)

    pkg_share = get_package_share_directory('fsai_visualisation')
    cfg_override = LaunchConfiguration('rviz_config').perform(context)
    if cfg_override:
        rviz_cfg = cfg_override
    else:
        env_cfg = os.path.join(pkg_share, 'config', 'fsai_%s.rviz' % env)
        rviz_cfg = env_cfg if os.path.exists(env_cfg) else os.path.join(pkg_share, 'config', 'fsai.rviz')

    # ── Always-on marker converters ────────────────────────────────────────
    nodes = [
        # Fused, coloured cones from the fusion node.
        _cone_viz('cone_visualizer', '/cones', '/cone_markers',
                  use_sim_time, show_unknown),
        # Raw LiDAR detections (pre-fusion) — a key debug tool. All grey, since
        # lidar_detector labels everything unknown_cone.
        _cone_viz('lidar_cone_visualizer', '/perception/lidar/cones', '/lidar_cone_markers',
                  use_sim_time, ParameterValue(True, value_type=bool), drop_car=True),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    # ── Env-specific debug visualisations ──────────────────────────────────
    # Add nodes that only make sense in one environment here. They are only
    # spawned for the matching env, so `env:=real` and `env:=sim` each get their
    # own tailored view. (Left as hooks — extend as debug needs grow.)
    if env == 'real':
        pass  # e.g. GPS track, /vcu diagnostics overlay, ZED odom path markers
    elif env == 'sim':
        pass  # e.g. CarMaker ground-truth cones / path overlay

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env', default_value='sim',
            description='sim | real — selects config/fsai_<env>.rviz (if present) and env-specific debug nodes'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='false for the LIVE car (default — no /clock exists); '
                        'set true only when monitoring a bag/CarMaker played with --clock'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='Override RViz .rviz path (default: auto-select fsai_<env>.rviz, else fsai.rviz)'),
        DeclareLaunchArgument(
            'show_unknown', default_value='true',
            description='Show LiDAR-only (uncoloured) cones on /cone_markers'),

        OpaqueFunction(function=launch_setup),
    ])
