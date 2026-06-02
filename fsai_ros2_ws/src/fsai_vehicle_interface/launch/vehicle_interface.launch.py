"""
vehicle_interface.launch.py
───────────────────────────
Launches the fsai_vehicle_interface node with parameters from the default
config file.  Override parameters on the command line:

  ros2 launch fsai_vehicle_interface vehicle_interface.launch.py \
    can_interface:=vcan0 \
    stale_command_timeout_ms:=200

Available arguments
───────────────────
  can_interface            (str,  default: can0)  — SocketCAN interface name
  stale_command_timeout_ms (int,  default: 100)   — drive command stale threshold [ms]
  imu_frame_id             (str,  default: imu)   — frame_id for /vcu/imu
  gps_frame_id             (str,  default: gps)   — frame_id for /vcu/gps
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Declare overridable launch arguments ──────────────────────────────────
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface name (e.g. can0, vcan0)",
    )

    stale_timeout_arg = DeclareLaunchArgument(
        "stale_command_timeout_ms",
        default_value="100",
        description="Age [ms] at which a drive command is considered stale",
    )

    imu_frame_arg = DeclareLaunchArgument(
        "imu_frame_id",
        default_value="imu",
        description="frame_id for /vcu/imu messages",
    )

    gps_frame_arg = DeclareLaunchArgument(
        "gps_frame_id",
        default_value="gps",
        description="frame_id for /vcu/gps messages",
    )

    # ── Node ─────────────────────────────────────────────────────────────────
    vehicle_interface_node = Node(
        package="fsai_vehicle_interface",
        executable="vehicle_interface_node",
        name="vehicle_interface",
        output="screen",
        emulate_tty=True,           # coloured RCLCPP_* output in terminal
        parameters=[
            # Load defaults from YAML first
            PathJoinSubstitution([
                FindPackageShare("fsai_vehicle_interface"),
                "config",
                "params.yaml",
            ]),
            # Then override with launch arguments (these take precedence)
            {
                "can_interface":            LaunchConfiguration("can_interface"),
                "stale_command_timeout_ms": LaunchConfiguration("stale_command_timeout_ms"),
                "imu_frame_id":             LaunchConfiguration("imu_frame_id"),
                "gps_frame_id":             LaunchConfiguration("gps_frame_id"),
            },
        ],
    )

    return LaunchDescription([
        can_interface_arg,
        stale_timeout_arg,
        imu_frame_arg,
        gps_frame_arg,
        vehicle_interface_node,
    ])
