"""Dynamic mission supervisor: lap counting and completion for the ADS-DV.

Sibling of ``static_profile_executor``. It does NOT command the vehicle. It watches
the selected mission and the interface state, counts laps for Autocross/Trackdrive by
start/finish orange-cone crossings, aggregates the intrinsic completion signals of the
Acceleration and Skidpad pipelines, and raises a single ``/dynamic_mission_complete``
that ``MissionManager`` forwards to ``/vehicle/mission_complete``.

Lap counting: the start/finish gate is located once (the first strong orange cluster is
transformed into the odom frame and pinned there), then a lap is counted each time the
car leaves the gate (beyond ``lap_rearm_radius_m``) and returns to it (within
``lap_gate_radius_m``). The car starting at the line does not count — it must arm by
leaving first.
"""

from __future__ import annotations

import math

from fsai_interfaces.msg import Cone3DArray, InterfaceState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DynamicMissionExecutor(Node):
    """Counts laps and declares completion for the four dynamic missions."""

    def __init__(self) -> None:
        super().__init__('dynamic_mission_executor')

        self.declare_parameter('mission_topic', '/mission/selected')
        self.declare_parameter('interface_state_topic', '/vehicle/interface_state')
        self.declare_parameter('cones_topic', '/cones')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('complete_topic', '/dynamic_mission_complete')
        self.declare_parameter('accel_finished_topic', '/accel/finished')
        self.declare_parameter('skidpad_finished_topic', '/skidpad/finished')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('autocross_laps', 1)
        self.declare_parameter('trackdrive_laps', 10)

        # Orange start/finish gate detection (vehicle frame).
        self.declare_parameter('orange_min_range', 0.5)
        self.declare_parameter('orange_max_range', 15.0)
        self.declare_parameter('orange_max_angle_deg', 60.0)
        self.declare_parameter('min_orange_cluster_size', 2)
        self.declare_parameter('orange_cluster_radius', 2.0)

        # Lap counting geometry (odom frame).
        self.declare_parameter('lap_gate_radius_m', 4.0)
        self.declare_parameter('lap_rearm_radius_m', 8.0)
        # Re-anchor the gate to the latest orange detection seen at least this far
        # ahead (m) — keeps the gate fixed to the true cones despite odom drift,
        # while ignoring parallax while passing through it.
        self.declare_parameter('gate_anchor_min_ahead_m', 2.0)

        self._mission_topic = str(self.get_parameter('mission_topic').value)
        self._complete_topic = str(self.get_parameter('complete_topic').value)
        self._autocross_laps = int(self.get_parameter('autocross_laps').value)
        self._trackdrive_laps = int(self.get_parameter('trackdrive_laps').value)
        self._orange_min_range = float(self.get_parameter('orange_min_range').value)
        self._orange_max_range = float(self.get_parameter('orange_max_range').value)
        self._orange_max_angle_rad = math.radians(float(self.get_parameter('orange_max_angle_deg').value))
        self._min_orange_cluster_size = int(self.get_parameter('min_orange_cluster_size').value)
        self._orange_cluster_radius = float(self.get_parameter('orange_cluster_radius').value)
        self._lap_gate_radius_m = float(self.get_parameter('lap_gate_radius_m').value)
        self._lap_rearm_radius_m = float(self.get_parameter('lap_rearm_radius_m').value)
        self._gate_anchor_min_ahead_m = float(self.get_parameter('gate_anchor_min_ahead_m').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        # Run state.
        self._selected_mission = 'none'
        self._interface_state = InterfaceState.WAIT_FOR_VCU
        self._car_x = 0.0
        self._car_y = 0.0
        self._car_yaw = 0.0
        self._have_odom = False
        self._gate: tuple[float, float] | None = None
        self._armed = False
        self._laps = 0
        self._accel_finished = False
        self._skidpad_finished = False
        self._complete_sent = False

        self.create_subscription(String, self._mission_topic, self._on_mission, 10)
        self.create_subscription(
            InterfaceState,
            str(self.get_parameter('interface_state_topic').value),
            self._on_interface_state,
            10,
        )
        self.create_subscription(Cone3DArray, str(self.get_parameter('cones_topic').value), self._on_cones, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10)
        self.create_subscription(Bool, str(self.get_parameter('accel_finished_topic').value), self._on_accel_finished, 10)
        self.create_subscription(Bool, str(self.get_parameter('skidpad_finished_topic').value), self._on_skidpad_finished, 10)

        self._complete_pub = self.create_publisher(Bool, self._complete_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._tick)

        self.get_logger().info(
            '── Dynamic Mission Executor ready  '
            f'(autocross={self._autocross_laps} lap, trackdrive={self._trackdrive_laps} laps) ──'
        )

    # ── Inputs ──────────────────────────────────────────────────────────────

    def _on_mission(self, msg: String) -> None:
        if msg.data == self._selected_mission:
            return
        self.get_logger().info(f'Dynamic mission: {self._selected_mission} -> {msg.data}')
        self._selected_mission = msg.data
        self._reset_run()

    def _on_interface_state(self, msg: InterfaceState) -> None:
        was_driving = self._interface_state == InterfaceState.DRIVING
        self._interface_state = int(msg.state)
        if was_driving and self._interface_state != InterfaceState.DRIVING:
            # Leaving DRIVING aborts the current run; a fresh DRIVING restarts it.
            self._reset_run()

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self._car_x = position.x
        self._car_y = position.y
        self._car_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self._have_odom = True

    def _on_cones(self, msg: Cone3DArray) -> None:
        # Lap-counted missions re-anchor the gate to the orange cones on every
        # approach, so accumulated odom drift never breaks lap counting.
        if self._selected_mission not in ('autocross', 'trackdrive'):
            return
        if not self._have_odom or self._interface_state != InterfaceState.DRIVING:
            return
        centroid = self._orange_centroid_vehicle(msg)
        if centroid is None:
            return
        fx, fy = centroid
        # Only anchor from a forward view; ignore parallax while passing through.
        if fx < self._gate_anchor_min_ahead_m:
            return
        cos_y, sin_y = math.cos(self._car_yaw), math.sin(self._car_yaw)
        gate = (
            self._car_x + fx * cos_y - fy * sin_y,
            self._car_y + fx * sin_y + fy * cos_y,
        )
        first = self._gate is None
        self._gate = gate
        if first:
            self.get_logger().info(f'Start/finish gate anchored at odom ({gate[0]:.1f}, {gate[1]:.1f})')

    def _on_accel_finished(self, msg: Bool) -> None:
        if msg.data and self._selected_mission == 'acceleration':
            self._accel_finished = True

    def _on_skidpad_finished(self, msg: Bool) -> None:
        if msg.data and self._selected_mission == 'skidpad':
            self._skidpad_finished = True

    # ── Completion logic ────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._complete_sent:
            return

        mission = self._selected_mission
        if mission == 'acceleration':
            if self._accel_finished:
                self._declare_complete('acceleration distance reached')
        elif mission == 'skidpad':
            if self._skidpad_finished:
                self._declare_complete('skidpad loops complete')
        elif mission in ('autocross', 'trackdrive'):
            self._update_laps()
            target = self._autocross_laps if mission == 'autocross' else self._trackdrive_laps
            if self._laps >= target:
                self._declare_complete(f'{mission} reached {self._laps}/{target} laps')

    def _update_laps(self) -> None:
        if self._gate is None or not self._have_odom:
            return
        if self._interface_state != InterfaceState.DRIVING:
            return
        distance = math.hypot(self._car_x - self._gate[0], self._car_y - self._gate[1])
        if distance > self._lap_rearm_radius_m:
            self._armed = True
        elif self._armed and distance < self._lap_gate_radius_m:
            self._armed = False
            self._laps += 1
            self.get_logger().info(f'Lap {self._laps} (crossed start/finish gate)')

    def _declare_complete(self, reason: str) -> None:
        self._complete_sent = True
        self.get_logger().info(f'Dynamic mission complete: {reason}')
        self._complete_pub.publish(Bool(data=True))

    def _reset_run(self) -> None:
        self._gate = None
        self._armed = False
        self._laps = 0
        self._accel_finished = False
        self._skidpad_finished = False
        self._complete_sent = False

    # ── Orange gate detection (vehicle frame) ───────────────────────────────

    def _orange_centroid_vehicle(self, msg: Cone3DArray) -> tuple[float, float] | None:
        orange: list[tuple[float, float]] = []
        for cone in msg.cones:
            if 'orange' not in cone.class_name.lower():
                continue
            fx = cone.position.x
            fy = cone.position.y
            if abs(math.atan2(fy, fx)) > self._orange_max_angle_rad:
                continue
            dist = math.hypot(fx, fy)
            if dist < self._orange_min_range or dist > self._orange_max_range:
                continue
            orange.append((fx, fy))

        if len(orange) < self._min_orange_cluster_size:
            return None
        cluster = self._largest_cluster(orange, self._orange_cluster_radius)
        if len(cluster) < self._min_orange_cluster_size:
            return None
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return cx, cy

    @staticmethod
    def _largest_cluster(points: list[tuple[float, float]], radius: float) -> list[tuple[float, float]]:
        best: list[tuple[float, float]] = []
        for anchor in points:
            group = [p for p in points if math.hypot(p[0] - anchor[0], p[1] - anchor[1]) <= radius]
            if len(group) > len(best):
                best = group
        return best

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DynamicMissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
