"""Skidpad navigation — geometric path generator with orange-cone crossing detection.

State machine
─────────────
  APPROACH   Drive straight. Detect orange cones → lock crossing centroid.
  RIGHT_LOOP Publish a rolling clockwise arc for the right circle (loop_count_target loops).
  LEFT_LOOP  Publish a rolling CCW arc for the left circle (loop_count_target loops).
  EXIT       Drive straight ahead and stop.
  STOPPED    Publish nothing; local_path_follower applies its stop brake.

Loop geometry
─────────────
  The orange crossing centroid anchors both skidpad circles once.  After that,
  the path generator follows right/left circle centres.  Optional mid-loop
  detection can nudge the active centre using blue/yellow cone midpoints, but
  only inside a middle arc window and only for cones that lie in the expected
  circular track band.  This avoids latching onto entry/exit straight cones.

Loop counting
─────────────
  Cumulative signed arc around the active circle centre.
    CW  (right): accumulates negative → threshold = -(2π × loop_count_target)
    CCW (left):  accumulates positive → threshold = +(2π × loop_count_target)

Path frame
──────────
  Path is published in the odom frame.  local_path_follower looks up the
  odom → Fr1A TF from the carmaker_odom_tf chain at runtime.
"""

from __future__ import annotations

import enum
import math

from fsai_interfaces.msg import Cone3DArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import numpy as _np
    from scipy.spatial import Delaunay as _Delaunay
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


class SkidpadState(enum.Enum):
    APPROACH = 'approach'
    RIGHT_LOOP = 'right_loop'
    LEFT_LOOP = 'left_loop'
    EXIT = 'exit'
    STOPPED = 'stopped'


class SkidpadPath(Node):
    """Publishes the skidpad driving line for local_path_follower to track."""

    def __init__(self) -> None:
        super().__init__('skidpad_path')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('cones_topic', '/cones')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('path_topic', '/nav/skidpad_path')
        self.declare_parameter('debug_topic', '/nav/skidpad_path_markers')
        # Mission self-gating (CSV of /mission/selected values; empty = always
        # publish). In the dynamic stack set mission_gates:="skidpad" and
        # path_topic:=/nav/active_path. finished_topic signals loops complete.
        self.declare_parameter('mission_topic', '/mission/selected')
        self.declare_parameter('mission_gates', '')
        self.declare_parameter('finished_topic', '/skidpad/finished')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate_hz', 10.0)

        # Path shape
        self.declare_parameter('path_point_spacing', 0.5)
        self.declare_parameter('lookahead_horizon', 20.0)

        # Track geometry
        self.declare_parameter('circle_radius', 9.125)

        # Orange-cone crossing detection
        self.declare_parameter('min_approach_dist', 20.0)
        self.declare_parameter('max_approach_dist', 60.0)
        self.declare_parameter('orange_min_range', 1.0)
        self.declare_parameter('orange_max_range', 15.0)
        self.declare_parameter('orange_max_angle_deg', 45.0)
        self.declare_parameter('orange_cluster_radius', 8.0)
        self.declare_parameter('min_orange_cluster_size', 2)
        self.declare_parameter('crossing_hold_m', 0.5)

        # Loop control
        self.declare_parameter('loop_count_target', 2)

        # Exit
        self.declare_parameter('exit_distance', 20.0)
        # Speed (m/s) below which the vehicle is considered stopped. /skidpad/finished
        # is only published once the car is actually at rest in STOPPED, so the
        # DRIVING gate stays open (follower keeps braking) until then — mirrors the
        # acceleration controller's stop check and satisfies spec §3.3.
        self.declare_parameter('finish_stop_speed', 0.3)

        # Mid-loop detection/correction. Method: off, pairing, delaunay, both, single_side.
        self.declare_parameter('mid_loop_detection_method', 'delaunay')
        self.declare_parameter('mid_loop_detection_start_deg', 75.0)
        self.declare_parameter('mid_loop_detection_end_deg', 285.0)
        self.declare_parameter('mid_loop_detection_min_ahead', 0.5)
        self.declare_parameter('mid_loop_detection_max_ahead', 10.0)
        self.declare_parameter('mid_loop_detection_track_width_min', 1.5)
        self.declare_parameter('mid_loop_detection_track_width_max', 5.0)
        self.declare_parameter('mid_loop_detection_radial_margin', 1.2)
        self.declare_parameter('mid_loop_detection_min_midpoints', 2)
        self.declare_parameter('mid_loop_detection_gain', 0.15)
        self.declare_parameter('mid_loop_detection_deadband', 0.20)
        self.declare_parameter('mid_loop_detection_max_step', 0.35)
        self.declare_parameter('mid_loop_single_side_track_width', 3.0)
        self.declare_parameter('mid_loop_single_side_radial_tolerance', 1.5)
        self.declare_parameter('mid_loop_single_side_min_cones', 1)

        # ── Read params ────────────────────────────────────────────────────
        cones_topic = str(self.get_parameter('cones_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._spacing = float(self.get_parameter('path_point_spacing').value)
        self._horizon = float(self.get_parameter('lookahead_horizon').value)
        self._circle_radius = float(self.get_parameter('circle_radius').value)

        self._min_approach_dist = float(self.get_parameter('min_approach_dist').value)
        self._max_approach_dist = float(self.get_parameter('max_approach_dist').value)
        self._orange_min_range = float(self.get_parameter('orange_min_range').value)
        self._orange_max_range = float(self.get_parameter('orange_max_range').value)
        self._orange_max_angle_rad = math.radians(
            float(self.get_parameter('orange_max_angle_deg').value))
        self._orange_cluster_radius = float(self.get_parameter('orange_cluster_radius').value)
        self._min_orange_cluster_size = int(self.get_parameter('min_orange_cluster_size').value)
        self._crossing_hold_m = float(self.get_parameter('crossing_hold_m').value)

        self._loop_count_target = int(self.get_parameter('loop_count_target').value)
        self._exit_distance = float(self.get_parameter('exit_distance').value)
        self._finish_stop_speed = float(self.get_parameter('finish_stop_speed').value)

        self._mid_loop_method = str(self.get_parameter('mid_loop_detection_method').value).lower()
        # 'none'/'disabled' are accepted aliases for 'off'. Prefer them on the CLI:
        # YAML 1.1 (used by ROS 2 param parsing) coerces the bare word 'off' to the
        # boolean False, which fails the string type check on declare_parameter.
        if self._mid_loop_method in ('none', 'disabled'):
            self._mid_loop_method = 'off'
        if self._mid_loop_method not in ('off', 'pairing', 'delaunay', 'both', 'single_side'):
            self.get_logger().warn(
                f'Unknown mid_loop_detection_method={self._mid_loop_method!r}; using off.'
            )
            self._mid_loop_method = 'off'
        self._mid_loop_start_rad = math.radians(
            float(self.get_parameter('mid_loop_detection_start_deg').value) % 360.0)
        self._mid_loop_end_rad = math.radians(
            float(self.get_parameter('mid_loop_detection_end_deg').value) % 360.0)
        self._mid_loop_min_ahead = float(self.get_parameter('mid_loop_detection_min_ahead').value)
        self._mid_loop_max_ahead = float(self.get_parameter('mid_loop_detection_max_ahead').value)
        self._mid_loop_tw_min = float(self.get_parameter('mid_loop_detection_track_width_min').value)
        self._mid_loop_tw_max = float(self.get_parameter('mid_loop_detection_track_width_max').value)
        self._mid_loop_radial_margin = float(self.get_parameter('mid_loop_detection_radial_margin').value)
        self._mid_loop_min_midpoints = int(self.get_parameter('mid_loop_detection_min_midpoints').value)
        self._mid_loop_gain = float(self.get_parameter('mid_loop_detection_gain').value)
        self._mid_loop_deadband = float(self.get_parameter('mid_loop_detection_deadband').value)
        self._mid_loop_max_step = float(self.get_parameter('mid_loop_detection_max_step').value)
        self._single_side_track_width = float(self.get_parameter('mid_loop_single_side_track_width').value)
        self._single_side_radial_tolerance = float(
            self.get_parameter('mid_loop_single_side_radial_tolerance').value)
        self._single_side_min_cones = int(self.get_parameter('mid_loop_single_side_min_cones').value)

        # ── State ──────────────────────────────────────────────────────────
        self._state = SkidpadState.APPROACH

        # Vehicle odom pose
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._heading: float = 0.0
        self._current_speed: float = 0.0
        self._odom_ready: bool = False

        # Odometry distance accumulator
        self._prev_pos: tuple[float, float] | None = None
        self._odom_distance: float = 0.0

        # Crossing anchor
        self._crossing_x: float = 0.0
        self._crossing_y: float = 0.0
        self._approach_heading: float = 0.0
        self._right_cx: float = 0.0
        self._right_cy: float = 0.0
        self._left_cx: float = 0.0
        self._left_cy: float = 0.0
        self._right_start_angle: float = 0.0
        self._left_start_angle: float = 0.0
        self._crossing_confirmed: bool = False

        # Orange centroid (locked on first detection after gate arms)
        self._orange_centroid: tuple[float, float] | None = None
        self._orange_cluster_points: list[tuple[float, float]] = []

        # Arc angle tracking
        self._arc_progress: float = 0.0
        self._prev_arc_angle: float | None = None

        # Crossing approach log gates
        self._logged_5m: bool = False
        self._logged_2m: bool = False

        # Exit distance tracking
        self._exit_dist_traveled: float = 0.0

        # Mid-loop detection debug
        self._mid_loop_status: str = 'inactive'
        self._last_mid_loop_cones_odom: list[tuple[float, float]] = []
        self._last_mid_loop_midpoints_odom: list[tuple[float, float]] = []
        self._last_mid_loop_delaunay_edges_odom: list[
            tuple[tuple[float, float], tuple[float, float]]
        ] = []
        self._last_mid_loop_delaunay_used_edges_odom: list[
            tuple[tuple[float, float], tuple[float, float]]
        ] = []
        self._last_mid_loop_single_side_vectors_odom: list[
            tuple[tuple[float, float], tuple[float, float]]
        ] = []

        gates_raw = str(self.get_parameter('mission_gates').value)
        self._mission_gates = {m.strip() for m in gates_raw.split(',') if m.strip()}
        self._selected_mission = 'none'
        self._finished_sent = False
        mission_topic = str(self.get_parameter('mission_topic').value)
        finished_topic = str(self.get_parameter('finished_topic').value)

        # ── ROS ────────────────────────────────────────────────────────────
        self.create_subscription(Cone3DArray, cones_topic, self._on_cones, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, mission_topic, self._on_mission, 10)
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._finished_pub = self.create_publisher(Bool, finished_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)

        self.get_logger().info(
            f'Skidpad path node ready  '
            f'radius={self._circle_radius:.3f} m  loops={self._loop_count_target}  '
            f'path={path_topic}  '
            f'mid_loop={self._mid_loop_method}  scipy={"yes" if _SCIPY_OK else "no"}'
        )

    # ── Odom callback ───────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        heading = 2.0 * math.atan2(qz, qw)

        if self._prev_pos is not None:
            dx = x - self._prev_pos[0]
            dy = y - self._prev_pos[1]
            step = math.hypot(dx, dy)
            self._odom_distance += step
            if self._state == SkidpadState.EXIT:
                self._exit_dist_traveled += step

        self._prev_pos = (x, y)
        self._vx = x
        self._vy = y
        self._heading = heading
        self._current_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )
        self._odom_ready = True

        self._update_arc_progress(x, y)

        # ── Crossing confirmation check (odom-driven) ──────────────────────
        if (
            self._state == SkidpadState.APPROACH
            and self._odom_distance >= self._min_approach_dist
            and self._orange_centroid is not None
        ):
            cx_odom, cy_odom = self._orange_centroid
            fwd_x = math.cos(heading)
            fwd_y = math.sin(heading)
            fwd_offset = (cx_odom - x) * fwd_x + (cy_odom - y) * fwd_y

            if fwd_offset < 5.0 and not self._logged_5m:
                self._logged_5m = True
                self.get_logger().info(
                    f'[crossing] 5 m gate: vehicle=({x:.2f},{y:.2f})  '
                    f'centroid=({cx_odom:.2f},{cy_odom:.2f})  fwd_offset={fwd_offset:.2f} m'
                )
            if fwd_offset < 2.0 and not self._logged_2m:
                self._logged_2m = True
                self.get_logger().info(
                    f'[crossing] 2 m gate: vehicle=({x:.2f},{y:.2f})  '
                    f'centroid=({cx_odom:.2f},{cy_odom:.2f})  fwd_offset={fwd_offset:.2f} m'
                )

            if fwd_offset <= self._crossing_hold_m:
                self._confirm_crossing(cx_odom, cy_odom, heading)

        # Exit completion
        if self._state == SkidpadState.EXIT:
            if self._exit_dist_traveled >= self._exit_distance:
                self._state = SkidpadState.STOPPED
                self.get_logger().info('Skidpad complete — transitioned to STOPPED.')

    # ── Cone callback ───────────────────────────────────────────────────────

    def _on_cones(self, msg: Cone3DArray) -> None:
        if not self._odom_ready:
            return

        # ── Initial centroid detection (APPROACH only) ──────────────────
        if self._state == SkidpadState.APPROACH:
            if self._odom_distance < self._min_approach_dist:
                self._orange_centroid = None
                self._orange_cluster_points = []
                return
            centroid = self._detect_orange_cluster(msg)
            if centroid is not None and self._orange_centroid is None:
                self._orange_centroid = centroid
                self.get_logger().info(
                    f'[centroid] locked: ({centroid[0]:.2f}, {centroid[1]:.2f})  '
                    f'vehicle=({self._vx:.2f},{self._vy:.2f})  '
                    f'odom_dist={self._odom_distance:.1f} m'
                )

        elif self._state in (SkidpadState.RIGHT_LOOP, SkidpadState.LEFT_LOOP):
            self._apply_mid_loop_detection(msg)

    # ── Orange cone detection ───────────────────────────────────────────────

    def _detect_orange_cluster(self, msg: Cone3DArray) -> tuple[float, float] | None:
        cos_h = math.cos(self._heading)
        sin_h = math.sin(self._heading)
        orange_odom: list[tuple[float, float]] = []
        self._orange_cluster_points = []

        for cone in msg.cones:
            if 'orange' not in cone.class_name.lower():
                continue
            fx = cone.position.x
            fy = cone.position.y
            angle_from_fwd = math.atan2(fy, fx)
            if abs(angle_from_fwd) > self._orange_max_angle_rad:
                continue
            dist = math.hypot(fx, fy)
            if dist < self._orange_min_range or dist > self._orange_max_range:
                continue
            ox = self._vx + fx * cos_h - fy * sin_h
            oy = self._vy + fx * sin_h + fy * cos_h
            orange_odom.append((ox, oy))

        if not orange_odom:
            return None

        cluster = self._largest_cluster(orange_odom, self._orange_cluster_radius)
        if len(cluster) < self._min_orange_cluster_size:
            return None

        self._orange_cluster_points = cluster
        centroid_x = sum(p[0] for p in cluster) / len(cluster)
        centroid_y = sum(p[1] for p in cluster) / len(cluster)
        return centroid_x, centroid_y

    @staticmethod
    def _largest_cluster(
        points: list[tuple[float, float]],
        radius: float,
    ) -> list[tuple[float, float]]:
        best: list[tuple[float, float]] = []
        for anchor in points:
            group = [
                p for p in points
                if math.hypot(p[0] - anchor[0], p[1] - anchor[1]) <= radius
            ]
            if len(group) > len(best):
                best = group
        return best

    # ── Crossing confirmation ───────────────────────────────────────────────

    def _confirm_crossing(self, cx: float, cy: float, heading: float) -> None:
        self._crossing_x = cx
        self._crossing_y = cy
        self._approach_heading = heading
        self._crossing_confirmed = True

        right_x = math.sin(heading)
        right_y = -math.cos(heading)

        r = self._circle_radius
        self._right_cx = cx + right_x * r
        self._right_cy = cy + right_y * r
        self._left_cx = cx - right_x * r
        self._left_cy = cy - right_y * r
        self._right_start_angle = math.atan2(cy - self._right_cy, cx - self._right_cx)
        self._left_start_angle = math.atan2(cy - self._left_cy, cx - self._left_cx)

        self._arc_progress = 0.0
        self._prev_arc_angle = None
        self._clear_mid_loop_debug('waiting')
        self._state = SkidpadState.RIGHT_LOOP

        self.get_logger().info(
            f'Crossing confirmed at ({cx:.2f}, {cy:.2f})  '
            f'heading={math.degrees(heading):.1f}°\n'
            f'  Right circle centre: ({self._right_cx:.2f}, {self._right_cy:.2f})\n'
            f'  Left  circle centre: ({self._left_cx:.2f}, {self._left_cy:.2f})\n'
            f'  → RIGHT_LOOP'
        )

    # ── Arc angle / loop counting ───────────────────────────────────────────

    def _update_arc_progress(self, vx: float, vy: float) -> None:
        if self._state == SkidpadState.RIGHT_LOOP:
            cx, cy = self._right_cx, self._right_cy
        elif self._state == SkidpadState.LEFT_LOOP:
            cx, cy = self._left_cx, self._left_cy
        else:
            return

        angle = math.atan2(vy - cy, vx - cx)

        if self._prev_arc_angle is not None:
            delta = angle - self._prev_arc_angle
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            self._arc_progress += delta

        self._prev_arc_angle = angle

        full_rotation = 2.0 * math.pi * self._loop_count_target

        if self._state == SkidpadState.RIGHT_LOOP:
            if self._arc_progress <= -full_rotation:
                self.get_logger().info(
                    f'Right loops done  '
                    f'(arc={math.degrees(self._arc_progress):.0f}°) → LEFT_LOOP'
                )
                self._arc_progress = 0.0
                self._prev_arc_angle = None
                self._clear_mid_loop_debug('waiting')
                self._state = SkidpadState.LEFT_LOOP

        elif self._state == SkidpadState.LEFT_LOOP:
            if self._arc_progress >= full_rotation:
                self.get_logger().info(
                    f'Left loops done  '
                    f'(arc={math.degrees(self._arc_progress):.0f}°) → EXIT'
                )
                self._clear_mid_loop_debug('inactive')
                self._state = SkidpadState.EXIT
                self._exit_dist_traveled = 0.0

    # ── Mid-loop detection / correction ────────────────────────────────────

    def _clear_mid_loop_debug(self, status: str) -> None:
        self._mid_loop_status = status
        self._last_mid_loop_cones_odom = []
        self._last_mid_loop_midpoints_odom = []
        self._last_mid_loop_delaunay_edges_odom = []
        self._last_mid_loop_delaunay_used_edges_odom = []
        self._last_mid_loop_single_side_vectors_odom = []

    def _apply_mid_loop_detection(self, msg: Cone3DArray) -> None:
        self._last_mid_loop_cones_odom = []
        self._last_mid_loop_midpoints_odom = []
        self._last_mid_loop_delaunay_edges_odom = []
        self._last_mid_loop_delaunay_used_edges_odom = []
        self._last_mid_loop_single_side_vectors_odom = []

        if self._mid_loop_method == 'off':
            self._mid_loop_status = 'off'
            return

        current_phase = self._current_lap_phase_rad()
        if not self._phase_in_mid_loop_window(current_phase):
            self._mid_loop_status = (
                f'waiting {math.degrees(current_phase):.0f} deg '
                f'[{math.degrees(self._mid_loop_start_rad):.0f},'
                f'{math.degrees(self._mid_loop_end_rad):.0f}]'
            )
            return

        active_center = self._active_circle_center()
        if active_center is None:
            self._mid_loop_status = 'inactive'
            return
        cx, cy = active_center

        blue_fr1a: list[tuple[float, float]] = []
        yellow_fr1a: list[tuple[float, float]] = []
        accepted_odom: list[tuple[float, float]] = []
        accepted_colored_odom: list[tuple[str, float, float]] = []

        for cone in msg.cones:
            name = cone.class_name.lower()
            is_blue = 'blue' in name
            is_yellow = 'yellow' in name
            if not (is_blue or is_yellow):
                continue

            fx = float(cone.position.x)
            fy = float(cone.position.y)
            if fx < self._mid_loop_min_ahead or fx > self._mid_loop_max_ahead:
                continue

            ox, oy = self._fr1a_to_odom(fx, fy)
            if not self._cone_in_mid_loop_window(ox, oy, cx, cy):
                continue

            cone_phase = self._point_loop_phase_rad(ox, oy, cx, cy)
            if not self._phase_in_mid_loop_window(cone_phase):
                continue

            accepted_odom.append((ox, oy))
            if is_blue:
                accepted_colored_odom.append(('blue', ox, oy))
                blue_fr1a.append((fx, fy))
            else:
                accepted_colored_odom.append(('yellow', ox, oy))
                yellow_fr1a.append((fx, fy))

        self._last_mid_loop_cones_odom = accepted_odom
        if self._mid_loop_method == 'single_side':
            midpoints_odom, vectors_odom = self._single_side_midpoints(
                accepted_colored_odom,
                cx,
                cy,
            )
            self._last_mid_loop_midpoints_odom = midpoints_odom
            self._last_mid_loop_single_side_vectors_odom = vectors_odom
            if len(midpoints_odom) < self._single_side_min_cones:
                self._mid_loop_status = (
                    f'single_side accepted={len(accepted_colored_odom)} '
                    f'midpoints={len(midpoints_odom)}'
                )
                return
            self._apply_midpoint_center_nudge(
                midpoints_odom,
                cx,
                cy,
                min_required=self._single_side_min_cones,
            )
            return

        if not blue_fr1a or not yellow_fr1a:
            self._mid_loop_status = f'active no boundary pair cones={len(accepted_odom)}'
            return

        midpoints_fr1a: list[tuple[float, float]] = []
        if self._mid_loop_method in ('delaunay', 'both'):
            if _SCIPY_OK:
                delaunay_midpoints, delaunay_edges, delaunay_used_edges = (
                    self._delaunay_midpoints_and_edges(blue_fr1a, yellow_fr1a)
                )
                midpoints_fr1a.extend(delaunay_midpoints)
                self._last_mid_loop_delaunay_edges_odom = self._edges_fr1a_to_odom(
                    delaunay_edges
                )
                self._last_mid_loop_delaunay_used_edges_odom = self._edges_fr1a_to_odom(
                    delaunay_used_edges
                )
            elif self._mid_loop_method == 'delaunay':
                self._mid_loop_status = 'delaunay unavailable'
                return

        if self._mid_loop_method in ('pairing', 'both'):
            midpoints_fr1a.extend(self._pair_midpoints(blue_fr1a, yellow_fr1a))

        midpoints_fr1a = self._dedupe_points(midpoints_fr1a)
        midpoints_odom: list[tuple[float, float]] = []
        for mx, my in midpoints_fr1a:
            ox, oy = self._fr1a_to_odom(mx, my)
            if not self._midpoint_in_mid_loop_window(ox, oy, cx, cy):
                continue
            midpoint_phase = self._point_loop_phase_rad(ox, oy, cx, cy)
            if not self._phase_in_mid_loop_window(midpoint_phase):
                continue
            midpoints_odom.append((ox, oy))

        self._last_mid_loop_midpoints_odom = midpoints_odom
        if len(midpoints_odom) < self._mid_loop_min_midpoints:
            self._mid_loop_status = f'active midpoints={len(midpoints_odom)}'
            return

        self._apply_midpoint_center_nudge(midpoints_odom, cx, cy)

    def _active_circle_center(self) -> tuple[float, float] | None:
        if self._state == SkidpadState.RIGHT_LOOP:
            return self._right_cx, self._right_cy
        if self._state == SkidpadState.LEFT_LOOP:
            return self._left_cx, self._left_cy
        return None

    def _current_lap_phase_rad(self) -> float:
        if self._state == SkidpadState.RIGHT_LOOP:
            return (-self._arc_progress) % (2.0 * math.pi)
        if self._state == SkidpadState.LEFT_LOOP:
            return self._arc_progress % (2.0 * math.pi)
        return 0.0

    def _point_loop_phase_rad(self, x: float, y: float, cx: float, cy: float) -> float:
        angle = math.atan2(y - cy, x - cx)
        if self._state == SkidpadState.RIGHT_LOOP:
            return (self._right_start_angle - angle) % (2.0 * math.pi)
        if self._state == SkidpadState.LEFT_LOOP:
            return (angle - self._left_start_angle) % (2.0 * math.pi)
        return 0.0

    def _phase_in_mid_loop_window(self, phase_rad: float) -> bool:
        if self._mid_loop_start_rad <= self._mid_loop_end_rad:
            return self._mid_loop_start_rad <= phase_rad <= self._mid_loop_end_rad
        return phase_rad >= self._mid_loop_start_rad or phase_rad <= self._mid_loop_end_rad

    def _fr1a_to_odom(self, fx: float, fy: float) -> tuple[float, float]:
        cos_h = math.cos(self._heading)
        sin_h = math.sin(self._heading)
        return (
            self._vx + fx * cos_h - fy * sin_h,
            self._vy + fx * sin_h + fy * cos_h,
        )

    def _edges_fr1a_to_odom(
        self,
        edges: list[tuple[tuple[float, float], tuple[float, float]]],
    ) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        return [
            (self._fr1a_to_odom(a[0], a[1]), self._fr1a_to_odom(b[0], b[1]))
            for a, b in edges
        ]

    def _cone_in_mid_loop_window(self, x: float, y: float, cx: float, cy: float) -> bool:
        radius = math.hypot(x - cx, y - cy)
        half_width = 0.5 * self._mid_loop_tw_max
        return (
            self._circle_radius - half_width - self._mid_loop_radial_margin
            <= radius
            <= self._circle_radius + half_width + self._mid_loop_radial_margin
        )

    def _midpoint_in_mid_loop_window(self, x: float, y: float, cx: float, cy: float) -> bool:
        radius = math.hypot(x - cx, y - cy)
        return abs(radius - self._circle_radius) <= self._mid_loop_radial_margin

    def _single_side_midpoints(
        self,
        colored_cones_odom: list[tuple[str, float, float]],
        cx: float,
        cy: float,
    ) -> tuple[
        list[tuple[float, float]],
        list[tuple[tuple[float, float], tuple[float, float]]],
    ]:
        half_width = 0.5 * self._single_side_track_width
        midpoints: list[tuple[float, float]] = []
        vectors: list[tuple[tuple[float, float], tuple[float, float]]] = []

        for colour, px, py in colored_cones_odom:
            outside = self._single_side_cone_is_outside(colour)
            if outside is None:
                continue

            dx = px - cx
            dy = py - cy
            radius = math.hypot(dx, dy)
            if radius < 1e-3:
                continue

            expected_radius = self._circle_radius + half_width if outside else self._circle_radius - half_width
            if abs(radius - expected_radius) > self._single_side_radial_tolerance:
                continue

            ux = dx / radius
            uy = dy / radius
            direction = -1.0 if outside else 1.0
            mx = px + direction * ux * half_width
            my = py + direction * uy * half_width

            midpoint_radius = math.hypot(mx - cx, my - cy)
            if abs(midpoint_radius - self._circle_radius) > self._single_side_radial_tolerance:
                continue
            midpoint_phase = self._point_loop_phase_rad(mx, my, cx, cy)
            if not self._phase_in_mid_loop_window(midpoint_phase):
                continue

            midpoints.append((mx, my))
            vectors.append(((px, py), (mx, my)))

        return self._dedupe_points(midpoints), vectors

    def _single_side_cone_is_outside(self, colour: str) -> bool | None:
        if self._state == SkidpadState.RIGHT_LOOP:
            if colour == 'blue':
                return True
            if colour == 'yellow':
                return False
        elif self._state == SkidpadState.LEFT_LOOP:
            if colour == 'yellow':
                return True
            if colour == 'blue':
                return False
        return None

    def _apply_midpoint_center_nudge(
        self,
        midpoints_odom: list[tuple[float, float]],
        cx: float,
        cy: float,
        min_required: int | None = None,
    ) -> None:
        required = self._mid_loop_min_midpoints if min_required is None else max(1, min_required)
        corrections: list[tuple[float, float]] = []
        for px, py in midpoints_odom:
            dx = px - cx
            dy = py - cy
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                continue
            error = dist - self._circle_radius
            if abs(error) < self._mid_loop_deadband:
                continue
            step = min(abs(error), self._mid_loop_max_step) * (1.0 if error > 0.0 else -1.0)
            corrections.append((step * dx / dist, step * dy / dist))

        if len(corrections) < required:
            self._mid_loop_status = f'active no nudge midpoints={len(midpoints_odom)}'
            return

        avg_x = sum(c[0] for c in corrections) / len(corrections)
        avg_y = sum(c[1] for c in corrections) / len(corrections)
        nudge_x = self._mid_loop_gain * avg_x
        nudge_y = self._mid_loop_gain * avg_y

        if self._state == SkidpadState.RIGHT_LOOP:
            self._right_cx += nudge_x
            self._right_cy += nudge_y
            self._right_start_angle = math.atan2(
                self._crossing_y - self._right_cy,
                self._crossing_x - self._right_cx,
            )
        elif self._state == SkidpadState.LEFT_LOOP:
            self._left_cx += nudge_x
            self._left_cy += nudge_y
            self._left_start_angle = math.atan2(
                self._crossing_y - self._left_cy,
                self._crossing_x - self._left_cx,
            )

        self._prev_arc_angle = None
        self._mid_loop_status = (
            f'nudged {len(corrections)} {self._mid_loop_method} '
            f'({nudge_x:.2f},{nudge_y:.2f})'
        )

    def _delaunay_midpoints_and_edges(
        self,
        blue: list[tuple[float, float]],
        yellow: list[tuple[float, float]],
    ) -> tuple[
        list[tuple[float, float]],
        list[tuple[tuple[float, float], tuple[float, float]]],
        list[tuple[tuple[float, float], tuple[float, float]]],
    ]:
        points = blue + yellow
        n_blue = len(blue)
        if len(points) < 3:
            return [], [], []

        try:
            triangulation = _Delaunay(_np.array(points, dtype=float))
        except Exception:
            return [], [], []

        midpoints: list[tuple[float, float]] = []
        all_edges: list[tuple[tuple[float, float], tuple[float, float]]] = []
        used_edges: list[tuple[tuple[float, float], tuple[float, float]]] = []
        seen_edges: set[tuple[int, int]] = set()
        for simplex in triangulation.simplices:
            for i in range(3):
                for j in range(i + 1, 3):
                    a = int(simplex[i])
                    b = int(simplex[j])
                    edge = (min(a, b), max(a, b))
                    if edge in seen_edges:
                        continue
                    seen_edges.add(edge)

                    ax, ay = points[a]
                    bx, by = points[b]
                    edge_points = ((ax, ay), (bx, by))
                    all_edges.append(edge_points)

                    a_is_blue = a < n_blue
                    b_is_blue = b < n_blue
                    if a_is_blue == b_is_blue:
                        continue

                    width = math.hypot(bx - ax, by - ay)
                    if not (self._mid_loop_tw_min <= width <= self._mid_loop_tw_max):
                        continue
                    used_edges.append(edge_points)
                    midpoints.append(((ax + bx) * 0.5, (ay + by) * 0.5))
        return midpoints, all_edges, used_edges

    def _pair_midpoints(
        self,
        blue: list[tuple[float, float]],
        yellow: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        midpoints: list[tuple[float, float]] = []
        for bx, by in blue:
            best_dist = float('inf')
            best_yellow: tuple[float, float] | None = None
            for yx, yy in yellow:
                dist = math.hypot(yx - bx, yy - by)
                if dist < best_dist:
                    best_dist = dist
                    best_yellow = (yx, yy)
            if best_yellow is None:
                continue
            if not (self._mid_loop_tw_min <= best_dist <= self._mid_loop_tw_max):
                continue
            yx, yy = best_yellow
            midpoints.append(((bx + yx) * 0.5, (by + yy) * 0.5))
        return midpoints

    @staticmethod
    def _dedupe_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        deduped: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()
        for x, y in points:
            key = (round(x * 20.0), round(y * 20.0))
            if key in seen:
                continue
            seen.add(key)
            deduped.append((x, y))
        return deduped

    # ── Path generation ─────────────────────────────────────────────────────

    def _generate_path_points(self) -> list[tuple[float, float]]:
        if not self._odom_ready:
            return []

        vx, vy = self._vx, self._vy

        if self._state == SkidpadState.APPROACH:
            return self._straight_points(vx, vy, self._heading, self._horizon)

        elif self._state == SkidpadState.RIGHT_LOOP:
            theta = math.atan2(vy - self._right_cy, vx - self._right_cx)
            return self._arc_points(
                self._right_cx, self._right_cy,
                self._circle_radius, theta, clockwise=True,
            )

        elif self._state == SkidpadState.LEFT_LOOP:
            theta = math.atan2(vy - self._left_cy, vx - self._left_cx)
            return self._arc_points(
                self._left_cx, self._left_cy,
                self._circle_radius, theta, clockwise=False,
            )

        elif self._state == SkidpadState.EXIT:
            remaining = self._exit_distance - self._exit_dist_traveled
            if remaining < 0.5:
                return []
            return self._straight_points(
                vx, vy, self._approach_heading, min(remaining, self._horizon),
            )

        else:  # STOPPED
            return []

    def _straight_points(
        self,
        x: float,
        y: float,
        heading: float,
        length: float,
    ) -> list[tuple[float, float]]:
        n = max(2, int(length / self._spacing) + 1)
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        return [
            (x + i * self._spacing * cos_h, y + i * self._spacing * sin_h)
            for i in range(n)
        ]

    def _arc_points(
        self,
        cx: float,
        cy: float,
        radius: float,
        start_theta: float,
        clockwise: bool,
    ) -> list[tuple[float, float]]:
        n = max(2, int(self._horizon / self._spacing) + 1)
        d_theta = self._spacing / radius
        sign = -1.0 if clockwise else 1.0
        return [
            (
                cx + radius * math.cos(start_theta + sign * i * d_theta),
                cy + radius * math.sin(start_theta + sign * i * d_theta),
            )
            for i in range(n)
        ]

    # ── Publish tick ────────────────────────────────────────────────────────

    def _on_mission(self, msg: String) -> None:
        mission = msg.data
        if mission == self._selected_mission:
            return
        self._selected_mission = mission
        # Re-arm each time this node's mission becomes active so a second run
        # (after a completed run + recovery) restarts cleanly from APPROACH
        # instead of staying latched in STOPPED with _finished_sent set.
        if not self._mission_gated_out():
            self._reset_run_state()
            self.get_logger().info('Skidpad selected — re-armed for a new run')

    def _reset_run_state(self) -> None:
        """Reset the run state machine to a fresh APPROACH (re-arm)."""
        self._state = SkidpadState.APPROACH
        self._prev_pos = None
        self._odom_distance = 0.0
        self._crossing_x = 0.0
        self._crossing_y = 0.0
        self._approach_heading = 0.0
        self._right_cx = 0.0
        self._right_cy = 0.0
        self._left_cx = 0.0
        self._left_cy = 0.0
        self._right_start_angle = 0.0
        self._left_start_angle = 0.0
        self._crossing_confirmed = False
        self._orange_centroid = None
        self._orange_cluster_points = []
        self._arc_progress = 0.0
        self._prev_arc_angle = None
        self._logged_5m = False
        self._logged_2m = False
        self._exit_dist_traveled = 0.0
        self._finished_sent = False

    def _mission_gated_out(self) -> bool:
        return bool(self._mission_gates) and self._selected_mission not in self._mission_gates

    def _publish_tick(self) -> None:
        if self._mission_gated_out():
            return

        if (
            self._state == SkidpadState.APPROACH
            and self._odom_distance > self._max_approach_dist
            and self._odom_ready
        ):
            self.get_logger().warn(
                'Orange cone crossing not detected within max_approach_dist — '
                'using current odom position as crossing (fallback).'
            )
            self._confirm_crossing(self._vx, self._vy, self._heading)

        points = self._generate_path_points()
        self._path_pub.publish(self._make_path_msg(points))
        self._debug_pub.publish(self._make_debug_markers(points))

        # Signal loops complete once STOPPED *and* the vehicle is actually at rest.
        # Holding here (skidpad publishes no path in STOPPED → follower keeps
        # braking) keeps the DRIVING gate open until the car stops, so we don't
        # send mission-complete while still rolling.
        if (
            self._state == SkidpadState.STOPPED
            and not self._finished_sent
            and self._current_speed <= self._finish_stop_speed
        ):
            self._finished_sent = True
            self._finished_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'Skidpad stopped ({self._current_speed:.2f} m/s) — mission finished'
            )

    # ── Message builders ────────────────────────────────────────────────────

    def _make_path_msg(self, points: list[tuple[float, float]]) -> Path:
        msg = Path()
        msg.header.frame_id = self._odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _make_debug_markers(self, path_points: list[tuple[float, float]]) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        delete = Marker()
        delete.header.frame_id = self._odom_frame
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        # ── Status text ──
        text = self._base_marker(0, stamp, 'skidpad_status', Marker.TEXT_VIEW_FACING)
        text.pose.position.x = self._vx
        text.pose.position.y = self._vy
        text.pose.position.z = 2.5
        text.scale.z = 0.35
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.lifetime.nanosec = 300_000_000

        full_rotation = 2.0 * math.pi * self._loop_count_target
        progress_pct = 0.0
        if self._state in (SkidpadState.RIGHT_LOOP, SkidpadState.LEFT_LOOP):
            progress_pct = min(100.0, 100.0 * abs(self._arc_progress) / full_rotation)
        armed_text = 'armed' if self._odom_distance >= self._min_approach_dist else 'blocked'

        text.text = (
            f'SKIDPAD  |  {self._state.value.upper()}\n'
            f'odom dist: {self._odom_distance:.1f} m\n'
            f'orange gate: {armed_text} @ {self._min_approach_dist:.1f} m\n'
            f'arc: {math.degrees(abs(self._arc_progress)):.0f}°  '
            f'progress: {progress_pct:.0f}%\n'
            f'mid-loop: {self._mid_loop_status}  '
            f'cones: {len(self._last_mid_loop_cones_odom)}  '
            f'midpoints: {len(self._last_mid_loop_midpoints_odom)}  '
            f'single: {len(self._last_mid_loop_single_side_vectors_odom)}  '
            f'tri: {len(self._last_mid_loop_delaunay_edges_odom)}/'
            f'{len(self._last_mid_loop_delaunay_used_edges_odom)}'
        )
        if self._crossing_confirmed:
            text.text += f'\ncrossing: ({self._crossing_x:.1f}, {self._crossing_y:.1f})'
        markers.markers.append(text)

        # ── Vehicle heading arrow ──
        arrow = self._base_marker(20, stamp, 'vehicle_heading', Marker.ARROW)
        arrow.points.append(self._point(self._vx, self._vy, 0.25))
        arrow.points.append(
            self._point(
                self._vx + 2.5 * math.cos(self._heading),
                self._vy + 2.5 * math.sin(self._heading),
                0.25,
            )
        )
        arrow.scale.x = 0.12
        arrow.scale.y = 0.35
        arrow.scale.z = 0.35
        arrow.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        arrow.lifetime.nanosec = 300_000_000
        markers.markers.append(arrow)

        # ── Published path ──
        if len(path_points) >= 2:
            path_marker = self._base_marker(1, stamp, 'skidpad_path', Marker.LINE_STRIP)
            path_marker.scale.x = 0.12
            path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.45, a=1.0)
            path_marker.lifetime.nanosec = 300_000_000
            for x, y in path_points:
                path_marker.points.append(self._point(x, y, 0.12))
            markers.markers.append(path_marker)
            markers.markers.append(
                self._sphere_marker(21, stamp, 'path_start',
                                    path_points[0][0], path_points[0][1],
                                    0.20, 0.35, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            )
            markers.markers.append(
                self._sphere_marker(22, stamp, 'path_end',
                                    path_points[-1][0], path_points[-1][1],
                                    0.20, 0.45, ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0))
            )

        # ── Circle outlines (once confirmed) ──
        if self._crossing_confirmed:
            right_active = self._state == SkidpadState.RIGHT_LOOP
            left_active = self._state == SkidpadState.LEFT_LOOP
            markers.markers.append(
                self._circle_outline(2, stamp, 'right_circle',
                                     self._right_cx, self._right_cy,
                                     self._circle_radius,
                                     ColorRGBA(r=1.0, g=0.45, b=0.0,
                                               a=0.95 if right_active else 0.25),
                                     line_width=0.12 if right_active else 0.04)
            )
            markers.markers.append(
                self._circle_outline(3, stamp, 'left_circle',
                                     self._left_cx, self._left_cy,
                                     self._circle_radius,
                                     ColorRGBA(r=0.2, g=0.5, b=1.0,
                                               a=0.95 if left_active else 0.25),
                                     line_width=0.12 if left_active else 0.04)
            )
            markers.markers.append(
                self._sphere_marker(23, stamp, 'right_circle_center',
                                    self._right_cx, self._right_cy,
                                    0.08, 0.35, ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.9))
            )
            markers.markers.append(
                self._sphere_marker(24, stamp, 'left_circle_center',
                                    self._left_cx, self._left_cy,
                                    0.08, 0.35, ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9))
            )
            markers.markers.append(
                self._text_marker(25, stamp, 'circle_labels',
                                  self._right_cx, self._right_cy,
                                  0.75, 'RIGHT\nCW',
                                  ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0))
            )
            markers.markers.append(
                self._text_marker(26, stamp, 'circle_labels',
                                  self._left_cx, self._left_cy,
                                  0.75, 'LEFT\nCCW',
                                  ColorRGBA(r=0.35, g=0.65, b=1.0, a=1.0))
            )
            markers.markers.append(
                self._sphere_marker(4, stamp, 'crossing_center',
                                    self._crossing_x, self._crossing_y,
                                    0.12, 0.65,
                                    ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
                                    lifetime_ns=0)
            )
            markers.markers.append(
                self._text_marker(27, stamp, 'crossing_label',
                                  self._crossing_x, self._crossing_y,
                                  1.0, 'CROSSING',
                                  ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0),
                                  lifetime_ns=0)
            )

        # ── Orange centroid during approach ──
        if self._state == SkidpadState.APPROACH and self._orange_centroid is not None:
            if self._orange_cluster_points:
                cluster = self._base_marker(28, stamp, 'orange_cluster', Marker.SPHERE_LIST)
                cluster.scale.x = 0.30
                cluster.scale.y = 0.30
                cluster.scale.z = 0.30
                cluster.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.9)
                cluster.lifetime.nanosec = 300_000_000
                for x, y in self._orange_cluster_points:
                    cluster.points.append(self._point(x, y, 0.12))
                markers.markers.append(cluster)

            cx, cy = self._orange_centroid
            markers.markers.append(
                self._sphere_marker(5, stamp, 'orange_centroid', cx, cy,
                                    0.18, 0.45, ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.95))
            )
            markers.markers.append(
                self._text_marker(29, stamp, 'orange_centroid_label', cx, cy,
                                  0.8, 'orange\ncentroid',
                                  ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0))
            )
            line = self._base_marker(30, stamp, 'orange_centroid_vector', Marker.LINE_STRIP)
            line.scale.x = 0.04
            line.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.75)
            line.lifetime.nanosec = 300_000_000
            line.points.append(self._point(self._vx, self._vy, 0.10))
            line.points.append(self._point(cx, cy, 0.10))
            markers.markers.append(line)

        # ── Mid-loop detection debug ──
        if self._state in (SkidpadState.RIGHT_LOOP, SkidpadState.LEFT_LOOP):
            if self._last_mid_loop_delaunay_edges_odom:
                tri_m = self._base_marker(33, stamp, 'mid_loop_delaunay_edges', Marker.LINE_LIST)
                tri_m.scale.x = 0.025
                tri_m.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.45)
                tri_m.lifetime.nanosec = 300_000_000
                for start, end in self._last_mid_loop_delaunay_edges_odom:
                    tri_m.points.append(self._point(start[0], start[1], 0.14))
                    tri_m.points.append(self._point(end[0], end[1], 0.14))
                markers.markers.append(tri_m)

            if self._last_mid_loop_delaunay_used_edges_odom:
                used_m = self._base_marker(34, stamp, 'mid_loop_delaunay_used_edges', Marker.LINE_LIST)
                used_m.scale.x = 0.07
                used_m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.95)
                used_m.lifetime.nanosec = 300_000_000
                for start, end in self._last_mid_loop_delaunay_used_edges_odom:
                    used_m.points.append(self._point(start[0], start[1], 0.28))
                    used_m.points.append(self._point(end[0], end[1], 0.28))
                markers.markers.append(used_m)

            if self._last_mid_loop_single_side_vectors_odom:
                single_m = self._base_marker(35, stamp, 'mid_loop_single_side_vectors', Marker.LINE_LIST)
                single_m.scale.x = 0.08
                single_m.color = ColorRGBA(r=1.0, g=0.2, b=0.9, a=0.95)
                single_m.lifetime.nanosec = 300_000_000
                for start, end in self._last_mid_loop_single_side_vectors_odom:
                    single_m.points.append(self._point(start[0], start[1], 0.32))
                    single_m.points.append(self._point(end[0], end[1], 0.32))
                markers.markers.append(single_m)

            if self._last_mid_loop_cones_odom:
                cone_m = self._base_marker(31, stamp, 'mid_loop_window_cones', Marker.SPHERE_LIST)
                cone_m.scale.x = 0.22
                cone_m.scale.y = 0.22
                cone_m.scale.z = 0.22
                cone_m.color = ColorRGBA(r=0.85, g=0.1, b=1.0, a=0.80)
                cone_m.lifetime.nanosec = 300_000_000
                for x, y in self._last_mid_loop_cones_odom:
                    cone_m.points.append(self._point(x, y, 0.18))
                markers.markers.append(cone_m)

            if self._last_mid_loop_midpoints_odom:
                mid_m = self._base_marker(32, stamp, 'mid_loop_midpoints', Marker.SPHERE_LIST)
                mid_m.scale.x = 0.28
                mid_m.scale.y = 0.28
                mid_m.scale.z = 0.28
                mid_m.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.90)
                mid_m.lifetime.nanosec = 300_000_000
                for x, y in self._last_mid_loop_midpoints_odom:
                    mid_m.points.append(self._point(x, y, 0.24))
                markers.markers.append(mid_m)

        return markers

    # ── Marker helpers ──────────────────────────────────────────────────────

    def _base_marker(self, mid: int, stamp, ns: str, mtype: int) -> Marker:
        m = Marker()
        m.header.frame_id = self._odom_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _point(x: float, y: float, z: float = 0.0) -> Point:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def _sphere_marker(
        self,
        mid: int,
        stamp,
        ns: str,
        x: float,
        y: float,
        z: float,
        diameter: float,
        colour: ColorRGBA,
        lifetime_ns: int = 300_000_000,
    ) -> Marker:
        m = self._base_marker(mid, stamp, ns, Marker.SPHERE)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = diameter
        m.color = colour
        m.lifetime.nanosec = lifetime_ns
        return m

    def _text_marker(
        self,
        mid: int,
        stamp,
        ns: str,
        x: float,
        y: float,
        z: float,
        text: str,
        colour: ColorRGBA,
        lifetime_ns: int = 300_000_000,
    ) -> Marker:
        m = self._base_marker(mid, stamp, ns, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.z = 0.35
        m.color = colour
        m.text = text
        m.lifetime.nanosec = lifetime_ns
        return m

    def _circle_outline(
        self,
        mid: int,
        stamp,
        ns: str,
        cx: float,
        cy: float,
        radius: float,
        colour: ColorRGBA,
        line_width: float = 0.06,
        n_seg: int = 72,
    ) -> Marker:
        m = self._base_marker(mid, stamp, ns, Marker.LINE_STRIP)
        m.scale.x = line_width
        m.color = colour
        m.lifetime.nanosec = 0
        for i in range(n_seg + 1):
            theta = 2.0 * math.pi * i / n_seg
            m.points.append(
                self._point(
                    cx + radius * math.cos(theta),
                    cy + radius * math.sin(theta),
                    0.04,
                )
            )
        return m


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SkidpadPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
