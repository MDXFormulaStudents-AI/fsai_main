"""Skidpad navigation — geometric path generator with orange-cone crossing detection.

State machine
─────────────
  APPROACH   Drive straight. Detect orange cones → lock crossing centroid.
  RIGHT_LOOP Publish a rolling clockwise arc for the right circle (loop_count_target loops).
  LEFT_LOOP  Publish a rolling CCW arc for the left circle (loop_count_target loops).
  EXIT       Drive straight ahead and stop.
  STOPPED    Publish nothing; local_path_follower applies its stop brake.

In-loop circle centre correction
──────────────────────────────────
  While traversing RIGHT_LOOP or LEFT_LOOP, every /cones message is examined for
  blue and yellow cones forming the track boundary.  Two methods are available
  (both enabled by default, results merged):

    Delaunay  — scipy.spatial.Delaunay triangulates all visible blue+yellow cones.
                Edges that cross between colours and have length ≈ track width
                give midpoints.  Works robustly at oblique view angles.

    Pairing   — each blue cone is matched to its nearest yellow cone; the
                midpoint is used.  Fast fallback, noisier at angle.

  When enough consistent midpoints are found ahead of the vehicle, the active
  circle centre is nudged toward them with a small gain (correction_gain).
  This corrects the odom drift that accumulates over the loop without
  depending on crossing orange-cone detection during motion.

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
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Delaunay triangulation — used for in-loop midpoint correction.
# Falls back to pairing-only if scipy is unavailable.
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

        # ── In-loop circle centre correction ───────────────────────────────
        # Enable the two midpoint strategies independently.
        self.declare_parameter('correction_use_delaunay', True)
        self.declare_parameter('correction_use_pairing', True)
        # Minimum / maximum forward distance (Fr1A X) to accept a midpoint.
        self.declare_parameter('correction_min_ahead', 0.5)
        self.declare_parameter('correction_max_ahead', 10.0)
        # Expected track width for Delaunay edge filtering (m).
        self.declare_parameter('correction_track_width_min', 1.5)
        self.declare_parameter('correction_track_width_max', 5.0)
        # Minimum number of midpoints required before a correction is applied.
        self.declare_parameter('correction_min_midpoints', 2)
        # Fraction of the computed error to apply per correction step (0–1).
        self.declare_parameter('correction_gain', 0.2)
        # Ignore corrections smaller than this (m) — deadband to suppress noise.
        self.declare_parameter('correction_deadband', 0.15)
        # Maximum single correction step (m) — safety clamp.
        self.declare_parameter('correction_max_step', 0.5)

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

        self._corr_use_delaunay = bool(self.get_parameter('correction_use_delaunay').value)
        self._corr_use_pairing = bool(self.get_parameter('correction_use_pairing').value)
        self._corr_min_ahead = float(self.get_parameter('correction_min_ahead').value)
        self._corr_max_ahead = float(self.get_parameter('correction_max_ahead').value)
        self._corr_tw_min = float(self.get_parameter('correction_track_width_min').value)
        self._corr_tw_max = float(self.get_parameter('correction_track_width_max').value)
        self._corr_min_mid = int(self.get_parameter('correction_min_midpoints').value)
        self._corr_gain = float(self.get_parameter('correction_gain').value)
        self._corr_deadband = float(self.get_parameter('correction_deadband').value)
        self._corr_max_step = float(self.get_parameter('correction_max_step').value)

        # ── State ──────────────────────────────────────────────────────────
        self._state = SkidpadState.APPROACH

        # Vehicle odom pose
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._heading: float = 0.0
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

        # Debug: last computed correction midpoints (odom frame) for display
        self._last_midpoints_odom: list[tuple[float, float]] = []

        # ── ROS ────────────────────────────────────────────────────────────
        self.create_subscription(Cone3DArray, cones_topic, self._on_cones, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)

        self.get_logger().info(
            f'Skidpad path node ready  '
            f'radius={self._circle_radius:.3f} m  loops={self._loop_count_target}  '
            f'path={path_topic}  '
            f'scipy={"yes" if _SCIPY_OK else "no (pairing only)"}'
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

        # ── In-loop circle centre correction (RIGHT_LOOP / LEFT_LOOP) ──
        elif self._state in (SkidpadState.RIGHT_LOOP, SkidpadState.LEFT_LOOP):
            self._apply_correction(msg)

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

        self._arc_progress = 0.0
        self._prev_arc_angle = None
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
                self._state = SkidpadState.LEFT_LOOP

        elif self._state == SkidpadState.LEFT_LOOP:
            if self._arc_progress >= full_rotation:
                self.get_logger().info(
                    f'Left loops done  '
                    f'(arc={math.degrees(self._arc_progress):.0f}°) → EXIT'
                )
                self._state = SkidpadState.EXIT
                self._exit_dist_traveled = 0.0

    # ── In-loop circle centre correction ───────────────────────────────────

    def _apply_correction(self, msg: Cone3DArray) -> None:
        """
        Compute track centreline midpoints from visible blue/yellow cones and
        nudge the active circle centre toward them to correct odom drift.
        """
        blue_fr1a: list[tuple[float, float]] = []
        yellow_fr1a: list[tuple[float, float]] = []

        for cone in msg.cones:
            name = cone.class_name.lower()
            fx = float(cone.position.x)
            fy = float(cone.position.y)

            # Forward-only gate: only cones ahead of the vehicle
            if fx < self._corr_min_ahead or fx > self._corr_max_ahead:
                continue

            if 'blue' in name:
                blue_fr1a.append((fx, fy))
            elif 'yellow' in name:
                yellow_fr1a.append((fx, fy))

        if not blue_fr1a or not yellow_fr1a:
            self._last_midpoints_odom = []
            return

        # Collect midpoints in Fr1A frame
        midpoints_fr1a: list[tuple[float, float]] = []

        if self._corr_use_delaunay and _SCIPY_OK:
            midpoints_fr1a.extend(
                self._delaunay_midpoints(blue_fr1a, yellow_fr1a)
            )

        if self._corr_use_pairing:
            midpoints_fr1a.extend(
                self._pair_midpoints(blue_fr1a, yellow_fr1a)
            )

        if not midpoints_fr1a:
            self._last_midpoints_odom = []
            return

        # Transform Fr1A → odom
        cos_h = math.cos(self._heading)
        sin_h = math.sin(self._heading)
        midpoints_odom = [
            (
                self._vx + mx * cos_h - my * sin_h,
                self._vy + mx * sin_h + my * cos_h,
            )
            for mx, my in midpoints_fr1a
        ]
        self._last_midpoints_odom = midpoints_odom

        # Compute corrections for each midpoint relative to active circle centre
        if self._state == SkidpadState.RIGHT_LOOP:
            cx, cy = self._right_cx, self._right_cy
        else:
            cx, cy = self._left_cx, self._left_cy

        corrections: list[tuple[float, float]] = []
        for px, py in midpoints_odom:
            dx = px - cx
            dy = py - cy
            d = math.hypot(dx, dy)
            if d < 1e-3:
                continue
            error = d - self._circle_radius
            if abs(error) < self._corr_deadband:
                continue
            # Clamp correction step size
            step = min(abs(error), self._corr_max_step) * (1.0 if error > 0 else -1.0)
            ux, uy = dx / d, dy / d
            corrections.append((step * ux, step * uy))

        if len(corrections) < self._corr_min_mid:
            return

        avg_cx = sum(c[0] for c in corrections) / len(corrections)
        avg_cy = sum(c[1] for c in corrections) / len(corrections)

        if self._state == SkidpadState.RIGHT_LOOP:
            self._right_cx += self._corr_gain * avg_cx
            self._right_cy += self._corr_gain * avg_cy
        else:
            self._left_cx += self._corr_gain * avg_cx
            self._left_cy += self._corr_gain * avg_cy

        self.get_logger().debug(
            f'[correction] nudge=({self._corr_gain * avg_cx:.3f},'
            f'{self._corr_gain * avg_cy:.3f}) from {len(corrections)} midpoints'
        )

    def _delaunay_midpoints(
        self,
        blue: list[tuple[float, float]],
        yellow: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        """
        Delaunay triangulate all visible cones and return midpoints of
        cross-colour edges whose length is within the expected track width band.
        """
        all_pts = blue + yellow
        n_blue = len(blue)
        if len(all_pts) < 3:
            return []

        pts_arr = _np.array(all_pts, dtype=float)
        try:
            tri = _Delaunay(pts_arr)
        except Exception:
            return []

        midpoints: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()

        for simplex in tri.simplices:
            for i in range(3):
                for j in range(i + 1, 3):
                    a, b = int(simplex[i]), int(simplex[j])
                    edge = (min(a, b), max(a, b))
                    if edge in seen:
                        continue
                    seen.add(edge)

                    a_blue = a < n_blue
                    b_blue = b < n_blue
                    if a_blue == b_blue:
                        continue  # same colour, skip

                    pa = all_pts[a]
                    pb = all_pts[b]
                    length = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
                    if not (self._corr_tw_min <= length <= self._corr_tw_max):
                        continue

                    mid_x = (pa[0] + pb[0]) * 0.5
                    mid_y = (pa[1] + pb[1]) * 0.5
                    if self._corr_min_ahead <= mid_x <= self._corr_max_ahead:
                        midpoints.append((mid_x, mid_y))

        return midpoints

    def _pair_midpoints(
        self,
        blue: list[tuple[float, float]],
        yellow: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        """
        For each blue cone find the nearest yellow cone and return the midpoint
        of the pair if the distance is within the track width band.
        """
        midpoints: list[tuple[float, float]] = []
        for bx, by in blue:
            best_d = float('inf')
            best_yx, best_yy = 0.0, 0.0
            for yx, yy in yellow:
                d = math.hypot(yx - bx, yy - by)
                if d < best_d:
                    best_d = d
                    best_yx, best_yy = yx, yy
            if not (self._corr_tw_min <= best_d <= self._corr_tw_max):
                continue
            mid_x = (bx + best_yx) * 0.5
            mid_y = (by + best_yy) * 0.5
            if self._corr_min_ahead <= mid_x <= self._corr_max_ahead:
                midpoints.append((mid_x, mid_y))
        return midpoints

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

    def _publish_tick(self) -> None:
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
            f'progress: {progress_pct:.0f}%  '
            f'midpoints: {len(self._last_midpoints_odom)}'
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

        # ── Correction midpoints during loops ──
        if self._last_midpoints_odom and self._state in (
            SkidpadState.RIGHT_LOOP, SkidpadState.LEFT_LOOP
        ):
            mid_m = self._base_marker(31, stamp, 'correction_midpoints', Marker.SPHERE_LIST)
            mid_m.scale.x = 0.25
            mid_m.scale.y = 0.25
            mid_m.scale.z = 0.25
            mid_m.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.85)
            mid_m.lifetime.nanosec = 300_000_000
            for mx, my in self._last_midpoints_odom:
                mid_m.points.append(self._point(mx, my, 0.20))
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
