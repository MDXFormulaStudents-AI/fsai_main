"""Local ego-frame path follower for simulator VehicleControl commands."""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from fsai_interfaces.msg import DriveCommand
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, String
import tf2_ros
from vehiclecontrol_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class LocalTarget:
    point: Point
    control_point: Point
    requested_lookahead_distance: float
    effective_lookahead_distance: float
    curvature: float
    steer_angle: float       # desired road-wheel angle (controller output)
    published_steer: float   # steer_angle * steer_command_gain (what goes on steer_ang)
    gas: float
    brake: float
    target_speed: float = 0.0  # curvature-adjusted target speed [m/s] (for DriveCommand)
    # Diagnostics (see _compute_target). cross_track_error is the signed lateral
    # offset of the control point from the path (+left of path direction).
    # nearest_point is the closest path point in the control frame (for markers).
    # achieved_curvature is yaw_rate/speed from odom, None when speed is too low.
    cross_track_error: float = 0.0
    nearest_point: Point | None = None
    achieved_curvature: float | None = None
    # Steering-mode diagnostics. path_curvature is the signed curvature of a
    # circle fitted to the forward path (the source of the lock/feedforward
    # steady-state term); mode is the active steering law.
    path_curvature: float = 0.0
    mode: str = 'pursuit'


class LocalPathFollower(Node):
    """Follows a path with pure pursuit after transforming it into the vehicle frame."""

    def __init__(
        self,
        node_name: str = 'local_path_follower',
        default_path_topic: str = '/nav/perceived_path',
        default_debug_topic: str = '/nav/local_path_follower_markers',
        default_enabled: bool = False,
        display_name: str = 'Local path follower',
    ):
        super().__init__(node_name)
        self._display_name = display_name

        self.declare_parameter('path_topic', default_path_topic)
        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('debug_topic', default_debug_topic)
        self.declare_parameter('control_frame', 'Fr1A')
        self.declare_parameter('enabled', default_enabled)
        # Real-car DriveCommand output. Self-gated on /mission/selected so this
        # follower only drives when a matching dynamic mission is active;
        # MissionManager gates again on DRIVING. Uses the PRE-gain road-wheel
        # angle (not the CMRosIF steer_command_gain output).
        self.declare_parameter('publish_drive_command', False)
        self.declare_parameter('drive_command_topic', '/dynamic_drive_command')
        self.declare_parameter('mission_topic', '/mission/selected')
        self.declare_parameter('mission_gates', ['skidpad', 'autocross', 'trackdrive'])
        self.declare_parameter('wheel_circumference_m', 1.674)
        self.declare_parameter('drive_torque_nm', 50.0)
        self.declare_parameter('max_axle_rpm', 500.0)
        self.declare_parameter('max_steer_angle_deg', 21.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('path_timeout_sec', 0.5)
        self.declare_parameter('wheelbase', 1.53)
        self.declare_parameter('control_point_x_offset', 0.5637)
        self.declare_parameter('control_point_y_offset', 0.0)
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('adaptive_lookahead_enabled', False)
        self.declare_parameter('lookahead_throttle_gain', 3.0)
        self.declare_parameter('min_lookahead_distance', 1.2)
        self.declare_parameter('max_lookahead_distance', 8.0)
        self.declare_parameter('min_x', 0.4)
        self.declare_parameter('max_x', 35.0)
        self.declare_parameter('max_abs_y', 10.0)
        self.declare_parameter('max_path_segment_gap', 2.0)
        self.declare_parameter('cruise_gas', 0.08)
        self.declare_parameter('min_gas', 0.03)
        self.declare_parameter('max_gas', 0.18)
        self.declare_parameter('stop_brake', 0.8)
        self.declare_parameter('max_steer', 0.6)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('steer_gain', 1.0)
        self.declare_parameter('max_steer_rate', 4.0)
        # Plant-inverse: multiplier from desired road-wheel angle (the controller
        # output, and the physical max_steer/max_steer_rate domain) to the value
        # published on steer_ang. 1.0 = identity (real car). In the CarMaker
        # (CMRosIF) sim the plant realises only ~0.146x of the commanded angle,
        # so ~6.85 makes the sim execute the desired road-wheel angle. Sim-only;
        # see steer_calibration.py.
        self.declare_parameter('steer_command_gain', 1.0)
        # Steering law selector:
        #   'pursuit'     — classic pure pursuit (steer_gain applies).
        #   'lock'        — pure open-loop: hold the fixed road-wheel angle that
        #                   geometrically yields skidpad_radius, sign from the
        #                   path, zero on the straights. No position feedback.
        #   'feedforward' — atan(L*path_curvature) + a gentle correction toward
        #                   the path (crosstrack_gain, heading_gain).
        self.declare_parameter('steering_mode', 'pursuit')
        # Locked-turn radius (m) for 'lock' mode. Wire to the skidpad circle
        # radius so the held angle matches the geometric loop.
        self.declare_parameter('skidpad_radius', 9.125)
        # |path curvature| below this (1/m) is treated as a straight (lock mode
        # holds 0; feedforward term ~0). 0.03 => radius > ~33 m counts straight,
        # well clear of the 1/9.125=0.11 loop.
        self.declare_parameter('path_straight_curvature', 0.03)
        # feedforward-mode feedback gains (rad per m, rad per rad).
        self.declare_parameter('crosstrack_gain', 0.15)
        self.declare_parameter('heading_gain', 0.5)
        self.declare_parameter('curvature_slowdown_gain', 3.0)
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('speed_brake_gain', 0.5)
        self.declare_parameter('max_speed_brake', 0.3)
        self.declare_parameter('selector_ctrl', 1)
        self.declare_parameter('debug_line_width', 0.06)
        self.declare_parameter('debug_point_diameter', 0.25)

        path_topic = str(self.get_parameter('path_topic').value)
        control_topic = str(self.get_parameter('control_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        self._control_frame = str(self.get_parameter('control_frame').value)
        self._enabled = bool(self.get_parameter('enabled').value)
        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))

        self._publish_drive_command = bool(self.get_parameter('publish_drive_command').value)
        drive_command_topic = str(self.get_parameter('drive_command_topic').value)
        mission_topic = str(self.get_parameter('mission_topic').value)
        self._mission_gates = set(self.get_parameter('mission_gates').value)
        self._wheel_circumference_m = max(1e-3, float(self.get_parameter('wheel_circumference_m').value))
        self._drive_torque_nm = float(self.get_parameter('drive_torque_nm').value)
        self._max_axle_rpm = float(self.get_parameter('max_axle_rpm').value)
        self._max_steer_angle_deg = float(self.get_parameter('max_steer_angle_deg').value)
        self._selected_mission = 'none'

        self._path_timeout_sec = float(self.get_parameter('path_timeout_sec').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._control_point_x_offset = float(self.get_parameter('control_point_x_offset').value)
        self._control_point_y_offset = float(self.get_parameter('control_point_y_offset').value)
        self._lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self._adaptive_lookahead_enabled = bool(self.get_parameter('adaptive_lookahead_enabled').value)
        self._lookahead_throttle_gain = float(self.get_parameter('lookahead_throttle_gain').value)
        self._min_lookahead_distance = float(self.get_parameter('min_lookahead_distance').value)
        self._max_lookahead_distance = float(self.get_parameter('max_lookahead_distance').value)
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._max_path_segment_gap = float(self.get_parameter('max_path_segment_gap').value)
        self._cruise_gas = float(self.get_parameter('cruise_gas').value)
        self._min_gas = float(self.get_parameter('min_gas').value)
        self._max_gas = float(self.get_parameter('max_gas').value)
        self._stop_brake = float(self.get_parameter('stop_brake').value)
        self._max_steer = float(self.get_parameter('max_steer').value)
        self._steer_sign = float(self.get_parameter('steer_sign').value)
        self._steer_gain = max(0.0, float(self.get_parameter('steer_gain').value))
        self._max_steer_rate = float(self.get_parameter('max_steer_rate').value)
        self._steer_command_gain = float(self.get_parameter('steer_command_gain').value)
        # Publish clamp lives in the published domain (road-wheel clamp * plant gain).
        self._max_published_steer = self._max_steer * abs(self._steer_command_gain)
        self._steering_mode = str(self.get_parameter('steering_mode').value).lower()
        if self._steering_mode not in ('pursuit', 'lock', 'feedforward'):
            self.get_logger().warn(
                f'Unknown steering_mode={self._steering_mode!r}; using pursuit.'
            )
            self._steering_mode = 'pursuit'
        self._skidpad_radius = max(1e-3, float(self.get_parameter('skidpad_radius').value))
        self._path_straight_curvature = abs(float(self.get_parameter('path_straight_curvature').value))
        self._crosstrack_gain = float(self.get_parameter('crosstrack_gain').value)
        self._heading_gain = float(self.get_parameter('heading_gain').value)
        self._curvature_slowdown_gain = float(self.get_parameter('curvature_slowdown_gain').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self._max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self._speed_brake_gain = float(self.get_parameter('speed_brake_gain').value)
        self._max_speed_brake = float(self.get_parameter('max_speed_brake').value)
        self._selector_ctrl = int(self.get_parameter('selector_ctrl').value)
        self._debug_line_width = float(self.get_parameter('debug_line_width').value)
        self._debug_point_diameter = float(self.get_parameter('debug_point_diameter').value)

        self._path: Path | None = None
        self._path_receive_sec = -1.0
        self._current_speed: float | None = None
        self._current_fwd_speed: float | None = None
        self._current_yaw_rate: float | None = None
        self._odom_receive_sec: float = -1.0
        self._last_gas = 0.0
        self._last_steer = 0.0
        self._last_command_sec = self._now_sec()
        self._last_stop_reason = ''
        self._last_stop_log_sec = -10.0
        self._last_diag_log_sec = -10.0

        # Minimum forward speed (m/s) before the achieved-curvature estimate
        # (yaw_rate / speed) is trustworthy — below this it is dominated by noise.
        self._diag_min_speed = 0.3

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, mission_topic, self._on_mission, 10)
        self._control_pub = self.create_publisher(VehicleControl, control_topic, 10)
        self._drive_pub = self.create_publisher(DriveCommand, drive_command_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._control_tick)

        mode = 'ACTIVE' if self._enabled else 'disabled debug-only'
        self.get_logger().info(
            f'{self._display_name} started in {mode} mode; path={path_topic}, '
            f'control={control_topic}, debug={debug_topic}, frame={self._control_frame}, '
            f'wheelbase={self._wheelbase:.4f} m, '
            f'control_point_offset=({self._control_point_x_offset:.4f}, '
            f'{self._control_point_y_offset:.4f}) m, '
            f'steering_mode={self._steering_mode}'
        )
        # The plant-inverse gain is a sim-only correction (see steer_calibration.py).
        # Anything other than identity on the real vehicle would amplify every
        # steering command, so make it impossible to miss.
        if abs(self._steer_command_gain - 1.0) > 1e-6:
            self.get_logger().warn(
                f'PLANT-INVERSE ACTIVE: steer_command_gain={self._steer_command_gain:.3f} '
                f'(!= 1.0). This is a SIMULATION-ONLY calibration; every published '
                f'steer_ang is scaled by it. Set steer_command_gain:=1.0 on the real vehicle.'
            )

    def _on_mission(self, msg: String) -> None:
        self._selected_mission = msg.data

    def _drive_gate_active(self) -> bool:
        return self._publish_drive_command and self._selected_mission in self._mission_gates

    def _on_path(self, msg: Path) -> None:
        self._path = msg
        self._path_receive_sec = self._now_sec()

    def _on_odom(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed = math.hypot(vx, vy)
        self._current_fwd_speed = vx
        self._current_yaw_rate = msg.twist.twist.angular.z
        self._odom_receive_sec = self._now_sec()

    def _control_tick(self) -> None:
        valid, reason = self._inputs_valid()
        if not valid:
            self._debug_pub.publish(self._make_debug_markers([], None, reason))
            if self._enabled:
                self._publish_stop(reason)
            if self._drive_gate_active():
                self._drive_pub.publish(self._drive_stop_command())
            return

        path_points, reason = self._path_points_in_control_frame()
        if not path_points:
            self._debug_pub.publish(self._make_debug_markers([], None, reason))
            if self._enabled:
                self._publish_stop(reason)
            if self._drive_gate_active():
                self._drive_pub.publish(self._drive_stop_command())
            return

        target = self._compute_target(path_points)
        if target is None:
            reason = 'no valid lookahead point'
            self._debug_pub.publish(self._make_debug_markers(path_points, None, reason))
            if self._enabled:
                self._publish_stop(reason)
            if self._drive_gate_active():
                self._drive_pub.publish(self._drive_stop_command())
            return

        self._debug_pub.publish(self._make_debug_markers(path_points, target, 'active'))
        if self._enabled:
            self._publish_control(target.gas, target.brake, target.published_steer)
        if self._drive_gate_active():
            self._drive_pub.publish(self._drive_command_from_target(target))
        self._log_diagnostics(target, len(path_points))

    def _drive_command_from_target(self, target: LocalTarget) -> DriveCommand:
        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        # PRE-gain road-wheel angle in degrees (not the CMRosIF steer_command_gain
        # output). DriveCommand convention: positive = left.
        cmd.steer_angle_deg = float(self._clamp(
            math.degrees(target.steer_angle), -self._max_steer_angle_deg, self._max_steer_angle_deg))
        if target.brake > 0.0:
            cmd.axle_speed_rpm = 0.0
            cmd.axle_torque_nm = 0.0
        else:
            axle_rpm = (max(0.0, target.target_speed) / self._wheel_circumference_m) * 60.0
            cmd.axle_speed_rpm = float(self._clamp(axle_rpm, 0.0, self._max_axle_rpm))
            cmd.axle_torque_nm = float(max(0.0, self._drive_torque_nm))
        cmd.brake_pct = float(self._clamp(target.brake * 100.0, 0.0, 100.0))
        return cmd

    def _drive_stop_command(self) -> DriveCommand:
        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.steer_angle_deg = 0.0
        cmd.axle_speed_rpm = 0.0
        cmd.axle_torque_nm = 0.0
        cmd.brake_pct = float(self._clamp(self._stop_brake * 100.0, 0.0, 100.0))
        return cmd

    def _inputs_valid(self) -> tuple[bool, str]:
        if self._path is None:
            return False, 'waiting for path'
        if len(self._path.poses) < 2:
            return False, 'path has fewer than two points'
        if self._now_sec() - self._path_receive_sec > self._path_timeout_sec:
            return False, 'path stale'
        return True, 'ok'

    def _path_points_in_control_frame(self) -> tuple[list[Point], str]:
        assert self._path is not None

        source_frame = self._path.header.frame_id or self._control_frame
        raw_points = [pose.pose.position for pose in self._path.poses]
        if source_frame == self._control_frame:
            return self._contiguous_forward_path(raw_points)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._control_frame,
                source_frame,
                Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            return [], f'tf unavailable {source_frame}->{self._control_frame}: {exc}'

        return self._contiguous_forward_path([self._transform_point(point, transform) for point in raw_points])

    def _contiguous_forward_path(self, points: list[Point]) -> tuple[list[Point], str]:
        segment = []
        last_segment_point = None

        for point in points:
            point_is_forward = self._point_is_forward_of_control_point(point)
            if not segment:
                if point_is_forward:
                    segment.append(point)
                    last_segment_point = point
                continue

            assert last_segment_point is not None
            gap = self._distance_xy(last_segment_point, point)
            if gap > self._max_path_segment_gap:
                break
            if not point_is_forward:
                break

            segment.append(point)
            last_segment_point = point

        if len(segment) < 2:
            return [], f'contiguous forward path has {len(segment)} point(s)'
        return segment, 'ok'

    def _compute_target(self, path_points: list[Point]) -> LocalTarget | None:
        if not path_points:
            return None

        control_points = [self._to_control_point_frame(point) for point in path_points]
        requested_lookahead = self._dynamic_lookahead()
        target_result = self._lookahead_point(control_points, requested_lookahead)
        if target_result is None:
            return None
        control_target, effective_lookahead = target_result

        target_distance_sq = max(
            control_target.x * control_target.x + control_target.y * control_target.y,
            1e-6,
        )
        pursuit_curvature = 2.0 * control_target.y / target_distance_sq

        cross_track_error, nearest_cp, path_heading = self._cross_track_error(control_points)
        path_curvature = self._path_curvature(control_points)

        # --- Steering law (selectable via steering_mode) --------------------
        # command_curvature is the geometric turn the controller intends. Every
        # mode reports it on the same footing so the curvature-based gas
        # slowdown, the wide/narrow tag and the cmd_k diagnostic stay comparable.
        if self._steering_mode == 'lock':
            # Pure open-loop lock: hold the fixed angle that geometrically yields
            # skidpad_radius, sign taken from the path, zero on the straights.
            # No position feedback — entry offset/heading error is preserved.
            if abs(path_curvature) < self._path_straight_curvature:
                command_curvature = 0.0
                desired_steer = 0.0
            else:
                command_curvature = math.copysign(1.0 / self._skidpad_radius, path_curvature)
                desired_steer = self._steer_sign * math.atan(self._wheelbase * command_curvature)
        elif self._steering_mode == 'feedforward':
            # Feed-forward the fitted path curvature (carries the steady state)
            # plus a gentle correction toward the path (nulls entry offset and
            # heading error). Feedback signs: steer right when left of the path
            # (e>0), steer toward the path's heading.
            command_curvature = path_curvature
            feedforward = math.atan(self._wheelbase * path_curvature)
            feedback = self._heading_gain * path_heading - self._crosstrack_gain * cross_track_error
            desired_steer = self._steer_sign * (feedforward + feedback)
        else:  # 'pursuit'
            command_curvature = pursuit_curvature
            desired_steer = self._steer_sign * self._steer_gain * math.atan(self._wheelbase * pursuit_curvature)

        # Clamp + rate-limit in the physical road-wheel domain, THEN apply the
        # plant-inverse gain at publish.
        desired_steer = self._clamp(desired_steer, -self._max_steer, self._max_steer)
        desired_steer = self._rate_limit_steer(desired_steer)
        published_steer = desired_steer * self._steer_command_gain

        curvature = command_curvature
        curvature_abs = abs(curvature)
        gas_scale = 1.0 / (1.0 + self._curvature_slowdown_gain * curvature_abs)
        gas = self._clamp(self._cruise_gas * gas_scale, self._min_gas, self._max_gas)
        brake = 0.0
        # Curvature-adjusted target speed for the real-car DriveCommand (axle rpm).
        target_speed = self._max_speed_mps * gas_scale

        # Speed cap: if current speed exceeds max_speed_mps, kill gas and apply
        # a light proportional brake.  Falls back gracefully if odom is unavailable.
        now_sec = self._now_sec()
        odom_fresh = (
            self._current_speed is not None
            and self._odom_receive_sec >= 0.0
            and now_sec - self._odom_receive_sec <= self._odom_timeout_sec
        )
        if odom_fresh and self._current_speed > self._max_speed_mps:
            overspeed = self._current_speed - self._max_speed_mps
            gas = 0.0
            brake = self._clamp(overspeed * self._speed_brake_gain, 0.0, self._max_speed_brake)

        nearest_point = self._to_control_frame(nearest_cp) if nearest_cp is not None else None

        return LocalTarget(
            point=self._to_control_frame(control_target),
            control_point=control_target,
            requested_lookahead_distance=requested_lookahead,
            effective_lookahead_distance=effective_lookahead,
            curvature=curvature,
            steer_angle=desired_steer,
            published_steer=published_steer,
            gas=gas,
            brake=brake,
            target_speed=target_speed,
            cross_track_error=cross_track_error,
            nearest_point=nearest_point,
            achieved_curvature=self._achieved_curvature(),
            path_curvature=path_curvature,
            mode=self._steering_mode,
        )

    def _achieved_curvature(self) -> float | None:
        """Curvature the vehicle is actually executing, κ = yaw_rate / speed.

        Returns None when odom is missing or forward speed is below
        _diag_min_speed (estimate too noisy to trust).
        """
        if (
            self._current_yaw_rate is None
            or self._current_fwd_speed is None
            or abs(self._current_fwd_speed) < self._diag_min_speed
        ):
            return None
        return self._current_yaw_rate / self._current_fwd_speed

    def _cross_track_error(self, control_points: list[Point]) -> tuple[float, Point | None, float]:
        """Signed lateral offset (m) of the control point (origin) from the path.

        Reuses the perpendicular-projection search from _fallback_lookahead_point.
        Sign: positive = the control point is to the LEFT of the path's forward
        direction. A car turning right (curvature < 0) that runs wide lies to the
        left of the path -> e > 0; turning left (curvature > 0), wide lies to the
        right -> e < 0. So `e * curvature < 0` means "wide", `> 0` means "narrow".

        Returns (signed_error, nearest_point_in_control_point_frame, path_heading).
        path_heading is atan2 of the nearest segment's direction in the control
        frame (car forward = +x): +ve => the path heads to the left of the car.
        """
        if not control_points:
            return 0.0, None, 0.0
        if len(control_points) == 1:
            return 0.0, control_points[0], 0.0

        best_dist_sq = float('inf')
        best_point: Point | None = None
        best_sign = 0.0
        best_heading = 0.0
        for start, end in zip(control_points[:-1], control_points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq < 1e-9:
                continue
            ratio = self._clamp(-(start.x * dx + start.y * dy) / length_sq, 0.0, 1.0)
            px = start.x + ratio * dx
            py = start.y + ratio * dy
            dist_sq = px * px + py * py
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_point = Point(x=px, y=py, z=start.z + ratio * (end.z - start.z))
                # cross(segment_dir, origin - nearest); +ve => origin left of path.
                best_sign = dy * px - dx * py
                best_heading = math.atan2(dy, dx)

        if best_point is None:
            return 0.0, control_points[0], 0.0
        magnitude = math.sqrt(best_dist_sq)
        error = magnitude if best_sign >= 0.0 else -magnitude
        return error, best_point, best_heading

    def _path_curvature(self, control_points: list[Point]) -> float:
        """Signed curvature (1/m) of a circle fitted to the forward path.

        Uses the Kåsa algebraic least-squares fit (minimises the algebraic
        distance to x^2+y^2+D*x+E*y+F=0). Averaging over every path point makes
        this far smoother than the single-lookahead-point pursuit curvature,
        which is what lets the lock/feedforward steady-state term sit still
        instead of chasing a jumping goal point.

        Sign convention matches pursuit curvature: +ve => turning left (fitted
        circle centre to the left, +y). Returns 0.0 when the fit is degenerate
        or the path is essentially straight.
        """
        n = len(control_points)
        if n < 3:
            return 0.0

        sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
        for p in control_points:
            z = p.x * p.x + p.y * p.y
            sx += p.x
            sy += p.y
            sxx += p.x * p.x
            syy += p.y * p.y
            sxy += p.x * p.y
            sxz += p.x * z
            syz += p.y * z
            sz += z

        # Normal equations for [D, E, F]:
        #   [sxx sxy sx][D]   [-sxz]
        #   [sxy syy sy][E] = [-syz]
        #   [sx  sy  n ][F]   [-sz ]
        a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
        rhs = [-sxz, -syz, -sz]
        det = (
            a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
        )
        if abs(det) < 1e-12:
            return 0.0

        def _solve_col(col: int) -> float:
            m = [row[:] for row in a]
            for r in range(3):
                m[r][col] = rhs[r]
            return (
                m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            ) / det

        d = _solve_col(0)
        e = _solve_col(1)
        f = _solve_col(2)
        centre_x = -0.5 * d
        centre_y = -0.5 * e
        radius_sq = centre_x * centre_x + centre_y * centre_y - f
        if radius_sq <= 1e-6:
            return 0.0
        radius = math.sqrt(radius_sq)
        # Centre to the left (+y) => turning left => +curvature.
        return math.copysign(1.0 / radius, centre_y)

    def _dynamic_lookahead(self) -> float:
        lookahead = self._lookahead_distance
        if not self._adaptive_lookahead_enabled:
            return lookahead

        gas_ratio = 0.0
        if self._max_gas > 1e-6:
            gas_ratio = self._clamp(self._last_gas / self._max_gas, 0.0, 1.0)
        lookahead += self._lookahead_throttle_gain * gas_ratio
        return self._clamp(lookahead, self._min_lookahead_distance, self._max_lookahead_distance)

    def _lookahead_point(self, path_points: list[Point], lookahead: float) -> tuple[Point, float] | None:
        if not path_points or lookahead <= 1e-6:
            return None

        radius_sq = lookahead * lookahead
        for start, end in zip(path_points[:-1], path_points[1:]):
            intersection = self._segment_circle_intersection(start, end, radius_sq)
            if intersection is not None:
                return intersection, lookahead

        fallback = self._fallback_lookahead_point(path_points, radius_sq)
        if fallback is None:
            return None
        fallback_distance = math.hypot(fallback.x, fallback.y)
        return fallback, fallback_distance

    @staticmethod
    def _segment_circle_intersection(start: Point, end: Point, radius_sq: float) -> Point | None:
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        a = dx * dx + dy * dy
        if a < 1e-9:
            return None

        b = 2.0 * (start.x * dx + start.y * dy)
        c = start.x * start.x + start.y * start.y - radius_sq
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None

        sqrt_discriminant = math.sqrt(discriminant)
        candidates = [
            (-b - sqrt_discriminant) / (2.0 * a),
            (-b + sqrt_discriminant) / (2.0 * a),
        ]
        for ratio in sorted(candidates):
            if 0.0 <= ratio <= 1.0:
                return Point(
                    x=start.x + ratio * dx,
                    y=start.y + ratio * dy,
                    z=start.z + ratio * dz,
                )
        return None

    @staticmethod
    def _fallback_lookahead_point(path_points: list[Point], radius_sq: float) -> Point | None:
        nearest_point = None
        nearest_distance_sq = float('inf')
        farthest_point = None
        farthest_distance_sq = -1.0

        for point in path_points:
            distance_sq = point.x * point.x + point.y * point.y
            if distance_sq > farthest_distance_sq:
                farthest_distance_sq = distance_sq
                farthest_point = point
            if distance_sq < nearest_distance_sq:
                nearest_distance_sq = distance_sq
                nearest_point = point

        for start, end in zip(path_points[:-1], path_points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            dz = end.z - start.z
            length_sq = dx * dx + dy * dy
            if length_sq < 1e-9:
                continue
            ratio = LocalPathFollower._clamp(
                -(start.x * dx + start.y * dy) / length_sq,
                0.0,
                1.0,
            )
            projected = Point(
                x=start.x + ratio * dx,
                y=start.y + ratio * dy,
                z=start.z + ratio * dz,
            )
            distance_sq = projected.x * projected.x + projected.y * projected.y
            if distance_sq < nearest_distance_sq:
                nearest_distance_sq = distance_sq
                nearest_point = projected

        if nearest_point is not None and nearest_distance_sq > radius_sq:
            return nearest_point
        return farthest_point

    def _rate_limit_steer(self, steer: float) -> float:
        now_sec = self._now_sec()
        dt = max(now_sec - self._last_command_sec, 1e-3)
        max_delta = self._max_steer_rate * dt
        steer = self._clamp(steer, self._last_steer - max_delta, self._last_steer + max_delta)
        self._last_steer = steer
        self._last_command_sec = now_sec
        return steer

    def _publish_control(self, gas: float, brake: float, steer: float) -> None:
        msg = VehicleControl()
        msg.use_vc = True
        msg.selector_ctrl = self._selector_ctrl
        msg.gas = float(self._clamp(gas, 0.0, 1.0))
        msg.brake = float(self._clamp(brake, 0.0, 1.0))
        msg.steer_ang = float(self._clamp(steer, -self._max_published_steer, self._max_published_steer))
        msg.steer_ang_vel = 0.0
        msg.steer_ang_acc = 0.0
        self._last_gas = msg.gas
        self._control_pub.publish(msg)

    def _log_diagnostics(self, target: LocalTarget, path_count: int) -> None:
        """Throttled (~3 Hz) one-line time series for offline tracking analysis.

        Fields: speed, effective lookahead, path points, commanded vs achieved
        curvature and steer, and signed cross-track error. Reading the sequence
        shows whether commanded κ oscillates (goal-point/gain instability) or the
        achieved κ lags a steady command (actuator/dynamics lag).
        """
        if not self._enabled:
            return
        now_sec = self._now_sec()
        if now_sec - self._last_diag_log_sec < 0.33:
            return
        self._last_diag_log_sec = now_sec

        speed = self._current_fwd_speed if self._current_fwd_speed is not None else float('nan')
        act_kappa = target.achieved_curvature
        act_kappa_str = f'{act_kappa:+.4f}' if act_kappa is not None else 'nan'
        act_steer_str = (
            f'{math.atan(self._wheelbase * act_kappa):+.3f}'
            if act_kappa is not None else 'nan'
        )
        tag = self._wide_narrow_tag(target.cross_track_error, target.curvature)
        self.get_logger().info(
            f'[diag] {target.mode} v={speed:+.2f} La={target.effective_lookahead_distance:.2f} '
            f'pts={path_count} '
            f'path_k={target.path_curvature:+.4f} '
            f'cmd_k={target.curvature:+.4f} act_k={act_kappa_str} '
            f'cmd_st={target.steer_angle:+.3f} pub_st={target.published_steer:+.3f} '
            f'act_st={act_steer_str} '
            f'xtrack={target.cross_track_error:+.2f} [{tag}]'
        )

    def _publish_stop(self, reason: str) -> None:
        now_sec = self._now_sec()
        if reason != self._last_stop_reason or now_sec - self._last_stop_log_sec > 2.0:
            self.get_logger().warn(f'Publishing stop command: {reason}')
            self._last_stop_reason = reason
            self._last_stop_log_sec = now_sec
        self._publish_control(0.0, self._stop_brake, 0.0)

    def _make_debug_markers(
        self,
        path_points: list[Point],
        target: LocalTarget | None,
        status: str,
    ) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._delete_all(stamp))
        markers.markers.append(self._status_text(stamp, target, status, len(path_points)))
        if path_points:
            markers.markers.append(self._line_strip(1, stamp, 'local_path', path_points, self._magenta()))
        if target is not None:
            control_origin = self._control_origin_point()
            markers.markers.append(self._point_list(2, stamp, 'lookahead_target', [target.point], self._orange()))
            markers.markers.append(
                self._line_strip(
                    3,
                    stamp,
                    'control_point_to_lookahead',
                    [control_origin, target.point],
                    self._white(),
                )
            )
            markers.markers.append(self._point_list(4, stamp, 'rear_axle_control_point', [control_origin], self._cyan()))
            if target.nearest_point is not None:
                tag = self._wide_narrow_tag(target.cross_track_error, target.curvature)
                if tag == 'on-line':
                    xtrack_colour = self._green()
                elif tag == 'WIDE':
                    xtrack_colour = self._red()
                else:
                    xtrack_colour = self._blue()
                markers.markers.append(
                    self._line_strip(
                        5,
                        stamp,
                        'cross_track',
                        [control_origin, target.nearest_point],
                        xtrack_colour,
                    )
                )
                markers.markers.append(
                    self._point_list(6, stamp, 'cross_track_nearest', [target.nearest_point], xtrack_colour)
                )
        return markers

    def _delete_all(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._control_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _status_text(self, stamp, target: LocalTarget | None, status: str, path_count: int) -> Marker:
        marker = self._base_marker(0, stamp, 'pure_pursuit_path_follower_status', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 1.5
        marker.pose.position.z = 1.5
        marker.scale.z = 0.28
        marker.color = self._white()
        enabled = 'true' if self._enabled else 'false'
        speed_str = f'{self._current_speed:.2f} m/s' if self._current_speed is not None else 'no odom'
        if target is None:
            marker.text = (
                f'{self._display_name}\nenabled: {enabled}\nstatus: {status}\n'
                f'path points: {path_count}  speed: {speed_str} / {self._max_speed_mps:.2f} m/s max'
            )
        else:
            cmd_r = self._radius_str(target.curvature)
            act_kappa = target.achieved_curvature
            act_kappa_str = f'{act_kappa:.3f}' if act_kappa is not None else '  -  '
            act_r = self._radius_str(act_kappa) if act_kappa is not None else '   -  '
            act_steer_str = (
                f'{math.atan(self._wheelbase * act_kappa):.2f}'
                if act_kappa is not None else '  -  '
            )
            tag = self._wide_narrow_tag(target.cross_track_error, target.curvature)
            marker.text = (
                f'{self._display_name}  enabled: {enabled}\n'
                f'status: {status}  path points: {path_count}\n'
                f'speed: {speed_str} / {self._max_speed_mps:.2f} m/s max\n'
                f'lookahead set: {target.requested_lookahead_distance:.2f} m  '
                f'effective: {target.effective_lookahead_distance:.2f} m\n'
                f'mode: {target.mode}  path κ={target.path_curvature:.3f}\n'
                f'cmd  κ={target.curvature:.3f} r={cmd_r}  steer={target.steer_angle:.2f} '
                f'(pub {target.published_steer:.2f})\n'
                f'act  κ={act_kappa_str} r={act_r}  steer={act_steer_str}\n'
                f'cross-track: {target.cross_track_error:+.2f} m  [{tag}]\n'
                f'gas: {target.gas:.2f}  brake: {target.brake:.2f}  '
                f'control target: x={target.control_point.x:.2f} y={target.control_point.y:.2f}'
            )
        return marker

    @staticmethod
    def _radius_str(curvature: float | None) -> str:
        """Turn radius (m) for a curvature, or '  inf' when nearly straight."""
        if curvature is None or abs(curvature) < 1e-4:
            return '  inf'
        return f'{1.0 / abs(curvature):5.1f}'

    @staticmethod
    def _wide_narrow_tag(cross_track_error: float, curvature: float) -> str:
        """Classify tracking offset. See _cross_track_error for the sign rule:
        e*curvature < 0 => wide (outside the turn), > 0 => narrow (inside)."""
        if abs(cross_track_error) < 0.15:
            return 'on-line'
        if abs(curvature) < 1e-3:
            return 'left' if cross_track_error > 0 else 'right'
        return 'WIDE' if cross_track_error * curvature < 0.0 else 'narrow'

    def _point_is_forward_of_control_point(self, point: Point) -> bool:
        control_point = self._to_control_point_frame(point)
        return (
            self._min_x <= control_point.x <= self._max_x
            and abs(control_point.y) <= self._max_abs_y
        )

    def _to_control_point_frame(self, point: Point) -> Point:
        return Point(
            x=point.x - self._control_point_x_offset,
            y=point.y - self._control_point_y_offset,
            z=point.z,
        )

    def _to_control_frame(self, point: Point) -> Point:
        return Point(
            x=point.x + self._control_point_x_offset,
            y=point.y + self._control_point_y_offset,
            z=point.z,
        )

    def _control_origin_point(self) -> Point:
        return Point(
            x=self._control_point_x_offset,
            y=self._control_point_y_offset,
            z=0.0,
        )

    def _line_strip(
        self,
        marker_id: int,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.LINE_STRIP)
        marker.scale.x = self._debug_line_width
        marker.color = colour
        marker.points = points
        return marker

    def _point_list(
        self,
        marker_id: int,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.SPHERE_LIST)
        marker.scale.x = self._debug_point_diameter
        marker.scale.y = self._debug_point_diameter
        marker.scale.z = self._debug_point_diameter
        marker.color = colour
        marker.points = points
        return marker

    def _base_marker(self, marker_id: int, stamp, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._control_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000
        return marker

    @staticmethod
    def _transform_point(point: Point, transform: TransformStamped) -> Point:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z = LocalPathFollower._rotate_vector(
            point.x,
            point.y,
            point.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        return Point(x=x + translation.x, y=y + translation.y, z=z + translation.z)

    @staticmethod
    def _rotate_vector(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            return x, y, z
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    @staticmethod
    def _distance_xy(left: Point, right: Point) -> float:
        return math.hypot(left.x - right.x, left.y - right.y)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _magenta() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.0, b=0.9, a=1.0)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

    @staticmethod
    def _cyan() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)

    @staticmethod
    def _red() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)

    @staticmethod
    def _blue() -> ColorRGBA:
        return ColorRGBA(r=0.15, g=0.4, b=1.0, a=1.0)


def _spin_node(node: LocalPathFollower) -> None:
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok() and node._enabled:
            node._publish_stop('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathFollower()
    _spin_node(node)


def pure_pursuit_main(args=None):
    rclpy.init(args=args)
    node = LocalPathFollower(
        node_name='pure_pursuit_path_follower',
        default_path_topic='/nav/persistent_path',
        default_debug_topic='/nav/pure_pursuit_path_follower_markers',
        default_enabled=True,
        display_name='Pure pursuit path follower',
    )
    _spin_node(node)


if __name__ == '__main__':
    main()
