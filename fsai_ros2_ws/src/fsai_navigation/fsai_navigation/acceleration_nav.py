"""Throttle/brake-only straight-track controller from local cone detections."""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from fsai_interfaces.msg import Cone3DArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from vehiclecontrol_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class LocalCone:
    colour: str
    point: Point
    class_name: str
    source: str
    confidence: float


@dataclass(frozen=True)
class EndpointCandidate:
    blue: LocalCone
    yellow: LocalCone
    stop_x: float
    width: float
    pair_x_gap: float


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    yaw: float
    receive_sec: float


@dataclass(frozen=True)
class AccelerationState:
    status: str
    stop_x: float
    remaining: float
    closing_speed: float
    target_speed: float
    gas: float
    brake: float
    candidate_frames: int
    complete: bool


class AccelerationNav(Node):
    """Drives a straight acceleration track and stops at the terminal cone pair."""

    def __init__(self):
        super().__init__('acceleration_nav')

        self.declare_parameter('cones_topic', '/cones')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('debug_topic', '/nav/acceleration_nav_markers')
        self.declare_parameter('enabled', False)
        self.declare_parameter('use_odom_distance', True)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cones_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('detection_to_odom_x_offset', 0.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 60.0)
        self.declare_parameter('max_abs_y', 8.0)
        self.declare_parameter('min_cones_per_side', 2)
        self.declare_parameter('successor_search_distance', 5.0)
        self.declare_parameter('max_endpoint_distance', 50.0)
        self.declare_parameter('candidate_match_distance', 2.0)
        self.declare_parameter('endpoint_confirm_frames', 3)
        self.declare_parameter('terminal_pair_max_x_gap', 2.0)
        self.declare_parameter('min_track_width', 1.5)
        self.declare_parameter('max_track_width', 5.0)
        self.declare_parameter('stop_position_mode', 'min')
        self.declare_parameter('stop_offset', 0.0)
        self.declare_parameter('distance_tolerance', 0.2)
        self.declare_parameter('cruise_gas', 0.08)
        self.declare_parameter('min_gas', 0.02)
        self.declare_parameter('max_gas', 0.50)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('target_decel', 1.2)
        self.declare_parameter('slowdown_distance', 8.0)
        self.declare_parameter('brake_distance', 3.0)
        self.declare_parameter('speed_tolerance', 0.25)
        self.declare_parameter('closing_speed_alpha', 0.35)
        self.declare_parameter('brake_gain', 0.5)
        self.declare_parameter('max_brake', 0.8)
        self.declare_parameter('hold_brake', 0.8)
        self.declare_parameter('selector_ctrl', 1)
        self.declare_parameter('debug_line_width', 0.06)
        self.declare_parameter('debug_point_diameter', 0.25)
        self.declare_parameter('debug_text_height', 0.28)

        cones_topic = self.get_parameter('cones_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        control_rate_hz = max(1.0, self.get_parameter('control_rate_hz').get_parameter_value().double_value)
        self._enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self._use_odom_distance = self.get_parameter('use_odom_distance').get_parameter_value().bool_value
        self._cones_timeout_sec = self.get_parameter('cones_timeout_sec').get_parameter_value().double_value
        self._odom_timeout_sec = self.get_parameter('odom_timeout_sec').get_parameter_value().double_value
        self._detection_to_odom_x_offset = self.get_parameter('detection_to_odom_x_offset').get_parameter_value().double_value
        self._min_x = self.get_parameter('min_x').get_parameter_value().double_value
        self._max_x = self.get_parameter('max_x').get_parameter_value().double_value
        self._max_abs_y = self.get_parameter('max_abs_y').get_parameter_value().double_value
        self._min_cones_per_side = int(self.get_parameter('min_cones_per_side').get_parameter_value().integer_value)
        self._successor_search_distance = self.get_parameter('successor_search_distance').get_parameter_value().double_value
        self._max_endpoint_distance = self.get_parameter('max_endpoint_distance').get_parameter_value().double_value
        self._candidate_match_distance = self.get_parameter('candidate_match_distance').get_parameter_value().double_value
        self._endpoint_confirm_frames = max(1, int(self.get_parameter('endpoint_confirm_frames').get_parameter_value().integer_value))
        self._terminal_pair_max_x_gap = self.get_parameter('terminal_pair_max_x_gap').get_parameter_value().double_value
        self._min_track_width = self.get_parameter('min_track_width').get_parameter_value().double_value
        self._max_track_width = self.get_parameter('max_track_width').get_parameter_value().double_value
        self._stop_position_mode = self.get_parameter('stop_position_mode').get_parameter_value().string_value.lower()
        self._stop_offset = self.get_parameter('stop_offset').get_parameter_value().double_value
        self._distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self._cruise_gas = self.get_parameter('cruise_gas').get_parameter_value().double_value
        self._min_gas = self.get_parameter('min_gas').get_parameter_value().double_value
        self._max_gas = self.get_parameter('max_gas').get_parameter_value().double_value
        self._max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self._target_decel = self.get_parameter('target_decel').get_parameter_value().double_value
        self._slowdown_distance = self.get_parameter('slowdown_distance').get_parameter_value().double_value
        self._brake_distance = self.get_parameter('brake_distance').get_parameter_value().double_value
        self._speed_tolerance = self.get_parameter('speed_tolerance').get_parameter_value().double_value
        self._closing_speed_alpha = self.get_parameter('closing_speed_alpha').get_parameter_value().double_value
        self._brake_gain = self.get_parameter('brake_gain').get_parameter_value().double_value
        self._max_brake = self.get_parameter('max_brake').get_parameter_value().double_value
        self._hold_brake = self.get_parameter('hold_brake').get_parameter_value().double_value
        self._selector_ctrl = int(self.get_parameter('selector_ctrl').get_parameter_value().integer_value)
        self._debug_line_width = self.get_parameter('debug_line_width').get_parameter_value().double_value
        self._debug_point_diameter = self.get_parameter('debug_point_diameter').get_parameter_value().double_value
        self._debug_text_height = self.get_parameter('debug_text_height').get_parameter_value().double_value

        self._cones: list[LocalCone] = []
        self._last_frame_id = 'Fr1A'
        self._last_stamp = self.get_clock().now().to_msg()
        self._cones_receive_sec = -1.0
        self._candidate: EndpointCandidate | None = None
        self._confirmed_endpoint: EndpointCandidate | None = None
        self._candidate_stop_x: float | None = None
        self._candidate_frames = 0
        self._last_endpoint_x: float | None = None
        self._last_endpoint_time_sec = -1.0
        self._odom: OdomSample | None = None
        self._latched_odom: OdomSample | None = None
        self._latched_terminal_distance: float | None = None
        self._odom_speed = 0.0
        self._closing_speed = 0.0
        self._complete = False
        self._endpoint_lost_after_confirm = False
        self._endpoint_status = 'waiting for cones'
        self._last_state = AccelerationState(
            status='waiting for cones',
            stop_x=float('nan'),
            remaining=float('nan'),
            closing_speed=0.0,
            target_speed=0.0,
            gas=0.0,
            brake=self._hold_brake,
            candidate_frames=0,
            complete=False,
        )

        self.create_subscription(Cone3DArray, cones_topic, self._on_cones, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._control_pub = self.create_publisher(VehicleControl, control_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._control_tick)

        mode = 'ACTIVE' if self._enabled else 'disabled debug-only'
        self.get_logger().info(
            f'Acceleration nav started in {mode} mode; cones={cones_topic}, '
            f'odom={odom_topic}, control={control_topic}, debug={debug_topic}'
        )

    def _on_cones(self, msg: Cone3DArray) -> None:
        self._last_frame_id = msg.header.frame_id or self._last_frame_id
        self._last_stamp = msg.header.stamp
        self._cones_receive_sec = self._now_sec()
        self._cones = self._extract_cones(msg)
        candidate, status = self._detect_endpoint(self._cones)
        self._endpoint_status = status
        self._update_endpoint(candidate)

    def _on_odom(self, msg: Odometry) -> None:
        now_sec = self._now_sec()
        pose = msg.pose.pose
        odom = OdomSample(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=self._yaw_from_quaternion(pose.orientation),
            receive_sec=now_sec,
        )

        if self._odom is not None:
            dt = max(now_sec - self._odom.receive_sec, 1e-3)
            heading = self._latched_odom.yaw if self._latched_odom is not None else odom.yaw
            dx = odom.x - self._odom.x
            dy = odom.y - self._odom.y
            projected_speed = (dx * math.cos(heading) + dy * math.sin(heading)) / dt
            self._odom_speed = max(0.0, projected_speed)

        self._odom = odom

    def _extract_cones(self, msg: Cone3DArray) -> list[LocalCone]:
        cones = []
        for cone in msg.cones:
            colour = self._colour_from_class_name(cone.class_name)
            if colour not in ('blue', 'yellow'):
                continue

            point = cone.position
            if not self._is_in_local_window(point):
                continue

            cones.append(
                LocalCone(
                    colour=colour,
                    point=Point(x=point.x, y=point.y, z=point.z),
                    class_name=cone.class_name,
                    source=cone.source,
                    confidence=float(cone.confidence),
                )
            )
        return cones

    def _detect_endpoint(self, cones: list[LocalCone]) -> tuple[EndpointCandidate | None, str]:
        blue = self._cones_by_colour(cones, 'blue')
        yellow = self._cones_by_colour(cones, 'yellow')
        if len(blue) < self._min_cones_per_side or len(yellow) < self._min_cones_per_side:
            return None, f'waiting for both sides blue={len(blue)} yellow={len(yellow)}'

        terminal_blue = self._terminal_cone(blue)
        terminal_yellow = self._terminal_cone(yellow)
        if terminal_blue is None or terminal_yellow is None:
            return None, 'no terminal cone on both sides'

        pair_x_gap = abs(terminal_blue.point.x - terminal_yellow.point.x)
        if pair_x_gap > self._terminal_pair_max_x_gap:
            return None, f'terminal pair x gap too large {pair_x_gap:.2f} m'

        width = self._distance_xy(terminal_blue.point, terminal_yellow.point)
        if width < self._min_track_width or width > self._max_track_width:
            return None, f'terminal track width invalid {width:.2f} m'

        stop_x = self._stop_x(terminal_blue, terminal_yellow)
        if stop_x > self._max_endpoint_distance:
            return None, f'candidate beyond max endpoint distance {stop_x:.2f} m'
        if stop_x < self._min_x:
            return None, f'candidate behind min x {stop_x:.2f} m'

        return (
            EndpointCandidate(
                blue=terminal_blue,
                yellow=terminal_yellow,
                stop_x=stop_x,
                width=width,
                pair_x_gap=pair_x_gap,
            ),
            'endpoint candidate',
        )

    def _terminal_cone(self, cones: list[LocalCone]) -> LocalCone | None:
        cones = sorted(cones, key=lambda cone: cone.point.x)
        max_gap = max(self._successor_search_distance, 0.1)
        for idx, cone in enumerate(cones):
            has_successor = any(
                0.0 < successor.point.x - cone.point.x <= max_gap
                for successor in cones[idx + 1:]
            )
            if not has_successor:
                return cone
        return None

    def _update_endpoint(self, candidate: EndpointCandidate | None) -> None:
        now_sec = self._now_sec()
        if candidate is None:
            self._candidate = None
            self._candidate_stop_x = None
            self._candidate_frames = 0
            if self._use_odom_distance and self._latched_terminal_distance is not None:
                return
            self._last_endpoint_x = None
            self._last_endpoint_time_sec = -1.0
            self._closing_speed = 0.0
            if self._confirmed_endpoint is not None and not self._complete:
                self._endpoint_lost_after_confirm = True
            return

        self._candidate = candidate
        self._endpoint_lost_after_confirm = False

        if self._candidate_stop_x is not None and abs(candidate.stop_x - self._candidate_stop_x) <= self._candidate_match_distance:
            self._candidate_frames += 1
        else:
            self._candidate_frames = 1
            self._closing_speed = 0.0
            self._last_endpoint_x = None
            self._last_endpoint_time_sec = -1.0
        self._candidate_stop_x = candidate.stop_x

        if self._last_endpoint_x is not None:
            dt = max(now_sec - self._last_endpoint_time_sec, 1e-3)
            measured_speed = max(0.0, (self._last_endpoint_x - candidate.stop_x) / dt)
            alpha = self._clamp(self._closing_speed_alpha, 0.0, 1.0)
            self._closing_speed = (1.0 - alpha) * self._closing_speed + alpha * measured_speed

        self._last_endpoint_x = candidate.stop_x
        self._last_endpoint_time_sec = now_sec

        if self._confirmed_endpoint is not None or self._candidate_frames >= self._endpoint_confirm_frames:
            self._confirmed_endpoint = candidate
            self._latch_endpoint_distance(candidate)

    def _latch_endpoint_distance(self, candidate: EndpointCandidate) -> None:
        if not self._use_odom_distance or self._odom is None:
            return

        self._latched_odom = self._odom
        self._latched_terminal_distance = candidate.stop_x - self._detection_to_odom_x_offset

    def _control_tick(self) -> None:
        state = self._compute_state()
        self._last_state = state
        self._debug_pub.publish(self._make_debug_markers(state))
        if self._enabled:
            self._publish_control(state.gas, state.brake)

    def _compute_state(self) -> AccelerationState:
        now_sec = self._now_sec()
        if self._complete:
            stop_x = self._current_terminal_x()
            if not math.isfinite(stop_x):
                endpoint = self._confirmed_endpoint
                stop_x = endpoint.stop_x if endpoint is not None else 0.0
            return self._state('complete hold', stop_x, 0.0, 0.0, 0.0, self._hold_brake, True)
        if self._use_odom_distance and self._latched_terminal_distance is not None:
            return self._compute_odom_latched_state()
        if self._use_odom_distance and not self._has_fresh_odom(now_sec):
            status = 'waiting for odom' if self._odom is None else 'odom stale'
            return self._state(status, float('nan'), 0.0, 0.0, 0.0, self._hold_brake, False)
        if self._cones_receive_sec < 0.0:
            return self._state('waiting for cones', float('nan'), 0.0, 0.0, 0.0, self._hold_brake, False)
        if now_sec - self._cones_receive_sec > self._cones_timeout_sec:
            return self._state('cones stale', float('nan'), 0.0, 0.0, 0.0, self._hold_brake, False)
        if self._endpoint_lost_after_confirm:
            return self._state('endpoint lost after confirm', float('nan'), self._closing_speed, 0.0, 0.0, self._hold_brake, False)
        if self._confirmed_endpoint is None:
            return self._state(
                self._endpoint_status,
                self._candidate.stop_x if self._candidate is not None else float('nan'),
                self._closing_speed,
                0.0,
                self._cruise_gas,
                0.0,
                False,
            )

        stop_x = self._confirmed_endpoint.stop_x
        remaining = stop_x - self._stop_offset
        if remaining <= self._distance_tolerance:
            self._complete = True
            return self._state('complete', stop_x, self._closing_speed, 0.0, 0.0, self._hold_brake, True)

        target_speed = self._target_speed(remaining)
        gas, brake = self._pedals_for(remaining, target_speed, self._closing_speed)
        return self._state('endpoint confirmed', stop_x, self._closing_speed, target_speed, gas, brake, False)

    def _compute_odom_latched_state(self) -> AccelerationState:
        now_sec = self._now_sec()
        if not self._has_fresh_odom(now_sec):
            stop_x = self._current_terminal_x()
            return self._state('odom stale after endpoint latch', stop_x, 0.0, 0.0, 0.0, self._hold_brake, False)

        stop_x = self._current_terminal_x()
        remaining = stop_x - self._stop_offset
        closing_speed = self._odom_speed
        if remaining <= self._distance_tolerance:
            self._complete = True
            return self._state('complete', stop_x, closing_speed, 0.0, 0.0, self._hold_brake, True)

        target_speed = self._target_speed(remaining)
        gas, brake = self._pedals_for(remaining, target_speed, closing_speed)
        return self._state('endpoint confirmed odom', stop_x, closing_speed, target_speed, gas, brake, False)

    def _current_terminal_x(self) -> float:
        travelled = self._odom_travelled_since_latch()
        if self._latched_terminal_distance is None or travelled is None:
            return float('nan')
        return self._latched_terminal_distance - travelled

    def _odom_travelled_since_latch(self) -> float | None:
        if self._odom is None or self._latched_odom is None:
            return None
        dx = self._odom.x - self._latched_odom.x
        dy = self._odom.y - self._latched_odom.y
        return dx * math.cos(self._latched_odom.yaw) + dy * math.sin(self._latched_odom.yaw)

    def _has_fresh_odom(self, now_sec: float | None = None) -> bool:
        if self._odom is None:
            return False
        if now_sec is None:
            now_sec = self._now_sec()
        return now_sec - self._odom.receive_sec <= self._odom_timeout_sec

    def _state(
        self,
        status: str,
        stop_x: float,
        closing_speed: float,
        target_speed: float,
        gas: float,
        brake: float,
        complete: bool,
    ) -> AccelerationState:
        remaining = stop_x - self._stop_offset if math.isfinite(stop_x) else float('nan')
        return AccelerationState(
            status=status,
            stop_x=stop_x,
            remaining=remaining,
            closing_speed=closing_speed,
            target_speed=target_speed,
            gas=self._clamp(gas, 0.0, 1.0),
            brake=self._clamp(brake, 0.0, 1.0),
            candidate_frames=self._candidate_frames,
            complete=complete,
        )

    def _target_speed(self, remaining: float) -> float:
        decel_speed = math.sqrt(max(0.0, 2.0 * max(self._target_decel, 0.1) * max(remaining, 0.0)))
        return min(max(self._max_speed, 0.0), decel_speed)

    def _pedals_for(self, remaining: float, target_speed: float, closing_speed: float) -> tuple[float, float]:
        if remaining <= self._brake_distance:
            brake_ratio = 1.0 - self._clamp(remaining / max(self._brake_distance, 0.1), 0.0, 1.0)
            return 0.0, self._clamp(max(0.2, brake_ratio * self._max_brake), 0.0, self._max_brake)

        speed_error = closing_speed - target_speed
        if speed_error > self._speed_tolerance:
            return 0.0, self._clamp(self._brake_gain * speed_error, 0.0, self._max_brake)

        if remaining <= self._slowdown_distance:
            gas_scale = self._clamp(remaining / max(self._slowdown_distance, 0.1), 0.0, 1.0)
            gas = max(self._min_gas, self._cruise_gas * gas_scale)
        else:
            gas = self._cruise_gas
        return self._clamp(gas, 0.0, self._max_gas), 0.0

    def _publish_control(self, gas: float, brake: float) -> None:
        msg = VehicleControl()
        msg.use_vc = True
        msg.selector_ctrl = self._selector_ctrl
        msg.gas = float(self._clamp(gas, 0.0, 1.0))
        msg.brake = float(self._clamp(brake, 0.0, 1.0))
        msg.steer_ang = 0.0
        msg.steer_ang_vel = 0.0
        msg.steer_ang_acc = 0.0
        self._control_pub.publish(msg)

    def _stop_x(self, blue: LocalCone, yellow: LocalCone) -> float:
        if self._stop_position_mode == 'average':
            return 0.5 * (blue.point.x + yellow.point.x)
        if self._stop_position_mode != 'min':
            self.get_logger().warn(f'Unknown stop_position_mode={self._stop_position_mode}; using min')
            self._stop_position_mode = 'min'
        return min(blue.point.x, yellow.point.x)

    def _make_debug_markers(self, state: AccelerationState) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        marker_list = [
            self._delete_all(stamp),
            self._status_text(stamp, state),
        ]

        blue = self._cones_by_colour(self._cones, 'blue')
        yellow = self._cones_by_colour(self._cones, 'yellow')
        if blue:
            marker_list.append(self._point_list(1, stamp, 'acceleration_blue_cones', [cone.point for cone in blue], self._blue(), 0.16))
        if yellow:
            marker_list.append(self._point_list(2, stamp, 'acceleration_yellow_cones', [cone.point for cone in yellow], self._yellow(), 0.16))

        endpoint = self._debug_endpoint(state)
        if endpoint is not None:
            endpoint_colour = self._green() if self._confirmed_endpoint is not None else self._orange()
            marker_list.append(self._point_list(3, stamp, 'acceleration_terminal_cones', [endpoint.blue.point, endpoint.yellow.point], endpoint_colour, self._debug_point_diameter))
            marker_list.append(self._line_strip(4, stamp, 'acceleration_terminal_pair', [endpoint.blue.point, endpoint.yellow.point], endpoint_colour, self._debug_line_width))
            marker_list.append(self._stop_line(5, stamp, endpoint, state))

        markers = MarkerArray()
        markers.markers = marker_list
        return markers

    def _debug_endpoint(self, state: AccelerationState) -> EndpointCandidate | None:
        endpoint = self._confirmed_endpoint or self._candidate
        if endpoint is None:
            return None
        if self._confirmed_endpoint is None or self._latched_terminal_distance is None or not math.isfinite(state.stop_x):
            return endpoint

        blue_x = state.stop_x + endpoint.blue.point.x - endpoint.stop_x
        yellow_x = state.stop_x + endpoint.yellow.point.x - endpoint.stop_x
        blue = LocalCone(
            colour=endpoint.blue.colour,
            point=Point(x=blue_x, y=endpoint.blue.point.y, z=endpoint.blue.point.z),
            class_name=endpoint.blue.class_name,
            source=endpoint.blue.source,
            confidence=endpoint.blue.confidence,
        )
        yellow = LocalCone(
            colour=endpoint.yellow.colour,
            point=Point(x=yellow_x, y=endpoint.yellow.point.y, z=endpoint.yellow.point.z),
            class_name=endpoint.yellow.class_name,
            source=endpoint.yellow.source,
            confidence=endpoint.yellow.confidence,
        )
        return EndpointCandidate(
            blue=blue,
            yellow=yellow,
            stop_x=state.stop_x,
            width=self._distance_xy(blue.point, yellow.point),
            pair_x_gap=abs(blue.point.x - yellow.point.x),
        )

    def _delete_all(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(self, marker_id: int, stamp, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000
        return marker

    def _status_text(self, stamp, state: AccelerationState) -> Marker:
        marker = self._base_marker(0, stamp, 'acceleration_nav_status', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 1.5
        marker.pose.position.z = 1.3
        marker.scale.z = self._debug_text_height
        marker.color = self._white()
        enabled = 'true' if self._enabled else 'false'
        distance_mode = 'odom' if self._use_odom_distance else 'local'
        stop_x = f'{state.stop_x:.2f}' if math.isfinite(state.stop_x) else 'nan'
        remaining = f'{state.remaining:.2f}' if math.isfinite(state.remaining) else 'nan'
        marker.text = (
            f'Acceleration nav  enabled: {enabled}\n'
            f'distance mode: {distance_mode}\n'
            f'status: {state.status}\n'
            f'stop_x: {stop_x} m  remaining: {remaining} m  frames: {state.candidate_frames}\n'
            f'closing: {state.closing_speed:.2f} m/s  target: {state.target_speed:.2f} m/s\n'
            f'gas: {state.gas:.2f}  brake: {state.brake:.2f}'
        )
        return marker

    def _line_strip(
        self,
        marker_id: int,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
        width: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
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
        diameter: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.SPHERE_LIST)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = colour
        marker.points = points
        return marker

    def _stop_line(self, marker_id: int, stamp, endpoint: EndpointCandidate, state: AccelerationState) -> Marker:
        y_min = min(endpoint.blue.point.y, endpoint.yellow.point.y)
        y_max = max(endpoint.blue.point.y, endpoint.yellow.point.y)
        stop_x = state.remaining if math.isfinite(state.remaining) else endpoint.stop_x
        stop_x = stop_x + self._stop_offset
        return self._line_strip(
            marker_id,
            stamp,
            'acceleration_stop_line',
            [Point(x=stop_x, y=y_min, z=0.0), Point(x=stop_x, y=y_max, z=0.0)],
            self._red() if state.brake > 0.0 else self._green(),
            self._debug_line_width,
        )

    def _is_in_local_window(self, point: Point) -> bool:
        return self._min_x <= point.x <= self._max_x and abs(point.y) <= self._max_abs_y

    @staticmethod
    def _cones_by_colour(cones: list[LocalCone], colour: str) -> list[LocalCone]:
        filtered = [cone for cone in cones if cone.colour == colour]
        filtered.sort(key=lambda cone: cone.point.x)
        return filtered

    @staticmethod
    def _colour_from_class_name(class_name: str) -> str | None:
        class_name = class_name.lower()
        if 'blue' in class_name:
            return 'blue'
        if 'yellow' in class_name:
            return 'yellow'
        return None

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _distance_xy(left: Point, right: Point) -> float:
        return math.hypot(left.x - right.x, left.y - right.y)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _blue() -> ColorRGBA:
        return ColorRGBA(r=0.1, g=0.35, b=1.0, a=1.0)

    @staticmethod
    def _yellow() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.8)

    @staticmethod
    def _red() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = AccelerationNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node._enabled:
            node._publish_control(0.0, node._hold_brake)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
