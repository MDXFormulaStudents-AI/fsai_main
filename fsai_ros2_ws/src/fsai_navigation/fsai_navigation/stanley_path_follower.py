"""Always-on local Stanley path follower for CarMaker VehicleControl."""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
import tf2_ros
from vehiclecontrol_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class StanleyTarget:
    nearest_point: Point
    path_heading: float
    cross_track_error: float
    heading_error: float
    cross_track_term: float
    curvature: float
    steer_angle: float
    gas: float
    speed: float


class StanleyPathFollower(Node):
    """Tracks a local path using Stanley steering and curvature-scaled throttle."""

    def __init__(self):
        super().__init__('stanley_path_follower')

        self.declare_parameter('path_topic', '/nav/perceived_path')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('debug_topic', '/nav/stanley_path_follower_markers')
        self.declare_parameter('control_frame', 'Fr1A')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('path_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('control_point_x_offset', 0.5637)
        self.declare_parameter('control_point_y_offset', 0.0)
        self.declare_parameter('stanley_gain', 0.8)
        self.declare_parameter('speed_softening', 1.0)
        self.declare_parameter('max_steer', 0.35)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('max_steer_rate', 4.0)
        self.declare_parameter('cruise_gas', 0.06)
        self.declare_parameter('min_gas', 0.03)
        self.declare_parameter('max_gas', 0.18)
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('curvature_slowdown_gain', 3.0)
        self.declare_parameter('stop_brake', 0.8)
        self.declare_parameter('min_x', -1.0)
        self.declare_parameter('max_x', 35.0)
        self.declare_parameter('max_abs_y', 10.0)
        self.declare_parameter('max_path_segment_gap', 2.0)
        self.declare_parameter('selector_ctrl', 1)
        self.declare_parameter('debug_line_width', 0.06)
        self.declare_parameter('debug_point_diameter', 0.25)

        path_topic = str(self.get_parameter('path_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        control_topic = str(self.get_parameter('control_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        self._control_frame = str(self.get_parameter('control_frame').value)
        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))

        self._path_timeout_sec = float(self.get_parameter('path_timeout_sec').value)
        self._odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self._control_point_x_offset = float(self.get_parameter('control_point_x_offset').value)
        self._control_point_y_offset = float(self.get_parameter('control_point_y_offset').value)
        self._stanley_gain = max(0.0, float(self.get_parameter('stanley_gain').value))
        self._speed_softening = max(0.05, float(self.get_parameter('speed_softening').value))
        self._max_steer = max(0.0, float(self.get_parameter('max_steer').value))
        self._steer_sign = float(self.get_parameter('steer_sign').value)
        self._max_steer_rate = max(0.0, float(self.get_parameter('max_steer_rate').value))
        self._cruise_gas = float(self.get_parameter('cruise_gas').value)
        self._min_gas = float(self.get_parameter('min_gas').value)
        self._max_gas = float(self.get_parameter('max_gas').value)
        self._lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self._curvature_slowdown_gain = float(self.get_parameter('curvature_slowdown_gain').value)
        self._stop_brake = float(self.get_parameter('stop_brake').value)
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._max_path_segment_gap = float(self.get_parameter('max_path_segment_gap').value)
        self._selector_ctrl = int(self.get_parameter('selector_ctrl').value)
        self._debug_line_width = float(self.get_parameter('debug_line_width').value)
        self._debug_point_diameter = float(self.get_parameter('debug_point_diameter').value)

        self._path: Path | None = None
        self._path_receive_sec = -1.0
        self._odom: Odometry | None = None
        self._odom_receive_sec = -1.0
        self._last_gas = 0.0
        self._last_steer = 0.0
        self._last_command_sec = self._now_sec()
        self._last_stop_reason = ''
        self._last_stop_log_sec = -10.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._control_pub = self.create_publisher(VehicleControl, control_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._control_tick)

        self.get_logger().info(
            f'Stanley path follower ACTIVE; path={path_topic}, odom={odom_topic}, '
            f'control={control_topic}, debug={debug_topic}, frame={self._control_frame}'
        )

    def _on_path(self, msg: Path) -> None:
        self._path = msg
        self._path_receive_sec = self._now_sec()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_receive_sec = self._now_sec()

    def _control_tick(self) -> None:
        valid, reason = self._inputs_valid()
        if not valid:
            self._debug_pub.publish(self._make_debug_markers([], None, reason))
            self._publish_stop(reason)
            return

        path_points, reason = self._path_points_in_control_frame()
        if not path_points:
            self._debug_pub.publish(self._make_debug_markers([], None, reason))
            self._publish_stop(reason)
            return

        target = self._compute_target(path_points)
        if target is None:
            reason = 'unable to compute Stanley target'
            self._debug_pub.publish(self._make_debug_markers(path_points, None, reason))
            self._publish_stop(reason)
            return

        self._debug_pub.publish(self._make_debug_markers(path_points, target, 'active'))
        self._publish_control(target.gas, 0.0, target.steer_angle)

    def _inputs_valid(self) -> tuple[bool, str]:
        now_sec = self._now_sec()
        if self._path is None:
            return False, 'waiting for path'
        if len(self._path.poses) < 2:
            return False, 'path has fewer than two points'
        if now_sec - self._path_receive_sec > self._path_timeout_sec:
            return False, 'path stale'
        if self._odom is None:
            return False, 'waiting for odom'
        if now_sec - self._odom_receive_sec > self._odom_timeout_sec:
            return False, 'odom stale'
        return True, 'ok'

    def _path_points_in_control_frame(self) -> tuple[list[Point], str]:
        assert self._path is not None
        source_frame = self._path.header.frame_id or self._control_frame
        raw_points = [pose.pose.position for pose in self._path.poses]
        if source_frame == self._control_frame:
            return self._contiguous_path(raw_points)

        try:
            transform = self._tf_buffer.lookup_transform(self._control_frame, source_frame, Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            return [], f'tf unavailable {source_frame}->{self._control_frame}: {exc}'
        return self._contiguous_path([self._transform_point(point, transform) for point in raw_points])

    def _contiguous_path(self, points: list[Point]) -> tuple[list[Point], str]:
        segment = []
        last_point = None
        for point in points:
            control_point = self._to_control_point_frame(point)
            point_in_window = (
                self._min_x <= control_point.x <= self._max_x
                and abs(control_point.y) <= self._max_abs_y
            )
            if not point_in_window:
                if segment:
                    break
                continue
            if last_point is not None and self._distance_xy(last_point, point) > self._max_path_segment_gap:
                break
            segment.append(point)
            last_point = point
        if len(segment) < 2:
            return [], f'contiguous path has {len(segment)} point(s)'
        return segment, 'ok'

    def _compute_target(self, path_points: list[Point]) -> StanleyTarget | None:
        control_points = [self._to_control_point_frame(point) for point in path_points]
        nearest = self._nearest_path_projection(control_points)
        if nearest is None:
            return None

        nearest_point, heading = nearest
        curvature = self._curvature_for_gas(control_points)
        tangent_x = math.cos(heading)
        tangent_y = math.sin(heading)
        cross_track_error = tangent_x * nearest_point.y - tangent_y * nearest_point.x
        heading_error = self._wrap_angle(heading)
        speed = self._vehicle_speed()
        cross_track_term = math.atan2(self._stanley_gain * cross_track_error, speed + self._speed_softening)
        steer = self._steer_sign * (heading_error + cross_track_term)
        steer = self._clamp(steer, -self._max_steer, self._max_steer)
        steer = self._rate_limit_steer(steer)
        gas = self._gas_for_curvature(curvature)
        return StanleyTarget(
            nearest_point=self._to_control_frame(nearest_point),
            path_heading=heading,
            cross_track_error=cross_track_error,
            heading_error=heading_error,
            cross_track_term=cross_track_term,
            curvature=curvature,
            steer_angle=steer,
            gas=gas,
            speed=speed,
        )

    def _curvature_for_gas(self, control_points: list[Point]) -> float:
        lookahead_result = self._lookahead_point(control_points, self._lookahead_distance)
        if lookahead_result is None:
            return 0.0
        control_target, _ = lookahead_result
        target_distance_sq = max(
            control_target.x * control_target.x + control_target.y * control_target.y,
            1e-6,
        )
        return 2.0 * control_target.y / target_distance_sq

    def _gas_for_curvature(self, curvature: float) -> float:
        curvature_abs = abs(curvature)
        gas_scale = 1.0 / (1.0 + self._curvature_slowdown_gain * curvature_abs)
        return self._clamp(self._cruise_gas * gas_scale, self._min_gas, self._max_gas)

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
            ratio = StanleyPathFollower._clamp(
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

    def _nearest_path_projection(self, control_points: list[Point]) -> tuple[Point, float] | None:
        best_point = None
        best_heading = 0.0
        best_distance_sq = float('inf')
        for start, end in zip(control_points[:-1], control_points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq < 1e-9:
                continue
            ratio = self._clamp(-(start.x * dx + start.y * dy) / length_sq, 0.0, 1.0)
            projected = Point(x=start.x + ratio * dx, y=start.y + ratio * dy, z=start.z)
            distance_sq = projected.x * projected.x + projected.y * projected.y
            if distance_sq < best_distance_sq:
                best_point = projected
                best_heading = math.atan2(dy, dx)
                best_distance_sq = distance_sq
        if best_point is None:
            return None
        return best_point, best_heading

    def _vehicle_speed(self) -> float:
        if self._odom is None:
            return 0.0
        linear = self._odom.twist.twist.linear
        return abs(linear.x)

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
        msg.steer_ang = float(self._clamp(steer, -self._max_steer, self._max_steer))
        msg.steer_ang_vel = 0.0
        msg.steer_ang_acc = 0.0
        self._last_gas = msg.gas
        self._control_pub.publish(msg)

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
        target: StanleyTarget | None,
        status: str,
    ) -> MarkerArray:
        stamp = self._debug_stamp()
        markers = MarkerArray()
        markers.markers.append(self._delete_all(stamp))
        markers.markers.append(self._status_text(stamp, target, status, len(path_points)))
        if path_points:
            markers.markers.append(self._line_strip(1, stamp, 'stanley_path', path_points, self._magenta()))
        if target is not None:
            control_origin = self._control_origin_point()
            markers.markers.append(self._point_list(2, stamp, 'nearest_path_point', [target.nearest_point], self._orange()))
            markers.markers.append(self._line_strip(3, stamp, 'cross_track_error', [control_origin, target.nearest_point], self._white()))
            heading_end = Point(
                x=target.nearest_point.x + 1.5 * math.cos(target.path_heading),
                y=target.nearest_point.y + 1.5 * math.sin(target.path_heading),
                z=target.nearest_point.z,
            )
            markers.markers.append(self._line_strip(4, stamp, 'path_heading', [target.nearest_point, heading_end], self._cyan()))
            markers.markers.append(self._point_list(5, stamp, 'rear_axle_control_point', [control_origin], self._cyan()))
        return markers

    def _debug_stamp(self):
        if self._path is not None:
            stamp = self._path.header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                return stamp
        return self.get_clock().now().to_msg()

    def _delete_all(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._control_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _status_text(self, stamp, target: StanleyTarget | None, status: str, path_count: int) -> Marker:
        marker = self._base_marker(0, stamp, 'stanley_path_follower_status', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 1.5
        marker.pose.position.z = 1.5
        marker.scale.z = 0.28
        marker.color = self._white()
        if target is None:
            marker.text = f'Stanley path follower\nstatus: {status}\npath points: {path_count}'
        else:
            marker.text = (
                f'Stanley path follower\n'
                f'status: {status}  path points: {path_count}\n'
                f'cte: {target.cross_track_error:.2f} m  heading: {math.degrees(target.heading_error):.1f} deg\n'
                f'cte term: {math.degrees(target.cross_track_term):.1f} deg  '
                f'steer: {target.steer_angle:.2f} rad\n'
                f'curvature: {target.curvature:.3f}  speed: {target.speed:.2f} m/s  '
                f'gas: {target.gas:.2f}'
            )
        return marker

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
        return Point(x=self._control_point_x_offset, y=self._control_point_y_offset, z=0.0)

    @staticmethod
    def _transform_point(point: Point, transform: TransformStamped) -> Point:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z = StanleyPathFollower._rotate_vector(
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
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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


def main(args=None):
    rclpy.init(args=args)
    node = StanleyPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
