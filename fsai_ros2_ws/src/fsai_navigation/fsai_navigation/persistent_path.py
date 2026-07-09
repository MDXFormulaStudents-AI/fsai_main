"""Short-lived fixed-frame path memory for local navigation dropouts."""

from __future__ import annotations

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class MemoryStatus:
    valid: bool
    reason: str
    age_sec: float
    remaining_length: float
    cross_track_error: float
    remaining_points: list[Point]


class PersistentPath(Node):
    """Stores valid paths in a fixed frame and republishes them while safe."""

    def __init__(self):
        super().__init__('persistent_path')

        self.declare_parameter('input_path_topic', '/nav/perceived_path')
        self.declare_parameter('output_path_topic', '/nav/persistent_path')
        self.declare_parameter('debug_topic', '/nav/persistent_path_markers')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('memory_frame', 'home')
        self.declare_parameter('control_frame', 'Fr1A')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_hold_sec', 3.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('min_store_points', 3)
        self.declare_parameter('min_store_path_length', 3.0)
        self.declare_parameter('min_remaining_path_length', 2.0)
        self.declare_parameter('min_forward_x', -0.5)
        self.declare_parameter('max_forward_x', 40.0)
        self.declare_parameter('max_abs_y', 12.0)
        self.declare_parameter('max_path_segment_gap', 2.5)
        self.declare_parameter('max_cross_track_error', 2.5)
        self.declare_parameter('debug_line_width', 0.07)
        self.declare_parameter('debug_point_diameter', 0.22)

        input_path_topic = str(self.get_parameter('input_path_topic').value)
        output_path_topic = str(self.get_parameter('output_path_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._memory_frame = str(self.get_parameter('memory_frame').value)
        self._control_frame = str(self.get_parameter('control_frame').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self._max_hold_sec = max(0.0, float(self.get_parameter('max_hold_sec').value))
        self._odom_timeout_sec = max(0.0, float(self.get_parameter('odom_timeout_sec').value))
        self._require_odom = bool(self.get_parameter('require_odom').value)
        self._min_store_points = max(2, int(self.get_parameter('min_store_points').value))
        self._min_store_path_length = max(0.0, float(self.get_parameter('min_store_path_length').value))
        self._min_remaining_path_length = max(0.0, float(self.get_parameter('min_remaining_path_length').value))
        self._min_forward_x = float(self.get_parameter('min_forward_x').value)
        self._max_forward_x = float(self.get_parameter('max_forward_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._max_path_segment_gap = max(0.0, float(self.get_parameter('max_path_segment_gap').value))
        self._max_cross_track_error = max(0.0, float(self.get_parameter('max_cross_track_error').value))
        self._debug_line_width = float(self.get_parameter('debug_line_width').value)
        self._debug_point_diameter = float(self.get_parameter('debug_point_diameter').value)

        self._memory_path: list[Point] = []
        self._memory_receive_sec = -1.0
        self._last_store_reason = 'waiting for valid input path'
        self._last_publish_reason = 'waiting for memory'
        self._odom_receive_sec = -1.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, input_path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._path_pub = self.create_publisher(Path, output_path_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)

        self.get_logger().info(
            f'Persistent path listening on {input_path_topic}; publishing path={output_path_topic}, '
            f'debug={debug_topic}, memory_frame={self._memory_frame}, control_frame={self._control_frame}'
        )

    def _on_path(self, msg: Path) -> None:
        stored_path, reason = self._stored_path_from_msg(msg)
        if stored_path:
            self._memory_path = stored_path
            self._memory_receive_sec = self._now_sec()
        self._last_store_reason = reason

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_receive_sec = self._now_sec()

    def _publish_tick(self) -> None:
        status = self._memory_status()
        self._last_publish_reason = status.reason
        path_points = self._memory_path if status.valid else []
        self._path_pub.publish(self._make_path_msg(path_points))
        self._debug_pub.publish(self._make_debug_markers(status, path_points))

    def _stored_path_from_msg(self, msg: Path) -> tuple[list[Point], str]:
        points = [pose.pose.position for pose in msg.poses]
        valid, reason = self._path_is_storable(points)
        if not valid:
            return [], reason

        source_frame = msg.header.frame_id or self._memory_frame
        if source_frame == self._memory_frame:
            memory_points = [self._copy_point(point) for point in points]
        else:
            try:
                transform = self._tf_buffer.lookup_transform(self._memory_frame, source_frame, Time())
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                return [], f'tf unavailable {source_frame}->{self._memory_frame}: {exc}'
            memory_points = [self._transform_point(point, transform) for point in points]

        valid, reason = self._path_is_storable(memory_points)
        if not valid:
            return [], reason
        return memory_points, f'stored {len(memory_points)} point(s) from {source_frame}'

    def _path_is_storable(self, points: list[Point]) -> tuple[bool, str]:
        if len(points) < self._min_store_points:
            return False, f'input path has {len(points)} point(s)'
        if any(not self._point_is_finite(point) for point in points):
            return False, 'input path has non-finite point(s)'
        length = self._path_length(points)
        if length < self._min_store_path_length:
            return False, f'input path too short: {length:.2f} m'
        max_gap = self._max_segment_gap(points)
        if max_gap > self._max_path_segment_gap:
            return False, f'input path gap too large: {max_gap:.2f} m'
        return True, 'ok'

    def _memory_status(self) -> MemoryStatus:
        now_sec = self._now_sec()
        if not self._memory_path:
            return self._status(False, 'waiting for memory', float('inf'), [], float('nan'))

        age_sec = now_sec - self._memory_receive_sec
        if age_sec > self._max_hold_sec:
            return self._status(False, 'memory expired by age', age_sec, [], float('nan'))

        if self._require_odom:
            if self._odom_receive_sec < 0.0:
                return self._status(False, 'waiting for odom', age_sec, [], float('nan'))
            if now_sec - self._odom_receive_sec > self._odom_timeout_sec:
                return self._status(False, 'odom stale', age_sec, [], float('nan'))

        control_points, reason = self._memory_points_in_control_frame()
        if not control_points:
            return self._status(False, reason, age_sec, [], float('nan'))

        remaining_points = self._remaining_forward_segment(control_points)
        if len(remaining_points) < 2:
            return self._status(False, f'remaining path has {len(remaining_points)} point(s)', age_sec, remaining_points, float('nan'))

        remaining_length = self._path_length(remaining_points)
        cross_track_error = self._cross_track_error(remaining_points)
        if remaining_length < self._min_remaining_path_length:
            return MemoryStatus(False, f'remaining path too short: {remaining_length:.2f} m', age_sec, remaining_length, cross_track_error, remaining_points)
        if abs(cross_track_error) > self._max_cross_track_error:
            return MemoryStatus(False, f'cross-track error too large: {cross_track_error:.2f} m', age_sec, remaining_length, cross_track_error, remaining_points)
        return MemoryStatus(True, 'active memory', age_sec, remaining_length, cross_track_error, remaining_points)

    def _status(
        self,
        valid: bool,
        reason: str,
        age_sec: float,
        remaining_points: list[Point],
        cross_track_error: float,
    ) -> MemoryStatus:
        remaining_length = self._path_length(remaining_points) if remaining_points else 0.0
        return MemoryStatus(valid, reason, age_sec, remaining_length, cross_track_error, remaining_points)

    def _memory_points_in_control_frame(self) -> tuple[list[Point], str]:
        if self._memory_frame == self._control_frame:
            return [self._copy_point(point) for point in self._memory_path], 'ok'
        try:
            transform = self._tf_buffer.lookup_transform(self._control_frame, self._memory_frame, Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            return [], f'tf unavailable {self._memory_frame}->{self._control_frame}: {exc}'
        return [self._transform_point(point, transform) for point in self._memory_path], 'ok'

    def _remaining_forward_segment(self, points: list[Point]) -> list[Point]:
        segment = []
        last_point = None
        for point in points:
            point_in_window = (
                self._min_forward_x <= point.x <= self._max_forward_x
                and abs(point.y) <= self._max_abs_y
            )
            if not point_in_window:
                if segment:
                    break
                continue
            if last_point is not None and self._distance_xy(last_point, point) > self._max_path_segment_gap:
                break
            segment.append(point)
            last_point = point
        return segment

    def _cross_track_error(self, points: list[Point]) -> float:
        best_distance_sq = float('inf')
        best_error = float('nan')
        for start, end in zip(points[:-1], points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq < 1e-9:
                continue
            ratio = self._clamp(-(start.x * dx + start.y * dy) / length_sq, 0.0, 1.0)
            projected = Point(x=start.x + ratio * dx, y=start.y + ratio * dy, z=start.z)
            distance_sq = projected.x * projected.x + projected.y * projected.y
            if distance_sq < best_distance_sq:
                heading = math.atan2(dy, dx)
                tangent_x = math.cos(heading)
                tangent_y = math.sin(heading)
                best_error = tangent_x * projected.y - tangent_y * projected.x
                best_distance_sq = distance_sq
        return best_error

    def _make_path_msg(self, points: list[Point]) -> Path:
        msg = Path()
        msg.header.frame_id = self._memory_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = self._copy_point(point)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _make_debug_markers(self, status: MemoryStatus, path_points: list[Point]) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._delete_all(stamp))
        markers.markers.append(self._status_text(stamp, status, len(path_points)))
        if path_points:
            markers.markers.append(self._line_strip(1, stamp, self._memory_frame, 'remembered_path_home', path_points, self._green()))
        if status.remaining_points:
            markers.markers.append(self._line_strip(2, stamp, self._control_frame, 'remaining_path_control_frame', status.remaining_points, self._magenta()))
            nearest = self._nearest_point(status.remaining_points)
            if nearest is not None:
                markers.markers.append(self._point_list(3, stamp, self._control_frame, 'nearest_memory_point', [nearest], self._orange()))
        return markers

    def _delete_all(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._memory_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _status_text(self, stamp, status: MemoryStatus, path_count: int) -> Marker:
        marker = self._base_marker(0, stamp, self._control_frame, 'persistent_path_status', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 1.5
        marker.pose.position.z = 1.8
        marker.scale.z = 0.28
        marker.color = self._white()
        age_text = 'inf' if math.isinf(status.age_sec) else f'{status.age_sec:.2f}'
        cte_text = 'nan' if math.isnan(status.cross_track_error) else f'{status.cross_track_error:.2f}'
        marker.text = (
            f'Persistent path\n'
            f'status: {status.reason}\n'
            f'valid: {status.valid}  age: {age_text} s\n'
            f'path points: {path_count}  remaining: {status.remaining_length:.2f} m\n'
            f'cte: {cte_text} m  store: {self._last_store_reason}'
        )
        return marker

    def _line_strip(
        self,
        marker_id: int,
        stamp,
        frame_id: str,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, frame_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = self._debug_line_width
        marker.color = colour
        marker.points = [self._copy_point(point) for point in points]
        return marker

    def _point_list(
        self,
        marker_id: int,
        stamp,
        frame_id: str,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, frame_id, namespace, Marker.SPHERE_LIST)
        marker.scale.x = self._debug_point_diameter
        marker.scale.y = self._debug_point_diameter
        marker.scale.z = self._debug_point_diameter
        marker.color = colour
        marker.points = [self._copy_point(point) for point in points]
        return marker

    @staticmethod
    def _base_marker(marker_id: int, stamp, frame_id: str, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
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
    def _nearest_point(points: list[Point]) -> Point | None:
        if not points:
            return None
        return min(points, key=lambda point: point.x * point.x + point.y * point.y)

    @staticmethod
    def _path_length(points: list[Point]) -> float:
        return sum(PersistentPath._distance_xy(start, end) for start, end in zip(points[:-1], points[1:]))

    @staticmethod
    def _max_segment_gap(points: list[Point]) -> float:
        if len(points) < 2:
            return 0.0
        return max(PersistentPath._distance_xy(start, end) for start, end in zip(points[:-1], points[1:]))

    @staticmethod
    def _point_is_finite(point: Point) -> bool:
        return math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)

    @staticmethod
    def _copy_point(point: Point) -> Point:
        return Point(x=point.x, y=point.y, z=point.z)

    @staticmethod
    def _transform_point(point: Point, transform: TransformStamped) -> Point:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z = PersistentPath._rotate_vector(
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
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

    @staticmethod
    def _magenta() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.0, b=0.9, a=1.0)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersistentPath()
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
