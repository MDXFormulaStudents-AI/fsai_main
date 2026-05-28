"""Conservative path follower for the ObjectList global path."""

from dataclasses import dataclass
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from vehiclecontrol_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class ControlTarget:
    nearest_index: int
    lookahead_index: int
    nearest_point: Point
    lookahead_point: Point
    target_speed: float
    curvature: float
    cross_track_error: float
    steer_angle: float
    gas: float
    brake: float


class GlobalPathFollower(Node):
    """Follows a nav_msgs/Path using pure pursuit and curvature speed limiting."""

    def __init__(self):
        super().__init__('global_path_follower')

        self.declare_parameter('path_topic', '/student/nav/global_path')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('debug_topic', '/student/nav/control_markers')
        self.declare_parameter('enabled', False)
        self.declare_parameter('allow_frame_mismatch', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('path_timeout_sec', 1.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('wheelbase', 1.6)
        self.declare_parameter('lookahead_distance', 2.8)
        self.declare_parameter('lookahead_time', 0.25)
        self.declare_parameter('min_lookahead_distance', 1.4)
        self.declare_parameter('max_lookahead_distance', 6.0)
        self.declare_parameter('curvature_preview_distance', 12.0)
        self.declare_parameter('curvature_point_step', 4)
        self.declare_parameter('min_speed', 1.2)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_lateral_accel', 3.0)
        self.declare_parameter('max_gas', 0.25)
        self.declare_parameter('cruise_gas', 0.08)
        self.declare_parameter('start_gas', 0.12)
        self.declare_parameter('speed_kp', 0.16)
        self.declare_parameter('brake_kp', 0.25)
        self.declare_parameter('max_brake', 0.4)
        self.declare_parameter('stop_brake', 0.8)
        self.declare_parameter('max_steer', 0.6)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('max_steer_rate', 4.0)
        self.declare_parameter('heading_gain', 0.35)
        self.declare_parameter('cross_track_gain', 0.45)
        self.declare_parameter('cross_track_softening', 1.0)
        self.declare_parameter('selector_ctrl', 1)
        self.declare_parameter('debug_line_width', 0.06)
        self.declare_parameter('debug_point_diameter', 0.25)

        path_topic = self.get_parameter('path_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        control_topic = self.get_parameter('control_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))

        self._enabled = bool(self.get_parameter('enabled').value)
        self._allow_frame_mismatch = bool(self.get_parameter('allow_frame_mismatch').value)
        self._path_timeout_sec = float(self.get_parameter('path_timeout_sec').value)
        self._odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self._lookahead_time = float(self.get_parameter('lookahead_time').value)
        self._min_lookahead_distance = float(self.get_parameter('min_lookahead_distance').value)
        self._max_lookahead_distance = float(self.get_parameter('max_lookahead_distance').value)
        self._curvature_preview_distance = float(self.get_parameter('curvature_preview_distance').value)
        self._curvature_point_step = max(1, int(self.get_parameter('curvature_point_step').value))
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._max_lateral_accel = float(self.get_parameter('max_lateral_accel').value)
        self._max_gas = float(self.get_parameter('max_gas').value)
        self._cruise_gas = float(self.get_parameter('cruise_gas').value)
        self._start_gas = float(self.get_parameter('start_gas').value)
        self._speed_kp = float(self.get_parameter('speed_kp').value)
        self._brake_kp = float(self.get_parameter('brake_kp').value)
        self._max_brake = float(self.get_parameter('max_brake').value)
        self._stop_brake = float(self.get_parameter('stop_brake').value)
        self._max_steer = float(self.get_parameter('max_steer').value)
        self._steer_sign = float(self.get_parameter('steer_sign').value)
        self._max_steer_rate = float(self.get_parameter('max_steer_rate').value)
        self._heading_gain = float(self.get_parameter('heading_gain').value)
        self._cross_track_gain = float(self.get_parameter('cross_track_gain').value)
        self._cross_track_softening = max(0.01, float(self.get_parameter('cross_track_softening').value))
        self._selector_ctrl = int(self.get_parameter('selector_ctrl').value)
        self._debug_line_width = float(self.get_parameter('debug_line_width').value)
        self._debug_point_diameter = float(self.get_parameter('debug_point_diameter').value)

        self._path_points: list[Point] = []
        self._path_frame_id = ''
        self._path_receive_sec = -1.0
        self._odom: Odometry | None = None
        self._odom_receive_sec = -1.0
        self._last_steer = 0.0
        self._last_command_sec = self._now_sec()
        self._last_warning_sec = -10.0
        self._last_stop_log_sec = -10.0
        self._last_stop_reason = ''
        self._last_status = 'waiting for path and odom'
        self._last_target: ControlTarget | None = None

        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._control_pub = self.create_publisher(VehicleControl, control_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._control_tick)

        mode = 'ACTIVE' if self._enabled else 'disabled debug-only'
        self.get_logger().info(
            f'Global path follower started in {mode} mode; path={path_topic}, '
            f'odom={odom_topic}, control={control_topic}, debug={debug_topic}'
        )

    def _on_path(self, msg: Path) -> None:
        self._path_points = [pose.pose.position for pose in msg.poses]
        self._path_frame_id = msg.header.frame_id
        self._path_receive_sec = self._now_sec()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_receive_sec = self._now_sec()

    def _control_tick(self) -> None:
        now_sec = self._now_sec()
        valid, reason = self._inputs_valid(now_sec)
        if not valid:
            self._last_status = reason
            self._last_target = None
            self._debug_pub.publish(self._make_debug_markers(None, reason))
            if self._enabled:
                self._publish_stop(reason)
            return

        target = self._compute_control_target(now_sec)
        if target is None:
            reason = 'unable to compute target'
            self._last_status = reason
            self._last_target = None
            self._debug_pub.publish(self._make_debug_markers(None, reason))
            if self._enabled:
                self._publish_stop(reason)
            return

        self._last_status = 'active' if self._enabled else 'disabled debug-only'
        self._last_target = target
        self._debug_pub.publish(self._make_debug_markers(target, self._last_status))
        if self._enabled:
            self._publish_control(target.gas, target.brake, target.steer_angle)

    def _inputs_valid(self, now_sec: float) -> tuple[bool, str]:
        if len(self._path_points) < 2:
            return False, 'waiting for path'
        if self._odom is None:
            return False, 'waiting for odom'
        if now_sec - self._path_receive_sec > self._path_timeout_sec:
            return False, 'path stale'
        if now_sec - self._odom_receive_sec > self._odom_timeout_sec:
            return False, 'odom stale'

        odom_frame = self._odom.header.frame_id
        if self._path_frame_id and odom_frame and self._path_frame_id != odom_frame:
            if not self._allow_frame_mismatch:
                return False, f'frame mismatch path={self._path_frame_id} odom={odom_frame}'
            if now_sec - self._last_warning_sec > 2.0:
                self.get_logger().warn(
                    f'Ignoring frame mismatch path={self._path_frame_id} odom={odom_frame}; '
                    'allow_frame_mismatch is true'
                )
                self._last_warning_sec = now_sec

        return True, 'ok'

    def _compute_control_target(self, now_sec: float) -> ControlTarget | None:
        odom = self._odom
        if odom is None:
            return None

        vehicle = odom.pose.pose.position
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)
        speed = self._speed_from_odom(odom)

        nearest_index, cross_track_error = self._nearest_forward_index(vehicle, yaw)
        lookahead = self._dynamic_lookahead(speed)
        lookahead_index = self._lookahead_index(nearest_index, lookahead)
        if lookahead_index is None:
            return None

        lookahead_point = self._path_points[lookahead_index]
        steer = self._steering_command(vehicle, yaw, speed, nearest_index, lookahead_index, cross_track_error)
        steer = self._clamp(self._steer_sign * steer, -self._max_steer, self._max_steer)
        steer = self._rate_limit_steer(steer, now_sec)

        path_curvature = self._max_preview_curvature(nearest_index)
        target_speed = self._target_speed(path_curvature, abs(cross_track_error))
        gas, brake = self._speed_command(speed, target_speed)

        return ControlTarget(
            nearest_index=nearest_index,
            lookahead_index=lookahead_index,
            nearest_point=self._path_points[nearest_index],
            lookahead_point=lookahead_point,
            target_speed=target_speed,
            curvature=path_curvature,
            cross_track_error=cross_track_error,
            steer_angle=steer,
            gas=gas,
            brake=brake,
        )

    def _nearest_forward_index(self, vehicle: Point, yaw: float) -> tuple[int, float]:
        best_index = 0
        best_distance_sq = float('inf')
        best_cross_track_error = 0.0
        fallback_index = 0
        fallback_distance_sq = float('inf')
        fallback_cross_track_error = 0.0

        for idx, point in enumerate(self._path_points):
            local_x, local_y = self._to_vehicle_frame(vehicle, yaw, point)
            distance_sq = local_x * local_x + local_y * local_y
            if distance_sq < fallback_distance_sq:
                fallback_distance_sq = distance_sq
                fallback_index = idx
                fallback_cross_track_error = local_y
            if local_x < -1.0:
                continue
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = idx
                best_cross_track_error = local_y

        if best_distance_sq == float('inf'):
            best_index = fallback_index
            best_cross_track_error = fallback_cross_track_error

        return best_index, best_cross_track_error

    def _steering_command(
        self,
        vehicle: Point,
        yaw: float,
        speed: float,
        nearest_index: int,
        lookahead_index: int,
        cross_track_error: float,
    ) -> float:
        lookahead_point = self._path_points[lookahead_index]
        target_x, target_y = self._to_vehicle_frame(vehicle, yaw, lookahead_point)
        target_distance_sq = max(target_x * target_x + target_y * target_y, 1e-6)
        curvature_to_target = 2.0 * target_y / target_distance_sq
        pure_pursuit = math.atan(self._wheelbase * curvature_to_target)

        path_heading = self._path_heading(nearest_index, lookahead_index)
        heading_error = self._normalize_angle(path_heading - yaw)
        cross_track_correction = math.atan2(
            self._cross_track_gain * cross_track_error,
            speed + self._cross_track_softening,
        )
        return pure_pursuit + self._heading_gain * heading_error + cross_track_correction

    def _path_heading(self, nearest_index: int, lookahead_index: int) -> float:
        start_index = self._clamp_index(nearest_index, len(self._path_points) - 1)
        end_index = self._clamp_index(lookahead_index, len(self._path_points) - 1)
        if end_index == start_index:
            if start_index < len(self._path_points) - 1:
                end_index = start_index + 1
            elif start_index > 0:
                start_index -= 1

        start = self._path_points[start_index]
        end = self._path_points[end_index]
        return math.atan2(end.y - start.y, end.x - start.x)

    def _dynamic_lookahead(self, speed: float) -> float:
        lookahead = self._lookahead_distance + speed * self._lookahead_time
        return self._clamp(lookahead, self._min_lookahead_distance, self._max_lookahead_distance)

    def _lookahead_index(self, start_index: int, lookahead: float) -> int | None:
        if not self._path_points:
            return None

        distance = 0.0
        for idx in range(start_index, len(self._path_points) - 1):
            distance += self._distance_xy(self._path_points[idx], self._path_points[idx + 1])
            if distance >= lookahead:
                return idx + 1

        return len(self._path_points) - 1

    def _max_preview_curvature(self, start_index: int) -> float:
        end_index = min(len(self._path_points) - 1, start_index + 1)
        distance = 0.0
        for idx in range(start_index, len(self._path_points) - 1):
            distance += self._distance_xy(self._path_points[idx], self._path_points[idx + 1])
            end_index = idx + 1
            if distance >= self._curvature_preview_distance:
                break

        available_points = end_index - start_index + 1
        if available_points < 3:
            return 0.0

        step = min(self._curvature_point_step, max(1, (available_points - 1) // 2))
        last_start_index = end_index - 2 * step

        max_curvature = 0.0
        for idx in range(start_index, last_start_index + 1):
            p0 = self._path_points[idx]
            p1 = self._path_points[idx + step]
            p2 = self._path_points[idx + 2 * step]
            max_curvature = max(max_curvature, self._curvature_from_points(p0, p1, p2))
        return max_curvature

    def _target_speed(self, curvature: float, cross_track_error: float) -> float:
        if curvature < 1e-4:
            speed = self._max_speed
        else:
            speed = math.sqrt(max(self._max_lateral_accel, 0.1) / curvature)
        speed = self._clamp(speed, self._min_speed, self._max_speed)
        if cross_track_error > 2.0:
            speed = min(speed, self._min_speed + 0.5)
        return speed

    def _speed_command(self, current_speed: float, target_speed: float) -> tuple[float, float]:
        error = target_speed - current_speed
        if error >= -0.1:
            gas = self._cruise_gas + self._speed_kp * max(error, 0.0)
            if current_speed < 0.3 and target_speed > 0.5:
                gas = max(gas, self._start_gas)
            return self._clamp(gas, 0.0, self._max_gas), 0.0

        brake = self._brake_kp * abs(error)
        return 0.0, self._clamp(brake, 0.0, self._max_brake)

    def _rate_limit_steer(self, steer: float, now_sec: float) -> float:
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
        self._control_pub.publish(msg)

    def _publish_stop(self, reason: str) -> None:
        now_sec = self._now_sec()
        if reason != self._last_stop_reason or now_sec - self._last_stop_log_sec > 2.0:
            self.get_logger().warn(f'Publishing stop command: {reason}')
            self._last_stop_reason = reason
            self._last_stop_log_sec = now_sec
        self._publish_control(0.0, self._stop_brake, 0.0)

    def _make_debug_markers(self, target: ControlTarget | None, status: str) -> MarkerArray:
        frame_id = self._path_frame_id or (self._odom.header.frame_id if self._odom else 'odom')
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._delete_all(frame_id, stamp))
        markers.markers.append(self._status_text(frame_id, stamp, target, status))

        if target is None:
            return markers

        markers.markers.append(
            self._point_list(1, frame_id, stamp, 'nearest_path_point', [target.nearest_point], self._blue())
        )
        markers.markers.append(
            self._point_list(2, frame_id, stamp, 'lookahead_path_point', [target.lookahead_point], self._green())
        )
        if self._odom is not None:
            vehicle = self._odom.pose.pose.position
            markers.markers.append(
                self._line_list(3, frame_id, stamp, 'vehicle_to_lookahead', [vehicle, target.lookahead_point], self._white())
            )
        preview = self._path_points[target.nearest_index:target.lookahead_index + 1]
        if len(preview) > 1:
            markers.markers.append(
                self._line_strip(4, frame_id, stamp, 'control_preview_path', preview, self._orange())
            )
        return markers

    def _status_text(self, frame_id: str, stamp, target: ControlTarget | None, status: str) -> Marker:
        marker = self._base_marker(0, frame_id, stamp, 'control_summary', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.7
        marker.scale.z = 0.45
        marker.color = self._white()
        enabled = 'true' if self._enabled else 'false'
        if target is None:
            marker.text = f'Global path follower\nenabled: {enabled}\nstatus: {status}'
        else:
            marker.text = (
                f'Global path follower  enabled: {enabled}\n'
                f'status: {status}\n'
                f'target speed: {target.target_speed:.2f} m/s  curvature: {target.curvature:.3f}\n'
                f'cte: {target.cross_track_error:.2f} m  steer: {target.steer_angle:.2f}\n'
                f'gas: {target.gas:.2f}  brake: {target.brake:.2f}'
            )
        return marker

    def _delete_all(self, frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _line_strip(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.LINE_STRIP)
        marker.scale.x = self._debug_line_width
        marker.color = colour
        marker.points = points
        return marker

    def _line_list(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.LINE_LIST)
        marker.scale.x = self._debug_line_width
        marker.color = colour
        marker.points = points
        return marker

    def _point_list(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.SPHERE_LIST)
        marker.scale.x = self._debug_point_diameter
        marker.scale.y = self._debug_point_diameter
        marker.scale.z = self._debug_point_diameter
        marker.color = colour
        marker.points = points
        return marker

    @staticmethod
    def _base_marker(marker_id: int, frame_id: str, stamp, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    @staticmethod
    def _to_vehicle_frame(vehicle: Point, yaw: float, point: Point) -> tuple[float, float]:
        dx = point.x - vehicle.x
        dy = point.y - vehicle.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy

    @staticmethod
    def _yaw_from_quaternion(quat) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _speed_from_odom(odom: Odometry) -> float:
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        return math.hypot(vx, vy)

    @staticmethod
    def _curvature_from_points(p0: Point, p1: Point, p2: Point) -> float:
        a = GlobalPathFollower._distance_xy(p0, p1)
        b = GlobalPathFollower._distance_xy(p1, p2)
        c = GlobalPathFollower._distance_xy(p0, p2)
        denominator = a * b * c
        if denominator < 1e-6:
            return 0.0
        cross = abs((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x))
        return 2.0 * cross / denominator

    @staticmethod
    def _distance_xy(left: Point, right: Point) -> float:
        return math.hypot(left.x - right.x, left.y - right.y)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    @staticmethod
    def _clamp_index(index: int, max_index: int) -> int:
        return max(0, min(index, max_index))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _blue() -> ColorRGBA:
        return ColorRGBA(r=0.1, g=0.35, b=1.0, a=1.0)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathFollower()
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


if __name__ == '__main__':
    main()
