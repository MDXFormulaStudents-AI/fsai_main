"""Drive a configurable forward distance using CarMaker odometry."""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from vehiclecontrol_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class DistanceState:
    status: str
    travelled: float
    remaining: float
    lateral_error: float
    projected_speed: float
    body_vx: float
    target_speed: float
    gas: float
    brake: float
    complete: bool


class ForwardDistanceController(Node):
    """Throttle/brake-only controller for a straight-line odometry distance."""

    def __init__(self):
        super().__init__('forward_distance_controller')

        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('debug_topic', '/nav/forward_distance_controller_markers')
        self.declare_parameter('enabled', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('target_distance', 10.0)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('stop_speed', 0.05)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('slowdown_distance', 3.0)
        self.declare_parameter('min_gas', 0.03)
        self.declare_parameter('max_gas', 0.18)
        self.declare_parameter('gas_gain', 0.08)
        self.declare_parameter('speed_tolerance', 0.15)
        self.declare_parameter('brake_gain', 0.8)
        self.declare_parameter('max_brake', 0.8)
        self.declare_parameter('hold_brake', 0.8)
        self.declare_parameter('selector_ctrl', 1)
        self.declare_parameter('debug_line_width', 0.06)
        self.declare_parameter('debug_point_diameter', 0.3)
        self.declare_parameter('debug_text_height', 0.28)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        self._enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        control_rate_hz = max(1.0, self.get_parameter('control_rate_hz').get_parameter_value().double_value)
        self._odom_timeout_sec = self.get_parameter('odom_timeout_sec').get_parameter_value().double_value
        self._target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self._distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self._stop_speed = self.get_parameter('stop_speed').get_parameter_value().double_value
        self._max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self._slowdown_distance = self.get_parameter('slowdown_distance').get_parameter_value().double_value
        self._min_gas = self.get_parameter('min_gas').get_parameter_value().double_value
        self._max_gas = self.get_parameter('max_gas').get_parameter_value().double_value
        self._gas_gain = self.get_parameter('gas_gain').get_parameter_value().double_value
        self._speed_tolerance = self.get_parameter('speed_tolerance').get_parameter_value().double_value
        self._brake_gain = self.get_parameter('brake_gain').get_parameter_value().double_value
        self._max_brake = self.get_parameter('max_brake').get_parameter_value().double_value
        self._hold_brake = self.get_parameter('hold_brake').get_parameter_value().double_value
        self._selector_ctrl = int(self.get_parameter('selector_ctrl').get_parameter_value().integer_value)
        self._debug_line_width = self.get_parameter('debug_line_width').get_parameter_value().double_value
        self._debug_point_diameter = self.get_parameter('debug_point_diameter').get_parameter_value().double_value
        self._debug_text_height = self.get_parameter('debug_text_height').get_parameter_value().double_value

        self._odom: Odometry | None = None
        self._odom_receive_sec = -1.0
        self._start_position: Point | None = None
        self._forward_x = 1.0
        self._forward_y = 0.0
        self._odom_frame = 'odom'
        self._child_frame = 'base_link'
        self._completed = False
        self._last_state: DistanceState | None = None

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._control_pub = self.create_publisher(VehicleControl, control_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._control_tick)

        mode = 'ACTIVE' if self._enabled else 'disabled debug-only'
        self.get_logger().info(
            f'Forward distance controller started in {mode} mode; odom={odom_topic}, '
            f'control={control_topic}, debug={debug_topic}, target={self._target_distance:.2f} m'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_receive_sec = self._now_sec()
        if self._start_position is None:
            self._capture_start(msg)

    def _capture_start(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        forward = self._forward_axis(msg.pose.pose.orientation)
        norm = math.hypot(forward.x, forward.y)
        if norm < 1e-9:
            self._forward_x = 1.0
            self._forward_y = 0.0
        else:
            self._forward_x = forward.x / norm
            self._forward_y = forward.y / norm

        self._start_position = Point(x=position.x, y=position.y, z=position.z)
        self._odom_frame = msg.header.frame_id or self._odom_frame
        self._child_frame = msg.child_frame_id or self._child_frame
        yaw = math.degrees(math.atan2(self._forward_y, self._forward_x))
        self.get_logger().info(
            f'Captured start pose in {self._odom_frame}; forward vector='
            f'({self._forward_x:.3f}, {self._forward_y:.3f}), yaw={yaw:.2f} deg'
        )

    def _control_tick(self) -> None:
        state = self._state_from_odom()
        self._last_state = state
        self._debug_pub.publish(self._make_debug_markers(state))

        if not self._enabled:
            return
        self._publish_control(state.gas, state.brake)

    def _state_from_odom(self) -> DistanceState:
        if self._odom is None or self._start_position is None:
            return self._state('waiting for odom', 0.0, self._target_distance, 0.0, 0.0, 0.0, 0.0, 0.0, self._hold_brake, False)

        if self._now_sec() - self._odom_receive_sec > self._odom_timeout_sec:
            travelled, lateral_error = self._travelled_distance(self._odom)
            return self._state('odom stale', travelled, self._target_distance - travelled, lateral_error, 0.0, 0.0, 0.0, 0.0, self._hold_brake, False)

        travelled, lateral_error = self._travelled_distance(self._odom)
        remaining = self._target_distance - travelled
        projected_speed = self._projected_speed(self._odom)
        body_vx = self._odom.twist.twist.linear.x

        if self._completed:
            return self._state('complete hold', travelled, remaining, lateral_error, projected_speed, body_vx, 0.0, 0.0, self._hold_brake, True)

        if remaining <= self._distance_tolerance:
            complete = abs(projected_speed) <= self._stop_speed
            if complete:
                self._completed = True
                status = 'complete'
            else:
                status = 'braking at target'
            return self._state(status, travelled, remaining, lateral_error, projected_speed, body_vx, 0.0, 0.0, self._hold_brake, complete)

        target_speed = self._target_speed(remaining)
        gas, brake = self._pedals_for(target_speed, projected_speed)
        return self._state('driving', travelled, remaining, lateral_error, projected_speed, body_vx, target_speed, gas, brake, False)

    def _state(
        self,
        status: str,
        travelled: float,
        remaining: float,
        lateral_error: float,
        projected_speed: float,
        body_vx: float,
        target_speed: float,
        gas: float,
        brake: float,
        complete: bool,
    ) -> DistanceState:
        return DistanceState(
            status=status,
            travelled=travelled,
            remaining=remaining,
            lateral_error=lateral_error,
            projected_speed=projected_speed,
            body_vx=body_vx,
            target_speed=target_speed,
            gas=self._clamp(gas, 0.0, 1.0),
            brake=self._clamp(brake, 0.0, 1.0),
            complete=complete,
        )

    def _travelled_distance(self, msg: Odometry) -> tuple[float, float]:
        assert self._start_position is not None
        position = msg.pose.pose.position
        dx = position.x - self._start_position.x
        dy = position.y - self._start_position.y
        travelled = dx * self._forward_x + dy * self._forward_y
        lateral_error = -dx * self._forward_y + dy * self._forward_x
        return travelled, lateral_error

    def _projected_speed(self, msg: Odometry) -> float:
        linear = msg.twist.twist.linear
        velocity = self._rotate_child_vector(msg.pose.pose.orientation, linear.x, linear.y, linear.z)
        return velocity.x * self._forward_x + velocity.y * self._forward_y

    def _target_speed(self, remaining: float) -> float:
        max_speed = max(0.0, self._max_speed)
        slowdown_distance = max(self._slowdown_distance, self._distance_tolerance, 1e-6)
        speed = max_speed * self._clamp(remaining / slowdown_distance, 0.0, 1.0)
        if speed < self._stop_speed:
            return 0.0
        return speed

    def _pedals_for(self, target_speed: float, projected_speed: float) -> tuple[float, float]:
        if target_speed <= self._stop_speed:
            return 0.0, self._hold_brake

        speed_error = target_speed - projected_speed
        if speed_error < -self._speed_tolerance:
            brake = self._brake_gain * abs(speed_error)
            return 0.0, self._clamp(brake, 0.0, self._max_brake)

        gas = self._min_gas + self._gas_gain * max(0.0, speed_error)
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

    def _make_debug_markers(self, state: DistanceState) -> MarkerArray:
        stamp = self._debug_marker_stamp()
        markers = MarkerArray()
        marker_list = [
            self._delete_all(stamp),
            self._status_text(stamp, state),
        ]

        if self._start_position is not None:
            start = self._copy_point(self._start_position)
            target = Point(
                x=self._start_position.x + self._target_distance * self._forward_x,
                y=self._start_position.y + self._target_distance * self._forward_y,
                z=self._start_position.z,
            )
            marker_list.extend(
                [
                    self._line_strip(1, stamp, 'target_line', [start, target], self._cyan()),
                    self._sphere(2, stamp, 'start', start, self._red()),
                    self._sphere(3, stamp, 'target', target, self._green()),
                ]
            )

        if self._odom is not None:
            current = self._copy_point(self._odom.pose.pose.position)
            marker_list.append(self._sphere(4, stamp, 'current_base_link', current, self._orange()))
        markers.markers = marker_list
        return markers

    def _debug_marker_stamp(self):
        if self._odom is not None:
            stamp = self._odom.header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                return stamp
        return self.get_clock().now().to_msg()

    def _delete_all(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._odom_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(self, marker_id: int, stamp, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._odom_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000
        return marker

    def _status_text(self, stamp, state: DistanceState) -> Marker:
        marker = self._base_marker(0, stamp, 'forward_distance_controller_status', Marker.TEXT_VIEW_FACING)
        marker.pose.position.z = 1.5
        if self._odom is not None:
            marker.pose.position.x = self._odom.pose.pose.position.x
            marker.pose.position.y = self._odom.pose.pose.position.y
        marker.scale.z = self._debug_text_height
        marker.color = self._white()
        enabled = 'true' if self._enabled else 'false'
        marker.text = (
            f'Forward distance controller  enabled: {enabled}\n'
            f'status: {state.status}  target: {self._target_distance:.2f} m\n'
            f'travelled: {state.travelled:.2f} m  remaining: {state.remaining:.2f} m  '
            f'lateral: {state.lateral_error:.2f} m\n'
            f'speed: projected={state.projected_speed:.2f} m/s  body_vx={state.body_vx:.2f} m/s  '
            f'target={state.target_speed:.2f} m/s\n'
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
    ) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.LINE_STRIP)
        marker.scale.x = self._debug_line_width
        marker.color = colour
        marker.points = points
        return marker

    def _sphere(self, marker_id: int, stamp, namespace: str, point: Point, colour: ColorRGBA) -> Marker:
        marker = self._base_marker(marker_id, stamp, namespace, Marker.SPHERE)
        marker.pose.position = point
        marker.scale.x = self._debug_point_diameter
        marker.scale.y = self._debug_point_diameter
        marker.scale.z = self._debug_point_diameter
        marker.color = colour
        return marker

    @staticmethod
    def _copy_point(point: Point) -> Point:
        return Point(x=point.x, y=point.y, z=point.z)

    @staticmethod
    def _forward_axis(quaternion) -> Point:
        return ForwardDistanceController._rotate_child_vector(quaternion, 1.0, 0.0, 0.0)

    @staticmethod
    def _rotate_child_vector(quaternion, x: float, y: float, z: float) -> Point:
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 1e-9:
            return Point(x=x, y=y, z=z)

        qx = quaternion.x / norm
        qy = quaternion.y / norm
        qz = quaternion.z / norm
        qw = quaternion.w / norm
        return Point(
            x=(1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z,
            y=2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z,
            z=2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z,
        )

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _red() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)

    @staticmethod
    def _cyan() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardDistanceController()
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
