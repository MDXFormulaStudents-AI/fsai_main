"""Wheel + IMU dead-reckoning odometry for the ADS-DV.

Synthesizes the ``nav_msgs/Odometry`` message the MDX FSAI nav stack already
consumes (``/carmaker/odom`` in sim) from the real vehicle's VCU sensors, so the
rest of the stack — and the ``home -> base_link -> Fr1A`` fixed-frame chain built
by the odom-TF node — works unchanged with ``odom_topic`` repointed here.

Model (2D unicycle):
  v   = mean rear-wheel rpm / 60 * wheel_circumference       [m/s]
  w   = imu.angular_velocity.z  (stationary bias removed)     [rad/s]
  yaw += w*dt ;  x += v*cos(yaw)*dt ;  y += v*sin(yaw)*dt

The fixed frame is anchored at the vehicle pose when the interface first enters
DRIVING, so ``odom`` == Fr1A-at-run-start and gyro bias accumulated during the
stationary handshake is discarded. Twist (v, w) is exact/instantaneous and does
not drift — which is all the follower and curvature estimate need.
"""

from __future__ import annotations

import math

from fsai_interfaces.msg import InterfaceState, WheelSpeeds
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped


class VehicleOdometry(Node):
    """Publishes dead-reckoned Odometry from /vcu/wheel_speeds + /vcu/imu."""

    def __init__(self) -> None:
        super().__init__('vehicle_odometry')

        self.declare_parameter('wheel_speeds_topic', '/vcu/wheel_speeds')
        self.declare_parameter('imu_topic', '/vcu/imu')
        self.declare_parameter('interface_state_topic', '/vehicle/interface_state')
        self.declare_parameter('odom_topic', '/odometry/vehicle')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wheel_circumference_m', 1.674)
        self.declare_parameter('use_rear_wheels_only', True)
        # Re-anchor pose to (0,0,0) when the interface enters DRIVING, pinning the
        # fixed frame at the run start line. Set false to anchor at node startup.
        self.declare_parameter('anchor_on_driving', True)
        # Stationary gyro-bias removal: while |v| < this, average yaw rate into a
        # bias estimate and subtract it during integration.
        self.declare_parameter('stationary_speed_thresh', 0.15)
        self.declare_parameter('bias_alpha', 0.02)
        # Also broadcast the raw odom -> base_frame TF. Leave OFF when the odom-TF
        # node owns the tree (it publishes odom -> home -> base_link).
        self.declare_parameter('publish_tf', False)

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._wheel_circumference_m = max(1e-3, float(self.get_parameter('wheel_circumference_m').value))
        self._use_rear_only = bool(self.get_parameter('use_rear_wheels_only').value)
        self._anchor_on_driving = bool(self.get_parameter('anchor_on_driving').value)
        self._stationary_speed_thresh = float(self.get_parameter('stationary_speed_thresh').value)
        self._bias_alpha = float(self.get_parameter('bias_alpha').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        # Integrator state.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._yaw_bias = 0.0
        self._last_sec: float | None = None
        self._interface_state = InterfaceState.WAIT_FOR_VCU

        self.create_subscription(
            WheelSpeeds, str(self.get_parameter('wheel_speeds_topic').value), self._on_wheel_speeds, 20)
        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value), self._on_imu, 20)
        self.create_subscription(
            InterfaceState, str(self.get_parameter('interface_state_topic').value), self._on_interface_state, 10)
        self._odom_pub = self.create_publisher(Odometry, str(self.get_parameter('odom_topic').value), 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self._publish_tf else None

        self.get_logger().info(
            f'Vehicle odometry: wheels+IMU dead-reckoning -> {self.get_parameter("odom_topic").value} '
            f'({self._odom_frame} -> {self._base_frame}), anchor_on_driving={self._anchor_on_driving}'
        )

    def _on_interface_state(self, msg: InterfaceState) -> None:
        entering_driving = (
            self._interface_state != InterfaceState.DRIVING
            and int(msg.state) == InterfaceState.DRIVING
        )
        self._interface_state = int(msg.state)
        if entering_driving and self._anchor_on_driving:
            self._x = self._y = self._yaw = 0.0
            self.get_logger().info('DRIVING — fixed frame anchored at current pose (0,0,0)')

    def _on_wheel_speeds(self, msg: WheelSpeeds) -> None:
        if self._use_rear_only:
            rpm = 0.5 * (msg.rl_rpm + msg.rr_rpm)
        else:
            rpm = 0.25 * (msg.fl_rpm + msg.fr_rpm + msg.rl_rpm + msg.rr_rpm)
        self._v = (rpm / 60.0) * self._wheel_circumference_m

    def _on_imu(self, msg: Imu) -> None:
        now_sec = self._stamp_to_sec(msg.header.stamp) or self._now_sec()
        if self._last_sec is None:
            self._last_sec = now_sec
            return
        dt = now_sec - self._last_sec
        self._last_sec = now_sec
        if dt <= 0.0 or dt > 0.5:  # skip first tick / big gaps
            return

        omega_raw = msg.angular_velocity.z
        # Stationary bias estimate + removal.
        if abs(self._v) < self._stationary_speed_thresh:
            self._yaw_bias = (1.0 - self._bias_alpha) * self._yaw_bias + self._bias_alpha * omega_raw
        omega = omega_raw - self._yaw_bias

        self._yaw = self._wrap(self._yaw + omega * dt)
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt

        self._publish(msg.header.stamp, omega)

    def _publish(self, stamp, omega: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = self._yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = self._v
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VehicleOdometry()
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
