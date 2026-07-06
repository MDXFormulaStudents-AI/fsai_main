#!/usr/bin/env python3
"""Broadcast CarMaker odometry in a startup-aligned fixed frame."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class CarmakerOdomTf(Node):
    """Converts /carmaker/odom into TF and RViz debug markers.

    Default mode publishes ``odom -> home -> base_link`` where ``home`` is the
    first observed base_link pose. That makes ``home -> base_link`` identity at
    node calibration/start while keeping raw CarMaker ``odom`` connected for
    debug markers that are naturally expressed in odom.
    """

    def __init__(self):
        super().__init__('carmaker_odom_tf')

        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('debug_topic', '/debug/carmaker_odom_tf_markers')
        self.declare_parameter('trace_topic', '/debug/carmaker_odom_tf_path')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_aligned_frame', True)
        self.declare_parameter('publish_parent_to_aligned_frame', True)
        self.declare_parameter('publish_base_link_to_fr1a_tf', True)
        self.declare_parameter('aligned_frame_id', 'home')
        self.declare_parameter('fr1a_frame_id', 'Fr1A')
        self.declare_parameter('base_link_to_fr1a_x', 0.0)
        self.declare_parameter('base_link_to_fr1a_y', 0.0)
        self.declare_parameter('base_link_to_fr1a_z', 0.0)
        self.declare_parameter('base_link_to_fr1a_roll_deg', 0.0)
        self.declare_parameter('base_link_to_fr1a_pitch_deg', 0.0)
        self.declare_parameter('base_link_to_fr1a_yaw_deg', 0.0)
        self.declare_parameter('alignment_mode', 'initial_pose')
        self.declare_parameter('manual_yaw_offset_deg', 0.0)
        self.declare_parameter('align_initial_position', True)
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('publish_trace_path', True)
        self.declare_parameter('child_frame_id_override', '')
        self.declare_parameter('line_width', 0.06)
        self.declare_parameter('origin_marker_diameter', 0.25)
        self.declare_parameter('base_link_marker_diameter', 0.35)
        self.declare_parameter('heading_arrow_length', 2.0)
        self.declare_parameter('heading_arrow_shaft_diameter', 0.08)
        self.declare_parameter('heading_arrow_head_diameter', 0.22)
        self.declare_parameter('heading_arrow_head_length', 0.35)
        self.declare_parameter('velocity_arrow_scale', 0.75)
        self.declare_parameter('min_velocity_arrow_speed', 0.05)
        self.declare_parameter('trail_enabled', True)
        self.declare_parameter('trail_max_points', 0)
        self.declare_parameter('trail_min_distance', 0.1)
        self.declare_parameter('text_height', 0.28)
        self.declare_parameter('marker_lifetime_sec', 0.25)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        self._trace_topic = self.get_parameter('trace_topic').get_parameter_value().string_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self._publish_aligned_frame = self.get_parameter('publish_aligned_frame').get_parameter_value().bool_value
        self._publish_parent_to_aligned_frame = self.get_parameter('publish_parent_to_aligned_frame').get_parameter_value().bool_value
        self._publish_base_link_to_fr1a_tf = self.get_parameter('publish_base_link_to_fr1a_tf').get_parameter_value().bool_value
        self._aligned_frame_id = self.get_parameter('aligned_frame_id').get_parameter_value().string_value
        self._fr1a_frame_id = self.get_parameter('fr1a_frame_id').get_parameter_value().string_value
        self._base_link_to_fr1a_x = self.get_parameter('base_link_to_fr1a_x').get_parameter_value().double_value
        self._base_link_to_fr1a_y = self.get_parameter('base_link_to_fr1a_y').get_parameter_value().double_value
        self._base_link_to_fr1a_z = self.get_parameter('base_link_to_fr1a_z').get_parameter_value().double_value
        self._base_link_to_fr1a_roll = math.radians(
            self.get_parameter('base_link_to_fr1a_roll_deg').get_parameter_value().double_value
        )
        self._base_link_to_fr1a_pitch = math.radians(
            self.get_parameter('base_link_to_fr1a_pitch_deg').get_parameter_value().double_value
        )
        self._base_link_to_fr1a_yaw = math.radians(
            self.get_parameter('base_link_to_fr1a_yaw_deg').get_parameter_value().double_value
        )
        self._alignment_mode = self.get_parameter('alignment_mode').get_parameter_value().string_value.lower()
        self._manual_yaw_offset = math.radians(
            self.get_parameter('manual_yaw_offset_deg').get_parameter_value().double_value
        )
        self._align_initial_position = self.get_parameter('align_initial_position').get_parameter_value().bool_value
        self._publish_debug_markers = self.get_parameter('publish_debug_markers').get_parameter_value().bool_value
        self._publish_trace_path = self.get_parameter('publish_trace_path').get_parameter_value().bool_value
        self._child_frame_id_override = self.get_parameter('child_frame_id_override').get_parameter_value().string_value
        self._line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self._origin_marker_diameter = self.get_parameter('origin_marker_diameter').get_parameter_value().double_value
        self._base_link_marker_diameter = self.get_parameter('base_link_marker_diameter').get_parameter_value().double_value
        self._heading_arrow_length = self.get_parameter('heading_arrow_length').get_parameter_value().double_value
        self._heading_arrow_shaft_diameter = self.get_parameter('heading_arrow_shaft_diameter').get_parameter_value().double_value
        self._heading_arrow_head_diameter = self.get_parameter('heading_arrow_head_diameter').get_parameter_value().double_value
        self._heading_arrow_head_length = self.get_parameter('heading_arrow_head_length').get_parameter_value().double_value
        self._velocity_arrow_scale = self.get_parameter('velocity_arrow_scale').get_parameter_value().double_value
        self._min_velocity_arrow_speed = self.get_parameter('min_velocity_arrow_speed').get_parameter_value().double_value
        self._trail_enabled = self.get_parameter('trail_enabled').get_parameter_value().bool_value
        self._trail_max_points = self.get_parameter('trail_max_points').get_parameter_value().integer_value
        self._trail_min_distance = self.get_parameter('trail_min_distance').get_parameter_value().double_value
        self._text_height = self.get_parameter('text_height').get_parameter_value().double_value
        self._marker_lifetime_sec = self.get_parameter('marker_lifetime_sec').get_parameter_value().double_value

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._debug_pub = self.create_publisher(MarkerArray, self._debug_topic, 10)
        self._trace_pub = self.create_publisher(Path, self._trace_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._last_empty_frame_warn_sec = -10.0
        self._last_mode_warn_sec = -10.0
        self._trail_frame = ''
        self._trail_points: list[Point] = []
        self._initial_raw_parent_frame = ''
        self._initial_position: Point | None = None
        self._initial_orientation: tuple[float, float, float, float] | None = None
        self._initial_yaw = 0.0
        self._published_base_link_to_fr1a_parent = ''

        output_frame = self._aligned_frame_id if self._publish_aligned_frame else 'raw odom'
        self.get_logger().info(
            f'CarMaker odom TF listening on {odom_topic}; publish_tf={self._publish_tf}, '
            f'output_frame={output_frame}, alignment_mode={self._alignment_mode}, '
            f'publish_parent_to_aligned_frame={self._publish_parent_to_aligned_frame}, '
            f'publish_base_link_to_fr1a_tf={self._publish_base_link_to_fr1a_tf}, '
            f'debug={self._publish_debug_markers} on {self._debug_topic}, '
            f'trace={self._publish_trace_path} on {self._trace_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        raw_parent_frame = msg.header.frame_id
        child_frame = self._child_frame_id_override or msg.child_frame_id
        if not raw_parent_frame or not child_frame:
            self._warn_empty_frame(raw_parent_frame, child_frame)
            return
        self._publish_base_link_to_fr1a_transform(child_frame)

        transform = self._output_transform_from_odom(msg, raw_parent_frame, child_frame)
        if transform is None:
            return

        base_link_origin = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z,
        )
        self._update_trail(transform.header.frame_id, base_link_origin)

        parent_to_aligned = self._parent_to_aligned_transform_from_odom(msg, raw_parent_frame, child_frame, transform)
        if self._publish_tf:
            if parent_to_aligned is not None:
                self._tf_broadcaster.sendTransform(parent_to_aligned)
            self._tf_broadcaster.sendTransform(transform)
        if self._publish_trace_path:
            self._trace_pub.publish(self._make_trace_path(transform.header.frame_id, msg.header.stamp))
        if self._publish_debug_markers:
            self._debug_pub.publish(self._make_debug_markers(msg, raw_parent_frame, transform))

    def _warn_empty_frame(self, parent_frame: str, child_frame: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_empty_frame_warn_sec < 2.0:
            return
        self._last_empty_frame_warn_sec = now_sec
        self.get_logger().warn(
            f'Cannot publish odometry TF with empty frame id: '
            f'parent={parent_frame!r}, child={child_frame!r}'
        )

    def _publish_base_link_to_fr1a_transform(self, child_frame: str) -> None:
        if not self._publish_tf or not self._publish_base_link_to_fr1a_tf:
            return
        if not child_frame or not self._fr1a_frame_id:
            return
        if child_frame == self._fr1a_frame_id:
            self._warn_bad_mode(f'Cannot publish self transform {child_frame!r} -> {self._fr1a_frame_id!r}')
            return
        if self._published_base_link_to_fr1a_parent == child_frame:
            return

        transform = TransformStamped()
        transform.header.frame_id = child_frame
        transform.child_frame_id = self._fr1a_frame_id
        transform.transform.translation.x = self._base_link_to_fr1a_x
        transform.transform.translation.y = self._base_link_to_fr1a_y
        transform.transform.translation.z = self._base_link_to_fr1a_z
        transform.transform.rotation = self._quaternion_to_msg(
            self._quaternion_from_rpy_tuple(
                self._base_link_to_fr1a_roll,
                self._base_link_to_fr1a_pitch,
                self._base_link_to_fr1a_yaw,
            )
        )
        self._static_tf_broadcaster.sendTransform(transform)
        self._published_base_link_to_fr1a_parent = child_frame
        self.get_logger().info(
            f'Published static {child_frame} -> {self._fr1a_frame_id} transform: '
            f't=({self._base_link_to_fr1a_x:.3f}, {self._base_link_to_fr1a_y:.3f}, '
            f'{self._base_link_to_fr1a_z:.3f}), rpy_deg=('
            f'{math.degrees(self._base_link_to_fr1a_roll):.2f}, '
            f'{math.degrees(self._base_link_to_fr1a_pitch):.2f}, '
            f'{math.degrees(self._base_link_to_fr1a_yaw):.2f})'
        )

    def _output_transform_from_odom(
        self,
        msg: Odometry,
        raw_parent_frame: str,
        child_frame: str,
    ) -> TransformStamped | None:
        if self._publish_aligned_frame:
            return self._aligned_transform_from_odom(msg, raw_parent_frame, child_frame)
        return self._raw_transform_from_odom(msg, raw_parent_frame, child_frame)

    @staticmethod
    def _raw_transform_from_odom(msg: Odometry, parent_frame: str, child_frame: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        return transform

    def _aligned_transform_from_odom(
        self,
        msg: Odometry,
        raw_parent_frame: str,
        child_frame: str,
    ) -> TransformStamped | None:
        if not self._aligned_frame_id:
            return None
        if self._aligned_frame_id == child_frame:
            self._warn_bad_mode(f'aligned_frame_id must not equal child frame {child_frame!r}')
            return None

        self._capture_initial_pose_if_needed(msg, raw_parent_frame)
        translation, rotation = self._relative_pose_from_initial(msg)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._aligned_frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation.x
        transform.transform.translation.y = translation.y
        transform.transform.translation.z = translation.z
        transform.transform.rotation = self._quaternion_to_msg(rotation)
        return transform

    def _parent_to_aligned_transform_from_odom(
        self,
        msg: Odometry,
        raw_parent_frame: str,
        child_frame: str,
        aligned_transform: TransformStamped,
    ) -> TransformStamped | None:
        if not self._publish_aligned_frame or not self._publish_parent_to_aligned_frame:
            return None
        if not self._aligned_frame_id or self._aligned_frame_id == raw_parent_frame:
            return None
        if aligned_transform.header.frame_id != self._aligned_frame_id or aligned_transform.child_frame_id != child_frame:
            return None

        raw_translation = Point(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            z=msg.pose.pose.position.z,
        )
        raw_rotation = self._quaternion_tuple(msg.pose.pose.orientation)
        aligned_translation = Point(
            x=aligned_transform.transform.translation.x,
            y=aligned_transform.transform.translation.y,
            z=aligned_transform.transform.translation.z,
        )
        aligned_rotation = self._quaternion_tuple(aligned_transform.transform.rotation)

        inverse_translation, inverse_rotation = self._inverse_transform_components(aligned_translation, aligned_rotation)
        translation, rotation = self._compose_transform_components(
            raw_translation,
            raw_rotation,
            inverse_translation,
            inverse_rotation,
        )

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = raw_parent_frame
        transform.child_frame_id = self._aligned_frame_id
        transform.transform.translation.x = translation.x
        transform.transform.translation.y = translation.y
        transform.transform.translation.z = translation.z
        transform.transform.rotation = self._quaternion_to_msg(rotation)
        return transform

    @classmethod
    def _inverse_transform_components(
        cls,
        translation: Point,
        rotation: tuple[float, float, float, float],
    ) -> tuple[Point, tuple[float, float, float, float]]:
        inverse_rotation = cls._quaternion_inverse(rotation)
        inverse_translation = cls._rotate_point(
            inverse_rotation,
            Point(x=-translation.x, y=-translation.y, z=-translation.z),
        )
        return inverse_translation, inverse_rotation

    @classmethod
    def _compose_transform_components(
        cls,
        left_translation: Point,
        left_rotation: tuple[float, float, float, float],
        right_translation: Point,
        right_rotation: tuple[float, float, float, float],
    ) -> tuple[Point, tuple[float, float, float, float]]:
        rotated_translation = cls._rotate_point(left_rotation, right_translation)
        return (
            Point(
                x=left_translation.x + rotated_translation.x,
                y=left_translation.y + rotated_translation.y,
                z=left_translation.z + rotated_translation.z,
            ),
            cls._quaternion_multiply(left_rotation, right_rotation),
        )

    def _warn_bad_mode(self, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_mode_warn_sec < 2.0:
            return
        self._last_mode_warn_sec = now_sec
        self.get_logger().warn(message)

    def _capture_initial_pose_if_needed(self, msg: Odometry, raw_parent_frame: str) -> None:
        if raw_parent_frame != self._initial_raw_parent_frame:
            self._initial_raw_parent_frame = raw_parent_frame
            self._initial_position = None
            self._initial_orientation = None

        if self._initial_position is not None and self._initial_orientation is not None:
            return

        position = msg.pose.pose.position
        self._initial_position = Point(x=position.x, y=position.y, z=position.z)
        self._initial_orientation = self._quaternion_tuple(msg.pose.pose.orientation)
        self._initial_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.get_logger().info(
            f'Captured {raw_parent_frame}->base startup pose for {self._aligned_frame_id}; '
            f'position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}), '
            f'yaw={math.degrees(self._initial_yaw):.2f} deg'
        )

    def _relative_pose_from_initial(self, msg: Odometry) -> tuple[Point, tuple[float, float, float, float]]:
        current_position = msg.pose.pose.position
        current_orientation = self._quaternion_tuple(msg.pose.pose.orientation)

        if self._alignment_mode == 'disabled':
            return (
                Point(x=current_position.x, y=current_position.y, z=current_position.z),
                current_orientation,
            )
        if self._alignment_mode == 'manual':
            align_rotation = self._quaternion_from_yaw_tuple(self._manual_yaw_offset)
            translation_source = Point(x=current_position.x, y=current_position.y, z=current_position.z)
            rotation = self._quaternion_multiply(align_rotation, current_orientation)
            return self._rotate_point(align_rotation, translation_source), rotation
        if self._alignment_mode == 'initial_yaw':
            align_rotation = self._quaternion_from_yaw_tuple(-self._initial_yaw)
            translation_source = self._position_delta_from_initial(current_position)
            if not self._align_initial_position:
                translation_source = Point(x=current_position.x, y=current_position.y, z=current_position.z)
            rotation = self._quaternion_multiply(align_rotation, current_orientation)
            return self._rotate_point(align_rotation, translation_source), rotation
        if self._alignment_mode != 'initial_pose':
            self._warn_bad_mode(f'Unknown alignment_mode={self._alignment_mode!r}; using initial_pose')
            self._alignment_mode = 'initial_pose'

        assert self._initial_orientation is not None
        inverse_initial = self._quaternion_inverse(self._initial_orientation)
        translation_source = self._position_delta_from_initial(current_position)
        if not self._align_initial_position:
            translation_source = Point(x=current_position.x, y=current_position.y, z=current_position.z)
        rotation = self._quaternion_multiply(inverse_initial, current_orientation)
        return self._rotate_point(inverse_initial, translation_source), rotation

    def _position_delta_from_initial(self, position) -> Point:
        if self._initial_position is None:
            return Point(x=position.x, y=position.y, z=position.z)
        return Point(
            x=position.x - self._initial_position.x,
            y=position.y - self._initial_position.y,
            z=position.z - self._initial_position.z,
        )

    def _make_debug_markers(
        self,
        msg: Odometry,
        raw_parent_frame: str,
        transform: TransformStamped,
    ) -> MarkerArray:
        frame_id = transform.header.frame_id
        child_frame = transform.child_frame_id
        position = transform.transform.translation
        orientation = transform.transform.rotation
        origin = Point(x=0.0, y=0.0, z=0.0)
        base_link_origin = Point(x=position.x, y=position.y, z=position.z)
        heading_end = self._heading_arrow_end(base_link_origin, orientation)
        velocity_end = self._velocity_arrow_end(base_link_origin, orientation, msg)

        marker_list = [
            self._delete_all(frame_id, msg.header.stamp),
            self._line_strip(
                1,
                frame_id,
                msg.header.stamp,
                'fixed_frame_to_base_link_line',
                [origin, base_link_origin],
                ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0),
            ),
            self._sphere(
                2,
                frame_id,
                msg.header.stamp,
                'fixed_frame_origin',
                origin,
                self._origin_marker_diameter,
                ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0),
            ),
            self._sphere(
                3,
                frame_id,
                msg.header.stamp,
                'base_link_origin',
                base_link_origin,
                self._base_link_marker_diameter,
                ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0),
            ),
            self._arrow(
                4,
                frame_id,
                msg.header.stamp,
                'base_link_forward_heading',
                [base_link_origin, heading_end],
                ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0),
            ),
            self._label(6, frame_id, child_frame, raw_parent_frame, msg, transform),
        ]
        if velocity_end is not None:
            marker_list.append(
                self._arrow(
                    5,
                    frame_id,
                    msg.header.stamp,
                    'base_link_velocity',
                    [base_link_origin, velocity_end],
                    ColorRGBA(r=1.0, g=0.0, b=0.9, a=1.0),
                )
            )
        if self._trail_enabled and len(self._trail_points) >= 2:
            marker_list.append(
                self._line_strip(
                    7,
                    frame_id,
                    msg.header.stamp,
                    'base_link_trajectory_trail',
                    self._copy_points(self._trail_points),
                    ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
                )
            )

        markers = MarkerArray()
        markers.markers = marker_list
        return markers

    def _make_trace_path(self, frame_id: str, stamp) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        poses = []
        for point in self._trail_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = self._copy_point(point)
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path.poses = poses
        return path

    @staticmethod
    def _delete_all(frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(self, marker_id: int, frame_id: str, stamp, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        self._set_lifetime(marker)
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
        marker.scale.x = self._line_width
        marker.color = colour
        marker.points = points
        return marker

    def _arrow(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.ARROW)
        marker.scale.x = self._heading_arrow_shaft_diameter
        marker.scale.y = self._heading_arrow_head_diameter
        marker.scale.z = self._heading_arrow_head_length
        marker.color = colour
        marker.points = points
        return marker

    def _sphere(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        point: Point,
        diameter: float,
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.SPHERE)
        marker.pose.position = point
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = colour
        return marker

    def _label(
        self,
        marker_id: int,
        frame_id: str,
        child_frame: str,
        raw_parent_frame: str,
        msg: Odometry,
        transform: TransformStamped,
    ) -> Marker:
        position = transform.transform.translation
        orientation = transform.transform.rotation
        linear = msg.twist.twist.linear
        yaw = self._yaw_from_quaternion(orientation)
        heading = self._forward_axis(orientation)
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        marker = self._base_marker(
            marker_id,
            frame_id,
            msg.header.stamp,
            'carmaker_odom_tf_label',
            Marker.TEXT_VIEW_FACING,
        )
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + 0.8
        marker.scale.z = self._text_height
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = (
            f'{frame_id} -> {child_frame}\n'
            f'raw frame: {raw_parent_frame}  mode: {self._alignment_mode}\n'
            f'x={position.x:.2f} y={position.y:.2f} z={position.z:.2f}\n'
            f'yaw={math.degrees(yaw):.1f} deg\n'
            f'base_link +x in {frame_id}: x={heading.x:.2f} y={heading.y:.2f}\n'
            f'body velocity: vx={linear.x:.2f} vy={linear.y:.2f} speed={speed:.2f} m/s'
        )
        return marker

    def _heading_arrow_end(self, base_link_origin: Point, orientation: Quaternion) -> Point:
        forward = self._forward_axis(orientation)
        length = max(0.1, self._heading_arrow_length)
        return Point(
            x=base_link_origin.x + length * forward.x,
            y=base_link_origin.y + length * forward.y,
            z=base_link_origin.z + length * forward.z,
        )

    def _velocity_arrow_end(self, base_link_origin: Point, orientation: Quaternion, msg: Odometry) -> Point | None:
        linear = msg.twist.twist.linear
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        if speed < self._min_velocity_arrow_speed:
            return None

        velocity = self._rotate_child_vector(
            self._quaternion_tuple(orientation),
            linear.x,
            linear.y,
            linear.z,
        )
        scale = max(0.0, self._velocity_arrow_scale)
        return Point(
            x=base_link_origin.x + scale * velocity.x,
            y=base_link_origin.y + scale * velocity.y,
            z=base_link_origin.z + scale * velocity.z,
        )

    def _update_trail(self, frame_id: str, point: Point) -> None:
        if not self._trail_enabled:
            return
        if frame_id != self._trail_frame:
            self._trail_frame = frame_id
            self._trail_points = []

        if self._trail_points and self._distance(self._trail_points[-1], point) < self._trail_min_distance:
            return

        self._trail_points.append(Point(x=point.x, y=point.y, z=point.z))
        if self._trail_max_points <= 0:
            return

        self._trail_max_points = max(2, self._trail_max_points)
        overflow = len(self._trail_points) - self._trail_max_points
        if overflow > 0:
            self._trail_points = self._trail_points[overflow:]

    @staticmethod
    def _copy_points(points: list[Point]) -> list[Point]:
        return [Point(x=point.x, y=point.y, z=point.z) for point in points]

    @staticmethod
    def _copy_point(point: Point) -> Point:
        return Point(x=point.x, y=point.y, z=point.z)

    @staticmethod
    def _distance(left: Point, right: Point) -> float:
        return math.sqrt(
            (left.x - right.x) * (left.x - right.x)
            + (left.y - right.y) * (left.y - right.y)
            + (left.z - right.z) * (left.z - right.z)
        )

    def _set_lifetime(self, marker: Marker) -> None:
        lifetime = max(0.0, self._marker_lifetime_sec)
        marker.lifetime.sec = int(lifetime)
        marker.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)

    @staticmethod
    def _quaternion_tuple(quaternion) -> tuple[float, float, float, float]:
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 1e-9:
            return 0.0, 0.0, 0.0, 1.0
        return quaternion.x / norm, quaternion.y / norm, quaternion.z / norm, quaternion.w / norm

    @staticmethod
    def _quaternion_to_msg(quaternion: tuple[float, float, float, float]) -> Quaternion:
        qx, qy, qz, qw = CarmakerOdomTf._normalize_quaternion_tuple(quaternion)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    @staticmethod
    def _normalize_quaternion_tuple(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = quaternion
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm

    @staticmethod
    def _quaternion_inverse(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = CarmakerOdomTf._normalize_quaternion_tuple(quaternion)
        return -qx, -qy, -qz, qw

    @staticmethod
    def _quaternion_multiply(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return CarmakerOdomTf._normalize_quaternion_tuple(
            (
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            )
        )

    @staticmethod
    def _quaternion_from_yaw_tuple(yaw: float) -> tuple[float, float, float, float]:
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    @staticmethod
    def _quaternion_from_rpy_tuple(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return CarmakerOdomTf._normalize_quaternion_tuple(
            (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        qx, qy, qz, qw = CarmakerOdomTf._quaternion_tuple(quaternion)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _forward_axis(quaternion) -> Point:
        return CarmakerOdomTf._rotate_child_vector(
            CarmakerOdomTf._quaternion_tuple(quaternion),
            1.0,
            0.0,
            0.0,
        )

    @staticmethod
    def _rotate_point(quaternion: tuple[float, float, float, float], point: Point) -> Point:
        return CarmakerOdomTf._rotate_child_vector(quaternion, point.x, point.y, point.z)

    @staticmethod
    def _rotate_child_vector(quaternion: tuple[float, float, float, float], x: float, y: float, z: float) -> Point:
        qx, qy, qz, qw = CarmakerOdomTf._normalize_quaternion_tuple(quaternion)
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


def main(args=None):
    rclpy.init(args=args)
    node = CarmakerOdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
