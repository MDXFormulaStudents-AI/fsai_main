"""
Global path planner from CarMaker ObjectList markers.

This comparison node assumes ``/carmaker/ObjectList`` provides a perfect global
map of cone-like objects. The sampled ObjectList markers do not expose cone
colour, so this node pairs markers by sorted marker ID and builds a global
midpoint path from those opposite-side pairs.
"""

from dataclasses import dataclass
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class ObjectCone:
    marker_id: int
    point: Point


class ObjectListGlobalPlanner(Node):
    """Builds a dense global midpoint path from paired ObjectList cones."""

    def __init__(self):
        super().__init__('object_list_global_planner')

        self.declare_parameter('input_topic', '/carmaker/ObjectList')
        self.declare_parameter('path_topic', '/student/nav/global_path')
        self.declare_parameter('debug_topic', '/student/nav/global_path_markers')
        self.declare_parameter('sample_spacing', 0.25)
        self.declare_parameter('max_marker_scale', 1.0)
        self.declare_parameter('max_pair_distance', 5.0)
        self.declare_parameter('close_path', False)
        self.declare_parameter('smoothing_enabled', False)
        self.declare_parameter('smoothing_iterations', 1)
        self.declare_parameter('smoothing_cut_ratio', 0.25)
        self.declare_parameter('line_width', 0.08)
        self.declare_parameter('point_diameter', 0.18)

        input_topic = self.get_parameter('input_topic').value
        self._path_topic = self.get_parameter('path_topic').value
        self._debug_topic = self.get_parameter('debug_topic').value
        self._sample_spacing = float(self.get_parameter('sample_spacing').value)
        self._max_marker_scale = float(self.get_parameter('max_marker_scale').value)
        self._max_pair_distance = float(self.get_parameter('max_pair_distance').value)
        self._close_path = bool(self.get_parameter('close_path').value)
        self._smoothing_enabled = bool(self.get_parameter('smoothing_enabled').value)
        self._smoothing_iterations = max(0, int(self.get_parameter('smoothing_iterations').value))
        self._smoothing_cut_ratio = float(self.get_parameter('smoothing_cut_ratio').value)
        self._line_width = float(self.get_parameter('line_width').value)
        self._point_diameter = float(self.get_parameter('point_diameter').value)

        self._sub = self.create_subscription(
            MarkerArray,
            input_topic,
            self._on_object_list,
            10,
        )
        self._path_pub = self.create_publisher(Path, self._path_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, self._debug_topic, 10)

        self.get_logger().info(
            f'ObjectList global planner listening on {input_topic}; '
            f'publishing path={self._path_topic}, debug={self._debug_topic}'
        )

    def _on_object_list(self, msg: MarkerArray) -> None:
        objects, frame_id, stamp = self._extract_objects(msg)
        pairs = self._pair_objects_by_id(objects)
        midpoints = [self._midpoint(left, right) for left, right in pairs]
        smoothed_midpoints = self._smooth_path(midpoints, self._close_path)
        path_points = self._sample_polyline(smoothed_midpoints, self._close_path)

        self._path_pub.publish(self._make_path_msg(path_points, frame_id, stamp))
        self._debug_pub.publish(
            self._make_debug_markers(
                frame_id,
                stamp,
                pairs,
                midpoints,
                smoothed_midpoints,
                path_points,
                len(objects),
            )
        )

    def _extract_objects(self, msg: MarkerArray) -> tuple[list[ObjectCone], str, object]:
        objects = []
        frame_id = 'Obj_F'
        stamp = self.get_clock().now().to_msg()

        for marker in msg.markers:
            if marker.header.frame_id:
                frame_id = marker.header.frame_id
                stamp = marker.header.stamp

            if marker.action != Marker.ADD:
                continue
            if marker.type != Marker.CUBE:
                continue
            if max(marker.scale.x, marker.scale.y, marker.scale.z) > self._max_marker_scale:
                continue

            position = marker.pose.position
            objects.append(
                ObjectCone(
                    marker_id=marker.id,
                    point=Point(x=position.x, y=position.y, z=position.z),
                )
            )

        objects.sort(key=lambda obj: obj.marker_id)
        return objects, frame_id, stamp

    def _pair_objects_by_id(self, objects: list[ObjectCone]) -> list[tuple[ObjectCone, ObjectCone]]:
        pairs = []
        for idx in range(0, len(objects) - 1, 2):
            left = objects[idx]
            right = objects[idx + 1]
            if self._distance_xy(left.point, right.point) > self._max_pair_distance:
                continue
            pairs.append((left, right))
        return pairs

    @staticmethod
    def _midpoint(left: ObjectCone, right: ObjectCone) -> Point:
        return Point(
            x=(left.point.x + right.point.x) / 2.0,
            y=(left.point.y + right.point.y) / 2.0,
            z=(left.point.z + right.point.z) / 2.0,
        )

    def _smooth_path(self, points: list[Point], close_path: bool) -> list[Point]:
        if not self._smoothing_enabled or self._smoothing_iterations < 1 or len(points) < 3:
            return points

        cut_ratio = min(max(self._smoothing_cut_ratio, 0.0), 0.45)
        if cut_ratio < 1e-6:
            return points

        smoothed = points
        for _ in range(self._smoothing_iterations):
            if close_path:
                next_points = []
                for idx, start in enumerate(smoothed):
                    end = smoothed[(idx + 1) % len(smoothed)]
                    next_points.append(self._interpolate(start, end, cut_ratio))
                    next_points.append(self._interpolate(start, end, 1.0 - cut_ratio))
            else:
                next_points = [smoothed[0]]
                for start, end in zip(smoothed[:-1], smoothed[1:]):
                    next_points.append(self._interpolate(start, end, cut_ratio))
                    next_points.append(self._interpolate(start, end, 1.0 - cut_ratio))
                next_points.append(smoothed[-1])
            smoothed = next_points

        return smoothed

    @staticmethod
    def _interpolate(start: Point, end: Point, ratio: float) -> Point:
        return Point(
            x=start.x + ratio * (end.x - start.x),
            y=start.y + ratio * (end.y - start.y),
            z=start.z + ratio * (end.z - start.z),
        )

    def _sample_polyline(self, points: list[Point], close_path: bool) -> list[Point]:
        if len(points) < 2:
            return points

        samples = []
        spacing = max(self._sample_spacing, 0.05)
        segments = list(zip(points[:-1], points[1:]))
        if close_path:
            segments.append((points[-1], points[0]))

        for start, end in segments:
            dx = end.x - start.x
            dy = end.y - start.y
            dz = end.z - start.z
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if length < 1e-6:
                continue

            steps = max(1, math.ceil(length / spacing))
            for step in range(steps):
                ratio = step / steps
                samples.append(
                    Point(
                        x=start.x + ratio * dx,
                        y=start.y + ratio * dy,
                        z=start.z + ratio * dz,
                    )
                )

        if not close_path:
            samples.append(points[-1])

        return samples

    def _make_path_msg(self, points: list[Point], frame_id: str, stamp) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _make_debug_markers(
        self,
        frame_id: str,
        stamp,
        pairs: list[tuple[ObjectCone, ObjectCone]],
        midpoints: list[Point],
        smoothed_midpoints: list[Point],
        path_points: list[Point],
        object_count: int,
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self._delete_all(frame_id, stamp))
        markers.markers.append(
            self._strategy_text(
                frame_id,
                stamp,
                object_count,
                len(pairs),
                len(midpoints),
                len(smoothed_midpoints),
                len(path_points),
            )
        )

        side_a = [left.point for left, _ in pairs]
        side_b = [right.point for _, right in pairs]
        if side_a:
            markers.markers.append(self._point_list(1, frame_id, stamp, 'object_side_a', side_a, self._blue(), self._point_diameter))
            markers.markers.append(self._line_strip(2, frame_id, stamp, 'object_side_a_line', side_a, self._blue(), self._line_width))
        if side_b:
            markers.markers.append(self._point_list(3, frame_id, stamp, 'object_side_b', side_b, self._yellow(), self._point_diameter))
            markers.markers.append(self._line_strip(4, frame_id, stamp, 'object_side_b_line', side_b, self._yellow(), self._line_width))
        if pairs:
            markers.markers.append(self._pair_lines(5, frame_id, stamp, pairs))
        if midpoints:
            markers.markers.append(self._point_list(6, frame_id, stamp, 'object_midpoints', midpoints, self._green(), self._point_diameter))
        if path_points:
            markers.markers.append(self._point_list(7, frame_id, stamp, 'object_global_path_points', path_points, self._green(), 0.1))
            markers.markers.append(self._line_strip(8, frame_id, stamp, 'object_global_path_line', path_points, self._green(), self._line_width))

        return markers

    @staticmethod
    def _delete_all(frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _strategy_text(
        self,
        frame_id: str,
        stamp,
        object_count: int,
        pair_count: int,
        midpoint_count: int,
        smoothed_midpoint_count: int,
        path_point_count: int,
    ) -> Marker:
        marker = self._base_marker(0, frame_id, stamp, 'object_global_summary', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.scale.z = 0.5
        marker.color = self._white()
        smoothing = 'off'
        if self._smoothing_enabled and self._smoothing_iterations > 0:
            smoothing = f'{self._smoothing_iterations}x @ {self._smoothing_cut_ratio:.2f}'
        marker.text = (
            'ObjectList global midpoint path\n'
            f'objects: {object_count}  pairs: {pair_count}\n'
            f'midpoints: {midpoint_count}  smoothed: {smoothed_midpoint_count}\n'
            f'path points: {path_point_count}  smoothing: {smoothing}'
        )
        return marker

    def _line_strip(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
        width: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color = colour
        marker.points = points + [points[0]] if self._close_path and len(points) > 2 else points
        return marker

    def _point_list(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
        diameter: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, namespace, Marker.SPHERE_LIST)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = colour
        marker.points = points
        return marker

    def _pair_lines(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        pairs: list[tuple[ObjectCone, ObjectCone]],
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, stamp, 'object_pair_lines', Marker.LINE_LIST)
        marker.scale.x = 0.035
        marker.color = self._grey()
        for left, right in pairs:
            marker.points.append(left.point)
            marker.points.append(right.point)
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
    def _distance_xy(left: Point, right: Point) -> float:
        return math.hypot(left.x - right.x, left.y - right.y)

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
    def _grey() -> ColorRGBA:
        return ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.45)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectListGlobalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
