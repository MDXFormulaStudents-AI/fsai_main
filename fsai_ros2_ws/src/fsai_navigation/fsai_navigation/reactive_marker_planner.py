"""
Marker-based reactive cone planner.

This node is intentionally visualization-only. It reads cone positions from
``/cone_markers`` while the bag data topics are unavailable, computes Strategy B
nearest-cone averages, and publishes debug markers for RViz.
"""

from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class ConeMarker:
    colour: str
    x: float
    y: float
    z: float


class ReactiveMarkerPlanner(Node):
    """Computes a local target from averaged blue/yellow cone markers."""

    def __init__(self):
        super().__init__('reactive_marker_planner')

        self.declare_parameter('input_topic', '/cone_markers')
        self.declare_parameter('debug_topic', '/student/nav/debug_markers')
        self.declare_parameter('cones_per_side', 3)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 20.0)
        self.declare_parameter('max_abs_y', 8.0)

        input_topic = self.get_parameter('input_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self._cones_per_side = self.get_parameter('cones_per_side').value
        self._min_x = self.get_parameter('min_x').value
        self._max_x = self.get_parameter('max_x').value
        self._max_abs_y = self.get_parameter('max_abs_y').value

        self._sub = self.create_subscription(
            MarkerArray,
            input_topic,
            self._on_markers,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, debug_topic, 10)

        self.get_logger().info(
            f'Reactive marker planner listening on {input_topic}; '
            f'publishing debug markers on {debug_topic}'
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        cones, frame_id, stamp = self._extract_cones(msg)
        blue = self._nearest_cones(cones, 'blue')
        yellow = self._nearest_cones(cones, 'yellow')

        debug = MarkerArray()
        debug.markers.append(self._delete_all(frame_id, stamp))

        if not blue or not yellow:
            self._pub.publish(debug)
            self.get_logger().debug(
                f'Need both blue and yellow cones; got blue={len(blue)} yellow={len(yellow)}'
            )
            return

        blue_avg = self._average_point(blue)
        yellow_avg = self._average_point(yellow)
        target = Point(
            x=(blue_avg.x + yellow_avg.x) / 2.0,
            y=(blue_avg.y + yellow_avg.y) / 2.0,
            z=(blue_avg.z + yellow_avg.z) / 2.0,
        )

        debug.markers.extend([
            self._sphere_marker(0, frame_id, stamp, 'blue_average', blue_avg, self._blue(), 0.225),
            self._sphere_marker(1, frame_id, stamp, 'yellow_average', yellow_avg, self._yellow(), 0.225),
            self._sphere_marker(2, frame_id, stamp, 'target', target, self._green(), 0.3),
            self._line_marker(3, frame_id, stamp, target),
        ])

        self._pub.publish(debug)
        self.get_logger().debug(
            f'target=({target.x:.2f}, {target.y:.2f}, {target.z:.2f}) '
            f'from blue={len(blue)} yellow={len(yellow)}'
        )

    def _extract_cones(self, msg: MarkerArray) -> tuple[list[ConeMarker], str, object]:
        cones = []
        frame_id = 'Fr1A'
        stamp = self.get_clock().now().to_msg()

        for marker in msg.markers:
            if marker.header.frame_id:
                frame_id = marker.header.frame_id
                stamp = marker.header.stamp

            if marker.action != Marker.ADD:
                continue
            if marker.ns != 'cones':
                continue
            if marker.type != Marker.MESH_RESOURCE:
                continue

            colour = self._colour_from_mesh(marker.mesh_resource)
            if colour not in ('blue', 'yellow'):
                continue

            pos = marker.pose.position
            if not self._is_in_local_window(pos):
                continue

            cones.append(ConeMarker(colour, pos.x, pos.y, pos.z))

        return cones, frame_id, stamp

    @staticmethod
    def _colour_from_mesh(mesh_resource: str) -> str | None:
        mesh = mesh_resource.lower()
        if 'blue' in mesh:
            return 'blue'
        if 'yellow' in mesh:
            return 'yellow'
        if 'orange' in mesh:
            return 'orange'
        return None

    def _is_in_local_window(self, point: Point) -> bool:
        return (
            self._min_x <= point.x <= self._max_x
            and abs(point.y) <= self._max_abs_y
        )

    def _nearest_cones(self, cones: list[ConeMarker], colour: str) -> list[ConeMarker]:
        same_colour = [cone for cone in cones if cone.colour == colour]
        same_colour.sort(key=lambda cone: cone.x)
        return same_colour[:self._cones_per_side]

    @staticmethod
    def _average_point(cones: list[ConeMarker]) -> Point:
        n = float(len(cones))
        return Point(
            x=sum(cone.x for cone in cones) / n,
            y=sum(cone.y for cone in cones) / n,
            z=sum(cone.z for cone in cones) / n,
        )

    @staticmethod
    def _delete_all(frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _sphere_marker(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        namespace: str,
        point: Point,
        colour: ColorRGBA,
        diameter: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = colour
        marker.lifetime.sec = 1
        return marker

    def _line_marker(self, marker_id: int, frame_id: str, stamp, target: Point) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'car_to_target'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color = self._white()
        marker.points = [Point(x=0.0, y=0.0, z=0.0), target]
        marker.lifetime.sec = 1
        return marker

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
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveMarkerPlanner()
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
