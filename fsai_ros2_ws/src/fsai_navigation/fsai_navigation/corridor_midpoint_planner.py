"""
Marker-based corridor midpoint planner.

This node is visualization-only. It reads coloured cone mesh markers from
``/cone_markers``, builds simple blue/yellow boundary lines, pairs cones by
similar forward position, and publishes a connected midpoint centreline.
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


class CorridorMidpointPlanner(Node):
    """Builds local corridor boundaries and a midpoint centreline."""

    def __init__(self):
        super().__init__('corridor_midpoint_planner')

        self.declare_parameter('input_topic', '/cone_markers')
        self.declare_parameter('debug_topic', '/student/nav/corridor_markers')
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 25.0)
        self.declare_parameter('max_abs_y', 8.0)
        self.declare_parameter('max_pair_x_gap', 3.0)
        self.declare_parameter('boundary_line_width', 0.05)
        self.declare_parameter('centerline_width', 0.08)
        self.declare_parameter('pair_line_width', 0.025)
        self.declare_parameter('midpoint_diameter', 0.2)

        input_topic = self.get_parameter('input_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._max_pair_x_gap = float(self.get_parameter('max_pair_x_gap').value)
        self._boundary_line_width = float(self.get_parameter('boundary_line_width').value)
        self._centerline_width = float(self.get_parameter('centerline_width').value)
        self._pair_line_width = float(self.get_parameter('pair_line_width').value)
        self._midpoint_diameter = float(self.get_parameter('midpoint_diameter').value)

        self._sub = self.create_subscription(
            MarkerArray,
            input_topic,
            self._on_markers,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, debug_topic, 10)

        self.get_logger().info(
            f'Corridor midpoint planner listening on {input_topic}; '
            f'publishing debug markers on {debug_topic}'
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        cones, frame_id, stamp = self._extract_cones(msg)
        blue = self._cones_by_colour(cones, 'blue')
        yellow = self._cones_by_colour(cones, 'yellow')
        pairs = self._pair_by_forward_position(blue, yellow)
        midpoints = [self._midpoint(left, right) for left, right in pairs]

        debug = MarkerArray()
        debug.markers.append(self._delete_all(frame_id, stamp))

        if blue:
            debug.markers.append(
                self._line_strip(
                    0,
                    frame_id,
                    stamp,
                    'blue_boundary',
                    [self._point_from_cone(cone) for cone in blue],
                    self._blue(),
                    self._boundary_line_width,
                )
            )

        if yellow:
            debug.markers.append(
                self._line_strip(
                    1,
                    frame_id,
                    stamp,
                    'yellow_boundary',
                    [self._point_from_cone(cone) for cone in yellow],
                    self._yellow(),
                    self._boundary_line_width,
                )
            )

        if pairs:
            debug.markers.append(self._pair_lines(2, frame_id, stamp, pairs))
            debug.markers.append(self._midpoint_spheres(3, frame_id, stamp, midpoints))

        if len(midpoints) >= 2:
            debug.markers.append(
                self._line_strip(
                    4,
                    frame_id,
                    stamp,
                    'centerline',
                    midpoints,
                    self._green(),
                    self._centerline_width,
                )
            )

        self._pub.publish(debug)
        self.get_logger().debug(
            f'blue={len(blue)} yellow={len(yellow)} pairs={len(pairs)} '
            f'midpoints={len(midpoints)}'
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

            position = marker.pose.position
            if not self._is_in_local_window(position):
                continue

            cones.append(ConeMarker(colour, position.x, position.y, position.z))

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

    @staticmethod
    def _cones_by_colour(cones: list[ConeMarker], colour: str) -> list[ConeMarker]:
        filtered = [cone for cone in cones if cone.colour == colour]
        filtered.sort(key=lambda cone: cone.x)
        return filtered

    def _pair_by_forward_position(
        self,
        blue: list[ConeMarker],
        yellow: list[ConeMarker],
    ) -> list[tuple[ConeMarker, ConeMarker]]:
        pairs = []
        used_yellow = set()

        for blue_cone in blue:
            candidates = [
                (abs(blue_cone.x - yellow_cone.x), idx, yellow_cone)
                for idx, yellow_cone in enumerate(yellow)
                if idx not in used_yellow
            ]
            if not candidates:
                break

            gap, yellow_idx, yellow_cone = min(candidates, key=lambda item: item[0])
            if gap > self._max_pair_x_gap:
                continue

            pairs.append((blue_cone, yellow_cone))
            used_yellow.add(yellow_idx)

        pairs.sort(key=lambda pair: (pair[0].x + pair[1].x) / 2.0)
        return pairs

    @staticmethod
    def _midpoint(left: ConeMarker, right: ConeMarker) -> Point:
        return Point(
            x=(left.x + right.x) / 2.0,
            y=(left.y + right.y) / 2.0,
            z=(left.z + right.z) / 2.0,
        )

    @staticmethod
    def _point_from_cone(cone: ConeMarker) -> Point:
        return Point(x=cone.x, y=cone.y, z=cone.z)

    @staticmethod
    def _delete_all(frame_id: str, stamp) -> Marker:
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
        width: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color = colour
        marker.points = points
        marker.lifetime.sec = 1
        return marker

    def _pair_lines(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        pairs: list[tuple[ConeMarker, ConeMarker]],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'pair_lines'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._pair_line_width
        marker.color = self._grey()
        for left, right in pairs:
            marker.points.append(self._point_from_cone(left))
            marker.points.append(self._point_from_cone(right))
        marker.lifetime.sec = 1
        return marker

    def _midpoint_spheres(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        midpoints: list[Point],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'midpoints'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._midpoint_diameter
        marker.scale.y = self._midpoint_diameter
        marker.scale.z = self._midpoint_diameter
        marker.color = self._green()
        marker.points = midpoints
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
    def _grey() -> ColorRGBA:
        return ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.45)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorMidpointPlanner()
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
