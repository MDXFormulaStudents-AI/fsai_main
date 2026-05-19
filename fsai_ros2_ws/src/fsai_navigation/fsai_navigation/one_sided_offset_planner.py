"""
One-sided boundary offset planner.

This visualization-only node fits a local polyline through one selected cone
colour, samples that polyline, computes a local normal at each sample, and
offsets those samples to estimate a centreline.
"""

from dataclasses import dataclass
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class ConeObservation:
    colour: str
    x: float
    y: float
    z: float


@dataclass
class TrackedCone:
    track_id: int
    colour: str
    x: float
    y: float
    z: float
    hit_count: int
    miss_count: int
    last_seen_sec: float

    @property
    def point(self) -> Point:
        return Point(x=self.x, y=self.y, z=self.z)


class OneSidedOffsetPlanner(Node):
    """Creates an offset centreline from a single cone boundary."""

    def __init__(self):
        super().__init__('one_sided_offset_planner')

        self.declare_parameter('input_topic', '/cone_markers')
        self.declare_parameter('debug_topic', '/student/nav/one_sided_offset_markers')
        self.declare_parameter('selected_colour', 'blue')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 35.0)
        self.declare_parameter('max_abs_y', 8.0)
        self.declare_parameter('match_radius', 1.25)
        self.declare_parameter('confirm_hits', 2)
        self.declare_parameter('miss_limit', 4)
        self.declare_parameter('max_track_age_sec', 0.5)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('min_cones_for_path', 3)
        self.declare_parameter('sample_spacing', 0.5)
        self.declare_parameter('offset_distance', 1.5)
        self.declare_parameter('offset_sign', -1.0)
        self.declare_parameter('boundary_line_width', 0.05)
        self.declare_parameter('centerline_width', 0.08)
        self.declare_parameter('boundary_point_diameter', 0.18)
        self.declare_parameter('centerline_point_diameter', 0.14)
        self.declare_parameter('tentative_diameter', 0.12)

        input_topic = self.get_parameter('input_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._selected_colour = str(self.get_parameter('selected_colour').value).lower()
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._match_radius = float(self.get_parameter('match_radius').value)
        self._confirm_hits = int(self.get_parameter('confirm_hits').value)
        self._miss_limit = int(self.get_parameter('miss_limit').value)
        self._max_track_age_sec = float(self.get_parameter('max_track_age_sec').value)
        self._smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self._min_cones_for_path = int(self.get_parameter('min_cones_for_path').value)
        self._sample_spacing = float(self.get_parameter('sample_spacing').value)
        self._offset_distance = float(self.get_parameter('offset_distance').value)
        self._offset_sign = float(self.get_parameter('offset_sign').value)
        self._boundary_line_width = float(self.get_parameter('boundary_line_width').value)
        self._centerline_width = float(self.get_parameter('centerline_width').value)
        self._boundary_point_diameter = float(self.get_parameter('boundary_point_diameter').value)
        self._centerline_point_diameter = float(self.get_parameter('centerline_point_diameter').value)
        self._tentative_diameter = float(self.get_parameter('tentative_diameter').value)

        self._tracks: list[TrackedCone] = []
        self._next_track_id = 0
        self._last_frame_id = 'Fr1A'
        self._last_stamp = self.get_clock().now().to_msg()
        self._last_marker_stamp_sec: float | None = None

        self._sub = self.create_subscription(
            MarkerArray,
            input_topic,
            self._on_markers,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_debug)

        self.get_logger().info(
            f'One-sided offset planner listening on {input_topic}; '
            f'using {self._selected_colour} cones; publishing {debug_topic}'
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        observations, frame_id, stamp, marker_stamp_sec = self._extract_observations(msg)
        now_sec = self._now_sec()
        self._last_frame_id = frame_id
        self._last_stamp = stamp

        if self._last_marker_stamp_sec is not None and marker_stamp_sec < self._last_marker_stamp_sec - 0.5:
            self._tracks.clear()
            self.get_logger().debug('Marker timestamp moved backwards; cleared local cone tracks')
        self._last_marker_stamp_sec = marker_stamp_sec

        self._match_and_update_tracks(observations, now_sec)
        self._prune_tracks(now_sec)

    def _extract_observations(
        self,
        msg: MarkerArray,
    ) -> tuple[list[ConeObservation], str, object, float]:
        observations = []
        frame_id = self._last_frame_id
        stamp = self._last_stamp

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
            if colour != self._selected_colour:
                continue

            position = marker.pose.position
            if not self._is_in_local_window(position):
                continue

            observations.append(
                ConeObservation(colour, position.x, position.y, position.z)
            )

        return observations, frame_id, stamp, self._stamp_to_sec(stamp)

    def _match_and_update_tracks(
        self,
        observations: list[ConeObservation],
        now_sec: float,
    ) -> None:
        candidate_matches = []
        for obs_idx, obs in enumerate(observations):
            for track_idx, track in enumerate(self._tracks):
                if obs.colour != track.colour:
                    continue
                distance = math.hypot(obs.x - track.x, obs.y - track.y)
                if distance <= self._match_radius:
                    candidate_matches.append((distance, obs_idx, track_idx))

        candidate_matches.sort(key=lambda item: item[0])
        matched_observations = set()
        matched_tracks = set()

        for _, obs_idx, track_idx in candidate_matches:
            if obs_idx in matched_observations or track_idx in matched_tracks:
                continue
            self._update_track(self._tracks[track_idx], observations[obs_idx], now_sec)
            matched_observations.add(obs_idx)
            matched_tracks.add(track_idx)

        for track_idx, track in enumerate(self._tracks):
            if track_idx not in matched_tracks:
                track.miss_count += 1

        for obs_idx, obs in enumerate(observations):
            if obs_idx not in matched_observations:
                self._tracks.append(
                    TrackedCone(
                        track_id=self._next_track_id,
                        colour=obs.colour,
                        x=obs.x,
                        y=obs.y,
                        z=obs.z,
                        hit_count=1,
                        miss_count=0,
                        last_seen_sec=now_sec,
                    )
                )
                self._next_track_id += 1

    def _update_track(self, track: TrackedCone, obs: ConeObservation, now_sec: float) -> None:
        alpha = self._smoothing_alpha
        track.x = (1.0 - alpha) * track.x + alpha * obs.x
        track.y = (1.0 - alpha) * track.y + alpha * obs.y
        track.z = (1.0 - alpha) * track.z + alpha * obs.z
        track.hit_count += 1
        track.miss_count = 0
        track.last_seen_sec = now_sec

    def _prune_tracks(self, now_sec: float) -> None:
        self._tracks = [
            track for track in self._tracks
            if track.miss_count <= self._miss_limit
            and now_sec - track.last_seen_sec <= self._max_track_age_sec
            and self._is_in_local_window(track.point)
        ]

    def _publish_debug(self) -> None:
        now_sec = self._now_sec()
        self._prune_tracks(now_sec)

        confirmed = [
            track for track in self._tracks
            if track.hit_count >= self._confirm_hits
        ]
        confirmed.sort(key=lambda track: track.x)
        tentative = [
            track for track in self._tracks
            if track.hit_count < self._confirm_hits
        ]

        boundary_points = [track.point for track in confirmed]
        sampled_boundary, centreline = self._offset_centreline(boundary_points)

        markers = MarkerArray()
        markers.markers.append(self._delete_all())

        if boundary_points:
            markers.markers.append(
                self._point_list(
                    0,
                    'one_sided_boundary_cones',
                    boundary_points,
                    self._selected_colour_rgba(),
                    self._boundary_point_diameter,
                )
            )

        if len(boundary_points) >= 2:
            markers.markers.append(
                self._line_strip(
                    1,
                    'one_sided_boundary_polyline',
                    boundary_points,
                    self._selected_colour_rgba(),
                    self._boundary_line_width,
                )
            )

        if sampled_boundary:
            markers.markers.append(
                self._point_list(
                    2,
                    'sampled_boundary_points',
                    sampled_boundary,
                    self._selected_colour_rgba(alpha=0.55),
                    0.08,
                )
            )

        if centreline:
            markers.markers.append(
                self._point_list(
                    3,
                    'one_sided_offset_points',
                    centreline,
                    self._green(),
                    self._centerline_point_diameter,
                )
            )
            if len(centreline) >= 2:
                markers.markers.append(
                    self._line_strip(
                        4,
                        'one_sided_offset_centerline',
                        centreline,
                        self._green(),
                        self._centerline_width,
                    )
                )

        if tentative:
            markers.markers.append(
                self._track_spheres(5, 'tentative_tracks', tentative, self._grey())
            )

        held_tracks = [track for track in confirmed if track.miss_count > 0]
        if held_tracks:
            markers.markers.append(
                self._track_spheres(6, 'held_tracks', held_tracks, self._orange())
            )

        self._pub.publish(markers)

    def _offset_centreline(self, boundary: list[Point]) -> tuple[list[Point], list[Point]]:
        if len(boundary) < self._min_cones_for_path:
            return [], []

        samples = self._sample_polyline(boundary)
        centreline = []
        for sample, tangent_x, tangent_y in samples:
            normal_x = -tangent_y
            normal_y = tangent_x
            centreline.append(
                Point(
                    x=sample.x + self._offset_sign * self._offset_distance * normal_x,
                    y=sample.y + self._offset_sign * self._offset_distance * normal_y,
                    z=sample.z,
                )
            )

        return [sample for sample, _, _ in samples], centreline

    def _sample_polyline(self, points: list[Point]) -> list[tuple[Point, float, float]]:
        samples = []
        spacing = max(self._sample_spacing, 0.05)

        for start, end in zip(points[:-1], points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            dz = end.z - start.z
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if length < 1e-6:
                continue

            tangent_x = dx / length
            tangent_y = dy / length
            steps = max(1, math.ceil(length / spacing))
            for step in range(steps):
                ratio = step / steps
                samples.append((
                    Point(
                        x=start.x + ratio * dx,
                        y=start.y + ratio * dy,
                        z=start.z + ratio * dz,
                    ),
                    tangent_x,
                    tangent_y,
                ))

        if len(points) >= 2:
            start = points[-2]
            end = points[-1]
            dx = end.x - start.x
            dy = end.y - start.y
            length = math.hypot(dx, dy)
            if length >= 1e-6:
                samples.append((end, dx / length, dy / length))

        return samples

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

    def _delete_all(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_frame_id
        marker.header.stamp = self._last_stamp
        marker.action = Marker.DELETEALL
        return marker

    def _line_strip(
        self,
        marker_id: int,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
        width: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color = colour
        marker.points = points
        return marker

    def _point_list(
        self,
        marker_id: int,
        namespace: str,
        points: list[Point],
        colour: ColorRGBA,
        diameter: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE_LIST)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = colour
        marker.points = points
        return marker

    def _track_spheres(
        self,
        marker_id: int,
        namespace: str,
        tracks: list[TrackedCone],
        colour: ColorRGBA,
    ) -> Marker:
        return self._point_list(
            marker_id,
            namespace,
            [track.point for track in tracks],
            colour,
            self._tentative_diameter,
        )

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_frame_id
        marker.header.stamp = self._last_stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000
        return marker

    def _selected_colour_rgba(self, alpha: float = 1.0) -> ColorRGBA:
        if self._selected_colour == 'yellow':
            return ColorRGBA(r=1.0, g=0.9, b=0.0, a=alpha)
        return ColorRGBA(r=0.1, g=0.35, b=1.0, a=alpha)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

    @staticmethod
    def _grey() -> ColorRGBA:
        return ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.35)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.7)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OneSidedOffsetPlanner()
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
