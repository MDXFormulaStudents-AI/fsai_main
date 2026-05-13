"""
Short-lived persistent corridor midpoint planner.

This node keeps a local-frame memory of cone marker observations so brief
perception dropouts do not immediately erase the corridor centreline. New cones
must be seen repeatedly before they are allowed to affect the path.
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


class PersistentCorridorPlanner(Node):
    """Builds a centreline from confirmed short-lived cone tracks."""

    def __init__(self):
        super().__init__('persistent_corridor_planner')

        self.declare_parameter('input_topic', '/cone_markers')
        self.declare_parameter('debug_topic', '/student/nav/persistent_corridor_markers')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 25.0)
        self.declare_parameter('max_abs_y', 8.0)
        self.declare_parameter('match_radius', 0.75)
        self.declare_parameter('confirm_hits', 2)
        self.declare_parameter('miss_limit', 8)
        self.declare_parameter('max_track_age_sec', 0.8)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('max_pair_x_gap', 3.0)
        self.declare_parameter('boundary_line_width', 0.05)
        self.declare_parameter('centerline_width', 0.08)
        self.declare_parameter('pair_line_width', 0.025)
        self.declare_parameter('midpoint_diameter', 0.2)
        self.declare_parameter('tentative_diameter', 0.14)

        input_topic = self.get_parameter('input_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._match_radius = float(self.get_parameter('match_radius').value)
        self._confirm_hits = int(self.get_parameter('confirm_hits').value)
        self._miss_limit = int(self.get_parameter('miss_limit').value)
        self._max_track_age_sec = float(self.get_parameter('max_track_age_sec').value)
        self._smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self._max_pair_x_gap = float(self.get_parameter('max_pair_x_gap').value)
        self._boundary_line_width = float(self.get_parameter('boundary_line_width').value)
        self._centerline_width = float(self.get_parameter('centerline_width').value)
        self._pair_line_width = float(self.get_parameter('pair_line_width').value)
        self._midpoint_diameter = float(self.get_parameter('midpoint_diameter').value)
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
            f'Persistent corridor planner listening on {input_topic}; '
            f'publishing debug markers on {debug_topic}'
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
            if colour not in ('blue', 'yellow'):
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
        tentative = [
            track for track in self._tracks
            if track.hit_count < self._confirm_hits
        ]

        blue = self._tracks_by_colour(confirmed, 'blue')
        yellow = self._tracks_by_colour(confirmed, 'yellow')
        pairs = self._pair_by_forward_position(blue, yellow)
        midpoints = [self._midpoint(left, right) for left, right in pairs]

        markers = MarkerArray()
        markers.markers.append(self._delete_all())

        if blue:
            markers.markers.append(
                self._line_strip(
                    0,
                    'persistent_blue_boundary',
                    [track.point for track in blue],
                    self._blue(),
                    self._boundary_line_width,
                )
            )
        if yellow:
            markers.markers.append(
                self._line_strip(
                    1,
                    'persistent_yellow_boundary',
                    [track.point for track in yellow],
                    self._yellow(),
                    self._boundary_line_width,
                )
            )
        if pairs:
            markers.markers.append(self._pair_lines(2, pairs))
            markers.markers.append(self._midpoint_spheres(3, midpoints))
        if len(midpoints) >= 2:
            markers.markers.append(
                self._line_strip(
                    4,
                    'persistent_centerline',
                    midpoints,
                    self._green(),
                    self._centerline_width,
                )
            )
        if tentative:
            markers.markers.append(self._track_spheres(5, 'tentative_tracks', tentative, self._grey()))

        held_tracks = [track for track in confirmed if track.miss_count > 0]
        if held_tracks:
            markers.markers.append(self._track_spheres(6, 'held_tracks', held_tracks, self._orange()))

        self._pub.publish(markers)

    def _pair_by_forward_position(
        self,
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
    ) -> list[tuple[TrackedCone, TrackedCone]]:
        pairs = []
        used_yellow = set()

        for blue_track in blue:
            candidates = [
                (abs(blue_track.x - yellow_track.x), idx, yellow_track)
                for idx, yellow_track in enumerate(yellow)
                if idx not in used_yellow
            ]
            if not candidates:
                break

            gap, yellow_idx, yellow_track = min(candidates, key=lambda item: item[0])
            if gap > self._max_pair_x_gap:
                continue

            pairs.append((blue_track, yellow_track))
            used_yellow.add(yellow_idx)

        pairs.sort(key=lambda pair: (pair[0].x + pair[1].x) / 2.0)
        return pairs

    @staticmethod
    def _tracks_by_colour(tracks: list[TrackedCone], colour: str) -> list[TrackedCone]:
        filtered = [track for track in tracks if track.colour == colour]
        filtered.sort(key=lambda track: track.x)
        return filtered

    @staticmethod
    def _midpoint(left: TrackedCone, right: TrackedCone) -> Point:
        return Point(
            x=(left.x + right.x) / 2.0,
            y=(left.y + right.y) / 2.0,
            z=(left.z + right.z) / 2.0,
        )

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

    def _pair_lines(self, marker_id: int, pairs: list[tuple[TrackedCone, TrackedCone]]) -> Marker:
        marker = self._base_marker(marker_id, 'persistent_pair_lines', Marker.LINE_LIST)
        marker.scale.x = self._pair_line_width
        marker.color = self._grey()
        for left, right in pairs:
            marker.points.append(left.point)
            marker.points.append(right.point)
        return marker

    def _midpoint_spheres(self, marker_id: int, midpoints: list[Point]) -> Marker:
        marker = self._base_marker(marker_id, 'persistent_midpoints', Marker.SPHERE_LIST)
        marker.scale.x = self._midpoint_diameter
        marker.scale.y = self._midpoint_diameter
        marker.scale.z = self._midpoint_diameter
        marker.color = self._green()
        marker.points = midpoints
        return marker

    def _track_spheres(
        self,
        marker_id: int,
        namespace: str,
        tracks: list[TrackedCone],
        colour: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE_LIST)
        marker.scale.x = self._tentative_diameter
        marker.scale.y = self._tentative_diameter
        marker.scale.z = self._tentative_diameter
        marker.color = colour
        marker.points = [track.point for track in tracks]
        return marker

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
    node = PersistentCorridorPlanner()
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
