"""
Perceived local path planner from fused cone detections.

This visualization/planning node mirrors a practical Formula Student local
navigation stack: it keeps short-lived cone tracks, prefers a two-sided corridor
midline, falls back to one-sided boundary offset, and briefly holds the previous
valid path during perception dropouts.
"""

from dataclasses import dataclass
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial import Delaunay, QhullError

from fsai_interfaces.msg import Cone3DArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

# Typical scales used to normalize the Delaunay path-cost terms before squaring
# (see PerceivedPath._path_cost). Kept as constants; the per-term weights are the
# tunable knobs (delaunay_w_* parameters).
_NORM_ANGLE_RAD = math.radians(45.0)  # a 45 deg kink is a "unit" of angle cost
_NORM_WIDTH_M = 0.5                    # 0.5 m width std-dev is a "unit"
_NORM_SIDE = 1.0                       # one full wrong-side cone is a "unit"


@dataclass(frozen=True)
class ConeObservation:
    colour: str
    class_name: str
    confidence: float
    source: str
    points_count: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class DelaunayGate:
    """A blue<->yellow Delaunay edge: its midpoint sits on the centre line."""
    blue: 'TrackedCone'
    yellow: 'TrackedCone'
    midpoint: Point
    width: float


@dataclass
class TrackedCone:
    track_id: int
    colour: str
    class_name: str
    confidence: float
    source: str
    points_count: int
    x: float
    y: float
    z: float
    hit_count: int
    miss_count: int
    last_seen_sec: float

    @property
    def point(self) -> Point:
        return Point(x=self.x, y=self.y, z=self.z)


class PerceivedPath(Node):
    """Publishes a local path from corridor, one-sided, or held-path logic."""

    def __init__(self):
        super().__init__('perceived_path')

        # Mission self-gating. mission_gates is a CSV of /mission/selected values
        # for which this node publishes its path; empty = always publish (default,
        # preserves the standalone sim workflow). In the dynamic mission stack set
        # mission_gates:="autocross,trackdrive" and path_topic:=/nav/active_path.
        self.declare_parameter('mission_topic', '/mission/selected')
        self.declare_parameter('mission_gates', '')
        self.declare_parameter('input_topic', '/cones')
        self.declare_parameter('path_topic', '/nav/perceived_path')
        self.declare_parameter('debug_topic', '/nav/perceived_path_markers')
        self.declare_parameter('midline_method', 'forward_pairs')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 35.0)
        self.declare_parameter('max_abs_y', 8.0)
        self.declare_parameter('match_radius', 1.25)
        self.declare_parameter('confirm_hits', 2)
        self.declare_parameter('miss_limit', 4)
        self.declare_parameter('max_track_age_sec', 1.5)
        self.declare_parameter('assign_unknown_by_side', True)
        self.declare_parameter('unknown_cone_min_abs_y', 0.5)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('min_cones_per_side', 2)
        self.declare_parameter('min_one_side_cones', 3)
        self.declare_parameter('min_pair_distance', 1.5)
        self.declare_parameter('max_pair_distance', 5.0)
        self.declare_parameter('max_pair_x_gap', 3.0)
        # Max absolute (Euclidean, axis-independent) distance between consecutive
        # ordered midpoints. The path is truncated at the first larger jump so it
        # cannot hop across the track to a spurious opposite-side midpoint. Applied
        # before sampling, because sampling interpolates across any gap and would
        # otherwise mask it from the follower's own segment-gap check.
        self.declare_parameter('max_midpoint_gap', 4.0)
        # Cost-based Delaunay midline (midline_method='delaunay'). Paths are grown
        # from the car through the triangle graph and ranked by a weighted cost.
        self.declare_parameter('delaunay_target_length_m', 10.0)
        self.declare_parameter('delaunay_seed_max_x', 8.0)
        self.declare_parameter('delaunay_beam_width', 5)
        self.declare_parameter('delaunay_max_depth', 15)
        self.declare_parameter('delaunay_w_angle', 1.0)
        self.declare_parameter('delaunay_w_width', 1.0)
        self.declare_parameter('delaunay_w_length', 1.0)
        self.declare_parameter('delaunay_w_side', 1.5)
        # Chaikin corner-cutting passes applied to the chosen centre-line, to
        # remove the zig-zag spikes of threading the Delaunay midpoint cloud.
        # 0 disables; 2-3 gives a smooth line.
        self.declare_parameter('delaunay_smoothing_iterations', 2)
        self.declare_parameter('sample_spacing', 0.5)
        self.declare_parameter('offset_distance', 1.5)
        self.declare_parameter('blue_offset_sign', -1.0)
        self.declare_parameter('yellow_offset_sign', 1.0)
        self.declare_parameter('fallback_preferred_colour', 'blue')
        self.declare_parameter('previous_path_hold_sec', 0.8)
        self.declare_parameter('boundary_line_width', 0.05)
        self.declare_parameter('centerline_width', 0.08)
        self.declare_parameter('point_diameter', 0.16)

        input_topic = self.get_parameter('input_topic').value
        path_topic = self.get_parameter('path_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self._midline_method = self._normalise_midline_method(
            str(self.get_parameter('midline_method').value)
        )
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._min_x = float(self.get_parameter('min_x').value)
        self._max_x = float(self.get_parameter('max_x').value)
        self._max_abs_y = float(self.get_parameter('max_abs_y').value)
        self._match_radius = float(self.get_parameter('match_radius').value)
        self._confirm_hits = int(self.get_parameter('confirm_hits').value)
        self._miss_limit = int(self.get_parameter('miss_limit').value)
        self._max_track_age_sec = float(self.get_parameter('max_track_age_sec').value)
        self._assign_unknown_by_side = bool(self.get_parameter('assign_unknown_by_side').value)
        self._unknown_cone_min_abs_y = float(self.get_parameter('unknown_cone_min_abs_y').value)
        self._smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self._min_cones_per_side = int(self.get_parameter('min_cones_per_side').value)
        self._min_one_side_cones = int(self.get_parameter('min_one_side_cones').value)
        self._min_pair_distance = float(self.get_parameter('min_pair_distance').value)
        self._max_pair_distance = float(self.get_parameter('max_pair_distance').value)
        self._max_pair_x_gap = float(self.get_parameter('max_pair_x_gap').value)
        self._max_midpoint_gap = float(self.get_parameter('max_midpoint_gap').value)
        self._delaunay_target_length_m = float(self.get_parameter('delaunay_target_length_m').value)
        self._delaunay_seed_max_x = float(self.get_parameter('delaunay_seed_max_x').value)
        self._delaunay_beam_width = max(1, int(self.get_parameter('delaunay_beam_width').value))
        self._delaunay_max_depth = max(1, int(self.get_parameter('delaunay_max_depth').value))
        self._delaunay_w_angle = float(self.get_parameter('delaunay_w_angle').value)
        self._delaunay_w_width = float(self.get_parameter('delaunay_w_width').value)
        self._delaunay_w_length = float(self.get_parameter('delaunay_w_length').value)
        self._delaunay_w_side = float(self.get_parameter('delaunay_w_side').value)
        self._delaunay_smoothing_iterations = max(0, int(self.get_parameter('delaunay_smoothing_iterations').value))
        self._sample_spacing = float(self.get_parameter('sample_spacing').value)
        self._offset_distance = float(self.get_parameter('offset_distance').value)
        self._blue_offset_sign = float(self.get_parameter('blue_offset_sign').value)
        self._yellow_offset_sign = float(self.get_parameter('yellow_offset_sign').value)
        self._fallback_preferred_colour = str(
            self.get_parameter('fallback_preferred_colour').value
        ).lower()
        self._previous_path_hold_sec = float(self.get_parameter('previous_path_hold_sec').value)
        self._boundary_line_width = float(self.get_parameter('boundary_line_width').value)
        self._centerline_width = float(self.get_parameter('centerline_width').value)
        self._point_diameter = float(self.get_parameter('point_diameter').value)

        self._tracks: list[TrackedCone] = []
        self._next_track_id = 0
        self._last_frame_id = 'Fr1A'
        self._last_stamp = self.get_clock().now().to_msg()
        self._last_input_stamp_sec: float | None = None
        self._previous_path: list[Point] = []
        self._previous_path_time_sec = -1.0
        self._previous_strategy = 'none'

        gates_raw = str(self.get_parameter('mission_gates').value)
        self._mission_gates = {m.strip() for m in gates_raw.split(',') if m.strip()}
        self._selected_mission = 'none'
        mission_topic = str(self.get_parameter('mission_topic').value)

        self._sub = self.create_subscription(
            Cone3DArray,
            input_topic,
            self._on_cones,
            10,
        )
        self.create_subscription(String, mission_topic, self._on_mission, 10)
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._debug_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_outputs)

        self.get_logger().info(
            f'Perceived path planner listening on {input_topic}; '
            f'publishing path={path_topic}, debug={debug_topic}, method={self._midline_method}'
        )

    def _on_cones(self, msg: Cone3DArray) -> None:
        observations, frame_id, stamp, input_stamp_sec = self._extract_observations(msg)
        now_sec = self._now_sec()
        self._last_frame_id = frame_id
        self._last_stamp = stamp

        if self._last_input_stamp_sec is not None and input_stamp_sec < self._last_input_stamp_sec - 0.5:
            self._tracks.clear()
            self._previous_path = []
            self.get_logger().debug('Cone timestamp moved backwards; cleared planner memory')
        self._last_input_stamp_sec = input_stamp_sec

        self._match_and_update_tracks(observations, now_sec)
        self._prune_tracks(now_sec)

    def _extract_observations(
        self,
        msg: Cone3DArray,
    ) -> tuple[list[ConeObservation], str, object, float]:
        observations = []
        frame_id = msg.header.frame_id or self._last_frame_id
        stamp = msg.header.stamp

        for cone in msg.cones:
            if cone.header.frame_id:
                frame_id = cone.header.frame_id
            if cone.header.stamp.sec or cone.header.stamp.nanosec:
                stamp = cone.header.stamp

            class_name = cone.class_name
            colour = self._colour_from_class_name(class_name)
            position = cone.position

            if colour is None:
                # LiDAR-only cone (unknown_cone) — assign side by Y position in vehicle frame.
                # Y > 0 is left (blue), Y < 0 is right (yellow).
                # Cones within unknown_cone_min_abs_y of the centreline are ambiguous; skip them.
                if self._assign_unknown_by_side:
                    if position.y >= self._unknown_cone_min_abs_y:
                        colour = 'blue'
                    elif position.y <= -self._unknown_cone_min_abs_y:
                        colour = 'yellow'
                    else:
                        continue
                else:
                    continue
            elif colour not in ('blue', 'yellow'):
                # orange or other named non-navigation class
                continue
            if not self._is_in_local_window(position):
                continue

            observations.append(
                ConeObservation(
                    colour=colour,
                    class_name=class_name,
                    confidence=float(cone.confidence),
                    source=cone.source,
                    points_count=len(cone.points),
                    x=position.x,
                    y=position.y,
                    z=position.z,
                )
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
            if obs_idx in matched_observations:
                continue
            self._tracks.append(
                TrackedCone(
                    track_id=self._next_track_id,
                    colour=obs.colour,
                    class_name=obs.class_name,
                    confidence=obs.confidence,
                    source=obs.source,
                    points_count=obs.points_count,
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
        track.class_name = obs.class_name
        track.confidence = obs.confidence
        track.source = obs.source
        track.points_count = obs.points_count
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

    def _on_mission(self, msg: String) -> None:
        self._selected_mission = msg.data

    def _mission_gated_out(self) -> bool:
        """True when self-gating is enabled and this mission is not selected — in
        which case the node stays silent so it never overwrites the shared path."""
        return bool(self._mission_gates) and self._selected_mission not in self._mission_gates

    def _publish_outputs(self) -> None:
        if self._mission_gated_out():
            return
        now_sec = self._now_sec()
        self._prune_tracks(now_sec)

        confirmed = [track for track in self._tracks if track.hit_count >= self._confirm_hits]
        tentative = [track for track in self._tracks if track.hit_count < self._confirm_hits]
        blue = self._tracks_by_colour(confirmed, 'blue')
        yellow = self._tracks_by_colour(confirmed, 'yellow')

        strategy, path, debug_context = self._calculate_path(blue, yellow, now_sec)
        if path and strategy not in ('held_previous_path', 'none'):
            self._previous_path = path
            self._previous_path_time_sec = now_sec
            self._previous_strategy = strategy

        self._path_pub.publish(self._make_path_msg(path))
        self._debug_pub.publish(
            self._make_debug_markers(
                strategy,
                path,
                blue,
                yellow,
                tentative,
                confirmed,
                debug_context,
            )
        )

    def _calculate_path(
        self,
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
        now_sec: float,
    ) -> tuple[str, list[Point], dict]:
        context: dict = {}

        if len(blue) >= self._min_cones_per_side and len(yellow) >= self._min_cones_per_side:
            if self._midline_method == 'delaunay':
                midpoints, delaunay_context = self._delaunay_cost_midline(blue, yellow)
                if len(midpoints) >= 2:
                    path = self._sample_polyline_points(midpoints)
                    delaunay_context['midpoints'] = midpoints
                    return 'delaunay_cost', path, delaunay_context
                # Delaunay produced nothing usable — fall through to forward_pairs.
                context = delaunay_context
            elif self._midline_method != 'forward_pairs':
                self.get_logger().warn(
                    f'Unknown midline_method={self._midline_method}; using forward_pairs'
                )
                self._midline_method = 'forward_pairs'

            pairs = self._pair_by_forward_position(blue, yellow)
            if len(pairs) >= 2:
                midpoints = [self._midpoint(left, right) for left, right in pairs]
                midpoints = self._limit_midpoint_gaps(midpoints)
                pairs = pairs[:len(midpoints)]
                if len(midpoints) >= 2:
                    path = self._sample_polyline_points(midpoints)
                    # Merge (not overwrite) so any Delaunay debug survives a fallback.
                    context = {**context, 'pairs': pairs, 'midpoints': midpoints}
                    return 'forward_pair_midpoint', path, context

        fallback_colour, fallback_tracks = self._choose_one_sided_fallback(blue, yellow)
        if len(fallback_tracks) >= self._min_one_side_cones:
            sampled_boundary, path = self._one_sided_offset_path(fallback_colour, fallback_tracks)
            context = {
                **context,
                'fallback_colour': fallback_colour,
                'fallback_tracks': fallback_tracks,
                'sampled_boundary': sampled_boundary,
            }
            return f'one_sided_{fallback_colour}', path, context

        if self._previous_path and now_sec - self._previous_path_time_sec <= self._previous_path_hold_sec:
            context = {**context, 'previous_strategy': self._previous_strategy}
            return 'held_previous_path', self._previous_path, context

        return 'none', [], context

    @staticmethod
    def _normalise_midline_method(method: str) -> str:
        normalised = method.strip().lower().replace('-', '_')
        if normalised in ('midpoint', 'midpoints', 'forward_midpoint', 'forward_midpoints'):
            return 'forward_pairs'
        return normalised

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
            width = self._distance_xy(blue_track.point, yellow_track.point)
            if width < self._min_pair_distance or width > self._max_pair_distance:
                continue
            pairs.append((blue_track, yellow_track))
            used_yellow.add(yellow_idx)

        pairs.sort(key=lambda pair: (pair[0].x + pair[1].x) / 2.0)
        return pairs

    # ── Cost-based Delaunay midline (paper sec. 4.3.1) ─────────────────────

    def _delaunay_cost_midline(
        self,
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
    ) -> tuple[list[Point], dict]:
        """Triangulate, grow candidate centre-lines from the car, pick lowest cost.

        Returns (chosen midpoints near->far, debug context). The context always
        carries the triangulation edges and every candidate gate midpoint so the
        selection is inspectable in RViz, even when no usable path is found.
        """
        gates, adjacency, raw_edges = self._delaunay_gates(blue, yellow)
        context: dict = {
            'delaunay_edges': raw_edges,
            'candidate_midpoints': [gate.midpoint for gate in gates],
        }
        if not gates:
            return [], context

        candidates = [path for path in self._grow_candidate_paths(gates, adjacency) if path]
        if not candidates:
            return [], context

        scored = sorted(candidates, key=lambda path: (
            self._path_cost(path, gates, include_length=True),
            math.hypot(gates[path[0]].midpoint.x, gates[path[0]].midpoint.y),
        ))
        best = scored[0]
        best_points = self._limit_midpoint_gaps([gates[idx].midpoint for idx in best])
        best_points = self._chaikin_smooth(best_points, self._delaunay_smoothing_iterations)
        context['pairs'] = [(gates[idx].blue, gates[idx].yellow) for idx in best]
        if len(scored) > 1:
            context['runner_up'] = [gates[idx].midpoint for idx in scored[1]]
        return best_points, context

    def _delaunay_gates(
        self,
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
    ) -> tuple[list[DelaunayGate], dict, list[tuple[Point, Point]]]:
        """Delaunay triangulate; return blue<->yellow gates, their triangle
        adjacency, and all triangulation edges (for debug)."""
        objects = blue + yellow
        if len(objects) < 3:
            return [], {}, []

        points = [(cone.x, cone.y) for cone in objects]
        try:
            triangulation = Delaunay(points)
        except QhullError as exc:
            self.get_logger().debug(f'Perceived Delaunay failed: {exc}')
            return [], {}, []

        raw_edge_keys = set()
        for simplex in triangulation.simplices:
            a, b, c = int(simplex[0]), int(simplex[1]), int(simplex[2])
            for i, j in ((a, b), (b, c), (c, a)):
                raw_edge_keys.add(tuple(sorted((i, j))))
        raw_edges = [(objects[i].point, objects[j].point) for i, j in raw_edge_keys]

        gate_index_by_edge: dict[tuple[int, int], int] = {}
        gates: list[DelaunayGate] = []

        def gate_for_edge(i: int, j: int) -> int | None:
            key = tuple(sorted((i, j)))
            if key in gate_index_by_edge:
                return gate_index_by_edge[key]
            ci, cj = objects[i], objects[j]
            if {ci.colour, cj.colour} != {'blue', 'yellow'}:
                return None
            blue_cone = ci if ci.colour == 'blue' else cj
            yellow_cone = cj if ci.colour == 'blue' else ci
            width = self._distance_xy(ci.point, cj.point)
            if width < self._min_pair_distance or width > self._max_pair_distance:
                return None
            midpoint = self._midpoint(blue_cone, yellow_cone)
            if not self._is_in_local_window(midpoint):
                return None
            index = len(gates)
            gates.append(DelaunayGate(blue=blue_cone, yellow=yellow_cone, midpoint=midpoint, width=width))
            gate_index_by_edge[key] = index
            return index

        adjacency: dict[int, set[int]] = {}
        for simplex in triangulation.simplices:
            a, b, c = int(simplex[0]), int(simplex[1]), int(simplex[2])
            triangle_gates = []
            for i, j in ((a, b), (b, c), (c, a)):
                gate_idx = gate_for_edge(i, j)
                if gate_idx is not None:
                    triangle_gates.append(gate_idx)
            for x in triangle_gates:
                for y in triangle_gates:
                    if x != y:
                        adjacency.setdefault(x, set()).add(y)

        return gates, adjacency, raw_edges

    def _grow_candidate_paths(self, gates: list[DelaunayGate], adjacency: dict) -> list[list[int]]:
        """Breadth-first, beam-pruned growth of candidate paths (gate-index lists)
        rooted at the car, stepping through the triangle adjacency graph."""
        # Seed with the gates nearest the car (so paths always root close to the
        # vehicle), preferring those within seed_max_x. Sorting by distance and
        # capping to the beam width keeps the near corridor from being pruned away.
        forward = [
            idx for idx, gate in enumerate(gates)
            if gate.midpoint.x > 0.0 and abs(gate.midpoint.y) <= self._max_abs_y
        ]
        forward.sort(key=lambda idx: math.hypot(gates[idx].midpoint.x, gates[idx].midpoint.y))
        seeds = [idx for idx in forward if gates[idx].midpoint.x <= self._delaunay_seed_max_x]
        seeds = (seeds or forward)[:self._delaunay_beam_width]
        if not seeds:
            return []

        live = [[seed] for seed in seeds]
        completed: list[list[int]] = []

        for _ in range(self._delaunay_max_depth):
            if not live:
                break
            # Break cost ties toward paths rooted nearest the car, so a clean
            # (near-zero-cost) corridor keeps its near-rooted path instead of
            # arbitrarily pruning it away.
            live.sort(key=lambda path: (
                self._path_cost(path, gates, include_length=False),
                math.hypot(gates[path[0]].midpoint.x, gates[path[0]].midpoint.y),
            ))
            live = live[:self._delaunay_beam_width]
            next_live: list[list[int]] = []
            for path in live:
                points = self._delaunay_path_points(path, gates)
                if self._polyline_length(points) >= self._delaunay_target_length_m:
                    completed.append(path)
                    continue
                extensions = [
                    nxt for nxt in adjacency.get(path[-1], ())
                    if nxt not in path and self._is_forward_step(points, gates[nxt].midpoint)
                ]
                if not extensions:
                    completed.append(path)
                    continue
                for nxt in extensions:
                    next_live.append(path + [nxt])
            live = next_live

        completed.extend(live)
        return completed

    def _path_cost(self, path: list[int], gates: list[DelaunayGate], include_length: bool) -> float:
        """Weighted sum of normalized-squared cost terms (paper Table 3 subset).

        Terms: max segment angle change (smoothness), gate-width std-dev,
        length vs. target (optional), and blue-left/yellow-right side-consistency
        weighted by cone confidence. Lower is better."""
        points = self._delaunay_path_points(path, gates)

        # angle change (max |heading delta| over consecutive segments)
        headings = [
            math.atan2(points[k + 1].y - points[k].y, points[k + 1].x - points[k].x)
            for k in range(len(points) - 1)
        ]
        angle_cost = 0.0
        for k in range(len(headings) - 1):
            angle_cost = max(angle_cost, abs(self._angle_diff(headings[k + 1], headings[k])))

        # width std-dev
        widths = [gates[idx].width for idx in path]
        width_cost = self._stddev(widths)

        # side consistency: blue should be left of heading into the gate, yellow right
        side_cost = 0.0
        for seg_idx, gate_idx in enumerate(path):
            hx = points[seg_idx + 1].x - points[seg_idx].x
            hy = points[seg_idx + 1].y - points[seg_idx].y
            gate = gates[gate_idx]
            mid = gate.midpoint
            cross_blue = hx * (gate.blue.y - mid.y) - hy * (gate.blue.x - mid.x)
            cross_yellow = hx * (gate.yellow.y - mid.y) - hy * (gate.yellow.x - mid.x)
            if cross_blue < 0.0:       # blue on the right -> wrong
                side_cost += gate.blue.confidence
            if cross_yellow > 0.0:     # yellow on the left -> wrong
                side_cost += gate.yellow.confidence
        side_cost = side_cost / max(1, len(path))

        # normalize (relative to typical scales), square, weight, sum
        n_angle = angle_cost / _NORM_ANGLE_RAD
        n_width = width_cost / _NORM_WIDTH_M
        n_side = side_cost / _NORM_SIDE
        total = (
            self._delaunay_w_angle * n_angle * n_angle
            + self._delaunay_w_width * n_width * n_width
            + self._delaunay_w_side * n_side * n_side
        )
        if include_length:
            length = self._polyline_length(points)
            n_length = (length - self._delaunay_target_length_m) / max(1e-3, self._delaunay_target_length_m)
            total += self._delaunay_w_length * n_length * n_length
        return total

    def _delaunay_path_points(self, path: list[int], gates: list[DelaunayGate]) -> list[Point]:
        """Car origin followed by the gate midpoints of the path."""
        return [Point(x=0.0, y=0.0, z=0.0)] + [gates[idx].midpoint for idx in path]

    @staticmethod
    def _is_forward_step(points: list[Point], candidate: Point) -> bool:
        """True if candidate lies ahead of the path's tip along its current heading."""
        last = points[-1]
        prev = points[-2] if len(points) >= 2 else Point(x=0.0, y=0.0, z=0.0)
        hx, hy = last.x - prev.x, last.y - prev.y
        return hx * (candidate.x - last.x) + hy * (candidate.y - last.y) > 0.0

    @staticmethod
    def _polyline_length(points: list[Point]) -> float:
        return sum(
            math.hypot(points[k + 1].x - points[k].x, points[k + 1].y - points[k].y)
            for k in range(len(points) - 1)
        )

    @staticmethod
    def _chaikin_smooth(points: list[Point], iterations: int) -> list[Point]:
        """Chaikin corner-cutting: replace each segment's endpoints with points at
        1/4 and 3/4, keeping the true endpoints. A few passes turn the jagged
        midpoint polyline into a smooth centre-line the follower can track cleanly."""
        for _ in range(iterations):
            if len(points) < 3:
                break
            smoothed = [points[0]]
            for a, b in zip(points[:-1], points[1:]):
                smoothed.append(Point(x=0.75 * a.x + 0.25 * b.x, y=0.75 * a.y + 0.25 * b.y, z=0.75 * a.z + 0.25 * b.z))
                smoothed.append(Point(x=0.25 * a.x + 0.75 * b.x, y=0.25 * a.y + 0.75 * b.y, z=0.25 * a.z + 0.75 * b.z))
            smoothed.append(points[-1])
            points = smoothed
        return points

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _stddev(values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    def _choose_one_sided_fallback(
        self,
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
    ) -> tuple[str, list[TrackedCone]]:
        if len(blue) > len(yellow):
            return 'blue', blue
        if len(yellow) > len(blue):
            return 'yellow', yellow
        if self._fallback_preferred_colour == 'yellow':
            return 'yellow', yellow
        return 'blue', blue

    def _one_sided_offset_path(
        self,
        colour: str,
        tracks: list[TrackedCone],
    ) -> tuple[list[Point], list[Point]]:
        boundary = [track.point for track in tracks]
        samples = self._sample_polyline_with_tangents(boundary)
        offset_sign = self._yellow_offset_sign if colour == 'yellow' else self._blue_offset_sign
        centreline = []

        for sample, tangent_x, tangent_y in samples:
            normal_x = -tangent_y
            normal_y = tangent_x
            centreline.append(
                Point(
                    x=sample.x + offset_sign * self._offset_distance * normal_x,
                    y=sample.y + offset_sign * self._offset_distance * normal_y,
                    z=sample.z,
                )
            )

        return [sample for sample, _, _ in samples], centreline

    def _limit_midpoint_gaps(self, midpoints: list[Point]) -> list[Point]:
        """Keep the leading run of midpoints whose consecutive absolute spacing is
        within max_midpoint_gap, dropping everything from the first larger jump on.

        Midpoints arrive ordered near-to-far (by x), so this retains the connected
        forward path and severs an over-long hop to a spurious opposite-side
        midpoint before it is sampled/interpolated into the published path.
        """
        if len(midpoints) < 2:
            return midpoints
        limited = [midpoints[0]]
        for previous, current in zip(midpoints[:-1], midpoints[1:]):
            if self._distance_xy(previous, current) > self._max_midpoint_gap:
                break
            limited.append(current)
        return limited

    def _sample_polyline_points(self, points: list[Point]) -> list[Point]:
        return [sample for sample, _, _ in self._sample_polyline_with_tangents(points)]

    def _sample_polyline_with_tangents(self, points: list[Point]) -> list[tuple[Point, float, float]]:
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

    def _make_path_msg(self, points: list[Point]) -> Path:
        path = Path()
        path.header.frame_id = self._last_frame_id
        path.header.stamp = self._last_stamp

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _make_debug_markers(
        self,
        strategy: str,
        path: list[Point],
        blue: list[TrackedCone],
        yellow: list[TrackedCone],
        tentative: list[TrackedCone],
        confirmed: list[TrackedCone],
        context: dict,
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self._delete_all())
        markers.markers.append(self._strategy_text(0, strategy, len(path), len(blue), len(yellow), confirmed))

        if blue:
            markers.markers.append(self._point_list(1, 'hybrid_blue_tracks', [t.point for t in blue], self._blue(), self._point_diameter))
            if len(blue) >= 2:
                markers.markers.append(self._line_strip(2, 'hybrid_blue_boundary', [t.point for t in blue], self._blue(), self._boundary_line_width))
        if yellow:
            markers.markers.append(self._point_list(3, 'hybrid_yellow_tracks', [t.point for t in yellow], self._yellow(), self._point_diameter))
            if len(yellow) >= 2:
                markers.markers.append(self._line_strip(4, 'hybrid_yellow_boundary', [t.point for t in yellow], self._yellow(), self._boundary_line_width))

        pairs = context.get('pairs', [])
        if pairs:
            markers.markers.append(self._pair_lines(5, pairs))

        midpoints = context.get('midpoints', [])
        if midpoints:
            markers.markers.append(self._point_list(6, 'hybrid_corridor_midpoints', midpoints, self._green(), self._point_diameter))

        sampled_boundary = context.get('sampled_boundary', [])
        fallback_colour = context.get('fallback_colour', '')
        if sampled_boundary:
            markers.markers.append(
                self._point_list(
                    7,
                    'hybrid_one_sided_boundary_samples',
                    sampled_boundary,
                    self._colour_rgba(fallback_colour, alpha=0.55),
                    0.08,
                )
            )

        if path:
            colour = self._orange() if strategy == 'held_previous_path' else self._green()
            markers.markers.append(self._point_list(8, 'hybrid_path_points', path, colour, 0.12))
            if len(path) >= 2:
                markers.markers.append(self._line_strip(9, 'hybrid_path_line', path, colour, self._centerline_width))

        if tentative:
            markers.markers.append(self._point_list(10, 'hybrid_tentative_tracks', [t.point for t in tentative], self._grey(), 0.12))

        held_tracks = [track for track in confirmed if track.miss_count > 0]
        if held_tracks:
            markers.markers.append(self._point_list(11, 'hybrid_held_tracks', [t.point for t in held_tracks], self._orange(), 0.12))

        # Delaunay planner debug: the triangulation, all candidate gate midpoints,
        # and the runner-up path the cost rejected.
        delaunay_edges = context.get('delaunay_edges', [])
        if delaunay_edges:
            markers.markers.append(self._edge_list(12, 'delaunay_edges', delaunay_edges, self._faint_grey()))
        candidate_midpoints = context.get('candidate_midpoints', [])
        if candidate_midpoints:
            markers.markers.append(self._point_list(13, 'delaunay_candidate_midpoints', candidate_midpoints, self._green(), 0.14))
        runner_up = context.get('runner_up', [])
        if len(runner_up) >= 2:
            markers.markers.append(self._line_strip(14, 'delaunay_runner_up', runner_up, self._faint_grey(), self._boundary_line_width))

        return markers

    def _edge_list(self, marker_id: int, namespace: str, edges: list[tuple[Point, Point]], colour: ColorRGBA) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_LIST)
        marker.scale.x = 0.04
        marker.color = colour
        for start, end in edges:
            marker.points.append(start)
            marker.points.append(end)
        return marker

    @staticmethod
    def _midpoint(left: TrackedCone, right: TrackedCone) -> Point:
        return Point(
            x=(left.x + right.x) / 2.0,
            y=(left.y + right.y) / 2.0,
            z=(left.z + right.z) / 2.0,
        )

    @staticmethod
    def _tracks_by_colour(tracks: list[TrackedCone], colour: str) -> list[TrackedCone]:
        filtered = [track for track in tracks if track.colour == colour]
        filtered.sort(key=lambda track: track.x)
        return filtered

    @staticmethod
    def _colour_from_class_name(class_name: str) -> str | None:
        class_name = class_name.lower()
        if 'blue' in class_name:
            return 'blue'
        if 'yellow' in class_name:
            return 'yellow'
        if 'orange' in class_name:
            return 'orange'
        return None

    @staticmethod
    def _distance_xy(left: Point, right: Point) -> float:
        return math.hypot(left.x - right.x, left.y - right.y)

    def _is_in_local_window(self, point: Point) -> bool:
        return self._min_x <= point.x <= self._max_x and abs(point.y) <= self._max_abs_y

    def _delete_all(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_frame_id
        marker.header.stamp = self._last_stamp
        marker.action = Marker.DELETEALL
        return marker

    def _strategy_text(
        self,
        marker_id: int,
        strategy: str,
        path_points: int,
        blue_count: int,
        yellow_count: int,
        confirmed: list[TrackedCone],
    ) -> Marker:
        marker = self._base_marker(marker_id, 'hybrid_strategy', Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 1.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.25
        marker.color = self._white()
        sources = self._source_summary(confirmed)
        marker.text = (
            f'strategy: {strategy}\n'
            f'method: {self._midline_method}\n'
            f'path points: {path_points}\n'
            f'blue: {blue_count}  yellow: {yellow_count}\n'
            f'sources: {sources}'
        )
        return marker

    @staticmethod
    def _source_summary(tracks: list[TrackedCone]) -> str:
        if not tracks:
            return 'none'
        counts = {}
        for track in tracks:
            source = track.source or 'unknown'
            counts[source] = counts.get(source, 0) + 1
        return ', '.join(f'{source}:{count}' for source, count in sorted(counts.items()))

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
        marker = self._base_marker(marker_id, 'hybrid_pair_lines', Marker.LINE_LIST)
        marker.scale.x = 0.025
        marker.color = self._grey()
        for left, right in pairs:
            marker.points.append(left.point)
            marker.points.append(right.point)
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

    def _colour_rgba(self, colour: str, alpha: float = 1.0) -> ColorRGBA:
        if colour == 'yellow':
            return self._yellow(alpha)
        if colour == 'blue':
            return self._blue(alpha)
        return self._grey()

    @staticmethod
    def _blue(alpha: float = 1.0) -> ColorRGBA:
        return ColorRGBA(r=0.1, g=0.35, b=1.0, a=alpha)

    @staticmethod
    def _yellow(alpha: float = 1.0) -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.9, b=0.0, a=alpha)

    @staticmethod
    def _green() -> ColorRGBA:
        return ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

    @staticmethod
    def _grey() -> ColorRGBA:
        return ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.35)

    @staticmethod
    def _faint_grey() -> ColorRGBA:
        return ColorRGBA(r=0.75, g=0.75, b=0.8, a=0.9)

    @staticmethod
    def _orange() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.8)

    @staticmethod
    def _white() -> ColorRGBA:
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PerceivedPath()
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
