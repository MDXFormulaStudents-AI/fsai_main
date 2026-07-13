# Skidpad Navigation Conversation Summary

## Objective

Build and tune the FSAI skidpad navigation stack so the vehicle can:

- Approach the skidpad entry crossing.
- Detect and lock the orange cone crossing centroid.
- Drive two clockwise right loops.
- Drive two counter-clockwise left loops.
- Exit straight and stop safely.

The main recurring issue was that odometry drift and imperfect vehicle tracking caused the fixed geometric loop path to become misaligned with the real cone layout. We explored several approaches for re-aligning the loop while trying to avoid false locks onto the straight entry/exit cones.

## Core Skidpad Geometry

Current assumptions:

- Centerline radius: `9.125 m`
- Track width: `3.0 m`
- Outside boundary radius: `10.625 m`
- Inside boundary radius: `7.625 m`
- Left/right circle center separation: `18.25 m`

The orange crossing centroid is used as the shared tangent/crossing point between the right and left skidpad circles.

## Important Runtime Setup

Before launching skidpad navigation, the CarMaker odom TF chain must be active:

```bash
ros2 launch fsai_visualisation odom_forward_debug.launch.py
```

Expected TF chain:

```text
odom -> home -> base_link -> Fr1A
```

Then launch skidpad navigation:

```bash
ros2 launch fsai_navigation skidpad.launch.py
```

Build command used repeatedly:

```bash
colcon build --packages-select fsai_navigation --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Key Files

- `fsai_ros2_ws/src/fsai_navigation/fsai_navigation/skidpad_path.py`
  - Skidpad path generation, state machine, orange crossing detection, mid-loop correction logic, RViz debug markers.

- `fsai_ros2_ws/src/fsai_navigation/launch/skidpad.launch.py`
  - Launches `skidpad_path` and `local_path_follower`, exposes skidpad and correction parameters.

- `fsai_ros2_ws/src/fsai_navigation/fsai_navigation/local_path_follower.py`
  - Pure-pursuit local path follower, speed cap, odom speed feedback, steering/gas/brake command output.

- `fsai_ros2_ws/src/fsai_navigation/setup.py`
  - Contains console script registration for `skidpad_path`.

## Current State Machine

The skidpad path node uses:

```text
APPROACH -> RIGHT_LOOP -> LEFT_LOOP -> EXIT -> STOPPED
```

Behavior:

- `APPROACH`: drive straight and detect the orange crossing centroid.
- `RIGHT_LOOP`: publish rolling clockwise arc around the right circle.
- `LEFT_LOOP`: publish rolling counter-clockwise arc around the left circle.
- `EXIT`: publish straight path along approach heading.
- `STOPPED`: publish no path; follower applies stop behavior.

Loop counting is based on cumulative signed arc around the active circle center:

- Right loop: clockwise, negative arc accumulation.
- Left loop: counter-clockwise, positive arc accumulation.

## Orange Crossing Detection Work

We fixed the initial crossing detection so that:

- The orange centroid is locked only after `min_approach_dist` arms the detector.
- Crossing confirmation is driven by odometry, not by continuously changing cone detections.
- The centroid is not allowed to jump from the actual crossing cones to far orange cones later.

Important params:

- `min_approach_dist`
- `max_approach_dist`
- `orange_max_range`
- `orange_max_angle_deg`
- `min_orange_cluster_size`
- `crossing_hold_m`

Correct parameter syntax example:

```bash
ros2 launch fsai_navigation skidpad.launch.py min_approach_dist:=6.0
```

## Re-Anchoring / Correction Approaches Tried

### 1. Crossing Re-Anchor

We tried a crossing-based `REANCHOR` state after a loop section.

Outcome:

- Late re-anchor missed the orange cones.
- Early re-anchor caused the car to go badly off-line before the first loop completed.
- This approach was abandoned.

### 2. Delaunay / Pairing Midpoint Correction

We added midpoint extraction from blue/yellow cone pairs:

- Delaunay triangulation using cross-colour edges within a track-width band.
- Nearest blue/yellow pairing fallback.
- Circle center nudged toward inferred midpoints.

Problem:

- When correction ran too early, it could lock onto cones straight ahead near the entry/exit and pull the path straight instead of into the circle.

### 3. Delayed Correction Gate

We added a correction delay by arc progress, e.g. only start correction after `60 deg` or `90 deg` into the loop.

Problem:

- Still not reliable enough.
- This was removed during the return-to-basics rollback.

### 4. Full Rollback To Fixed Circle Basics

We removed all re-anchor/correction tricks temporarily and kept only:

- Orange centroid detection.
- Fixed right/left circle centers.
- Arc-based loop counting.
- Path and debug visualization.

This gave a stable baseline again.

### 5. Current Approach: Mid-Loop Perception Window

We reintroduced correction under a stricter contract:

- Correction only runs during `RIGHT_LOOP` / `LEFT_LOOP`.
- Cone detections are filtered through a mid-loop arc window.
- Cone detections are also filtered through an expected radial band around the active circle.
- This is intended to reject cones from the start/exit straight.

Current method selector:

```text
mid_loop_detection_method:=off
mid_loop_detection_method:=pairing
mid_loop_detection_method:=delaunay
mid_loop_detection_method:=both
mid_loop_detection_method:=single_side
```

Default currently exposed in launch:

```text
mid_loop_detection_method:=delaunay
```

## Current Mid-Loop Detection Parameters

Launch args include:

```text
mid_loop_detection_method
mid_loop_detection_start_deg
mid_loop_detection_end_deg
mid_loop_detection_min_ahead
mid_loop_detection_max_ahead
mid_loop_detection_track_width_min
mid_loop_detection_track_width_max
mid_loop_detection_radial_margin
mid_loop_detection_min_midpoints
mid_loop_detection_gain
mid_loop_detection_deadband
mid_loop_detection_max_step
```

Defaults currently include:

```text
mid_loop_detection_method:=delaunay
mid_loop_detection_start_deg:=75.0
mid_loop_detection_end_deg:=285.0
mid_loop_detection_min_ahead:=0.5
mid_loop_detection_max_ahead:=10.0
mid_loop_detection_track_width_min:=1.5
mid_loop_detection_track_width_max:=5.0
mid_loop_detection_radial_margin:=1.2
mid_loop_detection_min_midpoints:=2
mid_loop_detection_gain:=0.15
mid_loop_detection_deadband:=0.20
mid_loop_detection_max_step:=0.35
```

Example to narrow the mid-loop perception window:

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_start_deg:=90.0 mid_loop_detection_end_deg:=260.0
```

## Delaunay Debugging

Because Delaunay was failing to produce midpoints even when cones appeared to be visible and in the perception window, we added debug markers.

Marker namespaces on `/nav/skidpad_path_markers`:

- `mid_loop_delaunay_edges`
  - Grey `LINE_LIST` showing all Delaunay triangulation edges from accepted in-window cones.

- `mid_loop_delaunay_used_edges`
  - Bright cyan `LINE_LIST` showing cross-colour edges that pass the track-width filter and generate midpoint candidates.

Status text includes:

```text
tri: all/used
```

Interpretation examples:

```text
tri: 0/0
```

No triangulation occurred, likely not enough accepted cones.

```text
tri: 8/0
```

Delaunay triangulated accepted cones, but no cross-colour edge passed filtering.

```text
tri: 8/2
```

Two Delaunay edges were used to form midpoint candidates.

## Single-Sided Centerline Formation

Issue observed:

- Early in the first loop, the vehicle slightly understeers.
- Because of line-of-sight limitations, perception may only see one cone colour, often blue cones on the outside of the loop.
- Delaunay/pairing require both blue and yellow, so no midpoint is produced.

Implemented solution:

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=single_side
```

This disables Delaunay/pairing and uses only single-sided cone offsets.

Encoded colour rules:

```text
RIGHT_LOOP:
  blue   = outside
  yellow = inside

LEFT_LOOP:
  yellow = outside
  blue   = inside
```

For a perceived cone in `odom`, with active circle center `(cx, cy)`:

```python
dx = cone_x - cx
dy = cone_y - cy
radius = hypot(dx, dy)
radial = (dx / radius, dy / radius)
```

If the cone is outside:

```python
virtual_midpoint = cone_position - radial * (track_width / 2)
```

If the cone is inside:

```python
virtual_midpoint = cone_position + radial * (track_width / 2)
```

The virtual midpoint is then passed into the same active-circle center nudge logic.

Single-side parameters:

```text
mid_loop_single_side_track_width:=3.0
mid_loop_single_side_radial_tolerance:=1.5
mid_loop_single_side_min_cones:=1
```

Single-side debug marker:

```text
mid_loop_single_side_vectors
```

This draws a line from the perceived cone to the virtual midpoint.

Status text includes:

```text
single: N
```

## Debug Topics And Markers

Primary RViz topics:

- `/nav/skidpad_path_markers` as `MarkerArray`
- `/nav/local_path_follower_markers` as `MarkerArray`
- `/nav/skidpad_path` as `Path`
- `/cones`
- `/carmaker/odom`
- `/tf` and `/tf_static`

Useful marker namespaces:

- `skidpad_status`
- `vehicle_heading`
- `skidpad_path`
- `path_start`
- `path_end`
- `right_circle`
- `left_circle`
- `right_circle_center`
- `left_circle_center`
- `crossing_center`
- `orange_cluster`
- `orange_centroid`
- `orange_centroid_vector`
- `mid_loop_window_cones`
- `mid_loop_midpoints`
- `mid_loop_delaunay_edges`
- `mid_loop_delaunay_used_edges`
- `mid_loop_single_side_vectors`

Useful CLI checks:

```bash
ros2 topic list | grep -E 'skidpad|local_path|cones|odom|tf'
ros2 topic echo /nav/skidpad_path_markers --once
ros2 topic echo /nav/local_path_follower_markers --once
ros2 topic echo /nav/skidpad_path --once
ros2 run tf2_ros tf2_echo odom Fr1A
```

## Local Path Follower Notes

The local path follower is pure-pursuit based.

Important points discussed:

- `wheelbase` is the distance from rear axle center to front axle center, not rear track width.
- `max_speed_mps` is a speed cap, not a target-speed controller.
- Steering is based on curvature and wheelbase.
- Gas is feed-forward with curvature slowdown and odom-based speed cap/brake.

Current launch exposes follower tuning parameters such as:

```text
wheelbase
lookahead_distance
steer_gain
max_steer
max_steer_rate
curvature_slowdown_gain
cruise_gas
min_gas
max_gas
speed_brake_gain
max_speed_brake
```

## Verification Performed

Throughout the session, after relevant changes, we ran:

```bash
python3 -m py_compile src/fsai_navigation/fsai_navigation/skidpad_path.py src/fsai_navigation/launch/skidpad.launch.py
```

```bash
colcon build --packages-select fsai_navigation --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

```bash
ros2 launch fsai_navigation skidpad.launch.py --show-args
```

We also ran short startup checks with launch overrides such as:

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=delaunay
```

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=both mid_loop_detection_start_deg:=90.0 mid_loop_detection_end_deg:=260.0
```

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=single_side
```

Startup checks correctly produced expected warnings when CarMaker/TF were not running.

## Current Recommended Test Sequence

1. Start CarMaker and odom TF:

```bash
ros2 launch fsai_visualisation odom_forward_debug.launch.py
```

2. Test single-sided mode only:

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=single_side
```

3. Watch `/nav/skidpad_path_markers` for:

```text
mid-loop: ...
cones: N
midpoints: N
single: N
```

4. In RViz, inspect:

- `mid_loop_window_cones`
- `mid_loop_single_side_vectors`
- `mid_loop_midpoints`
- active circle center movement

5. If start/exit cones leak in, narrow the arc window:

```bash
ros2 launch fsai_navigation skidpad.launch.py mid_loop_detection_method:=single_side mid_loop_detection_start_deg:=90.0 mid_loop_detection_end_deg:=260.0
```

6. If valid single-side cones are rejected, loosen:

```bash
mid_loop_single_side_radial_tolerance:=2.0
```

7. If center movement is too aggressive or noisy, reduce:

```bash
mid_loop_detection_gain:=0.08
mid_loop_detection_max_step:=0.20
```

## Known Open Questions

- Whether the single-sided virtual midpoint method is stable enough in the first right loop when only outside blue cones are visible.
- Whether the mid-loop arc window should start earlier than `75 deg` for single-sided mode.
- Whether the radial tolerance should be wider during early understeer.
- Whether the active circle center nudge should reset `_prev_arc_angle`; currently it does, to avoid arc-progress jumps after center movement.
- Whether the direct path-shift alternative is worth trying later. Current recommendation was to avoid it because it risks creating local discontinuities, while center nudging preserves a smooth circular path.

## Current Mental Model

The safest architecture is:

```text
orange centroid -> initial fixed circle centers
loop state machine -> rolling arc path
mid-loop window -> trusted cone subset
midpoint formation method -> virtual/real centerline points
circle center nudge -> smooth global path correction
```

The current implementation supports comparing the midpoint formation methods by changing only:

```bash
mid_loop_detection_method:=delaunay
mid_loop_detection_method:=pairing
mid_loop_detection_method:=both
mid_loop_detection_method:=single_side
mid_loop_detection_method:=off
```
