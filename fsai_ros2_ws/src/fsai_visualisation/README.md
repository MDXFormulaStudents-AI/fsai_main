# fsai_visualisation

Visualisation + **monitoring station** for the MDX Formula Student AI stack. It
converts the custom cone messages into RViz markers and opens a full RViz
dashboard of the perception / navigation / localization pipeline.

Run it **on the compute**, or on **any laptop on the same `ROS_DOMAIN_ID` (42)
and DDS network** — the marker converters run locally on the viewer and subscribe
to the compute's topics over the network, so the compute only has to publish. It
is **env-aware** (`env:=sim|real`), mirroring `fsai_perception`.

---

## What it does

- Runs **two `cone_visualizer` instances**: `/cones` → `/cone_markers` (fused,
  coloured) and `/perception/lidar/cones` → `/lidar_cone_markers` (raw LiDAR
  detections — a key debug tool, always grey since they're pre-camera)
- Fused cones (camera-confirmed colour) render at full brightness using embedded `.mtl` materials
- LiDAR-only / unknown cones render as semi-transparent white-tinted "ghost" meshes
- Publishes a static race car body + wheels marker at 10 Hz in the `Fr1A` frame
- Opens RViz with a full monitoring layout (`config/fsai.rviz`): point cloud,
  cones (fused + raw), camera detections, nav paths + follower debug, and odometry

> **Why the converters run on the viewer:** RViz cannot render the custom
> `fsai_interfaces/Cone3DArray` message, so `cone_visualizer` translates it to
> `visualization_msgs/MarkerArray`. Running it on the laptop keeps all rendering
> load off the compute.

---

## Nodes

### `cone_visualizer`

**File:** `fsai_visualisation/cone_visualizer.py`

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/cones` | Source `Cone3DArray` topic |
| `output_topic` | `/cone_markers` | Published `MarkerArray` for cones |
| `car_frame` | `Fr1A` | TF frame for the car mesh |
| `show_unknown` | `true` | Show LiDAR-only (uncoloured) cones — set `false` to display only camera-confirmed cones |

**Published topics:**

| Topic | Type | Description |
|---|---|---|
| `/cone_markers` | `visualization_msgs/MarkerArray` | Coloured cone meshes + text labels |
| `/car_marker` | `visualization_msgs/MarkerArray` | Race car body + 4 wheels at 10 Hz |

`visualisation.launch.py` runs **two** instances of this node: the primary
(`/cones` → `/cone_markers`) and a debug one (`lidar_cone_visualizer`,
`/perception/lidar/cones` → `/lidar_cone_markers`) whose `/car_marker` is remapped
away so the two don't collide.

---

## Meshes

All 3D assets live in `meshes/` and are installed automatically at build time.

```
meshes/
  cones/
    TrafficCone_Small_Blue.obj / .mtl     ← blue cone
    TrafficCone_Small_Yellow.obj / .mtl   ← yellow cone
    TrafficCone_Small_Orange.obj / .mtl   ← orange cone / unknown fallback
    TrafficCone_Large_Orange.obj / .mtl   ← large orange cone
  car/
    AFS_RaceCar_2024.obj / .mtl / .jpg   ← race car body
    AFS_RaceCar_2024_wheel.obj / .mtl    ← wheel (instanced at 4 positions)
```

Mesh URIs use `package://fsai_visualisation/meshes/...` so they resolve correctly on any machine that has the package installed — no hardcoded paths.

---

## RViz configuration

**File:** `config/fsai.rviz`

Full monitoring layout (✓ = enabled by default):

| Group | Display | Topic | On |
|---|---|---|:--:|
| Base | Grid | — | ✓ |
| Base | TF | — | ✓ |
| Perception | LiDAR Scan | `/perception/pointcloud` | ✓ |
| Perception | Cone Markers (fused) | `/cone_markers` | ✓ |
| Perception | **Raw LiDAR Cones** | `/lidar_cone_markers` | ✓ |
| Perception | YOLO Camera | `/perception/camera/image` | ✓ |
| Navigation | Active Path | `/nav/active_path` | ✓ |
| Navigation | Path Follower Debug | `/nav/local_path_follower_markers` | ✓ |
| Navigation | Perceived Path | `/nav/perceived_path_markers` | ✓ |
| Navigation | Fwd-Distance Debug | `/nav/forward_distance_controller_markers` | ☐ |
| Navigation | Skidpad Path | `/nav/skidpad_path_markers` | ☐ |
| Localization | Vehicle Odometry | `/odometry/vehicle` | ✓ |
| Car | Car Body | `/car_marker` | ✓ |

Mission-specific displays start disabled to reduce clutter — toggle them on for
the mission you're watching. Fixed frame: `Fr1A` (X-forward, Y-left, Z-up).

**Per-env configs:** the launch auto-selects `config/fsai_<env>.rviz` if it exists,
otherwise `config/fsai.rviz`. Because `fsai_perception`'s `bridge` normalizes topic
names, sim and real publish the *same* topics, so the single `fsai.rviz` serves
both — only add a `fsai_real.rviz` / `fsai_sim.rviz` if you want genuinely
divergent layouts.

**To add your own displays**, open `config/fsai.rviz` and add entries under the
`# ── ADD YOUR DISPLAYS HERE ───` comment near the bottom of the `Displays` list,
or add env-only debug nodes in `launch_setup()` in `visualisation.launch.py`.

---

## How to run

**Source the workspace first:**

```bash
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
```

**Launch the monitoring dashboard** (perception pipeline should be publishing —
locally or on the compute over the network):

```bash
ros2 launch fsai_visualisation visualisation.launch.py env:=real
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `env` | `sim` | `sim` \| `real` — selects `config/fsai_<env>.rviz` (if present) + env-specific debug nodes |
| `use_sim_time` | `true` | `true` for CarMaker or bag playback (`--clock`); **`false` for the LIVE real car** (otherwise RViz freezes waiting for `/clock`) |
| `rviz` | `true` | Launch RViz automatically |
| `rviz_config` | `''` | Override the `.rviz` path (default: auto-select `fsai_<env>.rviz`, else `fsai.rviz`) |
| `show_unknown` | `true` | Show LiDAR-only (uncoloured) cones on `/cone_markers` |

**Example — live real car:**

```bash
ros2 launch fsai_visualisation visualisation.launch.py env:=real use_sim_time:=false
```

### Running from a laptop (remote monitoring)

The dashboard is designed to run on a separate laptop while the stack runs on the
compute. On the laptop:

```bash
export ROS_DOMAIN_ID=42          # MUST match the compute (start_fsai.sh sets this)
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
ros2 launch fsai_visualisation visualisation.launch.py env:=real use_sim_time:=false
```

Requirements:

1. **Same `ROS_DOMAIN_ID` (42)** as the compute — the #1 reason you'd see nothing.
2. **Same DDS-reachable network** (same subnet / multicast, or matching discovery server).
3. **Workspace built on the laptop** — it needs `cone_visualizer` and the
   `fsai_interfaces` message types to convert cones and resolve topic types.

---

## Build

```bash
colcon build --packages-select fsai_visualisation --symlink-install
```

`--symlink-install` means edits to Python source take effect without rebuilding. Only rebuild if you add new mesh files or change `setup.py` / `package.xml`.

---

## Dependencies

| Package | Purpose |
|---|---|
| `rviz2` | 3D visualisation |
| `visualization_msgs` | `MarkerArray` / `Marker` message types |
| `fsai_interfaces` | `Cone3DArray` input message type |
| `ament_index_python` | Package share directory lookup for mesh URIs |
