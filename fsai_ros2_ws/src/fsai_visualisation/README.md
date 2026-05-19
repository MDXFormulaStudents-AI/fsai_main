# fsai_visualisation

Visualisation package for the MDX Formula Student AI perception pipeline. Converts the `/cones` topic into coloured 3D cone mesh markers and renders a car model — all viewable live in RViz.

Run this alongside `fsai_perception` to get a full visual display.

---

## What it does

- Subscribes to `/cones` (`fsai_interfaces/Cone3DArray`) and publishes RViz mesh markers for each detected cone
- Fused cones (camera-confirmed colour) render at full brightness using embedded `.mtl` materials
- LiDAR-only cones (unknown colour) render as semi-transparent white-tinted meshes
- Publishes a static race car body + wheels marker at 10 Hz in the `Fr1A` frame
- Launches RViz with a pre-configured display (`config/fsai.rviz`)

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

Pre-configured displays:

| Display | Topic | Description |
|---|---|---|
| Grid | — | XY ground plane, 30 × 30 m |
| TF | — | All coordinate frames |
| LiDAR Scan | `/perception/pointcloud` | Raw point cloud (Z-coloured) |
| Cone Markers | `/cone_markers` | 3D cone meshes + labels |
| YOLO Camera | `/perception/camera/image` | Annotated camera feed |
| Car Body | `/car_marker` | Race car mesh |

Fixed frame: `Fr1A` (X-forward, Y-left, Z-up)

**To add your own displays**, open `config/fsai.rviz` and add entries under the `# ── ADD YOUR DISPLAYS HERE ───` comment at the bottom of the `Displays` list.

---

## How to run

**Source the workspace first:**

```bash
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
```

**Launch visualisation (requires perception pipeline already running):**

```bash
ros2 launch fsai_visualisation visualisation.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `rviz` | `true` | Launch RViz automatically |
| `show_unknown` | `true` | Show LiDAR-only cones |

**Example — hide unknown cones and skip RViz:**

```bash
ros2 launch fsai_visualisation visualisation.launch.py show_unknown:=false rviz:=false
```

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
