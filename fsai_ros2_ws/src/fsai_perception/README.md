# fsai_perception

Perception pipeline for the MDX Formula Student AI race car. Takes raw sensor data from either the CarMaker simulation or real hardware (VLP-16 LiDAR + ZED2 camera) and publishes a stream of detected, coloured cone positions that the navigation stack can use directly.

---

## What it does

The pipeline detects traffic cones from LiDAR and camera data and fuses them into a single output — each cone has a 3D position in the vehicle frame and a colour label (blue, yellow, orange, large orange, or unknown if the camera could not classify it).

**Input:** LiDAR point cloud + RGB camera image (sim or real — see Configuration)  
**Output:** `/cones` — `fsai_interfaces/Cone3DArray` in the vehicle frame (`Fr1A` in sim, `base_link` on real hardware)

The navigation team only needs `/cones`. Everything else is internal.

---

## Architecture

```
CarMaker Simulation
        │
        ▼
  ┌──────────┐
  │  bridge  │   Converts raw CarMaker formats to standard ROS2 types
  └──────────┘
    │         │
    ▼         ▼
/perception   /perception
/pointcloud   /image
    │              │
    ▼              ▼
┌──────────────┐  ┌─────────────────┐
│lidar_detector│  │ camera_detector │
└──────────────┘  └─────────────────┘
    │                     │
    │  /perception/        │  /perception/
    │  lidar/cones         │  camera/detections
    │                     │
    └──────────┬──────────┘
               ▼
          ┌─────────┐
          │  fusion │   Matches LiDAR positions to camera colour labels
          └─────────┘
               │
               ▼
           /cones
    (Cone3DArray, Fr1A frame)
```

All four nodes are launched together via `perception.launch.py`.

---

## Nodes

### `bridge`

**File:** `fsai_perception/bridge.py`

Normalises raw sensor topics into clean internal topics. Input topics and formats differ between sim and real — everything downstream of the bridge is identical in both cases.

**Simulation (CarMaker) — `pointcloud_format: v1`:**

| From | To | Conversion |
|---|---|---|
| `/carmaker/pointcloud` (PointCloud v1) | `/perception/pointcloud` (PointCloud2) | numpy vectorised, intensity preserved |
| `/front_camera_rgb/image_raw` | `/perception/image` | relay |
| `/front_camera_depth/image_raw` (mono16, cm) | `/perception/depth` (32FC1, metres) | divide by 100, clip at `depth_max_m` |

**Real hardware — `pointcloud_format: v2`:**

| From | To | Conversion |
|---|---|---|
| `/velodyne_points` (PointCloud2) | `/perception/pointcloud` (PointCloud2) | relay — already the right format |
| `/zed/zed_node/rgb/image_rect_color` | `/perception/image` | relay |
| `/zed/zed_node/depth/depth_registered` (32FC1, metres) | `/perception/depth` (32FC1, metres) | relay — already the right format |

The `pointcloud_format` parameter controls which subscription type is used — `v1` for CarMaker's legacy PointCloud, `v2` for the VLP-16's native PointCloud2. Switch it in `perception.yaml` alongside the topic names.

---

### `lidar_detector`

**File:** `fsai_perception/lidar_detector.py`

Clusters the point cloud into candidate cone positions using DBSCAN.

**Pipeline:**

1. **Range filter** — keep only points between `min_distance_m` and `max_distance_m`
2. **Ground removal** — estimate ground as the 10th-percentile Z value; strip everything within `ground_threshold_m` of it
3. **DBSCAN clustering** — cluster in XY only (`eps`, `min_points` from config); Z is ignored for clustering so the algorithm is not confused by vertical variation within a single cone
4. **Cluster validation** — reject clusters whose XY footprint falls outside `cone_width_min_m` / `cone_width_max_m`. Optional height and intensity filters can be enabled in config
5. **Split merged clusters** — if a cluster is too wide (`split_threshold_m`) it probably contains two merged cones; k-means (k=2) splits it
6. **Centroid** — mean of remaining points becomes the cone position; confidence scales with point count

Publishes `Cone3DArray` on `/perception/lidar/cones` in the `lidar_frame` (configurable — `Lidar_F` in sim, `velodyne` on real hardware). All cones are labelled `unknown_cone` at this stage — colour comes from the camera.

**Key config knobs** (in `perception.yaml`):

| Parameter | Default | What it controls |
|---|---|---|
| `eps` | 0.35 m | DBSCAN search radius — increase if cones are missed, decrease if walls/barriers cluster as cones |
| `min_points` | 2 | Minimum LiDAR returns to form a cluster — 2 is aggressive (catches far cones); raise to 4–5 to reduce false positives |
| `max_distance_m` | 30 m | How far ahead to detect |
| `use_intensity_filter` | true | Retro-reflective tape check — disable if intensity data is unavailable or unreliable |
| `ground_threshold_m` | 0.10 m | Height above ground to keep — increase on rough terrain |

---

### `camera_detector`

**File:** `fsai_perception/camera_detector.py`

Runs a YOLO model on the RGB image to classify cone colours.

- Model: `models/best.pt` (YOLOv8, trained on FS cone images)
- Output classes: `blue_cone`, `yellow_cone`, `orange_cone`, `large_orange_cone`
- Publishes 2D bounding boxes + class name + confidence as `vision_msgs/Detection2DArray` on `/perception/camera/detections`
- Also publishes an annotated image on `/perception/camera/image` for visualisation (can be disabled in config to save bandwidth)

The fusion node handles camera/LiDAR rate mismatch by caching the latest detection batch with a timestamp — stale detections (older than `camera_max_age_s`) are discarded. In sim, the CarMaker camera renders at ~3.3 Hz (limited by IPGMovie). On the ZED2 the camera runs at its configured frame rate.

---

### `fusion`

**File:** `fsai_perception/fusion.py`

Runs at LiDAR rate (20 Hz). On each LiDAR frame:

1. **TF lookup** — transforms cone positions from `lidar_frame` → `output_frame` via TF2
2. **Project to image** — each cone centroid is projected into the camera image plane using intrinsics from the `camera_info` topic and extrinsics from a TF2 lookup (`output_frame` → `camera_frame`). Both are populated lazily on the first available message/transform — frames are skipped until both are ready
3. **Hungarian matching** — assigns each projected LiDAR cone to the nearest YOLO bounding box it falls inside (scipy `linear_sum_assignment`). Confidence-weighted cost discourages matching to low-confidence detections
4. **Colour assignment** — matched cones get the YOLO class name and confidence; unmatched cones stay as `unknown_cone` with the LiDAR confidence
5. **Age check** — if the newest camera frame is older than `camera_max_age_s` (default 1 s), all cones are published as `unknown_cone` for that LiDAR frame

Publishes `Cone3DArray` on `/cones` in `output_frame`. Z is set to 0 (ground plane) — navigation uses the XY position only.

**Camera projection model:**

```
output_frame position → subtract camera position (from TF) → rotate into camera body frame (from TF)
→ convert to optical frame (X-right, Y-down, Z-forward)
→ apply pinhole projection: u = fx * X/Z + cx,  v = fy * Y/Z + cy
```

Intrinsics (fx, fy, cx, cy) come from the `camera_info_topic`. The camera body → optical frame axis swap is: `X_opt = -Y_body`, `Y_opt = -Z_body`, `Z_opt = X_body`.

---

## Configuration

All tunable parameters are in `config/perception.yaml`. You should not need to change source code to tune the detector or switch environments.

```
config/
  perception.yaml   # all node parameters — sim and real blocks at the top, shared tuning below
```

### Switching between sim and real hardware

`perception.yaml` has two clearly marked blocks at the top — one for CarMaker simulation (active by default) and one for real hardware (VLP-16 + ZED2, commented out). To switch, comment out the sim block and uncomment the real block:

```yaml
# SIMULATION (active)          →  comment this out
bridge:
  pointcloud_in: /carmaker/pointcloud
  ...
fusion:
  lidar_frame:  Lidar_F
  camera_frame: Cam_F
  ...

# REAL HARDWARE (commented)    →  uncomment this
# bridge:
#   pointcloud_in: /velodyne_points
#   ...
# fusion:
#   lidar_frame:  velodyne
#   camera_frame: zed_camera_center
#   ...
```

Sensor mounting positions (for the TF tree) live in `fsai_sensors/fsai_sensors_bringup/config/sensor_mounts.yaml` — that is the single source of truth for where sensors are physically mounted. The perception pipeline reads those positions via TF2 at runtime.

---

## Topics

### Inputs

**Simulation (CarMaker):**

| Topic | Type | Source |
|---|---|---|
| `/carmaker/pointcloud` | `sensor_msgs/PointCloud` | CarMaker LiDAR (`Lidar_F`) |
| `/front_camera_rgb/image_raw` | `sensor_msgs/Image` | CarMaker RGB camera |
| `/front_camera_rgb/camera_info` | `sensor_msgs/CameraInfo` | CarMaker camera intrinsics |
| `/front_camera_depth/image_raw` | `sensor_msgs/Image` | CarMaker depth camera |
| `/tf_static` | TF2 | CarMaker (`Fr1A → Lidar_F`, `Fr1A → Cam_F`) |

**Real hardware (VLP-16 + ZED2):**

| Topic | Type | Source |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyne VLP-16 |
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | ZED2 RGB camera |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | ZED2 factory intrinsics |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | ZED2 depth |
| `/tf_static` | TF2 | `fsai_sensors_bringup` (`base_link → velodyne`, `base_link → zed_camera_center`) |

### Outputs (for other teams)

| Topic | Type | Description |
|---|---|---|
| `/cones` | `fsai_interfaces/Cone3DArray` | **Main output** — fused, coloured cones in `output_frame` (`Fr1A` in sim, `base_link` on real hardware) |

### Internal topics (between perception nodes)

| Topic | Description |
|---|---|
| `/perception/pointcloud` | PointCloud2, `lidar_frame` (`Lidar_F` in sim, `velodyne` on real hardware) |
| `/perception/image` | RGB image relay |
| `/perception/depth` | Depth in metres |
| `/perception/lidar/cones` | Raw LiDAR detections before colour fusion |
| `/perception/camera/detections` | YOLO bounding boxes + class names |
| `/perception/camera/image` | YOLO-annotated image (for debugging) |

---

## How to run

**Source the workspace first** (until this is added to `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
```

**Launch the full pipeline:**

```bash
ros2 launch fsai_perception perception.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `device` | `cuda:0` | YOLO inference device — use `cpu` if CUDA is unavailable |
| `use_sim_time` | `true` | Use CarMaker simulation clock. **Set to `false` on real hardware** — with `true`, nodes wait for `/clock` from CarMaker and will freeze if it's not running |
| `visualize` | `false` | Launch RViz (use `fsai_visualisation` instead for full viz) |

**Example — sim (default):**

```bash
ros2 launch fsai_perception perception.launch.py
```

**Example — real hardware:**

```bash
ros2 launch fsai_perception perception.launch.py use_sim_time:=false device:=cuda:0
```

---

## Build

From the workspace root:

```bash
colcon build --packages-select fsai_perception --symlink-install
```

`--symlink-install` means Python file edits take effect immediately without a rebuild. Only rebuild if you change `setup.py`, `package.xml`, or add new config/model files.

---

## Dependencies

| Package | Purpose |
|---|---|
| `sensor_msgs_py` | Efficient PointCloud2 read/write |
| `sklearn` (scikit-learn) | DBSCAN and KMeans clustering |
| `ultralytics` | YOLO inference |
| `cv_bridge` | ROS Image ↔ OpenCV conversion |
| `scipy` | Hungarian algorithm (`linear_sum_assignment`) and rotation maths |
| `tf2_ros` | Coordinate frame transforms |
| `fsai_interfaces` | `Cone3D` and `Cone3DArray` message types |