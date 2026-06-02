# fsai_sensors

MDX FSAI sensor bringup package. Provides a single launch entry point for all onboard sensors with per-sensor enable/disable flags, config-driven parameters, and static TF transforms for each sensor's mounting position on the car.

## Sensors

### ZED2 Stereo Camera
- **Driver:** [zed-ros2-wrapper](./zed-ros2-wrapper) (Stereolabs, pinned to `humble-v4.2.5`)
- **Interfaces:** [zed-ros2-interfaces](./zed-ros2-interfaces) (pinned to `4.2.5`)
- **SDK:** ZED SDK 4.2.5 — installed at `/usr/local/zed/`
- **Connection:** USB 3.0 (must be USB 3.0 — blue port or USB-C)
- **Calibration file:** `SN25558638.conf` — already present at `/usr/local/zed/settings/` on the Jetson. If missing, download from `https://calib.stereolabs.com/?SN=25558638` and place it there manually.

#### Key topics published
| Topic | Type | Description |
|---|---|---|
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | Rectified RGB image |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Depth map |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` | IMU (accelerometer + gyroscope) |
| `/zed/zed_node/pose` | `geometry_msgs/PoseStamped` | Camera pose (when positional tracking enabled) |

---

### Velodyne VLP-16 LiDAR
- **Driver:** `ros-humble-velodyne` (apt, pre-installed on the Jetson)
- **Connection:** Ethernet — Jetson `eno1` port, static IP `192.168.1.77/24` (configured persistently via NetworkManager, profile `vlp16-static`)
- **LiDAR IP:** `192.168.1.201` (factory default)
- **Scan rate:** 10 Hz (600 RPM)
- **Current FOV:** 180° forward-facing (`view_direction: 0.0`, `view_width: π`)

#### Key topics published
| Topic | Type | Description |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Processed 3D point cloud (~10 Hz) |
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | Raw UDP packets from the sensor |

#### Network setup (persistent — survives reboots)
The static IP is managed by NetworkManager. To verify:
```bash
ip addr show eno1   # should show inet 192.168.1.77/24
ping 192.168.1.201  # should reply from LiDAR
```

If the profile is ever lost:
```bash
sudo nmcli connection add type ethernet ifname eno1 con-name vlp16-static \
  ipv4.method manual ipv4.addresses 192.168.1.77/24 ipv4.gateway "" \
  connection.autoconnect yes
sudo nmcli connection up vlp16-static
```

---

## Configuration

All tunable parameters live in `fsai_sensors_bringup/config/` — edit these files rather than the launch file.

### `config/vlp16.yaml`
VLP-16 driver and pointcloud parameters: device IP, port, RPM, range limits, FOV direction and width, calibration file path.

### `config/sensor_mounts.yaml`
Static TF transforms defining where each sensor is physically mounted on the car relative to `base_link`.

> **TODO:** All x/y/z values are currently placeholder zeros. Measure the actual mounting positions on the car and update this file. A rebuild is required after changes (`colcon build --packages-select fsai_sensors_bringup`).

`base_link` convention (ROS REP-105): origin at centre of rear axle at ground level, X forward, Y left, Z up.

---

## Building

```bash
cd ~/fsai_main/fsai_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fsai_sensors_bringup
source install/setup.bash
```

For a full workspace build:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **Note:** After editing any file in `config/`, a rebuild is required — config files are read from the installed share directory, not the source tree.

---

## Running

### All sensors
```bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
ros2 launch fsai_sensors_bringup sensors.launch.py
```

### LiDAR only (no camera connected)
```bash
ros2 launch fsai_sensors_bringup sensors.launch.py enable_camera:=false
```

### Camera only (no LiDAR connected)
```bash
ros2 launch fsai_sensors_bringup sensors.launch.py enable_lidar:=false
```

### Available launch arguments
| Argument | Default | Description |
|---|---|---|
| `enable_camera` | `true` | Launch ZED2 camera |
| `enable_lidar` | `true` | Launch Velodyne VLP-16 |
| `camera_model` | `zed2` | ZED camera model |
| `lidar_ip` | `192.168.1.201` | LiDAR IP address |

---

## Verifying sensors

### LiDAR
```bash
ros2 topic hz /velodyne_points        # expect ~10 Hz
ros2 topic echo /tf_static            # should show velodyne → base_link
```

### Camera
```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/image_rect_color
lsusb | grep 2b03 && ls /dev/video*   # confirm USB detection
```

### Visualise in RViz2
```bash
rviz2
```
- Set **Fixed Frame** to `base_link`
- Add → By topic → `/velodyne_points` → PointCloud2
- Add → By topic → `/zed/zed_node/rgb/image_rect_color` → Image

---

## Notes
- ZED2 requires USB 3.0 — USB 2.0 will fail silently
- The ZED SDK Python API (`pyzed`) failed to install due to root-owned numpy dist-info; this does not affect the ROS wrapper which uses the C++ SDK directly
- ZED SDK AI models (object detection, neural depth) were not installed; re-run the SDK installer with the AI option if needed
- VLP-16 is mounted upside down on the car — `roll: 3.14159` in `sensor_mounts.yaml` corrects the coordinate frame
