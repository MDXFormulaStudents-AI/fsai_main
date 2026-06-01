# fsai_sensors

MDX FSAI sensor bringup package. Provides a single launch entry point for all onboard sensors.

## Sensors

### ZED2 Stereo Camera
- **Driver:** [zed-ros2-wrapper](./zed-ros2-wrapper) (Stereolabs, pinned to `humble-v4.2.5`)
- **Interfaces:** [zed-ros2-interfaces](./zed-ros2-interfaces) (pinned to `4.2.5`)
- **SDK:** ZED SDK 4.2 — installed at `/usr/local/zed/`
- **Connection:** USB 3.0 (must be USB 3.0 — blue port or USB-C)

#### Key topics published
| Topic | Type | Description |
|---|---|---|
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | Rectified RGB image |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Depth map |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` | IMU (accelerometer + gyroscope) |
| `/zed/zed_node/pose` | `geometry_msgs/PoseStamped` | Camera pose (when positional tracking enabled) |

### LiDAR
Not yet fitted. Add driver package to `fsai_sensors/` and uncomment the LiDAR block in `launch/sensors.launch.py`.

---

## Building

```bash
cd ~/fsai_main/fsai_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

### All sensors (single command)
```bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
ros2 launch fsai_sensors_bringup sensors.launch.py
ros2 launch fsai_sensors_bringup sensors.launch.py
```

### ZED2 only
```bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### Verify camera topics are live
```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/image_rect_color
```

### Check camera is detected on USB
```bash
lsusb | grep 2b03 && ls /dev/video*
```

## Notes
- ZED2 requires USB 3.0 — plugging into a USB 2.0 port will fail silently
- The ZED SDK Python API (`pyzed`) failed to install due to root-owned numpy dist-info; this does not affect the ROS wrapper which uses the C++ SDK
- ZED SDK AI models were not installed (object detection / neural depth not available); re-run `/usr/local/zed/get_python_api.py` and the SDK installer with AI option if needed
