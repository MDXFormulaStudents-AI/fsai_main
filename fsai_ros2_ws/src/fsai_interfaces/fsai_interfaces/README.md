# fsai_interfaces - Custom ROS2 Messages

This package defines custom message types for the Formula Student AI perception pipeline.

---

## What is this package?

`fsai_interfaces` is a **message-only package** that defines custom ROS2 message types used by the perception system. It doesn't contain any executable nodes - just message definitions.

**Think of it as:** A "data format library" that other packages use to communicate structured data.

---

## Why do we need custom messages?

Standard ROS2 messages (like `sensor_msgs`, `geometry_msgs`) don't have a format for:
- **3D cone positions** with class names and confidence scores
- **Multi-sensor fusion metadata** (camera vs LiDAR source)

So we created custom messages to represent our specific data structures.

---

## Message Definitions

### Cone3D.msg

Represents a single detected cone in 3D space.

```
std_msgs/Header header
geometry_msgs/Point position    # (x, y, z) in meters
string class_name                # blue_cone, yellow_cone, orange_cone
float32 confidence               # detection confidence [0.0, 1.0]
string source                    # "camera", "lidar", or "fused"
```

**Fields:**
- `header`: Timestamp and frame ID (e.g., "camera_frame")
- `position`: 3D coordinates (X=right, Y=down, Z=forward)
- `class_name`: Cone color/type
- `confidence`: How confident the detector is (0.0 to 1.0)
- `source`: Which sensor detected it (for fusion tracking)

### Cone3DArray.msg

An array of multiple 3D cone detections.

```
std_msgs/Header header
Cone3D[] cones
```

**Fields:**
- `header`: Timestamp and frame ID for the entire array
- `cones`: List of individual Cone3D detections

---

## How to Use

### View Message Definitions

```bash
source /home/mdxfsai/fsai_ws/install/setup.bash
ros2 interface show fsai_interfaces/msg/Cone3D
ros2 interface show fsai_interfaces/msg/Cone3DArray
```

### In Python Code

```python
from fsai_interfaces.msg import Cone3D, Cone3DArray

# Create a single cone
cone = Cone3D()
cone.position.x = 2.5
cone.position.y = 0.3
cone.position.z = 10.0
cone.class_name = "blue_cone"
cone.confidence = 0.87
cone.source = "camera"

# Create an array
cones_array = Cone3DArray()
cones_array.cones.append(cone)
```

### Subscribe to Topic

```bash
# View live 3D cone data
source /home/mdxfsai/fsai_ws/install/setup.bash
ros2 topic echo /cones_camera_frame
```

---

## Which Nodes Use These Messages?

### Publishers (Output)
- **Node 3 (`fsai_localizer_3d`)**: Publishes `Cone3DArray` to `/cones_camera_frame`

### Subscribers (Input)
- **Node 4 (`fsai_frame_transform`)**: Subscribes to `Cone3DArray` from `/cones_camera_frame`
- **Future nodes**: SLAM, path planning, etc.

---

## Building This Package

This package is built automatically with the perception workspace:

```bash
cd ~/fsai_ws
colcon build --packages-select fsai_interfaces
```

**Build output:**
- Message headers generated in `install/fsai_interfaces/include/`
- Python bindings in `install/fsai_interfaces/lib/python3.10/`

---

## Package Structure

```
fsai_interfaces/
├── msg/
│   ├── Cone3D.msg          # Single 3D cone definition
│   └── Cone3DArray.msg     # Array of 3D cones
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
└── README.md               # This file
```

---

## Dependencies

This package depends on:
- `std_msgs` (for Header)
- `geometry_msgs` (for Point)
- `rosidl_default_generators` (for message generation)

All dependencies are standard ROS2 packages.

---

## Future Extensions

As we add more features, we may add:
- `Cone2D.msg` - For 2D detections (if needed)
- `TrackState.msg` - For cone tracking over time
- `MapCone.msg` - For global map coordinates
- `LidarCone.msg` - For LiDAR-specific metadata

---

## Verification

Check that messages are properly generated:

```bash
source /home/mdxfsai/fsai_ws/install/setup.bash

# List all fsai_interfaces messages
ros2 interface list | grep fsai_interfaces

# Should show:
# fsai_interfaces/msg/Cone3D
# fsai_interfaces/msg/Cone3DArray
```

---

## Summary

✅ **What:** Custom message definitions for 3D cone data  
✅ **Why:** Standard ROS messages don't fit our needs  
✅ **Where:** Used by Node 3 (3D Localizer) and Node 4 (Frame Transform)  
✅ **How:** Import and use like any ROS2 message type  

This is a **foundational package** that enables communication between perception nodes!
