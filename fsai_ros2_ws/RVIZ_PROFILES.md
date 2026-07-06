# RViz Profiles

This workspace now includes checked-in RViz profiles and a launcher script at the workspace root:

- `./Start_RViz.sh`
- `./src/rviz_config/carmaker_perception.rviz`
- `./src/rviz_config/sensors.rviz`
- `./src/rviz_config/odom_forward_debug.rviz`

## Why this is the best fit here

This workspace currently contains message packages, not a utility package with launch files. Because of that, the lightest and most portable setup is:

1. Save the RViz display layout in `.rviz` files.
2. Launch RViz with a small wrapper script.

That keeps the setup easy to use on any machine without creating a new ROS package just to open RViz.

## Profiles

`carmaker` (default)

- `/lidar/pointcloud`
- `/detections/lidar_markers`
- `/cone_markers`
- `/slam/markers`
- `/car_marker`
- `/detections_image`

`sensors`

- `/lidar/pointcloud`
- `/camera/image_raw`
- `/camera/depth`

`odom_forward`

- `/debug/carmaker_odom_tf_markers`
- `/debug/carmaker_odom_tf_path`
- `/nav/forward_distance_controller_markers`
- `/carmaker/ObjectList`
- `/perception/pointcloud`
- `/perception/camera/image`
- `/car_marker`

## Important note about `Cone3DArray`

This workspace documents `/cones_camera_frame` as `fsai_interfaces/Cone3DArray`, but stock RViz cannot render that custom message directly. The intended RViz path is to visualize a `visualization_msgs/MarkerArray` topic such as `/cone_markers` instead.

That matches the project references under `references/finalyearproject`, where a cone visualizer publishes marker topics for RViz.

## Usage

After sourcing ROS 2 and your workspace:

```bash
./Start_RViz.sh
```

Open the sensor-only profile:

```bash
./Start_RViz.sh sensors
```

Open the odom and forward-distance debug profile:

```bash
./Start_RViz.sh odom_forward
```

Or use the dedicated wrapper:

```bash
./Start_Odom_Forward_RViz.sh
```
