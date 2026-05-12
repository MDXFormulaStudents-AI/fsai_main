# FSAI ROS2 Exercises

An introduction to ROS2 nodes, topics and RViz2 using real autonomous-vehicle
sensor data from a Formula Student AI simulation.

---

## What you will build

| Task | Description | Key concept |
|------|-------------|-------------|
| 1 | Interact with ROS2 from the command line | Topics, nodes, CLI tools |
| 2 | Relay node — re-publish a topic under a new name | Subscriber + Publisher |
| 3 | Compute the distance to each detected cone | Message fields, Maths, MarkerArray |
| 4 | Filtered detection counter | Filtering, counting, String publisher |
| 5 | Live image overlay (HUD) in RViz2 | cv_bridge, OpenCV, multi-subscriber |

---

## Prerequisites

- ROS2


---

## Step 1 — Download the ROS2 bag

The bag file contains ~57 seconds of sensor data from the CarMaker simulation
(LiDAR, camera, cone detections, odometry).

**Download link (OneDrive):**

> [ROSBags on OneDrive](https://auth.openai.com/oauth/authorize?response_type=code&client_id=app_EMoamEEZ73f0CkXaXp7hrann&redirect_uri=http%3A%2F%2Flocalhost%3A1455%2Fauth%2Fcallback&scope=openid+profile+email+offline_access&code_challenge=o8E9HnS4IG1WNoNMaMSMms833KfRUH6pA19lvT7A-eU&code_challenge_method=S256&id_token_add_organizations=true&codex_cli_simplified_flow=true&state=GGey8qXolBpgvO8MBY_pQYHqlVUnruGqiUyOhTgSnq0&originator=opencode)

extract the downloaded folder to:

```
~/fsai_main/fsai_ros2_ws/simbag_mobile/
```



---

## Step 2 — Source the workspace

Open a terminal and run these two commands.  You will need to do this in
**every new terminal** you open.

```bash
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
```

> **Tip — save yourself typing:** add both lines to your `~/.bashrc` so they
> run automatically every time you open a terminal:
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> echo "source ~/fsai_main/fsai_ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## Step 3 — Build the workspace

You only need to do this once (or after editing a node).

```bash
cd ~/fsai_main/fsai_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 4 — Play the bag in a loop

Open a **dedicated terminal** for the bag.  Leave it running while you work
through the tasks.

```bash
# Source first (if you haven't added it to ~/.bashrc)
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash

# Play the bag on loop so data never stops
ros2 bag play ~/fsai_main/fsai_ros2_ws/simbag_mobile/ --loop
```

You should see the terminal printing message counts as the bag plays.

---

## Step 5 — Open RViz2 (optional but recommended)

Open a **second terminal**:

```bash
cd ~/fsai_main/fsai_ros2_ws
./Start_RViz.sh
```

This loads the pre-configured `carmaker_perception.rviz` profile which already
shows the LiDAR scan, cone markers and the YOLO annotated camera.

---

## Task 1 — CLI Interaction

> **No coding required.** Learn to explore a running ROS2 system from the terminal.

With the bag playing, open a new terminal (source it first) and try each
command below.  Write down your answers to the questions.

```bash
# List every active topic
ros2 topic list

# See all the data flowing on the odometry topic (Ctrl-C to stop)
ros2 topic echo /carmaker/odom

# How fast is the LiDAR publishing?
ros2 topic hz /lidar/pointcloud

# How much bandwidth does the camera use?
ros2 topic bw /camera/image_raw

# What message type does the cone detection topic carry?
ros2 topic info /detections/lidar

# Show the full structure of that message type
ros2 interface show fsai_interfaces/msg/Cone3DArray

# List all running nodes
ros2 node list
```

**Questions to answer:**

1. How many topics are published by the bag?
2. What is the approximate publish rate (Hz) of `/lidar/pointcloud`?
3. What fields does a single `Cone3D` message contain?
4. What is the `class_name` field used for?
5. What is the `confidence` field?  What range of values does it take?
6. Can you find a topic that publishes the car's position?
   What message type does it use?

---

## Task 2 — Relay Node

**File:** `exercise/task2_relay.py`

Subscribe to `/detections/lidar` and re-publish the exact same data on
`/student/cones`.

### Run it

```bash
ros2 run exercise task2_relay
```

### Verify

Open a second terminal and echo the new topic:

```bash
ros2 topic echo /student/cones
```

Compare with the original:

```bash
ros2 topic hz /detections/lidar
ros2 topic hz /student/cones
```

They should publish at the same rate.

### Your tasks (see TODOs in the file)

- **TODO-1** Read through the node and make sure you understand every line.
- **TODO-2** Change the output topic name to `/student/<your_name>/cones`.
- **TODO-3** Add a running total of cones relayed since node start.

---

## Task 3 — Distance to Each Cone

**File:** `exercise/task3_distance.py`

Compute the Euclidean distance from the sensor origin to every detected cone.
Publish the results as floating text labels in RViz2.

### Euclidean distance reminder

```
distance = sqrt(x² + y² + z²)
```

Each cone's position is in the `cone.position.x / .y / .z` fields.

### Run it

```bash
ros2 run exercise task3_distance
```

### Add to RViz2

1. In RViz2 click **Add** → **By topic** → `/student/cone_distance_markers` → **MarkerArray** → **OK**
2. Distance labels should appear above each cone in the 3-D view.

### Verify on the command line

```bash
ros2 topic echo /student/cone_distance_markers
```

### Your tasks (see TODOs in the file)

- **TODO-1** Understand the distance formula in `_euclidean()`.
- **TODO-2** Change the label format to two decimal places and add the colour, e.g. `"3.14 m  [yellow]"`.
- **TODO-3** Add a SPHERE marker underneath each label.
- **TODO-4** Match the text colour to the cone colour.

---

## Task 4 — Filtered Detection Counter

**File:** `exercise/task4_counter.py`

Filter cones by confidence threshold, count by colour and publish a
summary string.

### Run it

```bash
ros2 run exercise task4_counter
```

### Verify

```bash
ros2 topic echo /student/cone_counts
```

You will see output like:

```
data: 'blue: 2 | yellow: 3  (threshold=0.70)'
```

### Your tasks (see TODOs in the file)

- **TODO-1** Change `CONFIDENCE_THRESHOLD` to 0.5, 0.8, 0.95 and observe the effect.
- **TODO-2** Add a `total` field to the output string.
- **TODO-3** Add separate publishers per colour: `/student/count/yellow`, `/student/count/blue`, etc.
- **TODO-4** Track and log the highest cone count seen since node start.

---

## Task 5 — Image Overlay (HUD)

**File:** `exercise/task5_image_overlay.py`

Draw a heads-up display on the live camera image showing cone counts and
distances, then view it in RViz2.

### Run it

```bash
ros2 run exercise task5_overlay
```

### Add to RViz2

1. In RViz2 click **Add** → **By topic** → `/student/annotated_image` → **Image** → **OK**
2. A camera panel should appear showing the HUD overlay.

### What the HUD shows

```
Cones: Y=3  B=2
  [yellow  ]    4.2 m
  [yellow  ]    6.8 m
  [blue    ]    9.1 m
```

### Your tasks (see TODOs in the file)

- **TODO-1** Read and understand `_draw_hud()` line by line.
- **TODO-2** Replace the black background with a semi-transparent rectangle.
- **TODO-3** Add a large "nearest cone" line at the top of the HUD.
- **TODO-4** Colour each distance line to match the cone colour.
- **TODO-5** Add a red warning in the image centre when a cone is within 3 m.

---

## Quick reference — build and run

```bash
# Build after any code change
cd ~/fsai_main/fsai_ros2_ws
colcon build --symlink-install
source install/setup.bash

# Terminal 1 — bag on loop
ros2 bag play ~/fsai_main/fsai_ros2_ws/simbag_mobile/ --loop

# Terminal 2 — RViz2
cd ~/fsai_main/fsai_ros2_ws && ./Start_RViz.sh

# Terminal 3+ — one per task
ros2 run exercise task2_relay
ros2 run exercise task3_distance
ros2 run exercise task4_counter
ros2 run exercise task5_overlay
```

---

## Debugging tips

| Symptom | What to check |
|---------|---------------|
| `package not found` | Did you run `colcon build` and `source install/setup.bash`? |
| No data on topic | Is the bag still playing? Run `ros2 topic hz /detections/lidar` |
| RViz shows nothing | Check the **Fixed Frame** (top of Displays panel) — set it to `Fr1A` |
| Markers appear in wrong place | Check `frame_id` in the marker matches the Fixed Frame |
| Image display is black | Confirm the bag is playing and the encoding matches (`bgr8`) |
| `ModuleNotFoundError: cv_bridge` | Run `sudo apt install ros-humble-cv-bridge` |

---

## Topics used in these exercises

| Topic | Type | Description |
|-------|------|-------------|
| `/detections/lidar` | `fsai_interfaces/Cone3DArray` | LiDAR cone detections with position, colour, confidence |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw RGB camera feed |
| `/carmaker/odom` | `nav_msgs/Odometry` | Car position and velocity |
| `/lidar/pointcloud` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud |
| `/student/cones` | `fsai_interfaces/Cone3DArray` | Your relay output (Task 2) |
| `/student/cone_distance_markers` | `visualization_msgs/MarkerArray` | Your distance labels (Task 3) |
| `/student/cone_counts` | `std_msgs/String` | Your counter output (Task 4) |
| `/student/annotated_image` | `sensor_msgs/Image` | Your HUD image (Task 5) |
