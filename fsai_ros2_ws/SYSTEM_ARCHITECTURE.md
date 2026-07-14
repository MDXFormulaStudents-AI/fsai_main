# MDX FSAI Autonomous Stack — System Architecture

**Middlesex University Formula Student AI — shared IMechE ADS-DV**

This document is the single top-level reference for the whole ROS 2 workspace
(`fsai_ros2_ws`). It explains what each package does, how the nodes interconnect,
how the stack talks to the ADS-DV vehicle over CAN, and how to run it in both
simulation and on the real car. Read this first; each package also has its own
`README.md` / `ARCHITECTURE.md` for detail.

---

## 1. What this system is

An autonomous driving stack that drives the **IMechE ADS-DV** (a shared Formula
Student AI vehicle) around the FS-AI dynamic events, and executes the static
inspections. We do **not** build or modify the car — our deliverable is entirely
software running on the on-board compute (**Jetson AGX Orin** with a **VLP-16
LiDAR** and **ZED2i camera**), talking to the vehicle's **VCU over CAN** via the
FS-AI API.

The seven competition missions the stack must implement:

| AMI | Mission             | Required behaviour                                                              |
| --: | ------------------- | ------------------------------------------------------------------------------- |
|   0 | NOT_SELECTED        | idle                                                                            |
|   1 | Acceleration        | 75 m straight, stop within 100 m, mission complete                              |
|   2 | Skidpad             | figure-8 (right ×2, left ×2), stop within 25 m                                |
|   3 | Autocross           | single lap, stop within 30 m                                                    |
|   4 | Trackdrive          | **10 laps (count them ourselves — no VCU lap signal)**, stop within 30 m |
|   5 | Static Inspection A | steering sweep, ramp to 200 rpm in 10 s, stop in 5 s, AS_FINISHED               |
|   6 | Static Inspection B | spin to 50 rpm, trigger EBS → AS_EMERGENCY                                     |
|   7 | Autonomous Demo     | steer sweep, drive 10 m, stop, drive 10 m, deploy EBS                           |

Hard rules baked into the design: handshake reply < 50 ms, actuator command every
10 ms (100 Hz), EBS on sensor loss, never command brake + torque simultaneously.

---

## 2. Top-level architecture

Data flows sensors → perception → planning → control, with **localization**
feeding a fixed frame, **mission control** selecting/gating the active pipeline,
and **the vehicle interface** as the only component that touches CAN.

```
  VLP-16 LiDAR    ZED2i camera
        │              │
        ▼              ▼
 ┌───────────────────────────────┐   fsai_sensors  (raw drivers + sensor TFs)
 │  pointcloud        images      │
 └───────┬───────────────┬────────┘
         ▼               ▼
 ┌───────────────────────────────┐   fsai_perception
 │ lidar_detector  camera_detector│   clusters + detects cones
 │            └── fusion ──────────┼─► /cones   (Cone3DArray, frame Fr1A, ~20 Hz)
 └───────────────────────────────┘
         │
         ▼
 ┌───────────────────────────────┐   fsai_navigation  (path generation + control)
 │ perceived_path / skidpad_path  │──► /nav/active_path (or /nav/perceived_path…)
 │ forward_distance_controller    │
 │ local_path_follower ───────────┼─► VehicleControl  → /carmaker/VehicleControl  (SIM)
 │                                │─► DriveCommand    → /dynamic_drive_command    (REAL, gated)
 └───────────────────────────────┘
         ▲  odom (twist + pose)           ▲ /mission/selected (self-gate)
         │                                │
 ┌───────────────┐              ┌────────────────────────────────┐
 │fsai_localization│            │ fsai_mission_control            │
 │ vehicle_odometry│            │ mission_manager (router/gate)   │
 │  wheels+IMU → Odometry       │ static_profile_executor         │
 │  + odom→home→base_link→Fr1A  │ dynamic_mission_executor (laps) │
 └───────────────┘              └───────────────┬────────────────┘
                                                │ /vehicle/drive_command
                                                │ /vehicle/mission_complete
                                                │ /vehicle/estop
                                                ▼
 ┌───────────────────────────────────────────────────────────────┐
 │ fsai_vehicle_interface  (C++, the ONLY CAN-aware node)          │
 │  - ADS-DV handshake state machine                               │
 │  - FS-AI API (vendored) over SocketCAN                          │
 │  - publishes /vcu/status, /vehicle/interface_state, /vcu/*      │
 └───────────────────────────────┬───────────────────────────────┘
                                  │ SocketCAN (can0 / vcan0), 100 Hz
                                  ▼
                            ADS-DV VCU
```

---

## 3. The CAN interface and the ADS-DV relationship

**`fsai_vehicle_interface` (C++) is the hardware abstraction layer and the only
node that knows the vehicle exists.** Every other node speaks ROS 2 topics and has
no knowledge of CAN, the VCU, or the ADS-DV state machine. It vendors the FS-AI C
API (`can.c`, `fs-ai_api.c`) and runs a deterministic 100 Hz loop.

**It owns the ADS-DV mission handshake** as an internal state machine:

```
WAIT_FOR_VCU → WAIT_FOR_MISSION → MISSION_SELECTED → WAIT_FOR_GO → DRIVING → FINISHING → FINISHED
Any state → EMERGENCY if AS_EMERGENCY_BRAKE
FINISHED / EMERGENCY → WAIT_FOR_MISSION after full VCU power cycle
```

It publishes the current state on `/vehicle/interface_state`. **`DRIVING` (state 4)
is the gate**: drive commands are only forwarded to CAN in that state.

**The topic contract** (what the stack must publish, what the interface publishes):

| Direction          | Topic                             | Type                               | Meaning                                                                                      |
| ------------------ | --------------------------------- | ---------------------------------- | -------------------------------------------------------------------------------------------- |
| stack → interface | `/vehicle/drive_command`        | `fsai_interfaces/DriveCommand`   | steer °, axle rpm, torque Nm, brake % (forwarded to CAN only in DRIVING, if fresh < 100 ms) |
| stack → interface | `/vehicle/mission_complete`     | `std_msgs/Bool`                  | publish`true` once → FINISHING → MISSION_FINISHED                                        |
| stack → interface | `/vehicle/estop`                | `std_msgs/Bool`                  | latched EBS (Static B, Demo, sensor-loss rule T4.2.2)                                        |
| interface → stack | `/vehicle/interface_state`      | `fsai_interfaces/InterfaceState` | the DRIVING gate                                                                             |
| interface → stack | `/vcu/status`                   | `fsai_interfaces/VcuStatus`      | **`ami_state`** (mission selected), `as_state`, RES GO, faults                     |
| interface → stack | `/vcu/wheel_speeds`             | `fsai_interfaces/WheelSpeeds`    | FL/FR/RL/RR rpm + pulse counts                                                               |
| interface → stack | `/vcu/imu`                      | `sensor_msgs/Imu`                | accel +**yaw rate** (`angular_velocity.z`)                                           |
| interface → stack | `/vcu/gps`                      | `sensor_msgs/NavSatFix`          | rover-only, ~3–5 m (no RTK)                                                                 |
| interface → stack | `/vcu/steering`, `/vcu/brake` | `Float32`, `Float32MultiArray` | actual values                                                                                |

Safety behaviours owned here (not in the stack): stale-command zeroing (100 ms),
brake/torque implausibility handling, EBS latch, 20 zero-frames on shutdown.
E-stop in software is **RES-physical only**; `ESTOP_REQUEST` is otherwise `NO`.

**Bench/HiL testing without the car:** `fsai_vehicle_interface/tools/vcu_simulator.py`
emulates the VCU over `vcan0` (menu-driven AMI selection + handshake stepping),
so the whole selection→drive→complete flow can be exercised with no hardware.

---

## 4. Package reference

### 4.1 `fsai_interfaces` — shared messages

Custom message definitions used across the stack. Build this first.

| Message                     | Purpose                                                                                                       |
| --------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `Cone3D`, `Cone3DArray` | perceived cones:`position`, `class_name` (colour), `confidence`, `source`                             |
| `DriveCommand`            | real-car actuator command:`steer_angle_deg` (+=left), `axle_speed_rpm`, `axle_torque_nm`, `brake_pct` |
| `VcuStatus`               | VCU status with`AMI_*` / `AS_*` constants                                                                 |
| `WheelSpeeds`             | four-wheel rpm + pulse counts                                                                                 |
| `InterfaceState`          | interface state-machine state with constants (`DRIVING = 4`)                                                |

### 4.2 `fsai_sensors` — raw sensor drivers

Brings up the physical sensors and their static TFs: the **VLP-16** (via
`velodyne_driver` + `velodyne_pointcloud`) producing a point cloud, and the
**ZED2i** camera. Publishes raw pointcloud + images + sensor-frame transforms
(`Lidar_F`, `Cam_F`, …). Launch: `fsai_sensors_bringup/launch/sensors.launch.py`.

### 4.3 `fsai_perception` — cones from sensors

| Node                | Role                                                                                                                                                                                                |
| ------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `lidar_detector`  | ground removal + Euclidean clustering of the point cloud → cone candidates (`/perception/lidar/cones`, frame `Lidar_F`)                                                                        |
| `camera_detector` | detects/colours cones in the ZED image →`/perception/camera/detections`                                                                                                                          |
| `fusion`          | fuses LiDAR geometry + camera colour →**`/cones`** (`Cone3DArray`, frame **`Fr1A`**, ~20 Hz). Camera colour is matched onto LiDAR clusters; unmatched/stale → `unknown_cone`. |
| `bridge`          | simulation cone bridge (CarMaker/CMRosIF sensor input path)                                                                                                                                         |

Launch: `perception.launch.py`. **`/cones` is the single perception output the
whole nav stack consumes** — identical topic/frame in sim and on the real car, so
nothing downstream changes between them.

### 4.4 `fsai_localization` — odometry + fixed frame

| Node                 | Role                                                                                                                                                                                                                                                                                                                                                                                        |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `vehicle_odometry` | **wheel + IMU dead-reckoning.** `v = mean rear-wheel rpm/60 × wheel_circumference`; `ω = /vcu/imu.angular_velocity.z` (stationary bias removed); integrates a 2D unicycle. Anchors the fixed frame at the **DRIVING** transition (so `odom` == Fr1A-at-run-start). Publishes `nav_msgs/Odometry` (pose in fixed frame, exact twist in body) on `/odometry/vehicle`. |

Launch: `vehicle_odometry.launch.py` — starts `vehicle_odometry` **and reuses
`carmaker_odom_tf`** (from `fsai_visualisation`) to build the
`odom → home → base_link → Fr1A` TF chain from that odom. On the real car this
node **replaces CarMaker's `/carmaker/odom`**; you point every consumer's
`odom_topic` at `/odometry/vehicle`. Most consumers need only **twist** (drift-free);
see §6.

### 4.5 `fsai_navigation` — path generation + vehicle control

The planning/control half. Two roles: *path generators* (produce a `nav_msgs/Path`)
and *followers* (turn a path or a distance target into actuator commands).

| Node                            | Mission(s)                     | Role                                                                                                                                                                                    |
| ------------------------------- | ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `perceived_path`              | autocross, trackdrive          | cone → midline planner (cost-based Delaunay or forward-pairs), short-lived cone tracker, ego-frame                                                                                     |
| `skidpad_path`                | skidpad                        | geometric figure-8 generator + state machine (orange crossing → right×2 → left×2 → exit → STOPPED)                                                                                |
| `forward_distance_controller` | acceleration                   | straight-line odometry distance controller (target 75 m) — needs no path                                                                                                               |
| `local_path_follower`         | skidpad, autocross, trackdrive | **pure-pursuit follower.** Outputs both sim `VehicleControl` and real `DriveCommand`.                                                                                         |
| *alternates*                  | —                             | `acceleration_nav`, `stanley_path_follower`, `pure_pursuit_path_follower`, `global_path_follower`, `ground_truth_path`, `steer_calibration` (sim plant-inverse measurement) |

Key follower behaviours:

- **Dual output.** `VehicleControl` (gas 0–1, `steer_ang` rad) for the CarMaker
  (CMRosIF) sim on `/carmaker/VehicleControl`; **`DriveCommand`** (steer °, axle
  rpm, torque, brake %) for the real car on `/dynamic_drive_command`, computed
  from the **pre-gain** road-wheel angle and a curvature-adjusted target speed.
- **Plant-inverse `steer_command_gain`.** The CMRosIF sim realises only ~0.146× of
  the commanded road-wheel angle, so the sim publishes `steer_ang × 6.85`. This is
  **simulation-only**; on the real car `steer_command_gain = 1.0` (and the
  `DriveCommand` uses the un-scaled angle regardless). The follower logs a loud
  warning whenever the gain ≠ 1.0.
- **Self-gating.** Path generators and followers subscribe to `/mission/selected`
  and only act for their mission (`mission_gates`), so the whole nav stack can run
  at once and only the selected pipeline commands the car. Gated generators publish
  to a shared `/nav/active_path` so **one** follower serves skidpad/autocross/trackdrive.

Launches: `skidpad.launch.py`, `perceived_local_path.launch.py`,
`local_path_follower.launch.py` (mostly sim/standalone development).

### 4.6 `fsai_mission_control` — mission routing + orchestration

Sits above the vehicle interface; **never touches CAN**. Decides which command
source may reach `/vehicle/drive_command`.

| Node                         | Role                                                                                                                                                                                                                                                            |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mission_manager`          | reads`/vcu/status.ami_state` → publishes **`/mission/selected`** (string); forwards the active candidate command to `/vehicle/drive_command` **only when the selected mission matches and interface is DRIVING**; forwards completion/e-stop |
| `static_profile_executor`  | runs deterministic YAML profiles for Static A/B + Autonomous Demo →`/static_drive_command`                                                                                                                                                                   |
| `dynamic_mission_executor` | for the dynamic missions:**counts laps** (orange start/finish gate, re-anchored each pass so it is drift-immune), aggregates completion (accel distance / skidpad STOPPED / lap target) → `/dynamic_mission_complete`                                  |

Routing table (candidate → forwarded, gated by mission + DRIVING):

| Candidate                                                   | Forwarded to                  | Missions                                     |
| ----------------------------------------------------------- | ----------------------------- | -------------------------------------------- |
| `/static_drive_command`                                   | `/vehicle/drive_command`    | static_inspection_a/b, autonomous_demo       |
| `/dynamic_drive_command`                                  | `/vehicle/drive_command`    | acceleration, skidpad, autocross, trackdrive |
| `/static_mission_complete`, `/dynamic_mission_complete` | `/vehicle/mission_complete` | (matching mission)                           |
| `/static_estop`                                           | `/vehicle/estop`            | static_inspection_b, autonomous_demo         |

Launch: **`mission_bringup.launch.py`** — the single entry point for **all 7
missions** (one `mission_manager` + `static_profile_executor` +
`dynamic_mission_executor` + the gated path generators + one follower +
`forward_distance_controller`). `mission_control.launch.py` remains as a
static-only convenience; **do not run it alongside `mission_bringup.launch.py`**,
as that would spawn a second `mission_manager`.

### 4.7 `fsai_vehicle_interface` — CAN HAL

See §3. C++, vendored FS-AI API, ADS-DV handshake, `/vcu/*` publication.
Launch: `vehicle_interface.launch.py can_interface:=can0|vcan0`.

### 4.8 `fsai_visualisation` — TF + RViz

| Node                 | Role                                                                                                                                                                                             |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `carmaker_odom_tf` | turns a`nav_msgs/Odometry` into the `odom → home → base_link → Fr1A` chain (`home` pinned at startup pose). **Reused by `fsai_localization` on the real car** despite the name. |
| `cone_visualizer`  | RViz markers for cones                                                                                                                                                                           |

Launches: `odom_forward_debug.launch.py`, `visualisation.launch.py`.
`rviz_config` holds the RViz layouts.

---

## 5. Frames / TF tree

```
odom ──► home ──► base_link ──► Fr1A ──► (cones, planning, control)
                        ├─► Lidar_F   (VLP-16 mount, static)
                        └─► Cam_F     (ZED2i mount, static)
```

- **`odom`** — continuous fixed frame; in sim from CarMaker, on the real car from
  `vehicle_odometry`. Drifts slowly (this is expected).
- **`home`** — the fixed frame pinned at the vehicle pose at start / DRIVING. This
  is the "global fixed frame at Fr1A-at-start" the stack plans against.
- **`base_link`** — the moving vehicle body.
- **`Fr1A`** — the CarMaker vehicle-reference origin used throughout the stack;
  `/cones` and ego-frame planning live here. Rear axle sits at `x = +0.5637 m` in
  Fr1A (the follower's `control_point_x_offset`); wheelbase 1.53 m.
- Sensor frames (`Lidar_F`, `Cam_F`) are static children of `base_link` from the
  vehicle datasheet mounting positions. On hardware, set the real
  `base_link → Fr1A` and sensor offsets (they are zeroed in sim).

**TF rule:** each frame has exactly one parent. Sensor frames are leaves and never
interfere with the `home`/`base_link` chain.

---

## 6. Mission flow (AMI → pipeline → completion)

1. Operator selects a mission on the ADS-DV touchscreen → VCU sets `ami_state`.
2. `fsai_vehicle_interface` publishes it on `/vcu/status`; runs the handshake to
   `DRIVING` when RES GO is pressed.
3. `mission_manager` maps `ami_state` → `/mission/selected` (string).
4. Every pipeline **self-gates** on `/mission/selected`; only the selected one acts:
   - **acceleration** → `forward_distance_controller` (75 m) → `DriveCommand`.
   - **skidpad** → `skidpad_path` → `/nav/active_path` → `local_path_follower`.
   - **autocross / trackdrive** → `perceived_path` → `/nav/active_path` → `local_path_follower`.
5. The active follower publishes `/dynamic_drive_command`; `mission_manager`
   forwards it to `/vehicle/drive_command` (only in DRIVING).
6. `dynamic_mission_executor` decides completion → `/dynamic_mission_complete`:
   - acceleration: 75 m reached + stopped;
   - skidpad: `skidpad_path` reaches STOPPED (`/skidpad/finished`);
   - autocross: 1 lap; trackdrive: 10 laps (orange gate crossings).
7. `mission_manager` forwards → `/vehicle/mission_complete` → interface FINISHING →
   VCU AS_FINISHED.

**Drift note:** most consumers need only **velocity** (drift-free): the follower
uses twist for its speed cap and achieved-curvature; `perceived_path` is ego-frame;
acceleration is straight-line wheel odometry (~1 %). The only long-horizon pose
consumer is lap counting, and it **re-anchors the orange gate every pass**, so
dead-reckoning heading drift never breaks it.

---

## 7. Simulation vs. real car (two integration modes)

The stack runs in two environments with the **same perception and planning**:

|                 | CarMaker (CMRosIF) sim                             | Real car / FS-AI CAN HiL                                                 |
| --------------- | -------------------------------------------------- | ------------------------------------------------------------------------ |
| Odometry        | `/carmaker/odom` (ground truth)                  | `vehicle_odometry` → `/odometry/vehicle`                            |
| Cones           | `bridge` / sim sensors → `/cones`             | VLP-16 + ZED2i → perception →`/cones`                                |
| Vehicle command | `VehicleControl` → `/carmaker/VehicleControl` | `DriveCommand` → mission_manager → `/vehicle/drive_command` → CAN |
| Steering        | `steer_command_gain := 6.85` (plant-inverse)     | `steer_command_gain := 1.0`                                            |
| Mission select  | publish`/vcu/status` or use `vcu_simulator`    | ADS-DV touchscreen (real VCU)                                            |

The single switch between them is **`odom_topic`** (which odom the nav nodes read)
plus `steer_command_gain`. Entry points and `/cones` are identical.

---

## 8. How to run

Build (interfaces first is handled automatically by colcon dependency order):

```bash
cd ~/FS-AI/fsai_main/fsai_ros2_ws
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 8.1 Real car / CAN HiL

```bash
# 1. CAN up (real: can2 @ 500k; HiL bench: vcan0 + vcu_simulator.py)
sudo ip link set can2 up type can bitrate 500000

# 2. Vehicle interface (CAN HAL + handshake + /vcu/*)
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can2

# 3. Sensors + perception (→ /cones)
ros2 launch fsai_sensors  sensors.launch.py
ros2 launch fsai_perception perception.launch.py

# 4. Localization (→ /odometry/vehicle + home/Fr1A TF)
ros2 launch fsai_localization vehicle_odometry.launch.py

# 5. Mission stack (ALL 7 missions), pointed at the real odom.
#    One launch handles static + dynamic; the operator picks the mission on the
#    ADS-DV touchscreen and the manager routes to the matching pipeline.
ros2 launch fsai_mission_control mission_bringup.launch.py \
  odom_topic:=/odometry/vehicle steer_command_gain:=1.0
```

### 8.2 CarMaker (CMRosIF) simulation

```bash
# odom TF chain from CarMaker
ros2 launch fsai_visualisation odom_forward_debug.launch.py
# perception bridge → /cones, then the mission stack with the sim plant-inverse
ros2 launch fsai_mission_control mission_bringup.launch.py \
  odom_topic:=/carmaker/odom steer_command_gain:=6.85
```

### 8.3 Bench test (no car, no sim)

```bash
sudo modprobe vcan; sudo ip link add dev vcan0 type vcan; sudo ip link set vcan0 up
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=vcan0
python3 src/fsai_vehicle_interface/tools/vcu_simulator.py   # select AMI, step to AS_DRIVING
# watch: /mission/selected, /vehicle/interface_state, /vehicle/drive_command
```

---

## 9. How to extend / integrate

- **New dynamic mission / controller.** Follow the documented pattern: the
  controller self-gates on `/mission/selected`, publishes a **candidate** command
  (`/dynamic_drive_command`), and `mission_manager` forwards it when the mission
  matches and DRIVING. Add completion detection to `dynamic_mission_executor`.
  Never publish directly to `/vehicle/drive_command`.
- **New static profile.** Drop a YAML in `fsai_mission_control/profiles/`
  (`static_profile_executor` loads any `*.yaml`).
- **Swap the odom source.** Publish `nav_msgs/Odometry` on a topic and point every
  node's `odom_topic` at it; the `home/Fr1A` TF builder consumes it unchanged.
- **New sensor/perception.** As long as it publishes `/cones` (`Cone3DArray` in
  `Fr1A`), the whole nav stack works with no changes.

---

## 10. Key design decisions & gotchas

- **One CAN owner.** Only `fsai_vehicle_interface` speaks CAN; everything else is
  ROS 2 topics. Do not bypass it.
- **DRIVING is the gate.** Nothing moves the car unless `interface_state == DRIVING`
  and commands are fresh (< 100 ms).
- **`steer_command_gain` is sim-only.** Default `1.0` (real, do-no-harm) in the
  launch files; sim opts into `6.85`. A loud startup warning fires when it ≠ 1.0.
- **`DriveCommand` ≠ gas pedal.** It is `axle_speed_rpm` (a speed *limit*) +
  `axle_torque_nm`, not a throttle. The follower maps its target speed to rpm;
  tune `drive_torque_nm` / `max_axle_rpm` on HiL. Never send torque + brake together.
- **Lap counting is perception-anchored**, not pure odometry — it survives 10 laps
  of dead-reckoning drift by re-pinning the orange gate each pass.
- **Trackdrive has no VCU lap signal** — the stack counts the 10 laps itself.
- **Steer sign** (`+= left`) and **wheel circumference** must be validated/calibrated
  against the real ADS-DV; the sim values are placeholders.

---

## 11. Package / launch / entry-point index

| Package                    | Language | Key executables                                                                                               | Launch files                                                                                 |
| -------------------------- | -------- | ------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `fsai_interfaces`        | msg      | —                                                                                                            | —                                                                                           |
| `fsai_sensors`           | py       | (velodyne/ZED drivers)                                                                                        | `sensors.launch.py`                                                                        |
| `fsai_perception`        | py       | `lidar_detector`, `camera_detector`, `fusion`, `bridge`                                               | `perception.launch.py`, `sensors.launch.py`                                              |
| `fsai_localization`      | py       | `vehicle_odometry`                                                                                          | `vehicle_odometry.launch.py`                                                               |
| `fsai_navigation`        | py       | `perceived_path`, `skidpad_path`, `local_path_follower`, `forward_distance_controller` (+ alternates) | `skidpad.launch.py`, `perceived_local_path.launch.py`, `local_path_follower.launch.py` |
| `fsai_mission_control`   | py       | `mission_manager`, `static_profile_executor`, `dynamic_mission_executor`                                | `mission_bringup.launch.py` (all 7), `mission_control.launch.py` (static-only)           |
| `fsai_vehicle_interface` | C++      | `vehicle_interface_node`                                                                                    | `vehicle_interface.launch.py`                                                              |
| `fsai_visualisation`     | py       | `carmaker_odom_tf`, `cone_visualizer`                                                                     | `odom_forward_debug.launch.py`, `visualisation.launch.py`                                |
| `rviz_config`            | cfg      | —                                                                                                            | —                                                                                           |
