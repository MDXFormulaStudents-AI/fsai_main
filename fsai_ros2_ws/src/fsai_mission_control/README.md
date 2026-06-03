# fsai_mission_control

ROS 2 mission supervision and deterministic static-inspection execution for the MDX Formula Student AI stack.

This package sits above `fsai_vehicle_interface`. It does not talk to CAN, it does not implement the ADS-DV handshake directly, and it does not replace the vehicle-interface state machine. Instead, it decides which mission-specific command source is allowed to reach `/vehicle/drive_command`, and it provides deterministic command profiles for Static Inspection A, Static Inspection B, and Autonomous Demo.

The intended architecture is:

```text
+--------------------------------------------------------------+
| Mission-specific command sources                             |
|                                                              |
| Static profiles  | future dynamic controller | future tests   |
| /static_drive_*  | /dynamic_drive_command    | candidate cmds |
+-------------------------------+------------------------------+
                                |
                                | candidate ROS 2 topics
+-------------------------------v------------------------------+
| fsai_mission_control                                         |
|                                                              |
| MissionManager                                               |
| - reads VCU AMI mission selection                            |
| - reads vehicle-interface state                              |
| - publishes /mission/selected                                |
| - forwards only the active mission command source            |
|                                                              |
| StaticProfileExecutor                                        |
| - loads YAML profiles for Static A/B and Autonomous Demo     |
| - publishes candidate static commands only                   |
+-------------------------------+------------------------------+
                                |
                                | /vehicle/drive_command
                                | /vehicle/mission_complete
                                | /vehicle/estop
+-------------------------------v------------------------------+
| fsai_vehicle_interface                                       |
|                                                              |
| - owns CAN and the FS-AI API                                 |
| - owns ADS-DV handshake state machine                        |
| - clamps/stales drive commands                               |
| - sends AI2VCU frames                                        |
+-------------------------------v------------------------------+
                                |
                                | SocketCAN can0 / can2 / vcan0
+-------------------------------v------------------------------+
| ADS-DV VCU                                                   |
+--------------------------------------------------------------+
```

The key design rule is: `fsai_vehicle_interface` remains the hardware abstraction layer, while `fsai_mission_control` handles mission-level routing and deterministic static tasks.

---

## Contents

- [Purpose](#purpose)
- [Why This Package Exists](#why-this-package-exists)
- [Repository Layout](#repository-layout)
- [Nodes](#nodes)
- [Topics Reference](#topics-reference)
- [Parameters](#parameters)
- [Static Inspection Profiles](#static-inspection-profiles)
- [State Machine Interaction](#state-machine-interaction)
- [Build](#build)
- [Run](#run)
- [Usage Flow on the Vehicle](#usage-flow-on-the-vehicle)
- [Bench Testing Without the Vehicle](#bench-testing-without-the-vehicle)
- [Extending for Dynamic Missions](#extending-for-dynamic-missions)
- [Safety Notes](#safety-notes)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)

---

## Purpose

`fsai_mission_control` provides two responsibilities:

1. Mission routing: decide which command source is allowed to command the vehicle.
2. Static inspection and Autonomous Demo execution: generate deterministic commands for Static Inspection A, Static Inspection B, and Autonomous Demo.

The package contains two ROS 2 nodes:

| Node | Executable | Responsibility |
|---|---|---|
| `MissionManager` | `mission_manager` | Converts VCU AMI selection into `/mission/selected`, gates command forwarding, and forwards completion/e-stop requests to `fsai_vehicle_interface`. |
| `StaticProfileExecutor` | `static_profile_executor` | Loads YAML command profiles for Static A, Static B, and Autonomous Demo, and publishes candidate static command topics. |

The package deliberately publishes static commands to candidate topics first. The static executor does not publish directly to `/vehicle/drive_command`. This prevents multiple nodes from independently commanding the vehicle.

---

## Why This Package Exists

The ADS-DV stack has several different command-producing modes:

- Static Inspection A.
- Static Inspection B.
- Autonomous Demo.
- Acceleration.
- Skidpad.
- Autocross.
- Trackdrive.
- Bench/HIL test utilities.

Only one source should be allowed to command `/vehicle/drive_command` at a time. This package introduces that arbitration point.

Current behaviour:

- Static A, Static B, and Autonomous Demo are supported.
- Commands are forwarded only when the VCU-selected AMI mission matches the active profile and `fsai_vehicle_interface` reports `InterfaceState.DRIVING`.
- Static mission completion is forwarded to `/vehicle/mission_complete`.
- Static B and Autonomous Demo e-stop requests are forwarded to `/vehicle/estop`.

Future behaviour should follow the same pattern for dynamic missions.

---

## Repository Layout

```text
fsai_mission_control/
+-- fsai_mission_control/
|   +-- __init__.py
|   +-- mission_manager.py             # Mission selection and command routing node
|   +-- static_profile_executor.py     # YAML static profile executor node
+-- launch/
|   +-- mission_control.launch.py      # Launches both nodes
+-- profiles/
|   +-- static_inspection_a.yaml       # Static A deterministic command profile
|   +-- static_inspection_b.yaml       # Static B deterministic command profile
|   +-- autonomous_demo.yaml           # Autonomous Demo deterministic command profile
+-- resource/
|   +-- fsai_mission_control           # ament package marker
+-- package.xml
+-- setup.cfg
+-- setup.py
+-- README.md
```

---

## Nodes

### `mission_manager`

`mission_manager` is the supervisor/router. It watches the VCU mission selection and the current vehicle-interface state, then decides what may be forwarded to the real vehicle command topics.

Responsibilities:

- Subscribe to `/vcu/status` from `fsai_vehicle_interface`.
- Convert `VcuStatus.ami_state` into a readable mission string.
- Publish that string on `/mission/selected`.
- Subscribe to static candidate topics.
- Forward static drive commands to `/vehicle/drive_command` only when Static A, Static B, or Autonomous Demo is selected and the vehicle interface is `DRIVING`.
- Forward Static A mission completion to `/vehicle/mission_complete`.
- Forward Static B and Autonomous Demo e-stop requests to `/vehicle/estop`.

Mission-name mapping:

| VCU AMI constant | `/mission/selected` string |
|---|---|
| `AMI_NOT_SELECTED` | `none` |
| `AMI_ACCELERATION` | `acceleration` |
| `AMI_SKIDPAD` | `skidpad` |
| `AMI_AUTOCROSS` | `autocross` |
| `AMI_TRACK_DRIVE` | `trackdrive` |
| `AMI_STATIC_INSPECTION_A` | `static_inspection_a` |
| `AMI_STATIC_INSPECTION_B` | `static_inspection_b` |
| `AMI_AUTONOMOUS_DEMO` | `autonomous_demo` |
| Unknown value | `unknown` |

Forwarding rules:

| Input | Forwarded output | Required selected mission | Required interface state |
|---|---|---|---|
| `/static_drive_command` | `/vehicle/drive_command` | `static_inspection_a`, `static_inspection_b`, or `autonomous_demo` | `DRIVING` |
| `/static_mission_complete` | `/vehicle/mission_complete` | `static_inspection_a`, `static_inspection_b`, or `autonomous_demo` | Any |
| `/static_estop` | `/vehicle/estop` | `static_inspection_b` or `autonomous_demo` | Any |

The manager does not modify drive commands. It only routes them. Command clamping, command timeout handling, and CAN transmission remain inside `fsai_vehicle_interface`.

### `static_profile_executor`

`static_profile_executor` runs deterministic command profiles from YAML.

Responsibilities:

- Subscribe to `/mission/selected`.
- Load the matching YAML profile when Static A, Static B, or Autonomous Demo is selected.
- Wait until `/vehicle/interface_state` reports `DRIVING` before starting the active profile.
- Publish candidate commands on `/static_drive_command` at the configured publish rate.
- Publish `/static_mission_complete` when a mission-complete profile finishes.
- Publish `/static_estop` when an e-stop profile finishes.
- For Static A, wait for wheel speeds to drop below the configured stopped threshold before publishing mission complete.
- For Autonomous Demo distance steps, integrate rear wheel odometry using the configured `wheel_circumference_m` to measure distance travelled.

Important behaviour:

- The executor is intentionally not connected directly to `/vehicle/drive_command`.
- If the interface leaves `DRIVING` while a profile is running, the profile resets.
- When `DRIVING` returns, the active profile starts again from the beginning.
- If the selected mission changes, the executor resets its current profile state.
- If no supported mission is selected, the executor stays idle.

---

## Topics Reference

### `mission_manager` subscriptions

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/vcu/status` | `fsai_interfaces/msg/VcuStatus` | `fsai_vehicle_interface` | Reads `ami_state` to know which mission the operator selected on the VCU touchscreen. |
| `/vehicle/interface_state` | `fsai_interfaces/msg/InterfaceState` | `fsai_vehicle_interface` | Reads whether the vehicle interface is currently in `DRIVING`. |
| `/static_drive_command` | `fsai_interfaces/msg/DriveCommand` | `static_profile_executor` | Candidate static actuator command. |
| `/static_mission_complete` | `std_msgs/msg/Bool` | `static_profile_executor` | Candidate mission-complete request. |
| `/static_estop` | `std_msgs/msg/Bool` | `static_profile_executor` | Candidate software e-stop request for Static B and Autonomous Demo. |

### `mission_manager` publications

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/mission/selected` | `std_msgs/msg/String` | Mission-specific nodes | Software routing topic derived from VCU AMI selection. |
| `/vehicle/drive_command` | `fsai_interfaces/msg/DriveCommand` | `fsai_vehicle_interface` | Final gated drive command. |
| `/vehicle/mission_complete` | `std_msgs/msg/Bool` | `fsai_vehicle_interface` | Final mission-complete signal. |
| `/vehicle/estop` | `std_msgs/msg/Bool` | `fsai_vehicle_interface` | Final software e-stop signal. |

### `static_profile_executor` subscriptions

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/mission/selected` | `std_msgs/msg/String` | `mission_manager` | Selects and loads the active profile. |
| `/vehicle/interface_state` | `fsai_interfaces/msg/InterfaceState` | `fsai_vehicle_interface` | Starts or pauses profile execution based on `DRIVING`. |
| `/vcu/wheel_speeds` | `fsai_interfaces/msg/WheelSpeeds` | `fsai_vehicle_interface` | Used by Static A to confirm the vehicle has stopped before mission complete. Used by Autonomous Demo distance steps for odometry. |

### `static_profile_executor` publications

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/static_drive_command` | `fsai_interfaces/msg/DriveCommand` | `mission_manager` | Candidate static command. |
| `/static_mission_complete` | `std_msgs/msg/Bool` | `mission_manager` | Candidate mission-complete request. |
| `/static_estop` | `std_msgs/msg/Bool` | `mission_manager` | Candidate e-stop request. |

---

## Parameters

### `mission_manager`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | `double` | `10.0` | Rate used to republish `/mission/selected`. Clamped to at least `1.0`. |

### `static_profile_executor`

| Parameter | Type | Default (launch) | Description |
|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | Rate used to evaluate the profile and publish `/static_drive_command`. Clamped to at least `1.0`. |
| `profile_directory` | `string` | `""` | Optional directory containing profile YAML files. Empty means use the installed package share directory. |
| `wheel_circumference_m` | `double` | `1.674` | Rear wheel circumference in metres used for distance step odometry. Default is calculated for Toyo Proxes R888R 225/45 R13 tyres. Tune if the car consistently over- or undershoots distance steps. |

The default profile directory resolves to:

```text
<workspace>/install/fsai_mission_control/share/fsai_mission_control/profiles
```

---

## Static Inspection Profiles

Profiles are YAML files stored in `profiles/` and installed into the package share directory. Any `*.yaml` file added to `profiles/` is automatically included at build time.

### Profile fields

| Field | Required | Description |
|---|---|---|
| `mission` | Yes | Human-readable profile mission name. |
| `completion_action` | Yes | Either `mission_complete` or `estop`. |
| `require_stopped_before_completion` | Optional | If `true`, wait for wheel speeds below threshold before mission complete. Used by Static A. |
| `stopped_rpm_threshold` | Optional | Wheel-speed threshold in rpm. Defaults to `10.0`. |
| `steps` | Yes | Ordered list of deterministic command steps. |

### Step fields

| Field | Description |
|---|---|
| `name` | Descriptive step name. |
| `type` | `hold`, `ramp`, or `distance`. |
| `duration_s` | Step duration in seconds. Required for `hold` and `ramp`. |
| `distance_m` | Target distance in metres. Required for `distance` steps. Uses rear-wheel odometry. |
| `command` | Command dict used by `hold` and `distance` steps. |
| `start` | Start command dict used by `ramp` steps. |
| `end` | End command dict used by `ramp` steps. |

Command fields match `fsai_interfaces/msg/DriveCommand`:

| Field | Unit | Notes |
|---|---|---|
| `steer_angle_deg` | degrees | Positive means left. ADS-DV range is `-21` to `+21` degrees. |
| `axle_speed_rpm` | rpm | Non-negative axle speed limit. |
| `axle_torque_nm` | Nm | Non-negative axle torque request. Zeroed automatically if `brake_pct > 0`. |
| `brake_pct` | percent | Clamped to `0.0`–`100.0`. If greater than zero, the executor zeros torque before publishing. |

---

### Static Inspection A

File: `profiles/static_inspection_a.yaml`

Requirements (FS-AI 2026 Rules IN2.2):
- Full steering sweep.
- Ramp axle speed to 200 rpm in 10 s.
- Stop within 5 s.
- Declare mission complete → VCU enters AS_FINISHED.

| Step | Type | Duration | Command behaviour |
|---|---|---|---|
| `centre_steering` | `hold` | `1.0 s` | Hold steering centred, vehicle stationary. |
| `sweep_to_left_lock` | `ramp` | `2.0 s` | Ramp steering from `0` to `+21 deg`. |
| `sweep_to_right_lock` | `ramp` | `4.0 s` | Ramp steering from `+21` to `-21 deg`. |
| `return_steering_to_centre` | `ramp` | `2.0 s` | Ramp steering from `-21` to `0 deg`. |
| `ramp_to_200rpm` | `ramp` | `10.0 s` | Ramp axle speed `0→200 rpm` with `50 Nm` torque. |
| `stop_with_brake` | `hold` | `5.0 s` | `0 rpm`, `0 Nm`, `20%` brake. |

Completion: `mission_complete`. Waits for all wheel speeds ≤ 10 rpm before publishing.

---

### Static Inspection B

File: `profiles/static_inspection_b.yaml`

Requirements (FS-AI 2026 Rules IN2.2):
- Spin drivetrain to 50 rpm.
- Vehicle (software) triggers EBS → VCU enters AS_EMERGENCY_BRAKE.

| Step | Type | Duration | Command behaviour |
|---|---|---|---|
| `ramp_to_50rpm` | `ramp` | `3.0 s` | Ramp axle speed `0→50 rpm` with `8 Nm` torque. |
| `hold_50rpm` | `hold` | `2.0 s` | Hold `50 rpm` with `8 Nm` torque. |

Completion: `estop`. Publishes `/static_estop` → forwarded to `/vehicle/estop`.

---

### Autonomous Demo

File: `profiles/autonomous_demo.yaml`

Requirements (FS-AI 2026 Rules IN2.3):
- Sweep steering left and right, return to straight.
- Accelerate for at least 10 m.
- Stop within a further 10 m.
- Accelerate for a further 10 m.
- Deploy EBS.

| Step | Type | Exit condition | Command behaviour |
|---|---|---|---|
| `steer_sweep_left` | `ramp` | `2.0 s` | Steer `0→+21 deg`, light brake holds car stationary. |
| `steer_sweep_right` | `ramp` | `4.0 s` | Steer `+21→-21 deg`. |
| `return_to_centre` | `ramp` | `2.0 s` | Steer `-21→0 deg`. |
| `drive_first_leg` | `distance` | `12.0 m` | `150 rpm` limit, `20 Nm`. Exits when rear-wheel odometry reaches 12 m. |
| `stop_mid` | `hold` | `5.0 s` | `0 rpm`, `30%` brake. |
| `drive_second_leg` | `distance` | `12.0 m` | `150 rpm` limit, `20 Nm`. Exits when rear-wheel odometry reaches 12 m. |

Completion: `estop`. Publishes `/static_estop` → forwarded to `/vehicle/estop`.

Tuning notes:
- 12 m gives a margin over the 10 m rule minimum. Adjust if needed.
- `wheel_circumference_m` (default `1.674 m` for Toyo 225/45 R13) must match the fitted tyres. If the car consistently over- or undershoots, adjust this parameter in the launch file.
- The `stop_mid` brake duration assumes a conservative approach speed. If `axle_torque_nm` is increased, extend `duration_s` proportionally.

---

## State Machine Interaction

### Vehicle-interface state machine

`fsai_vehicle_interface` owns the real ADS-DV handshake. It publishes the current state on `/vehicle/interface_state`.

```text
WAIT_FOR_VCU → WAIT_FOR_MISSION → MISSION_SELECTED → WAIT_FOR_GO → DRIVING → FINISHING → FINISHED
Any state → EMERGENCY if VCU enters AS_EMERGENCY_BRAKE
FINISHED / EMERGENCY → WAIT_FOR_MISSION after VCU power cycle (LV Master + ASMS + TSMS off then on)
```

`mission_manager` only forwards drive commands when the interface state equals `DRIVING`.

### Mission-control routing

`MissionManager` derives routing from `/vcu/status.ami_state`:

```text
AMI_NOT_SELECTED        → /mission/selected = none          → no forwarding
AMI_STATIC_INSPECTION_A → /mission/selected = static_inspection_a
AMI_STATIC_INSPECTION_B → /mission/selected = static_inspection_b
AMI_AUTONOMOUS_DEMO     → /mission/selected = autonomous_demo
Other AMI states        → matching mission string, no forwarding yet
```

For Static A, Static B, and Autonomous Demo, drive commands are forwarded only when the interface is `DRIVING`.

### Static profile lifecycle

```text
IDLE
  No Static A/B/Demo mission selected.

LOAD_PROFILE
  /mission/selected changed to a supported mission.
  The matching YAML file is loaded.

ARMED_WAITING_FOR_DRIVING
  Profile loaded, /vehicle/interface_state is not DRIVING.
  No commands published.

EXECUTING
  Interface state is DRIVING. Profile starts.
  Time-based steps exit when elapsed_in_step >= duration_s.
  Distance steps exit when rear-wheel odometry >= distance_m.
  Step transitions are logged.

WAITING_FOR_STOP
  Static A only. Profile steps complete, wheel speeds above threshold.
  Last command continues publishing.

COMPLETE
  Static A publishes /static_mission_complete.
  Static B and Autonomous Demo publish /static_estop.
```

### Full Autonomous Demo flow

```text
VCU touchscreen selects Autonomous Demo (AMI = 7)
  → mission_manager publishes /mission/selected = autonomous_demo
  → static_profile_executor loads autonomous_demo.yaml
  → fsai_vehicle_interface reaches DRIVING
  → steering sweep executes (time-based ramp steps)
  → drive_first_leg executes (distance step, exits at 12 m)
  → stop_mid executes (time-based hold, 5 s brake)
  → drive_second_leg executes (distance step, exits at 12 m)
  → static_profile_executor publishes /static_estop
  → mission_manager forwards /vehicle/estop
  → fsai_vehicle_interface sends ESTOP_YES to VCU → AS_EMERGENCY_BRAKE
```

---

## Build

Build from the ROS workspace root:

```bash
cd /home/mdxfsai/fsai_main/fsai_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fsai_interfaces fsai_vehicle_interface fsai_mission_control
source install/setup.bash
```

---

## Run

### Launch both mission-control nodes

```bash
cd /home/mdxfsai/fsai_main/fsai_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fsai_mission_control mission_control.launch.py
```

### Run with vehicle interface

Terminal 1:
```bash
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can2
```

Terminal 2:
```bash
ros2 launch fsai_mission_control mission_control.launch.py
```

### Override wheel circumference

```bash
ros2 run fsai_mission_control static_profile_executor \
  --ros-args -p wheel_circumference_m:=1.65
```

---

## Usage Flow on the Vehicle

### Static Inspection A or B

1. Bring up CAN and launch `fsai_vehicle_interface`.
2. Launch `fsai_mission_control`.
3. Select Static A or B on the VCU touchscreen.
4. Complete the ADS-DV handshake (ASMS on, TSMS on, wait for AS_READY, RES GO).
5. Profile executes automatically once the interface reaches `DRIVING`.
6. Static A ends with mission complete → VCU enters AS_FINISHED.
7. Static B ends with software e-stop → VCU enters AS_EMERGENCY_BRAKE.

### Autonomous Demo

Same as above but select Autonomous Demo (AMI 7) on the touchscreen. The profile runs the steer sweep, two 12 m drive legs with a stop in between, then deploys EBS.

### Monitoring

```bash
ros2 topic echo /mission/selected
ros2 topic echo /vehicle/interface_state
ros2 topic echo /vcu/status
ros2 topic echo /static_drive_command
ros2 topic echo /vehicle/drive_command
ros2 topic echo /vehicle/estop
```

---

## Bench Testing Without the Vehicle

Publish fake inputs to drive the executor without CAN:

```bash
# Fake Autonomous Demo selection
ros2 topic pub --rate 10 /vcu/status fsai_interfaces/msg/VcuStatus "{ami_state: 7}"

# Fake DRIVING state
ros2 topic pub --rate 20 /vehicle/interface_state fsai_interfaces/msg/InterfaceState \
  "{state: 4, state_name: 'DRIVING'}"

# Fake wheel speeds (needed for distance odometry and Static A stop check)
ros2 topic pub --rate 20 /vcu/wheel_speeds fsai_interfaces/msg/WheelSpeeds \
  "{fl_rpm: 120.0, fr_rpm: 120.0, rl_rpm: 120.0, rr_rpm: 120.0}"

# Monitor output
ros2 topic echo /static_drive_command
ros2 topic echo /static_estop
```

For Static A use `ami_state: 5`, for Static B use `ami_state: 6`.

---

## Extending for Dynamic Missions

Recommended pattern for adding dynamic missions:

1. Dynamic controller publishes to a candidate topic, e.g. `/dynamic_drive_command`.
2. Add a subscription for that candidate in `MissionManager`.
3. Forward to `/vehicle/drive_command` only when the selected mission matches and interface is `DRIVING`.

| Candidate topic | Forwarded topic | Allowed missions | Required interface state |
|---|---|---|---|
| `/dynamic_drive_command` | `/vehicle/drive_command` | `acceleration`, `skidpad`, `autocross`, `trackdrive` | `DRIVING` |

---

## Safety Notes

This package is intentionally conservative:

- It does not talk to CAN.
- It does not bypass `fsai_vehicle_interface`.
- It does not publish commands directly to `/vehicle/drive_command`.
- It does not forward commands unless the selected mission is compatible.
- It does not forward drive commands unless the interface state is `DRIVING`.

| Layer | Responsibility |
|---|---|
| `static_profile_executor` | Generate deterministic candidate commands. |
| `mission_manager` | Select and gate the active command source. |
| `fsai_vehicle_interface` | Own ADS-DV handshake, stale command timeout, actuator clamping, CAN transmission, and e-stop latching. |
| VCU | Own final safety enforcement and vehicle actuation. |

Important:

- Static B and Autonomous Demo intentionally trigger `/vehicle/estop` at profile completion. E-stop is latched at the VCU level and requires a full power cycle before another run.
- Autonomous Demo distance steps are open-loop odometry — they do not close the loop on actual position. Tune `wheel_circumference_m` to calibrate.
- Static profiles are open-loop command profiles, not closed-loop controllers.

---

## Troubleshooting

### `/mission/selected` stays `none`

Check that `/vcu/status` is being published and `ami_state` is not `AMI_NOT_SELECTED`.

```bash
ros2 topic echo /vcu/status
ros2 topic echo /mission/selected
```

### `/static_drive_command` is missing

- `/mission/selected` must be `static_inspection_a`, `static_inspection_b`, or `autonomous_demo`.
- `/vehicle/interface_state` must be `DRIVING`.
- The profile YAML must exist in the active profile directory.

### `/static_drive_command` exists but `/vehicle/drive_command` is missing

`mission_manager` is not forwarding. Check the selected mission and interface state:

```bash
ros2 topic echo /mission/selected
ros2 topic echo /vehicle/interface_state
```

### Distance steps complete too early or too late

The odometry uses rear wheel RPM and `wheel_circumference_m`. Adjust the parameter if the car over- or undershoots:

```bash
ros2 param get /static_profile_executor wheel_circumference_m
```

### Static A never publishes mission complete

Wheel speeds must be at or below `stopped_rpm_threshold` (default `10 rpm`) on all four wheels. Check:

```bash
ros2 topic echo /vcu/wheel_speeds
```

### Static B or Demo e-stop not forwarded

Check `/static_estop` is being published and that the selected mission is `static_inspection_b` or `autonomous_demo`:

```bash
ros2 topic echo /static_estop
ros2 topic echo /mission/selected
```

---

## Known Limitations

1. Dynamic mission routing (Acceleration, Skidpad, Autocross, Trackdrive) is not wired yet. Dynamic controllers should publish to candidate topics and be connected through `MissionManager`.

2. Autonomous Demo distance steps use open-loop rear-wheel odometry. Accuracy depends on `wheel_circumference_m` matching the actual rolling circumference under load, which is typically 1–2% less than the unloaded geometric value.

3. Static profiles are open-loop. They do not close the loop on actual wheel speed or steering angle during execution.

4. If a profile YAML file is missing or malformed, `static_profile_executor` raises an exception and exits. This is intentional — a malformed vehicle command profile should fail loudly.

5. If `DRIVING` is lost during a profile, the profile resets. When `DRIVING` returns, it starts again from the beginning.

6. Static A waits indefinitely for stopped wheel speeds if `/vcu/wheel_speeds` is absent. This prevents premature mission-complete signaling but requires wheel-speed data during testing.
