# fsai_mission_control

ROS 2 mission supervision and deterministic static-inspection execution for the MDX Formula Student AI stack.

This package sits above `fsai_vehicle_interface`. It does not talk to CAN, it does not implement the ADS-DV handshake directly, and it does not replace the vehicle-interface state machine. Instead, it decides which mission-specific command source is allowed to reach `/vehicle/drive_command`, and it provides deterministic command profiles for Static Inspection A and Static Inspection B.

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
| - loads YAML profiles for Static A/B                         |
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
+-------------------------------+------------------------------+
                                |
                                | SocketCAN can0 / vcan0
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
2. Static inspection execution: generate deterministic commands for Static Inspection A and Static Inspection B.

The package currently includes two ROS 2 nodes:

| Node | Executable | Responsibility |
|---|---|---|
| `MissionManager` | `mission_manager` | Converts VCU AMI selection into `/mission/selected`, gates command forwarding, and forwards completion/e-stop requests to `fsai_vehicle_interface`. |
| `StaticProfileExecutor` | `static_profile_executor` | Loads Static A/B YAML command profiles and publishes candidate static command topics. |

The package deliberately publishes static commands to candidate topics first. The static executor does not publish directly to `/vehicle/drive_command`. This prevents multiple nodes from independently commanding the vehicle.

---

## Why This Package Exists

The ADS-DV stack has several different command-producing modes:

- Static Inspection A.
- Static Inspection B.
- Acceleration.
- Skidpad.
- Autocross.
- Trackdrive.
- Autonomous demo.
- Bench/HIL test utilities.

Only one source should be allowed to command `/vehicle/drive_command` at a time. If static inspection, navigation, and test tooling all publish to `/vehicle/drive_command` directly, there is no single point of arbitration.

This package introduces that arbitration point.

Current behavior:

- Static A and Static B are supported.
- Static commands are forwarded only when the VCU-selected AMI mission is Static A or Static B.
- Static commands are forwarded only when `fsai_vehicle_interface` reports `InterfaceState.DRIVING`.
- Static mission completion is forwarded to `/vehicle/mission_complete`.
- Static B e-stop is forwarded to `/vehicle/estop`.

Future behavior should follow the same pattern:

- Dynamic navigation publishes to a candidate topic, for example `/dynamic_drive_command`.
- `MissionManager` forwards that candidate topic only when the selected mission is a dynamic mission and the vehicle interface is in `DRIVING`.
- `/vehicle/drive_command` remains the single final command topic consumed by `fsai_vehicle_interface`.

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
- Forward static drive commands to `/vehicle/drive_command` only when Static A/B is selected and the vehicle interface is `DRIVING`.
- Forward Static A mission completion to `/vehicle/mission_complete`.
- Forward Static B e-stop request to `/vehicle/estop`.

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
| `/static_drive_command` | `/vehicle/drive_command` | `static_inspection_a` or `static_inspection_b` | `DRIVING` |
| `/static_mission_complete` | `/vehicle/mission_complete` | `static_inspection_a` or `static_inspection_b` | Any state |
| `/static_estop` | `/vehicle/estop` | `static_inspection_b` only | Any state |

The manager does not modify drive commands. It only routes them. Command clamping, command timeout handling, and CAN transmission remain inside `fsai_vehicle_interface`.

### `static_profile_executor`

`static_profile_executor` runs deterministic static-inspection command profiles from YAML.

Responsibilities:

- Subscribe to `/mission/selected`.
- Load `static_inspection_a.yaml` when Static A is selected.
- Load `static_inspection_b.yaml` when Static B is selected.
- Wait until `/vehicle/interface_state` reports `DRIVING` before starting the active profile.
- Publish candidate commands on `/static_drive_command` at the configured publish rate.
- Publish `/static_mission_complete` when a mission-complete profile finishes.
- Publish `/static_estop` when an e-stop profile finishes.
- For Static A, wait for wheel speeds to drop below the configured stopped threshold before publishing mission complete.

Important behavior:

- The executor is intentionally not connected directly to `/vehicle/drive_command`.
- If the interface leaves `DRIVING` while a profile is running, the profile timer is reset.
- When `DRIVING` returns, the active profile starts again from the beginning.
- If the selected mission changes, the executor resets its current profile state.
- If no Static A/B mission is selected, the executor stays idle.

---

## Topics Reference

### `mission_manager` subscriptions

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/vcu/status` | `fsai_interfaces/msg/VcuStatus` | `fsai_vehicle_interface` | Reads `ami_state` to know which mission the operator selected on the VCU touchscreen. |
| `/vehicle/interface_state` | `fsai_interfaces/msg/InterfaceState` | `fsai_vehicle_interface` | Reads whether the vehicle interface is currently in `DRIVING`. |
| `/static_drive_command` | `fsai_interfaces/msg/DriveCommand` | `static_profile_executor` | Candidate static actuator command. |
| `/static_mission_complete` | `std_msgs/msg/Bool` | `static_profile_executor` | Candidate mission-complete request. |
| `/static_estop` | `std_msgs/msg/Bool` | `static_profile_executor` | Candidate software e-stop request for Static B. |

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
| `/mission/selected` | `std_msgs/msg/String` | `mission_manager` | Selects and loads the active static profile. |
| `/vehicle/interface_state` | `fsai_interfaces/msg/InterfaceState` | `fsai_vehicle_interface` | Starts or pauses profile execution based on `DRIVING`. |
| `/vcu/wheel_speeds` | `fsai_interfaces/msg/WheelSpeeds` | `fsai_vehicle_interface` | Used by Static A to confirm the vehicle has stopped before mission complete. |

### `static_profile_executor` publications

| Topic | Type | Destination | Purpose |
|---|---|---|---|
| `/static_drive_command` | `fsai_interfaces/msg/DriveCommand` | `mission_manager` | Candidate static command. |
| `/static_mission_complete` | `std_msgs/msg/Bool` | `mission_manager` | Candidate mission-complete request. |
| `/static_estop` | `std_msgs/msg/Bool` | `mission_manager` | Candidate Static B e-stop request. |

---

## Parameters

### `mission_manager`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | `double` | `10.0` | Rate used to republish `/mission/selected`. The value is clamped to at least `1.0`. |

### `static_profile_executor`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | Rate used to evaluate the profile and publish `/static_drive_command`. The value is clamped to at least `1.0`. |
| `profile_directory` | `string` | `""` | Optional directory containing static profile YAML files. Empty means use the installed package share directory. |

The default profile directory resolves to:

```text
<workspace>/install/fsai_mission_control/share/fsai_mission_control/profiles
```

when the package is installed by colcon.

To run with a custom profile directory:

```bash
ros2 run fsai_mission_control static_profile_executor \
  --ros-args -p profile_directory:=/absolute/path/to/profiles
```

The current launch file does not expose launch arguments for these parameters. Use direct `ros2 run` commands for parameter overrides, or add launch arguments if repeated parameterized launches are needed.

---

## Static Inspection Profiles

Static profiles are YAML files stored in `profiles/` and installed into the package share directory.

Each profile contains:

| Field | Required | Description |
|---|---|---|
| `mission` | Yes | Human-readable profile mission name. |
| `completion_action` | Yes | Either `mission_complete` or `estop`. |
| `require_stopped_before_completion` | Optional | If true, wait for wheel speeds below threshold before mission complete. Used by Static A. |
| `stopped_rpm_threshold` | Optional | Wheel-speed threshold in rpm. Default behavior in the profile uses `10.0`. |
| `steps` | Yes | Ordered list of deterministic command steps. |

Each step contains:

| Field | Description |
|---|---|
| `name` | Descriptive step name for readability. |
| `type` | `hold` or `ramp`. |
| `duration_s` | Step duration in seconds. |
| `command` | Command used by `hold` steps. |
| `start` | Start command used by `ramp` steps. |
| `end` | End command used by `ramp` steps. |

Command fields match `fsai_interfaces/msg/DriveCommand`:

| Field | Unit | Notes |
|---|---|---|
| `steer_angle_deg` | degrees | Positive means left. Static profiles use the ADS-DV steering request range of `-21` to `+21` degrees. |
| `axle_speed_rpm` | rpm | Non-negative axle speed request. |
| `axle_torque_nm` | Nm | Non-negative axle torque request. |
| `brake_pct` | percent | Clamped to `0.0` to `100.0`. If brake is greater than zero, the executor zeros torque before publishing. |

### Static Inspection A

File:

```text
profiles/static_inspection_a.yaml
```

Purpose:

- Perform a full steering sweep.
- Ramp axle speed to `200 rpm` in `10 s`.
- Stop with brake for `5 s`.
- Wait until wheel speeds are below `10 rpm`.
- Publish mission complete, which allows `fsai_vehicle_interface` to move toward `AS_FINISHED`.

Current profile sequence:

| Step | Type | Duration | Command behavior |
|---|---|---|---|
| `centre_steering` | `hold` | `1.0 s` | Hold steering centered and vehicle stationary. |
| `sweep_to_left_lock` | `ramp` | `2.0 s` | Ramp steering from `0 deg` to `+21 deg`. |
| `sweep_to_right_lock` | `ramp` | `4.0 s` | Ramp steering from `+21 deg` to `-21 deg`. |
| `return_steering_to_centre` | `ramp` | `2.0 s` | Ramp steering from `-21 deg` to `0 deg`. |
| `ramp_to_200rpm` | `ramp` | `10.0 s` | Ramp axle speed from `0 rpm` to `200 rpm` with `10 Nm` torque request. |
| `stop_with_brake` | `hold` | `5.0 s` | Request `0 rpm`, `0 Nm`, and `20%` brake. |

Completion behavior:

- `completion_action: mission_complete`.
- `require_stopped_before_completion: true`.
- The executor waits until all four wheel speeds are at or below `10 rpm` in absolute value.
- If no `/vcu/wheel_speeds` message has been received, the executor waits and does not publish mission complete.

### Static Inspection B

File:

```text
profiles/static_inspection_b.yaml
```

Purpose:

- Spin to `50 rpm`.
- Hold briefly.
- Trigger software e-stop, causing the ADS-DV state to move toward `AS_EMERGENCY_BRAKE`.

Current profile sequence:

| Step | Type | Duration | Command behavior |
|---|---|---|---|
| `ramp_to_50rpm` | `ramp` | `3.0 s` | Ramp axle speed from `0 rpm` to `50 rpm` with `8 Nm` torque request. |
| `hold_50rpm` | `hold` | `2.0 s` | Hold `50 rpm` with `8 Nm` torque request. |

Completion behavior:

- `completion_action: estop`.
- The executor publishes `/static_estop`.
- `mission_manager` forwards that request to `/vehicle/estop` only when the selected mission is `static_inspection_b`.

---

## State Machine Interaction

There are two state concepts to keep separate:

1. The ADS-DV vehicle-interface state machine in `fsai_vehicle_interface`.
2. The mission-control routing/profile lifecycle in `fsai_mission_control`.

### Vehicle-interface state machine

`fsai_vehicle_interface` owns the real ADS-DV handshake. It publishes the current internal state on `/vehicle/interface_state`.

Expected state progression:

```text
WAIT_FOR_VCU
  -> WAIT_FOR_MISSION
  -> MISSION_SELECTED
  -> WAIT_FOR_GO
  -> DRIVING
  -> FINISHING
  -> FINISHED

Any state can move to EMERGENCY if the VCU enters AS_EMERGENCY_BRAKE.
```

Meaning of the states for mission control:

| Interface state | Meaning for `fsai_mission_control` |
|---|---|
| `WAIT_FOR_VCU` | No VCU frames yet. Do not command. |
| `WAIT_FOR_MISSION` | VCU is alive, but no mission is selected. Do not command. |
| `MISSION_SELECTED` | Mission is selected, but the car is not ready to drive. Do not command. |
| `WAIT_FOR_GO` | Waiting for RES GO. Do not command. |
| `DRIVING` | Commands may be forwarded if the selected mission matches. |
| `FINISHING` | Vehicle interface is finishing after mission complete. Do not command. |
| `FINISHED` | Mission is finished. Do not command. |
| `EMERGENCY` | Emergency brake state. Do not command. |

`mission_manager` only forwards drive commands when the interface state equals `DRIVING`.

### Mission-control routing state

`MissionManager` has a simple routing state derived from `/vcu/status.ami_state`:

```text
AMI_NOT_SELECTED          -> /mission/selected = none
AMI_STATIC_INSPECTION_A   -> /mission/selected = static_inspection_a
AMI_STATIC_INSPECTION_B   -> /mission/selected = static_inspection_b
Other AMI states          -> matching mission string, but no dynamic forwarding yet
```

For static missions, the routing state can be described as:

```text
No static mission selected
  -> publish /mission/selected = none or another mission
  -> ignore /static_drive_command

Static A/B selected, interface not DRIVING
  -> publish /mission/selected = static_inspection_a or static_inspection_b
  -> ignore /static_drive_command
  -> static_profile_executor remains armed but paused

Static A/B selected, interface DRIVING
  -> forward /static_drive_command to /vehicle/drive_command
  -> forward valid completion/e-stop requests
```

### Static profile lifecycle

`StaticProfileExecutor` lifecycle:

```text
IDLE
  No Static A/B mission selected.

LOAD_PROFILE
  /mission/selected changed to static_inspection_a or static_inspection_b.
  The matching YAML file is loaded.

ARMED_WAITING_FOR_DRIVING
  Profile is loaded, but /vehicle/interface_state is not DRIVING.
  No commands are published.

EXECUTING
  Interface state is DRIVING.
  The profile timer starts.
  Candidate commands are published on /static_drive_command.

WAITING_FOR_STOP
  Static A only.
  Profile steps are finished, but wheel speeds are missing or above threshold.
  Last command continues to be published.

COMPLETE
  Static A publishes /static_mission_complete.
  Static B publishes /static_estop.
  The profile stops publishing new commands until the mission selection changes.
```

Full Static A flow:

```text
VCU touchscreen selects Static A
  -> fsai_vehicle_interface publishes /vcu/status.ami_state = AMI_STATIC_INSPECTION_A
  -> mission_manager publishes /mission/selected = static_inspection_a
  -> static_profile_executor loads static_inspection_a.yaml
  -> fsai_vehicle_interface reaches /vehicle/interface_state = DRIVING
  -> static_profile_executor publishes /static_drive_command
  -> mission_manager forwards to /vehicle/drive_command
  -> static profile finishes braking step
  -> static_profile_executor waits for /vcu/wheel_speeds below threshold
  -> static_profile_executor publishes /static_mission_complete
  -> mission_manager forwards /vehicle/mission_complete
  -> fsai_vehicle_interface enters FINISHING and signals mission finished to the VCU
  -> VCU acknowledges AS_FINISHED
```

Full Static B flow:

```text
VCU touchscreen selects Static B
  -> fsai_vehicle_interface publishes /vcu/status.ami_state = AMI_STATIC_INSPECTION_B
  -> mission_manager publishes /mission/selected = static_inspection_b
  -> static_profile_executor loads static_inspection_b.yaml
  -> fsai_vehicle_interface reaches /vehicle/interface_state = DRIVING
  -> static_profile_executor publishes /static_drive_command
  -> mission_manager forwards to /vehicle/drive_command
  -> static profile finishes hold step
  -> static_profile_executor publishes /static_estop
  -> mission_manager forwards /vehicle/estop
  -> fsai_vehicle_interface requests emergency braking from the VCU
```

---

## Build

Build from the ROS workspace root, not from the repository root.

Correct workspace directory:

```bash
cd /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws
```

Source the ROS 2 distribution installed on the target machine. In this environment that is Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

If the target machine uses Humble, use:

```bash
source /opt/ros/humble/setup.bash
```

Build the relevant packages:

```bash
colcon build \
  --packages-select fsai_interfaces fsai_vehicle_interface fsai_mission_control \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Then source the workspace overlay:

```bash
source install/setup.bash
```

Build all packages if desired:

```bash
colcon build --symlink-install
source install/setup.bash
```

If you accidentally build from `/home/bashesam33/FS-AI/fsai_main` instead of `/home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws`, remove the accidental root-level colcon artifacts:

```bash
cd /home/bashesam33/FS-AI/fsai_main
rm -rf build install log
cd fsai_ros2_ws
```

---

## Run

### Launch both mission-control nodes

```bash
cd /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch fsai_mission_control mission_control.launch.py
```

Replace `jazzy` with `humble` if running on a Humble machine.

### Run nodes individually

Terminal 1:

```bash
ros2 run fsai_mission_control mission_manager
```

Terminal 2:

```bash
ros2 run fsai_mission_control static_profile_executor
```

Run the static executor with a custom profile directory:

```bash
ros2 run fsai_mission_control static_profile_executor \
  --ros-args -p profile_directory:=/absolute/path/to/profiles
```

### Run with vehicle interface

Terminal 1:

```bash
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py
```

Terminal 2:

```bash
ros2 launch fsai_mission_control mission_control.launch.py
```

The launch order is not critical for mission control. If topics are not available yet, the nodes simply wait for messages.

---

## Usage Flow on the Vehicle

Expected operational flow for Static Inspection A or B:

1. Bring up CAN and launch `fsai_vehicle_interface`.
2. Launch `fsai_mission_control`.
3. Select Static Inspection A or B on the VCU touchscreen.
4. Confirm `/mission/selected` updates to `static_inspection_a` or `static_inspection_b`.
5. Wait for the vehicle interface to complete the ADS-DV handshake.
6. When RES GO causes the VCU to enter `AS_DRIVING`, `fsai_vehicle_interface` publishes `/vehicle/interface_state = DRIVING`.
7. The static executor starts the active profile.
8. The mission manager forwards static commands to `/vehicle/drive_command`.
9. Static A ends by forwarding `/vehicle/mission_complete` after the car is stopped.
10. Static B ends by forwarding `/vehicle/estop`.

Useful monitoring commands:

```bash
ros2 topic echo /mission/selected
ros2 topic echo /vehicle/interface_state
ros2 topic echo /vcu/status
ros2 topic echo /static_drive_command
ros2 topic echo /vehicle/drive_command
ros2 topic echo /vehicle/mission_complete
ros2 topic echo /vehicle/estop
```

Expected topic behavior:

- `/mission/selected` should update after the VCU AMI selection changes.
- `/static_drive_command` should appear only when Static A/B is selected and the interface is `DRIVING`.
- `/vehicle/drive_command` should match `/static_drive_command` only when the manager is forwarding.
- For Static A, `/vehicle/mission_complete` should publish `true` after the profile finishes and wheel speeds are below threshold.
- For Static B, `/vehicle/estop` should publish `true` after the profile finishes.

---

## Bench Testing Without the Vehicle

The mission-control package can be tested without CAN by publishing fake `/vcu/status`, `/vehicle/interface_state`, and `/vcu/wheel_speeds` messages.

Terminal 1:

```bash
cd /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fsai_mission_control mission_control.launch.py
```

Terminal 2, monitor output:

```bash
source /opt/ros/jazzy/setup.bash
source /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws/install/setup.bash

ros2 topic echo /mission/selected
```

Terminal 3, fake Static A selection:

```bash
source /opt/ros/jazzy/setup.bash
source /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws/install/setup.bash

ros2 topic pub --rate 10 /vcu/status fsai_interfaces/msg/VcuStatus \
  "{ami_state: 5}"
```

Terminal 4, fake DRIVING state:

```bash
source /opt/ros/jazzy/setup.bash
source /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws/install/setup.bash

ros2 topic pub --rate 20 /vehicle/interface_state fsai_interfaces/msg/InterfaceState \
  "{state: 4, state_name: 'DRIVING'}"
```

Terminal 5, monitor generated commands:

```bash
source /opt/ros/jazzy/setup.bash
source /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws/install/setup.bash

ros2 topic echo /static_drive_command
```

Terminal 6, allow Static A completion by publishing stopped wheel speeds:

```bash
source /opt/ros/jazzy/setup.bash
source /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws/install/setup.bash

ros2 topic pub --rate 20 /vcu/wheel_speeds fsai_interfaces/msg/WheelSpeeds \
  "{fl_rpm: 0.0, fr_rpm: 0.0, rl_rpm: 0.0, rr_rpm: 0.0}"
```

To test Static B, publish AMI value `6` instead of `5`:

```bash
ros2 topic pub --rate 10 /vcu/status fsai_interfaces/msg/VcuStatus \
  "{ami_state: 6}"
```

Then monitor:

```bash
ros2 topic echo /static_estop
ros2 topic echo /vehicle/estop
```

Expected result:

- Static A produces a steering sweep, a speed ramp, a braking command, then mission complete once wheel speeds are stopped.
- Static B ramps to `50 rpm`, holds briefly, then publishes e-stop.

---

## Extending for Dynamic Missions

This package is designed to become the central command router for all missions, not only static inspection.

Recommended pattern for adding dynamic missions:

1. Keep dynamic navigation/control outside `fsai_vehicle_interface`.
2. Have the dynamic controller publish to a candidate topic, for example `/dynamic_drive_command`.
3. Add a subscription for that candidate topic in `MissionManager`.
4. Forward that candidate command to `/vehicle/drive_command` only when the selected mission is appropriate and the interface state is `DRIVING`.
5. Keep `/vehicle/drive_command` as the single final command topic consumed by `fsai_vehicle_interface`.

Example dynamic forwarding rule:

| Candidate topic | Forwarded topic | Allowed missions | Required interface state |
|---|---|---|---|
| `/dynamic_drive_command` | `/vehicle/drive_command` | `acceleration`, `skidpad`, `autocross`, `trackdrive`, `autonomous_demo` | `DRIVING` |

This keeps mission ownership explicit and avoids multiple command sources racing on `/vehicle/drive_command`.

---

## Safety Notes

This package is intentionally conservative:

- It does not talk to CAN.
- It does not bypass `fsai_vehicle_interface`.
- It does not publish static commands directly to `/vehicle/drive_command`.
- It does not forward commands unless the selected mission is compatible.
- It does not forward drive commands unless the interface state is `DRIVING`.

Safety responsibilities are split as follows:

| Layer | Responsibility |
|---|---|
| `static_profile_executor` | Generate deterministic candidate commands for static inspection. |
| `mission_manager` | Select and gate the active command source. |
| `fsai_vehicle_interface` | Own ADS-DV handshake, stale command timeout, actuator clamping, CAN transmission, mission-complete signaling, and e-stop forwarding. |
| VCU | Own final safety enforcement and vehicle actuation. |

Important operational notes:

- Static B intentionally triggers `/vehicle/estop` at the end of the profile.
- E-stop behavior is latched at the vehicle-interface/VCU level and generally requires a reset before another run.
- Static A waits for wheel speeds below threshold before mission complete. If wheel-speed data is missing, the mission will not complete.
- The deterministic profiles are open-loop command profiles. They are not closed-loop speed controllers.

---

## Troubleshooting

### `Package 'fsai_mission_control' not found`

The package has not been built or the workspace overlay has not been sourced.

```bash
cd /home/bashesam33/FS-AI/fsai_main/fsai_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fsai_mission_control --symlink-install
source install/setup.bash
ros2 pkg list | grep fsai_mission_control
```

### `ModuleNotFoundError: No module named 'yaml'`

Install the ROS/system YAML dependency and rebuild:

```bash
sudo apt install python3-yaml
```

The dependency is declared in `package.xml` as `python3-yaml`.

### `/mission/selected` stays `none`

Check that `/vcu/status` is being published and that `ami_state` is not `AMI_NOT_SELECTED`.

```bash
ros2 topic echo /vcu/status
ros2 topic echo /mission/selected
```

If `/vcu/status` is missing, inspect `fsai_vehicle_interface` and the CAN/VCU setup.

### `/static_drive_command` is missing

Check these conditions:

- `/mission/selected` must be `static_inspection_a` or `static_inspection_b`.
- `/vehicle/interface_state` must be `DRIVING`.
- The static profile YAML file must exist in the active profile directory.

Useful commands:

```bash
ros2 topic echo /mission/selected
ros2 topic echo /vehicle/interface_state
ros2 param get /static_profile_executor profile_directory
```

### `/static_drive_command` exists but `/vehicle/drive_command` is missing

`mission_manager` is not forwarding. Check:

- The selected mission is Static A or Static B.
- `/vehicle/interface_state.state` is exactly `4`, which corresponds to `InterfaceState.DRIVING`.
- `mission_manager` is running.

```bash
ros2 node list
ros2 topic echo /mission/selected
ros2 topic echo /vehicle/interface_state
ros2 topic echo /static_drive_command
ros2 topic echo /vehicle/drive_command
```

### Static A finishes profile steps but never publishes mission complete

Static A requires stopped wheel speeds before completion.

Check:

```bash
ros2 topic echo /vcu/wheel_speeds
```

The absolute value of all four wheel speeds must be at or below the profile threshold, currently `10 rpm`.

### Static B does not trigger `/vehicle/estop`

Check:

- `/mission/selected` is `static_inspection_b`.
- The profile has reached the end of the `hold_50rpm` step.
- `mission_manager` is running.

Monitor:

```bash
ros2 topic echo /static_estop
ros2 topic echo /vehicle/estop
```

### Build fails in `fsai_vehicle_interface`

This package does not modify `fsai_vehicle_interface`. If `fsai_vehicle_interface` fails to build, inspect that package separately rather than adding CAN/HAL fixes here.

### Build was accidentally run from the wrong directory

If `build/`, `install/`, and `log/` were created under `fsai_main/`, remove those accidental artifacts and rebuild from `fsai_ros2_ws/`:

```bash
cd /home/bashesam33/FS-AI/fsai_main
rm -rf build install log
cd fsai_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## Known Limitations

1. Only Static Inspection A and Static Inspection B are currently implemented as command sources.

2. Dynamic mission routing is not wired yet. Dynamic controllers should be adapted to candidate topics before being connected through `MissionManager`.

3. Static profiles are open-loop. They do not close the loop on actual wheel speed or steering angle during the profile. Wheel speed is currently used only to confirm Static A has stopped before mission complete.

4. The launch file currently has no launch arguments. Parameter overrides are easiest with direct `ros2 run` commands.

5. If a profile YAML file is missing or malformed, `static_profile_executor` raises an exception and exits. This is intentional because a malformed vehicle command profile should fail loudly.

6. If `DRIVING` is lost during a static profile, the profile timer resets. When `DRIVING` returns, the profile starts again from the beginning.

7. Static A waits forever for stopped wheel speeds if `/vcu/wheel_speeds` is absent. This prevents premature `AS_FINISHED` signaling but requires wheel-speed data during real or simulated testing.

---

## Summary

`fsai_mission_control` is the software mission supervisor for the stack. It keeps `fsai_vehicle_interface` clean as the CAN/ADS-DV HAL, while providing a safe routing layer for mission-specific command sources.

Current supported use cases:

- Static Inspection A deterministic steering/speed/brake profile.
- Static Inspection B deterministic speed/e-stop profile.
- Central mission selection topic via `/mission/selected`.
- Single final command path into `/vehicle/drive_command`.

Future use cases should extend the same router pattern rather than publishing directly to `/vehicle/drive_command` from multiple nodes.
