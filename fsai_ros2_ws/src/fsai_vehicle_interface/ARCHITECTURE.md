# fsai_vehicle_interface — Architecture Document

**Middlesex University Formula Student AI**
**Author:** MDX FSAI Software Team
**Status:** Design complete — implementation reference

---

## 1. Purpose

This document captures every design decision made for `fsai_vehicle_interface`.
It is the single source of truth for why the package is structured the way it is.
Read this before touching any code.

---

## 2. What this package is

`fsai_vehicle_interface` is a ROS 2 C++ node that acts as the **hardware abstraction
layer** between the MDX FSAI autonomous software stack and the FS-AI ADS-DV vehicle
over CAN bus.

It is the only component in the stack that knows the vehicle exists.
Every other node — perception, planning, control — talks to this node via ROS 2 topics
and has no knowledge of CAN, the VCU, or the ADS-DV state machine.

```
┌─────────────────────────────────────────────────┐
│           MDX FSAI ROS 2 Stack (Python)         │
│                                                 │
│   Perception ──► Planning ──► Control Node      │
│                                  │    ▲         │
│              /vehicle/drive_cmd  │    │ /vcu/*  │
│           /vehicle/mission_done  │    │         │
└──────────────────────────────────┼────┼─────────┘
                                   ▼    │
                    ┌──────────────────────────────┐
                    │   fsai_vehicle_interface      │
                    │        (C++ ROS 2 node)       │
                    │                              │
                    │   ┌─────────────────────┐   │
                    │   │   State Machine      │   │
                    │   │   (mission flow)     │   │
                    │   └─────────────────────┘   │
                    │                              │
                    │   ┌─────────────────────┐   │
                    │   │  FS-AI API (vendored)│   │
                    │   │  can.c / fs-ai_api.c │   │
                    │   └─────────────────────┘   │
                    └──────────────┬───────────────┘
                                   │ CAN bus (can0 / vcan0)
                    ┌──────────────▼───────────────┐
                    │         ADS-DV VCU            │
                    └──────────────────────────────┘
```

---

## 3. Why C++ for the node

The FS-AI API (`fs-ai_api.c`, `can.c`) is a C library. Wrapping C in C++ is
trivial and natural — the header already has `extern "C"` guards. Wrapping it in
Python would require ctypes or cffi and is significantly more complex and fragile.

The rest of the MDX stack is Python. This is not a problem. ROS 2 is language-agnostic
by design — the DDS middleware (FastRTPS, already configured in your `.bashrc`) handles
all serialization between nodes regardless of language. The Python planning nodes
simply publish and subscribe to ROS 2 topics. They never call C++ directly.

### How Python nodes use this package's messages

```python
from fsai_interfaces.msg import VcuStatus, DriveCommand, WheelSpeeds, InterfaceState

# Check which mission was selected by the operator
def status_cb(msg):
    if msg.ami_state == VcuStatus.AMI_ACCELERATION:
        # run acceleration algorithm
    elif msg.ami_state == VcuStatus.AMI_TRACK_DRIVE:
        # run track drive algorithm

# Check when it is safe to send drive commands
def state_cb(msg):
    if msg.state == InterfaceState.DRIVING:
        # safe to publish DriveCommand

# Send a drive command
cmd = DriveCommand()
cmd.steer_angle_deg = 5.0
cmd.axle_torque_nm  = 20.0
cmd.axle_speed_rpm  = 800.0
cmd.brake_pct       = 0.0
publisher.publish(cmd)
```

`rosidl` automatically generates both C++ headers and Python classes from the same
`.msg` files when the workspace is built. No manual bindings needed.

---

## 4. Self-contained design

This package vendors the FS-AI API source directly under `vendor/`:

```
vendor/
├── can.c         ← Linux SocketCAN wrapper
├── can.h
├── fs-ai_api.c   ← Main API implementation
└── fs-ai_api.h   ← Public API header (used by the C++ node)
```

These files are copied from the `FS-AI_API` repository and compiled directly by this
package's `CMakeLists.txt`. There is **no dependency on the FS-AI_API repo being
present** on the target machine. Clone `fsai_main`, build with colcon, done.

> **Note:** The vendor files must be kept in sync with the upstream FS-AI_API repo
> if the API is updated. The vendored version is pinned to the 2021 revision.

---

## 5. Topics

### 5.1 Published by this node (VCU data → ROS 2 stack)

| Topic                        | Type                               | Content                                | Rate   |
| ---------------------------- | ---------------------------------- | -------------------------------------- | ------ |
| `/vcu/status`              | `fsai_interfaces/VcuStatus`      | AS state, AMI state, RES GO, faults    | 100 Hz |
| `/vcu/wheel_speeds`        | `fsai_interfaces/WheelSpeeds`    | FL/FR/RL/RR rpm + pulse counts         | 100 Hz |
| `/vcu/steering`            | `std_msgs/Float32`               | Actual steer angle [deg]               | 100 Hz |
| `/vcu/brake`               | `std_msgs/Float32MultiArray`     | [front_pct, rear_pct]                  | 100 Hz |
| `/vcu/imu`                 | `sensor_msgs/Imu`                | Acceleration [m/s²], rotation [rad/s] | 100 Hz |
| `/vcu/gps`                 | `sensor_msgs/NavSatFix`          | Lat/lon [deg], altitude [m]            | 100 Hz |
| `/vehicle/interface_state` | `fsai_interfaces/InterfaceState` | Internal state machine state           | 100 Hz |

### 5.2 Subscribed by this node (stack → vehicle)

| Topic                         | Type                             | Publisher     | Purpose                     |
| ----------------------------- | -------------------------------- | ------------- | --------------------------- |
| `/vehicle/drive_command`    | `fsai_interfaces/DriveCommand` | Control node  | Steer, torque, speed, brake |
| `/vehicle/mission_complete` | `std_msgs/Bool`                | Planning node | Signal mission is done      |

---

## 6. New messages (live in `fsai_interfaces`)

Four new messages are required. They are provided in `messages_for_fsai_interfaces/`
and must be copied into `fsai_ros2_ws/src/fsai_interfaces/msg/` before building.
See Section 11 for transfer instructions.

### `DriveCommand.msg`

Drive commands from the control node to this interface node.
Intentionally minimal — state machine fields (mission status, direction, estop,
handshake) are owned entirely by this node and never exposed to the stack.

### `VcuStatus.msg`

All status data from the VCU, with enum constants baked in for clean Python usage.
Includes AS state, AMI state (mission selection), RES GO signal, and fault flags.

> **Note on fault bits:** The FS-AI API (`fs_ai_api_vcu2ai` struct) does not expose
> the raw fault bits from the VCU2AI_Status CAN frame. The fields are included in the
> message definition for forward compatibility and will be populated as `false` until
> the API is extended or the raw frame is read directly.

### `WheelSpeeds.msg`

All four wheel speeds and pulse counts in one message with a shared timestamp.

### `InterfaceState.msg`

The internal state machine state with enum constants. The Python stack uses this to
know when it is safe to send drive commands (`DRIVING`) and when the run is over
(`FINISHED`, `EMERGENCY`).

---

## 7. The ADS-DV state machine — correctly implemented

This is the most safety-critical part of the package. The straight-line test script
(`temp/fs-ai_api_straight_line.c`) had two bugs that caused problems at the ZF
workshop. Both are fixed here.

### 7.1 Correct VCU ↔ AI handshake sequence

```
VCU AS_STATE               AI MISSION_STATUS (we send)    Notes
─────────────────────────────────────────────────────────────────────
AS_INIT / AS_OFF           NOT_SELECTED (0)

  ↓ Operator selects mission on touchscreen → AMI_STATE changes

AS_OFF                     SELECTED (1)                   We respond immediately

  ↓ VCU arms EBS, runs 5-second timer, TSAL flashes blue

AS_READY                   SELECTED (1)                   Hold SELECTED

  ↓ ASR activates RES GO switch

AS_DRIVING                 RUNNING (2)      ← BUG FIX 1  Was SELECTED in old script

  ↓ AI stack signals mission complete via /vehicle/mission_complete

AS_DRIVING (still)         FINISHED (3)                   AI signals done first

  ↓ VCU acknowledges

AS_FINISHED                FINISHED (3)                   VCU confirms

  → Full reset required: LV Master off, ASMS off, TSMS off
  → On VCU power-up: AS_OFF + AMI_NOT_SELECTED → back to WAIT_FOR_MISSION ← BUG FIX 2
```

### 7.2 State machine states

```
WAIT_FOR_VCU (0)
  Entry: no VCU CAN frames received yet
  Sends: MISSION_NOT_SELECTED, DIRECTION_NEUTRAL, zero drive
  Exit:  first VCU status frame received → WAIT_FOR_MISSION

WAIT_FOR_MISSION (1)
  Entry: VCU alive but no mission selected on touchscreen
  Sends: MISSION_NOT_SELECTED, DIRECTION_NEUTRAL, zero drive
  Exit:  AMI_STATE != NOT_SELECTED → MISSION_SELECTED

MISSION_SELECTED (2)
  Entry: operator selected mission on touchscreen
  Sends: MISSION_SELECTED, DIRECTION_NEUTRAL, zero drive
  Exit:  AS_STATE = AS_READY → WAIT_FOR_GO
         AMI_STATE = NOT_SELECTED → WAIT_FOR_MISSION (mission cancelled)

WAIT_FOR_GO (3)
  Entry: VCU is AS_READY, 5-second timer running on VCU side
  Sends: MISSION_SELECTED, DIRECTION_NEUTRAL, zero drive
  Exit:  AS_STATE = AS_DRIVING → DRIVING
         AMI_STATE = NOT_SELECTED → WAIT_FOR_MISSION

DRIVING (4)
  Entry: VCU is AS_DRIVING, RES GO fired
  Sends: MISSION_RUNNING, DIRECTION_FORWARD
  Forwards: /vehicle/drive_command → CAN (if fresh, else zero)
  Exit:  /vehicle/mission_complete received → FINISHING
         AS_STATE = EMERGENCY_BRAKE → EMERGENCY (any state)

FINISHING (5)
  Entry: AI stack signalled mission complete
  Sends: MISSION_FINISHED, DIRECTION_NEUTRAL, zero drive
  Exit:  AS_STATE = AS_FINISHED → FINISHED

FINISHED (6)
  Entry: VCU confirmed AS_FINISHED
  Sends: MISSION_FINISHED, DIRECTION_NEUTRAL, zero drive
  Exit:  VCU power-cycled (AS_STATE = AS_OFF AND AMI = NOT_SELECTED) → WAIT_FOR_MISSION

EMERGENCY (7)
  Entry: AS_STATE = AS_EMERGENCY_BRAKE (from any state)
  Sends: MISSION_NOT_SELECTED, DIRECTION_NEUTRAL, zero drive
  Exit:  VCU power-cycled (AS_STATE = AS_OFF AND AMI = NOT_SELECTED) → WAIT_FOR_MISSION
```

### 7.3 Bug fixes from test script

| Bug                        | Old behaviour                                                        | Fixed behaviour                                                               |
| -------------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| MISSION_RUNNING never sent | Jumped 0→1→3, skipping RUNNING(2). VCU raised MISSION_STATUS_FAULT | DRIVING state sends RUNNING(2) correctly                                      |
| FINISHED dead end          | No exit from FINISHED if AMI still selected. Script stuck forever    | FINISHED and EMERGENCY exit when VCU power-cycles (AS_OFF + AMI_NOT_SELECTED) |

---

## 8. Stale command safety

If the planning/control node crashes or stops publishing `/vehicle/drive_command`,
the interface node must not continue sending the last known command indefinitely.

**Behaviour:** If the last received `DriveCommand` is older than `stale_command_timeout_ms`
(default 100ms), all drive outputs are **zeroed immediately**:

- `AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0`
- `AI2VCU_AXLE_SPEED_REQUEST_rpm = 0.0`
- `AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0.0`
- `AI2VCU_BRAKE_PRESS_REQUEST_pct = 0.0`

The node logs a warning at 1Hz when this occurs. The state machine is not affected —
recovery happens automatically when fresh commands resume.

---

## 9. E-stop

**E-stop is physical RES only.** The `AI2VCU_ESTOP_REQUEST` field is always set to
`ESTOP_NO` in software. The physical RES handled by the ASR is the only e-stop
mechanism. This is consistent with ADS-DV operational safety requirements.

---

## 10. Dual-vehicle architecture (real car + CarMaker sim)

The MDX stack publishes **two separate command topics** simultaneously:

```
Control Node
  │
  ├── /vehicle/drive_command        [fsai_interfaces/DriveCommand]
  │       consumed by: fsai_vehicle_interface (real car, CAN)
  │
  └── /vehicle/carmaker_control     [vehiclecontrol_msgs/VehicleControl]
          consumed by: CarMaker bridge (simulation)
```

The CarMaker message format (`gas` pedal 0–1, `selector_ctrl` gear, `steer_ang` in
radians) is fundamentally different from the CAN format (`axle_torque_nm`, `brake_pct`,
`steer_angle_deg`). A shared message format would be lossy or bloated. Dual-publish
keeps each consumer's interface clean and native.

When running on real hardware: only `fsai_vehicle_interface` is active.
When running in simulation: only the CarMaker bridge is active.
The control node publishes both at all times and neither consumer interferes with the
other.

---

## 11. Deployment — transfer instructions

### Step 1: Copy message files

Copy all four files from `messages_for_fsai_interfaces/` into the interfaces package:

```bash
cp messages_for_fsai_interfaces/*.msg \
   path/to/fsai_ros2_ws/src/fsai_interfaces/msg/
```

Then update `fsai_interfaces/CMakeLists.txt` to include the new messages in the
`rosidl_generate_interfaces` call:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone3D.msg"
  "msg/Cone3DArray.msg"
  "msg/VehicleControl.msg"
  "msg/DriveCommand.msg"       # add these
  "msg/VcuStatus.msg"          # add these
  "msg/WheelSpeeds.msg"        # add these
  "msg/InterfaceState.msg"     # add these
  DEPENDENCIES std_msgs geometry_msgs
)
```

### Step 2: Copy this package

Copy the entire `fsai_vehicle_interface/` folder into:

```
fsai_ros2_ws/src/fsai_vehicle_interface/
```

### Step 3: Set up CAN interface (on the AI computer)

```bash
# Load kernel modules (once per boot, or add to /etc/modules)
sudo modprobe can_dev
sudo modprobe can
sudo modprobe can_raw

# Hardware CAN at 500 kbps
sudo ip link set up can0 type can bitrate 500000

# OR virtual CAN for testing without hardware
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

### Step 4: Build

```bash
cd fsai_ros2_ws
colcon build --packages-select fsai_interfaces fsai_vehicle_interface
source install/setup.bash
```

Build `fsai_interfaces` first (or let colcon resolve order via package.xml dependencies).

### Step 5: Run	

```bash
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can0
```

---

## 12. Parameters

| Parameter                    | Default  | Description                                          |
| ---------------------------- | -------- | ---------------------------------------------------- |
| `can_interface`            | `can0` | CAN interface name (can0, vcan0, etc.)               |
| `stale_command_timeout_ms` | `100`  | ms before /vehicle/drive_command is considered stale |
| `loop_rate_hz`             | `100`  | Main loop rate. 100Hz = 10ms matches VCU CAN timing  |
| `imu_frame_id`             | `imu`  | TF frame ID for IMU messages                         |
| `gps_frame_id`             | `gps`  | TF frame ID for GPS messages                         |

---

## 13. Known limitations and future work

| Item                    | Notes                                                                                                                                                    |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Fault bits always false | `fs_ai_api_vcu2ai` does not expose raw VCU fault bits. Fields reserved in `VcuStatus.msg` for when the API is extended or raw frame reading is added |
| IMU covariance unknown  | `sensor_msgs/Imu` covariance matrices set to -1 (unknown). Calibration data needed for SLAM                                                            |
| GPS accuracy            | PCAN-GPS provides raw NMEA-style data. No RTK correction                                                                                                 |
| Single CAN interface    | Only one CAN interface supported per node instance                                                                                                       |
| No reconnection         | If CAN goes down, node must be restarted                                                                                                                 |
