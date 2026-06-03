# fsai_vehicle_interface — System Information

Detailed reference for every topic, message field, and expected data value for this package. Read this to know exactly what the vehicle interface publishes, what it expects from the stack, and what the data looks like at runtime.

---

## Table of Contents

- [Topics Overview](#topics-overview)
- [What the Stack Must Publish (Inputs)](#what-the-stack-must-publish-inputs)
- [What This Node Publishes (Outputs)](#what-this-node-publishes-outputs)
- [Message Field Reference](#message-field-reference)
- [State Machine Quick Reference](#state-machine-quick-reference)
- [Runtime Behaviour Notes](#runtime-behaviour-notes)

---

## Topics Overview

```
          YOUR STACK (planning / control)
                      │
        ┌─────────────┼──────────────────┐
        │             │                  │
        ▼             ▼                  ▼
/vehicle/         /vehicle/         /vehicle/
drive_command     mission_complete  estop
        │             │                  │
        └─────────────▼──────────────────┘
                      │
             fsai_vehicle_interface
             (100 Hz main loop)
                      │
        ┌─────────────┼───────────────────────────────┐
        │             │             │                  │
        ▼             ▼             ▼                  ▼
/vcu/status   /vcu/wheel_speeds  /vcu/imu     /vehicle/interface_state
/vcu/steering /vcu/brake         /vcu/gps
        │
        ▼
  ADS-DV VCU (CAN bus, 100 Hz)
```

---

## What the Stack Must Publish (Inputs)

### `/vehicle/drive_command` — `fsai_interfaces/DriveCommand`

The main actuator command. Published by your control node. Only forwarded to the VCU when the state machine is in `DRIVING` state and the command is fresh (< 100ms old).

| Field | Type | Range | Example | Notes |
|---|---|---|---|---|
| `header.stamp` | `time` | — | `now()` | Timestamp of the command |
| `steer_angle_deg` | `float32` | ± VCU max | `5.0` | Positive = left. Clamped internally to VCU steering limit. |
| `axle_speed_rpm` | `float32` | 0 → ~1143 | `800.0` | Axle speed limit [rpm]. Motor RPM = axle × 3.5 (gear ratio), capped at 4000 rpm motor-side. |
| `axle_torque_nm` | `float32` | 0 → VCU max | `20.0` | Torque request [Nm]. Ignored if `brake_pct > 0`. |
| `brake_pct` | `float32` | 0.0 – 100.0 | `0.0` | Hydraulic brake pressure [%]. If > 0, torque is zeroed by VCU. |

> **Critical:** Never send `axle_torque_nm > 0` and `brake_pct > 0` simultaneously. The VCU treats this as implausible and fires EBS. Always zero one before setting the other.

> **Stale timeout:** If no `DriveCommand` is received within `stale_command_timeout_ms` (default 100ms), all actuator outputs are zeroed and a warning is logged at 1 Hz. The state machine is not affected — commands resume normally when publishing restarts.

**Minimum publish rate:** 10 Hz. Recommended: 100 Hz to match the main loop.

---

### `/vehicle/mission_complete` — `std_msgs/Bool`

Publish `true` **once** when your mission algorithm is done. Triggers the `FINISHING` state — the vehicle interface sends `MISSION_FINISHED` to the VCU and waits for `AS_FINISHED`.

```python
from std_msgs.msg import Bool
done_pub.publish(Bool(data=True))
```

Only the first `true` matters. Publishing `false` is ignored.

---

### `/vehicle/estop` — `std_msgs/Bool`

Publish `true` to command the EBS immediately from software. **Latched — once triggered it cannot be un-triggered.** The VCU will transition to `AS_EMERGENCY_BRAKE` and requires a full power cycle before the next run.

**When to use this:**

| Situation | Rule |
|---|---|
| Static Inspection B | Stack must deliberately trigger EBS after spinning to 50 rpm |
| Autonomous Demo | Stack must deploy EBS at the end of the demo run |
| Sensor loss | Rule T4.2.2 — if any required sensor signal is lost, trigger EBS immediately. Judges may ask to see source code proving this. |

```python
from std_msgs.msg import Bool
estop_pub.publish(Bool(data=True))
# From this point on, every CAN frame sends ESTOP_YES to the VCU
```

---

## What This Node Publishes (Outputs)

### `/vehicle/interface_state` — `fsai_interfaces/InterfaceState` — 100 Hz

**The most important topic for your stack.** Wait for `DRIVING` before publishing any drive commands. This is the gate.

| Field | Type | Example | Description |
|---|---|---|---|
| `state` | `uint8` | `4` | Current state. Compare using constants (see table below). |
| `state_name` | `string` | `"DRIVING"` | Human-readable name — useful for logging |

| State | Value | Meaning |
|---|---|---|
| `WAIT_FOR_VCU` | 0 | No CAN frames from VCU yet. Car not powered on. |
| `WAIT_FOR_MISSION` | 1 | VCU alive. Operator has not selected a mission on the touchscreen. |
| `MISSION_SELECTED` | 2 | Mission selected. VCU arming EBS, running 5s timer. TSAL flashing blue. |
| `WAIT_FOR_GO` | 3 | VCU armed (`AS_READY`). Waiting for safety officer to press RES GO. |
| `DRIVING` | **4** | **RES GO pressed. Drive commands forwarded to CAN. Car will move.** |
| `FINISHING` | 5 | `/vehicle/mission_complete` received. Sending `MISSION_FINISHED` to VCU. |
| `FINISHED` | 6 | VCU confirmed `AS_FINISHED`. Full power cycle required before next run. |
| `EMERGENCY` | 7 | `AS_EMERGENCY_BRAKE` triggered. Full power cycle required. |

```python
from fsai_interfaces.msg import InterfaceState

def state_cb(self, msg):
    self.safe_to_drive = (msg.state == InterfaceState.DRIVING)
```

---

### `/vcu/status` — `fsai_interfaces/VcuStatus` — 100 Hz

Full VCU status. The most important fields are `as_state` and `ami_state`.

| Field | Type | Example | Description |
|---|---|---|---|
| `header.stamp` | `time` | now | Wall clock timestamp |
| `as_state` | `uint8` | `3` | Autonomous system state (see constants below) |
| `ami_state` | `uint8` | `1` | Mission selected on the operator touchscreen (see constants below) |
| `res_go_signal` | `bool` | `true` | `true` = safety officer has pressed RES GO |
| `handshake_receive_bit` | `bool` | `true` | VCU handshake bit — mirrored back to VCU automatically |
| `fault_status` | `bool` | `false` | Always `false` — FS-AI API does not expose raw fault bits yet |
| `shutdown_request` | `bool` | `false` | VCU requesting AI to shut down |
| `shutdown_cause` | `uint8` | `0` | Reason for shutdown (see constants below) |

**`as_state` constants:**

| Constant | Value |
|---|---|
| `AS_INIT` | 0 |
| `AS_OFF` | 1 |
| `AS_READY` | 2 |
| `AS_DRIVING` | 3 |
| `AS_EMERGENCY_BRAKE` | 4 |
| `AS_FINISHED` | 5 |

**`ami_state` constants — which mission was selected:**

| Constant | Value | Your stack must... |
|---|---|---|
| `AMI_NOT_SELECTED` | 0 | Wait |
| `AMI_ACCELERATION` | 1 | 75m straight, stop within 100m, send mission complete |
| `AMI_SKIDPAD` | 2 | Figure-8 (right×2, left×2), stop within 25m |
| `AMI_AUTOCROSS` | 3 | Single lap, stop within 30m, delete run data |
| `AMI_TRACK_DRIVE` | 4 | 10 laps (count them yourself — no VCU signal), stop within 30m |
| `AMI_STATIC_INSPECTION_A` | 5 | Steering sweep, ramp to 200rpm in 10s, stop in 5s |
| `AMI_STATIC_INSPECTION_B` | 6 | Spin to 50rpm, then publish `true` to `/vehicle/estop` |
| `AMI_AUTONOMOUS_DEMO` | 7 | Steer left/right, drive 10m, stop, drive 10m, deploy EBS |

**`shutdown_cause` constants:**

| Constant | Value |
|---|---|
| `SHUTDOWN_NO_SHUTDOWN` | 0 |
| `SHUTDOWN_AI_COMPUTER_REQUEST` | 1 |
| `SHUTDOWN_HVIL_OPEN` | 2 |
| `SHUTDOWN_HVIL_SHORT` | 3 |
| `SHUTDOWN_EBS_FAULT` | 4 |
| `SHUTDOWN_OFFBOARD_CHARGER` | 5 |
| `SHUTDOWN_AI_COMMS_FAULT` | 6 |
| `SHUTDOWN_AUTONOMOUS_BRAKING` | 7 |
| `SHUTDOWN_MISSION_STATUS` | 8 |
| `SHUTDOWN_CHARGE_PROCEDURE` | 9 |
| `SHUTDOWN_BMS_FAULT` | 10 |

---

### `/vcu/wheel_speeds` — `fsai_interfaces/WheelSpeeds` — 100 Hz

All four wheel speeds and cumulative pulse counts.

| Field | Type | Example | Description |
|---|---|---|---|
| `header.stamp` | `time` | now | Wall clock timestamp |
| `fl_rpm` | `float32` | `312.4` | Front-left axle RPM |
| `fr_rpm` | `float32` | `311.9` | Front-right axle RPM |
| `rl_rpm` | `float32` | `314.2` | Rear-left axle RPM |
| `rr_rpm` | `float32` | `313.8` | Rear-right axle RPM |
| `fl_pulse_count` | `uint16` | `4821` | Front-left cumulative pulse count — rolls over at 65535 |
| `fr_pulse_count` | `uint16` | `4819` | Front-right cumulative pulse count |
| `rl_pulse_count` | `uint16` | `4825` | Rear-left cumulative pulse count |
| `rr_pulse_count` | `uint16` | `4823` | Rear-right cumulative pulse count |

> **Lap counting (Trackdrive):** The VCU sends no lap signal. Use pulse count delta or position-based detection to count 10 laps yourself.

> **Odometry:** Pulse count delta × wheel circumference gives distance travelled per wheel. Average the rear two for straight-line odometry.

---

### `/vcu/steering` — `std_msgs/Float32` — 100 Hz

Actual steering angle reported by the VCU.

| Field | Type | Example | Description |
|---|---|---|---|
| `data` | `float32` | `4.8` | Actual steering angle [deg]. Positive = left. May differ from commanded angle due to mechanical lag. |

---

### `/vcu/brake` — `std_msgs/Float32MultiArray` — 100 Hz

Actual brake pressures from the VCU.

| Index | Example | Description |
|---|---|---|
| `data[0]` | `0.0` | Front brake pressure [%] |
| `data[1]` | `0.0` | Rear brake pressure [%] |

---

### `/vcu/imu` — `sensor_msgs/Imu` — 100 Hz

IMU data from the ADS-DV onboard IMU.

| Field | Type | Example | Description |
|---|---|---|---|
| `header.frame_id` | `string` | `"imu"` | TF frame ID (configurable via `imu_frame_id` parameter) |
| `linear_acceleration.x` | `float64` | `0.12` | Forward acceleration [m/s²] |
| `linear_acceleration.y` | `float64` | `-0.05` | Lateral acceleration [m/s²] (positive = left) |
| `linear_acceleration.z` | `float64` | `9.81` | Vertical (gravity) [m/s²] |
| `angular_velocity.x` | `float64` | `0.001` | Roll rate [rad/s] |
| `angular_velocity.y` | `float64` | `0.002` | Pitch rate [rad/s] |
| `angular_velocity.z` | `float64` | `0.015` | Yaw rate [rad/s] — useful for path tracking |
| All covariance matrices | — | `-1` | Set to `-1` (unknown) — FS-AI API provides no calibration data |

> Raw values from the API are in milli-g (acceleration) and deg/s (rotation). Converted internally: milli-g × 9.80665/1000 → m/s², deg/s × π/180 → rad/s.

---

### `/vcu/gps` — `sensor_msgs/NavSatFix` — 100 Hz

GPS position from the ADS-DV onboard PCAN-GPS.

| Field | Type | Example | Description |
|---|---|---|---|
| `header.frame_id` | `string` | `"gps"` | TF frame ID (configurable via `gps_frame_id` parameter) |
| `latitude` | `float64` | `51.5074` | Decimal degrees. Negative = South. |
| `longitude` | `float64` | `-0.1278` | Decimal degrees. Negative = West. |
| `altitude` | `float64` | `12.3` | Metres above sea level |
| `status.status` | `int8` | `0` | `STATUS_NO_FIX = -1`, `STATUS_FIX = 0` |
| `position_covariance_type` | `uint8` | `0` | `COVARIANCE_TYPE_UNKNOWN` — no RTK correction |

> **Rules note:** No DGPS base station is permitted on site. GPS is rover-only — accuracy is ~3–5m. Do not rely on GPS for precise localisation during dynamic events.

---

## State Machine Quick Reference

```
Power on
    │
    ▼
WAIT_FOR_VCU ──── first CAN frame ────► WAIT_FOR_MISSION
                                               │
                                    operator selects mission
                                               │
                                               ▼
                                       MISSION_SELECTED
                                               │
                                         AS_READY (VCU arms EBS)
                                               │
                                               ▼
                                         WAIT_FOR_GO
                                               │
                                         RES GO pressed
                                               │
                                               ▼
                                           DRIVING  ◄─── drive commands forwarded
                                               │
                                    /vehicle/mission_complete = true
                                               │
                                               ▼
                                           FINISHING
                                               │
                                         AS_FINISHED
                                               │
                                               ▼
                                           FINISHED

Any state ──► EMERGENCY  if  AS_EMERGENCY_BRAKE
FINISHED / EMERGENCY ──► WAIT_FOR_MISSION  after full VCU power cycle
```

---

## Runtime Behaviour Notes

**Handshake (50ms constraint):**  
The VCU expects a handshake reply within 50ms or it fires EBS. This node runs at 100Hz (10ms per tick) — the constraint is met comfortably. Do not block the main loop or add sleep calls.

**Stale drive command:**  
If `/vehicle/drive_command` has not been received within `stale_command_timeout_ms` (default 100ms), all actuator outputs are zeroed. A `WARN` is logged at 1Hz while stale. Publishing resumes normally when fresh commands arrive — no restart needed.

**EBS latch:**  
Once `/vehicle/estop` receives `true`, `ESTOP_YES` is sent in every subsequent CAN frame permanently. The only way to clear it is a full VCU power cycle.

**Shutdown frames:**  
On node shutdown (SIGINT or process exit), the destructor sends 20 zero frames (~200ms) before closing the CAN socket. This prevents `AI_COMMS_LOST` from firing on the VCU immediately after the node exits.

**CAN interface failure:**  
`fs_ai_api_init()` is called once on startup. If the CAN interface goes down mid-run, the API background thread stops delivering data but the node does not crash — it continues publishing stale data. Restart the node to recover.
