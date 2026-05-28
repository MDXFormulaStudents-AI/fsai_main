# fsai_vehicle_interface — Tools

Standalone scripts for testing and validating the vehicle interface node without needing the real ADS-DV car.

---

## `vcu_simulator.py` — VCU Simulator

Runs on a **separate laptop** connected to the Jetson via a USB CAN adapter. Pretends to be the ADS-DV VCU — sends all the CAN frames the real VCU would send and reads back everything the Jetson sends in return. Lets you walk through the full ADS-DV handshake sequence and verify the state machine is working correctly.

### What it validates

- The Jetson node starts up and begins sending CAN frames immediately
- State machine transitions are correct (WAIT_FOR_VCU → WAIT_FOR_MISSION → MISSION_SELECTED → WAIT_FOR_GO → DRIVING → FINISHING → FINISHED)
- `MISSION_RUNNING` is sent (not `MISSION_SELECTED`) when `AS_DRIVING` is received — the critical bug fix from the ZF workshop
- Drive commands from ROS2 are correctly translated into CAN frames (torque, RPM, steer, brake)
- EBS can be triggered from software via `/vehicle/estop`
- Stale command timeout works — if `/vehicle/drive_command` stops publishing, outputs zero

### Hardware setup

```
Jetson (fsai_vehicle_interface)          Laptop (vcu_simulator.py)
         │                                        │
    USB CAN adapter                          USB CAN adapter
         │                                        │
         └──────── CAN cable (CANH/CANL) ─────────┘
                   120Ω terminator at each end
```

Both adapters must be at the same bitrate — **500 kbps** (ADS-DV standard).

### Dependencies

```bash
pip install python-can
```

### CAN interface setup (laptop)

**SocketCAN (Linux, most USB CAN adapters):**
```bash
sudo ip link set can0 up type can bitrate 500000
```

**PEAK PCAN-USB:**
```bash
# use bustype pcan — no ip link needed
python3 vcu_simulator.py --interface PCAN_USBBUS1 --bustype pcan
```

**CANable / slcan device:**
```bash
sudo slcan_attach -f -s6 -o /dev/ttyACM0
sudo slcand ttyACM0 can0
sudo ip link set can0 up
```

### Running

**On the Jetson first:**
```bash
sudo ip link set can0 up type can bitrate 500000
source /opt/ros/humble/setup.bash
source ~/fsai_main/fsai_ros2_ws/install/setup.bash
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can0
```

**Then on the laptop:**
```bash
python3 vcu_simulator.py
# or with a non-default interface:
python3 vcu_simulator.py --interface can0
python3 vcu_simulator.py --interface PCAN_USBBUS1 --bustype pcan
```

### Controls

```
Handshake sequence (run these in order):
  1  →  AS_OFF              VCU alive — Jetson should move to WAIT_FOR_MISSION
  2  →  AMI: ACCELERATION   Mission selected — Jetson should move to MISSION_SELECTED
  3  →  AS_READY            VCU armed, EBS 5s timer — Jetson should move to WAIT_FOR_GO
  4  →  AS_DRIVING + GO     RES GO fired — Jetson should move to DRIVING, send MISSION_RUNNING
  5  →  AS_FINISHED         Run complete — Jetson should move to FINISHED

Other:
  e  →  AS_EMERGENCY_BRAKE  Jetson should move to EMERGENCY immediately
  r  →  Reset               Back to AS_OFF, no mission
  q  →  Quit

AMI missions (set while in AS_OFF before pressing 3):
  a  →  ACCELERATION        s  →  SKIDPAD
  x  →  AUTOCROSS           t  →  TRACK_DRIVE
  A  →  STATIC_INSPECTION_A B  →  STATIC_INSPECTION_B
  d  →  AUTONOMOUS_DEMO
```

### What to look for

The status line at the bottom updates every 500ms:

```
VCU→  AS: AS_DRIVING          AMI: ACCELERATION     RES_GO: 1
AI→   MISSION: RUNNING        DIR: FORWARD    ESTOP: 0
      STEER: +5.0°  TRQ: 20.0Nm  RPM: 2800  BRAKE: 0.0%  RX#: 4821
```

| Check | Expected |
|---|---|
| After pressing `1` | `MISSION: NOT_SELECTED`, `DIR: NEUTRAL` |
| After pressing `2` | `MISSION: SELECTED`, `DIR: NEUTRAL` |
| After pressing `4` | `MISSION: RUNNING`, `DIR: FORWARD` ← critical |
| After pressing `5` | `MISSION: FINISHED`, `DIR: NEUTRAL` |
| After pressing `e` | `MISSION: NOT_SELECTED`, `DIR: NEUTRAL`, state machine → EMERGENCY |
| `RX#` counter | Incrementing at ~100/s confirms CAN frames are flowing both ways |
| `STEER/TRQ/RPM` | Non-zero when your control node publishes `/vehicle/drive_command` in DRIVING state |

On the Jetson side, watch the ROS2 logs for state transitions:
```bash
ros2 topic echo /vehicle/interface_state
```
