#!/usr/bin/env python3
"""
VCU Simulator — runs on the laptop connected via USB CAN.

Pretends to be the ADS-DV VCU. Sends VCU2AI CAN frames at 100Hz and
reads back AI2VCU frames from the Jetson running fsai_vehicle_interface.
Walks through the full ADS-DV handshake sequence interactively.

Setup on the laptop
-------------------
1. Install python-can:
       pip install python-can

2. Bring up the USB CAN interface (adjust interface name as needed):
       sudo ip link set can0 up type can bitrate 500000
   OR for a CANable/slcan device:
       sudo slcan_attach -f -s6 -o /dev/ttyACM0
       sudo slcand ttyACM0 can0
       sudo ip link set can0 up

3. Run:
       python3 vcu_simulator.py
   OR specify a different interface:
       python3 vcu_simulator.py --interface can0
       python3 vcu_simulator.py --interface PCAN_USBBUS1 --bustype pcan

CAN frame reference (from fs-ai_api.c)
---------------------------------------
VCU sends (laptop → Jetson):
  0x520  VCU2AI_Status       8 bytes
    data[0] bit0 = HANDSHAKE_RECEIVE_BIT
    data[1] bit3 = RES_GO_SIGNAL
    data[2] bits0-3 = AS_STATE
    data[2] bits4-7 = AMI_STATE

  0x521  VCU2AI_Drive_F      8 bytes
    bytes[4:6] uint16 LE = FRONT_AXLE_TORQUE_MAX_raw  (÷10 = Nm)

  0x522  VCU2AI_Drive_R      8 bytes
    bytes[4:6] uint16 LE = REAR_AXLE_TORQUE_MAX_raw   (÷10 = Nm)

  0x523  VCU2AI_Steer        4 bytes
    bytes[0:2] int16  LE = STEER_ANGLE_raw             (÷10 = deg)
    bytes[2:4] uint16 LE = STEER_ANGLE_MAX_raw         (÷10 = deg)

  0x524  VCU2AI_Brake        2 bytes
    byte[0] = BRAKE_PRESS_F_raw   (÷2 = %)
    byte[2] = BRAKE_PRESS_R_raw

  0x525  VCU2AI_Wheel_speeds 8 bytes
    bytes[0:2] uint16 LE = FL rpm
    bytes[2:4] uint16 LE = FR rpm
    bytes[4:6] uint16 LE = RL rpm
    bytes[6:8] uint16 LE = RR rpm

  0x526  VCU2AI_Wheel_counts 8 bytes
    bytes[0:2] uint16 LE = FL pulse count
    ...same pattern for FR, RL, RR

Jetson sends (Jetson → laptop):
  0x510  AI2VCU_Status       8 bytes
    data[0] bit0  = HANDSHAKE_SEND_BIT
    data[1] bit0  = ESTOP_REQUEST       (1=YES)
    data[1] bits4-5 = MISSION_STATUS    (0=NOT_SEL,1=SEL,2=RUNNING,3=FINISHED)
    data[1] bits6-7 = DIRECTION_REQUEST (0=NEUTRAL,1=FORWARD)

  0x511  AI2VCU_Drive_F      4 bytes
    bytes[0:2] uint16 LE = FRONT_AXLE_TRQ_REQUEST_raw (÷10 = Nm)
    bytes[2:4] uint16 LE = FRONT_MOTOR_SPEED_MAX_rpm

  0x512  AI2VCU_Drive_R      4 bytes  (same layout as Drive_F, rear)

  0x513  AI2VCU_Steer        2 bytes
    bytes[0:2] int16 LE = STEER_REQUEST_raw (÷10 = deg)

  0x514  AI2VCU_Brake        2 bytes
    byte[0] = HYD_PRESS_F_REQ_raw (÷2 = %)
    byte[1] = HYD_PRESS_R_REQ_raw
"""

import argparse
import struct
import sys
import termios
import threading
import time
import tty

try:
    import can
except ImportError:
    print("ERROR: python-can not installed. Run:  pip install python-can")
    sys.exit(1)

# ── CAN IDs ───────────────────────────────────────────────────────────────────

# VCU → AI  (we send these)
VCU2AI_STATUS_ID       = 0x520
VCU2AI_DRIVE_F_ID      = 0x521
VCU2AI_DRIVE_R_ID      = 0x522
VCU2AI_STEER_ID        = 0x523
VCU2AI_BRAKE_ID        = 0x524
VCU2AI_WHEEL_SPEEDS_ID = 0x525
VCU2AI_WHEEL_COUNTS_ID = 0x526

# AI → VCU  (we read these)
AI2VCU_STATUS_ID  = 0x510
AI2VCU_DRIVE_F_ID = 0x511
AI2VCU_DRIVE_R_ID = 0x512
AI2VCU_STEER_ID   = 0x513
AI2VCU_BRAKE_ID   = 0x514

# ── State enums ───────────────────────────────────────────────────────────────

AS_STATE = {
    0: 'AS_INIT',
    1: 'AS_OFF',
    2: 'AS_READY',
    3: 'AS_DRIVING',
    4: 'AS_EMERGENCY_BRAKE',
    5: 'AS_FINISHED',
}

AMI_STATE = {
    0: 'NOT_SELECTED',
    1: 'ACCELERATION',
    2: 'SKIDPAD',
    3: 'AUTOCROSS',
    4: 'TRACK_DRIVE',
    5: 'STATIC_INSPECTION_A',
    6: 'STATIC_INSPECTION_B',
    7: 'AUTONOMOUS_DEMO',
}

MISSION_STATUS = {
    0: 'NOT_SELECTED',
    1: 'SELECTED',
    2: 'RUNNING',
    3: 'FINISHED',
}

DIRECTION = {0: 'NEUTRAL', 1: 'FORWARD'}

# ── Shared state ──────────────────────────────────────────────────────────────

vcu_state = {
    'handshake_bit':  0,
    'res_go':         0,
    'as_state':       1,   # AS_OFF
    'ami_state':      0,   # NOT_SELECTED
    'wheel_rpm':      0,
    'pulse_count':    0,
    'steer_angle':    0,   # raw (÷10 = deg)
    'steer_max':      272, # raw (÷10 = 27.2 deg)
    'torque_max':     2000,# raw (÷10 = 200 Nm)
}

ai_state = {
    'handshake_bit':   0,
    'estop':           0,
    'mission_status':  0,
    'direction':       0,
    'front_torque_nm': 0.0,
    'front_rpm':       0,
    'rear_torque_nm':  0.0,
    'rear_rpm':        0,
    'steer_deg':       0.0,
    'brake_f_pct':     0.0,
    'brake_r_pct':     0.0,
    'rx_count':        0,
}

lock = threading.Lock()
running = True

# ── Frame builders ────────────────────────────────────────────────────────────

def build_vcu2ai_status(s):
    data = bytearray(8)
    data[0] = s['handshake_bit'] & 0x01
    data[1] = (s['res_go'] & 0x01) << 3
    data[2] = (s['as_state'] & 0x0F) | ((s['ami_state'] & 0x0F) << 4)
    return bytes(data)

def build_vcu2ai_drive(torque_max_raw):
    data = bytearray(8)
    struct.pack_into('<H', data, 4, torque_max_raw)
    return bytes(data)

def build_vcu2ai_steer(angle_raw, max_raw):
    data = bytearray(4)
    struct.pack_into('<h', data, 0, angle_raw)
    struct.pack_into('<H', data, 2, max_raw)
    return bytes(data)

def build_vcu2ai_brake():
    return bytes(2)  # zero brake

def build_vcu2ai_wheel_speeds(rpm):
    data = bytearray(8)
    for i in range(4):
        struct.pack_into('<H', data, i*2, rpm)
    return bytes(data)

def build_vcu2ai_wheel_counts(count):
    data = bytearray(8)
    for i in range(4):
        struct.pack_into('<H', data, i*2, count & 0xFFFF)
    return bytes(data)

# ── Frame parser ──────────────────────────────────────────────────────────────

def parse_ai2vcu(msg):
    with lock:
        if msg.arbitration_id == AI2VCU_STATUS_ID:
            ai_state['handshake_bit']  = msg.data[0] & 0x01
            ai_state['estop']          = msg.data[1] & 0x01
            ai_state['mission_status'] = (msg.data[1] >> 4) & 0x03
            ai_state['direction']      = (msg.data[1] >> 6) & 0x03
            ai_state['rx_count']      += 1

        elif msg.arbitration_id == AI2VCU_DRIVE_F_ID:
            trq_raw = struct.unpack_from('<H', msg.data, 0)[0]
            rpm     = struct.unpack_from('<H', msg.data, 2)[0]
            ai_state['front_torque_nm'] = trq_raw / 10.0
            ai_state['front_rpm']       = rpm

        elif msg.arbitration_id == AI2VCU_DRIVE_R_ID:
            trq_raw = struct.unpack_from('<H', msg.data, 0)[0]
            rpm     = struct.unpack_from('<H', msg.data, 2)[0]
            ai_state['rear_torque_nm'] = trq_raw / 10.0
            ai_state['rear_rpm']       = rpm

        elif msg.arbitration_id == AI2VCU_STEER_ID:
            raw = struct.unpack_from('<h', msg.data, 0)[0]
            ai_state['steer_deg'] = raw / 10.0

        elif msg.arbitration_id == AI2VCU_BRAKE_ID:
            ai_state['brake_f_pct'] = msg.data[0] / 2.0
            ai_state['brake_r_pct'] = msg.data[1] / 2.0

# ── Threads ───────────────────────────────────────────────────────────────────

def tx_thread(bus):
    """Send all VCU frames at 100 Hz."""
    while running:
        t_start = time.monotonic()
        with lock:
            s = dict(vcu_state)

        try:
            bus.send(can.Message(arbitration_id=VCU2AI_STATUS_ID,
                                 data=build_vcu2ai_status(s), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_DRIVE_F_ID,
                                 data=build_vcu2ai_drive(s['torque_max']), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_DRIVE_R_ID,
                                 data=build_vcu2ai_drive(s['torque_max']), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_STEER_ID,
                                 data=build_vcu2ai_steer(s['steer_angle'], s['steer_max']), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_BRAKE_ID,
                                 data=build_vcu2ai_brake(), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_WHEEL_SPEEDS_ID,
                                 data=build_vcu2ai_wheel_speeds(s['wheel_rpm']), is_extended_id=False))
            bus.send(can.Message(arbitration_id=VCU2AI_WHEEL_COUNTS_ID,
                                 data=build_vcu2ai_wheel_counts(s['pulse_count']), is_extended_id=False))

            with lock:
                vcu_state['pulse_count'] += 1
                vcu_state['handshake_bit'] ^= 1  # toggle handshake every frame

        except can.CanError as e:
            print(f"\n[TX ERROR] {e}")

        elapsed = time.monotonic() - t_start
        time.sleep(max(0, 0.01 - elapsed))


def rx_thread(bus):
    """Receive and decode AI2VCU frames."""
    while running:
        try:
            msg = bus.recv(timeout=0.1)
            if msg:
                parse_ai2vcu(msg)
        except can.CanError as e:
            print(f"\n[RX ERROR] {e}")


def display_thread():
    """Print status every 500ms."""
    while running:
        time.sleep(0.5)
        with lock:
            v = dict(vcu_state)
            a = dict(ai_state)

        print(
            f"\r\033[K"  # clear line
            f"VCU→  AS:{AS_STATE.get(v['as_state'],'?'):20s}  "
            f"AMI:{AMI_STATE.get(v['ami_state'],'?'):20s}  "
            f"RES_GO:{v['res_go']}  "
            f"| AI→  MISSION:{MISSION_STATUS.get(a['mission_status'],'?'):12s}  "
            f"DIR:{DIRECTION.get(a['direction'],'?'):8s}  "
            f"ESTOP:{a['estop']}  "
            f"STEER:{a['steer_deg']:+6.1f}°  "
            f"TRQ:{a['front_torque_nm']:5.1f}Nm  "
            f"RPM:{a['front_rpm']:4d}  "
            f"BRAKE:{a['brake_f_pct']:4.1f}%  "
            f"RX#:{a['rx_count']}",
            end='', flush=True
        )

# ── Keyboard input ────────────────────────────────────────────────────────────

MENU = """
╔══════════════════════════════════════════════════════════════╗
║           MDX FSAI — VCU Simulator                           ║
╠══════════════════════════════════════════════════════════════╣
║  Handshake sequence:                                         ║
║    1  →  AS_OFF    (VCU alive, no mission)                   ║
║    2  →  AS_OFF  + AMI_ACCELERATION   (mission selected)     ║
║    3  →  AS_READY + AMI_ACCELERATION  (VCU armed)            ║
║    4  →  AS_DRIVING + RES_GO=1        (GO signal fired)      ║
║    5  →  AS_FINISHED                  (run complete)         ║
║    e  →  AS_EMERGENCY_BRAKE           (EBS triggered)        ║
║                                                              ║
║  AMI missions (set while in AS_OFF):                         ║
║    a  →  ACCELERATION      s  →  SKIDPAD                    ║
║    x  →  AUTOCROSS         t  →  TRACK_DRIVE                ║
║    A  →  STATIC_INSP_A     B  →  STATIC_INSP_B              ║
║    d  →  AUTONOMOUS_DEMO                                     ║
║                                                              ║
║  r  →  Reset to AS_OFF / no mission                          ║
║  q  →  Quit                                                  ║
╚══════════════════════════════════════════════════════════════╝
"""

KEY_MAP = {
    '1': ('as_state', 1, 'AS_OFF (VCU alive)'),
    '2': ('ami_state', 1, 'AMI: ACCELERATION'),
    '3': ('as_state', 2, 'AS_READY'),
    '4': ('as_state', 3, 'AS_DRIVING + RES_GO'),
    '5': ('as_state', 5, 'AS_FINISHED'),
    'e': ('as_state', 4, 'AS_EMERGENCY_BRAKE'),
    'a': ('ami_state', 1, 'AMI: ACCELERATION'),
    's': ('ami_state', 2, 'AMI: SKIDPAD'),
    'x': ('ami_state', 3, 'AMI: AUTOCROSS'),
    't': ('ami_state', 4, 'AMI: TRACK_DRIVE'),
    'A': ('ami_state', 5, 'AMI: STATIC_INSPECTION_A'),
    'B': ('ami_state', 6, 'AMI: STATIC_INSPECTION_B'),
    'd': ('ami_state', 7, 'AMI: AUTONOMOUS_DEMO'),
}


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_loop():
    global running
    print(MENU)
    print("Status: (updates every 500ms)\n")

    while running:
        ch = getch()

        if ch == 'q':
            print("\n\nQuitting...")
            running = False
            break

        elif ch == 'r':
            with lock:
                vcu_state['as_state']  = 1   # AS_OFF
                vcu_state['ami_state'] = 0   # NOT_SELECTED
                vcu_state['res_go']    = 0
                vcu_state['wheel_rpm'] = 0
            print(f"\n[RESET] AS_OFF, AMI: NOT_SELECTED")

        elif ch == '4':
            with lock:
                vcu_state['as_state']  = 3   # AS_DRIVING
                vcu_state['res_go']    = 1
                vcu_state['wheel_rpm'] = 312
            print(f"\n[→] AS_DRIVING + RES_GO=1 + wheel_rpm=312")

        elif ch in KEY_MAP:
            field, value, label = KEY_MAP[ch]
            with lock:
                vcu_state[field] = value
            print(f"\n[→] {label}")

        else:
            pass  # ignore unknown keys

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='MDX FSAI VCU Simulator')
    parser.add_argument('--interface', default='can0',
                        help='CAN interface name (default: can0)')
    parser.add_argument('--bustype', default='socketcan',
                        help='python-can bus type (default: socketcan). '
                             'Use "pcan" for PEAK PCAN-USB.')
    parser.add_argument('--bitrate', type=int, default=500000,
                        help='CAN bitrate (default: 500000)')
    args = parser.parse_args()

    print(f"Connecting to CAN bus: interface={args.interface} bustype={args.bustype} bitrate={args.bitrate}")
    try:
        bus = can.interface.Bus(channel=args.interface,
                                bustype=args.bustype,
                                bitrate=args.bitrate)
    except Exception as e:
        print(f"ERROR: Could not open CAN interface: {e}")
        print()
        print("Make sure the interface is up, e.g.:")
        print(f"  sudo ip link set {args.interface} up type can bitrate {args.bitrate}")
        sys.exit(1)

    print(f"CAN bus open. Starting threads...")

    threads = [
        threading.Thread(target=tx_thread,      args=(bus,), daemon=True),
        threading.Thread(target=rx_thread,      args=(bus,), daemon=True),
        threading.Thread(target=display_thread,              daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        keyboard_loop()
    except KeyboardInterrupt:
        global running
        running = False

    bus.shutdown()
    print("Bus closed. Done.")


if __name__ == '__main__':
    main()
