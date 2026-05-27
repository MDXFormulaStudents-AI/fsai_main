#!/usr/bin/env python3
"""
VCU Simulator — runs on the laptop connected via USB CAN (e.g. PEAK PCAN-USB).

Pretends to be the ADS-DV VCU. Sends VCU2AI CAN frames at 100Hz and reads
back AI2VCU frames from the Jetson running fsai_vehicle_interface.

Uses raw Linux SocketCAN — no external dependencies, standard library only.

Setup
-----
1. Load the CAN driver and bring up the interface (PEAK PCAN-USB example):
       sudo modprobe peak_usb
       sudo ip link set can0 up type can bitrate 500000
       sudo ip link set can0 txqueuelen 1000

2. Run:
       python3 vcu_simulator.py
   or with a different interface:
       python3 vcu_simulator.py --interface can0
"""

from __future__ import annotations

import argparse
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty

# ── CAN frame packing ─────────────────────────────────────────────────────────

CAN_FRAME_FMT  = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

def pack_frame(can_id: int, data: bytes) -> bytes:
    return struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b'\x00'))

def unpack_frame(raw: bytes) -> tuple[int, int, bytes]:
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    return can_id, dlc, data[:dlc]

# ── CAN IDs ───────────────────────────────────────────────────────────────────

# We send these (simulating VCU → AI)
VCU2AI_STATUS_ID       = 0x520
VCU2AI_DRIVE_F_ID      = 0x521
VCU2AI_DRIVE_R_ID      = 0x522
VCU2AI_STEER_ID        = 0x523
VCU2AI_BRAKE_ID        = 0x524
VCU2AI_WHEEL_SPEEDS_ID = 0x525
VCU2AI_WHEEL_COUNTS_ID = 0x526

# We read these (AI → VCU, from Jetson)
AI2VCU_STATUS_ID  = 0x510
AI2VCU_DRIVE_F_ID = 0x511
AI2VCU_DRIVE_R_ID = 0x512
AI2VCU_STEER_ID   = 0x513
AI2VCU_BRAKE_ID   = 0x514

# ── State enums ───────────────────────────────────────────────────────────────

AS_NAMES      = {0:'AS_INIT', 1:'AS_OFF', 2:'AS_READY', 3:'AS_DRIVING', 4:'AS_EMERGENCY_BRAKE', 5:'AS_FINISHED'}
AMI_NAMES     = {0:'NOT_SELECTED', 1:'ACCELERATION', 2:'SKIDPAD', 3:'AUTOCROSS', 4:'TRACK_DRIVE', 5:'STATIC_INSP_A', 6:'STATIC_INSP_B', 7:'AUTONOMOUS_DEMO'}
MISSION_NAMES = {0:'NOT_SELECTED', 1:'SELECTED', 2:'RUNNING', 3:'FINISHED'}
DIR_NAMES     = {0:'NEUTRAL', 1:'FORWARD'}

# ── Shared state ──────────────────────────────────────────────────────────────

vcu = {
    'as_state':    1,    # AS_OFF
    'ami_state':   0,    # NOT_SELECTED
    'res_go':      0,
    'handshake':   0,
    'wheel_rpm':   0,
    'pulse_count': 0,
    'steer_raw':   0,    # ÷10 = deg
    'steer_max':   272,  # ÷10 = 27.2 deg
    'torque_max':  2000, # ÷10 = 200 Nm
}

ai = {
    'handshake':   0,
    'estop':       0,
    'mission':     0,
    'direction':   0,
    'front_trq':   0.0,
    'front_rpm':   0,
    'rear_trq':    0.0,
    'rear_rpm':    0,
    'steer_deg':   0.0,
    'brake_f':     0.0,
    'brake_r':     0.0,
    'rx_count':    0,
}

lock    = threading.Lock()
running = True

# ── Frame builders ────────────────────────────────────────────────────────────

def build_status() -> bytes:
    with lock:
        d = bytearray(8)
        d[0] = vcu['handshake'] & 0x01
        d[1] = (vcu['res_go'] & 0x01) << 3
        d[2] = (vcu['as_state'] & 0x0F) | ((vcu['ami_state'] & 0x0F) << 4)
        return bytes(d)

def build_drive(torque_max_raw: int) -> bytes:
    d = bytearray(8)
    struct.pack_into('<H', d, 4, torque_max_raw)
    return bytes(d)

def build_steer() -> bytes:
    with lock:
        d = bytearray(4)
        struct.pack_into('<h', d, 0, vcu['steer_raw'])
        struct.pack_into('<H', d, 2, vcu['steer_max'])
        return bytes(d)

def build_wheel_speeds() -> bytes:
    with lock:
        d = bytearray(8)
        for i in range(4):
            struct.pack_into('<H', d, i*2, vcu['wheel_rpm'])
        return bytes(d)

def build_wheel_counts() -> bytes:
    with lock:
        d = bytearray(8)
        for i in range(4):
            struct.pack_into('<H', d, i*2, vcu['pulse_count'] & 0xFFFF)
        return bytes(d)

# ── Frame parser ──────────────────────────────────────────────────────────────

def parse_ai2vcu(can_id: int, data: bytes) -> None:
    with lock:
        if can_id == AI2VCU_STATUS_ID and len(data) >= 2:
            ai['handshake'] = data[0] & 0x01
            ai['estop']     = data[1] & 0x01
            ai['mission']   = (data[1] >> 4) & 0x03
            ai['direction'] = (data[1] >> 6) & 0x03
            ai['rx_count'] += 1

        elif can_id == AI2VCU_DRIVE_F_ID and len(data) >= 4:
            ai['front_trq'] = struct.unpack_from('<H', data, 0)[0] / 10.0
            ai['front_rpm'] = struct.unpack_from('<H', data, 2)[0]

        elif can_id == AI2VCU_DRIVE_R_ID and len(data) >= 4:
            ai['rear_trq']  = struct.unpack_from('<H', data, 0)[0] / 10.0
            ai['rear_rpm']  = struct.unpack_from('<H', data, 2)[0]

        elif can_id == AI2VCU_STEER_ID and len(data) >= 2:
            ai['steer_deg'] = struct.unpack_from('<h', data, 0)[0] / 10.0

        elif can_id == AI2VCU_BRAKE_ID and len(data) >= 2:
            ai['brake_f'] = data[0] / 2.0
            ai['brake_r'] = data[1] / 2.0

# ── TX / RX threads ───────────────────────────────────────────────────────────

def tx_rx_thread(interface: str) -> None:
    global running
    import socket as _socket

    sock = _socket.socket(_socket.AF_CAN, _socket.SOCK_RAW, _socket.CAN_RAW)
    sock.bind((interface,))
    sock.setblocking(False)

    next_tx = time.monotonic()

    try:
        while running:
            now = time.monotonic()

            # ── TX ──
            if now >= next_tx:
                with lock:
                    torque_max = vcu['torque_max']
                    vcu['handshake'] ^= 1
                    vcu['pulse_count'] += 1

                frames = [
                    pack_frame(VCU2AI_STATUS_ID,       build_status()),
                    pack_frame(VCU2AI_DRIVE_F_ID,      build_drive(torque_max)),
                    pack_frame(VCU2AI_DRIVE_R_ID,      build_drive(torque_max)),
                    pack_frame(VCU2AI_STEER_ID,        build_steer()),
                    pack_frame(VCU2AI_BRAKE_ID,        bytes(2)),
                    pack_frame(VCU2AI_WHEEL_SPEEDS_ID, build_wheel_speeds()),
                    pack_frame(VCU2AI_WHEEL_COUNTS_ID, build_wheel_counts()),
                ]
                for f in frames:
                    try:
                        sock.send(f)
                    except BlockingIOError:
                        pass  # TX buffer momentarily full — skip frame

                next_tx += 0.01
                if next_tx < now:
                    next_tx = now + 0.01

            # ── RX ──
            readable, _, _ = select.select([sock], [], [], max(0, next_tx - time.monotonic()))
            if readable:
                try:
                    raw = sock.recv(CAN_FRAME_SIZE)
                    can_id, _, data = unpack_frame(raw)
                    parse_ai2vcu(can_id, data)
                except Exception:
                    pass

    finally:
        sock.close()


def display_thread() -> None:
    while running:
        time.sleep(0.5)
        with lock:
            v = dict(vcu)
            a = dict(ai)

        print(
            f"\r\033[K"
            f"VCU→ AS:{AS_NAMES.get(v['as_state'],'?'):20s} "
            f"AMI:{AMI_NAMES.get(v['ami_state'],'?'):16s} "
            f"RES:{v['res_go']}  |  "
            f"AI→ MISSION:{MISSION_NAMES.get(a['mission'],'?'):12s} "
            f"DIR:{DIR_NAMES.get(a['direction'],'?'):8s} "
            f"ESTOP:{a['estop']}  "
            f"STEER:{a['steer_deg']:+5.1f}°  "
            f"TRQ:{a['front_trq']:5.1f}Nm  "
            f"RPM:{a['front_rpm']:4d}  "
            f"BRK:{a['brake_f']:4.1f}%  "
            f"RX:{a['rx_count']}",
            end='', flush=True
        )

# ── Keyboard ──────────────────────────────────────────────────────────────────

MENU = """
╔══════════════════════════════════════════════════════════╗
║          MDX FSAI — VCU Simulator                        ║
╠══════════════════════════════════════════════════════════╣
║  Handshake (run in order):                               ║
║    1 → AS_OFF          (VCU alive)                       ║
║    2 → AMI mission     (use a/s/x/t/A/B/d first)        ║
║    3 → AS_READY        (VCU armed)                       ║
║    4 → AS_DRIVING + GO (RES fired, wheels spin)          ║
║    5 → AS_FINISHED                                       ║
║    e → AS_EMERGENCY_BRAKE                                ║
║                                                          ║
║  AMI missions:                                           ║
║    a=ACCELERATION  s=SKIDPAD  x=AUTOCROSS  t=TRACKDRIVE  ║
║    A=STATIC_A      B=STATIC_B d=DEMO                     ║
║                                                          ║
║    r → reset      q → quit                               ║
╚══════════════════════════════════════════════════════════╝
"""

def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_loop() -> None:
    global running
    print(MENU)
    while running:
        ch = getch()
        if   ch == 'q': running = False; print("\nQuitting...")
        elif ch == 'r':
            with lock: vcu['as_state']=1; vcu['ami_state']=0; vcu['res_go']=0; vcu['wheel_rpm']=0
            print("\n[RESET]")
        elif ch == '1':
            with lock: vcu['as_state'] = 1
            print(f"\n[→] AS_OFF")
        elif ch == '2':
            with lock: ami = vcu['ami_state']
            print(f"\n[→] AMI confirmed: {AMI_NAMES.get(ami,'?')} — press 3 for AS_READY")
        elif ch == '3':
            with lock: vcu['as_state'] = 2
            print(f"\n[→] AS_READY")
        elif ch == '4':
            with lock: vcu['as_state']=3; vcu['res_go']=1; vcu['wheel_rpm']=312
            print(f"\n[→] AS_DRIVING + RES_GO=1")
        elif ch == '5':
            with lock: vcu['as_state'] = 5
            print(f"\n[→] AS_FINISHED")
        elif ch == 'e':
            with lock: vcu['as_state'] = 4
            print(f"\n[→] AS_EMERGENCY_BRAKE")
        elif ch == 'a':
            with lock: vcu['ami_state'] = 1
            print(f"\n[AMI] ACCELERATION")
        elif ch == 's':
            with lock: vcu['ami_state'] = 2
            print(f"\n[AMI] SKIDPAD")
        elif ch == 'x':
            with lock: vcu['ami_state'] = 3
            print(f"\n[AMI] AUTOCROSS")
        elif ch == 't':
            with lock: vcu['ami_state'] = 4
            print(f"\n[AMI] TRACK_DRIVE")
        elif ch == 'A':
            with lock: vcu['ami_state'] = 5
            print(f"\n[AMI] STATIC_INSPECTION_A")
        elif ch == 'B':
            with lock: vcu['ami_state'] = 6
            print(f"\n[AMI] STATIC_INSPECTION_B")
        elif ch == 'd':
            with lock: vcu['ami_state'] = 7
            print(f"\n[AMI] AUTONOMOUS_DEMO")

# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description='MDX FSAI VCU Simulator')
    parser.add_argument('--interface', default='can0',
                        help='SocketCAN interface name (default: can0)')
    args = parser.parse_args()

    print(f"Opening SocketCAN interface: {args.interface}")

    signal.signal(signal.SIGINT,  lambda *_: globals().update(running=False))
    signal.signal(signal.SIGTERM, lambda *_: globals().update(running=False))

    t1 = threading.Thread(target=tx_rx_thread,  args=(args.interface,), daemon=True)
    t2 = threading.Thread(target=display_thread,                         daemon=True)
    t1.start()
    t2.start()

    try:
        keyboard_loop()
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        global running
        running = False

    t1.join(timeout=1)
    print("Done.")


if __name__ == '__main__':
    main()
