#!/bin/bash
# MDX FSAI — headless full-stack launcher (real vehicle)
# Launched by fsai-stack.service on boot as user mdxfsai, or by hand over SSH.
#
# Brings up the entire autonomous stack in dependency order with readiness
# gating between stages, then blocks. If ANY component exits, the whole stack
# is torn down together (all-or-nothing). A SIGTERM/SIGINT (e.g. from
# `systemctl stop fsai-stack` or Ctrl-C) cleanly stops every launch.
#
# NOTE: do not enable `set -e` — wait_for_topic intentionally returns non-zero
# on a (non-fatal) timeout, and that must not abort the script.

source /opt/ros/humble/setup.bash
source /home/mdxfsai/fsai_main/fsai_ros2_ws/install/setup.bash

# Match the domain the interactive shells and remote monitors use. systemd does
# NOT source ~/.bashrc, so this must be set explicitly here.
export ROS_DOMAIN_ID=42

PIDS=()
CLEANING=""
WATCHDOG_PID=""
# ZED camera is launched on its own (LiDAR disabled here) so it can be restarted
# independently of the critical LiDAR path by zed_supervisor.
ZED_LAUNCH_ARGS=(fsai_sensors_bringup sensors.launch.py enable_lidar:=false)

cleanup() {
  local code="${1:-1}"
  # Idempotency guard: both the signal trap and the wait -n path can call this.
  [ -n "$CLEANING" ] && return
  CLEANING=1
  echo "[fsai] tearing down stack..."
  # Stop the ZED supervisor first (it owns + auto-restarts the non-critical
  # camera, which lives outside PIDS). SIGTERM lets its own trap gracefully tear
  # down its ZED child; the final `wait` below reaps it. (KillMode=control-group
  # is the systemd-level backstop for any straggler.)
  [ -n "$WATCHDOG_PID" ] && kill -TERM "$WATCHDOG_PID" 2>/dev/null || true
  # Graceful first: SIGINT each ros2 launch (reverse dependency order) so it
  # shuts its own nodes down in an orderly way.
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -INT "${PIDS[$i]}" 2>/dev/null || true
  done
  # Grace period for the launches to reap their node trees.
  local t=0
  while [ "$t" -lt 10 ]; do
    sleep 1; t=$((t + 1))
    local alive=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && { alive=1; break; }; done
    [ "$alive" -eq 0 ] && break
  done
  # Escalate: hard-kill any launch still standing and its direct children
  # (catches stubborn driver nodes that outlived their launch's SIGINT).
  for p in "${PIDS[@]}"; do
    pkill -KILL -P "$p" 2>/dev/null || true
    kill  -KILL    "$p" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit "$code"
}
# Operator stop (systemctl stop / Ctrl-C): clean exit, no auto-restart.
trap 'cleanup 0' SIGINT SIGTERM

# wait_for_topic <topic> [timeout_s]
# Blocks until the topic appears in the graph, or the timeout elapses. On
# timeout it WARNS and returns 1 but does NOT abort — a missing sensor must not
# prevent the rest of the stack from coming up.
wait_for_topic() {
  local topic="$1"; local timeout="${2:-30}"; local waited=0
  echo "[fsai] waiting for ${topic} (up to ${timeout}s)..."
  while ! ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    sleep 1; waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      echo "[fsai] WARNING: ${topic} not present after ${timeout}s — continuing anyway"
      return 1
    fi
  done
  echo "[fsai] ${topic} ready (${waited}s)"
}

# launch <description> <package> <launch_file> [args...]
launch() {
  local desc="$1"; shift
  echo "[fsai] launching ${desc}..."
  ros2 launch "$@" &
  PIDS+=($!)
}

# zed_supervisor — the ZED2 on a Jetson frequently fails or hangs on COLD BOOT
# (USB enumeration / CUDA not ready yet). The camera is NON-CRITICAL: perception
# degrades to LiDAR-only cones without it, so it is deliberately kept OUT of the
# all-or-nothing PIDS set (otherwise a camera crash would tear down and restart
# the entire stack). This runs as its own process — it starts the ZED and
# restarts ONLY the camera if its image topic disappears, leaving the rest of the
# stack untouched. Its SIGTERM/INT trap tears down its own ZED child on stop.
zed_supervisor() {
  local topic=/zed/zed_node/rgb/image_rect_color
  local zed_pid=""
  trap 'kill -INT "$zed_pid" 2>/dev/null; sleep 2; pkill -KILL -P "$zed_pid" 2>/dev/null; exit 0' SIGTERM SIGINT
  echo "[fsai] launching zed camera (non-critical)..."
  ros2 launch "${ZED_LAUNCH_ARGS[@]}" & zed_pid=$!
  while true; do
    sleep 15
    if ! ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "[fsai] WARNING: ZED image topic absent — restarting ZED camera"
      kill -INT "$zed_pid" 2>/dev/null || true
      sleep 2; pkill -KILL -P "$zed_pid" 2>/dev/null || true
      ros2 launch "${ZED_LAUNCH_ARGS[@]}" & zed_pid=$!
    fi
  done
}

# ── 1. Vehicle interface — CAN HAL + handshake + /vcu/* ──────────────────────
launch "vehicle interface (can2)" \
  fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can2
wait_for_topic /vcu/status 30

# ── 2. Sensors — LiDAR (critical) + ZED2i camera (non-critical, supervised) ──
# LiDAR is in the all-or-nothing set. The camera runs under zed_supervisor so it
# survives its flaky cold-boot failures without gating boot or tearing the stack
# down, and auto-restarts if it drops.
launch "lidar (VLP-16)" fsai_sensors_bringup sensors.launch.py enable_camera:=false
wait_for_topic /velodyne_points 30

zed_supervisor &
WATCHDOG_PID=$!
# Non-fatal (#1): surface a missing/late camera in the boot log, then continue.
wait_for_topic /zed/zed_node/rgb/image_rect_color 20

# ── 3. Perception — bridge/lidar/camera/fusion → /cones ─────────────────────
# env:=real  → VLP-16 + ZED2 topics (not CarMaker).
# use_sim_time:=false → live wall clock; nothing publishes /clock on the real car.
launch "perception" fsai_perception perception.launch.py env:=real use_sim_time:=false
wait_for_topic /cones 30

# ── 4. Localization — wheel+IMU odom → /odometry/vehicle + TF chain ─────────
launch "localization" fsai_localization vehicle_odometry.launch.py
wait_for_topic /odometry/vehicle 30

# ── 5. Mission stack — all 7 missions (real-car defaults baked into launch) ─
launch "mission stack" fsai_mission_control mission_bringup.launch.py

echo "[fsai] full stack up — waiting on the ADS-DV. Monitoring components."

# Block until any component exits, then tear the whole stack down. Exit non-zero
# so fsai-stack.service (Restart=on-failure) brings the stack back up.
wait -n "${PIDS[@]}"
echo "[fsai] a component exited unexpectedly — shutting down remaining processes."
cleanup 1
