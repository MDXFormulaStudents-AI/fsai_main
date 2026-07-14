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

cleanup() {
  local code="${1:-1}"
  # Idempotency guard: both the signal trap and the wait -n path can call this.
  [ -n "$CLEANING" ] && return
  CLEANING=1
  echo "[fsai] tearing down stack..."
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

# ── 1. Vehicle interface — CAN HAL + handshake + /vcu/* ──────────────────────
launch "vehicle interface (can2)" \
  fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can2
wait_for_topic /vcu/status 30

# ── 2. Sensors — VLP-16 + ZED2i drivers ─────────────────────────────────────
launch "sensors" fsai_sensors_bringup sensors.launch.py
wait_for_topic /velodyne_points 30

# ── 3. Perception — bridge/lidar/camera/fusion → /cones ─────────────────────
launch "perception" fsai_perception perception.launch.py
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
