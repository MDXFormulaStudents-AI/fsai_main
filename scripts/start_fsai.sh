#!/bin/bash
# MDX FSAI — headless stack launcher
# Launched by fsai-stack.service on boot as user mdxfsai.

set -e

source /opt/ros/humble/setup.bash
source /home/mdxfsai/fsai_main/fsai_ros2_ws/install/setup.bash

echo "[fsai] Launching vehicle interface (can2)..."
ros2 launch fsai_vehicle_interface vehicle_interface.launch.py can_interface:=can2 &
VIF_PID=$!

sleep 4

echo "[fsai] Launching mission control..."
ros2 launch fsai_mission_control mission_control.launch.py &
MC_PID=$!

# Wait for either process to exit, then kill the other
wait -n $VIF_PID $MC_PID
echo "[fsai] A node exited — shutting down remaining processes..."
kill $VIF_PID $MC_PID 2>/dev/null || true
wait $VIF_PID $MC_PID 2>/dev/null || true
