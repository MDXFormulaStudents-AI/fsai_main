# MDX FSAI — Remote Monitoring

Monitor the Jetson's autonomous stack from another computer on the **same
network**. Nothing here commands the car — these are read-only status checks.

## 1. One-time setup on the remote machine

1. Install ROS 2 Humble (desktop) and source it:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Match the Jetson's ROS domain** (the stack runs on domain 42):
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. Be on the same LAN/subnet as the Jetson with multicast allowed (default
   DDS discovery). If your interactive Jetson shells set a custom
   `RMW_IMPLEMENTATION`, set the **same** one here — otherwise leave it default.

Sanity check that you can see the stack:
```bash
ros2 node list        # should list mission_manager, vehicle_interface, fusion, ...
ros2 topic list        # should show /vcu/status, /cones, /odometry/vehicle, ...
```
If these are empty, the domain ID, RMW, or network/firewall don't match.

## 2. Controlling the stack (over SSH to the Jetson)

```bash
ssh mdxfsai@<jetson-ip>

sudo systemctl restart fsai-stack     # restart the WHOLE stack, one command
sudo systemctl stop    fsai-stack     # stop everything
sudo systemctl start   fsai-stack     # start everything
systemctl status fsai-stack           # is it up?
journalctl -u fsai-stack -f           # live stack logs (readiness gating, crashes)
journalctl -u fsai-can   -f           # CAN bring-up logs
```

## 3. Key topics to monitor

Run each `ros2 topic echo <topic>` on the remote machine. For high-rate topics,
prefer `ros2 topic hz <topic>` to confirm it's alive without flooding the console.

### Vehicle interface / handshake
| Topic | Command | What to look for |
|-------|---------|------------------|
| `/vehicle/interface_state` | `ros2 topic echo /vehicle/interface_state` | `state_name`: WAIT_FOR_MISSION → … → DRIVING → FINISHED |
| `/vcu/status` | `ros2 topic echo /vcu/status` | `as_state`, `ami_state`, `res_go_signal`, fault flags |

### Mission routing / commands
| Topic | Command | What to look for |
|-------|---------|------------------|
| `/mission/selected` | `ros2 topic echo /mission/selected` | selected mission string (e.g. `acceleration`) |
| `/dynamic_drive_command` | `ros2 topic echo /dynamic_drive_command` | candidate cmd from the active follower |
| `/static_drive_command` | `ros2 topic echo /static_drive_command` | candidate cmd for static missions |
| `/vehicle/drive_command` | `ros2 topic echo /vehicle/drive_command` | **forwarded** cmd (only in DRIVING): steer/rpm/torque/brake |
| `/vehicle/mission_complete` | `ros2 topic echo /vehicle/mission_complete` | `true` once when the mission finishes |
| `/accel/finished` | `ros2 topic echo /accel/finished` | acceleration reached target |
| `/dynamic_mission_complete` | `ros2 topic echo /dynamic_mission_complete` | dynamic mission completion |

### Localization / odometry
| Topic | Command | What to look for |
|-------|---------|------------------|
| `/odometry/vehicle` | `ros2 topic echo /odometry/vehicle` | pose.x advancing, twist.linear.x = speed |
| TF chain | `ros2 run tf2_ros tf2_echo odom Fr1A` | odom → home → base_link → Fr1A intact |

### Perception (high-rate — use `hz`)
| Topic | Command | What to look for |
|-------|---------|------------------|
| `/cones` | `ros2 topic hz /cones` | ~20 Hz; cone array to the nav stack |
| `/velodyne_points` | `ros2 topic hz /velodyne_points` | LiDAR driver alive |

### VCU sensor feedback
| Topic | Command | What to look for |
|-------|---------|------------------|
| `/vcu/wheel_speeds` | `ros2 topic echo /vcu/wheel_speeds` | FL/FR/RL/RR rpm (0 when stationary) |
| `/vcu/imu` | `ros2 topic echo /vcu/imu` | `angular_velocity.z` = yaw rate |
| `/vcu/steering` | `ros2 topic echo /vcu/steering` | actual steering angle |
| `/vcu/brake` | `ros2 topic echo /vcu/brake` | actual brake values |

## 4. Fast health sweep

```bash
export ROS_DOMAIN_ID=42
ros2 node list                              # all nodes present?
ros2 topic hz /vcu/status                   # interface publishing (~100 Hz)?
ros2 topic hz /cones                         # perception alive (~20 Hz)?
ros2 topic hz /odometry/vehicle              # localization alive?
ros2 topic echo /vehicle/interface_state --once   # current handshake state
ros2 topic echo /mission/selected --once          # current mission
```
