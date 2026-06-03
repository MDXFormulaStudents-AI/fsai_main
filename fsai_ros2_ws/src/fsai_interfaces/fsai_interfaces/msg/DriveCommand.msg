# fsai_interfaces/msg/DriveCommand.msg
#
# Drive command from the MDX FSAI control node to fsai_vehicle_interface.
# Published on /vehicle/drive_command at the control node's output rate.
#
# IMPORTANT: This message is intentionally minimal.
# Mission status, direction request, e-stop, and handshake are owned
# entirely by fsai_vehicle_interface and are never set from the stack.
#
# If brake_pct > 0.0, the VCU (and this node) will zero torque.
# Brake has priority over torque — do not send both simultaneously.
#
# Commands are considered stale after stale_command_timeout_ms (default 100ms).
# Stale commands result in all outputs being zeroed immediately.

std_msgs/Header header

float32 steer_angle_deg       # Steering angle request [deg]. Positive = left.
                              # Clamped to ± steer_max reported by VCU.

float32 axle_speed_rpm        # Axle speed limit request [rpm]. Non-negative.
                              # Motor speed = axle_speed * 3.5 gear ratio, capped at 4000 rpm.

float32 axle_torque_nm        # Axle torque request [Nm]. Non-negative.
                              # Clamped to VCU torque max. Speed-dependent limit applied internally.

float32 brake_pct             # Hydraulic brake pressure request [%]. Range: 0.0 - 100.0.
                              # If > 0.0: torque_nm is ignored (brake has priority).
