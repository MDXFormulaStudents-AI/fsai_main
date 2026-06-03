# fsai_interfaces/msg/WheelSpeeds.msg
#
# Wheel speed and pulse count data published by fsai_vehicle_interface
# on /vcu/wheel_speeds at 100 Hz.
#
# Wheel speeds are in axle RPM (not motor RPM).
# Pulse counts are cumulative 16-bit counters that roll over at 65535.

std_msgs/Header header

float32 fl_rpm             # Front-left wheel speed [rpm]
float32 fr_rpm             # Front-right wheel speed [rpm]
float32 rl_rpm             # Rear-left wheel speed [rpm]
float32 rr_rpm             # Rear-right wheel speed [rpm]

uint16 fl_pulse_count      # Front-left cumulative pulse count
uint16 fr_pulse_count      # Front-right cumulative pulse count
uint16 rl_pulse_count      # Rear-left cumulative pulse count
uint16 rr_pulse_count      # Rear-right cumulative pulse count
