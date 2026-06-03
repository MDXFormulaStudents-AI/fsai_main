# fsai_interfaces/msg/InterfaceState.msg
#
# Internal state machine state of fsai_vehicle_interface.
# Published on /vehicle/interface_state at 100 Hz.
#
# Python usage example:
#   from fsai_interfaces.msg import InterfaceState
#   if msg.state == InterfaceState.DRIVING:
#       # safe to publish DriveCommand

# --- State constants ---
uint8 WAIT_FOR_VCU      = 0   # No VCU CAN frames received yet
uint8 WAIT_FOR_MISSION  = 1   # VCU alive, waiting for operator to select mission
uint8 MISSION_SELECTED  = 2   # Mission selected, waiting for AS_READY
uint8 WAIT_FOR_GO       = 3   # AS_READY, waiting for RES GO signal
uint8 DRIVING           = 4   # AS_DRIVING — drive commands are forwarded to CAN
uint8 FINISHING         = 5   # Mission complete signal received, waiting for AS_FINISHED
uint8 FINISHED          = 6   # AS_FINISHED — full VCU reset required before next run
uint8 EMERGENCY         = 7   # AS_EMERGENCY_BRAKE — full VCU reset required

uint8  state            # Current state. Use constants above.
string state_name       # Human-readable state name for logging (e.g. "DRIVING")
