# fsai_interfaces/msg/VcuStatus.msg
#
# VCU status data published by fsai_vehicle_interface on /vcu/status.
# Published at 100 Hz.
#
# Python usage example:
#   from fsai_interfaces.msg import VcuStatus
#   if msg.as_state == VcuStatus.AS_DRIVING:
#       ...
#   if msg.ami_state == VcuStatus.AMI_ACCELERATION:
#       ...

# --- AS_STATE constants ---
uint8 AS_INIT              = 0
uint8 AS_OFF               = 1
uint8 AS_READY             = 2
uint8 AS_DRIVING           = 3
uint8 AS_EMERGENCY_BRAKE   = 4
uint8 AS_FINISHED          = 5

# --- AMI_STATE constants (mission selected on VCU touchscreen) ---
uint8 AMI_NOT_SELECTED         = 0
uint8 AMI_ACCELERATION         = 1
uint8 AMI_SKIDPAD              = 2
uint8 AMI_AUTOCROSS            = 3
uint8 AMI_TRACK_DRIVE          = 4
uint8 AMI_STATIC_INSPECTION_A  = 5
uint8 AMI_STATIC_INSPECTION_B  = 6
uint8 AMI_AUTONOMOUS_DEMO      = 7

# --- SHUTDOWN_CAUSE constants ---
uint8 SHUTDOWN_NO_SHUTDOWN           = 0
uint8 SHUTDOWN_AI_COMPUTER_REQUEST   = 1
uint8 SHUTDOWN_HVIL_OPEN             = 2
uint8 SHUTDOWN_HVIL_SHORT            = 3
uint8 SHUTDOWN_EBS_FAULT             = 4
uint8 SHUTDOWN_OFFBOARD_CHARGER      = 5
uint8 SHUTDOWN_AI_COMMS_FAULT        = 6
uint8 SHUTDOWN_AUTONOMOUS_BRAKING    = 7
uint8 SHUTDOWN_MISSION_STATUS        = 8
uint8 SHUTDOWN_CHARGE_PROCEDURE      = 9
uint8 SHUTDOWN_BMS_FAULT             = 10

std_msgs/Header header

uint8 as_state               # Autonomous system state. Use AS_* constants above.
uint8 ami_state              # Mission selected on touchscreen. Use AMI_* constants above.
bool  res_go_signal          # true = RES GO fired by ASR. false = NO GO.
bool  handshake_receive_bit  # Handshake bit received from VCU.

# --- Fault and warning flags ---
# Note: the FS-AI API (fs_ai_api_vcu2ai struct) does not currently expose the raw
# fault bits from the VCU2AI_Status CAN frame. These fields are always false in the
# current implementation and are reserved for future use when the API is extended.
bool fault_status
bool warning_status
bool mission_status_fault
bool ebs_fault
bool ai_comms_lost
bool bms_fault
bool autonomous_braking_fault
bool brake_plausibility_fault
bool hvil_open_fault
bool hvil_short_fault
bool charge_procedure_fault
bool offboard_charger_fault

bool  shutdown_request       # VCU requesting AI to shut down.
uint8 shutdown_cause         # Enumerated cause. Use SHUTDOWN_* constants above.
