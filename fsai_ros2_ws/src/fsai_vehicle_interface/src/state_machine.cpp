#include "fsai_vehicle_interface/state_machine.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fsai_vehicle_interface
{

// ── Helpers ──────────────────────────────────────────────────────────────────

std::string vehicle_state_name(VehicleState state)
{
  switch (state) {
    case VehicleState::WAIT_FOR_VCU:     return "WAIT_FOR_VCU";
    case VehicleState::WAIT_FOR_MISSION: return "WAIT_FOR_MISSION";
    case VehicleState::MISSION_SELECTED: return "MISSION_SELECTED";
    case VehicleState::WAIT_FOR_GO:      return "WAIT_FOR_GO";
    case VehicleState::DRIVING:          return "DRIVING";
    case VehicleState::FINISHING:        return "FINISHING";
    case VehicleState::FINISHED:         return "FINISHED";
    case VehicleState::EMERGENCY:        return "EMERGENCY";
    default:                             return "UNKNOWN";
  }
}

// ── StateMachine ─────────────────────────────────────────────────────────────

StateMachine::StateMachine()
: state_(VehicleState::WAIT_FOR_VCU),
  mission_status_(MISSION_NOT_SELECTED),
  direction_(DIRECTION_NEUTRAL)
{
}

void StateMachine::transition_to(VehicleState new_state)
{
  if (new_state == state_) {
    return;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("fsai_state_machine"),
    "State: %s -> %s",
    vehicle_state_name(state_).c_str(),
    vehicle_state_name(new_state).c_str());

  state_ = new_state;
  update_outputs();
}

void StateMachine::update_outputs()
{
  switch (state_) {
    case VehicleState::WAIT_FOR_VCU:
    case VehicleState::WAIT_FOR_MISSION:
      mission_status_ = MISSION_NOT_SELECTED;
      direction_      = DIRECTION_NEUTRAL;
      break;

    case VehicleState::MISSION_SELECTED:
    case VehicleState::WAIT_FOR_GO:
      mission_status_ = MISSION_SELECTED;
      direction_      = DIRECTION_NEUTRAL;
      break;

    case VehicleState::DRIVING:
      // MISSION_RUNNING is the critical fix — the test script sent SELECTED here
      mission_status_ = MISSION_RUNNING;
      direction_      = DIRECTION_FORWARD;
      break;

    case VehicleState::FINISHING:
    case VehicleState::FINISHED:
      mission_status_ = MISSION_FINISHED;
      direction_      = DIRECTION_NEUTRAL;
      break;

    case VehicleState::EMERGENCY:
      mission_status_ = MISSION_NOT_SELECTED;
      direction_      = DIRECTION_NEUTRAL;
      break;
  }
}

void StateMachine::update(
  const fs_ai_api_vcu2ai & vcu2ai,
  bool mission_complete,
  bool has_vcu_status)
{
  const bool mission_requested =
    (vcu2ai.VCU2AI_AMI_STATE != AMI_NOT_SELECTED);

  const fs_ai_api_as_state_e as_state = vcu2ai.VCU2AI_AS_STATE;

  // ── Emergency: highest priority, overrides any state ─────────────────────
  if (has_vcu_status && as_state == AS_EMERGENCY_BRAKE) {
    if (state_ != VehicleState::EMERGENCY) {
      RCLCPP_ERROR(
        rclcpp::get_logger("fsai_state_machine"),
        "AS_EMERGENCY_BRAKE detected — zeroing all outputs");
    }
    transition_to(VehicleState::EMERGENCY);
    return;
  }

  // ── Normal state machine ──────────────────────────────────────────────────
  switch (state_) {

    case VehicleState::WAIT_FOR_VCU:
      if (has_vcu_status) {
        transition_to(VehicleState::WAIT_FOR_MISSION);
      }
      break;

    case VehicleState::WAIT_FOR_MISSION:
      if (mission_requested) {
        transition_to(VehicleState::MISSION_SELECTED);
      }
      break;

    case VehicleState::MISSION_SELECTED:
      if (!mission_requested) {
        // Operator cancelled mission on touchscreen
        RCLCPP_WARN(
          rclcpp::get_logger("fsai_state_machine"),
          "Mission cancelled by operator (AMI cleared)");
        transition_to(VehicleState::WAIT_FOR_MISSION);
      } else if (as_state == AS_DRIVING) {
        // VCU went straight to AS_DRIVING (skipped AS_READY) — handle gracefully
        RCLCPP_WARN(
          rclcpp::get_logger("fsai_state_machine"),
          "VCU entered AS_DRIVING without AS_READY — proceeding");
        transition_to(VehicleState::DRIVING);
      } else if (as_state == AS_READY) {
        transition_to(VehicleState::WAIT_FOR_GO);
      }
      break;

    case VehicleState::WAIT_FOR_GO:
      if (!mission_requested) {
        RCLCPP_WARN(
          rclcpp::get_logger("fsai_state_machine"),
          "Mission cancelled by operator (AMI cleared while in WAIT_FOR_GO)");
        transition_to(VehicleState::WAIT_FOR_MISSION);
      } else if (as_state == AS_DRIVING) {
        transition_to(VehicleState::DRIVING);
      }
      break;

    case VehicleState::DRIVING:
      if (mission_complete) {
        RCLCPP_INFO(
          rclcpp::get_logger("fsai_state_machine"),
          "Mission complete signal received — sending MISSION_FINISHED");
        transition_to(VehicleState::FINISHING);
      } else if (as_state != AS_DRIVING) {
        // VCU dropped out of AS_DRIVING without declaring emergency.
        // This should not normally happen.
        RCLCPP_WARN(
          rclcpp::get_logger("fsai_state_machine"),
          "AS_STATE left AS_DRIVING unexpectedly (as_state=%d) — returning to WAIT_FOR_GO",
          static_cast<int>(as_state));
        transition_to(VehicleState::WAIT_FOR_GO);
      }
      break;

    case VehicleState::FINISHING:
      if (as_state == AS_FINISHED) {
        RCLCPP_INFO(
          rclcpp::get_logger("fsai_state_machine"),
          "VCU confirmed AS_FINISHED");
        transition_to(VehicleState::FINISHED);
      }
      break;

    case VehicleState::FINISHED:
    case VehicleState::EMERGENCY:
      // Wait for a full VCU power cycle.
      // Recovery condition: VCU is back to AS_OFF AND AMI is cleared.
      // This only happens after the operator turns off LV Master, ASMS, and TSMS,
      // then powers back up — which resets the touchscreen mission selection.
      if (has_vcu_status && as_state == AS_OFF && !mission_requested) {
        RCLCPP_INFO(
          rclcpp::get_logger("fsai_state_machine"),
          "VCU reset detected (AS_OFF, AMI cleared) — ready for new mission");
        transition_to(VehicleState::WAIT_FOR_MISSION);
      }
      break;
  }
}

}  // namespace fsai_vehicle_interface
