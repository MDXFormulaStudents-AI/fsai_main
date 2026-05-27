#pragma once

#include <cstdint>
#include <string>

// Vendor FS-AI API — C header, wrapped for C++ inclusion
extern "C" {
#include "fs-ai_api.h"
}

namespace fsai_vehicle_interface
{

/**
 * @brief Internal state machine states.
 *
 * Values match InterfaceState.msg constants exactly.
 * Do not change the numeric values without updating the message.
 */
enum class VehicleState : uint8_t
{
  WAIT_FOR_VCU     = 0,
  WAIT_FOR_MISSION = 1,
  MISSION_SELECTED = 2,
  WAIT_FOR_GO      = 3,
  DRIVING          = 4,
  FINISHING        = 5,
  FINISHED         = 6,
  EMERGENCY        = 7
};

/**
 * @brief Returns the human-readable name of a VehicleState.
 */
std::string vehicle_state_name(VehicleState state);

/**
 * @brief ADS-DV mission handshake state machine.
 *
 * Owns the full mission lifecycle:
 *   WAIT_FOR_VCU → WAIT_FOR_MISSION → MISSION_SELECTED → WAIT_FOR_GO
 *   → DRIVING → FINISHING → FINISHED
 *
 * Any state can transition to EMERGENCY if AS_EMERGENCY_BRAKE is detected.
 * FINISHED and EMERGENCY recover when the VCU is power-cycled (AS_OFF + AMI clear).
 *
 * Outputs:
 *   - get_mission_status() : what to send as AI2VCU_MISSION_STATUS
 *   - get_direction()      : what to send as AI2VCU_DIRECTION_REQUEST
 *   - should_forward_commands() : true only in DRIVING state
 */
class StateMachine
{
public:
  StateMachine();

  /**
   * @brief Update the state machine. Call once per main loop iteration (10 ms).
   *
   * @param vcu2ai          Latest data from fs_ai_api_vcu2ai_get_data()
   * @param mission_complete true if /vehicle/mission_complete was received this tick
   * @param has_vcu_status  true once at least one VCU2AI_Status CAN frame has arrived
   */
  void update(
    const fs_ai_api_vcu2ai & vcu2ai,
    bool mission_complete,
    bool has_vcu_status);

  VehicleState get_state() const { return state_; }
  fs_ai_api_mission_status_e get_mission_status() const { return mission_status_; }
  fs_ai_api_direction_request_e get_direction() const { return direction_; }

  /**
   * @brief Returns true only when in DRIVING state.
   * The vehicle_interface_node uses this to decide whether to forward
   * drive commands or zero all outputs.
   */
  bool should_forward_commands() const { return state_ == VehicleState::DRIVING; }

private:
  VehicleState state_;
  fs_ai_api_mission_status_e mission_status_;
  fs_ai_api_direction_request_e direction_;

  void transition_to(VehicleState new_state);
  void update_outputs();
};

}  // namespace fsai_vehicle_interface
