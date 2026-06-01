# Known Issues — MDX FSAI Stack

Issues observed during HiL testing with IPG CarMaker that interfere with correct system operation. Organised by severity.

---

## 1. AMI state does not clear on simulated power cycle

**Where:** CarMaker HiL VCU model  
**Affects:** Mission recovery after `FINISHED` or `EMERGENCY` states

**Expected behaviour (per ADS-DV User Manual, Section 6.2, step 16):**  
After a mission ends in either `AS_FINISHED` or `AS_EMERGENCY_BRAKE`, the operator turns off the LV Master Switch, ASMS, and TSMS. On power-up the VCU resets to `AS_OFF` and the touchscreen mission selection clears to `AMI_NOT_SELECTED`.

**Observed behaviour:**  
In the CarMaker simulation, toggling the simulated power switches brings the VCU back to `AS_OFF`, but the `AMI_STATE` field in the `VCU2AI_Status` CAN frame (0x520 byte 2, upper nibble) remains set to the previously selected mission. It never returns to `0x0`.

**Impact:**  
The vehicle interface state machine recovery condition requires **both** `AS_OFF` and `AMI = NOT_SELECTED` simultaneously:

```cpp
if (has_vcu_status && as_state == AS_OFF && !mission_requested) {
    transition_to(VehicleState::WAIT_FOR_MISSION);
}
```

Because AMI never clears, `mission_requested` stays `true` and the state machine is permanently stuck in `FINISHED` or `EMERGENCY` — a new mission cannot be started without restarting the node.

**Root cause:** The CarMaker ADS-DV CAN model does not reset the AMI state field when simulated power switches are toggled. This is a CarMaker model misconfiguration, not a stack bug.

**Fix:** In the CarMaker VCU CAN model, bind the AMI state output to the simulated power switch state so that AMI returns to `0` when LV Master / ASMS / TSMS are off.

---

## 2. TSAL indication skips the 5-second arming phase in simulation

**Where:** CarMaker HiL VCU model  
**Affects:** Understanding of the system startup sequence; does not block operation

**Expected behaviour (per ADS-DV User Manual, Section 6.2, steps 12–14):**  
After mission selection, the VCU transitions from `AS_Off` to `AS_Ready`. During `AS_Ready`, the TSAL should show:
- Blue flashing for a minimum of 5 seconds (the arming / standoff timer, to allow the ASR to move clear).
- Then transition to flashing Yellow, indicating the RES GO signal is possible.

A GO signal issued **before** the 5-second blue phase completes should be ignored by the VCU.

**Observed behaviour:**  
In the CarMaker simulation, as soon as a mission is selected the VCU moves to `AS_Ready` and the simulated TSAL shows flashing Yellow immediately — the 5-second Blue phase does not appear. The system behaves as if the arming timer has already elapsed.

**Impact:**  
Operationally limited — a GO signal before 5 seconds still does nothing (the real timing constraint is enforced at the VCU state machine level). However, this means the simulated startup sequence does not match what will happen on the real car, which could cause the team to send RES GO too early on track and have it ignored, causing confusion.

**Root cause:** The CarMaker VCU model does not simulate the 5-second TSAL blue arming timer. The `AS_Ready` state is entered with the GO condition immediately asserted.

**Fix:** Add a 5-second timer in the CarMaker VCU model between entering `AS_Ready` and asserting the GO-possible condition, matching the real VCU firmware behaviour.

---
