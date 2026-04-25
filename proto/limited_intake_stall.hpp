// proto/limited_intake_stall.hpp
//
// PROTOTYPE: pulsed intake with stall detection and anti-jam reverse.
//
// This is a prototype version of the existing src/main.cpp limitedIntake().
// The current production version pulses intake on/off with a fixed duty cycle
// (350 ms ON / 150 ms OFF) but does not adapt when balls actually jam.
//
// This version polls intake.get_actual_velocity() during each ON pulse. If
// the motor's shaft RPM drops below STALL_THRESHOLD_RPM while we're driving
// it forward, we treat that as a jam and pulse the intake in reverse for a
// short duration to clear before resuming.
//
// Status: untested as of 2026-04-22. Ready to lift into a real module
// (e.g. src/scoring.cpp) once it's been validated on the field.
//
// Public API takes motor group references so this is portable — no global
// dependency on main.cpp's `intake`/`conveyor` declarations.

#pragma once

#include "pros/motor_group.hpp"

namespace proto {

// Run the conveyor continuously while pulsing the intake forward, with stall
// detection. If the intake stalls, briefly reverse it to clear the jam.
//
// Parameters:
//   intake_motors   — the motor group whose forward direction pulls balls in.
//                     Forward direction = `move(-voltage)` (matches the
//                     existing limitedIntake convention).
//   conveyor_motors — the motor group that pushes loaded balls upward.
//                     Forward = `move(+voltage)`.
//   duration_ms     — total time to run the loop, in milliseconds. The actual
//                     elapsed time may slightly overshoot if a stall handler
//                     is in progress when the deadline passes.
void limitedIntakeStall(pros::MotorGroup& intake_motors,
                        pros::MotorGroup& conveyor_motors,
                        int duration_ms);

}  // namespace proto
