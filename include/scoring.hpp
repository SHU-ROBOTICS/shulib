// include/scoring.hpp
//
// Subroutines for the intake + conveyor + scoring mechanisms during the
// legacy time-based auton.
//
// The main entry point is `runIntake(IntakeAction)`. The struct is
// designed so call sites are self-documenting — every field has a single
// clear purpose and a sign convention spelled out below.

#pragma once

// Parameters for runIntake(). All voltages are V5 motor voltage units
// (-127..127). Sign conventions are spelled out per field; flip the sign
// to reverse direction.
struct IntakeAction {
  // Total time the action runs, in milliseconds.
  int duration_ms;

  // Voltage commanded to the intake motor.
  //   positive = pull balls IN  (collect direction)
  //   negative = push balls OUT (reverse / spit)
  // 127 = full power.
  int intake_voltage;

  // Voltage commanded to the conveyor motor.
  //   positive = convey UP   (toward the score tube / unloader)
  //   negative = convey DOWN (back toward the intake)
  // 127 = full power.
  int conveyor_voltage;

  // Optional pulse pattern, applied to the intake only. The conveyor runs
  // continuously throughout. Pulsing meters ball flow into the conveyor so
  // a flood of balls doesn't jam the intake.
  //
  // Set both pulse_on_ms and pulse_off_ms to 0 to disable pulsing
  // (intake runs continuously for the full duration_ms).
  int pulse_on_ms;
  int pulse_off_ms;
};

// Run the intake + conveyor for `a.duration_ms` milliseconds with the
// commanded voltages. If pulse_on_ms and pulse_off_ms are both nonzero,
// the intake is cycled on/off accordingly; otherwise it runs continuously.
// Both motors are stopped before the function returns.
void runIntake(const IntakeAction& a);

// -------------------------------------------------------------------------
// Legacy "shake" routine. Drives the chassis back and forth 8 times. Was
// used to dislodge stuck balls during scoring; replaced by the wall-ram
// approach in the current auton. Kept here in case we want to revive it.
struct tubeParams {
  int time;
  int power;
};
void tubeFunction(void* params);
