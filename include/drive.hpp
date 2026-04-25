// include/drive.hpp
//
// Time-based drive primitives for the legacy auton. None of these read
// sensors — every motion is voltage × time using the constants in
// include/calibration.hpp.
//
// API tiers, from raw to convenient:
//   driveStraight(v)       — set both sides to a commanded voltage with the
//                            right-side bias applied. Lowest level; one call,
//                            no timing or ramping.
//   tempMovement(t, dir)   — drive at full voltage for t milliseconds, with
//                            ramp-up kick + multi-step ramp-down baked in.
//                            dir = +1 forward, -1 reverse.
//   tempTurn(t, dir)       — pivot in place at TURN_VOLTAGE for t ms.
//                            dir = +1 clockwise, -1 counter-clockwise.
//   driveInches(in)        — distance helper. Computes time from CRUISE_SPEED
//                            and STARTUP_LOSS, then calls tempMovement.
//                            Sign of in selects forward/reverse.
//   turnDegrees(deg)       — angle helper. Same idea, calls tempTurn.

#pragma once

// Lowest-level: command both drivetrain sides at a single voltage with the
// right-side drift bias applied. Clamps to [-127, 127].
void driveStraight(int voltage);

// Mid-level: timed forward/reverse drive with ramp-up + ramp-down.
//   time:      drive duration in milliseconds at full voltage (the kick and
//              ramp-down add ~250 ms of additional motion outside this).
//   backwards: +1 = forward, -1 = reverse. Any other value is undefined.
void tempMovement(int time, int backwards);

// Mid-level: timed in-place pivot at TURN_VOLTAGE.
//   time:      turn duration in milliseconds.
//   backwards: +1 = clockwise (CW, "right"), -1 = counter-clockwise.
void tempTurn(int time, int backwards);

// High-level: drive a distance in inches. Sign of inches selects direction.
// Internally maps distance -> time via calibration::CRUISE_SPEED_* and
// STARTUP_LOSS_*, then delegates to tempMovement.
void driveInches(double inches);

// High-level: turn an angle in degrees. Sign selects direction (+CW, -CCW).
// Internally maps angle -> time via calibration::CRUISE_RATE_TURN and
// STARTUP_LOSS_TURN, then delegates to tempTurn.
void turnDegrees(double degrees);
