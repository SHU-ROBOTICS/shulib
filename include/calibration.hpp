// include/calibration.hpp
//
// Calibration constants for the time-based dead-reckoning auton (legacy).
// All measured values from Phase 0 (forward/reverse/turn calibration) and
// Phase C (drift compensation), 2026-04-21 / 2026-04-22.
//
// These are the source of truth for everything the time-based auton uses to
// translate distance/angle commands into motor voltage and time. If a
// helper function ever uses a hardcoded number that should live here,
// move it.
//
// Re-calibrate after any of these change:
//   - Battery model swap
//   - Drivetrain mechanical change (gears, wheels, weight, tire wear)
//   - Floor / mat surface change
//   - Brake mode setting change in initialize()

#pragma once

namespace calibration {

// ---------------------------------------------------------------------------
// Turn voltage. 60 instead of 100 because at 100V the robot spins multiple
// unpredictable rotations per second (we have no IMU to count them) and
// becomes impossible to measure by eye. 60V gives a single sub-360° turn.
// Measured: 253°/s at 60V (Phase 0.4).
// ---------------------------------------------------------------------------
constexpr int TURN_VOLTAGE = 60;

// ---------------------------------------------------------------------------
// Forward / reverse drive constants.
// Linear fit: actual_distance = CRUISE_SPEED * time_ms - STARTUP_LOSS
// Inverted in driveInches(): time_ms = (distance + STARTUP_LOSS) / CRUISE_SPEED
//
// STARTUP_LOSS captures the net of ramp-up loss (~3.58") minus ramp-down gain
// (~2.78"). With the kick + ramp-down currently in tempMovement, net is ~0.8".
// Short drives (d < ~6") spend most of their time in ramp-up, so they're
// inherently less reliable; design auton with legs ≥ 8" where possible.
// ---------------------------------------------------------------------------
constexpr double CRUISE_SPEED_FORWARD = 0.0584;   // in/ms at cruise
constexpr double STARTUP_LOSS_FORWARD = 0.8;      // net inches lost to ramp

// Reverse estimated from forward × Phase 0 fwd/rev ratio (0.9795). Reverse
// has not been independently re-measured under the new brake-mode + bias
// setup; lift to its own data point if accuracy demands.
constexpr double CRUISE_SPEED_REVERSE = 0.0572;   // 0.0584 * 0.9795
constexpr double STARTUP_LOSS_REVERSE = 0.8;      // assume same as forward

// ---------------------------------------------------------------------------
// Turn rate constants (60V).
// Linear fit: actual_angle = CRUISE_RATE * time_ms - STARTUP_LOSS
// CCW is not independently calibrated — assumed symmetric with CW for now.
// CW measures ~85–90° for commanded 90° in this fit (a few-degree undershoot
// at the current values). We hit a phone-compass measurement noise floor
// around ±3° so further tuning past that point isn't reliable.
// ---------------------------------------------------------------------------
constexpr double CRUISE_RATE_TURN  = 0.410;       // deg/ms at cruise rotation
constexpr double STARTUP_LOSS_TURN = 53.0;        // deg lost to rotation ramp-up

// ---------------------------------------------------------------------------
// Drift compensation. Right-side drivetrain has intentionally different
// (slightly larger) gearing for traction, so at identical voltage the right
// wheels travel slightly less distance than the left. Without compensation
// the robot drifts left on every forward/reverse run.
//
// Tune empirically: run autonDriftTune (driveInches(48)), measure L/R
// endpoint spread, raise this if right still lags, lower if right now
// overshoots. Goal: spread <0.25" over a 48" drive.
// Only applied to straight drives (tempMovement) via driveStraight(); turns
// already need asymmetric voltage to pivot.
// ---------------------------------------------------------------------------
constexpr double RIGHT_DRIVE_BIAS = 1.08;

}  // namespace calibration
