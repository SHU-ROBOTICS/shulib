// include/config.hpp
//
// Configuration for the sensor-driven auton (the new path being built on top
// of the legacy time-based auton). Port assignments, AI Vision class IDs,
// and tuning constants for IMU-driven motion live here so a config edit
// never requires hunting through multiple .cpp files.
//
// The legacy time-based auton uses include/calibration.hpp instead. Both
// header coexist — calibration.hpp covers the time-based constants, this
// file covers the sensor-driven additions.

#pragma once

namespace config {

// ---------------------------------------------------------------------------
// Smart port assignments for the new sensors. Confirmed with the user
// 2026-04-22.
// ---------------------------------------------------------------------------
constexpr int AI_VISION_PORT = 5;
constexpr int IMU_PORT       = 6;

// ---------------------------------------------------------------------------
// AI Vision class IDs (V5RC Push Back model).
// V5RC Push Back's classifier exposes only TWO classes: blue and red blocks.
// Do NOT add other classes here unless the model is reconfigured — the
// score tube, walls, and load zones are NOT classified.
//
// PROS reports the class as an integer in pros::AIVision::Object::id. The
// numeric IDs are stable for a given model and need to be confirmed via the
// AI Vision Utility live preview. Fill in once the user confirms.
// ---------------------------------------------------------------------------
constexpr int CLASS_ID_BLUE_BLOCK = 0;  // TODO: confirm from AI Vision Utility
constexpr int CLASS_ID_RED_BLOCK  = 1;  // TODO: confirm from AI Vision Utility

// ---------------------------------------------------------------------------
// IMU-driven turn parameters.
// turnToHeading() runs a P-controller on (target_heading - current_heading)
// at TURN_P_GAIN, clamped to TURN_VOLTAGE_CAP. Loop ends when the error has
// stayed below TURN_TOLERANCE_DEG for TURN_SETTLE_MS milliseconds (a settle
// window prevents the loop from declaring done while still rotating).
// ---------------------------------------------------------------------------
constexpr double TURN_P_GAIN          = 2.0;   // voltage per degree of error
constexpr int    TURN_VOLTAGE_CAP     = 80;    // hard ceiling per side
constexpr double TURN_TOLERANCE_DEG   = 1.5;   // |error| under this = arrived
constexpr int    TURN_SETTLE_MS       = 100;   // must hold for this many ms
constexpr int    TURN_TIMEOUT_MS      = 3000;  // hard cap so we never hang

// ---------------------------------------------------------------------------
// IMU-driven heading correction during straight drives.
// driveStraightWithHeading() drives forward time-based (using the legacy
// CRUISE_SPEED constants for distance->time) but actively corrects heading
// drift in real time via differential left/right voltage. The static
// RIGHT_DRIVE_BIAS used by the time-based driveStraight() is NOT applied
// here — heading feedback subsumes it.
// ---------------------------------------------------------------------------
constexpr double DRIVE_HEADING_P_GAIN = 1.5;   // voltage delta per degree of error
constexpr int    DRIVE_HEADING_LIMIT  = 30;    // max +/- shift on either side
constexpr int    DRIVE_LOOP_PERIOD_MS = 10;    // inner-loop period

}  // namespace config
