// include/legacy_auton.hpp
//
// Auton routes for the time-based dead-reckoning auton (the "legacy" auton,
// per CLAUDE.md — kept until the sensor-driven Skills routine is field-
// validated). Each route is a self-contained function that the selector in
// src/main.cpp's autonomous() picks one of.
//
// To add a new route: declare it here, define it in src/legacy_auton.cpp,
// and add a line to autonomous() in main.cpp. New autons that need new
// hardware (IMU, AI Vision) should go in their own module — keep this one
// for time-based work only.

#pragma once

// ---------------------------------------------------------------------------
// Test / diagnostic routes
// ---------------------------------------------------------------------------

// Spin each drivetrain motor individually for ~24% power, one at a time, with
// a pause between motors. Used for hardware sanity (Phase A): all left-side
// motors should spin in the same direction; all right-side motors in the
// opposite. Watch the wheels and note any that go the wrong way.
void autonMotorTest();

// Drive 12" forward, turn 90° CW, drive 12" back. Used to verify the
// driveInches/turnDegrees helpers are calibrated correctly.
void autonTestHelpers();

// Drive 48" forward, no turns. Used to measure L/R endpoint spread and
// tune calibration::RIGHT_DRIVE_BIAS.
void autonDriftTune();

// Single isolated 12" forward drive with no rotation or return leg. Used
// when you need unlimited time to measure where the robot lands.
void autonJustDrive12();

// ---------------------------------------------------------------------------
// Match routes
// ---------------------------------------------------------------------------

// Conservative fallback: drive 6" forward, stop. Use when paired with an
// unpredictable alliance partner or on an unknown field.
void autonSafe();

// The current scoring auton. Drives to the tube, drops the tongue, rams the
// wall, loads balls, raises the crane + releaser, reverses into the score
// tube, pulses the unload, reverses away, and lowers the crane/releaser.
void autonMatchOriginal();
