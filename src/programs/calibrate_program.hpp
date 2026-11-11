#pragma once
//
// "shulib Calibrate" — the standard Calibrate (include/shulib/hal/pros/calibrate.hpp, chunk CAL1)
// with this build's robot's facts (src/robots/calibrate_facts.hpp). Built with
// `make ROBOT=tank PROGRAM=calibrate`, uploaded to slot 4. src/main.cpp's five entry points call
// these five functions, and nothing else of Calibrate's. (Until CAL1 Part B, 2026-11-05, robot two
// had a Calibrate program of its own here: R3d Part 2F-2's, which asked WHICH END after the push.)

namespace shulib::bench {

/// initialize(): the boot banner with this build's beacon (constructs nothing, powers nothing).
void calibrateInitialize(const char* variantBeacon, const char* competitionStatusBits);
/// disabled(): Calibrate's state screen.
void calibrateDisabled();
/// competition_initialize(): Calibrate's state screen.
void calibrateConnected();
/// autonomous(): Calibrate's state screen — no motion, ever.
void calibrateAutonomous();
/// opcontrol(): run calibrations until the field disables the robot.
void runCalibrate();

}  // namespace shulib::bench
