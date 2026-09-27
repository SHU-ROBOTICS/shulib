#pragma once
//
// "shulib Calibrate" — the program in which ROBOT TWO MEASURES ITS OWN DRIVETRAIN
// (chunk R3d Part 2, 2026-09-18; its screen rewritten at R3d Part 2F-2, 2026-09-24). Built with
// `make ROBOT=tank PROGRAM=calibrate`, uploaded to slot 4 as "shulib Calibrate".
//
// The interface is one function, exactly like "shulib Drive"'s: the program IS the build,
// there is no chooser, and the operator picks it by NAME from the brain's slot list (the
// team lead's ruling, R3b Part 0b). src/main.cpp's opcontrol() calls it.
//
// IT POWERS THE MOTORS. The Part 2 version ran five starts on robot two on 2026-09-24 and
// finished none; THIS version (Part 2F-2) first ran there that evening -- run 1 STOPPED on
// SPIN_DISAGREE, run 2 finished unrecorded (docs/internal/chunks/R3d-PROGRESS.md, RUN 1 / 2).

namespace shulib::bench {

/// Run calibrations on the robot this binary was built for, one after another (RUN AGAIN on the
/// finished and the stopped screen starts the next, with a new calibrator). It runs its own
/// 10 ms loop and returns only when the field disables the robot (cleanly, so its all-stop guard
/// runs before PROS tears the task down) or when no run can start at all (a refusal painted on
/// the screen before anything is powered); a program fault inside the loop stops every motor,
/// paints its screen and idles until disabled. Every motor is left at 0 V + Coast on every C++
/// exit path. The screen shows calibration::viewFor()'s words; the USB serial port carries the
/// full record of every run, each line prefixed `[R3D]`.
void runCalibrate();

/// initialize()'s "shulib Calibrate" branch (G1 Part 2c moved it here from src/main.cpp,
/// verbatim): the `booted: shulib CALIBRATE build [<beacon>]` banner with the run status and the
/// build stamp, and the `competition status at boot:` line, from the two strings src/main.cpp
/// passes — the beacon is a literal in src/build_selection.hpp's one chain, so only the build that
/// selected this program links it. Constructs nothing and powers nothing: the program finds its own motors
/// from the brain's registry when opcontrol() runs it.
void calibrateInitialize(const char* variantBeacon, const char* competitionStatusBits);

}  // namespace shulib::bench
