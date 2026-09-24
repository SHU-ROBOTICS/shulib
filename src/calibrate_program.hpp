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
// finished none; THIS version (Part 2F-2) has NEVER RUN ON HARDWARE.

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

}  // namespace shulib::bench
