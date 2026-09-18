#pragma once
//
// "shulib Calibrate" — the program in which ROBOT TWO MEASURES ITS OWN DRIVETRAIN
// (chunk R3d Part 2, 2026-09-18). Built with `make ROBOT=tank PROGRAM=calibrate`,
// uploaded to slot 4 as "shulib Calibrate".
//
// The interface is one function, exactly like "shulib Drive"'s: the program IS the build,
// there is no chooser, and the operator picks it by NAME from the brain's slot list (the
// team lead's ruling, R3b Part 0b). src/main.cpp's opcontrol() calls it.
//
// IT POWERS THE MOTORS. It has NEVER RUN ON HARDWARE.

namespace shulib::bench {

/// Run one calibration, start to finish, on the robot this binary was built for. Never
/// returns while driver control lasts: it runs its own 10 ms loop, exits cleanly on a field
/// DISABLE (so its all-stop guard runs before PROS tears the task down), and leaves every
/// motor at 0 V + Coast on every C++ exit path. Everything it shows on the brain's screen it
/// also prints to the USB serial port with a `[R3D]` prefix.
void runCalibrate();

}  // namespace shulib::bench
