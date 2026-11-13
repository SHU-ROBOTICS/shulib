#pragma once
//
// "shulib Tune" — LIVE TUNING of the robot's MECHANISMS on the brain screen (chunk TUNE1-B1,
// 2026-11-10). Built with `make ROBOT=<any robot whose mechanism table is not empty> PROGRAM=tune`,
// uploaded to slot 7 as "shulib Tune". The program itself is the LIBRARY's
// (include/shulib/mechanism/tune_program.hpp, on a V5 through include/shulib/hal/pros/tune.hpp): this
// file and its .cpp only hand it the build's mechanism table, kMechanisms — so a robot gets Tune by
// registering its table (src/robots/mechanism_table.hpp), with no line of its own HERE. The build names
// each robot once, as for every program: its ROBOT value in the Makefile's MECHANISM_ROBOTS, a
// `<robot> && SHULIB_PROGRAM_TUNE` branch in src/build_selection.hpp's chain and its define in that
// file's Tune guard, and a `<robot>-tune` build in tools/src_build_gate.py's tables (the src build gate
// fails a robot with mechanisms missing from any of them — its step 0, or the guard's #error in its build).
//
// IT POWERS THE MECHANISMS (never the drive: it makes no drive motor). Nothing is made before READY on
// its first screen; ANY controller button stops every mechanism. It has not run on a robot.

namespace shulib::bench {

/// initialize()'s "shulib Tune" branch: the boot lines (the beacon, the build stamp, the competition
/// status in words) through the program's own log — terminal and /usd/tune_log.txt. Makes nothing.
void tuneInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// opcontrol()'s: the Tune program over the build's mechanisms. Returns when VEXos reports the robot
/// DISABLED or autonomous starting (every mechanism released first); PROS calls it again when driver
/// control resumes, and it carries on.
void runTune();

}  // namespace shulib::bench
