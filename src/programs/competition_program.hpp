#pragma once
//
// "shulib Competition" -- THE COMPETITION PROGRAM (chunk COMP-b, 2026-09-30), built with
// `make ROBOT=tank PROGRAM=competition`, uploaded to slot 2 (slot 5 stays the team's own old-code
// fallback -- T4 Part 0).
//
// What it is, in one paragraph: the robot boots from its calibration record, the auton to run is
// chosen on the brain's menu before the match (NONE, the default, runs nothing), and when the field
// -- or the controller's Timed Run -- starts the autonomous period, the chosen auton runs on the
// PROGRAM'S OWN TASK through competition::runAuton (the same call the Auton Tester makes from its
// RUN) under a run guard, and a DISABLE or the end of the period ends it at once through the
// guard's abandon latch; then driver control runs the team's driver file
// (src/team/robot_two_driver.hpp). Everything that decides the order of those events lives in
// PROS-free headers the host suite runs (competition_watch.hpp, competition_run.hpp,
// competition_screen.hpp); src/programs/competition_program.cpp builds the objects, makes the PROS
// reads, and paints. src/main.cpp's five entry points forward to the five functions below.
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station K is its first run).

namespace shulib::bench {

/// initialize()'s tank-competition branch: the banner, the boot source, the robot built over the
/// PROS device provider with the run guard as its pacer and the competition watch under it, the
/// driver file bound, the team's autons registered, the menu, the boot's chain proof, the SD run
/// log, and the program's auton task started. On any refusal it paints the refusal screen, says it
/// on serial, starts NO task and returns: nothing is powered, and nothing repaints over the
/// refusal.
void competitionInitialize(const char* variantBeacon, const char* competitionStatusBits);

/// disabled()'s: one line on entry and one on return (B5(1)), and the screen handed to the menu
/// (the auton task paints it). Commands nothing and paints nothing.
void competitionDisabled();

/// competition_initialize()'s -- PROS runs it when a field or a competition switch is connected:
/// the same as competitionDisabled(). Commands nothing and paints nothing.
void competitionConnected();

/// autonomous()'s: a request to the program's auton task, then a wait (checking every tick) for
/// its terminal word. Commands nothing itself, paints nothing, and on a refused boot only says so.
void competitionAutonomous();

/// opcontrol()'s: the wait for the auton task's terminal word (no upper bound; AUTON ENDING on the
/// screen after 0.25 s), then the team's driver file until a disable or a change to autonomous,
/// the driver page on the screen. Returns when driver control ends (PROS calls it again).
void runCompetition();

}  // namespace shulib::bench
