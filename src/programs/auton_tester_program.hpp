#pragma once
//
// "shulib Auton Tester" -- run ONE of the team's autons on the bench, from the brain's screen, with
// no competition switch (COMP-t, 2026-09-28; the team lead, 2026-09-27: "a seperate auton program
// so that it doesnt need to be in comp mode to run it"). Built with
// `make ROBOT=tank PROGRAM=autontest`.
//
// The auton runs through EXACTLY the library path the competition program's autonomous() uses:
// the team's registry (src/team/autons.cpp), competition::runAuton, a sequence::RunGuard as the
// Chassis's pacer, the entry's own schedule, the start-pose re-seat and the guaranteed end action.
// Only the TRIGGER differs: RUN on the screen and a 3 s countdown, instead of the field. That is why
// the tester does not reverse the team lead's 2026-09-25 ruling that autons run only through the
// real autonomous() -- the call is the same; nothing here is an auton hook in another program.
//
// Everything after boot runs on the program's OWN task (the team lead's ruling for the competition
// program, "our own auton task", applied here): PROS does not delete a task the program created on
// a mode change (A4 register HA-161), so no run is ever left half-done on a deleted task's stack.
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station M).

namespace shulib::bench {

/// initialize()'s tank-autontest branch: the banner, the boot source, the robot built over the
/// PROS device provider with the RunGuard as its pacer, the team's autons registered, the boot's
/// chain proof, the SD run log opened, and the tester's task started. On any refusal it paints the
/// refusal screen, says it on serial, starts NO task and returns: nothing is powered, and nothing
/// repaints over the refusal.
void autonTesterInitialize(const char* variantBeacon, const char* competitionStatusBits);

}  // namespace shulib::bench
