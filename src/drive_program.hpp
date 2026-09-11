#pragma once
//
// The DRIVE PROGRAM for robot two (chunk R3b Part 0b, 2026-09-10): a program that just
// drives, from the sticks, through the library's hal/pros ADAPTERS with open-loop volts.
//
// Built as its own program -- `make ROBOT=tank PROGRAM=drive`, uploaded to its own slot as
// "shulib Drive" (team lead's ruling, 2026-09-10: the operator picks a program by name from
// the brain's slot list; a "Drive" that hid the tester behind a timeout would be one more
// thing to explain at a field). The bench tester (src/bench_r3a.cpp, "Bench Tests") stays the
// wheels-up diagnosis tool in its own slot; both share ONE chassis table
// (src/chassis_table.hpp) so they cannot disagree about which robot they are on.
//
// WHAT IT IS NOT: the library's motion stack. No Chassis, no MotionScheduler, no odometry,
// no heading -- those are R3b Parts 1-3, and the banner at boot says so. The library has
// still not driven a robot; this program drives one through the library's adapters.
//
// This lives in src/, not include/shulib/, deliberately: it makes raw <pros/*> calls for
// the brain screen, the competition state and the tick delay, and include/shulib/ is
// PROS-free by CI guard. Its PURE pieces -- the degradation policy and the coupled-side
// monitor -- are in include/shulib/teleop/ and host-tested.

namespace shulib::bench {

/// The whole drive program. Banner (serial + SD), one ProsMotor per table port inside a try
/// each (a port that refuses is a WARNING and the program drives on without it, under the
/// pure degradation policy), ProsController master, ProsLineDisplay, ProsBattery; then a
/// 10 ms loop: sticks -> teleop::mapSticks -> teleop::tankSideVolts at 12 V -> every member of
/// each side. No dead-man. Two 1-second NON-FATAL cuts that log, count and re-arm: a persisted
/// coupled-side disagreement (the shared monitor) and any member over 2.4 A for 250 ms.
/// Runtime dead-port detection marks a member ABSENT and re-evaluates the policy. Every exit
/// path -- including a field DISABLE, polled each tick -- leaves every motor at 0 V, coast.
/// Returns only when the field disables the robot or the table refuses; PROS calls opcontrol()
/// again when driver control resumes.
void runDrive();

}  // namespace shulib::bench
