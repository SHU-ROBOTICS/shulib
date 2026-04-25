// src/legacy_auton.cpp
//
// Implementations of the time-based auton routes. See include/legacy_auton.hpp
// for the public API and rationale.

#include "legacy_auton.hpp"

#include "drive.hpp"
#include "hardware.hpp"
#include "scoring.hpp"

#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <cstdio>

// ---------------------------------------------------------------------------
// Motor sanity test
// ---------------------------------------------------------------------------

namespace {

// Spin one drivetrain motor at ~24% of its declared direction for 800 ms.
// `signed_port` uses the PROS reversal convention (negative = reversed).
// Internal helper for autonMotorTest only.
void spinOneMotor(int signed_port, const char* label) {
  int port = signed_port > 0 ? signed_port : -signed_port;
  int dir = signed_port > 0 ? 1 : -1;
  pros::Motor m(port);
  printf("[motorTest] port %2d (%s, sign=%+d): SPIN\n", port, label, dir);
  m.move(30 * dir);
  pros::delay(800);
  m.move(0);
  pros::delay(1500);
}

}  // namespace

void autonMotorTest() {
  printf("[motorTest] ====== drivetrain individual motor test ======\n");
  printf("[motorTest] 8 motors, 800ms each at ~24%% power, 1.5s gap.\n");
  printf("[motorTest] Watch the wheels. All LEFT motors should spin the same\n");
  printf("[motorTest] direction; all RIGHT motors the opposite direction.\n");
  printf("[motorTest] If any wheel rolls wrong, note which port.\n\n");
  spinOneMotor(11,  "LEFT  1");
  spinOneMotor(-12, "LEFT  2");
  spinOneMotor(13,  "LEFT  3");
  spinOneMotor(-14, "LEFT  4");
  spinOneMotor(-16, "RIGHT 1");
  spinOneMotor(17,  "RIGHT 2");
  spinOneMotor(-18, "RIGHT 3");
  spinOneMotor(15,  "RIGHT 4 (new, post-shuffle)");
  printf("[motorTest] ====== test complete ======\n");
}

// ---------------------------------------------------------------------------
// Helper / drift verification routes
// ---------------------------------------------------------------------------

void autonTestHelpers() {
  driveInches(12);
  pros::delay(2000);
  turnDegrees(90);
  pros::delay(2000);
  driveInches(-12);
}

void autonDriftTune() {
  driveInches(48);
}

void autonJustDrive12() {
  driveInches(12);
}

// ---------------------------------------------------------------------------
// Match routes
// ---------------------------------------------------------------------------

void autonSafe() {
  driveInches(6);
}

void autonMatchOriginal() {
  printf("\n[auton] ================ MATCH AUTON START ================\n");
  pros::lcd::print(2, "auton: starting");

  // 1. Drive forward 38" (was 37; bumped to reach the wall tube cleanly).
  printf("[auton] Step 1: driving forward 38\" to approach scoring side\n");
  pros::lcd::print(2, "1: fwd 38in");
  driveInches(38);
  pros::delay(300);

  // 2. Turn 90 degrees left (CCW)
  printf("[auton] Step 2: turning 90 degrees LEFT (CCW) to face the goal\n");
  pros::lcd::print(2, "2/9: turn 90L");
  turnDegrees(-90);
  pros::delay(300);

  // 3a. Drop tongue FIRST so the pneumatic has time to fully extend before
  // the robot pushes into the scoring position.
  printf("[auton] Step 3a: dropping tongue (unloader down)\n");
  pros::lcd::print(2, "3a: tongue DOWN");
  unloader.toggle();
  pros::delay(400);

  // 3b. Drive forward to scoring position — 11" (a couple more than the
  // original 9" to reach the tube once the tongue is down).
  printf("[auton] Step 3b: driving forward 11\" into scoring position\n");
  pros::lcd::print(2, "3b: fwd 11in");
  driveInches(11);
  pros::delay(300);

  // 4a. Drive HARD into the wall once to seat the robot firmly.
  printf("[auton] Step 4a: RAMMING into the wall at max voltage for 500ms\n");
  pros::lcd::print(2, "4a: WALL RAM");
  driveStraight(127);
  pros::delay(500);
  driveStraight(0);
  pros::delay(200);

  // 4b. Load balls. Intake pulls IN at full power; conveyor moves balls
  // UP at full power. Pulsing the intake (350 on / 150 off) prevents jam
  // when too many balls arrive at once.
  // To tune: change intake_voltage (negative = reverse), conveyor_voltage,
  // or set pulse_*_ms = 0 to run continuously.
  printf("[auton] Step 4b: loading balls (full power, pulsed intake, 2.4s)\n");
  pros::lcd::print(2, "4b: intake 2.4s");
  IntakeAction load = {
    .duration_ms      = 2400,
    .intake_voltage   = 127,    // +127 = pull IN at full power
    .conveyor_voltage = 127,    // +127 = convey UP at full power
    .pulse_on_ms      = 350,
    .pulse_off_ms     = 150,
  };
  runIntake(load);
  pros::delay(300);

  // 5. Heading correction FIRST (before raising crane) so the reverse isn't
  // delayed by the pneumatic deploy time.
  printf("[auton] Step 5: heading correction -16 deg (CCW)\n");
  pros::lcd::print(2, "5: turn -16");
  turnDegrees(-16);
  pros::delay(300);

  // 6a. Raise the crane (column) UP. Stays up through reverse and unload.
  printf("[auton] Step 6a: raising crane (column UP)\n");
  pros::lcd::print(2, "6a: crane UP");
  column.toggle();
  pros::delay(500);

  // 6b. Raise the releaser flap. Stays up through reverse and unload.
  printf("[auton] Step 6b: raising releaser flap\n");
  pros::lcd::print(2, "6b: releaser UP");
  releaser.toggle();
  pros::delay(300);

  // 7. Drive backwards 9" deep into the score tube.
  printf("[auton] Step 7: reversing 9\" deep into the score tube\n");
  pros::lcd::print(2, "7: rev 9in->tube");
  driveInches(-20);
  pros::delay(300);

  // 8. Unload balls into the tube. Intake keeps pulling IN at full power
  // (clears any straggler near the bottom of the conveyor); conveyor
  // pushes balls UP at full power through the open releaser into the
  // score tube. Continuous (no pulsing) — we want everything OUT.
  printf("[auton] Step 8: UNLOADING (full power, continuous, 2s)\n");
  pros::lcd::print(2, "8: UNLOADING");
  IntakeAction unload = {
    .duration_ms      = 2000,
    .intake_voltage   = 127,    // +127 = pull IN, keeps stragglers moving up
    .conveyor_voltage = 127,    // +127 = full UP into the score tube
    .pulse_on_ms      = 0,      // 0/0 = no pulsing, run continuously
    .pulse_off_ms     = 0,
  };
  runIntake(unload);
  pros::delay(300);

  // 9. Drive backwards away from the tube.
  printf("[auton] Step 9: reversing 20\" away from tube\n");
  pros::lcd::print(2, "9: rev 20in");
  driveInches(10);
  pros::delay(300);

  // 10. Lower crane and releaser. Raise Tongue.
  printf("[auton] Step 10: lowering releaser and crane — routine complete\n");
  pros::lcd::print(2, "10: lower crane");
  releaser.toggle();
  pros::delay(300);
  column.toggle();
  pros::delay(400);
  printf("[auton] Step 10. Raising tongue)\n");
  unloader.toggle();
  pros::delay(400);

  printf("[auton] ================ MATCH AUTON DONE ================\n\n");
  pros::lcd::print(2, "auton: DONE");
}
