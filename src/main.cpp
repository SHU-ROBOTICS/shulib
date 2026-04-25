// src/main.cpp
//
// PROS lifecycle entry points (initialize/disabled/competition_initialize/
// autonomous/opcontrol) + the auton selector + driver-control loop.
// Everything else is in modules:
//
//   include/calibration.hpp     — tuning constants
//   include/hardware.hpp        — extern decls; defs in src/hardware.cpp
//   include/drive.hpp           — drive primitives; impl in src/drive.cpp
//   include/scoring.hpp         — intake/conveyor helpers; impl in src/scoring.cpp
//   include/legacy_auton.hpp    — time-based auton routes; impl in src/legacy_auton.cpp
//   proto/                      — staged prototypes, not compiled into binary
//
// opcontrol stays here intentionally — known-working driver-control code
// is too valuable to risk a refactor right before competition.

#include "main.h"
#include "pros/misc.h"

#include "hardware.hpp"
#include "legacy_auton.hpp"

void initialize() {
	pros::lcd::initialize();
	pros::delay(50);

	// Brake mode = "brake" (not the default "coast"): motors apply electrical
	// braking on voltage=0 instead of free-spinning. Asymmetric sides decelerate
	// together via back-EMF, eliminating the gearbox-tension "hard turn" seen
	// on sudden stops with mismatched gearing.
	left.set_brake_mode_all(pros::MotorBrake::brake);
	right.set_brake_mode_all(pros::MotorBrake::brake);

	pros::lcd::print(0, "Batt: %.2f V (%.0f%%)",
	                 pros::battery::get_voltage() / 1000.0,
	                 pros::battery::get_capacity());
	pros::lcd::set_text(1, "Calypso ready");
}

void disabled() {}

void competition_initialize() {}

// ============================================================================
// AUTON SELECTION — uncomment exactly ONE line below.
// New autons (e.g. the future sensor-driven Skills routine) go in their own
// module and add one new line here.
// ============================================================================
void autonomous() {
  autonMatchOriginal();    // Match auton per user's route spec (2026-04-22)
  // autonTestHelpers();
  // autonJustDrive12();
  // autonDriftTune();
  // autonMotorTest();
  // autonSafe();
}

// ============================================================================
// DRIVER CONTROL — left untouched on purpose. Working today, don't refactor.
// ============================================================================

pros::Controller c(pros::E_CONTROLLER_MASTER);

void externalControls(){
  if (c.get_digital(DIGITAL_R1)) {
    intake.move(-127);
    conveyor.move(127);
  } else {
    if (c.get_digital(DIGITAL_R2)) {
      intake.move(127);
      conveyor.move(-127);
    } else {
		  intake.move(0);
      conveyor.move(0);
    }
  }

  if(c.get_digital_new_press(DIGITAL_RIGHT)){
    unloader.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_Y)){
    releaser.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_L1)){
	descore.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_L2)){
	column.toggle();
  }
}

void opcontrol() {
	while (true) {
		double vert = c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double rotate = c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(vert, rotate);

		externalControls();

    	pros::lcd::print(0, "Batt: %.2f V (%.0f%%)",
    	                 pros::battery::get_voltage() / 1000.0,
    	                 pros::battery::get_capacity());

		pros::delay(10);
	}
}