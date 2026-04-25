// src/scoring.cpp
//
// Implementations of the intake/conveyor/scoring subroutines used by the
// legacy time-based auton. See include/scoring.hpp for the public API.

#include "scoring.hpp"

#include "hardware.hpp"

#include "pros/rtos.hpp"

#include <cstdint>

void runIntake(const IntakeAction& a) {
  // Conveyor runs continuously for the whole action.
  conveyor.move(a.conveyor_voltage);

  const bool pulsing = (a.pulse_on_ms > 0 && a.pulse_off_ms > 0);
  const uint32_t start = pros::millis();

  if (pulsing) {
    // Intake cycles on (a.intake_voltage) and off (0) while the conveyor
    // continues. Loop ends when we hit duration_ms; one in-flight pulse can
    // overshoot the deadline by up to (pulse_on + pulse_off) ms.
    while (pros::millis() - start < static_cast<uint32_t>(a.duration_ms)) {
      intake.move(a.intake_voltage);
      pros::delay(a.pulse_on_ms);
      intake.move(0);
      pros::delay(a.pulse_off_ms);
    }
  } else {
    // Continuous intake for the full duration.
    intake.move(a.intake_voltage);
    pros::delay(a.duration_ms);
  }

  intake.move(0);
  conveyor.move(0);
}

void tubeFunction(void* params) {
  tubeParams* args = static_cast<tubeParams*>(params);
  int time = args->time;
  int power = args->power;

  for (int i = 0; i < 8; i++) {
    chassis.arcade(power, 0);
    pros::delay(time);
    chassis.arcade(-20, 0);
    pros::delay(time);
  }

  chassis.arcade(0, 0);
  pros::delay(100);
}
