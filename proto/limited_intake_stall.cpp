// proto/limited_intake_stall.cpp
//
// Implementation of the stall-detecting pulsed intake. See header for
// rationale and status.

#include "limited_intake_stall.hpp"

#include "pros/rtos.hpp"

#include <cstdint>
#include <cstdio>

namespace proto {

namespace {

// Tuning knobs. Defaults chosen as reasonable starting points; iterate after
// field testing.
constexpr double STALL_THRESHOLD_RPM = 20.0;  // |velocity| < this counts as stalled
constexpr int    PULSE_ON_MS         = 350;   // max duration of a forward pulse
constexpr int    STARTUP_GRACE_MS    = 100;   // ignore velocity readings during ramp-up
constexpr int    PULSE_OFF_MS        = 150;   // pause between non-stalled pulses
constexpr int    ANTI_JAM_REVERSE_MS = 250;   // reverse duration when jam is detected
constexpr int    ANTI_JAM_VOLT       = 60;    // reverse voltage during anti-jam
constexpr int    ANTI_JAM_RECOVERY_MS = 80;   // pause after anti-jam reverse
constexpr int    INTAKE_FORWARD_VOLT = -90;   // forward (suction-in) voltage
constexpr int    CONVEYOR_VOLT       = 90;    // continuous conveyor voltage
constexpr int    POLL_INTERVAL_MS    = 40;    // velocity poll interval

}  // namespace

void limitedIntakeStall(pros::MotorGroup& intake_motors,
                        pros::MotorGroup& conveyor_motors,
                        int duration_ms) {
  conveyor_motors.move(CONVEYOR_VOLT);

  const uint32_t start = pros::millis();
  while (pros::millis() - start < static_cast<uint32_t>(duration_ms)) {
    intake_motors.move(INTAKE_FORWARD_VOLT);
    pros::delay(STARTUP_GRACE_MS);

    // Watch for a stall over the rest of the on-pulse.
    const uint32_t pulse_start = pros::millis();
    bool stalled = false;
    while (pros::millis() - pulse_start
           < static_cast<uint32_t>(PULSE_ON_MS - STARTUP_GRACE_MS)) {
      const double vel = intake_motors.get_actual_velocity();
      if (vel > -STALL_THRESHOLD_RPM && vel < STALL_THRESHOLD_RPM) {
        stalled = true;
        break;
      }
      pros::delay(POLL_INTERVAL_MS);
    }

    if (stalled) {
      printf("[limitedIntakeStall] stall detected — reversing %d ms to clear\n",
             ANTI_JAM_REVERSE_MS);
      intake_motors.move(ANTI_JAM_VOLT);
      pros::delay(ANTI_JAM_REVERSE_MS);
      intake_motors.move(0);
      pros::delay(ANTI_JAM_RECOVERY_MS);
    } else {
      intake_motors.move(0);
      pros::delay(PULSE_OFF_MS);
    }
  }

  intake_motors.move(0);
  conveyor_motors.move(0);
}

}  // namespace proto
