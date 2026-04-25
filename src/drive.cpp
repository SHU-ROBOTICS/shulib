// src/drive.cpp
//
// Implementations of the legacy time-based drive primitives. See drive.hpp
// for the public API and include/calibration.hpp for the tuning constants.

#include "drive.hpp"

#include "calibration.hpp"
#include "hardware.hpp"

#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <cstdio>

using namespace calibration;

// ---------------------------------------------------------------------------

void driveStraight(int voltage) {
  int left_cmd = voltage;
  int right_cmd = static_cast<int>(voltage * RIGHT_DRIVE_BIAS);
  if (right_cmd > 127)  right_cmd = 127;
  if (right_cmd < -127) right_cmd = -127;
  left.move(left_cmd);
  right.move(right_cmd);
}

// ---------------------------------------------------------------------------

void tempMovement(int time, int backwards) {
  uint32_t start = pros::millis();
  double batt_v = pros::battery::get_voltage() / 1000.0;
  double batt_pct = pros::battery::get_capacity();
  printf("[tempMovement] START t=%lu cmd_time=%d dir=%d batt=%.2fV (%.0f%%) right_bias=%.3f\n",
         (unsigned long)start, time, backwards, batt_v, batt_pct, RIGHT_DRIVE_BIAS);

  // Ramp-up matching: during the first 100 ms, slow the left side down AND
  // kick the right side up so both sides reach cruise at similar rates.
  // Without this, short drives have a large L/R spread because the steady-
  // state RIGHT_DRIVE_BIAS undercompensates during ramp-up.
  // Empirical: kicking right at 120V slowed it (current limit) — 115V is
  // the sweet spot. Slowing left to 55V matches right's natural pace.
  left.move(55 * backwards);
  right.move(115 * backwards);
  pros::delay(100);

  driveStraight(100 * backwards);
  uint32_t end = start + time;

  // Diagnostic: log one motor's velocity from each side to see if the bias
  // is producing balanced wheel speeds at runtime.
  pros::Motor diagLeft(11);
  pros::Motor diagRight(17);
  while (pros::millis() < end) {
    uint32_t now = pros::millis();
    double lvel = diagLeft.get_actual_velocity();
    double rvel = diagRight.get_actual_velocity();
    printf("[tempMovement] t=%lu elapsed=%lu batt=%.2fV lvel=%.1f rvel=%.1f ratio=%.3f\n",
           (unsigned long)now, (unsigned long)(now - start),
           pros::battery::get_voltage() / 1000.0,
           lvel, rvel, (lvel != 0) ? rvel / lvel : 0.0);
    pros::delay(50);
  }

  // Multi-step ramp-down: 100V -> 50 -> 25 -> 10 -> brake, over 150 ms.
  // The final 10V step (near stiction) lets motors drop to very low RPM
  // before brake mode engages so the brake-snap becomes negligible.
  driveStraight(50 * backwards);
  pros::delay(50);
  driveStraight(25 * backwards);
  pros::delay(50);
  driveStraight(10 * backwards);
  pros::delay(50);
  driveStraight(0);

  uint32_t elapsed = pros::millis() - start;
  printf("[tempMovement] STOP elapsed=%lu ms\n", (unsigned long)elapsed);
  pros::lcd::print(4, "Moved: %lu ms", (unsigned long)elapsed);
}

// ---------------------------------------------------------------------------

void tempTurn(int time, int backwards) {
  uint32_t start = pros::millis();
  double batt_v = pros::battery::get_voltage() / 1000.0;
  double batt_pct = pros::battery::get_capacity();
  printf("[tempTurn] START t=%lu cmd_time=%d dir=%d voltage=%d batt=%.2fV (%.0f%%)\n",
         (unsigned long)start, time, backwards, TURN_VOLTAGE, batt_v, batt_pct);
  chassis.arcade(0, TURN_VOLTAGE * backwards);
  uint32_t end = start + time;
  while (pros::millis() < end) {
    uint32_t now = pros::millis();
    printf("[tempTurn] t=%lu elapsed=%lu batt=%.2fV\n",
           (unsigned long)now, (unsigned long)(now - start),
           pros::battery::get_voltage() / 1000.0);
    pros::delay(50);
  }
  chassis.arcade(0, 0);
  uint32_t elapsed = pros::millis() - start;
  printf("[tempTurn] STOP elapsed=%lu ms\n", (unsigned long)elapsed);
  pros::lcd::print(4, "Turned: %lu ms", (unsigned long)elapsed);
}

// ---------------------------------------------------------------------------

void driveInches(double inches) {
  int dir = (inches >= 0) ? 1 : -1;
  double speed = (dir == 1) ? CRUISE_SPEED_FORWARD : CRUISE_SPEED_REVERSE;
  double loss  = (dir == 1) ? STARTUP_LOSS_FORWARD : STARTUP_LOSS_REVERSE;
  double magnitude = (inches >= 0) ? inches : -inches;
  int time_ms = static_cast<int>((magnitude + loss) / speed);
  printf("[driveInches] requested=%.2f dir=%d time_ms=%d (speed=%.4f loss=%.2f)\n",
         inches, dir, time_ms, speed, loss);
  tempMovement(time_ms, dir);
}

// ---------------------------------------------------------------------------

void turnDegrees(double degrees) {
  int dir = (degrees >= 0) ? 1 : -1;
  double magnitude = (degrees >= 0) ? degrees : -degrees;
  int time_ms = static_cast<int>((magnitude + STARTUP_LOSS_TURN) / CRUISE_RATE_TURN);
  printf("[turnDegrees] requested=%.2f dir=%d time_ms=%d (rate=%.3f loss=%.1f)\n",
         degrees, dir, time_ms, CRUISE_RATE_TURN, STARTUP_LOSS_TURN);
  tempTurn(time_ms, dir);
}
