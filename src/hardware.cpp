// src/hardware.cpp
//
// Definitions of every hardware global on the robot — drivetrain, intake,
// conveyor, sensors, pneumatics, and the LemLib chassis. Everything that
// physically maps to a port lives here. include/hardware.hpp has the
// matching extern declarations that other modules use.
//
// If the builder reroutes a port, change it here. If the meaning of a
// pneumatic changes, update the role comment here.

#include "hardware.hpp"

#include "config.hpp"

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

// ---------------------------------------------------------------------------
// Drivetrain
// ---------------------------------------------------------------------------
// Left side: ports 11, 12, 13, 14 (12 and 14 reversed via the negative sign).
// Right side: ports 15, 16, 17, 18 (16 and 18 reversed). Port 15 was added
// after the 19 -> 15 shuffle done by the builder.
//
// All motors use the BLUE cartridge per the declarations below, but the
// right side is mechanically geared down a touch for traction (intentional;
// see calibration::RIGHT_DRIVE_BIAS for the software-side compensation).
pros::MotorGroup left({11, -12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup right({-16, 17, -18, 15}, pros::MotorGearset::blue);

// LemLib drivetrain handle. Track width 15", wheel diameter 3", placeholder
// RPM 450, drift radius 2. RPM/radius are unused for the legacy time-based
// auton (no closed-loop motion); they exist so the Chassis constructor
// compiles cleanly.
lemlib::Drivetrain tachyon(&left, &right, 15, 3, 450 /*placeholder*/, 2);

// ---------------------------------------------------------------------------
// Manipulator motors
// ---------------------------------------------------------------------------
pros::MotorGroup intake({1, -2});
pros::MotorGroup conveyor({3, -4});

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
// NOTE (2026-04-21): smart port scan confirmed NO sensors are actually
// wired. The IMU and both rotation sensors below are declared but read
// garbage at runtime. LemLib odometry (chassis.getPose, chassis.calibrate)
// therefore does NOT work. All motion must be time-based / dead-reckoning
// until sensors are physically installed. Declarations stay so LemLib's
// Chassis constructor (which requires OdomSensors) still compiles.
pros::Imu imu(6);                // NOT WIRED
pros::Rotation horizontal(21);   // NOT WIRED
pros::Rotation vertical(-20);    // NOT WIRED

// ---------------------------------------------------------------------------
// Pneumatics
// ---------------------------------------------------------------------------
// Role reference — updated 2026-04-22 after confirming each one physically
// with the builder. Variable names predate the role check, so don't trust
// the name alone:
//   column    — the CRANE. Raises the unloader mechanism up to the height
//               of the score tube so the unloader can reach it.
//   releaser  — flap at the end of the column/crane. Stays DOWN during
//               intake to prevent balls from falling through the conveyor.
//               Toggled UP at the score tube so balls can flow in.
//   unloader  — the "tongue". First toggle drops it down to seat at the
//               score position; second toggle retracts it up to push balls
//               into the score tube.
//   descore   — a stick that comes up/down to block or remove enemy balls
//               from the score tube. Defensive mechanism, not used for our
//               own scoring.
pros::adi::Pneumatics column('A', false);
pros::adi::Pneumatics releaser('B', false);
pros::adi::Pneumatics unloader('C', false);
pros::adi::Pneumatics descore('D', false);

// ---------------------------------------------------------------------------
// LemLib chassis pieces
// ---------------------------------------------------------------------------
// Tracking wheels reference the (unwired) rotation sensors above. Both are
// inert at runtime; they exist only to satisfy LemLib's OdomSensors API.
lemlib::TrackingWheel horizontalTracking(&horizontal, 1.5, -4);
lemlib::TrackingWheel verticalTracking(&vertical, 1.5, 0);

lemlib::OdomSensors odoms(&verticalTracking, nullptr, &horizontalTracking, nullptr, &imu);

// PID settings for LemLib's lateral and angular controllers. NOT used by
// the legacy time-based auton; kept here for the eventual sensor-driven
// rewrite. Numbers are placeholders pending real tuning.
lemlib::ControllerSettings translational(6, 0, 3, 0, 1, 100, 3, 500, 0);
lemlib::ControllerSettings rotational  (1.25, 0, 2, 0, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(tachyon, translational, rotational, odoms);
