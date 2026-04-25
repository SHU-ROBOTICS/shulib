// include/hardware.hpp
//
// Extern declarations for the robot's hardware globals. Definitions live in
// src/main.cpp; every other module includes this header to access them.
//
// Putting the definitions in main.cpp (rather than a separate hardware.cpp)
// is deliberate: hardware ports + reversal signs change rarely, and keeping
// them next to the auton selector makes the "what's plugged in" answer
// findable in one obvious place.

#pragma once

#include "ai_vision.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

// Drivetrain motor groups. Right side has intentionally different gearing
// for traction (handled in software via calibration::RIGHT_DRIVE_BIAS).
extern pros::MotorGroup left;
extern pros::MotorGroup right;

// Manipulator motor groups.
extern pros::MotorGroup intake;
extern pros::MotorGroup conveyor;

// Sensors. The IMU was wired to port 6 on 2026-04-22 — sensor-driven code
// can now use it (see include/sensor_drive.hpp). The two rotation sensors
// remain UNWIRED — declarations exist only so LemLib's Chassis constructor
// compiles; do not read them.
extern pros::Imu imu;
extern pros::Rotation horizontal;     // NOT WIRED
extern pros::Rotation vertical;       // NOT WIRED

// V5 AI Vision Sensor (276-8659) on port 5. Configured via the AI Vision
// Utility with the V5RC Push Back model — only red and blue blocks are
// classified. See include/config.hpp for class IDs.
extern ai_vision::Camera vision;

// Pneumatics. See main.cpp for the role of each (column = crane,
// releaser = flap, unloader = tongue, descore = defensive stick).
extern pros::adi::Pneumatics column;
extern pros::adi::Pneumatics releaser;
extern pros::adi::Pneumatics unloader;
extern pros::adi::Pneumatics descore;

// LemLib chassis. Used by tempTurn() (raw arcade for in-place pivot) and
// opcontrol (driver arcade). NOT used for closed-loop motion — odometry
// is non-functional without sensors.
extern lemlib::Chassis chassis;
