#pragma once
//
// xdrive_config.hpp — THE INVENTED X-DRIVE, AS DATA (G1 Part 2c, 2026-09-26).
//
// ═══ READ THIS FIRST: NO ROBOT HAS THESE PORTS ══════════════════════════════════════
// Every number below is INVENTED (A4 register HA-111). No X-drive robot has been measured; the
// ports, the motor signs, the cartridge (HA-15), the drive radius (HA-17), the tracking-wheel
// sizes and offsets (HA-12, HA-13) and the GPS lever arm (HA-10) are stand-ins from the day the
// whole object graph first booted (2026-08-12). The build that uses them — `make ROBOT=xdrive`,
// src/programs/xdrive_program.cpp — CANNOT boot on a real robot: on robot two the provider refuses port 1
// (the radio) before anything is opened, and the program says so on its screen and on serial.
// Robot two's ports come from the calibration record on its SD card (src/common/boot_source.{hpp,cpp})
// and are never typed here.
//
// ═══ WHAT CHANGED AT G1 PART 2c ═════════════════════════════════════════════════════
// Until Part 2c this robot was a hand-wired object graph in src/main.cpp: port constants, a
// `struct Robot` holding four ProsMotors, a ProsImu, a ProsGps, two ProsRotations, the Pilons
// odometry, the localizer, the context, the motion deps and the Chassis, built by a function-local
// static on first use. It is now ONE config::RobotConfig, built by robot::Robot over
// hal::pros::ProsDeviceProvider — exactly the way "shulib Drive" builds robot two, and the way a
// student's program will build theirs. The numbers are main.cpp's, unchanged:
// test/xdrive_config_test.cpp pins every field against a transcription of the old constants, and
// the session header's port map against the text main.cpp's portMapString() printed.
//
// ONE DIFFERENCE IN THE GRAPH, stated so it is a decision and not a surprise: robot::Robot
// installs a GPS CORRECTOR whenever a config says a GPS is present (robot.hpp, "ABSENT IS NOT
// NULL"; test/robot_builder_test.cpp case 6b pins that graph bit for bit against a hand-wired one
// with the corrector). main.cpp's graph put the ProsGps in the context and gave the localizer NO
// corrector. The config says what this robot carries — HA-111's map defines a GPS on port 9 — and
// the builder decides what that means; the alternative (declaring the GPS absent to keep the old
// corrector-less graph) would drop the GPS from the port map and the context, and would describe a
// different robot. Neither graph has booted on a robot over real adapters either way (the one
// X-drive boot on record, 2026-08-12, was the old graph over FAKE devices).
//
// ═══ constexpr ══════════════════════════════════════════════════════════════════════
// `inline constexpr`, the form robot_config.hpp's header promises for a hand-typed or generated
// config: every field is a literal type with a constexpr default, so the compiler checks the whole
// value at compile time. This is the tree's FIRST constexpr RobotConfig instance, and
// test/xdrive_config_test.cpp proves it is one with static_asserts that read it.
//
// PROS-free, so the host test includes it from src/ (`robots/xdrive_config.hpp`), as it does the
// chassis table.

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The invented X-drive (A4 register HA-111): four GREEN wheel motors FL 1, BL 2, BR -3, FR -4 in
/// x_drive.hpp's canonical wheel order (the signs INVENTED with the rest), the IMU on 10, a 7.0 in
/// drive radius, two 2.0 in tracking wheels on rotation sensors 5 (forward, offset -3.0 in LEFT) and
/// 6 (lateral, offset -4.5 in FORWARD), and a GPS on 9 with a zero lever arm. Tuning: the library's
/// PROVISIONAL defaults, as main.cpp's `Chassis{deps, pacer}` had them. The identity's name carries
/// the HA-111 label, because `config::XDriveDrivetrain` has no provenance field and the banner
/// (`robot::Robot::describe()`) must say what this robot is.
inline constexpr config::RobotConfig kInventedXDrive{
    .identity = {.name = "X-drive INVENTED ports (HA-111)"},
    .drivetrain = {.kind = config::DrivetrainKind::XDrive,
                   .xdrive = {.ports = {1, 2, -3, -4},  // FL, BL, BR, FR — signs INVENTED (HA-111)
                              .cartridge = hal::MotorCartridge::Green,  // HA-15's stand-in
                              .imuPort = 10,
                              .driveRadius = units::Length{7.0}}},  // HA-17's stand-in
    .odometry = {.kind = config::OdometryKind::TrackingWheels,
                 .wheels = {.forward = {.port = 5,
                                        .wheelDiameter = units::Length{2.0},  // HA-13
                                        .offset = units::Length{-3.0}},       // HA-12
                            .lateral = {.port = 6,
                                        .wheelDiameter = units::Length{2.0},
                                        .offset = units::Length{-4.5}}}},
    .sensors = {.gps = {.present = true,
                        .port = 9,
                        .leverArmForward = units::Length{0.0},  // HA-10's stand-in
                        .leverArmLeft = units::Length{0.0}}},
};

}  // namespace shulib::bench
