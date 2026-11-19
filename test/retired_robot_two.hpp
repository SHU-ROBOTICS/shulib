#pragma once
//
// retired_robot_two.hpp — THE OLD ROBOT TWO, kept as a host-test FIXTURE (chunk ROB1-BT, 2026-11-16).
//
// Until ROB1-BT robot two was the tank chassis with two claw pivots and a pneumatic claw, and its chassis
// table was the project's one MEASURED, SIGNED table (the census and MOTOR WATCH of 2026-09-10). Robot two
// was rebuilt as robot one's twin (the team lead, 2026-11-13), so src/robots/chassis_table.hpp's
// kRobotTwoChassis is no longer that table. The host tests that need "a real measured, signed table" or "a
// robot with claws" read this file:
//   * kChassis    — the old kRobotTwoChassis, byte for byte;
//   * kMechanisms — the old claws: test/mechanism_fixtures.hpp's mech_fixture::robot_two (its old region).
// NO BUILD SELECTS EITHER. A test that means "robot two's real tables" reads shulib::bench::kRobotTwoChassis
// and robot_two::kRobotTwoMechanisms — the twin's.

#include "mechanism_fixtures.hpp"
#include "robots/chassis_table.hpp"

namespace retired_robot_two {

/// The OLD robot two's chassis table, as src/robots/chassis_table.hpp held it until ROB1-BT.
inline constexpr shulib::bench::ChassisTable kChassis = {
    .robot = "2026 TANK CHASSIS (robot two)",
    .left = {-11, 12, -13, 14, -15},
    .leftCount = 5,
    .right = {20, -19, 18, -17, 16},
    .rightCount = 5,
    .cartridge = shulib::bench::Cartridge::Blue,
    .imuPort = 2,
    .measured = true,
    .signsMeasured = true,
    .provenance = "ports + IMU 2 + radio 1: census 2026-09-10 evening, all ten motors; "
                  "SIGNS: MOTOR WATCH, two front-first pushes 2026-09-10 evening, identical; "
                  "front = the 15/16 end (team lead); cartridge BLUE off a motor",
    .wheelDiameterIn = 2.75,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "wheel 2.75 in REPORTED by the team lead 2026-09-14 (standard wheels; "
                          "not yet rulered on this robot); track width UNSET (tape owed); ratio "
                          "UNSET (\"600 rpm\" reported = a cartridge, not a tooth count)",
};

/// The OLD robot two's mechanisms: the claw pivots on ports 6 and 7 and the pneumatic claw on line A.
inline constexpr const shulib::mechanism::MechanismTable& kMechanisms = mech_fixture::robot_two::kRobotTwoMechanisms;

}  // namespace retired_robot_two
