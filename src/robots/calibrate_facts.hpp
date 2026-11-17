#pragma once
//
// calibrate_facts.hpp — THIS BUILD'S ROBOT, stated as the standard Calibrate's facts (chunk CAL1,
// Part B, 2026-11-05). PROS-free, so the host suite reads it for every robot.
//
// The standard Calibrate (include/shulib/calibration/calibrate_program.hpp) takes only what a
// person can state: the drive kind, the cartridge, and the ports the mechanisms use. In this
// repository those facts already live in the robot's two tables — the chassis table's cartridge
// (and its typed IMU port and name, as an optional cross-check and a banner word) and the
// mechanism list's motor roles — so this file MAPS them, and states nothing of its own: two
// copies of a fact is how two programs come to disagree about one robot. The tables are the
// build's (kChassis, kMechanisms: src/build_selection.hpp's one chain chose the robot), so every
// robot the chain can build gets its Calibrate with no line written for it here.
//
// THE KIND: a ChassisTable IS a tank's table (two sides, no strafe), so its kind is Tank; the
// invented X-drive carries kNoChassis, whose UNSET cartridge refuses before anything is powered
// (and the Makefile builds no Calibrate for it).
//
// THE MECHANISMS (the MECH brief's N9): one fact per MOTOR ROLE of the mechanism list — its
// physical port, as mechanism::declaredMotorPorts() lists them (the library's one rule, the one the
// drive's boot check reads too), and its mechanism's NAME (a two-motor lift gives two facts with
// one name). A 3-wire line (the old robot two's pneumatic claw, line A — retired with its claws at
// ROB1-BT) is not a motor and never a fact: the census lists smart ports only. Until CAL1 Part B robot
// two's own Calibrate dropped the declared ports from its census itself (dropDeclaredMechanismPorts(),
// deleted with it).
//
// THE TYPED DRIVE PORTS (CAL1 D6 (a), as the check of D6 (a) corrected it): withTypedDriveFrom()
// takes them from the description the BOOT would drive from with no record on the card, and only
// when the boot could drive from it. Neither cascade robot's table can (since ROB1-BT robot two's is
// robot one's twin: every field UNSET until Calibrate measures each robot), so their facts carry
// NONE, and each robot's first Calibrate on an empty card says FIRST RECORD. Whoever makes a table a
// valid description rewords the worksheet's Station G robot-two sentence in the same change —
// test/calibrate_facts_test.cpp's parity case turns red then, by design.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "common/robot_words.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::bench {

/// The table's cartridge as the facts' (UNSET stays unset: the program refuses it).
constexpr std::optional<hal::MotorCartridge> cartridgeFact(Cartridge c) {
    switch (c) {
        case Cartridge::Red: return hal::MotorCartridge::Red;
        case Cartridge::Green: return hal::MotorCartridge::Green;
        case Cartridge::Blue: return hal::MotorCartridge::Blue;
        case Cartridge::Unset: break;
    }
    return std::nullopt;
}

/// The facts robot `c` / `m` state (header): kind Tank, the table's cartridge, one mechanism fact per
/// motor role of `m` (its port from mechanism::declaredMotorPorts(), its mechanism's name), the
/// table's robot name and typed IMU port, the robot's RECORD NAME when `recordName` is given (ROB1-BT;
/// this build's: its words' recordName, kCalibrateFacts below) — and NO typed drive ports
/// (withTypedDriveFrom() adds them).
constexpr calibration::CalibrateFacts calibrateFactsFor(const ChassisTable& c, const MechanismTable& m,
                                                        const char* recordName = nullptr) {
    calibration::CalibrateFacts f{};
    f.drive = config::DrivetrainKind::Tank;  // a ChassisTable is a tank's: Calibrate is built only for ROBOT=tank|one
    f.cartridge = cartridgeFact(c.cartridge);
    int ports[calibration::kMaxMechanismPorts]{};
    const std::size_t n = mechanism::declaredMotorPorts(m, std::span<int>{ports});
    // The names, in the SAME order declaredMotorPorts() walks: mechanism by mechanism, role by role.
    std::size_t k = 0;
    for (std::size_t i = 0; i < m.count && i < mechanism::kMaxMechanisms; ++i) {
        for (std::size_t r = 0; r < m.specs[i].motorCount && r < mechanism::kMaxMotorRoles; ++r) {
            if (k < n) {
                f.mechanisms[k].port = static_cast<std::uint8_t>(ports[k]);
                f.mechanisms[k].name = m.specs[i].name;
                ++k;
            }
        }
    }
    f.robot = c.robot;
    f.imuPortStated = c.imuPort;
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    if (c.pulseYawFloorDeg > 0.0) {
        f.calibrator.pulseYawFloor = units::AngleDim{c.pulseYawFloorDeg * kDeg};
    }
    if (c.restYawRateFloorDegPerS > 0.0) {
        f.calibrator.restYawRateFloor = units::AngularVelocity{c.restYawRateFloorDegPerS * kDeg};
    }
    f.recordName = recordName;
    return f;
}

/// THE TYPED DRIVE PORTS (CAL1 D6 (a)): `f` with the drive ports of the description the BOOT would
/// drive from with no record on the card — tableToDescription(c), the boot's own function — and only
/// when it is valid(), the boot's own test (calibration::chooseBootSource: no decoded record and a
/// VALID table -> UseTable). A table the boot cannot drive from (unsigned, or its geometry UNSET, as
/// both cascade robots' are) is no front anything drives with, so it is no reference: its robot's first
/// Calibrate gets the FIRST RECORD note. Not constexpr (tableToDescription and valid() are not), so
/// it is applied where the program starts, not in kCalibrateFacts; test/calibrate_facts_test.cpp pins
/// it against tableToDescription(c) for every robot's table.
inline calibration::CalibrateFacts withTypedDriveFrom(calibration::CalibrateFacts f, const ChassisTable& c) {
    const hal::DrivetrainDescription d = tableToDescription(c);
    f.typedDrive = d.valid() ? calibration::typedDrivePortsFrom(d) : calibration::TypedDrivePorts{};
    return f;
}

/// True when `f`'s mechanism facts are EXACTLY `m`'s motor roles: the ports
/// mechanism::declaredMotorPorts(m) lists, in its order, the count included (every later entry
/// unused, port 0 and no name), each named by a name of `m`'s own. Independent of
/// calibrateFactsFor(): it walks the library's port list, not the mapping's loop. Checked on
/// kCalibrateFacts at compile time below — on the host suite's bench build `m` is the empty list, so
/// it is the ARM builds (the src gate's tank-calibrate and one-calibrate) that check the cascade's motors there;
/// test/calibrate_facts_test.cpp checks it for every robot's table on the host.
constexpr bool mechanismFactsAre(const calibration::CalibrateFacts& f, const MechanismTable& m) {
    int ports[calibration::kMaxMechanismPorts]{};
    const std::size_t n = mechanism::declaredMotorPorts(m, std::span<int>{ports});
    for (std::size_t k = 0; k < calibration::kMaxMechanismPorts; ++k) {
        if (k >= n) {
            if (f.mechanisms[k].port != 0 || f.mechanisms[k].name != nullptr) {
                return false;
            }
            continue;
        }
        if (f.mechanisms[k].port != ports[k] || f.mechanisms[k].name == nullptr) {
            return false;
        }
        bool named = false;
        for (std::size_t i = 0; i < m.count && i < mechanism::kMaxMechanisms; ++i) {
            named = named || f.mechanisms[k].name == m.specs[i].name;
        }
        if (!named) {
            return false;
        }
    }
    return true;
}

/// THIS build's stated facts (the drive kind, the cartridge, the mechanism ports, the record's robot name)
/// — checked at compile time: the standard Calibrate accepts them, and their mechanism facts are exactly THIS
/// build's list's motor roles (kMechanisms — a list named here other than the selection's, or none,
/// fails every ARM build of a robot with mechanisms: CAL1 Part B's review round). The program runs on
/// withTypedDriveFrom(kCalibrateFacts, kChassis) (src/programs/calibrate_program.cpp).
inline constexpr calibration::CalibrateFacts kCalibrateFacts =
    calibrateFactsFor(kChassis, kMechanisms, ThisRobotWords::value.recordName);
static_assert(calibration::factsProblem(kCalibrateFacts).why == nullptr,
              "this build's robot states facts the standard Calibrate refuses");
// THE RECORD NAMES ITS ROBOT (ROB1-BT): Calibrate writes the name the SAME robot's boot requires (both
// read this build's words) — its robot beacon's name.
static_assert(kRobotId == RobotId::BenchBot || kRobotId == RobotId::InventedXDrive ||
                  sameWord(kCalibrateFacts.recordName, robotRecordName(kRobotId)),
              "this build's Calibrate must name THIS robot in the record it writes (robotRecordName(kRobotId))");
static_assert(mechanismFactsAre(kCalibrateFacts, kMechanisms),
              "this build's Calibrate facts do not list exactly this build's mechanism motor ports");

}  // namespace shulib::bench
