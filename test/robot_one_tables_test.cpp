// ROBOT ONE'S TABLES (chunk ROB1-A, 2026-11-11; the ROB1 brief's D5, D6, tests T1, T8, T9). Say "host test
// plant", never "simulator". Robot one does not exist in this project's records: every number below is its
// facts ledger's PLACEHOLDER (src/robots/robot_one_facts.hpp), and the plant is sim::ArmPlant with INVENTED
// constants (HA-183) shaped from the declaration. Nothing here has run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * EVERY SELECTION (T1, GM2): robot one's chassis table and mechanism list, by ADDRESS — never robot two's,
//     never the empty list (Calibrate would then take the cascade's motors for drive candidates).
//   * THE CASCADE AS DECLARED (D5): a Lift of two motors facing each other, a MEASURED shape with a row at the
//     bottom, at EVERY level and at the top, its range the bottom and top rows, homed by stalling DOWN with
//     its declaration's own signature, capped, Hold when stopped, and its six preset slots the ledger's.
//   * ON THE HOST TEST PLANT (T8, GM18): started mid-travel with its encoders reading zero there, `home`
//     drives it DOWN under its cap, stops on the stall signature at the bottom stop and calls that the bottom
//     row; then every level is reached within the service's tolerance and HELD; the cap is written into BOTH
//     motors at bind. A target past the top is a compile refusal (test/compile_refusals/s81).
//   * CALIBRATE NEVER POWERS IT (T9, GM20): calibrateFactsFor() over robot one's tables declares EVERY cascade
//     motor port, named by its mechanism; and robot one's two fact checks (every non-drive motor declared, the
//     flipper's kind) are applied in the table file (the compile refusals s79 and s80 prove each refuses).

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>

#include "mechanism_position_rig.hpp"
#include "robots/calibrate_facts.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/units/quantity.hpp"
#include "source_pins.hpp"

namespace sb = shulib::bench;
namespace mech = shulib::mechanism;
namespace facts = sb::robot_one_facts;
using mech::MechanismOutcome;
using sb::RobotId;
using shulib::units::Time;

namespace {

const mech::MechanismSpec& cascadeSpec() { return sb::robot_one::kRobotOneMechanisms.specs[0]; }

/// The cascade's height at output angle `deg`, by the ledger's rows (straight between rows — HA-254).
double heightAt(double deg) {
    const auto& rows = facts::kCascadeRows.value().row;
    for (std::size_t i = 1; i < facts::kCascadeRowCount; ++i) {
        if (deg <= rows[i].degrees || i + 1 == facts::kCascadeRowCount) {
            const double d0 = rows[i - 1].degrees;
            const double d1 = rows[i].degrees;
            const double h0 = rows[i - 1].height.value();
            const double h1 = rows[i].height.value();
            return h0 + (h1 - h0) * (deg - d0) / (d1 - d0);
        }
    }
    return rows[0].height.value();
}

/// The host test plant for the cascade, from its DECLARATION: the ledger's cartridge (its free speed),
/// gearing (motor teeth : spool teeth) and rows (the stops at the bottom and top rows' output angles);
/// gravity below the declared hold volts, a little friction. Started at `startDeg` with its encoders
/// reading ZERO there — where it is is unknown until it homes.
shulib::sim::ArmPlantConfig cascadePlant(double startDeg) {
    const double gear = static_cast<double>(facts::kCascadeTeeth.value().spool) /
                        static_cast<double>(facts::kCascadeTeeth.value().motor);
    shulib::sim::ArmPlantConfig pc =
        pos_rig::armPlant(pos_rig::cartridgeRpm(facts::kCascadeCartridge.value()), gear,
                          0.8 * facts::kCascadeHoldVolts.value(), 1.0, 0.3, startDeg);
    pc.constantGravity = true;
    pc.minStopDeg = facts::kCascadeRows.value().row[0].degrees;
    pc.maxStopDeg = facts::kCascadeRows.value().row[facts::kCascadeRowCount - 1].degrees;
    pc.encoderZeroDeg = startDeg;
    return pc;
}

}  // namespace

// ═══ T1 — the selections ═════════════════════════════════════════════════════════════

TEST_CASE("robot one tables (T1): chassisTableFor and mechanismTableFor give robot one ITS OWN table and list, by address") {
    // BUG CAUGHT (GM2): mechanismTableFor(RobotOne) returning kNoMechanisms — Calibrate would take the
    // cascade's motors for drive candidates and POWER them; or robot two's claws on robot one.
    CHECK(&sb::chassisTableFor(RobotId::RobotOne) == &sb::kRobotOneChassis);
    CHECK(&sb::mechanismTableFor(RobotId::RobotOne) == &sb::robot_one::kRobotOneMechanisms);
    CHECK(&sb::mechanismTableFor(RobotId::RobotOne) != &sb::kNoMechanisms);
    CHECK(&sb::mechanismTableFor(RobotId::RobotOne) != &sb::robot_two::kRobotTwoMechanisms);
    CHECK(&sb::chassisTableFor(RobotId::RobotOne) != &sb::kRobotTwoChassis);
    // Robot one's table: every drivetrain fact UNSET (Calibrate discovers them) — except the drive cartridge
    // and the IMU's port (the facts form's A2 and A4), which Calibrate cannot discover: the coordinator TYPES
    // those two here at guide 21's order, step 1, with the provenance saying who read them. So: while the
    // provenance still says "nothing typed", both are UNSET and the refusal names the cartridge; once it does
    // not, a typed cartridge and an IMU on a real port (the verifier of ROB1-A, F6: these pins went red the
    // day the procedure was followed). The ports, the geometry and `measured` stay Calibrate's either way.
    const sb::ChassisTable& t = sb::kRobotOneChassis;
    const bool nothingTyped = std::string_view{t.provenance}.rfind("nothing typed", 0) == 0;
    CHECK(t.leftCount == 0);
    CHECK(t.rightCount == 0);
    if (nothingTyped) {
        CHECK(t.cartridge == sb::Cartridge::Unset);
        CHECK(t.imuPort == 0);
    } else {
        CHECK(t.cartridge != sb::Cartridge::Unset);
        CHECK(t.imuPort >= 1);
        CHECK(t.imuPort <= 21);
    }
    CHECK_FALSE(t.measured);
    CHECK_FALSE(t.signsMeasured);
    CHECK(t.trackWidthIn == 0.0);
    CHECK(t.externalRatio == 0.0);
    CHECK(t.wheelDiameterIn == 0.0);
    CHECK(std::string_view{t.robot} == "2026 CASCADE ROBOT (robot one)");
    // Every unknown is NAMED by the table's own refusal, never guessed.
    // (No ports until Calibrate: the drive refusal stands either way; it names the cartridge only while unset.)
    char missing[160] = "";
    CHECK(sb::describeMissing(t, missing, sizeof missing, true));
    CHECK((std::string_view{missing}.find("cartridge") != std::string_view::npos) == nothingTyped);
}

// ═══ D5 — the cascade as declared ════════════════════════════════════════════════════

TEST_CASE("robot one tables (D5): the cascade is a two-motor Lift on a MEASURED shape with a row at every level, homed by stalling DOWN, capped, every number the ledger's") {
    // BUG CAUGHT: a cascade declared from a guess instead of the ledger; a shape with rows only at the stops
    // (a level between them off by however much the spool's layering bends the line — HA-254); a homing
    // push UP; a range that is not the bottom and top rows; a preset slot that drifted from the ledger's.
    const mech::MechanismSpec& s = cascadeSpec();
    REQUIRE(sb::robot_one::kRobotOneMechanisms.count == 1);  // the flipper is not declared until its kind is known (D6)
    CHECK(std::string_view{s.name} == "cascade");
    CHECK(s.kind == mech::Kind::Position);
    CHECK(s.position.units == mech::Units::Inches);
    CHECK(s.motorCount == 2);
    CHECK(sb::robot_one::kRobotOneMechanisms.wiring[0].motors[0] == facts::kCascadePorts.value().first);
    CHECK(sb::robot_one::kRobotOneMechanisms.wiring[0].motors[1] == facts::kCascadePorts.value().second);
    CHECK(s.motors[0].reversed == facts::kCascadeDirections.value().first.reversed);
    CHECK(s.motors[1].reversed == facts::kCascadeDirections.value().second.reversed);
    CHECK(s.cartridge == facts::kCascadeCartridge.value());
    CHECK(s.position.gearRatio == doctest::Approx(60.0 / 12.0));
    CHECK(s.currentCapAmps == facts::kCascadeCapAmps.value());
    // The stall signature — ALL THREE facts, in the declaration's cut-out AND its homing (the review of ROB1-A,
    // RV2: the rpm threshold wired to the amps fact kept the suite green — a stalled motor on the host test
    // plant reads ~0 rpm either way, so T8 cannot tell; only the declaration can).
    constexpr double kRpmToRadS = 2.0 * 3.14159265358979323846 / 60.0;
    CHECK(s.position.stall.amps == facts::kCascadeStallAmps.value());
    CHECK(s.position.stall.speedRadS == doctest::Approx(facts::kCascadeStallRpm.value() * kRpmToRadS));
    CHECK(s.position.stall.time == facts::kCascadeStallTime.value().value());
    CHECK(s.position.homeStall.amps == facts::kCascadeStallAmps.value());
    CHECK(s.position.homeStall.speedRadS == doctest::Approx(facts::kCascadeStallRpm.value() * kRpmToRadS));
    CHECK(s.position.homeStall.time == facts::kCascadeStallTime.value().value());
    // The ledger's three stall facts are three DIFFERENT numbers, so a field read from the wrong one shows.
    static_assert(facts::kCascadeStallAmps.placeholder != facts::kCascadeStallRpm.placeholder &&
                  facts::kCascadeStallAmps.placeholder != facts::kCascadeStallTime.placeholder.value() &&
                  facts::kCascadeStallRpm.placeholder != facts::kCascadeStallTime.placeholder.value());
    // The gravity hold is the hold-volts fact, everywhere (a string cascade: constant), and homing puts it at
    // its bottom row (RV3 was caught only indirectly, through homing).
    CHECK(s.position.gravity == mech::Gravity::Constant);
    CHECK(s.position.kG == facts::kCascadeHoldVolts.value());
    CHECK(s.position.homePosition == facts::cascadeBottom().value());
    CHECK(s.position.homeVolts == facts::kCascadeHomingPushVolts.value());
    CHECK(s.position.unhomedVolts == facts::kCascadeUnhomedVolts.value());
    // Rows at the bottom, EVERY level and the top: each level's height is a row's height.
    const auto& rows = facts::kCascadeRows.value().row;
    CHECK(facts::kCascadeRowCount >= 3);
    for (std::size_t i = 1; i < facts::kCascadeRowCount; ++i) {
        CHECK(rows[i - 1].height.value() < rows[i].height.value());
        CHECK(rows[i - 1].degrees < rows[i].degrees);
    }
    for (std::size_t i = 0; i < mech::kMaxPresets; ++i) {
        const facts::Level& lv = facts::lv(i);
        CAPTURE(i);
        if (lv.name == nullptr) {
            continue;
        }
        bool onARow = false;
        for (const auto& r : rows) {
            onARow = onARow || r.height.value() == lv.at.value();
        }
        CHECK(onARow);
        CHECK(std::string_view{s.position.presets[i].name} == lv.name);
        CHECK(s.position.presets[i].value == lv.at.value());
    }
    // The range is the bottom and top rows; homing is DOWN to the bottom row under its own signature.
    CHECK(s.position.minPosition == rows[0].height.value());
    CHECK(s.position.maxPosition == rows[facts::kCascadeRowCount - 1].height.value());
    CHECK(s.position.homePosition == rows[0].height.value());
    CHECK(facts::kCascadeHomingPushVolts.value() < 0.0);  // DOWN (the builder's ruling)
    CHECK(s.safeMode == shulib::config::SafeBrakeMode::Hold);
}

// ═══ T8 — the cascade on the host test plant ═══════════════════════════════════════════

TEST_CASE("robot one tables (T8): on the host test plant the cascade HOMES DOWN under its cap onto its bottom row, then reaches and holds every level; the cap is on BOTH motors") {
    // BUG CAUGHT (GM18 is refused at build — a homing push UP); here: a home that never stalls (the
    // signature unreachable under the cap: the placeholders would fail the robot's first boot), a home that
    // calls the wrong place zero, a level the service cannot reach on its own measured shape, a cap written
    // into motor 1 only.
    const double startDeg = 300.0;  // above the bottom stop, its encoders reading zero here
    pos_rig::ArmRig r{cascadeSpec(), cascadePlant(startDeg)};
    // The cap: written into BOTH motors at bind, and read back.
    CHECK(r.motors[0].currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    CHECK(r.motors[1].currentLimit().value() == doctest::Approx(facts::kCascadeCapAmps.value()));
    CHECK_FALSE(r.slot().homed());

    // HOME: DOWN into the bottom stop, under the cap, ended by the stall signature.
    r.slot().home(Time{10.0});  // the placeholder gearing moves the output ~60 deg/s at 6 V
    double worst = 0.0;
    double lowestCommand = 0.0;
    bool contact = false;
    int n = 0;
    while (r.slot().running() && n < 2000) {
        r.tick();
        ++n;
        lowestCommand = std::min(lowestCommand, r.volts());
        if (!contact && r.trueDeg() <= facts::kCascadeRows.value().row[0].degrees + 1e-9) {
            contact = true;
            continue;  // the impact sample is never judged (the cap does not limit momentum — HA-222)
        }
        if (contact) {
            worst = std::max(worst, r.amps());
        }
    }
    MESSAGE("robot one's cascade home on the host test plant: " << n << " ticks, ended at " << r.trueDeg()
            << " deg, worst " << worst << " A after contact, lowest command " << lowestCommand << " V");
    REQUIRE(contact);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.slot().homed());
    CHECK(lowestCommand < 0.0);  // it pushed DOWN
    CHECK(lowestCommand >= facts::kCascadeHomingPushVolts.value() - 1e-9);
    CHECK(worst <= facts::kCascadeCapAmps.value() + 1e-12);
    CHECK(worst >= facts::kCascadeStallAmps.value());  // the signature was really reached
    CHECK(std::abs(r.slot().position() - facts::cascadeBottom().value()) < 1e-6);  // homing calls the bottom stop the bottom row

    // EVERY LEVEL: reached within the service's tolerance (0.25 in) by its TRUE height, then held.
    const double tolIn = cascadeSpec().position.tolerance;
    for (std::size_t i = 0; i < mech::kMaxPresets; ++i) {
        const facts::Level& lv = facts::lv(i);
        if (lv.name == nullptr) {
            continue;
        }
        CAPTURE(lv.name);
        r.slot().moveToPreset(static_cast<std::uint8_t>(i), Time{10.0});
        r.runStep(2000);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(std::abs(heightAt(r.trueDeg()) - lv.at.value()) <= tolIn + 0.05);
        r.tick(150);  // held for 1.5 s: it does not sag off the level
        CHECK(std::abs(heightAt(r.trueDeg()) - lv.at.value()) <= tolIn + 0.05);
    }
}

// ═══ T9 — Calibrate never powers a mechanism ═════════════════════════════════════════════

TEST_CASE("robot one tables (T9): calibrateFactsFor() over robot one's tables declares EVERY cascade motor port, named by its mechanism") {
    // BUG CAUGHT (GM20): a cascade port dropped from the wiring's declared ports — Calibrate would take that
    // motor for a drive candidate and POWER it when it turns with the push (landmine 4).
    const shulib::calibration::CalibrateFacts f =
        sb::calibrateFactsFor(sb::kRobotOneChassis, sb::robot_one::kRobotOneMechanisms);
    int declared = 0;
    for (const auto& m : f.mechanisms) {
        if (m.port == 0) continue;
        ++declared;
        CHECK(std::string_view{m.name} == "cascade");
    }
    CHECK(declared == cascadeSpec().motorCount);
    bool first = false;
    bool second = false;
    for (const auto& m : f.mechanisms) {
        first = first || m.port == facts::kCascadePorts.value().first;
        second = second || m.port == facts::kCascadePorts.value().second;
    }
    CHECK(first);
    CHECK(second);
    CHECK(sb::mechanismFactsAre(f, sb::robot_one::kRobotOneMechanisms));
    // Every Devices-screen non-drive motor (the ledger's B6) is declared — the predicate both ways.
    CHECK(sb::everyListedMotorDeclared(facts::kNonDriveMotorPorts.value(), sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::everyListedMotorDeclared(facts::PortList{{8, 9, 10}, 3}, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::everyListedMotorDeclared(facts::PortList{{10}, 1}, sb::robot_one::kRobotOneMechanisms));
    // The flipper's kind against the list: "not on the robot yet" and a passive bar match a list with no
    // flipper; a motor arm and a pneumatic do not.
    CHECK(sb::flipperKindMatches(facts::FlipperKind::NotOnTheRobotYet, sb::robot_one::kRobotOneMechanisms));
    CHECK(sb::flipperKindMatches(facts::FlipperKind::PassiveBar, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::flipperKindMatches(facts::FlipperKind::MotorArm, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::flipperKindMatches(facts::FlipperKind::Pneumatic, sb::robot_one::kRobotOneMechanisms));
    // ...and the other direction (the review of ROB1-A, RV8/RV9) is test/robot_one_flipper_forms_example_test.cpp's:
    // "not on the robot yet" or a passive bar against a list that DOES declare a flipper, and an inches Lift
    // named "flipper" against a motor arm.
    // ...and both checks are APPLIED to the real ledger in the table file (deleting one would compile).
    const std::string code = source_pins::squash(source_pins::stripComments(source_pins::readFile("src/robots/mechanism_table.hpp")));
    CHECK(source_pins::count(code, "static_assert(robotOneNonDriveMotorsDeclared(robot_one_facts::kNonDriveMotorPorts.value(), robot_one::kRobotOneMechanisms),") == 1);
    CHECK(source_pins::count(code, "static_assert(robotOneFlipperKindMatches(robot_one_facts::kFlipperKind.value(), robot_one::kRobotOneMechanisms),") == 1);
}

TEST_CASE("robot one tables ([R5]): every robot-one line of the mechanism table is BELOW the end of robot two's region") {
    // BUG CAUGHT (GM15b's cause): a robot-one line added above `// [/robot two mechanisms]` — robot two's
    // declarations record their LINE numbers into the list its packages carry (std::source_location), so
    // every robot-two package would change while its machine code and strings stayed the same (P6).
    const std::string text = source_pins::readFile("src/robots/mechanism_table.hpp");
    const std::size_t endTwo = text.find("// [/robot two mechanisms]");
    REQUIRE(endTwo != std::string::npos);
    const std::string above = text.substr(0, endTwo);
    CHECK(above.find("robot_one") == std::string::npos);
    CHECK(above.find("robot one") == std::string::npos);
    CHECK(above.find("ROB1") == std::string::npos);
    // ...and robot one's region names nothing of robot two's (T7's before-twin half).
    const std::size_t a = text.find("// [robot one mechanisms]");
    const std::size_t b = text.find("// [/robot one mechanisms]");
    REQUIRE(a != std::string::npos);
    REQUIRE(b != std::string::npos);
    REQUIRE(endTwo < a);
    const std::string one = source_pins::stripComments(text.substr(a, b - a));
    CHECK(one.find("robot_two") == std::string::npos);
    CHECK(one.find("kRobotTwo") == std::string::npos);
}
