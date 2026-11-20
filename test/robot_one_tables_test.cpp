// ROBOT ONE'S TABLES (chunk ROB1-A, 2026-11-11; the ROB1 brief's D5, D6, tests T1, T8, T9; ROB1-C2, 2026-11-15:
// its six real mechanisms). Say "host test plant", never "simulator". Every number below is its facts ledger's
// (src/robots/robot_one_facts.hpp) — TYPED from the builder's answers, or a PLACEHOLDER where still owed — and
// the plant is sim::ArmPlant with INVENTED constants (HA-183) shaped from the declaration. Nothing here has run
// on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * EVERY SELECTION (T1, GM2): robot one's chassis table and mechanism list, by ADDRESS — never robot two's,
//     never the empty list (Calibrate would then take the cascade's motors for drive candidates).
//   * THE CASCADE AS DECLARED (D5): a Lift of two motors facing each other, a MEASURED shape with a row at the
//     bottom, at EVERY level and at the top, its range the bottom and top rows, homed by stalling DOWN with
//     its declaration's own signature, capped, Hold when stopped, and its six preset slots the ledger's (none
//     named since ROB1-C2: the builder's cascade is manual for now).
//   * THE OTHER FIVE (ROB1-C2): two side toggles — POWER mechanisms, one motor each on the side toggle ports
//     (the right one's fact, the left the other), green, capped, braked when let go, each its own direction
//     fact — and three ONE-line pistons on the builder's letters, each starting and made safe in its UNPOWERED
//     position, its active level and stroke time the ledger's.
//   * ON THE HOST TEST PLANT (T8, GM18): started mid-travel with its encoders reading zero there, `home`
//     drives it DOWN under its cap, stops on the stall signature at the bottom stop and calls that the bottom
//     row; then every level — and, with none named, a quarter and three quarters of its range — is reached
//     within the service's tolerance and HELD; the cap is written into BOTH motors at bind. A target past the
//     top is a compile refusal (test/compile_refusals/s81).
//   * CALIBRATE NEVER POWERS THEM (T9, GM20): calibrateFactsFor() over robot one's tables declares EVERY
//     mechanism motor port — the cascade's two and the side toggles' two — named by its mechanism; and robot
//     one's two fact checks (every non-drive motor declared, the flipper's kind) are applied in the table file
//     (the compile refusals s79 and s80 prove each refuses).

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
    // Robot one's table: the cartridge typed from the builder's answer (the one fact Calibrate cannot discover);
    // everything else MEASURED by shulib Calibrate and transcribed (below).
    const sb::ChassisTable& t = sb::kRobotOneChassis;
    const bool nothingTyped = std::string_view{t.provenance}.rfind("nothing typed", 0) == 0;
    // 2026-11-17: shulib Calibrate MEASURED robot one's drive (RUN 3); the brain had no SD card, so the run's
    // result is TRANSCRIBED into the table (its provenance says so) and the programs boot from it with the
    // boot's "no record" warning. Pinned to the run's numbers: a typo here drives the robot wrong in silence.
    CHECK_FALSE(nothingTyped);
    CHECK(t.cartridge == sb::Cartridge::Blue);  // the team's builder, 2026-11-15, read off the robot
    CHECK(std::string_view{t.provenance}.rfind("MEASURED by shulib Calibrate on robot one, 2026-11-17", 0) == 0);
    CHECK(std::string_view{t.provenance}.find("no SD card") != std::string_view::npos);
    CHECK(t.measured);
    CHECK(t.signsMeasured);
    REQUIRE(t.leftCount == 5);
    REQUIRE(t.rightCount == 5);
    const int left[5] = {+11, -12, +13, -14, +15};
    const int right[5] = {-16, +17, -18, +19, -20};
    for (std::size_t i = 0; i < 5; ++i) {
        CHECK(t.left[i] == left[i]);
        CHECK(t.right[i] == right[i]);
    }
    CHECK(t.imuPort == 5);
    CHECK(t.wheelDiameterIn == doctest::Approx(2.0 * (1.320179 + 1.332460) / 2.0).epsilon(1e-6));  // ipr L/R mean x 2
    CHECK(t.trackWidthIn == doctest::Approx(13.2305));
    CHECK(t.externalRatio == 1.0);
    CHECK(std::string_view{t.robot} == "2026 CASCADE ROBOT (robot one)");
    // Nothing is missing any more: no program refuses for the drive.
    char missing[160] = "";
    CHECK_FALSE(sb::describeMissing(t, missing, sizeof missing, true));
}

// ═══ D5 — the cascade as declared ════════════════════════════════════════════════════

TEST_CASE("robot one tables (D5): the cascade is a two-motor Lift on a MEASURED shape with a row at every level, homed by stalling DOWN, capped, every number the ledger's") {
    // BUG CAUGHT: a cascade declared from a guess instead of the ledger; a shape with rows only at the stops
    // (a level between them off by however much the spool's layering bends the line — HA-254); a homing
    // push UP; a range that is not the bottom and top rows; a preset slot that drifted from the ledger's.
    const mech::MechanismSpec& s = cascadeSpec();
    REQUIRE(sb::robot_one::kRobotOneMechanisms.count == 6);  // the cascade first, then ROB1-C2's five
    CHECK(std::string_view{s.name} == "cascade");
    CHECK(s.kind == mech::Kind::Position);
    CHECK(s.position.units == mech::Units::Inches);
    CHECK(s.motorCount == 2);
    CHECK(sb::robot_one::kRobotOneMechanisms.wiring[0].motors[0] == facts::kCascadePorts.value().first);
    CHECK(sb::robot_one::kRobotOneMechanisms.wiring[0].motors[1] == facts::kCascadePorts.value().second);
    CHECK(s.motors[0].reversed == facts::kCascadeDirections.value().first.reversed);
    CHECK(s.motors[1].reversed == facts::kCascadeDirections.value().second.reversed);
    CHECK(s.cartridge == facts::kCascadeCartridge.value());
    // DIRECT (the builder): one motor turn per spool turn — typed as the 1:1 a tooth count can say.
    CHECK(facts::kCascadeTeeth.value().motor == facts::kCascadeTeeth.value().spool);
    CHECK(s.position.gearRatio == doctest::Approx(1.0));
    CHECK(s.cartridge == shulib::hal::MotorCartridge::Blue);
    CHECK(s.currentCapAmps == facts::kCascadeCapAmps.value());
    // The stall signature — ALL THREE facts, in the declaration's cut-out AND its homing (the review of ROB1-A,
    // RV2: the rpm threshold wired to the amps fact kept the suite green — a stalled motor on the host test
    // plant reads ~0 rpm either way, so T8 cannot tell; only the declaration can). The homing's AMPS are its
    // own fact since the review of ROB1-C2 (a gentle push never draws the jam signature); its speed and time
    // are the jam signature's.
    constexpr double kRpmToRadS = 2.0 * 3.14159265358979323846 / 60.0;
    CHECK(s.position.stall.amps == facts::kCascadeStallAmps.value());
    CHECK(s.position.stall.speedRadS == doctest::Approx(facts::kCascadeStallRpm.value() * kRpmToRadS));
    CHECK(s.position.stall.time == facts::kCascadeStallTime.value().value());
    CHECK(s.position.homeStall.amps == facts::kCascadeHomingStallAmps.value());
    CHECK(s.position.homeStall.speedRadS == doctest::Approx(facts::kCascadeStallRpm.value() * kRpmToRadS));
    CHECK(s.position.homeStall.time == facts::kCascadeStallTime.value().value());
    // The ledger's three stall facts are three DIFFERENT numbers, so a field read from the wrong one shows.
    static_assert(facts::kCascadeStallAmps.value() != facts::kCascadeStallRpm.value() &&
                  facts::kCascadeStallAmps.value() != facts::kCascadeStallTime.value().value() &&
                  facts::kCascadeStallRpm.value() != facts::kCascadeStallTime.value().value());
    // ...and the coordinator's starting pair keeps chapter 18's order: the signature MORE than 0.2 A under the cap.
    static_assert(facts::kCascadeStallAmps.value() < facts::kCascadeCapAmps.value() - 0.2);
    // The gravity hold is the hold-volts fact, everywhere (a string cascade: constant), and homing puts it at
    // its bottom row (RV3 was caught only indirectly, through homing).
    CHECK(s.position.gravity == mech::Gravity::Constant);
    CHECK(s.position.kG == facts::kCascadeHoldVolts.value());
    CHECK(s.position.homePosition == facts::cascadeBottom().value());
    CHECK(s.position.homeVolts == facts::kCascadeHomingPushVolts.value());
    CHECK(s.position.unhomedVolts == facts::kCascadeUnhomedVolts.value());
    // Rows at the bottom, EVERY level and the top: each level's height is a row's height.
    const auto& rows = facts::kCascadeRows.value().row;
    CHECK(facts::kCascadeRowCount >= 2);  // the library's least (measured(): two to eight)
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
    // Every NAMED level is a preset, and nothing else is (none today: manual for now).
    std::size_t named = 0;
    for (std::size_t i = 0; i < mech::kMaxPresets; ++i) {
        named += facts::lv(i).name != nullptr ? std::size_t{1} : std::size_t{0};
    }
    CHECK(s.position.presetCount == named);
    // The range is the bottom row to the top row less the soft top's margin (2026-11-20); homing is DOWN to the
    // bottom row under its own signature.
    CHECK(s.position.minPosition == rows[0].height.value());
    CHECK(s.position.maxPosition == doctest::Approx(rows[facts::kCascadeRowCount - 1].height.value() - facts::kCascadeTopMargin.value().value()));
    CHECK(s.position.homePosition == rows[0].height.value());
    CHECK(facts::kCascadeHomingPushVolts.value() < 0.0);  // DOWN (the builder's ruling)
    CHECK(s.safeMode == shulib::config::SafeBrakeMode::Hold);
}

// ═══ T8 — the cascade on the host test plant ═══════════════════════════════════════════

TEST_CASE("robot one tables (API 2.10): BOTH robots' cascades go down at most the ledger's kCascadeMaxDownVolts -- 7.2 V, 60 % of 12; up is unlimited") {
    // BUG CAUGHT: the team lead's "it needs to go down to 60" (2026-11-20) lost from a declaration (the library
    // would drive the cascade down at 12 V again), typed as another number, or a different limit on robot two.
    CHECK(facts::kCascadeMaxDownVolts.typed);
    CHECK(facts::kCascadeMaxDownVolts.value() == doctest::Approx(7.2));
    CHECK(cascadeSpec().position.maxDownVolts == facts::kCascadeMaxDownVolts.value());
    CHECK(sb::robot_two::cascade.spec().position.maxDownVolts == sb::robot_two_facts::kCascadeMaxDownVolts.value());
    CHECK(sb::robot_two_facts::kCascadeMaxDownVolts.value() == facts::kCascadeMaxDownVolts.value());
    CHECK(mech::specRefusal(cascadeSpec()) == mech::Refusal::None);  // its homing push (down) is within it
}

TEST_CASE("robot one tables (2026-11-20): the cascade's range tops out kCascadeTopMargin UNDER its top row (1 in), on both robots") {
    // BUG CAUGHT: a held R1 braking only at the top stop itself (the team lead: "on the bottom and the top it doesnt
    // auto stop so it stalls"); the margin lost, or applied to the bottom (homing puts the bottom ON its stop).
    CHECK(facts::kCascadeTopMargin.typed);
    CHECK(facts::kCascadeTopMargin.value().value() == doctest::Approx(1.0));
    CHECK(cascadeSpec().position.maxPosition == doctest::Approx(facts::cascadeTop().value() - 1.0));
    CHECK(cascadeSpec().position.minPosition == facts::cascadeBottom().value());
    CHECK(sb::robot_two::cascade.spec().position.maxPosition == doctest::Approx(sb::robot_two_facts::cascadeSoftTop().value()));
}

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
    CHECK(worst >= facts::kCascadeHomingStallAmps.value());  // the HOMING signature was really reached
    // The home ends on its OWN signature, not the jam one (the review of ROB1-C2: one 1.5 A signature for both
    // left a gentle push unable to end a home): the declaration's homing signature is the homing facts'.
    CHECK(cascadeSpec().position.homeStall.amps == doctest::Approx(facts::kCascadeHomingStallAmps.value()));
    CHECK(cascadeSpec().position.stall.amps == doctest::Approx(facts::kCascadeStallAmps.value()));
    CHECK(std::abs(r.slot().position() - facts::cascadeBottom().value()) < 1e-6);  // homing calls the bottom stop the bottom row

    // EVERY LEVEL: reached within the service's tolerance (0.25 in) by its TRUE height, then held.
    const double tolIn = cascadeSpec().position.tolerance;
    int levels = 0;
    for (std::size_t i = 0; i < mech::kMaxPresets; ++i) {
        const facts::Level& lv = facts::lv(i);
        if (lv.name == nullptr) {
            continue;
        }
        ++levels;
        CAPTURE(lv.name);
        r.slot().moveToPreset(static_cast<std::uint8_t>(i), Time{10.0});
        r.runStep(2000);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(std::abs(heightAt(r.trueDeg()) - lv.at.value()) <= tolIn + 0.05);
        r.tick(150);  // held for 1.5 s: it does not sag off the level
        CHECK(std::abs(heightAt(r.trueDeg()) - lv.at.value()) <= tolIn + 0.05);
    }
    // ...and with NO named level (ROB1-C2: manual for now) a quarter and three quarters of its range — up, then
    // down — so the cascade is still driven and held on its real shape, whatever levels are typed.
    const double bottom = facts::cascadeBottom().value();
    const double top = facts::cascadeTop().value();
    for (const double target : {bottom + 0.75 * (top - bottom), bottom + 0.25 * (top - bottom)}) {
        CAPTURE(target);
        r.slot().moveTo(target, Time{10.0}, /*computed=*/false);
        r.runStep(2000);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(std::abs(heightAt(r.trueDeg()) - target) <= tolIn + 0.05);
        r.tick(150);  // held for 1.5 s
        CHECK(std::abs(heightAt(r.trueDeg()) - target) <= tolIn + 0.05);
    }
    MESSAGE("robot one's cascade reached " << levels << " named level(s) and two range points on the host test plant");
}

// ═══ T9 — Calibrate never powers a mechanism ═════════════════════════════════════════════

TEST_CASE("robot one tables (T9): calibrateFactsFor() over robot one's tables declares EVERY mechanism motor port -- the cascade's two and the side toggles' two -- named by its mechanism") {
    // BUG CAUGHT (GM20): a cascade or side-toggle port dropped from the wiring's declared ports — Calibrate would
    // take that motor for a drive candidate and POWER it when it turns with the push (landmine 4).
    const shulib::calibration::CalibrateFacts f =
        sb::calibrateFactsFor(sb::kRobotOneChassis, sb::robot_one::kRobotOneMechanisms);
    int declared = 0;
    for (const auto& m : f.mechanisms) {
        if (m.port == 0) continue;
        ++declared;
        CAPTURE(static_cast<int>(m.port));
        const std::string_view name{m.name};
        if (m.port == facts::kCascadePorts.value().first || m.port == facts::kCascadePorts.value().second) {
            CHECK(name == "cascade");
        } else if (m.port == facts::kRightTogglePort.value()) {
            CHECK(name == "right toggle");
        } else {
            CHECK(m.port == facts::leftTogglePort());
            CHECK(name == "left toggle");
        }
    }
    CHECK(declared == 4);
    // Every port the builder listed on the Devices screen is among them: 10, 1, 9, 2.
    for (int i = 0; i < facts::kNonDriveMotorPorts.value().count; ++i) {
        const int p = facts::kNonDriveMotorPorts.value().port[i];
        CAPTURE(p);
        bool found = false;
        for (const auto& m : f.mechanisms) {
            found = found || m.port == p;
        }
        CHECK(found);
    }
    CHECK(facts::kNonDriveMotorPorts.value().count == 4);
    CHECK(sb::mechanismFactsAre(f, sb::robot_one::kRobotOneMechanisms));
    // Every Devices-screen non-drive motor (the ledger's B6) is declared — the predicate both ways.
    CHECK(sb::everyListedMotorDeclared(facts::kNonDriveMotorPorts.value(), sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::everyListedMotorDeclared(facts::PortList{{8, 9, 10}, 3}, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::everyListedMotorDeclared(facts::PortList{{11}, 1}, sb::robot_one::kRobotOneMechanisms));
    // The flipper's kind against the list (ROB1-C2: the front toggle, a pneumatic named "flipper"): a
    // pneumatic matches; "not on the robot yet", a passive bar and a motor arm do not.
    CHECK(sb::flipperKindMatches(facts::FlipperKind::Pneumatic, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::flipperKindMatches(facts::FlipperKind::NotOnTheRobotYet, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::flipperKindMatches(facts::FlipperKind::PassiveBar, sb::robot_one::kRobotOneMechanisms));
    CHECK_FALSE(sb::flipperKindMatches(facts::FlipperKind::MotorArm, sb::robot_one::kRobotOneMechanisms));
    // ...and the other direction (the review of ROB1-A, RV8/RV9) is test/robot_one_flipper_forms_example_test.cpp's:
    // "not on the robot yet" or a passive bar against a list that DOES declare a flipper, and an inches Lift
    // named "flipper" against a motor arm.
    // ...and both checks are APPLIED to each robot's real facts in the table file (deleting one would compile;
    // ROB1-BT made the two checks every robot's — robot two's facts against robot two's list too).
    const std::string code = source_pins::squash(source_pins::stripComments(source_pins::readFile("src/robots/mechanism_table.hpp")));
    CHECK(source_pins::count(code, "static_assert(nonDriveMotorsDeclared(robot_one_facts::kNonDriveMotorPorts.value(), robot_one::kRobotOneMechanisms),") == 1);
    CHECK(source_pins::count(code, "static_assert(flipperKindAgrees(robot_one_facts::kFlipperKind.value(), robot_one::kRobotOneMechanisms),") == 1);
    CHECK(source_pins::count(code, "static_assert(nonDriveMotorsDeclared(robot_two_facts::kNonDriveMotorPorts.value(), robot_two::kRobotTwoMechanisms),") == 1);
    CHECK(source_pins::count(code, "static_assert(flipperKindAgrees(robot_two_facts::kFlipperKind.value(), robot_two::kRobotTwoMechanisms),") == 1);
}

TEST_CASE("robot one tables (ROB1-C2): the side toggles are two POWER mechanisms on the side toggle ports, and the three pistons ONE line each on the builder's letters, starting in their UNPOWERED position") {
    // BUG CAUGHT: a side toggle declared as an Arm (angles and a homing nobody gave), both on one port, the left
    // on the RIGHT's port, a side toggle with no cap at 12 V, or one that coasts on when let go; a piston on the
    // wrong letter, with a second line, or STARTING in its powered position (it would move at power-up).
    const sb::MechanismTable& t = sb::robot_one::kRobotOneMechanisms;
    const auto at = [&](std::string_view name) -> std::size_t {
        for (std::size_t i = 0; i < t.count; ++i) {
            if (std::string_view{t.specs[i].name} == name) return i;
        }
        FAIL("no mechanism named " << name);
        return 0;
    };
    const std::size_t right = at("right toggle");
    const std::size_t left = at("left toggle");
    for (const std::size_t i : {right, left}) {
        const mech::MechanismSpec& s = t.specs[i];
        CAPTURE(s.name);
        CHECK(s.kind == mech::Kind::Power);
        CHECK(s.motorCount == 1);
        CHECK(s.cartridge == shulib::hal::MotorCartridge::Green);
        CHECK(s.cartridge == facts::kSideToggleCartridge.value());
        CHECK(s.currentCapAmps == facts::kSideToggleCapAmps.value());
        CHECK(s.currentCapAmps > 0.0);  // the cap is what makes 12 V held against a stop acceptable
        CHECK(s.safeMode == shulib::config::SafeBrakeMode::Brake);  // "when I let go it will stop"
    }
    CHECK(t.wiring[right].motors[0] == facts::kRightTogglePort.value());
    CHECK(t.wiring[left].motors[0] == facts::leftTogglePort());
    CHECK(t.wiring[right].motors[0] != t.wiring[left].motors[0]);
    CHECK(t.specs[right].motors[0].reversed == facts::kRightToggleDirection.value().reversed);
    CHECK(t.specs[left].motors[0].reversed == facts::kLeftToggleDirection.value().reversed);
    // The two side toggle ports are the builder's 9 and 2, in either order; leftTogglePort() is the other one,
    // and 0 (no port: the wiring refuses it by name) for a right port that is neither.
    CHECK(((facts::kSideTogglePorts.value().first == 9 && facts::kSideTogglePorts.value().second == 2) ||
           (facts::kSideTogglePorts.value().first == 2 && facts::kSideTogglePorts.value().second == 9)));
    CHECK(facts::leftTogglePort() + facts::kRightTogglePort.value() == 11);
    const struct {
        const char* name;
        char letter;
        const char* rest;
        const char* active;
        bool activeLevel;
        double stroke;
    } pistons[3] = {
        {"claw", 'A', "closed", "open", facts::kClawOpenLevel.value(), facts::kClawStrokeTime.value().value()},
        {"flipper", 'B', "closed", "open", facts::kFlipperOpenLevel.value(), facts::kFlipperStrokeTime.value().value()},
        {"ramp", 'C', "up", "down", facts::kRampDownLevel.value(), facts::kRampStrokeTime.value().value()},
    };
    for (const auto& p : pistons) {
        CAPTURE(p.name);
        const std::size_t i = at(p.name);
        const mech::MechanismSpec& s = t.specs[i];
        CHECK(s.kind == mech::Kind::Pneumatic);
        CHECK(s.lineCount == 1);
        CHECK(static_cast<int>(t.wiring[i].lines[0]) == p.letter - 'A' + 1);
        REQUIRE(s.pneumatic.positionCount == 2);
        CHECK(std::string_view{s.pneumatic.positions[0].name} == p.rest);
        CHECK(std::string_view{s.pneumatic.positions[1].name} == p.active);
        CHECK(s.pneumatic.positions[1].levels[0] == p.activeLevel);
        CHECK(s.pneumatic.positions[0].levels[0] == !p.activeLevel);
        // start and safe: the position at LOW — the solenoid unpowered — whichever name that is
        CHECK(s.pneumatic.positions[s.pneumatic.start].levels[0] == shulib::mechanism::words::Low);
        CHECK(s.pneumatic.start == s.pneumatic.safe);
        CHECK(s.pneumatic.actuationTime == p.stroke);
    }
    CHECK(facts::kClawLine.value() == 'A');
    CHECK(facts::kFlipperLine.value() == 'B');
    CHECK(facts::kRampLine.value() == 'C');
    // unpoweredPosition() both ways: an active position at HIGH rests in the other; at LOW, in itself.
    static_assert(std::string_view{facts::unpoweredPosition(shulib::mechanism::words::High, "closed", "open")} == "closed");
    static_assert(std::string_view{facts::unpoweredPosition(shulib::mechanism::words::Low, "closed", "open")} == "open");
}

TEST_CASE("robot one tables (T7): robot one's region names nothing of robot two's") {
    // (ROB1-A's [R5] half — every robot-one line BELOW robot two's region, so robot two's packages kept their
    // recorded line numbers — is RETIRED at ROB1-BT: robot two became robot one's twin and its packages change
    // by design; robot one's region now comes first. test/robot_twin_test.cpp holds the two regions equal.)
    const std::string text = source_pins::readFile("src/robots/mechanism_table.hpp");
    // ...and robot one's region names nothing of robot two's (T7's before-twin half).
    const std::size_t a = text.find("// [robot one mechanisms]");
    const std::size_t b = text.find("// [/robot one mechanisms]");
    REQUIRE(a != std::string::npos);
    REQUIRE(b != std::string::npos);
    const std::string one = source_pins::stripComments(text.substr(a, b - a));
    CHECK(one.find("robot_two") == std::string::npos);
    CHECK(one.find("kRobotTwo") == std::string::npos);
}
