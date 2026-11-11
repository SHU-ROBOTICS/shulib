// THE CURRENT CAP — a mechanism's `.currentCap`, written into every one of its motors at boot, read back,
// re-checked once a second, and enforced by the MOTOR (chunk MECH-S-A, 2026-11-05; the MECH-S brief's D1
// (a), D2 (a), D12 (b); tests CA2, CA4, CA5, CA7–CA10, CA14, CA15). Say "host test plant", never
// "simulator": the motor's limit is modelled by sim::ArmPlant (INVENTED, HA-183) — whether a real V5 motor
// holds its limit like that, with nothing ticking, is A4 register HA-219, never measured. Nothing here has
// run on a robot.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * EVERY MOTOR (CA4): a two-motor lift's cap lands on BOTH motors (MA2: motor 0 only), each read back;
//     the boot line says per motor AND in all (MA8: the total not × motors), verbatim, once.
//   * D12 (b) (CA4): a mechanism with NO cap is READ and NEVER written — at boot and for ten seconds after
//     (MA9: a no-cap motor written); one whose limit reads other than 2.5 A gets ONE Info line naming both
//     causes (MA1: dropped); one at 2.5 A logs nothing new.
//   * A MOTOR THAT WILL NOT (CA5): refused, another number read back, no limit to set — a Warn each,
//     naming the port and both numbers, no "set and read back" claim, and the mechanism still runs.
//   * THE MARGIN (CA2): a signature 0.19 A under the cap boots with the Warn, 0.21 A under without, and
//     EXACTLY 0.2 A under with it (the ordering is strict: signature < cap − 0.2) — judged in whole mA, so
//     the edge is exact on BOTH sides of binary rounding: 1.2 − 1.0 is 0.19999999999999996 in doubles and
//     2.5 − 2.3 is 0.20000000000000018, both exactly 200 mA (review: the edge made exclusive, and an amps
//     comparison with no tolerance, each turns one of the two red) — a stall signature, a homing one, a
//     power's, a flywheel's (MA12: dropped). A signature is judged as the MOTOR reads (the fresh verifier):
//     its FIRST tripping whole mA, never its nearest — 0.9994 A under 1.2 A warns (its nearest, 999 mA,
//     did not), 0.999 A does not, and two doubles where a bare ceil is one mA off (2.007 A; one ulp above
//     0.938 A) land on the edge from each side; the detector itself is shown tripping at each of those mA.
//   * THE RE-CHECK (CA10): a motor that "loses power" is written again within a second, ONE Warn per loss
//     (MA6: never re-written); one that keeps its cap is never re-written; one set to 1.8 A by something
//     else gets ONE Warn in ten seconds and is never written (MA10: any change re-written); a STARVED
//     service re-checks nothing (landmine 20) and catches up at its next tick. ITS PACE (review): exactly
//     one read per capped motor per check, the checks 100 or 101 ticks apart (a second on the 10 ms rig —
//     never every tick, never 1.1 s), and a limit lost on the tick after a check NOT rewritten 0.98 s later
//     but rewritten by 1.01 s; a motor that keeps REFUSING its cap is retried every second, silently, and
//     capped the moment it accepts.
//   * EVERY MOTOR KIND (review): an Arm's, a Lift's, a Power's and a Flywheel's `.currentCap` each reach
//     the spec and the build's signature rule, and an Arm's is written to its motor at bind.
//   * THE DEVICE'S UNIT (review, over the PROS shim — only ProsMotor rounds to whole mA): a cap of 2.4995 A
//     (sent as 2500 mA), 1.2005 A (1201 mA) and 1.2345 A (1235 mA) each reads back as ITSELF — "set and read
//     back", no Warn, and never re-written in ten seconds (it was: once a second, forever).
//   * THE MOTOR, NOT THE LIBRARY (CA7–CA9, on the host test plant): a 12 V push into an obstacle with
//     NOTHING ticking for 1.5 s stays at or under the cap (MA4: the plant ignores the limit); a move into
//     an obstacle STALLS as the uncapped one does, pulling no more than the cap after contact; a homing
//     push into the top stop — and, for the team's string cascade, DOWN into its bottom stop — homes under
//     the cap. The FIRST sample after an impact is never judged (landmine 4: this plant stops a joint dead
//     in one step, and the cap does not limit momentum — HA-222).
//   * THE ORDER OF FOUR CURRENTS (CA15, the checker's V1): capped at 1.2 A with a 1.0 A signature, a 3 V
//     lift holds clean; a 2.5 V piece raises the hold past the signature and the hold ends STALLED — the
//     documented reason the guide states the order, its sentence pinned with the boot Warn's.
//   * ROBOT TWO (CA14): its boot is byte for byte the boot of a provider that answers no limit at all,
//     while both claw motors read 2.5 A; a claw motor at 1.8 A adds exactly one Info line.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_position_rig.hpp"
#include "mechanism_service_rig.hpp"
#include "pros/shim_control.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/manipulation/stall_detector.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/units/quantity.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace sim = shulib::sim;
using mech::MechanismOutcome;
using mech::MechanismTable;
using shulib::units::Current;
using shulib::units::Time;
using shulib::units::Voltage;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerIn = 360.0 / (kPi * 1.5);  // a 1.5 in spool

}  // namespace

namespace cap_fx {
using namespace shulib::mechanism::words;
// A two-motor spool lift capped at 1.2 A per motor, its signature 0.3 A under the cap (no margin Warn).
inline constexpr Lift lift{.name = "lift", .motors = {Forward, Reversed}, .cartridge = Red, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold,
                           .stall = {.amps = 0.9, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                           .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in),
                           .presets = {{"floor", 0_in}, {"high", 22_in}}};
// A power mechanism with NO cap (D12 (b): read, never written).
inline constexpr Power intake{.name = "intake", .motors = {Forward}, .cartridge = Green, .safe = Coast};
inline constexpr MechanismTable kTable = mechanisms(wire(lift, motors(11, 12)), wire(intake, motors(5)));

// One capped single-motor power (the singular boot line).
inline constexpr Power pivot{.name = "pivot", .motors = {Forward}, .cartridge = Red, .safe = Hold,
                             .currentCap = {.amps = 0.8}};
inline constexpr MechanismTable kPivot = mechanisms(wire(pivot, motors(3)));

// THE MARGIN (CA2): a signature 0.19, 0.21 and exactly 0.20 A under a 1.2 A cap; a homing one; a power's;
// a flywheel's.
inline constexpr Lift m019{.name = "m019", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold,
                           .stall = {.amps = 1.01, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                           .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift m021{.name = "m021", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold,
                           .stall = {.amps = 0.99, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                           .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift m020{.name = "m020", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold,
                           .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                           .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift homing{.name = "homing", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                             .shape = spool(1.5_in), .safe = Hold,
                             .stall = {.amps = 0.5, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                             .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt),
                             .home = homeByStall(0_in, -3_volt, {.amps = 1.05, .belowMotorRpm = 5, .lasting = 0.2_s})};
inline constexpr Power roller{.name = "roller", .motors = {Forward}, .cartridge = Green, .safe = Coast,
                              .stall = {.amps = 0.7, .belowMotorRpm = 5, .lasting = 0.2_s}, .currentCap = {.amps = 0.8}};
inline constexpr Flywheel wheel{.name = "wheel", .motors = {Forward}, .cartridge = Blue, .gearing = direct,
                                .safe = Coast, .stall = {.amps = 1.4, .belowMotorRpm = 50, .lasting = 0.3_s},
                                .currentCap = {.amps = 1.5}, .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 500}}};
// The edge again where the binary difference falls ABOVE 0.2 (2.5 − 2.3 = 0.20000000000000018; 1.2 − 1.0
// falls below it): both are exactly 200 mA, both warn.
inline constexpr Lift m2523{.name = "m2523", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold,
                            .stall = {.amps = 2.3, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 2.5},
                            .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr MechanismTable kMargins =
    mechanisms(wire(m019, motors(1)), wire(m021, motors(2)), wire(m020, motors(3)), wire(homing, motors(4)),
               wire(roller, motors(5)), wire(wheel, motors(6)), wire(m2523, motors(7)));

// THE MARGIN AS THE MOTOR READS IT (the fresh verifier): a signature is never SENT, so it is never rounded —
// the StallDetector trips on the first whole-mA reading at or above it. Each lift below LANDS on the 200 mA
// edge (or one mA past it) only when the signature is judged that way, never at its nearest mA.
//   s9996: 0.9996 A under 1.2 A — trips at 1000 mA: 200 under, WARNS (0.2004 A below in amps).
//   s9994: 0.9994 A under 1.2 A — trips at 1000 mA: 200 under, WARNS (its nearest mA, 999, would be 201: silent).
//   s999:  0.999 A under 1.2 A  — trips at 999 mA: 201 under, NO Warn (the inclusive edge from the quiet side).
//   s2007: 2.007 A under 2.208 A — 2.007 * 1000 is 2007.0000000000002 in doubles, so a bare ceil says 2008
//          (200 under, a false Warn); the detector trips at 2007 (201 under): NO Warn.
//   s938u: one ulp above 0.938 A under 1.139 A — its * 1000 is 938.0 in doubles, so a bare ceil says 938 (201
//          under, silent); 938 mA reads 0.938 A, BELOW it, so the detector trips at 939 (200 under): WARNS.
inline constexpr Lift s9996{.name = "s9996", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold,
                            .stall = {.amps = 0.9996, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                            .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift s9994{.name = "s9994", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold,
                            .stall = {.amps = 0.9994, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                            .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift s999{.name = "s999", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                           .shape = spool(1.5_in), .safe = Hold,
                           .stall = {.amps = 0.999, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 1.2},
                           .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift s2007{.name = "s2007", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold,
                            .stall = {.amps = 2.007, .belowMotorRpm = 5, .lasting = 0.5_s}, .currentCap = {.amps = 2.208},
                            .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Lift s938u{.name = "s938u", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold,
                            .stall = {.amps = 0.9380000000000001, .belowMotorRpm = 5, .lasting = 0.5_s},
                            .currentCap = {.amps = 1.139}, .range = {0_in, 24_in},
                            .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr MechanismTable kSubMilliamp =
    mechanisms(wire(s9996, motors(1)), wire(s9994, motors(2)), wire(s999, motors(3)), wire(s2007, motors(4)),
               wire(s938u, motors(5)));

// EVERY MOTOR KIND states a cap (review: an Arm's `.currentCap` reaching its spec had no test). Each a
// distinct number, its signature 0.3 A under it.
inline constexpr Arm cappedArm{.name = "capped arm", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84),
                               .safe = Hold, .stall = {.amps = 1.2, .belowMotorRpm = 5, .lasting = 0.5_s},
                               .currentCap = {.amps = 1.5}, .range = {0, 120}, .gravity = cosine(1.0_volt, 30),
                               .home = restsAtStop(0), .presets = {{"down", 0}, {"up", 110}}};
inline constexpr Lift cappedLift{.name = "capped lift", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                                 .shape = spool(1.5_in), .safe = Hold,
                                 .stall = {.amps = 0.8, .belowMotorRpm = 5, .lasting = 0.5_s},
                                 .currentCap = {.amps = 1.1}, .range = {0_in, 24_in},
                                 .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
inline constexpr Power cappedPower{.name = "capped power", .motors = {Forward}, .cartridge = Green, .safe = Coast,
                                   .stall = {.amps = 0.4, .belowMotorRpm = 5, .lasting = 0.2_s},
                                   .currentCap = {.amps = 0.7}};
inline constexpr Flywheel cappedWheel{.name = "capped wheel", .motors = {Forward}, .cartridge = Blue, .gearing = direct,
                                      .safe = Coast, .stall = {.amps = 1.6, .belowMotorRpm = 50, .lasting = 0.3_s},
                                      .currentCap = {.amps = 1.9}, .atSpeed = within(60, 0.1_s),
                                      .presets = {{"shoot", 500}}};
inline constexpr MechanismTable kArm = mechanisms(wire(cappedArm, motors(7)));

// THE DEVICE'S UNIT (review): caps that are not a whole milliamp, booted over the PROS shim, where
// ProsMotor rounds each to the nearest mA on its way out (2.4995 -> 2500, 1.2005 -> 1201, 1.2345 -> 1235).
inline constexpr Power halfUp{.name = "half up", .motors = {Forward}, .cartridge = Red, .safe = Hold,
                              .currentCap = {.amps = 2.4995}};
inline constexpr Power halfOver{.name = "half over", .motors = {Forward}, .cartridge = Red, .safe = Hold,
                                .currentCap = {.amps = 1.2005}};
inline constexpr Power fraction{.name = "fraction", .motors = {Forward}, .cartridge = Red, .safe = Hold,
                                .currentCap = {.amps = 1.2345}};
inline constexpr MechanismTable kFractions =
    mechanisms(wire(halfUp, motors(1)), wire(halfOver, motors(2)), wire(fraction, motors(3)));
}  // namespace cap_fx

namespace {

/// The MECH lines that speak of a current limit or cap (every other boot line — a home, a starting value —
/// is another chunk's).
std::vector<std::string> capLines(const mech_rig::Rig& r) {
    std::vector<std::string> out;
    for (const std::string& l : r.mechLines()) {
        if (l.find("current") != std::string::npos) {
            out.push_back(l);
        }
    }
    return out;
}
/// The level of the MECH line equal to `text`; fails the test when there is not exactly one.
hal::LogLevel levelOf(const mech_rig::Rig& r, const std::string& text) {
    int n = 0;
    hal::LogLevel level = hal::LogLevel::Debug;
    for (int i = 0; i < r.sink.size(); ++i) {
        if (r.sink.at(i).subsystem == "MECH" && r.sink.at(i).message == text) {
            ++n;
            level = r.sink.at(i).level;
        }
    }
    CHECK(n == 1);
    return level;
}

const std::string kLiftCapLine = "'lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back";

}  // namespace

// ═══ 1. AT BOOT ═══════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-S-A CA4: a stated cap is written to EVERY motor and read back, said once per motor AND in all; a no-cap motor is never written") {
    mech_rig::Rig r{cap_fx::kTable};
    auto& m11 = r.devices.fakeMotor(11);
    auto& m12 = r.devices.fakeMotor(12);
    auto& m5 = r.devices.fakeMotor(5);
    CHECK(m11.currentLimit().value() == 1.2);
    CHECK(m12.currentLimit().value() == 1.2);  // BUG CAUGHT (MA2): written to motor 0 only
    CHECK(m11.currentLimitWrites() == 1);
    CHECK(m12.currentLimitWrites() == 1);
    CHECK(m5.currentLimitWrites() == 0);       // BUG CAUGHT (MA9): a no-cap motor written at boot
    CHECK(m5.currentLimit().value() == 2.5);
    // ONE cap line, verbatim (BUG CAUGHT (MA8): the total not × the motors), at Info
    CHECK(capLines(r) == std::vector<std::string>{kLiftCapLine});
    CHECK(levelOf(r, kLiftCapLine) == hal::LogLevel::Info);
    // ten seconds of ticks: the capped motors are never re-written (they keep their cap), the no-cap one
    // is never written, and nothing more is said
    r.pace(1000);
    CHECK(m11.currentLimitWrites() == 1);
    CHECK(m12.currentLimitWrites() == 1);
    CHECK(m5.currentLimitWrites() == 0);
    CHECK(capLines(r).size() == 1U);
}

TEST_CASE("MECH-S-A CA4: a single capped motor -- the boot line's singular and its total") {
    mech_rig::Rig r{cap_fx::kPivot};
    CHECK(capLines(r) == std::vector<std::string>{
                             "'pivot': current cap 0.80 A per motor (1 motor: 0.80 A in all) -- set and read back"});
    CHECK(r.devices.fakeMotor(3).currentLimit().value() == 0.8);
}

TEST_CASE("MECH-S-A CA4 (D12 (b)): NO cap and a motor that reads 1.0 A -- ONE Info line naming both causes, and NOTHING written, ever") {
    mech_rig::Rig r{cap_fx::kTable, [](hal::fake::FakeDeviceProvider& d) {
                        d.fakeMotor(5).setDeviceCurrentLimit(Current{1.0});  // a cap another program left
                    }};
    const std::string info =
        "'intake': motor 1 (port 5): its current limit reads 1.00 A: a cap left by another program (HA-220), or "
        "VEXos's sharing with many motors plugged in (HA-221) -- nothing written";
    CHECK(capLines(r) == std::vector<std::string>{kLiftCapLine, info});  // BUG CAUGHT (MA1): the Info dropped
    CHECK(levelOf(r, info) == hal::LogLevel::Info);
    CHECK(r.devices.fakeMotor(5).currentLimitWrites() == 0);
    CHECK(r.devices.fakeMotor(5).currentLimit().value() == 1.0);  // not undone (D12 (b))
    r.pace(500);  // no re-check of a mechanism with no cap: nothing written, nothing more said
    CHECK(r.devices.fakeMotor(5).currentLimitWrites() == 0);
    CHECK(capLines(r).size() == 2U);
}

TEST_CASE("MECH-S-A CA5: a motor that REFUSES its cap, or reads back another number -- a Warn naming the port and both numbers; it still runs") {
    SUBCASE("refused (motor 2, port 12)") {
        mech_rig::Rig r{cap_fx::kTable, [](hal::fake::FakeDeviceProvider& d) { d.fakeMotor(12).refuseCurrentLimit(true); }};
        const std::string warn =
            "'lift': motor 2 (port 12) did not take its current cap: asked 1.20 A, the write was refused, reads back "
            "2.50 A -- it runs; that motor is not capped as stated";
        CHECK(capLines(r) == std::vector<std::string>{warn});  // and NO "set and read back" claim
        CHECK(levelOf(r, warn) == hal::LogLevel::Warn);
        CHECK(r.devices.fakeMotor(11).currentLimit().value() == 1.2);  // the other motor is capped
        // it runs: a move begins (rests at its stop: homed at bind)
        r.svc.position(0).moveTo(10.0, Time{1.0}, false);
        CHECK(r.svc.position(0).running());
        // A MOTOR THAT KEEPS REFUSING (review): it still reads the default, so every check writes it again —
        // SILENTLY (its boot Warn said it; BUG CAUGHT (review): a Warn every second on a flaky port) — about
        // once a second: the boot's write, then the checks at 1, 2, 3 and 4 s (and 5 s, by the rounding of
        // a 10 ms clock's sum)
        auto& m12 = r.devices.fakeMotor(12);
        r.pace(500);
        CHECK(capLines(r) == std::vector<std::string>{warn});
        CHECK(m12.currentLimitWrites() >= 5);
        CHECK(m12.currentLimitWrites() <= 6);
        CHECK(m12.currentLimit().value() == 2.5);
        // ... and the moment the port takes the write, the motor is capped — still with no new line
        m12.refuseCurrentLimit(false);
        const int before = m12.currentLimitWrites();
        r.pace(102);
        CHECK(m12.currentLimit().value() == 1.2);
        CHECK(m12.currentLimitWrites() == before + 1);
        CHECK(capLines(r) == std::vector<std::string>{warn});
        // capped now, a LATER loss is a new event: one Warn for it
        r.pace(102);
        m12.losePower();
        r.pace(102);
        CHECK(m12.currentLimit().value() == 1.2);
        CHECK(capLines(r).size() == 2U);
    }
    SUBCASE("another number read back (motor 1, port 11 keeps 2.0 A)") {
        mech_rig::Rig r{cap_fx::kTable,
                        [](hal::fake::FakeDeviceProvider& d) { d.fakeMotor(11).currentLimitTakesInstead(2.0); }};
        const std::string warn =
            "'lift': motor 1 (port 11) did not take its current cap: asked 1.20 A, the write was accepted, reads back "
            "2.00 A -- it runs; that motor is not capped as stated";
        CHECK(capLines(r) == std::vector<std::string>{warn});
        CHECK(levelOf(r, warn) == hal::LogLevel::Warn);
        // the re-check never fights it (it reads neither its cap nor the default): no second write, no
        // second line, in five seconds
        r.pace(500);
        CHECK(r.devices.fakeMotor(11).currentLimitWrites() == 1);
        CHECK(capLines(r).size() == 1U);
    }
    SUBCASE("this provider gave no limit for motor 2 (a bare IMotor bound on port 12)") {
        hal::fake::FakeMotor bare;  // outlives the rig
        mech_rig::Rig r{cap_fx::kTable,
                        [&bare](hal::fake::FakeDeviceProvider& d) { d.bindMotor(12, static_cast<hal::IMotor&>(bare)); }};
        const std::string warn =
            "'lift': motor 2 (port 12): current cap 1.20 A stated, but this provider cannot cap a motor -- the motor's "
            "own 2.50 A applies";
        CHECK(capLines(r) == std::vector<std::string>{warn});
        CHECK(levelOf(r, warn) == hal::LogLevel::Warn);
        CHECK(bare.currentLimitWrites() == 0);  // never reached through a cast
        CHECK(r.devices.fakeMotor(11).currentLimit().value() == 1.2);
        r.pace(300);
        CHECK(capLines(r).size() == 1U);
    }
}

TEST_CASE("MECH-S-A CA2: a signature within 0.2 A of the cap boots with a Warn -- 0.19 and exactly 0.20 under do, 0.21 under does not") {
    mech_rig::Rig r{cap_fx::kMargins};
    const auto warn = [](const char* name, const char* what, const char* amps, const char* cap) {
        return std::string{"'"} + name + "': its " + what + " (" + amps + " A) is within 0.2 A of its current cap (" +
               cap + " A): a capped motor may read a little under its cap, so it may never trip (HA-223)";
    };
    const std::vector<std::string> lines = r.mechLines();
    const auto has = [&lines](const std::string& l) { return std::count(lines.begin(), lines.end(), l); };
    CHECK(has(warn("m019", "stall signature", "1.01", "1.20")) == 1);   // BUG CAUGHT (MA12): dropped
    // the EXACT edge, 200 mA, from both sides of binary rounding (1.2 - 1.0 = 0.19999999999999996 and
    // 2.5 - 2.3 = 0.20000000000000018 in doubles): BUG CAUGHT (review) — the edge made exclusive reds the
    // first, an amps comparison with no tolerance the second
    CHECK(has(warn("m020", "stall signature", "1.00", "1.20")) == 1);
    CHECK(has(warn("m2523", "stall signature", "2.30", "2.50")) == 1);
    CHECK(has(warn("m021", "stall signature", "0.99", "1.20")) == 0);   // 0.21 under: no Warn
    for (const std::string& l : lines) {
        CHECK(l.find("'m021': its") == std::string::npos);
    }
    CHECK(has(warn("homing", "homing signature", "1.05", "1.20")) == 1);  // the homing push's
    CHECK(has(warn("homing", "stall signature", "0.50", "1.20")) == 0);   // its move's is far under
    CHECK(has(warn("roller", "stall signature", "0.70", "0.80")) == 1);   // a power's
    CHECK(has(warn("wheel", "stall signature", "1.40", "1.50")) == 1);    // a flywheel's
    CHECK(levelOf(r, warn("m019", "stall signature", "1.01", "1.20")) == hal::LogLevel::Warn);
    // nothing is refused at boot: every one of them is built and capped
    for (std::size_t i = 0; i < r.svc.size(); ++i) {
        CAPTURE(i);
        CHECK(r.svc.slot(i).built());
    }
    CHECK(r.devices.fakeMotor(4).currentLimit().value() == 1.2);
}

TEST_CASE("MECH-S-A CA2 (verifier): the margin is judged as the MOTOR reads -- a signature's first tripping whole mA, never its nearest") {
    // the premise, on the detector itself: each signature first trips at the reading the fixtures say
    const auto tripsAt = [](double sig, double ma) {
        shulib::manipulation::StallDetector d{{.currentAtLeast = Current{sig},
                                               .speedAtMost = shulib::units::AngularVelocity{1.0},
                                               .persistence = Time{0.0}}};
        return d.update(Time{0.0}, hal::motorMilliampsToCanonical(ma), shulib::units::AngularVelocity{0.0});
    };
    CHECK_FALSE(tripsAt(0.9994, 999.0));
    CHECK(tripsAt(0.9994, 1000.0));
    CHECK_FALSE(tripsAt(0.9996, 999.0));
    CHECK(tripsAt(0.9996, 1000.0));
    CHECK_FALSE(tripsAt(0.999, 998.0));
    CHECK(tripsAt(0.999, 999.0));
    CHECK_FALSE(tripsAt(2.007, 2006.0));
    CHECK(tripsAt(2.007, 2007.0));
    CHECK_FALSE(tripsAt(0.9380000000000001, 938.0));
    CHECK(tripsAt(0.9380000000000001, 939.0));

    mech_rig::Rig r{cap_fx::kSubMilliamp};
    const auto warn = [](const char* name, const char* amps, const char* cap) {
        return std::string{"'"} + name + "': its stall signature (" + amps + " A) is within 0.2 A of its current cap (" +
               cap + " A): a capped motor may read a little under its cap, so it may never trip (HA-223)";
    };
    const std::vector<std::string> lines = r.mechLines();
    const auto has = [&lines](const std::string& l) { return std::count(lines.begin(), lines.end(), l); };
    const auto saysAnything = [&lines](const char* name) {
        const std::string head = std::string{"'"} + name + "': its";
        return std::count_if(lines.begin(), lines.end(), [&head](const std::string& l) { return l.rfind(head, 0) == 0; });
    };
    CHECK(has(warn("s9996", "1.00", "1.20")) == 1);  // 1000 mA: 200 under
    CHECK(has(warn("s9994", "1.00", "1.20")) == 1);  // BUG CAUGHT (verifier): its nearest mA (999) was silent
    CHECK(saysAnything("s999") == 0);                // 999 mA: 201 under
    CHECK(saysAnything("s2007") == 0);               // 2007 mA, not a bare ceil's 2008: 201 under
    CHECK(has(warn("s938u", "0.94", "1.14")) == 1);  // 939 mA, not a bare ceil's 938: 200 under
    for (std::size_t i = 0; i < r.svc.size(); ++i) {
        CAPTURE(i);
        CHECK(r.svc.slot(i).built());
    }
}

// ═══ 2. THE ONCE-A-SECOND RE-CHECK (CA10) ════════════════════════════════════════════

TEST_CASE("MECH-S-A CA10: a motor that LOSES POWER is written again within a second, ONE Warn per loss; its mate is never re-written") {
    mech_rig::Rig r{cap_fx::kTable};
    auto& m11 = r.devices.fakeMotor(11);
    auto& m12 = r.devices.fakeMotor(12);
    const std::string lost =
        "'lift': motor 1 (port 11): its current limit reads 2.50 A, the motor's default (it lost power? HA-220) -- its "
        "cap 1.20 A written again";
    r.pace(35);
    m11.losePower();
    CHECK(m11.currentLimit().value() == 2.5);
    r.pace(101);  // one second and a tick
    CHECK(m11.currentLimit().value() == 1.2);  // BUG CAUGHT (MA6): the re-check never re-writes
    CHECK(m11.currentLimitWrites() == 2);
    CHECK(levelOf(r, lost) == hal::LogLevel::Warn);
    // lost again 0.01 s after that re-write (about 0.36 s after the check that made it — the checks fall
    // at about t = 1.0, 2.0, ...; the case lost on the tick right AFTER a check is the pace test's, below)
    r.pace(1);
    m11.losePower();
    r.pace(101);
    CHECK(m11.currentLimit().value() == 1.2);
    CHECK(m11.currentLimitWrites() == 3);
    // ONE Warn per loss: two losses, two lines — and nothing between them
    int n = 0;
    for (const std::string& l : r.mechLines()) {
        n += l == lost ? 1 : 0;
    }
    CHECK(n == 2);
    r.pace(800);
    CHECK(m12.currentLimitWrites() == 1);  // its mate kept its cap: never re-written
    CHECK(m11.currentLimitWrites() == 3);
}

TEST_CASE("MECH-S-A CA10: a limit changed to 1.8 A by something else -- ONE Warn in ten seconds, and NEVER written") {
    mech_rig::Rig r{cap_fx::kTable};
    auto& m12 = r.devices.fakeMotor(12);
    r.pace(200);
    m12.setDeviceCurrentLimit(Current{1.8});
    r.pace(1000);
    const std::string other =
        "'lift': motor 2 (port 12): its current limit reads 1.80 A, not its cap 1.20 A: something else set it "
        "(another program, or VEXos's sharing -- HA-221) -- not written again";
    CHECK(levelOf(r, other) == hal::LogLevel::Warn);   // exactly ONE in ten seconds
    CHECK(m12.currentLimitWrites() == 1);              // BUG CAUGHT (MA10): any change re-written
    CHECK(m12.currentLimit().value() == 1.8);
    // back at its cap, then changed again: said again (once per change)
    m12.setDeviceCurrentLimit(Current{1.2});
    r.pace(150);
    m12.setDeviceCurrentLimit(Current{1.8});
    r.pace(150);
    int n = 0;
    for (const std::string& l : r.mechLines()) {
        n += l == other ? 1 : 0;
    }
    CHECK(n == 2);
    CHECK(m12.currentLimitWrites() == 1);
}

TEST_CASE("MECH-S-A CA10 (landmine 20): a STARVED service re-checks nothing -- it catches up at its next tick") {
    mech_rig::Rig r{cap_fx::kTable};
    auto& m11 = r.devices.fakeMotor(11);
    r.pace(10);
    m11.losePower();
    r.clock.advance(Time{3.0});  // three seconds with NOTHING ticking (a student's blocking loop)
    CHECK(m11.currentLimitWrites() == 1);  // never its own task: nothing re-wrote it
    r.pace(1);                              // the next tick is past the check time
    CHECK(m11.currentLimitWrites() == 2);
    CHECK(m11.currentLimit().value() == 1.2);
}

TEST_CASE("MECH-S-A CA10 (review): the re-check's PACE -- one read per capped motor per check, a second apart, never every tick; a loss right after a check waits for the next") {
    // A read on a real port is a device call: a re-check every 10 ms tick (BUG CAUGHT (review R4): the
    // `t < nextCapCheck_` guard dropped) would be one per motor per tick; one every 1.1 s (BUG CAUGHT
    // (review R3)) leaves a lost cap lost for longer than the header promises. The rig ticks at 10 ms,
    // its clock a running sum of 0.01 s, so a second is 100 ticks or 101 (the sum's rounding).
    mech_rig::Rig r{cap_fx::kPivot};
    auto& m3 = r.devices.fakeMotor(3);
    std::vector<int> checks;  // the ticks on which the service read motor 3's limit
    long reads = m3.currentLimitReads();
    for (int tick = 0; tick < 1000; ++tick) {
        r.pace(1);
        const long now = m3.currentLimitReads();
        if (now != reads) {
            CHECK(now == reads + 1);  // ONE read per capped motor per check
            checks.push_back(tick);
        }
        reads = now;
    }
    REQUIRE(checks.size() >= 9U);
    CHECK(checks.size() <= 10U);
    CHECK(checks.front() >= 99);   // bound at t = 0: the first check a second later (tick 99 runs at
    CHECK(checks.front() <= 100);  // t = 0.99 + the sum's rounding; tick 100 at t = 1.00)
    for (std::size_t i = 1; i < checks.size(); ++i) {
        CAPTURE(i);
        CHECK(checks[i] - checks[i - 1] >= 100);
        CHECK(checks[i] - checks[i - 1] <= 101);
    }
    // a limit lost on the tick right AFTER a check: NOT rewritten 0.98 s later, rewritten by 1.01 s
    reads = m3.currentLimitReads();
    int waited = 0;
    while (m3.currentLimitReads() == reads && waited < 102) {  // pace up to the next check, inclusive
        r.pace(1);
        ++waited;
    }
    REQUIRE(m3.currentLimitReads() == reads + 1);
    const long readsAtLoss = m3.currentLimitReads();
    const int writes = m3.currentLimitWrites();
    m3.losePower();
    r.pace(98);
    CHECK(m3.currentLimitReads() == readsAtLoss);  // no check yet
    CHECK(m3.currentLimitWrites() == writes);
    CHECK(m3.currentLimit().value() == 2.5);
    r.pace(3);
    CHECK(m3.currentLimitWrites() == writes + 1);
    CHECK(m3.currentLimit().value() == 0.8);
}

TEST_CASE("MECH-S-A (review): EVERY motor kind carries its .currentCap into its spec and the build's signature rule -- and an Arm's reaches its motor at bind") {
    // BUG CAUGHT (review K1): Arm::spec() dropping its `s.currentCapAmps = currentCap.amps;` passed the
    // whole suite — every capped declaration in it was a Lift, a Power or a Flywheel.
    struct KindCase {
        const char* what;
        mech::MechanismSpec spec;
        double cap;
    };
    const KindCase kinds[] = {{"an Arm", cap_fx::cappedArm.spec(), 1.5},
                          {"a Lift", cap_fx::cappedLift.spec(), 1.1},
                          {"a Power", cap_fx::cappedPower.spec(), 0.7},
                          {"a Flywheel", cap_fx::cappedWheel.spec(), 1.9}};
    for (const KindCase& k : kinds) {
        CAPTURE(k.what);
        CHECK(k.spec.currentCapAmps == k.cap);
        CHECK(mech::specRefusal(k.spec) == mech::Refusal::None);
        // the same declaration with its signature AT its cap is refused by the cap's rule — so a kind whose
        // spec() lost the cap would be accepted here
        mech::MechanismSpec atCap = k.spec;
        mech::StallSpec& st = atCap.kind == mech::Kind::Power      ? atCap.power.stall
                              : atCap.kind == mech::Kind::Speed    ? atCap.speed.stall
                                                                   : atCap.position.stall;
        st.amps = k.cap;
        CHECK(mech::specRefusal(atCap) == mech::Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    }
    // a declaration that states none carries 0 (none stated — the motor's own 2.5 A)
    CHECK(mech_fixture::robot_x::arm.spec().currentCapAmps == 0.0);
    CHECK(cap_fx::intake.spec().currentCapAmps == 0.0);
    // an Arm bound: its motor capped, the boot line said
    mech_rig::Rig r{cap_fx::kArm};
    CHECK(r.devices.fakeMotor(7).currentLimit().value() == 1.5);
    CHECK(r.devices.fakeMotor(7).currentLimitWrites() == 1);
    CHECK(capLines(r) == std::vector<std::string>{
                             "'capped arm': current cap 1.50 A per motor (1 motor: 1.50 A in all) -- set and read back"});
}

TEST_CASE("MECH-S-A (review): over the PROS shim a cap that is not a whole milliamp reads back as ITSELF -- set and read back, no Warn, never re-written") {
    // The motor holds whole milliamps; the service now compares in them (a +-0.5 mA tolerance on the
    // doubles called 2.4995 A's read-back of 2500 mA "not the cap" AND "the default": a Warn of two equal
    // numbers, then a re-write every second, forever; 1.2005 A's 1201 mA a Warn, and never checked again).
    // BUG CAUGHT (review): the old comparison. Only ProsMotor rounds — FakeMotor keeps the exact double — so
    // this runs over the shim.
    pros::shim::resetAll();
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeDeviceRegistry reg;
    for (const std::uint8_t p : {std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{3}}) {
        reg.setKind(p, hal::DeviceKind::Motor);
    }
    hal::pros::ProsDeviceProvider prov{reg};
    const MechanismTable& t = cap_fx::kFractions;
    std::array<shulib::config::MechanismConfig, mech::kMaxMechanisms> configs{};
    for (std::size_t i = 0; i < t.count; ++i) {
        configs[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    shulib::robot::MechanismParts parts;
    parts.build(std::span<const shulib::config::MechanismConfig>{configs.data(), t.count}, prov);
    mech_rig::World world;
    world.clock = &clock;
    mech::MechanismService svc{world};
    svc.bind(t, parts.devices(), clock, faults, sink);
    // what each motor was SENT, in whole mA (hal::motorCurrentLimitToMilliamps' rounding)
    CHECK(pros::shim::motorState(1).currentLimitMa == 2500);
    CHECK(pros::shim::motorState(2).currentLimitMa == 1201);
    CHECK(pros::shim::motorState(3).currentLimitMa == 1235);
    std::vector<std::string> lines;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "MECH" && sink.at(i).message.find("current") != std::string::npos) {
            lines.push_back(sink.at(i).message);
            CHECK(sink.at(i).level == hal::LogLevel::Info);  // no Warn: each took its cap
        }
    }
    REQUIRE(lines.size() == 3U);
    CHECK(lines[0] == "'half up': current cap 2.50 A per motor (1 motor: 2.50 A in all) -- set and read back");
    CHECK(lines[1].find("'half over': current cap ") == 0);
    CHECK(lines[1].find("-- set and read back") != std::string::npos);
    CHECK(lines[2].find("'fraction': current cap ") == 0);
    CHECK(lines[2].find("-- set and read back") != std::string::npos);
    // ten seconds of re-checks: none of them is ever written again, and nothing more is said
    for (int i = 0; i < 1000; ++i) {
        svc.pace();
    }
    for (const int p : {1, 2, 3}) {
        CAPTURE(p);
        CHECK(pros::shim::motorState(p).setCurrentLimitCalls == 1);
    }
    int capSaid = 0;
    for (int i = 0; i < sink.size(); ++i) {
        capSaid += sink.at(i).subsystem == "MECH" && sink.at(i).message.find("current") != std::string::npos ? 1 : 0;
    }
    CHECK(capSaid == 3);
    // and the 1201 mA one that loses power (its device back at 2500 mA) is written again within a second,
    // to the same whole milliamps; the others are left alone
    pros::shim::motorState(2).currentLimitMa = 2500;
    for (int i = 0; i < 102; ++i) {
        svc.pace();
    }
    CHECK(pros::shim::motorState(2).currentLimitMa == 1201);
    CHECK(pros::shim::motorState(2).setCurrentLimitCalls == 2);
    CHECK(pros::shim::motorState(1).setCurrentLimitCalls == 1);
    pros::shim::resetAll();
}

// ═══ 3. ROBOT TWO (CA14) ═════════════════════════════════════════════════════════════

TEST_CASE("MECH-S-A CA14: robot two's boot is byte for byte a no-limit provider's while both claw motors read 2.5 A -- and nothing is written") {
    // The twin: a provider that answers NO current limit (each claw motor bound as a bare IMotor) — the
    // service then reads nothing, writes nothing and says nothing about limits: the boot before MECH-S-A.
    hal::fake::FakeMotor bare6;
    hal::fake::FakeMotor bare7;
    mech_rig::Rig before{shulib::bench::mechanismTableFor(shulib::bench::RobotId::RobotTwo), [&](hal::fake::FakeDeviceProvider& d) {
                             d.bindMotor(6, static_cast<hal::IMotor&>(bare6));
                             d.bindMotor(7, static_cast<hal::IMotor&>(bare7));
                         }};
    mech_rig::Rig now{shulib::bench::mechanismTableFor(shulib::bench::RobotId::RobotTwo)};
    REQUIRE(now.sink.size() == before.sink.size());
    for (int i = 0; i < now.sink.size(); ++i) {
        CAPTURE(i);
        CHECK(now.sink.at(i).level == before.sink.at(i).level);
        CHECK(now.sink.at(i).subsystem == before.sink.at(i).subsystem);
        CHECK(now.sink.at(i).message == before.sink.at(i).message);
    }
    CHECK(now.devices.fakeMotor(6).currentLimitWrites() == 0);
    CHECK(now.devices.fakeMotor(7).currentLimitWrites() == 0);
    now.pace(500);
    before.pace(500);
    CHECK(now.sink.size() == before.sink.size());
    CHECK(now.devices.fakeMotor(6).currentLimitWrites() == 0);
}

TEST_CASE("MECH-S-A CA14: robot two with claw pivot 6's motor at 1.8 A -- exactly ONE more line, an Info, and still nothing written") {
    mech_rig::Rig base{shulib::bench::mechanismTableFor(shulib::bench::RobotId::RobotTwo)};
    mech_rig::Rig r{shulib::bench::mechanismTableFor(shulib::bench::RobotId::RobotTwo), [](hal::fake::FakeDeviceProvider& d) {
                        d.fakeMotor(6).setDeviceCurrentLimit(Current{1.8});
                    }};
    REQUIRE(r.sink.size() == base.sink.size() + 1);
    const std::string info =
        "'claw pivot 6': motor 1 (port 6): its current limit reads 1.80 A: a cap left by another program (HA-220), or "
        "VEXos's sharing with many motors plugged in (HA-221) -- nothing written";
    CHECK(levelOf(r, info) == hal::LogLevel::Info);
    CHECK(r.devices.fakeMotor(6).currentLimitWrites() == 0);
    CHECK(r.devices.fakeMotor(6).currentLimit().value() == 1.8);
}

// ═══ 4. THE MOTOR ENFORCES IT — on the host test plant (CA7, CA8, CA9, CA15) ═════════════

namespace {
/// The probes' spool lift on the host test plant: a red cartridge direct on a 1.5 in spool, its true kV
/// the cartridge's, 0.3 V of friction, `kG` of gravity, its stops at 0 and 24 in (`top`: the top stop's
/// height, inches), starting at `startIn` with its encoders reading 0 there.
sim::ArmPlantConfig spoolPlant(double kG, double startIn = 0.0, double top = 24.0) {
    sim::ArmPlantConfig pc = pos_rig::armPlant(100, 1, kG, 1.0, 0.3, startIn * kDegPerIn);
    pc.constantGravity = true;
    pc.minStopDeg = 0.0;
    pc.maxStopDeg = top * kDegPerIn;
    pc.encoderZeroDeg = 0.0;
    return pc;
}
/// A spool lift's spec with `stallAmps`, `capAmps` (0: none), `kG` and homing `h` — robot one's cascade
/// is its own robot file's (its numbers owed by the team); this is the probes' lift.
mech::MechanismSpec liftSpec(double stallAmps, double capAmps, double kG, mech::Homing h) {
    using namespace shulib::mechanism::words;
    mech::Lift l{.name = "lift", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in),
                 .safe = Hold, .stall = {.amps = stallAmps, .belowMotorRpm = 5, .lasting = 0.5_s},
                 .currentCap = {.amps = capAmps}, .range = {0_in, 24_in}, .gravity = constantGravity(Voltage{kG}),
                 .home = h, .presets = {{"floor", 0_in}, {"high", 22_in}}};
    return l.spec();
}
mech::MechanismSpec restingLift(double stallAmps, double capAmps, double kG = 1.0) {
    return liftSpec(stallAmps, capAmps, kG, mech::restsAtStop(shulib::units::Length{0.0}));
}
}  // namespace

TEST_CASE("MECH-S-A CA7 (STARVED, P10): a 12 V push into an obstacle with NOTHING ticking for 1.5 s -- at or under the cap throughout") {
    const auto starved = [](double cap) {
        pos_rig::ArmRig r{restingLift(1.0, cap), spoolPlant(1.2)};
        r.plant->setObstacleDeg(15.0 * kDegPerIn);
        r.slot().jog(Voltage{12.0});
        int ticks = 0;
        while (r.trueDeg() < 15.0 * kDegPerIn - 1e-9 && ticks < 1000) {
            r.tick();
            r.slot().jog(Voltage{12.0});
            ++ticks;
        }
        REQUIRE(ticks < 1000);  // it reached the obstacle
        r.tick(10);             // 0.1 s pushing, ticked (the stall watch needs 0.5 s: it has not tripped)
        REQUIRE(r.volts() == doctest::Approx(12.0));
        double worst = 0.0;
        for (int i = 0; i < 150; ++i) {  // STARVED: the world steps, the service never ticks
            r.world.pace();
            worst = std::max(worst, r.amps());
        }
        CHECK(r.volts() == doctest::Approx(12.0));  // still commanded at full: nothing limited the command
        return worst;
    };
    CHECK(starved(0.0) == doctest::Approx(2.5));        // uncapped: the motor's own 2.5 A
    CHECK(starved(1.2) <= 1.2 + 1e-12);                  // BUG CAUGHT (MA4): the plant ignoring the limit reads 2.5
}

TEST_CASE("MECH-S-A CA8 (P10): a move into an obstacle under a cap -- STALLED as uncapped, pulling no more than the cap after contact") {
    struct Out {
        MechanismOutcome outcome;
        double elapsed;
        double worstAfterContact;
        int stallFaults;
    };
    const auto run = [](double cap) {
        pos_rig::ArmRig r{restingLift(1.0, cap), spoolPlant(1.2)};
        r.plant->setObstacleDeg(15.0 * kDegPerIn);
        r.slot().moveTo(22.0, Time{4.0}, false);
        bool contact = false;
        double worst = 0.0;
        int n = 0;
        while (r.slot().running() && n < 1000) {
            r.tick();
            ++n;
            if (!contact && r.trueDeg() >= 15.0 * kDegPerIn - 1e-9) {
                contact = true;
                continue;  // the impact sample: never judged (landmine 4 — the plant stops dead in one step)
            }
            if (contact) {
                worst = std::max(worst, r.amps());
            }
        }
        REQUIRE(contact);
        return Out{r.slot().lastStep().outcome, r.slot().lastStep().elapsed, worst, r.stallFaults()};
    };
    const Out free = run(0.0);
    const Out capped = run(1.2);
    CHECK(free.outcome == MechanismOutcome::Stalled);
    CHECK(capped.outcome == MechanismOutcome::Stalled);
    CHECK(capped.stallFaults == 1);
    CHECK(free.worstAfterContact == doctest::Approx(2.5));
    CHECK(capped.worstAfterContact <= 1.2 + 1e-12);
    MESSAGE("CA8: stalled uncapped at " << free.elapsed << " s, capped at " << capped.elapsed << " s");
    CHECK(std::abs(capped.elapsed - free.elapsed) <= 0.25);  // the cap did not cost the stall watch its verdict
}

TEST_CASE("MECH-S-A CA9 (P11, D5): homing by stall UNDER a cap -- UP into the top stop, and DOWN into the bottom stop (the team's cascade)") {
    struct Out {
        MechanismOutcome outcome;
        double worstAfterContact;
        bool homed;
    };
    const auto home = [](mech::Homing h, double cap, double startIn, double kG) {
        pos_rig::ArmRig r{liftSpec(cap > 0.0 ? cap - 0.5 : 0.5, cap, kG, h), spoolPlant(kG, startIn)};
        const double stopDeg = h.at > 1.0 ? 24.0 * kDegPerIn : 0.0;
        r.slot().home(Time{4.0});
        bool contact = false;
        double worst = 0.0;
        int n = 0;
        while (r.slot().running() && n < 1000) {
            r.tick();
            ++n;
            if (!contact && std::abs(r.trueDeg() - stopDeg) < 1e-9) {
                contact = true;
                continue;  // the impact sample: never judged (landmine 4)
            }
            if (contact) {
                worst = std::max(worst, r.amps());
            }
        }
        REQUIRE(contact);
        return Out{r.slot().lastStep().outcome, worst, r.slot().homed()};
    };
    using shulib::units::Length;
    // UP: a 12 V push into the top stop, its signature 0.75 A under a 1.0 A cap (P11: 0.78 s, <= 1.00 A)
    const mech::Homing up = mech::homeByStall(Length{24.0}, Voltage{12.0}, {.amps = 0.75, .belowMotorRpm = 5,
                                                                           .lasting = Time{0.2}});
    const Out upFree = home(up, 0.0, 10.0, 1.2);
    const Out upCapped = home(up, 1.0, 10.0, 1.2);
    CHECK(upFree.outcome == MechanismOutcome::Succeeded);
    CHECK(upFree.worstAfterContact == doctest::Approx(2.5));
    CHECK(upCapped.outcome == MechanismOutcome::Succeeded);
    CHECK(upCapped.homed);
    CHECK(upCapped.worstAfterContact <= 1.0 + 1e-12);
    // DOWN (the team's 4-stage cascade, strings both ways, homed by stalling DOWN into its bottom stop —
    // the MECH-S brief's D5 was written for a gravity-return lift; logged as this run's delta): a -6 V push
    // (1.25 A against the stop uncapped) under a 1.0 A cap, its signature 0.6 A
    const mech::Homing down = mech::homeByStall(Length{0.0}, Voltage{-6.0}, {.amps = 0.6, .belowMotorRpm = 5,
                                                                             .lasting = Time{0.2}});
    const Out downFree = home(down, 0.0, 8.0, 1.2);
    const Out downCapped = home(down, 1.0, 8.0, 1.2);
    CHECK(downFree.outcome == MechanismOutcome::Succeeded);
    CHECK(downFree.worstAfterContact > 1.0);  // the cap matters here: uncapped it pushes harder
    CHECK(downCapped.outcome == MechanismOutcome::Succeeded);
    CHECK(downCapped.homed);
    CHECK(downCapped.worstAfterContact <= 1.0 + 1e-12);
}

TEST_CASE("MECH-S-A CA15 (the checker's V1): FOUR currents in order -- a 1.0 A signature under a 1.2 A cap holds a 3 V lift clean; a 2.5 V piece ends the hold STALLED") {
    pos_rig::ArmRig r{restingLift(1.0, 1.2, 3.0), spoolPlant(3.0)};
    // the boot said the signature is within 0.2 A of the cap (exactly 0.2 — the ordering is strict)
    CHECK(r.mechCount("'lift': its stall signature (1.00 A) is within 0.2 A of its current cap (1.20 A): a capped "
                      "motor may read a little under its cap, so it may never trip (HA-223)") == 1);
    r.slot().moveTo(12.0, Time{6.0}, false);
    r.runStep();
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    double hold = 0.0;
    for (int i = 0; i < 1200; ++i) {  // 12 s of holding the 3 V lift
        r.tick();
        hold = r.amps();
    }
    CHECK(r.stallFaults() == 0);
    MESSAGE("CA15: the 3 V lift holds at " << hold << " A");
    CHECK(hold < 1.0);
    r.plant->addLoadVolts(2.5);  // a game piece: the hold now needs more than the signature
    double withPiece = 0.0;  // the most the hold drew with the piece, before the trip let the motor go
    for (int i = 0; i < 600 && r.stallFaults() == 0; ++i) {
        r.tick();
        if (r.stallFaults() == 0) {
            withPiece = std::max(withPiece, r.amps());
        }
    }
    MESSAGE("CA15: with the piece the hold drew up to " << withPiece << " A");
    // the guide's number (review: it was a MESSAGE only): above the 1.0 A signature, at or under the cap
    CHECK(withPiece > 1.0);
    CHECK(withPiece <= 1.2 + 1e-12);
    CHECK(r.stallFaults() == 1);  // the documented reason the four currents must keep their order
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    // the guide states the order — pinned here so the text and the reason stay together
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/docs/guide/18-mechanisms.md"};
    REQUIRE(in.good());
    std::stringstream guide;
    guide << in.rdbuf();
    CHECK(guide.str().find("the heaviest **hold** (with a game piece) < the stall signature < the cap − 0.2 A") !=
          std::string::npos);
}

// ═══ 5. BIND'S RULE for the limits (a Devices that disagrees with its declaration) ═══════════════

namespace {
template <class F>
std::string bindRefusal(F&& f) {
    try {
        f();
    } catch (const shulib::PreconditionError& e) {
        return e.what();
    }
    return "";
}
}  // namespace

TEST_CASE("MECH-S-A bind: a current limit only for a BUILT motor mechanism's own motors -- past its motors, on a pneumatic or an ABSENT one refused") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    mech_rig::World world;
    world.clock = &clock;
    hal::fake::FakeMotor m1, m2;
    std::array<hal::IMotor*, 1> one{&m1};
    hal::MotorMechanism g1{one, hal::BrakeMode::Hold, "pivot"};
    hal::fake::FakeDigitalOut lineA;
    std::array<hal::IDigitalOut*, 1> l1{&lineA};
    hal::PneumaticMechanism p1{l1, false, "claw"};
    const mech::MechanismSpec pivot = cap_fx::pivot.spec();
    const mech::MechanismSpec claw = mech_fixture::robot_two::kRobotTwoMechanisms.specs[2];
    const auto bindOne = [&](const mech::MechanismSpec& s, const mech::Devices& d) {
        mech::MechanismService svc{world};
        const std::string msg = bindRefusal([&] {
            svc.bind(std::span<const mech::MechanismSpec>{&s, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
                     sink);
        });
        CHECK_FALSE(svc.bound());  // nothing stored ...
        CHECK(m1.currentLimitWrites() == 0);  // ... and nothing written: every check comes first
        CHECK(m2.currentLimitWrites() == 0);
        return msg;
    };
    const std::string why = "a current limit for a motor the mechanism does not have";
    mech::Devices past{.motors = &g1};
    past.currentLimits[0] = &m1;
    past.currentLimits[1] = &m2;  // a one-motor mechanism with a second limit
    CHECK(bindOne(pivot, past).find(why) != std::string::npos);
    mech::Devices last{.motors = &g1};
    last.currentLimits[3] = &m2;  // the last slot
    CHECK(bindOne(pivot, last).find(why) != std::string::npos);
    mech::Devices onLines{.pneumatic = &p1};
    onLines.currentLimits[0] = &m1;
    CHECK(bindOne(claw, onLines).find(why) != std::string::npos);
    mech::Devices absent{.absence = "port 3: nothing plugged in"};
    absent.currentLimits[0] = &m1;
    CHECK(bindOne(pivot, absent).find(why) != std::string::npos);
    // and the legal form binds, and caps
    mech::MechanismService ok{world};
    mech::Devices good{.motors = &g1};
    good.currentLimits[0] = &m1;
    ok.bind(std::span<const mech::MechanismSpec>{&pivot, 1}, std::span<const mech::Devices>{&good, 1}, clock, faults,
            sink);
    CHECK(ok.bound());
    CHECK(m1.currentLimit().value() == 0.8);
}
