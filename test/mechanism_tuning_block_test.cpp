// mechanism/live_tuning.hpp — THE COPY BLOCK (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief's D9 (a),
// re-check #14, §2.2 TA5, TA9's block half, TA10). ADVERSARIAL: a TEST-LOCAL PARSER reads the block back
// the way a student copies it — each line into the robot file's own declaration (its `.tuning` cleared:
// the block restates every `.tuning` number the file needs) — and the declaration's spec() must equal the
// LIVE spec FIELD BY FIELD, bit for bit. A block that leaves out a derived number whose re-derivation
// differs (landmine 12) builds to another spec; a block that states every number (a stale kV after a
// cartridge change) fails the minimal-set pins. A random walk of presses checks the printed precision
// round-trips every number the page can produce. Every line at most 200 bytes (TermSink's cut —
// landmine 16). Over the host test plant (HA-183); nothing here has run on a robot.

#include "doctest.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "mechanism_live_tuning_rig.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/mechanism/live_tuning.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace units = shulib::units;
using mech::Tunable;
using tune_rig::TuneRig;

/// The block mechanism `k` copies: every TUNE line (begin, members, end).
std::vector<std::string> block(const TuneRig& r, std::size_t k, int* count = nullptr) {
    shulib::hal::fake::FakeTelemetrySink s;
    const int n = r.tuning.copyBlock(k, s);
    if (count != nullptr) {
        *count = n;
    }
    std::vector<std::string> out;
    for (int i = 0; i < s.size(); ++i) {
        REQUIRE(s.at(i).subsystem == "TUNE");
        REQUIRE(s.at(i).level == shulib::hal::LogLevel::Info);
        out.push_back(s.at(i).message);
    }
    return out;
}
/// Only the member lines (between the begin and the end line), checking those two and the count.
std::vector<std::string> members(const TuneRig& r, std::size_t k) {
    int n = -1;
    const std::vector<std::string> all = block(r, k, &n);
    REQUIRE(all.size() >= 2);
    const std::string name = r.tuning.spec(k).name;
    CHECK(all.front().rfind("'" + name + "': copy begins -- write these into its declaration (tuned live): ", 0) == 0);
    CHECK(all.back() == "'" + name + "': copy ends -- " + std::to_string(n) + (n == 1 ? " line" : " lines"));
    CHECK(static_cast<int>(all.size()) == n + 2);
    for (const std::string& l : all) {
        CHECK(l.size() <= 200u);  // TermSink cuts a message past 200 bytes (diag/term_sink.hpp; landmine 16)
    }
    return std::vector<std::string>(all.begin() + 1, all.end() - 1);
}

// ── THE TEST-LOCAL PARSER: one line into a declaration, the C++ way a student writes it ────────────
/// Whether the block names any `.tuning` number (then they replace the file's `.tuning` whole).
bool hasTuning(const std::vector<std::string>& lines) {
    for (const std::string& l : lines) {
        if (l.rfind(".tuning.", 0) == 0) {
            return true;
        }
    }
    return false;
}
/// sscanf that must read the WHOLE line (the review: "2.05_volts)" would still give "%lf_volt)" its two
/// numbers — sscanf stops at the first literal that differs). -1 when anything is left over.
template <class... A>
int scanAll(const std::string& l, const char* fmt, A*... a) {
    const std::string f = std::string{fmt} + "%n";
    int n = -1;
    const int got = std::sscanf(l.c_str(), f.c_str(), a..., &n);
    return n == static_cast<int>(l.size()) ? got : -1;
}
bool parseCommon(mech::Stall& stall, mech::CurrentCap& cap, const std::string& l) {
    double a = 0;
    double b = 0;
    double c = 0;
    if (scanAll(l, ".stall = {.amps = %lf, .belowMotorRpm = %lf, .lasting = %lf_s}", &a, &b, &c) == 3) {
        stall = mech::Stall{.amps = a, .belowMotorRpm = b, .lasting = units::Time{c}};
        return true;
    }
    if (scanAll(l, ".currentCap = {.amps = %lf}", &a) == 1) {
        cap = mech::CurrentCap{.amps = a};
        return true;
    }
    return false;
}
bool parseTuning(mech::PositionTuning& t, const std::string& l) {
    char name[32] = {};
    double v = 0;
    int end = -1;
    if (std::sscanf(l.c_str(), ".tuning.%31[^ ] = %lf%n", name, &v, &end) != 2) {
        return false;
    }
    const std::string n{name};
    CHECK(l.substr(static_cast<std::size_t>(end)) == (n == "settle" ? "_s" : ""));  // nothing else after the number
    if (n == "kP") t.kP = v;
    else if (n == "kI") t.kI = v;
    else if (n == "kD") t.kD = v;
    else if (n == "kS") t.kS = v;
    else if (n == "kV") t.kV = v;
    else if (n == "kA") t.kA = v;
    else if (n == "maxSpeed") t.maxSpeed = v;
    else if (n == "maxAcceleration") t.maxAcceleration = v;
    else if (n == "within") t.within = v;
    else if (n == "settle") {
        REQUIRE(l.size() > 2);
        CHECK(l.substr(l.size() - 2) == "_s");
        t.settle = units::Time{v};
    } else {
        return false;
    }
    return true;
}
/// A position declaration (an Arm or a Lift) with the block's lines written into it.
template <class Decl>
mech::MechanismSpec rebuilt(Decl d, const std::vector<std::string>& lines, bool inches) {
    if (hasTuning(lines)) {
        d.tuning = {};  // the block's .tuning lines REPLACE the file's .tuning = {...} whole
    }
    for (const std::string& l : lines) {
        INFO(l);
        double a = 0, b = 0, c = 0, e = 0, f = 0;
        if (parseCommon(d.stall, d.currentCap, l) || parseTuning(d.tuning, l)) {
            continue;
        }
        if (inches && scanAll(l, ".range = {.min = %lf_in, .max = %lf_in}", &a, &b) == 2) {
            if constexpr (std::is_same_v<Decl, mech::Lift>) {
                d.range = mech::InchRange{units::Length{a}, units::Length{b}};
            }
            continue;
        }
        if (!inches && scanAll(l, ".range = {.min = %lf, .max = %lf}", &a, &b) == 2) {
            if constexpr (std::is_same_v<Decl, mech::Arm>) {
                d.range = mech::DegreeRange{a, b};
            }
            continue;
        }
        if (scanAll(l, ".gravity = constantGravity(%lf_volt)", &a) == 1) {
            d.gravity = mech::constantGravity(units::Voltage{a});
            continue;
        }
        if (scanAll(l, ".gravity = cosine(%lf_volt, %lf)", &a, &b) == 2) {
            d.gravity = mech::cosine(units::Voltage{a}, b);
            continue;
        }
        const char* stallHome = inches ? ".home = homeByStall(%lf_in, %lf_volt, {.amps = %lf, .belowMotorRpm = %lf, .lasting = %lf_s})"
                                       : ".home = homeByStall(%lf, %lf_volt, {.amps = %lf, .belowMotorRpm = %lf, .lasting = %lf_s})";
        if (scanAll(l, stallHome, &a, &b, &c, &e, &f) == 5) {
            d.home = mech::homeByStall(a, units::Voltage{b}, mech::Stall{c, e, units::Time{f}});
            continue;
        }
        const char* switchHome = inches ? ".home = homeBySwitch(%lf_in, %lf_volt)" : ".home = homeBySwitch(%lf, %lf_volt)";
        if (scanAll(l, switchHome, &a, &b) == 2) {
            d.home = mech::homeBySwitch(a, units::Voltage{b});
            continue;
        }
        FAIL("a line the parser does not know: " << l);
    }
    return d.spec();
}
mech::MechanismSpec rebuiltFlywheel(mech::Flywheel d, const std::vector<std::string>& lines) {
    if (hasTuning(lines)) {
        d.tuning = {};
    }
    for (const std::string& l : lines) {
        INFO(l);
        double a = 0, b = 0;
        char name[16] = {};
        if (parseCommon(d.stall, d.currentCap, l)) {
            continue;
        }
        if (scanAll(l, ".atSpeed = within(%lf, %lf_s)", &a, &b) == 2) {
            d.atSpeed = mech::within(a, units::Time{b});
            continue;
        }
        if (scanAll(l, ".tuning.%15[^ ] = %lf", name, &a) == 2) {
            const std::string n{name};
            if (n == "kS") d.tuning.kS = a;
            else if (n == "kV") d.tuning.kV = a;
            else if (n == "kP") d.tuning.kP = a;
            else if (n == "kI") d.tuning.kI = a;
            else FAIL("a flywheel tuning line the parser does not know: " << l);
            continue;
        }
        FAIL("a line the parser does not know: " << l);
    }
    return d.spec();
}
mech::MechanismSpec rebuiltPower(mech::Power d, const std::vector<std::string>& lines) {
    for (const std::string& l : lines) {
        INFO(l);
        double a = 0;
        if (parseCommon(d.stall, d.currentCap, l)) {
            continue;
        }
        if (scanAll(l, ".maxVolts = %lf_volt", &a) == 1) {
            d.maxVolts = units::Voltage{a};
            continue;
        }
        FAIL("a line the parser does not know: " << l);
    }
    return d.spec();
}
/// The block of mechanism `k`, read back into its robot-file declaration, equals the live spec field by field.
void roundTrips(const TuneRig& r, std::size_t k) {
    const std::vector<std::string> lines = members(r, k);
    const mech::MechanismSpec& live = r.tuning.spec(k);
    std::string name = live.name;
    mech::MechanismSpec back{};
    if (name == "cascade") back = rebuilt(tune_fixture::cascade, lines, true);
    else if (name == "lift") back = rebuilt(tune_fixture::tallLift, lines, true);
    else if (name == "mid home lift") back = rebuilt(tune_fixture::midHome, lines, true);
    else if (name == "eighteen char lift") back = rebuilt(tune_fixture::longNumbers, lines, true);
    else if (name == "a twenty-three char lft") back = rebuilt(tune_fixture::longName, lines, true);
    else if (name == "swing") back = rebuilt(tune_fixture::swing, lines, false);
    else if (name == "switch lift") back = rebuilt(tune_fixture::switchLift, lines, true);
    else if (name == "flywheel") back = rebuiltFlywheel(tune_fixture::flywheel, lines);
    else if (name == "pivot") back = rebuiltPower(tune_fixture::pivot, lines);
    else if (name == "roller") back = rebuiltPower(tune_fixture::roller, lines);
    else FAIL("no declaration for " << name);
    std::string all;
    for (const std::string& l : lines) {
        all += l + "\n";
    }
    INFO(all);
    CHECK(tune_rig::firstDifference(back, live) == "");
}
}  // namespace

// ── TA10 ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA10: the SMALLEST block -- a top speed alone carries the acceleration it was tuned with, and NOT kV; kP alone is kP only") {
    TuneRig r{tune_fixture::kCascade};
    CHECK(members(r, 0).empty());  // untuned: a begin, an end, "0 lines"
    REQUIRE(r.tuning.apply(0, Tunable::MaxSpeed, 2.0).ok);
    const std::vector<std::string> a = members(r, 0);
    REQUIRE(a.size() == 2);
    CHECK(a[0] == ".tuning.maxSpeed = 2");
    // the acceleration as it RAN (the starting 0.25 s ramp of the file's 5.89 in/s), not 2 / 0.25 = 8
    char want[64];
    std::snprintf(want, sizeof want, ".tuning.maxAcceleration = %s",
                  mech::detail::shortest(r.tuning.spec(0).position.maxAcceleration).text);
    CHECK(a[1] == want);
    CHECK(r.tuning.spec(0).position.maxAcceleration != 8.0);
    for (const std::string& l : a) {
        CHECK(l.find(".tuning.kV") == std::string::npos);  // derived, untouched: never stated (a stale kV later)
    }
    roundTrips(r, 0);

    TuneRig p{tune_fixture::kCascade};
    REQUIRE(p.tuning.step(0, Tunable::KP, +1).ok);
    CHECK(members(p, 0) == std::vector<std::string>{".tuning.kP = 0.3125"});
    roundTrips(p, 0);
    // and stepped back to the file's number: nothing to copy, nothing changed
    REQUIRE(p.tuning.step(0, Tunable::KP, -1).ok);
    CHECK(members(p, 0).empty());
    CHECK_FALSE(p.tuning.changed(0));

    // the acceleration alone: the top speed stays derived
    TuneRig q{tune_fixture::kCascade};
    REQUIRE(q.tuning.step(0, Tunable::MaxAcceleration, -1).ok);
    const std::vector<std::string> c = members(q, 0);
    REQUIRE(c.size() == 1);
    CHECK(c[0].rfind(".tuning.maxAcceleration = ", 0) == 0);
    roundTrips(q, 0);
}

TEST_CASE("TA10: a number the robot file STATES stays in the block (the student replaces .tuning = {...} whole), in member order") {
    TuneRig r{tune_fixture::kTallLift};  // .tuning = {.kD = 0.01}
    CHECK(members(r, 0).empty());
    // a change OUTSIDE .tuning: no .tuning line — the file's own .tuning = {.kD = 0.01} stays as written
    REQUIRE(r.tuning.step(0, Tunable::StallAmps, -1).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".stall = {.amps = 1.95, .belowMotorRpm = 5, .lasting = 0.5_s}"});
    roundTrips(r, 0);
    REQUIRE(r.tuning.undo().ok);
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".tuning.kP = 0.3125", ".tuning.kD = 0.01"});
    roundTrips(r, 0);
}

TEST_CASE("TA9: the block states kA only when it is not 0") {
    TuneRig r{tune_fixture::kSwing};
    REQUIRE(r.tuning.step(0, Tunable::KA, +1).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".tuning.kA = 0.0014"});
    roundTrips(r, 0);
    REQUIRE(r.tuning.step(0, Tunable::KA, -1).ok);
    CHECK(r.tuning.value(0, Tunable::KA) == 0.0);
    CHECK(members(r, 0).empty());
}

// ── TA5 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA5: every member a block can name reads back into the declaration as the live spec, field by field") {
    {
        TuneRig r{tune_fixture::kCascade};
        for (const auto& [t, d] : std::vector<std::pair<Tunable, int>>{
                 {Tunable::KP, +1}, {Tunable::StallAmps, -1}, {Tunable::StallRpm, +1}, {Tunable::CurrentCap, -1},
                 {Tunable::RangeMax, -1}, {Tunable::GravityHold, +1}, {Tunable::Settle, +1}, {Tunable::Within, -1},
                 {Tunable::KV, +1}, {Tunable::KS, +1}}) {
            REQUIRE(r.tuning.step(0, t, d).ok);
        }
        const std::vector<std::string> l = members(r, 0);
        CHECK(l == std::vector<std::string>{
                       ".stall = {.amps = 1.95, .belowMotorRpm = 5.25, .lasting = 0.5_s}", ".currentCap = {.amps = 2.45}",
                       ".range = {.min = 0_in, .max = 23.75_in}", ".gravity = constantGravity(1.05_volt)",
                       ".tuning.kP = 0.3125", ".tuning.kS = 0.05", ".tuning.kV = " +
                           std::string{mech::detail::shortest(r.tuning.value(0, Tunable::KV)).text},
                       ".tuning.within = 0.2", ".tuning.settle = 0.15_s"});
        roundTrips(r, 0);
    }
    {
        TuneRig r{tune_fixture::kMidHome};
        REQUIRE(r.tuning.step(0, Tunable::HomeVolts, -1).ok);
        REQUIRE(r.tuning.step(0, Tunable::HomeStallLasting, +1).ok);
        CHECK(members(r, 0) == std::vector<std::string>{
                                   ".home = homeByStall(6_in, -2.05_volt, {.amps = 0.6, .belowMotorRpm = 4, .lasting = 0.25_s})"});
        roundTrips(r, 0);
    }
    {
        TuneRig r{tune_fixture::kSwing};
        REQUIRE(r.tuning.step(0, Tunable::GravityHold, -1).ok);
        REQUIRE(r.tuning.step(0, Tunable::HomeStallAmps, +1).ok);
        REQUIRE(r.tuning.step(0, Tunable::RangeMax, -1).ok);
        CHECK(members(r, 0) == std::vector<std::string>{
                                   ".range = {.min = 0, .max = 119}", ".gravity = cosine(0.95_volt, 30)",
                                   ".home = homeByStall(0, -3_volt, {.amps = 0.5, .belowMotorRpm = 2, .lasting = 0.2_s})"});
        roundTrips(r, 0);
    }
    {
        TuneRig r{tune_fixture::kFlyAndPivot};
        REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
        REQUIRE(r.tuning.step(0, Tunable::AtSpeedBand, -1).ok);
        REQUIRE(r.tuning.step(1, Tunable::MaxVolts, -1).ok);
        REQUIRE(r.tuning.step(1, Tunable::StallLasting, +1).ok);
        REQUIRE(r.tuning.step(2, Tunable::CurrentCap, -1).ok);
        CHECK(members(r, 0) == std::vector<std::string>{".atSpeed = within(57.1429, 0.1_s)", ".tuning.kP = 0.0625"});
        CHECK(members(r, 1) == std::vector<std::string>{".stall = {.amps = 1.5, .belowMotorRpm = 5, .lasting = 0.35_s}",
                                                        ".maxVolts = 11.95_volt"});
        CHECK(members(r, 2) == std::vector<std::string>{".currentCap = {.amps = 2.45}"});
        roundTrips(r, 0);
        roundTrips(r, 1);
        roundTrips(r, 2);
    }
}

TEST_CASE("TA5: long typed numbers and the longest name -- every line within 200 bytes, read back exactly") {
    TuneRig r{tune_fixture::kLongNumbers};
    for (const auto& [t, d] : std::vector<std::pair<Tunable, int>>{
             {Tunable::HomeVolts, -1}, {Tunable::StallLasting, +1}, {Tunable::RangeMax, -1}, {Tunable::GravityHold, -1},
             {Tunable::CurrentCap, -1}, {Tunable::MaxSpeed, -1}, {Tunable::KD, -1}}) {
        INFO(mech::parameterOf(t).name);
        REQUIRE(r.tuning.step(0, t, d).ok);
    }
    std::size_t longest = 0;
    for (const std::string& l : block(r, 0)) {
        longest = std::max(longest, l.size());
    }
    CHECK(longest <= 200);
    CHECK(longest > 100);  // the homing line, with its long typed numbers
    roundTrips(r, 0);
    // the bound by construction: the longest line is 77 fixed bytes and five numbers of at most 24 characters
    CHECK(77 + 5 * mech::kCopyNumberChars <= 200);
    CHECK(std::strlen(mech::detail::shortest(-2.2250738585072014e-308).text) == mech::kCopyNumberChars);
    // the longest NAME: its begin and end lines within 200 bytes too
    TuneRig n{tune_fixture::kLongName};
    REQUIRE(n.tuning.step(0, Tunable::KP, +1).ok);
    for (const std::string& l : block(n, 0)) {
        CHECK(l.size() <= 200u);
    }
    roundTrips(n, 0);
}

TEST_CASE("TA5: a random walk of presses -- after every few, each mechanism's block reads back as its live spec (the printed precision round-trips)") {
    struct Robot {
        const mech::MechanismTable* table;
    };
    const Robot robots[] = {{&tune_fixture::kCascade}, {&tune_fixture::kMidHome}, {&tune_fixture::kSwing},
                            {&tune_fixture::kFlyAndPivot}, {&tune_fixture::kTallLift}, {&tune_fixture::kSwitchLift}};
    unsigned seed = 12345u;
    const auto next = [&seed] {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) & 0x7fffu;
    };
    int applied = 0;
    for (const Robot& rb : robots) {
        TuneRig r{*rb.table};
        for (int press = 0; press < 240; ++press) {
            const std::size_t k = next() % r.tuning.count();
            const mech::ParameterList l = mech::parametersOf(r.tuning.spec(k));
            if (l.count == 0) {
                continue;
            }
            const Tunable t = l.items[next() % l.count].id;
            const int dir = (next() & 1u) != 0u ? +1 : -1;
            const bool fine = (next() % 4u) == 0u;
            const mech::Applied a = r.tuning.step(k, t, dir, fine);
            applied += a.ok ? 1 : 0;
            if (press % 12 == 11) {
                for (std::size_t m = 0; m < r.tuning.count(); ++m) {
                    roundTrips(r, m);
                }
            }
        }
    }
    CHECK(applied > 600);  // most presses are inside the rules: the walk really moved the numbers
}

TEST_CASE("the copy block's begin line names the declaration's file and line") {
    TuneRig r{tune_fixture::kCascade};
    const std::vector<std::string> b = block(r, 0);
    const std::string& first = b.front();
    CHECK(first.find("mechanism_live_tuning_rig.hpp:") != std::string::npos);
    CHECK(first.substr(first.rfind(':') + 1) == std::to_string(r.tuning.spec(0).declaredLine));
}

// ── TUNE1-A's review (2026-11-10): the smallest block, exactly ───────────────────────────────────
TEST_CASE("TA10 (review): a DERIVED acceleration is never stated -- a file stating only its top speed, kP changed: kP and the top speed, no acceleration") {
    using namespace shulib::mechanism::words;
    static constexpr Lift statedSpeed{.name = "stated speed", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                                      .shape = spool(1.5_in), .safe = Hold,
                                      .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                                      .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt),
                                      .home = restsAtStop(0_in), .tuning = {.maxSpeed = 5}};
    static constexpr mech::MechanismTable table = mech::mechanisms(mech::wire(statedSpeed, mech::motors(1)));
    TuneRig r{table};
    REQUIRE(r.tuning.spec(0).position.maxAcceleration == 20.0);  // derived: 5 in/s reached in 0.25 s
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    const std::vector<std::string> l = members(r, 0);
    CHECK(l == std::vector<std::string>{".tuning.kP = 0.3125", ".tuning.maxSpeed = 5"});  // 20 re-derives: not stated
    CHECK(tune_rig::firstDifference(rebuilt(statedSpeed, l, true), r.tuning.spec(0)) == "");
}

TEST_CASE("TA10 (review): a number the file states AT the value the library would work it out to is LEFT OUT -- the block still rebuilds the same spec") {
    using namespace shulib::mechanism::words;
    // a Green 12:36 lift: free 400 deg/s at the output, so the starting kV is 12/400 = 0.03 — the file states exactly it
    static constexpr Lift statedAtStart{.name = "at start", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                                        .shape = spool(1.5_in), .safe = Hold,
                                        .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                                        .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt),
                                        .home = restsAtStop(0_in), .tuning = {.kP = 0.4, .kV = 0.03}};
    static constexpr mech::MechanismTable table = mech::mechanisms(mech::wire(statedAtStart, mech::motors(1)));
    TuneRig r{table};
    REQUIRE(mech::positionStartingValues(r.tuning.spec(0)).kV == 0.03);
    REQUIRE(r.tuning.step(0, Tunable::MaxSpeed, -1).ok);
    const std::vector<std::string> l = members(r, 0);
    REQUIRE(l.size() == 3);
    CHECK(l[0] == ".tuning.kP = 0.4");  // stated, and the library would make it 0.25: kept
    CHECK(l[1].rfind(".tuning.maxSpeed = ", 0) == 0);
    CHECK(l[2].rfind(".tuning.maxAcceleration = ", 0) == 0);
    for (const std::string& x : l) {
        CHECK(x.find(".tuning.kV") == std::string::npos);  // stated AT its derived 0.03: not in the block (the header says so)
    }
    CHECK(tune_rig::firstDifference(rebuilt(statedAtStart, l, true), r.tuning.spec(0)) == "");
}

TEST_CASE("TA10 (review): the header's example block, exactly -- the cascade, one [+] on kP and a 2 in/s top speed") {
    TuneRig r{tune_fixture::kCascade};
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    REQUIRE(r.tuning.apply(0, Tunable::MaxSpeed, 2.0).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".tuning.kP = 0.3125", ".tuning.maxSpeed = 2",
                                                    ".tuning.maxAcceleration = 23.561944901923447"});
    roundTrips(r, 0);
}

TEST_CASE("TA5 (review): a SWITCH home's line reads back -- .home = homeBySwitch(at, push) exactly as the declaration writes it") {
    TuneRig r{tune_fixture::kSwitchLift};
    REQUIRE(r.tuning.step(0, Tunable::HomeVolts, +1).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".home = homeBySwitch(12_in, 6.05_volt)"});
    roundTrips(r, 0);
    REQUIRE(r.tuning.step(0, Tunable::KP, -1).ok);
    REQUIRE(r.tuning.step(0, Tunable::RangeMin, +1).ok);
    CHECK(members(r, 0) == std::vector<std::string>{".range = {.min = 0.25_in, .max = 24_in}",
                                                    ".home = homeBySwitch(12_in, 6.05_volt)", ".tuning.kP = 0.2"});
    roundTrips(r, 0);
}

// ── TUNE1-A's verification round (2026-11-11): the FLYWHEEL's smallest block ──────────────────────
TEST_CASE("TA10 (verification round): a FLYWHEEL's numbers its file STATES stay in the block, in member order, beside the one changed live -- and the derived kV is never stated") {
    using namespace shulib::mechanism::words;
    // the fixture flywheel, its file stating kS and kI (not their starting 0 and 0.2): the student replaces
    // .tuning = {...} whole with the block, so a stated number the block left out would fall back to its start
    static constexpr Flywheel statedFly{.name = "stated fly", .motors = {Forward}, .cartridge = Blue,
                                        .gearing = gears(72, 12), .safe = Coast,
                                        .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                                        .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 3000}},
                                        .tuning = {.kS = 0.3, .kI = 0.5}};
    static constexpr mech::MechanismTable table = mech::mechanisms(mech::wire(statedFly, mech::motors(1)));
    TuneRig r{table};
    REQUIRE(mech::speedStartingValues(r.tuning.spec(0)).kI == 0.2);  // so 0.5 is a STATED number
    CHECK(members(r, 0).empty());
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    const std::vector<std::string> l = members(r, 0);
    CHECK(l == std::vector<std::string>{".tuning.kS = 0.3", ".tuning.kP = 0.0625", ".tuning.kI = 0.5"});
    CHECK(tune_rig::firstDifference(rebuiltFlywheel(statedFly, l), r.tuning.spec(0)) == "");
    // the same press on the fixture flywheel (its file states no .tuning): kP alone
    TuneRig f{tune_fixture::kFlyAndPivot};
    REQUIRE(f.tuning.step(0, Tunable::KP, +1).ok);
    CHECK(members(f, 0) == std::vector<std::string>{".tuning.kP = 0.0625"});
    roundTrips(f, 0);
}
