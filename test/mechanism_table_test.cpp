// Adversarial tests for the mechanism table (chunk T3, the program half's prep, 2026-09-26; the
// pneumatic claw from T3's addendum, E2; REWRITTEN for the mechanism core's words by MECH-A5,
// 2026-10-29): robot two's claw pivots and its pneumatic claw exactly as the team lead stated them,
// now as DECLARATIONS and their wiring in `namespace robot_two`; the claw's open level on ONE line;
// WHERE each declaration was written (the recorded path, pinned by its suffix); the region G3a's
// generator wraps, byte for byte the test fixture's; the selection for EVERY robot; the drive check
// over every smart ROLE and the IMU (mechanismsClearOf — a SENSOR on a drive port included, the MECH
// brief's M29); "is this a declared mechanism port"; "shulib Calibrate"'s candidate filter as a pure
// function over the new list; and the one Calibrate call site pinned by reading its source.
//
// WHAT LEFT WITH THE OLD ENTRY TYPES (MECH-A5). The cases that broke the table's own five
// consistency checks one field at a time (an UNSET field, a port of 0 / 22 / a repeat, a line of 0 /
// 9, a count past the array, the polarity typed as a literal) are GONE with those checks: the
// declarations and mechanisms(...) refuse every one AT BUILD, by name, and the compile-refusal
// harness (test/compile_refusals/) proves each refusal fires (MECH-A1). This file pins the facts
// only this file can: robot two's VALUES, and this file's own drive check and selection.
//
// WHAT THIS TU SEES. The list lives in src/ (a per-robot composition fact, not library API) and is
// PROS-free, so this test includes it from src/. Robot two's list is defined in every build under its
// own name (robot_two::kRobotTwoMechanisms), so the contents cases read THE REAL DECLARATIONS, while
// kMechanisms, the build's selection, is the host's: no build switch is set, so
// src/build_selection.hpp's chain takes its #else (the bench build) and none is declared. The
// selection itself, mechanismTableFor(), is called for EVERY robot below.
//
// WHAT THIS TU CANNOT DO: run the call site. src/programs/calibrate_program.cpp is a whole PROS program
// (pros/screen.h, pros/misc.h and the rest are not in the host shim). So the filter's LOGIC lives in
// the table's pure dropDeclaredMechanismPorts(), which the cases here RUN; and what is left at the call
// site is pinned by reading the program's code, comments stripped, EXACTLY (the last case) — UNCHANGED
// by MECH-A5: the call reads `kMechanisms`, which IS the new list (the MECH brief's N9 fallback; CAL1
// Part B replaces the drop). What that cannot see — that the lines print on the robot and no claw
// moves — is worksheet Station J's.

#include "doctest.h"

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/mechanism/wiring.hpp"

using shulib::bench::ChassisTable;
using shulib::bench::dropDeclaredMechanismPorts;
using shulib::bench::DroppedMechanismPorts;
using shulib::bench::isDeclaredMechanismPort;
using shulib::bench::kMaxPort;
using shulib::bench::kMechanisms;
using shulib::bench::kNoMechanisms;
using shulib::bench::MechanismTable;
using shulib::bench::mechanismsClearOf;
using shulib::bench::tableSideOf;
namespace mech = shulib::mechanism;
namespace two = shulib::bench::robot_two;

namespace {

/// Robot two's REAL drive table (src/robots/chassis_table.hpp's kRobotTwoChassis: LEFT -11 +12 -13
/// +14 -15, RIGHT +20 -19 +18 -17 +16, IMU on port 2 — test/chassis_table_test.cpp pins those
/// literals), a copy for a case to break one field of.
[[nodiscard]] ChassisTable robotTwoChassis() { return shulib::bench::kRobotTwoChassis; }

/// A copy of robot two's REAL list, for a case to break one field of (a hand-built list: the build's
/// checks ran on the original only).
[[nodiscard]] MechanismTable robotTwo() { return two::kRobotTwoMechanisms; }

[[nodiscard]] std::vector<int> asInts(const std::int8_t* p, std::size_t n) {
    std::vector<int> v;
    for (std::size_t i = 0; i < n; ++i) v.push_back(static_cast<int>(p[i]));
    return v;
}

/// What one call of Calibrate's filter left behind: the count it returned, the WHOLE port array
/// afterwards (every slot the caller handed in, so a stale port past the kept ones is visible),
/// and the dropped list.
struct Filtered {
    std::size_t kept;
    std::vector<int> array;
    std::vector<int> dropped;

    /// The first `kept` slots: the ports that stay drive candidates.
    [[nodiscard]] std::vector<int> keptPorts() const {
        return {array.begin(), array.begin() + static_cast<std::ptrdiff_t>(kept)};
    }
    /// Every slot past the kept ones: must all be 0 (never a port).
    [[nodiscard]] bool tailCleared() const {
        for (std::size_t i = kept; i < array.size(); ++i) {
            if (array[i] != 0) return false;
        }
        return true;
    }
};

/// Runs the filter the way Calibrate does — in place on a port array — with the dropped list
/// PRE-FILLED WITH GARBAGE, so a filter that forgot to reset it shows the garbage.
[[nodiscard]] Filtered filter(const MechanismTable& t, const std::vector<int>& ports) {
    std::vector<std::int8_t> v;
    for (const int p : ports) v.push_back(static_cast<std::int8_t>(p));
    DroppedMechanismPorts dropped{};
    for (std::int8_t& p : dropped.ports) p = 99;
    dropped.count = 3;
    const std::size_t kept =
        dropDeclaredMechanismPorts(t, std::span<std::int8_t>{v.data(), v.size()}, dropped);
    return Filtered{kept, asInts(v.data(), v.size()), asInts(dropped.ports, dropped.count)};
}

/// src/programs/calibrate_program.cpp's CODE: every comment removed and every run of whitespace squeezed
/// to one space, so a pinned statement matches however clang-format wraps it, and a statement
/// that has been commented out no longer counts as present. String and character literals are
/// copied whole (a "//" inside one is not a comment; the spaces inside one are kept); a ' right
/// after a letter, digit or _ is a digit separator, not a literal.
[[nodiscard]] std::string codeOnly(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool space = false;
    const auto flush = [&] {
        if (space && !out.empty()) out.push_back(' ');
        space = false;
    };
    std::size_t i = 0;
    while (i < src.size()) {
        const char c = src[i];
        const char next = i + 1 < src.size() ? src[i + 1] : '\0';
        if (c == '/' && next == '/') {
            while (i < src.size() && src[i] != '\n') ++i;
            space = true;
            continue;
        }
        if (c == '/' && next == '*') {
            const std::size_t end = src.find("*/", i + 2);
            i = end == std::string::npos ? src.size() : end + 2;
            space = true;
            continue;
        }
        const bool separator =
            c == '\'' && i > 0
            && (std::isalnum(static_cast<unsigned char>(src[i - 1])) != 0 || src[i - 1] == '_');
        if ((c == '"' || c == '\'') && !separator) {
            flush();
            out.push_back(c);
            ++i;
            while (i < src.size() && src[i] != c) {
                if (src[i] == '\\' && i + 1 < src.size()) out.push_back(src[i++]);
                out.push_back(src[i++]);
            }
            if (i < src.size()) out.push_back(src[i++]);
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            space = true;
            ++i;
            continue;
        }
        flush();
        out.push_back(c);
        ++i;
    }
    return out;
}

/// A file of this repository, read whole (SHULIB_SOURCE_DIR is the repo root on the host suite).
[[nodiscard]] std::string readRepoFile(const char* relative) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/" + relative};
    REQUIRE(in.good());
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

/// How many times `needle` occurs in `hay` (non-overlapping).
[[nodiscard]] std::size_t occurrences(const std::string& hay, const std::string& needle) {
    std::size_t n = 0;
    for (std::size_t at = hay.find(needle); at != std::string::npos;
         at = hay.find(needle, at + needle.size())) {
        ++n;
    }
    return n;
}

/// The 1-based line of the FIRST line of the table file holding `text` (0 if none): the recorded-path
/// pins read the file, so a moved line moves its pin with it.
[[nodiscard]] std::uint32_t tableLineOf(const std::string& text) {
    const std::string file = readRepoFile("src/robots/mechanism_table.hpp");
    const std::size_t at = file.find(text);
    if (at == std::string::npos) return 0;
    std::uint32_t line = 1;
    for (std::size_t i = 0; i < at; ++i) line += file[i] == '\n' ? 1U : 0U;
    return line;
}

/// The text strictly between `begin` and `end` (both whole lines) in `file`, or "" when either is
/// missing or not exactly once.
[[nodiscard]] std::string region(const std::string& file, const std::string& begin, const std::string& end) {
    if (occurrences(file, begin) != 1 || occurrences(file, end) != 1) return "";
    const std::size_t b = file.find(begin) + begin.size();
    const std::size_t e = file.find(end);
    return e > b ? file.substr(b, e - b) : "";
}

[[nodiscard]] bool endsWith(const char* s, const std::string& suffix) {
    if (s == nullptr) return false;
    const std::string v{s};
    return v.size() >= suffix.size() && v.compare(v.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

TEST_CASE("mechanism table: robot two's claw pivots are EXACTLY what the team lead stated "
          "(2026-09-26) -- the real declarations and wiring, not a copy") {
    // BUG CAUGHT: any typo in src/robots/mechanism_table.hpp's robot-two region — a wrong port (the
    // control drives some other motor), a pivot declared Reversed without a measurement, a wrong
    // cartridge (RED gearing written as something else), a safe state other than Hold (a released claw
    // drops), a stall signature nobody measured, a ceiling below full power (R1 / R2 would no longer
    // be 12 V), the pivots merged into one two-motor mechanism (both claws would move at once), or the
    // list's ORDER changed (the boot's config index, the claw row and every boot line follow it). The
    // BUTTONS and the VOLTS are the driver file's (test/robot_two_driver_test.cpp).
    const MechanismTable& t = two::kRobotTwoMechanisms;
    REQUIRE(t.count == 3);
    struct Pivot {
        std::size_t index;
        const char* name;
        int port;
    };
    for (const Pivot p : {Pivot{0, "claw pivot 6", 6}, Pivot{1, "claw pivot 7", 7}}) {
        CAPTURE(p.name);
        const mech::MechanismSpec& s = t.specs[p.index];
        CHECK(std::string_view{s.name} == p.name);
        CHECK(s.kind == mech::Kind::Power);
        REQUIRE(s.motorCount == 1);
        CHECK_FALSE(s.motors[0].reversed);  // + = the motor's own forward: the way is UNMEASURED (HA-153)
        CHECK(s.cartridge == shulib::hal::MotorCartridge::Red);
        CHECK(s.safeMode == shulib::config::SafeBrakeMode::Hold);
        CHECK_FALSE(s.power.stall.written());  // no signature: the driver is the only cut-off (HA-154)
        CHECK(s.power.maxVolts == 12.0);       // full power passes through
        CHECK(s.lineCount == 0);
        CHECK(s.angleSensor.kind == mech::SensorKind::None);
        CHECK(s.pieceSensor.kind == mech::SensorKind::None);
        const mech::MechanismWiring& w = t.wiring[p.index];
        CHECK(w.motors[0] == p.port);
        CHECK(w.motors[1] == 0);
        CHECK(w.angleSensor == 0);
        CHECK(w.pieceSensorPort == 0);
        // ...and the config the boot derives from it: the signed port as typed, RED, Hold.
        const shulib::config::MechanismConfig c = mech::toMechanismConfig(s, w);
        CHECK(c.motorPorts[0] == p.port);
        CHECK(c.motorCount == 1);
        CHECK(c.cartridge == shulib::hal::MotorCartridge::Red);
        CHECK(c.safeMode == shulib::config::SafeBrakeMode::Hold);
    }
    CHECK(mechanismsClearOf(t, robotTwoChassis()));
}

TEST_CASE("mechanism table: robot two's PNEUMATIC claw is EXACTLY what the team lead stated "
          "(2026-09-26) -- 3-wire A, 'closed' at the start and when made safe, closed = LOW, a 0.5 s "
          "move (HA-185)") {
    // BUG CAUGHT: any typo in the claw's declaration — the wrong 3-wire line (the toggle fires some
    // other valve, or nothing), the claw starting OPEN (it drops its preload the moment the program
    // starts), a safe position of OPEN (a library-forced safe state — the run guard's end of run —
    // would drop what the claw holds), the open level flipped without a measurement (the claw then
    // OPENS at program start, while HA-159 and Station J.9 still quote LOW = closed), a second line
    // nobody reported, `grabs` lost from 'closed' (a sensorless grab would then SUCCEED instead of
    // reading UNCONFIRMED, counted done — the team lead's Q2 (c)), or the move time changed without
    // Station J (an auton's set() would wait a time nobody measured — HA-185, PROVISIONAL).
    const MechanismTable& t = two::kRobotTwoMechanisms;
    const mech::MechanismSpec& s = t.specs[2];
    CHECK(std::string_view{s.name} == "claw");
    CHECK(s.kind == mech::Kind::Pneumatic);
    CHECK(s.motorCount == 0);
    REQUIRE(s.lineCount == 1);  // ONE line: what was reported (single or double, HA-159)
    const mech::PneumaticSpec& q = s.pneumatic;
    REQUIRE(q.positionCount == 2);
    CHECK(std::string_view{q.positions[0].name} == "closed");
    CHECK_FALSE(q.positions[0].levels[0]);  // closed = LOW: the GUESS (HA-159), Station J.9 settles it
    CHECK(q.positions[0].confirm == mech::Confirm::Sees);  // `grabs`
    CHECK(std::string_view{q.positions[1].name} == "open");
    CHECK(q.positions[1].levels[0]);        // open = HIGH ("line ON = open", the T3 addendum's start)
    CHECK(q.positions[1].confirm == mech::Confirm::None);
    CHECK(q.start == 0);                    // 'closed'
    CHECK(q.safe == 0);                     // 'closed'
    CHECK(q.actuationTime == 0.5);          // PROVISIONAL (HA-185): Station J times it
    CHECK(q.confirmWindow == 0.0);
    CHECK(s.pieceSensor.kind == mech::SensorKind::None);  // nothing can confirm a grab
    const mech::MechanismWiring& w = t.wiring[2];
    CHECK(w.lines[0] == mech::Line::A);
    CHECK(w.lines[1] == mech::Line::None);
    // The LEVELS the boot derives: CLOSED is LOW at the start and as the safe value.
    const shulib::config::MechanismConfig c = mech::toMechanismConfig(s, w);
    CHECK(c.pneumaticCount == 1);
    CHECK(c.pneumaticPorts[0] == 1);
    CHECK(c.pneumaticStart == shulib::config::PneumaticLevel::Low);
    CHECK(c.pneumaticSafe == shulib::config::PneumaticLevel::Low);
    // The claw's line is a 3-WIRE port: it does not make smart port 1 a mechanism (Calibrate would
    // then drop a motor on port 1 for a claw that is not there).
    CHECK_FALSE(isDeclaredMechanismPort(t, 1));
}

TEST_CASE("mechanism table: the claw's open level is written ONCE, on its .positions line -- a swap "
          "of Low and High moves the start AND safe levels together, and nothing else names a level") {
    // BUG CAUGHT: the claw's level stated in a second place (a start or safe LEVEL beside the
    // positions — flipping one would leave the other, and the claw would open at boot or on a forced
    // safe state), or a projection that reads a position's NAME instead of its levels. Station J.9's
    // fix for a claw that opens on the wrong press is ONE edit: swap Low and High on that line.
    const std::string code = codeOnly(readRepoFile("src/robots/mechanism_table.hpp"));
    CHECK(occurrences(code, ".positions = ") == 1);
    CHECK(occurrences(code, ".positions = {{\"closed\", {Low}, grabs}, {\"open\", {High}}},") == 1);
    CHECK(occurrences(code, ".start = \"closed\",") == 1);  // a position's NAME, never a level
    CHECK(occurrences(code, ".safe = \"closed\",") == 1);
    CHECK(occurrences(code, "Low") == 1);
    CHECK(occurrences(code, "High") == 1);
    // The swap, applied to a copy: both levels follow, the positions' names do not move.
    mech::MechanismSpec swapped = two::kRobotTwoMechanisms.specs[2];
    swapped.pneumatic.positions[0].levels[0] = true;   // closed = HIGH
    swapped.pneumatic.positions[1].levels[0] = false;  // open = LOW
    const shulib::config::MechanismConfig c = mech::toMechanismConfig(swapped, two::kRobotTwoMechanisms.wiring[2]);
    CHECK(c.pneumaticStart == shulib::config::PneumaticLevel::High);
    CHECK(c.pneumaticSafe == shulib::config::PneumaticLevel::High);
    CHECK(swapped.pneumatic.start == 0);
    CHECK(swapped.pneumatic.safe == 0);
}

TEST_CASE("mechanism table: WHERE each of robot two's declarations and wire(...) lines was written -- "
          "its own line of src/robots/mechanism_table.hpp, pinned by the path's SUFFIX") {
    // BUG CAUGHT (the MECH brief's §3.1, moved here from the fixture by MECH-A5 — A1's D18): a
    // recorded location that is not the team's own line — the closing brace, a library header, the
    // fixture's copy — would send the bench page's direction check ("change it HERE") to the wrong
    // line. The path is the one the compiler opened, so host and brain differ: the SUFFIX is pinned.
    const MechanismTable& t = two::kRobotTwoMechanisms;
    struct Pin {
        std::size_t index;
        const char* nameLine;
        const char* wireLine;
    };
    for (const Pin pin : {Pin{0, "inline constexpr Power clawPivot6{.name = \"claw pivot 6\",",
                              "    wire(clawPivot6, motors(6)),"},
                          Pin{1, "inline constexpr Power clawPivot7{.name = \"claw pivot 7\",",
                              "    wire(clawPivot7, motors(7)),"},
                          Pin{2, "    .name = \"claw\",", "    wire(claw, lines('A')));"}}) {
        CAPTURE(pin.nameLine);
        const mech::MechanismSpec& s = t.specs[pin.index];
        const std::uint32_t expected = tableLineOf(pin.nameLine);
        REQUIRE(expected > 0);
        CHECK(s.declaredLine == expected);
        CHECK(endsWith(s.declaredFile, "src/robots/mechanism_table.hpp"));
        for (std::size_t m = 0; m < s.motorCount; ++m) {
            CHECK(s.motors[m].line == expected);  // a motor role points at its `.name` line
            CHECK(endsWith(s.motors[m].file, "src/robots/mechanism_table.hpp"));
        }
        CHECK(t.wiring[pin.index].line == tableLineOf(pin.wireLine));
        CHECK(endsWith(t.wiring[pin.index].file, "src/robots/mechanism_table.hpp"));
    }
    CHECK(t.specs[2].pneumatic.levelsLine == tableLineOf("    .name = \"claw\","));
}

TEST_CASE("mechanism table: robot two's region is ONE block G3a's generator can wrap, and it is BYTE FOR "
          "BYTE the test fixture's (test/mechanism_fixtures.hpp)") {
    // BUG CAUGHT: a hand-written line inside the region (G3a's generator would overwrite it, or refuse
    // the file — the MECH brief's §6.2 item 2), a marker missing or doubled, the selection or a check
    // drifting INTO the region, or the fixture every library test of robot two runs drifting from the
    // real file (a library test would then pass on declarations the robot does not have).
    const std::string file = readRepoFile("src/robots/mechanism_table.hpp");
    const std::string real = region(file, "// [robot two mechanisms]\n", "// [/robot two mechanisms]\n");
    REQUIRE_FALSE(real.empty());
    CHECK(occurrences(real, "inline constexpr ") == 4);  // the three declarations and the list
    CHECK(occurrences(real, "wire(") == 3);
    CHECK(real.find("mechanismTableFor") == std::string::npos);
    CHECK(real.find("static_assert") == std::string::npos);
    CHECK(real.find("namespace") == std::string::npos);
    const std::string fixture =
        region(readRepoFile("test/mechanism_fixtures.hpp"), "// [robot two mechanisms]\n", "// [/robot two mechanisms]\n");
    CHECK(real == fixture);
    // ...and the fixture's list equals the real one, field by field through the projection.
    const MechanismTable& f = mech_fixture::robot_two::kRobotTwoMechanisms;
    REQUIRE(f.count == two::kRobotTwoMechanisms.count);
    for (std::size_t i = 0; i < f.count; ++i) {
        CAPTURE(i);
        const shulib::config::MechanismConfig a = mech::toMechanismConfig(f.specs[i], f.wiring[i]);
        const shulib::config::MechanismConfig b =
            mech::toMechanismConfig(two::kRobotTwoMechanisms.specs[i], two::kRobotTwoMechanisms.wiring[i]);
        CHECK(std::string_view{a.name} == std::string_view{b.name});
        CHECK(a.motorPorts[0] == b.motorPorts[0]);
        CHECK(a.pneumaticPorts[0] == b.pneumaticPorts[0]);
        CHECK(a.pneumaticStart == b.pneumaticStart);
        CHECK(a.pneumaticSafe == b.pneumaticSafe);
        CHECK(f.specs[i].pneumatic.actuationTime == two::kRobotTwoMechanisms.specs[i].pneumatic.actuationTime);
    }
}

TEST_CASE("mechanism table: the HOST build's selection is 'none' -- only robot two declares mechanisms") {
    // BUG CAUGHT: the selection handing robot two's list to a build that is not robot two (the bench
    // bot's Calibrate would then refuse its own ports 6 and 7 for claws it does not have). The tank
    // side is the next case's, and the file's static_asserts keyed on kRobotId, on every `make
    // ROBOT=tank` (the src build gate's four tank builds compile them).
    CHECK(&kMechanisms == &kNoMechanisms);
    CHECK(kMechanisms.count == 0);
    for (int p = 1; p <= kMaxPort; ++p) {
        CAPTURE(p);
        CHECK_FALSE(isDeclaredMechanismPort(kMechanisms, p));
    }
    // On the host the filter therefore keeps everything, in order.
    const Filtered f = filter(kMechanisms, {6, 7, 11, 20});
    CHECK(f.kept == 4);
    CHECK(f.array == std::vector<int>{6, 7, 11, 20});
    CHECK(f.dropped.empty());
}

TEST_CASE("mechanism table: the selection -- mechanismTableFor() gives robot two its claws and every "
          "other robot none, BY ADDRESS, and each robot's list is clear of its own drive") {
    // BUG CAUGHT: a wrong robot in mechanismTableFor() — robot two handed kNoMechanisms (Calibrate
    // would take its claw pivots as drive candidates again, and "shulib Drive" would have no claws),
    // or the bench bot handed robot two's claws. By ADDRESS, never a count (G3a's ask: a generated list
    // may hold any number). The per-robot loop is today's three robots (G3a Part 2's kEveryRobotId will
    // replace the list when it lands — the MECH brief's §6.2 item 8).
    using shulib::bench::chassisTableFor;
    using shulib::bench::mechanismTableFor;
    using shulib::bench::RobotId;
    CHECK(&mechanismTableFor(RobotId::RobotTwo) == &two::kRobotTwoMechanisms);
    CHECK(&mechanismTableFor(RobotId::BenchBot) == &kNoMechanisms);
    CHECK(&mechanismTableFor(RobotId::InventedXDrive) == &kNoMechanisms);
    // The host is the bench build.
    CHECK(&kMechanisms == &mechanismTableFor(shulib::bench::kRobotId));
    for (const RobotId r : {RobotId::BenchBot, RobotId::RobotTwo, RobotId::InventedXDrive}) {
        CAPTURE(static_cast<int>(r));
        CHECK(mechanismsClearOf(mechanismTableFor(r), chassisTableFor(r)));
    }
}

TEST_CASE("mechanism table: 'is this a declared mechanism port' -- 6 and 7 yes, every drive port "
          "and every other port no") {
    // BUG CAUGHT: a predicate that matches too much (Calibrate would skip a DRIVE motor and calibrate
    // a side one short), too little (it would gear, coast and pulse a claw), or matches the port-less
    // value 0. The sweep includes port 1: the pneumatic claw's 3-WIRE line A is not smart port 1.
    const MechanismTable& t = two::kRobotTwoMechanisms;
    CHECK(isDeclaredMechanismPort(t, 6));
    CHECK(isDeclaredMechanismPort(t, 7));
    const ChassisTable drive = robotTwoChassis();
    for (int p = -3; p <= kMaxPort + 3; ++p) {
        CAPTURE(p);
        if (p == 6 || p == 7) continue;
        CHECK_FALSE(isDeclaredMechanismPort(t, p));
    }
    // Every drive port of robot two, explicitly, both sides.
    for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}) {
        CAPTURE(p);
        CHECK(tableSideOf(drive, p) != 0);
        CHECK_FALSE(isDeclaredMechanismPort(t, p));
    }
    // A pivot declared REVERSED (the fix Station J may call for) is still its physical port: the
    // wiring holds physical ports and the direction is the declaration's, so flipping a pivot can
    // never hand a claw back to Calibrate.
    MechanismTable flipped = robotTwo();
    flipped.specs[0].motors[0].reversed = true;
    flipped.specs[1].motors[0].reversed = true;
    CHECK(isDeclaredMechanismPort(flipped, 6));
    CHECK(isDeclaredMechanismPort(flipped, 7));
    CHECK_FALSE(isDeclaredMechanismPort(flipped, -6));  // a PHYSICAL port is never negative
    // A role left at port 0 is not "port 0 is a mechanism": 0 is never a port.
    MechanismTable zero = robotTwo();
    zero.wiring[1].motors[0] = 0;
    CHECK_FALSE(isDeclaredMechanismPort(zero, 0));
    // Entries past `count` are not declared, whatever they hold.
    MechanismTable one = robotTwo();
    one.count = 1;
    CHECK(isDeclaredMechanismPort(one, 6));
    CHECK_FALSE(isDeclaredMechanismPort(one, 7));
    // A motor role past the declaration's motorCount is not a motor (a stale wiring slot).
    MechanismTable stale = robotTwo();
    stale.wiring[0].motors[1] = 9;
    CHECK_FALSE(isDeclaredMechanismPort(stale, 9));
    // A SENSOR's port is not a MOTOR port: the census lists motors (header) — robot y's wrist has its
    // angle sensor on 10 and its roller a distance sensor on 9.
    const MechanismTable& y = mech_fixture::robot_y::kRobotYMechanisms;
    CHECK(isDeclaredMechanismPort(y, 1));
    CHECK_FALSE(isDeclaredMechanismPort(y, 10));
    CHECK_FALSE(isDeclaredMechanismPort(y, 9));
    // Every motor of a two-motor mechanism (robot y's DR4B on 3 and 4).
    CHECK(isDeclaredMechanismPort(y, 3));
    CHECK(isDeclaredMechanismPort(y, 4));
}

TEST_CASE("mechanism table: the predicate's 1..21 guard at its EDGES -- a mechanism on port 21 "
          "or port 1 is declared and dropped; 0 and 22 are never ports") {
    // BUG CAUGHT: an off-by-one in the guard (`port >= 21`, `port < 2`). Robot two's claws are on 6
    // and 7, where no such slip shows; a mechanism added later on port 21 (free on robot two) or port
    // 1 would then NOT be dropped, and Calibrate would gear, coast and pulse it with the suite green.
    MechanismTable edges = robotTwo();
    edges.wiring[0].motors[0] = 21;
    edges.wiring[1].motors[0] = 1;
    CHECK(isDeclaredMechanismPort(edges, 21));
    CHECK(isDeclaredMechanismPort(edges, 1));
    for (const int p : {0, 2, 6, 7, 20, 22}) {
        CAPTURE(p);
        CHECK_FALSE(isDeclaredMechanismPort(edges, p));
    }
    const Filtered f = filter(edges, {1, 2, 11, 20, 21});
    CHECK(f.keptPorts() == std::vector<int>{2, 11, 20});
    CHECK(f.dropped == std::vector<int>{1, 21});
    CHECK(f.tailCleared());
    // A role OUTSIDE 1..21 (a list the build refuses) never makes that number a port.
    for (const int bad : {22, 0, 255}) {
        CAPTURE(bad);
        MechanismTable beyond = robotTwo();
        beyond.wiring[0].motors[0] = static_cast<std::uint8_t>(bad);
        CHECK_FALSE(isDeclaredMechanismPort(beyond, 22));
        CHECK_FALSE(isDeclaredMechanismPort(beyond, 0));
        CHECK(isDeclaredMechanismPort(beyond, 7));  // the other entry still counts
    }
}

TEST_CASE("mechanism table: Calibrate's candidate filter, in place -- the census's 6, 7, 11-18, "
          "20 leave EXACTLY the drive ports at the front, in order, zeros after, 6 and 7 dropped") {
    // BUG CAUGHT: a filter that drops a drive port, keeps a claw, reorders the list (the
    // calibrator's port list and its motor list must stay index-aligned), duplicates or loses a
    // port while re-listing in place, leaves a stale port past the kept ones, returns a count
    // that is not the number kept, or carries an old dropped list into the log.
    // The list is robot two's census as it will read once the claws are plugged in: the drive
    // motors the 2026-09-24/25 captures saw (11-18 and 20; 19 unplugged) plus 6 and 7.
    const Filtered f = filter(two::kRobotTwoMechanisms, {6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(f.kept == 9);
    CHECK(f.array == std::vector<int>{11, 12, 13, 14, 15, 16, 17, 18, 20, 0, 0});  // every slot
    CHECK(f.dropped == std::vector<int>{6, 7});  // and NOT the 99s the list was pre-filled with

    // Order is the CENSUS's, not sorted: an interleaved list comes back in its own order, the
    // claws in the middle and at the end.
    const Filtered mixed = filter(two::kRobotTwoMechanisms, {20, 7, 11, 6, 3});
    CHECK(mixed.kept == 3);
    CHECK(mixed.array == std::vector<int>{20, 11, 3, 0, 0});
    CHECK(mixed.dropped == std::vector<int>{7, 6});
    const Filtered last = filter(two::kRobotTwoMechanisms, {11, 12, 6});
    CHECK(last.array == std::vector<int>{11, 12, 0});
    CHECK(last.dropped == std::vector<int>{6});

    // The claws UNPLUGGED (today's captures): nothing is dropped, nothing moves, nothing is said.
    const Filtered today = filter(two::kRobotTwoMechanisms, {11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(today.kept == 9);
    CHECK(today.array == std::vector<int>{11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(today.dropped.empty());

    // Only the claws plugged in: both dropped, no candidate left (Calibrate then refuses on its
    // own "at least two" check, which now counts motors besides the declared mechanisms).
    const Filtered onlyClaws = filter(two::kRobotTwoMechanisms, {6, 7});
    CHECK(onlyClaws.kept == 0);
    CHECK(onlyClaws.array == std::vector<int>{0, 0});
    CHECK(onlyClaws.dropped == std::vector<int>{6, 7});

    // An empty census: nothing kept, and the garbage-filled dropped list comes back EMPTY.
    const Filtered none = filter(two::kRobotTwoMechanisms, {});
    CHECK(none.kept == 0);
    CHECK(none.array.empty());
    CHECK(none.dropped.empty());

    // Every one of the 21 ports at once (a full brain): 19 kept, 2 dropped, order kept.
    std::vector<int> all;
    std::vector<int> expect;
    for (int p = 1; p <= kMaxPort; ++p) {
        all.push_back(p);
        if (p != 6 && p != 7) expect.push_back(p);
    }
    const Filtered full = filter(two::kRobotTwoMechanisms, all);
    CHECK(full.kept == 19);
    CHECK(full.keptPorts() == expect);
    CHECK(full.tailCleared());
    CHECK(full.dropped == std::vector<int>{6, 7});

    // More declared ports than the dropped list holds (impossible from a census, which lists each
    // port once): every one still LEAVES the candidates; only the log would miss the extras.
    const Filtered flood = filter(two::kRobotTwoMechanisms, std::vector<int>(25, 6));
    CHECK(flood.kept == 0);
    CHECK(flood.tailCleared());
    CHECK(flood.dropped.size() == static_cast<std::size_t>(kMaxPort));
}

TEST_CASE("mechanism table: THE DRIVE CHECK -- no role, a MOTOR or a SENSOR, on a drive port (either "
          "side, by |port|) or the IMU's; a 3-wire line never collides with a smart port") {
    // BUG CAUGHT (the MECH brief's §1.13, mutation M29): a drive check over the MOTOR roles only (until
    // MECH-A5 it was `mechanismsDisjointFromChassis`, motors only) — an angle sensor or a piece sensor
    // typed onto a drive port would build, and at boot the provider would hand the drive's motor port
    // to a sensor adapter (RobotConfig::valid() refuses it at run time; this is the build's mirror). A
    // check reading one side only, the signed entry instead of |port|, missing the IMU, or reading a
    // 3-wire LINE number as a smart port (robot two's IMU is port 2 and line B is 2).
    const ChassisTable drive = robotTwoChassis();
    CHECK(mechanismsClearOf(two::kRobotTwoMechanisms, drive));
    for (std::size_t i = 0; i < 2; ++i) {
        CAPTURE(i);
        CHECK(tableSideOf(drive, two::kRobotTwoMechanisms.wiring[i].motors[0]) == 0);
    }
    MechanismTable onLeft = robotTwo();
    onLeft.wiring[0].motors[0] = 11;  // the left side's -11, a signed entry: matched by |port|
    CHECK_FALSE(mechanismsClearOf(onLeft, drive));
    MechanismTable onLeftLast = robotTwo();
    onLeftLast.wiring[0].motors[0] = 15;  // the left side's LAST entry, -15 (MECH-A5's review, R21: a
    CHECK_FALSE(mechanismsClearOf(onLeftLast, drive));  // loop stopping one short passed every other case)
    MechanismTable onRight = robotTwo();
    onRight.wiring[1].motors[0] = 16;  // the right side's +16, its LAST entry
    CHECK_FALSE(mechanismsClearOf(onRight, drive));
    MechanismTable onRightFirst = robotTwo();
    onRightFirst.wiring[1].motors[0] = 20;  // the right side's FIRST entry, +20
    CHECK_FALSE(mechanismsClearOf(onRightFirst, drive));
    MechanismTable onImu = robotTwo();
    onImu.wiring[1].motors[0] = 2;  // the IMU's port
    CHECK_FALSE(mechanismsClearOf(onImu, drive));
    // With no IMU stated (imuPort 0 = UNSET), no role collides with "the IMU".
    ChassisTable noImu = drive;
    noImu.imuPort = 0;
    CHECK(mechanismsClearOf(two::kRobotTwoMechanisms, noImu));
    // A 3-WIRE LINE whose number equals the IMU's port is a different connector: robot two's claw on
    // line A (1) beside an IMU on smart port 1, and on line B beside its real IMU on port 2.
    ChassisTable imuOne = drive;
    imuOne.imuPort = 1;
    CHECK(mechanismsClearOf(two::kRobotTwoMechanisms, imuOne));
    MechanismTable lineB = robotTwo();
    lineB.wiring[2].lines[0] = mech::Line::B;
    CHECK(mechanismsClearOf(lineB, drive));
    // THE SENSORS (M29). Robot y's roles sit on 1..10 and lines A and E; beside robot two's drive
    // with its IMU moved to 21 (robot y's swing is on 2), robot y is clear — and each SENSOR moved
    // onto a drive port or the IMU's is refused.
    ChassisTable driveImu21 = drive;
    driveImu21.imuPort = 21;
    const MechanismTable& y = mech_fixture::robot_y::kRobotYMechanisms;
    REQUIRE(mechanismsClearOf(y, driveImu21));
    MechanismTable angleOnLeft = y;
    angleOnLeft.wiring[0].angleSensor = 13;  // the wrist's rotation sensor on the left's -13
    CHECK_FALSE(mechanismsClearOf(angleOnLeft, driveImu21));
    MechanismTable pieceOnRight = y;
    pieceOnRight.wiring[4].pieceSensorPort = 18;  // the roller's optical sensor on the right's +18
    CHECK_FALSE(mechanismsClearOf(pieceOnRight, driveImu21));
    MechanismTable distanceOnImu = y;
    distanceOnImu.wiring[6].pieceSensorPort = 21;  // the grip's distance sensor on the IMU's port
    CHECK_FALSE(mechanismsClearOf(distanceOnImu, driveImu21));
    MechanismTable motorOnRight = y;
    motorOnRight.wiring[2].motors[1] = 19;  // the DR4B's SECOND motor on the right's -19
    CHECK_FALSE(mechanismsClearOf(motorOnRight, driveImu21));
    // ...and a robot with no mechanisms is clear of anything.
    CHECK(mechanismsClearOf(kNoMechanisms, drive));
    CHECK(mechanismsClearOf(kNoMechanisms, driveImu21));
}

TEST_CASE("mechanism table: the selection checks name robot two by ADDRESS, and the file has no #if") {
    // BUG CAUGHT: a selection pin by COUNT (G3a's generator may write any number — the MECH brief's §6.2
    // item 4), a selection made by a build switch read here (only src/build_selection.hpp's chain may —
    // the src build gate's step 0 says so too), or kMechanisms bound to a fixed robot.
    const std::string code = codeOnly(readRepoFile("src/robots/mechanism_table.hpp"));
    CHECK(code.find("#if") == std::string::npos);
    CHECK(code.find(".count ==") == std::string::npos);
    CHECK(occurrences(code, "inline constexpr const MechanismTable& kMechanisms = mechanismTableFor(kRobotId);") == 1);
    CHECK(occurrences(code, "static_assert(kRobotId != RobotId::RobotTwo || &kMechanisms == &robot_two::kRobotTwoMechanisms,") == 1);
    CHECK(occurrences(code, "static_assert(clearOfItsDrive(robot_two::kRobotTwoMechanisms, kRobotTwoChassis),") == 1);
    CHECK(occurrences(code, "static_assert(clearOfItsDrive(kMechanisms, kChassis),") == 1);
}

TEST_CASE("mechanism table: the source reader this suite pins Calibrate with -- comments go, "
          "literals stay whole, whitespace squeezes to one space") {
    // BUG CAUGHT: a reader that leaves comments in (a statement commented out would still "be
    // there"), cuts a string at a "//" inside it, squeezes the spaces inside a literal (the pinned
    // log line would stop matching the text the robot prints), or reads a digit separator as the
    // start of a character literal (everything after it would be taken for a literal).
    CHECK(codeOnly("a  // gone\n  b /* gone\n too */ c") == "a b c");
    CHECK(codeOnly("x(\"// kept  as is\", '/', 'a');") == "x(\"// kept  as is\", '/', 'a');");
    CHECK(codeOnly("s = \"a \\\" // b\"; // gone") == "s = \"a \\\" // b\";");
    CHECK(codeOnly("n = 1'000; // gone\nm = 2;") == "n = 1'000; m = 2;");
    CHECK(codeOnly("// n = f(x);\n") == "");
    CHECK(codeOnly("\n\t  {\n  }\n") == "{ }");
}

TEST_CASE("mechanism table: shulib Calibrate's ONE call site -- right after the census has filled "
          "the list, EXACTLY as pinned, and step 4 builds from the filtered list only") {
    // BUG CAUGHT: the call deleted, commented out, moved ABOVE the census loop (it would filter an
    // empty list, and the loop would then add 6 and 7 back as candidates), moved after step 4
    // (every claw already geared BLUE and set to Coast), fed some other table, its count thrown
    // away, the log loop walking the wrong list (the line Station J.2 checks would name drive
    // ports), the list touched again between the filter and step 4, or step 4 building from a
    // second copy of the census that the filter never saw.
    //
    // WHY BY READING THE SOURCE: runCalibrate() is a whole PROS program (pros/screen.h,
    // pros/misc.h and the screen, touch and SD code have no host shim), so it cannot be compiled,
    // let alone run, here. The LOGIC is in dropDeclaredMechanismPorts(), which the cases above
    // RUN; what is left at the call site — one assignment, one log loop, and where they sit — is
    // pinned here as text, EXACTLY: this case fails on any edit to those lines, and whoever edits
    // them updates this case in the same change. What text cannot show — that the lines print on
    // the robot and no claw moves — is worksheet Station J.2's.
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/src/programs/calibrate_program.cpp"};
    REQUIRE(in.good());
    std::stringstream text;
    text << in.rdbuf();
    const std::string code = codeOnly(text.str());
    REQUIRE(code.size() > 20000);

    // 1. THE FILTER, right where the census has FINISHED filling the list: the census loop's motor
    //    branch (it fills ports[], the ONE list), the loop's end, the census's closing count line,
    //    then the filter block — contiguous, so the block cannot move above the loop, and nothing
    //    can come between the census and the filter.
    const std::string filterSite =
        "if (kind == hal::DeviceKind::Motor && n < hal::kMaxSmartPorts) { "
        "ports[n] = static_cast<std::int8_t>(p); ++n; } } "
        "emitf(\"  %u motor(s), %d IMU(s)\", static_cast<unsigned>(n), imuCount); "
        "{ DroppedMechanismPorts dropped{}; "
        "n = dropDeclaredMechanismPorts(kMechanisms, {ports, n}, dropped); "
        "for (std::size_t i = 0; i < dropped.count; ++i) { "
        "emitf(\"port %d: claw pivot (declared) -- not a candidate, not touched\", "
        "static_cast<int>(dropped.ports[i])); } }";
    const std::size_t filter = code.find(filterSite);
    REQUIRE_MESSAGE(filter != std::string::npos,
                    "the census + filter block no longer reads exactly as pinned");
    CHECK(code.find(filterSite, filter + 1) == std::string::npos);
    const std::size_t census = code.find("emit(\"STAGE 0  DEVICE CENSUS");
    REQUIRE(census != std::string::npos);
    CHECK(census < filter);
    // Exactly one call: a second one would be a second filter to keep in step.
    const std::size_t call = code.find("dropDeclaredMechanismPorts(");
    REQUIRE(call != std::string::npos);
    CHECK(code.find("dropDeclaredMechanismPorts(", call + 1) == std::string::npos);

    // 2. STEP 4 builds each adapter from ports[i] — the filtered list — and from nothing else.
    const std::string step4 =
        "for (std::size_t i = 0; i < n; ++i) { const int port = ports[i]; try { "
        "candidates[built].port = port; ports[built] = static_cast<std::int8_t>(port); "
        "candidates[built].motor.emplace(ports[built], gearset); ++built; }";
    const std::size_t build = code.find(step4);
    REQUIRE_MESSAGE(build != std::string::npos, "step 4's loop no longer reads exactly as pinned");
    const std::size_t firstAdapter = code.find(".motor.emplace(");
    REQUIRE(firstAdapter != std::string::npos);
    CHECK(filter < build);
    CHECK(firstAdapter > build);  // the FIRST adapter anywhere is step 4's
    // The census writes no second copy of the list: nothing reads a candidate's port before step 4
    // has written it.
    CHECK(code.find("candidates[i].port") == std::string::npos);
    CHECK(code.find("candidates[n].port") == std::string::npos);

    // 3. Between the filter and step 4, nothing changes the count or the list.
    const std::size_t after = filter + filterSite.size();
    const std::string between = code.substr(after, build - after);
    for (const char* write : {"ports[", " n = ", " n += ", " n -= ", "++n", "--n", "n++", "n--",
                              "&n", "std::span"}) {
        CAPTURE(write);
        CHECK(between.find(write) == std::string::npos);
    }

    // The program includes the table it filters by (written from src/ since SRC1).
    CHECK(code.find("#include \"robots/mechanism_table.hpp\"") != std::string::npos);
}
