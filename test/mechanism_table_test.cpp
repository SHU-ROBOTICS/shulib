// Adversarial tests for the mechanism table (chunk T3, the program half's prep, 2026-09-26; the
// pneumatic claw from T3's addendum, E2, the same day): robot two's two claw pivots and its
// pneumatic claw exactly as the team lead stated them, the claw's polarity as ONE constant, the
// "is this a declared mechanism port" predicate, "shulib Calibrate"'s candidate filter as a pure
// function, the consistency checks broken one at a time with hand-built tables, and the one
// Calibrate call site pinned by reading its source.
//
// WHAT THIS TU SEES. The table lives in src/ (a per-robot composition fact, not library API) and
// is PROS-free so this test can include it by relative path, as test/chassis_table_test.cpp
// includes the chassis table. UNLIKE the chassis table, robot two's entries are defined in every
// build under their own name (kRobotTwoMechanisms), so the contents test below reads THE REAL
// BYTES — a swapped button in src/mechanism_table.hpp turns this file red — while kMechanisms, the
// build's selection, is the host's: no variant define, so none declared. Robot two's DRIVE ports
// are not visible here (the chassis table defines them only under the tank define, and defining it
// in this TU would give kChassis two definitions across the suite), so the disjointness case
// re-types them from src/chassis_table.hpp, exactly as chassis_table_test.cpp does; the header's
// own static_assert checks the real pair on every `make ROBOT=tank`.
//
// WHAT THIS TU CANNOT DO: run the call site. src/calibrate_program.cpp is a whole PROS program
// (pros/screen.h, pros/misc.h and the rest are not in the host shim). So the filter's LOGIC —
// re-listing the survivors in place, the new count, the dropped list — lives in the table's pure
// dropDeclaredMechanismPorts(), which the cases here RUN; and what is left at the call site (one
// assignment, one log loop, where they sit, and step 4 building from the filtered list) is pinned
// by reading the program's code, comments stripped, EXACTLY (the last case). What that cannot see
// — that the lines print on the robot and no claw moves — is worksheet Station J's.

#include "doctest.h"

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mechanism_table.hpp"

using shulib::bench::absPort;
using shulib::bench::Cartridge;
using shulib::bench::ChassisTable;
using shulib::bench::dropDeclaredMechanismPorts;
using shulib::bench::DroppedMechanismPorts;
using shulib::bench::isDeclaredMechanismPort;
using shulib::bench::kClawOpenCommand;
using shulib::bench::kMaxAdiLine;
using shulib::bench::kMaxMechanisms;
using shulib::bench::kMaxPneumatics;
using shulib::bench::kMaxPort;
using shulib::bench::kMechanisms;
using shulib::bench::kNoMechanisms;
using shulib::bench::kRobotTwoMechanisms;
using shulib::bench::kTeleopRezeroButton;
using shulib::bench::MechanismTable;
using shulib::bench::mechanismButtonsAvoid;
using shulib::bench::mechanismButtonsUnique;
using shulib::bench::mechanismCountValid;
using shulib::bench::mechanismForwardReverseDistinct;
using shulib::bench::mechanismPortsValid;
using shulib::bench::mechanismsDisjointFromChassis;
using shulib::bench::mechanismTableComplete;
using shulib::bench::mechanismVoltsValid;
using shulib::bench::motorEntryComplete;
using shulib::bench::motorEntryCount;
using shulib::bench::MotorMechanismEntry;
using shulib::bench::pneumaticCommandFor;
using shulib::bench::pneumaticEntryComplete;
using shulib::bench::pneumaticEntryCount;
using shulib::bench::pneumaticLinesValid;
using shulib::bench::PneumaticMechanismEntry;
using shulib::bench::PneumaticPosition;
using shulib::bench::pneumaticSafeCommand;
using shulib::bench::pneumaticStartCommand;
using shulib::bench::tableSideOf;
using shulib::hal::BrakeMode;
using shulib::hal::ControllerButton;

namespace {

/// Robot two's DRIVE table as committed in src/chassis_table.hpp (the SHULIB_ROBOT_TANK_2026
/// block), re-typed because this TU cannot see it (header): LEFT -11 +12 -13 +14 -15, RIGHT +20
/// -19 +18 -17 +16, IMU on port 2, BLUE.
[[nodiscard]] ChassisTable robotTwoChassis() {
    return ChassisTable{
        .robot = "test copy of robot two",
        .left = {-11, 12, -13, 14, -15},
        .leftCount = 5,
        .right = {20, -19, 18, -17, 16},
        .rightCount = 5,
        .cartridge = Cartridge::Blue,
        .imuPort = 2,
        .measured = true,
        .signsMeasured = true,
        .provenance = "test",
        .wheelDiameterIn = 2.75,
        .trackWidthIn = 0.0,
        .externalRatio = 0.0,
        .geometryProvenance = "test",
    };
}

/// A copy of robot two's REAL table, for a case to break one field of.
[[nodiscard]] MechanismTable robotTwo() { return kRobotTwoMechanisms; }

/// Every consistency check at once — what the header's static_asserts demand of a build's table
/// (all but check 8, the chassis pairing, which takes a second table).
[[nodiscard]] bool allChecksPass(const MechanismTable& t) {
    return mechanismCountValid(t) && mechanismTableComplete(t) && mechanismPortsValid(t)
           && pneumaticLinesValid(t) && mechanismForwardReverseDistinct(t)
           && mechanismButtonsUnique(t) && mechanismButtonsAvoid(t, kTeleopRezeroButton)
           && mechanismVoltsValid(t);
}

/// Robot two with a SECOND pneumatic entry — a copy of the claw on line `line`, toggled by
/// `toggle` — for the cases that need two of them (the real table declares one).
[[nodiscard]] MechanismTable robotTwoWithSecondPneumatic(int line, ControllerButton toggle) {
    MechanismTable t = kRobotTwoMechanisms;
    t.pneumatics[1] = t.pneumatics[0];
    t.pneumatics[1].name = "second pneumatic";
    t.pneumatics[1].adiLine = static_cast<std::uint8_t>(line);
    t.pneumatics[1].toggle = toggle;
    t.pneumaticCount = 2;
    return t;
}

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

/// src/calibrate_program.cpp's CODE: every comment removed and every run of whitespace squeezed
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

}  // namespace

TEST_CASE("mechanism table: robot two's claw pivots are EXACTLY what the team lead stated "
          "(2026-09-26) -- the real table, not a copy") {
    // BUG CAUGHT: any typo in src/mechanism_table.hpp's robot-two block — a swapped R1/R2 (the
    // claw runs backwards on the button the driver thinks is forward), the pivots' buttons crossed
    // (R1 moves the port-7 claw), a wrong port (the binding drives some other motor), a sign
    // flipped without a measurement, a wrong cartridge (RED gearing written as something else), a
    // safe mode other than Hold (a released claw drops), or a voltage other than full power.
    const MechanismTable& t = kRobotTwoMechanisms;
    REQUIRE(t.motorCount == 2);

    const MotorMechanismEntry& six = t.motors[0];
    CHECK(std::strcmp(six.name, "claw pivot 6") == 0);
    CHECK(six.signedPort == +6);  // POSITIVE: the way is UNMEASURED; a flip is a measurement
    CHECK(six.cartridge == Cartridge::Red);
    REQUIRE(six.safeMode.has_value());
    CHECK(*six.safeMode == BrakeMode::Hold);
    REQUIRE(six.forward.has_value());
    REQUIRE(six.reverse.has_value());
    CHECK(*six.forward == ControllerButton::R1);
    CHECK(*six.reverse == ControllerButton::R2);
    CHECK(six.volts == 12.0);
    REQUIRE(six.provenance != nullptr);
    CHECK(std::strstr(six.provenance, "team lead 2026-09-26") != nullptr);

    const MotorMechanismEntry& seven = t.motors[1];
    CHECK(std::strcmp(seven.name, "claw pivot 7") == 0);
    CHECK(seven.signedPort == +7);
    CHECK(seven.cartridge == Cartridge::Red);
    REQUIRE(seven.safeMode.has_value());
    CHECK(*seven.safeMode == BrakeMode::Hold);
    REQUIRE(seven.forward.has_value());
    REQUIRE(seven.reverse.has_value());
    CHECK(*seven.forward == ControllerButton::L1);
    CHECK(*seven.reverse == ControllerButton::L2);
    CHECK(seven.volts == 12.0);
    REQUIRE(seven.provenance != nullptr);
    CHECK(std::strstr(seven.provenance, "team lead 2026-09-26") != nullptr);

    // And the table as a whole passes every check the header's static_asserts make.
    CHECK(allChecksPass(t));
    CHECK(mechanismsDisjointFromChassis(t, robotTwoChassis()));
}

TEST_CASE("mechanism table: robot two's PNEUMATIC claw is EXACTLY what the team lead stated "
          "(2026-09-26) -- 3-wire A, CLOSED at start, toggled by A, safe CLOSED, the polarity "
          "the ONE constant") {
    // BUG CAUGHT: any typo in the claw's entry — the wrong 3-wire line (the toggle fires some
    // other valve, or nothing), the claw starting OPEN (it drops its preload the moment the
    // program starts), a safe position of OPEN (a library-forced safe state — COMP's end-of-run
    // guard — would drop what the claw holds), the wrong button (the driver presses A and nothing
    // happens, or another mechanism's button fires the claw), the polarity typed as a level instead
    // of read from kClawOpenCommand, the wrong name, or a missing provenance — and the polarity
    // flipped without a measurement (the claw then OPENS at program start and drops what it holds,
    // while HA-159 and Station J.9 still say `true`).
    const MechanismTable& t = kRobotTwoMechanisms;
    // PINNED LIKE THE PIVOTS' SIGNS: the value is a guess, and a flip is Station J.9's MEASUREMENT
    // (J.9.3's verdict, A4 register HA-159). The code edit stays one line — the entry reads the
    // constant and every level follows it (the next case) — but the flip is RECORDED: it changes
    // this pin, HA-159 (its index row and its claim) and J.9's quote, in the same change.
    CHECK(kClawOpenCommand == true);  // "line ON = open": UNVERIFIED until Station J.9
    REQUIRE(t.pneumaticCount == 1);

    const PneumaticMechanismEntry& claw = t.pneumatics[0];
    REQUIRE(claw.name != nullptr);
    CHECK(std::strcmp(claw.name, "claw") == 0);
    CHECK(claw.adiLine == 1);  // 3-wire port A
    REQUIRE(claw.openCommand.has_value());
    CHECK(*claw.openCommand == kClawOpenCommand);
    REQUIRE(claw.start.has_value());
    CHECK(*claw.start == PneumaticPosition::Closed);
    REQUIRE(claw.safe.has_value());
    CHECK(*claw.safe == PneumaticPosition::Closed);
    REQUIRE(claw.toggle.has_value());
    CHECK(*claw.toggle == ControllerButton::A);
    REQUIRE(claw.provenance != nullptr);
    CHECK(std::strlen(claw.provenance) > 0);
    CHECK(std::strstr(claw.provenance, "team lead 2026-09-26 (port A; polarity unknown)")
          != nullptr);

    // The LEVELS: CLOSED is the level that does not open it, at start and as the safe value.
    CHECK(pneumaticCommandFor(claw, PneumaticPosition::Open) == kClawOpenCommand);
    CHECK(pneumaticCommandFor(claw, PneumaticPosition::Closed) == !kClawOpenCommand);
    CHECK(pneumaticStartCommand(claw) == !kClawOpenCommand);
    CHECK(pneumaticSafeCommand(claw) == !kClawOpenCommand);

    // The claw's line is a 3-WIRE port: it does not make smart port 1 a mechanism (Calibrate
    // would then drop a motor on port 1 for a claw that is not there).
    CHECK_FALSE(isDeclaredMechanismPort(t, 1));
    CHECK(allChecksPass(t));
}

TEST_CASE("mechanism table: the claw's polarity is ONE constant -- a flip moves the start AND safe "
          "levels together and never a position, and the entry reads the constant, not a literal") {
    // BUG CAUGHT: a level helper that ignores the polarity (the claw would open at boot whenever
    // the guess is wrong, and flipping the constant would not fix it), one that ignores the
    // entry's own position (a start of OPEN would still boot closed), or the polarity typed into
    // the entry as a literal — after which flipping kClawOpenCommand at Station J would change
    // nothing on the robot, with the suite green.
    for (const bool open : {true, false}) {
        CAPTURE(open);
        PneumaticMechanismEntry e = kRobotTwoMechanisms.pneumatics[0];
        e.openCommand = open;
        CHECK(pneumaticCommandFor(e, PneumaticPosition::Open) == open);
        CHECK(pneumaticCommandFor(e, PneumaticPosition::Closed) == !open);
        CHECK(pneumaticStartCommand(e) == !open);  // starts CLOSED, whichever level that is
        CHECK(pneumaticSafeCommand(e) == !open);
        CHECK(*e.start == PneumaticPosition::Closed);  // the POSITIONS do not move with it
        CHECK(*e.safe == PneumaticPosition::Closed);
        // Each helper reads its OWN position.
        e.start = PneumaticPosition::Open;
        CHECK(pneumaticStartCommand(e) == open);
        CHECK(pneumaticSafeCommand(e) == !open);
        e.start = PneumaticPosition::Closed;
        e.safe = PneumaticPosition::Open;
        CHECK(pneumaticStartCommand(e) == !open);
        CHECK(pneumaticSafeCommand(e) == open);
    }

    // The table's text, comments stripped: the constant is defined ONCE, and the ONE polarity in
    // the table is that constant — so flipping it is the whole fix, and a literal cannot hide.
    const std::string code = codeOnly(readRepoFile("src/mechanism_table.hpp"));
    CHECK(occurrences(code, "inline constexpr bool kClawOpenCommand = ") == 1);
    CHECK(occurrences(code, ".openCommand = ") == 1);
    CHECK(occurrences(code, ".openCommand = kClawOpenCommand,") == 1);
}

TEST_CASE("mechanism table: the HOST build's selection is 'none' -- only robot two declares "
          "mechanisms") {
    // BUG CAUGHT: the #if selecting robot two's table for a build that is not robot two (the bench
    // bot's Calibrate would then refuse its own ports 6 and 7 for claws it does not have). The tank
    // side of the selection is asserted by the header itself under the tank define, on every
    // `make ROBOT=tank` (the src build gate's four tank builds compile it).
    CHECK(&kMechanisms == &kNoMechanisms);
    CHECK(kMechanisms.motorCount == 0);
    CHECK(motorEntryCount(kMechanisms) == 0);
    CHECK(kMechanisms.pneumaticCount == 0);
    CHECK(pneumaticEntryCount(kMechanisms) == 0);
    CHECK(allChecksPass(kNoMechanisms));
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

TEST_CASE("mechanism table: 'is this a declared mechanism port' -- 6 and 7 yes, every drive port "
          "and every other port no") {
    // BUG CAUGHT: a predicate that matches too much (Calibrate would skip a DRIVE motor and
    // calibrate a side one short), too little (it would gear, coast and pulse a claw), or matches
    // the port-less value 0. The sweep includes port 1: the pneumatic claw's 3-WIRE line 1 is not
    // smart port 1.
    const MechanismTable& t = kRobotTwoMechanisms;
    CHECK(isDeclaredMechanismPort(t, 6));
    CHECK(isDeclaredMechanismPort(t, 7));
    const ChassisTable drive = robotTwoChassis();
    for (int p = -3; p <= kMaxPort + 3; ++p) {
        CAPTURE(p);
        if (p == 6 || p == 7) continue;
        CHECK_FALSE(isDeclaredMechanismPort(t, p));
    }
    // Every drive port of robot two, explicitly, both sides.
    for (int p : {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}) {
        CAPTURE(p);
        CHECK(tableSideOf(drive, p) != 0);
        CHECK_FALSE(isDeclaredMechanismPort(t, p));
    }
    // A FLIPPED pivot (the fix Station J may call for) is still its physical port: the match is on
    // |entry|, so flipping a sign can never hand a claw back to Calibrate.
    MechanismTable flipped = robotTwo();
    flipped.motors[0].signedPort = -6;
    flipped.motors[1].signedPort = -7;
    CHECK(isDeclaredMechanismPort(flipped, 6));
    CHECK(isDeclaredMechanismPort(flipped, 7));
    CHECK_FALSE(isDeclaredMechanismPort(flipped, -6));  // a PHYSICAL port is never negative
    // An entry left at port 0 is not "port 0 is a mechanism": 0 is never a port.
    MechanismTable zero = robotTwo();
    zero.motors[1].signedPort = 0;
    CHECK_FALSE(isDeclaredMechanismPort(zero, 0));
    // Entries past `count` are not declared, whatever they hold.
    MechanismTable one = robotTwo();
    one.motorCount = 1;
    CHECK(isDeclaredMechanismPort(one, 6));
    CHECK_FALSE(isDeclaredMechanismPort(one, 7));
}

TEST_CASE("mechanism table: the predicate's 1..21 guard at its EDGES -- a mechanism on port 21 "
          "or port 1 is declared and dropped; 0 and 22 are never ports") {
    // BUG CAUGHT: an off-by-one in the guard (`port >= 21`, `port < 2`). Robot two's claws are on
    // 6 and 7, where no such slip shows; a mechanism added later on port 21 (free on robot two) or
    // port 1 would then NOT be dropped, and Calibrate would gear, coast and pulse it with the
    // suite green. The table's own port check accepts 1 and 21 (the ports case below), so nothing
    // else would stop it.
    MechanismTable edges = robotTwo();
    edges.motors[0].signedPort = 21;
    edges.motors[1].signedPort = -1;  // flipped, on physical port 1
    REQUIRE(allChecksPass(edges));
    CHECK(isDeclaredMechanismPort(edges, 21));
    CHECK(isDeclaredMechanismPort(edges, 1));
    for (int p : {0, 2, 6, 7, 20, 22}) {
        CAPTURE(p);
        CHECK_FALSE(isDeclaredMechanismPort(edges, p));
    }
    const Filtered f = filter(edges, {1, 2, 11, 20, 21});
    CHECK(f.keptPorts() == std::vector<int>{2, 11, 20});
    CHECK(f.dropped == std::vector<int>{1, 21});
    CHECK(f.tailCleared());

    // An entry OUTSIDE 1..21 (a table the static_asserts refuse) never makes that number a port.
    for (int bad : {22, -22, 0}) {
        CAPTURE(bad);
        MechanismTable beyond = robotTwo();
        beyond.motors[0].signedPort = static_cast<std::int8_t>(bad);
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
    const Filtered f = filter(kRobotTwoMechanisms, {6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(f.kept == 9);
    CHECK(f.array == std::vector<int>{11, 12, 13, 14, 15, 16, 17, 18, 20, 0, 0});  // every slot
    CHECK(f.dropped == std::vector<int>{6, 7});  // and NOT the 99s the list was pre-filled with

    // Order is the CENSUS's, not sorted: an interleaved list comes back in its own order, the
    // claws in the middle and at the end.
    const Filtered mixed = filter(kRobotTwoMechanisms, {20, 7, 11, 6, 3});
    CHECK(mixed.kept == 3);
    CHECK(mixed.array == std::vector<int>{20, 11, 3, 0, 0});
    CHECK(mixed.dropped == std::vector<int>{7, 6});
    const Filtered last = filter(kRobotTwoMechanisms, {11, 12, 6});
    CHECK(last.array == std::vector<int>{11, 12, 0});
    CHECK(last.dropped == std::vector<int>{6});

    // The claws UNPLUGGED (today's captures): nothing is dropped, nothing moves, nothing is said.
    const Filtered today = filter(kRobotTwoMechanisms, {11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(today.kept == 9);
    CHECK(today.array == std::vector<int>{11, 12, 13, 14, 15, 16, 17, 18, 20});
    CHECK(today.dropped.empty());

    // Only the claws plugged in: both dropped, no candidate left (Calibrate then refuses on its
    // own "at least two" check, which now counts motors besides the declared mechanisms).
    const Filtered onlyClaws = filter(kRobotTwoMechanisms, {6, 7});
    CHECK(onlyClaws.kept == 0);
    CHECK(onlyClaws.array == std::vector<int>{0, 0});
    CHECK(onlyClaws.dropped == std::vector<int>{6, 7});

    // An empty census: nothing kept, and the garbage-filled dropped list comes back EMPTY.
    const Filtered none = filter(kRobotTwoMechanisms, {});
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
    const Filtered full = filter(kRobotTwoMechanisms, all);
    CHECK(full.kept == 19);
    CHECK(full.keptPorts() == expect);
    CHECK(full.tailCleared());
    CHECK(full.dropped == std::vector<int>{6, 7});

    // More declared ports than the dropped list holds (impossible from a census, which lists each
    // port once): every one still LEAVES the candidates; only the log would miss the extras.
    const Filtered flood = filter(kRobotTwoMechanisms, std::vector<int>(25, 6));
    CHECK(flood.kept == 0);
    CHECK(flood.tailCleared());
    CHECK(flood.dropped.size() == static_cast<std::size_t>(kMaxPort));
}

TEST_CASE("mechanism table: the claw ports are not robot two's drive ports, nor its IMU port -- "
          "and the check sees a collision on either side, by |port|") {
    // BUG CAUGHT: a claw typed onto a drive port. The binding would then drive a wheel on R1, the
    // chassis would command the claw, and Calibrate would drop a drive motor. The header
    // static_asserts this for every tank build; here the predicate itself is broken on purpose.
    const ChassisTable drive = robotTwoChassis();
    CHECK(mechanismsDisjointFromChassis(kRobotTwoMechanisms, drive));
    for (std::size_t i = 0; i < kRobotTwoMechanisms.motorCount; ++i) {
        CAPTURE(i);
        CHECK(tableSideOf(drive, absPort(kRobotTwoMechanisms.motors[i].signedPort)) == 0);
    }

    MechanismTable onLeft = robotTwo();
    onLeft.motors[0].signedPort = 11;  // the left side's -11, typed positive
    CHECK_FALSE(mechanismsDisjointFromChassis(onLeft, drive));
    MechanismTable onRight = robotTwo();
    onRight.motors[1].signedPort = -16;  // the right side's +16, typed negative
    CHECK_FALSE(mechanismsDisjointFromChassis(onRight, drive));
    MechanismTable onImu = robotTwo();
    onImu.motors[1].signedPort = 2;  // the IMU's port
    CHECK_FALSE(mechanismsDisjointFromChassis(onImu, drive));
    // With no IMU stated (imuPort 0 = UNSET), no claw port collides with "the IMU".
    ChassisTable noImu = drive;
    noImu.imuPort = 0;
    CHECK(mechanismsDisjointFromChassis(kRobotTwoMechanisms, noImu));
    // Nothing declared is disjoint from anything.
    CHECK(mechanismsDisjointFromChassis(kNoMechanisms, drive));
}

TEST_CASE("mechanism table: an UNSET field anywhere makes a declared entry incomplete -- a value "
          "is never invented") {
    // BUG CAUGHT: a table entry typed without one of its facts. The field would silently take its
    // value-initialised value: Coast for the safe mode (a released claw DROPS), L1 for a button
    // (another pivot's button, and the library loop's default re-zero button), 0 V, port 0.
    CHECK(motorEntryComplete(kRobotTwoMechanisms.motors[0]));
    CHECK(motorEntryComplete(kRobotTwoMechanisms.motors[1]));

    const auto brokenBy = [](auto&& breakIt) {
        MechanismTable t = robotTwo();
        breakIt(t.motors[1]);
        return mechanismTableComplete(t);
    };
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.name = nullptr; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.name = ""; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.provenance = nullptr; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.provenance = ""; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.signedPort = 0; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.cartridge = Cartridge::Unset; }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.safeMode.reset(); }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.forward.reset(); }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.reverse.reset(); }));
    CHECK_FALSE(brokenBy([](MotorMechanismEntry& e) { e.volts = 0.0; }));

    // THE REASON FOR std::optional, pinned: a field LEFT OUT is value-initialised — the same thing
    // a designated initializer does to a member it does not name — and for these three that is
    // "unset", refused, not Coast / L1 / L1. (Built by assignment rather than with designators so
    // that clang's -Wmissing-designated-field-initializers, an error under -Werror, stays quiet.)
    MotorMechanismEntry forgotten{};
    forgotten.name = "forgot three fields";
    forgotten.signedPort = 9;
    forgotten.cartridge = Cartridge::Red;
    forgotten.volts = 6.0;
    forgotten.provenance = "test";
    CHECK_FALSE(forgotten.safeMode.has_value());
    CHECK_FALSE(forgotten.forward.has_value());
    CHECK_FALSE(forgotten.reverse.has_value());
    CHECK_FALSE(motorEntryComplete(forgotten));

    // Only the DECLARED entries are judged: a garbage entry past `motorCount` is not.
    MechanismTable one = robotTwo();
    one.motorCount = 1;
    one.motors[1] = MotorMechanismEntry{};
    CHECK(mechanismTableComplete(one));
}

TEST_CASE("mechanism table: an UNSET field anywhere makes a declared PNEUMATIC entry incomplete -- "
          "no invented line, polarity, position or button") {
    // BUG CAUGHT: a claw entry typed without one of its facts, which would silently take its
    // value-initialised value: line 0 (no port), polarity `false` (an invented guess — the one fact
    // that is NOT known), start or safe position Closed (plausible, and still invented), toggle L1
    // (port 7's forward button, and the library loop's default re-zero button). Also: a
    // completeness check that forgot to look at the pneumatic entries at all.
    CHECK(pneumaticEntryComplete(kRobotTwoMechanisms.pneumatics[0]));
    CHECK(mechanismTableComplete(kRobotTwoMechanisms));

    const auto brokenBy = [](auto&& breakIt) {
        MechanismTable t = robotTwo();
        breakIt(t.pneumatics[0]);
        return mechanismTableComplete(t);
    };
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.name = nullptr; }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.name = ""; }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.provenance = nullptr; }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.provenance = ""; }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.adiLine = 0; }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.openCommand.reset(); }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.start.reset(); }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.safe.reset(); }));
    CHECK_FALSE(brokenBy([](PneumaticMechanismEntry& e) { e.toggle.reset(); }));

    // THE REASON FOR std::optional, pinned for this kind too: the four fields LEFT OUT come back
    // "unset", refused — not false / Closed / Closed / L1. (By assignment, as above.)
    PneumaticMechanismEntry forgotten{};
    forgotten.name = "forgot four fields";
    forgotten.adiLine = 2;
    forgotten.provenance = "test";
    CHECK_FALSE(forgotten.openCommand.has_value());
    CHECK_FALSE(forgotten.start.has_value());
    CHECK_FALSE(forgotten.safe.has_value());
    CHECK_FALSE(forgotten.toggle.has_value());
    CHECK_FALSE(pneumaticEntryComplete(forgotten));

    // A second pneumatic entry with one field unset is refused too (the loop reads every entry,
    // not only the first)...
    MechanismTable second = robotTwoWithSecondPneumatic(2, ControllerButton::B);
    REQUIRE(mechanismTableComplete(second));
    second.pneumatics[1].safe.reset();
    CHECK_FALSE(mechanismTableComplete(second));
    // ...and a garbage entry past `pneumaticCount` is not judged.
    MechanismTable none = robotTwo();
    none.pneumaticCount = 0;
    none.pneumatics[0] = PneumaticMechanismEntry{};
    CHECK(mechanismTableComplete(none));
}

TEST_CASE("mechanism table: ports must be real and declared once -- 0, 22, -22 and a repeat "
          "(+6 / -6) are refused; 1 and 21 are fine") {
    // BUG CAUGHT: a port beyond the brain's 21, the "no port" value, or one motor declared twice
    // (two bindings fighting over one motor, or — with a sign — one declared both ways).
    CHECK(mechanismPortsValid(kRobotTwoMechanisms));
    for (int bad : {0, 22, -22, 100, -100}) {
        CAPTURE(bad);
        MechanismTable t = robotTwo();
        t.motors[0].signedPort = static_cast<std::int8_t>(bad);
        CHECK_FALSE(mechanismPortsValid(t));
    }
    for (int edge : {1, 21, -21, -1}) {
        CAPTURE(edge);
        MechanismTable t = robotTwo();
        t.motors[0].signedPort = static_cast<std::int8_t>(edge);
        CHECK(mechanismPortsValid(t));
    }
    MechanismTable twice = robotTwo();
    twice.motors[1].signedPort = 6;
    CHECK_FALSE(mechanismPortsValid(twice));
    MechanismTable twiceSigned = robotTwo();
    twiceSigned.motors[1].signedPort = -6;
    CHECK_FALSE(mechanismPortsValid(twiceSigned));
}

TEST_CASE("mechanism table: 3-wire lines must be real and declared once -- 0, 9 and 255 are "
          "refused; 1 ('A') and 8 ('H') are fine; two entries on one line are refused") {
    // BUG CAUGHT: the claw typed on "line 9" (the brain has eight 3-wire ports, A..H), the
    // "no line" value, an off-by-one at either end, or two pneumatics on one line (two toggles
    // fighting over one valve).
    CHECK(kMaxAdiLine == 8);
    CHECK(pneumaticLinesValid(kRobotTwoMechanisms));
    for (int bad : {0, 9, 10, 22, 255}) {
        CAPTURE(bad);
        MechanismTable t = robotTwo();
        t.pneumatics[0].adiLine = static_cast<std::uint8_t>(bad);
        CHECK_FALSE(pneumaticLinesValid(t));
    }
    for (int edge : {1, 2, 7, 8}) {
        CAPTURE(edge);
        MechanismTable t = robotTwo();
        t.pneumatics[0].adiLine = static_cast<std::uint8_t>(edge);
        CHECK(pneumaticLinesValid(t));
    }
    // Two entries: on one line refused, on two lines fine — and the SECOND entry's range is read
    // too (not only the first's).
    CHECK_FALSE(pneumaticLinesValid(robotTwoWithSecondPneumatic(1, ControllerButton::B)));
    CHECK(pneumaticLinesValid(robotTwoWithSecondPneumatic(2, ControllerButton::B)));
    CHECK(allChecksPass(robotTwoWithSecondPneumatic(8, ControllerButton::B)));
    CHECK_FALSE(pneumaticLinesValid(robotTwoWithSecondPneumatic(9, ControllerButton::B)));
    CHECK_FALSE(pneumaticLinesValid(robotTwoWithSecondPneumatic(0, ControllerButton::B)));
    // A line past `pneumaticCount` is not judged.
    MechanismTable none = robotTwo();
    none.pneumaticCount = 0;
    none.pneumatics[0].adiLine = 9;
    CHECK(pneumaticLinesValid(none));

    // A 3-wire LINE is not a SMART port, whatever its number: the claw on line 6, 7 or 2 does not
    // collide with the pivots on smart ports 6 and 7, nor with the IMU on smart port 2; and
    // wherever the claw's line is, Calibrate's predicate and filter see exactly the pivots.
    const ChassisTable drive = robotTwoChassis();
    for (int line = 1; line <= kMaxAdiLine; ++line) {
        CAPTURE(line);
        MechanismTable t = robotTwo();
        t.pneumatics[0].adiLine = static_cast<std::uint8_t>(line);
        CHECK(allChecksPass(t));
        CHECK(mechanismsDisjointFromChassis(t, drive));
        for (int p = 1; p <= kMaxPort; ++p) {
            CAPTURE(p);
            CHECK(isDeclaredMechanismPort(t, p) == (p == 6 || p == 7));
        }
    }
}

TEST_CASE("mechanism table: the buttons -- forward != reverse, one button one pivot, and never "
          "shulib Teleop's re-zero button (Y)") {
    // BUG CAUGHT: port 7 typed on L1/R2 (R2 would drive both claws), one button meaning both ways,
    // or a claw bound to the button that re-zeroes field-centric driving (every claw press would
    // also re-zero the heading).
    CHECK(mechanismForwardReverseDistinct(kRobotTwoMechanisms));
    CHECK(mechanismButtonsUnique(kRobotTwoMechanisms));
    CHECK(mechanismButtonsAvoid(kRobotTwoMechanisms, kTeleopRezeroButton));
    // Y: the team lead's ruling, 2026-09-26 (T3 addendum §0) — A became the claw's toggle.
    CHECK(kTeleopRezeroButton == ControllerButton::Y);

    // forward == reverse on one entry.
    MechanismTable same = robotTwo();
    same.motors[0].reverse = ControllerButton::R1;
    CHECK_FALSE(mechanismForwardReverseDistinct(same));

    // THE BRIEF'S MUTATION, as a table: port 7 on L1 / R2 — R2 is port 6's reverse.
    MechanismTable l1r2 = robotTwo();
    l1r2.motors[1].reverse = ControllerButton::R2;
    CHECK(mechanismForwardReverseDistinct(l1r2));  // each entry alone is fine...
    CHECK_FALSE(mechanismButtonsUnique(l1r2));     // ...the table is not
    // Every pairing across two entries: forward/forward, forward/reverse, reverse/forward,
    // reverse/reverse.
    MechanismTable ff = robotTwo();
    ff.motors[1].forward = ControllerButton::R1;
    CHECK_FALSE(mechanismButtonsUnique(ff));
    MechanismTable fr = robotTwo();
    fr.motors[1].forward = ControllerButton::R2;
    CHECK_FALSE(mechanismButtonsUnique(fr));
    MechanismTable rf = robotTwo();
    rf.motors[1].reverse = ControllerButton::R1;
    CHECK_FALSE(mechanismButtonsUnique(rf));
    // Swapping the two pivots' buttons wholesale is still a UNIQUE table — which is exactly why the
    // contents case above pins each entry's buttons by name: no consistency check can catch it.
    MechanismTable crossed = robotTwo();
    crossed.motors[0].forward = ControllerButton::L1;
    crossed.motors[0].reverse = ControllerButton::L2;
    crossed.motors[1].forward = ControllerButton::R1;
    crossed.motors[1].reverse = ControllerButton::R2;
    CHECK(allChecksPass(crossed));

    // THE ADDENDUM'S MUTATION: a pivot on Y, the reserved button — forward or reverse. Each such
    // table is still UNIQUE (nothing else binds Y): only the reserved-button check catches it.
    MechanismTable yFwd = robotTwo();
    yFwd.motors[0].forward = ControllerButton::Y;
    CHECK(mechanismButtonsUnique(yFwd));
    CHECK_FALSE(mechanismButtonsAvoid(yFwd, kTeleopRezeroButton));
    MechanismTable yRev = robotTwo();
    yRev.motors[1].reverse = ControllerButton::Y;
    CHECK(mechanismButtonsUnique(yRev));
    CHECK_FALSE(mechanismButtonsAvoid(yRev, kTeleopRezeroButton));
    // The check is about the button it is GIVEN: robot two's table binds L1, which is the library
    // loop's DEFAULT re-zero button — a Teleop that kept the default would collide.
    CHECK_FALSE(mechanismButtonsAvoid(kRobotTwoMechanisms, ControllerButton::L1));

    // Unset buttons collide with nothing here (completeness refuses them — one check, one fault).
    MechanismTable unset = robotTwo();
    unset.motors[0].forward.reset();
    unset.motors[0].reverse.reset();
    unset.motors[1].forward.reset();
    CHECK(mechanismForwardReverseDistinct(unset));
    CHECK(mechanismButtonsUnique(unset));
    CHECK_FALSE(mechanismTableComplete(unset));
}

TEST_CASE("mechanism table: the buttons across the pivots AND the claw -- the claw's A is its own, "
          "it takes no pivot's button, no pivot takes A, and nothing takes Y") {
    // BUG CAUGHT: the claw typed on a pivot's button (one press would open the claw AND swing a
    // pivot), a pivot typed on A (the same collision the other way round), two toggles on one
    // button, the claw on the re-zero button Y — or a uniqueness / reserved-button check that
    // never looks at the pneumatic entries at all.
    //
    // THE ADDENDUM'S MUTATION: the claw on R1 (port 6's forward) — and on each other pivot button.
    for (const ControllerButton b : {ControllerButton::R1, ControllerButton::R2,
                                     ControllerButton::L1, ControllerButton::L2}) {
        CAPTURE(static_cast<int>(b));
        MechanismTable t = robotTwo();
        t.pneumatics[0].toggle = b;
        CHECK_FALSE(mechanismButtonsUnique(t));
        CHECK(mechanismButtonsAvoid(t, kTeleopRezeroButton));  // uniqueness is what catches it
        CHECK(mechanismForwardReverseDistinct(t));
    }
    // A pivot on A, as its forward and as its reverse: it collides with the claw.
    MechanismTable pivotFwdA = robotTwo();
    pivotFwdA.motors[0].forward = ControllerButton::A;
    CHECK_FALSE(mechanismButtonsUnique(pivotFwdA));
    CHECK(mechanismButtonsAvoid(pivotFwdA, kTeleopRezeroButton));  // A is not reserved any more
    MechanismTable pivotRevA = robotTwo();
    pivotRevA.motors[1].reverse = ControllerButton::A;
    CHECK_FALSE(mechanismButtonsUnique(pivotRevA));
    // Two toggles: on one button refused; on two buttons fine. The SECOND toggle against the
    // pivots is read too (not only the first).
    CHECK_FALSE(mechanismButtonsUnique(robotTwoWithSecondPneumatic(2, ControllerButton::A)));
    CHECK(mechanismButtonsUnique(robotTwoWithSecondPneumatic(2, ControllerButton::B)));
    CHECK_FALSE(mechanismButtonsUnique(robotTwoWithSecondPneumatic(2, ControllerButton::L2)));
    // The claw on Y, the re-zero button: unique, and refused by the reserved-button check; a second
    // pneumatic on Y the same.
    MechanismTable clawOnY = robotTwo();
    clawOnY.pneumatics[0].toggle = ControllerButton::Y;
    CHECK(mechanismButtonsUnique(clawOnY));
    CHECK_FALSE(mechanismButtonsAvoid(clawOnY, kTeleopRezeroButton));
    const MechanismTable secondOnY = robotTwoWithSecondPneumatic(2, ControllerButton::Y);
    CHECK(mechanismButtonsUnique(secondOnY));
    CHECK_FALSE(mechanismButtonsAvoid(secondOnY, kTeleopRezeroButton));
    // WHY THE RE-ZERO MOVED: robot two's table binds A now, so a Teleop that kept its old re-zero
    // button (A) would collide — the coordinator's merge step (header, kTeleopRezeroButton).
    CHECK_FALSE(mechanismButtonsAvoid(kRobotTwoMechanisms, ControllerButton::A));
    // An unset toggle collides with nothing here (completeness refuses it — one check, one fault).
    MechanismTable unset = robotTwo();
    unset.pneumatics[0].toggle.reset();
    CHECK(mechanismButtonsUnique(unset));
    CHECK(mechanismButtonsAvoid(unset, kTeleopRezeroButton));
    CHECK_FALSE(mechanismTableComplete(unset));
    // A toggle past `pneumaticCount` is not judged, whatever it binds.
    MechanismTable past = robotTwo();
    past.pneumatics[1] = past.pneumatics[0];
    past.pneumatics[1].toggle = ControllerButton::R1;
    CHECK(mechanismButtonsUnique(past));
}

TEST_CASE("mechanism table: volts must be in (0, 12] -- 0, negative, 12.01, NaN and infinity are "
          "refused; 12 exactly and a small positive value are fine") {
    // BUG CAUGHT: a sign typed into the voltage (the direction lives in the port's sign and the
    // buttons, never here), a value the motor clamps anyway, or a NaN that a naive `v <= 0 || v >
    // 12` would wave through.
    CHECK(mechanismVoltsValid(kRobotTwoMechanisms));
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    for (double bad : {0.0, -0.0, -12.0, -1.0, 12.01, 13.0, kNaN, kInf, -kInf}) {
        CAPTURE(bad);
        MechanismTable t = robotTwo();
        t.motors[1].volts = bad;
        CHECK_FALSE(mechanismVoltsValid(t));
    }
    for (double good : {12.0, 0.5, 6.0}) {
        CAPTURE(good);
        MechanismTable t = robotTwo();
        t.motors[1].volts = good;
        CHECK(mechanismVoltsValid(t));
    }
}

TEST_CASE("mechanism table: a count past the array is refused, and nothing reads past the array") {
    // BUG CAUGHT: a hand-typed count larger than its entries array (either kind); every loop would
    // read past its end. The count check refuses it, and every other check reads at most the
    // array's length.
    MechanismTable t = robotTwo();
    t.motorCount = kMaxMechanisms + 1;
    CHECK_FALSE(mechanismCountValid(t));
    CHECK(motorEntryCount(t) == kMaxMechanisms);
    t.motorCount = kMaxMechanisms;
    CHECK(mechanismCountValid(t));
    MechanismTable p = robotTwo();
    p.pneumaticCount = kMaxPneumatics + 1;
    CHECK_FALSE(mechanismCountValid(p));
    CHECK(pneumaticEntryCount(p) == kMaxPneumatics);
    p.pneumaticCount = kMaxPneumatics;
    CHECK(mechanismCountValid(p));
    CHECK(kMaxPneumatics == static_cast<std::size_t>(kMaxAdiLine));  // one per 3-wire port
    // A full table of six can never pass: six entries need twelve distinct buttons, and one of the
    // twelve is reserved — the header's arithmetic for kMaxMechanisms.
    MechanismTable six{};
    six.motorCount = kMaxMechanisms;
    const ControllerButton buttons[12] = {
        ControllerButton::L1,   ControllerButton::L2,    ControllerButton::R1,
        ControllerButton::R2,   ControllerButton::Up,    ControllerButton::Down,
        ControllerButton::Left, ControllerButton::Right, ControllerButton::X,
        ControllerButton::B,    ControllerButton::Y,     ControllerButton::A};
    for (std::size_t i = 0; i < kMaxMechanisms; ++i) {
        six.motors[i] = MotorMechanismEntry{
            .name = "pivot",
            .signedPort = static_cast<std::int8_t>(1 + i),
            .cartridge = Cartridge::Red,
            .safeMode = BrakeMode::Hold,
            .forward = buttons[2 * i],
            .reverse = buttons[2 * i + 1],
            .volts = 12.0,
            .provenance = "test",
        };
    }
    CHECK(mechanismButtonsUnique(six));
    CHECK_FALSE(mechanismButtonsAvoid(six, kTeleopRezeroButton));
    CHECK_FALSE(allChecksPass(six));
    // Five, without the reserved button, is the most that passes.
    MechanismTable five = six;
    five.motorCount = 5;
    CHECK(allChecksPass(five));
    // With pneumatics the arithmetic is 2 x motors + pneumatics <= 11: five motor entries leave
    // ONE button (A) for one pneumatic, and a second pneumatic has none left — Y is reserved and
    // every other button is taken.
    MechanismTable fivePlusClaw = five;
    fivePlusClaw.pneumatics[0] = kRobotTwoMechanisms.pneumatics[0];  // on A
    fivePlusClaw.pneumaticCount = 1;
    CHECK(allChecksPass(fivePlusClaw));
    for (const ControllerButton b : buttons) {
        CAPTURE(static_cast<int>(b));
        MechanismTable fivePlusTwo = fivePlusClaw;
        fivePlusTwo.pneumatics[1] = fivePlusClaw.pneumatics[0];
        fivePlusTwo.pneumatics[1].adiLine = 2;
        fivePlusTwo.pneumatics[1].toggle = b;
        fivePlusTwo.pneumaticCount = 2;
        CHECK_FALSE(allChecksPass(fivePlusTwo));
    }
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
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/src/calibrate_program.cpp"};
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

    // The program includes the table it filters by.
    CHECK(code.find("#include \"mechanism_table.hpp\"") != std::string::npos);
}
