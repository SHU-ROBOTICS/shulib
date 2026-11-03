// calibrate_program_typed_test.cpp — the standard Calibrate's TYPED REFERENCE (chunk CAL1 Part A2b,
// 2026-10-30: the team lead's D6 (a) — with no usable record on a present card, a team's TYPED drive
// ports are the front check's reference), RUN WHOLE on the A2 host test plant through the library's
// fakes (test/calibrate_program_rig.hpp). The CAL1 brief's §2.A tests, A2b's share:
//   T7  test 2's TYPED half — agreeing exactly (FRONT MATCHES YOUR TYPED DRIVE PORTS), agreeing in
//       front but not in every port (the per-port lines; FRONT MATCHES, SOME TYPED PORTS DIFFER),
//       flipped and HELD in the typed ports' own words (USE NEW, then FIX YOUR TYPED DRIVE PORTS; RUN
//       AGAIN; a field disable — "the card is unchanged (it holds no usable record)", never "keeps its
//       record"), a half-flipped tie and no port in common (CANNOT TELL), a record on the card AND
//       typed ports flipped against it (the run EQUALS the untyped run: the typed ports change
//       nothing), files that do not decode (both, the backup alone, an EMPTY record file: each file's
//       own line, then the typed ports decide), no card (no check, typed or not: the run EQUALS the
//       untyped no-card run), a declared drive port the typed ports drive with (the MECHANISM PORT
//       TURNED evidence in their words), and FRONT FLIPPED against them followed by a MECHANISM PORT
//       TURNED question (the NOTE's order: PORT N KEPT over FIX YOUR TYPED DRIVE PORTS).
//       CAL1-A2b's review (2026-10-30) added the edges it found unpinned: typed lists only SHORT a
//       pushed port (T7b-3: still partial), typed ports 1 and 21 named (T7b-4), a record recovered
//       from the BACKUP beating the typed ports (T7g-2), a typed RIGHT drive port declared (T7j-2),
//       T7k's typed hold words and screen with a question to come, and the typed NOTEs kept under a
//       NOT SAVED header (T7l, ruling 2).
//   T6g-T6i test 6's typed refusals — a typed port outside 1..21, twice on one side, once on each
//       side, only one side typed (the LEFT only, the RIGHT only) — each FACTS REFUSED naming
//       typedDrive.left[i] / .right[i] and its value before ANY device is asked for; a mechanism
//       problem reported first; zero slots skipped, the order kept (and the LAST slot alone typed —
//       T6h-2); and factsProblem(), hasTypedDrive(), typedDriveDescription() and
//       typedDrivePortsFrom() at COMPILE time (the last slot alone, and a duplicate in adjacent
//       slots, since the review).
// The typed ports here are the CAL1 brief's probe's (its case 8): robot two's 2026 typed table as it
// was typed on 2026-10-08 — LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16, the end the plant
// drives forward — written literally, never read from src/ (robot two's table is G3a's to flip).
//
// Every expected string is written in the test — read off the brief's probe goldens
// (brief-CAL1 d6a/v3/run_new/), never computed by the code under test. Nothing here has run on a
// robot.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "calibrate_program_rig.hpp"
#include "doctest.h"
#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/fake/fake_file_store.hpp"

namespace {

using namespace cal_prog_rig;
using shulib::calibration::CalibrateFacts;
using shulib::calibration::kRecordBackupFileName;
using shulib::calibration::kRecordFileName;
using shulib::hal::DrivetrainDescription;
using shulib::hal::fake::FakeFileFault;
using shulib::hal::fake::FakeFileFaultPlan;
using shulib::hal::fake::FakeFileOp;
using shulib::hal::fake::FakeFileStore;

constexpr std::size_t npos = std::string::npos;

const char* const kFrontLedSides = "LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20";
const char* const kOtherEndSides = "LEFT -16 +17 -18 +19 -20 | RIGHT +11 -12 +13 -14 +15";

const char* const kCardTyped =
    "card       : no usable record yet -- this run's FRONT is checked against your typed drive ports";
const char* const kNoRecordTyped =
    "front check: the card holds no record yet -- checking against your typed drive ports instead";
const char* const kUndecodedTyped =
    "front check: the card's record does not decode -- checking against your typed drive ports instead";
const char* const kTypedList = "typed drive ports: LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +16";
const char* const kTypedSame =
    "front check against your typed drive ports: 10 drive port(s) in both -- 10 agree, 0 flipped, 0 sign "
    "only, 0 side only; 0 in the typed ports only, 0 in this run's only: SAME FRONT";
const char* const kTypedFlipped =
    "front check against your typed drive ports: 10 drive port(s) in both -- 0 agree, 10 flipped, 0 sign "
    "only, 0 side only; 0 in the typed ports only, 0 in this run's only: FRONT FLIPPED";
const char* const kTypedFlip[5] = {
    "FRONT FLIPPED -- your typed drive ports call the end that LED this push its BACK.",
    "Either the BACK led by mistake (touch RUN AGAIN: nothing is saved), or the FRONT led",
    "and the typed ports are wrong, or the drive cables were swapped since they were typed",
    "(touch USE NEW: saved as pushed -- then fix the typed ports). The push cannot tell",
    "these apart. USE NEW reverses every auton tuned on the typed ports. NOTHING IS SAVED YET.",
};
const char* const kCannotTell =
    "front check: CANNOT TELL (no drive port in common, or no majority) -- saved as measured; check the "
    "robot drives the right way before an auton";
const char* const kUnchanged = "the card is unchanged (it holds no usable record)";

// The NOTE row, drawn (amber, the third SMALL row).
const char* const kDrawNoteSame = "print S 8 212 pen=e8b23a eraser=101418 'FRONT MATCHES YOUR TYPED DRIVE PORTS'";
const char* const kDrawNotePartial =
    "print S 8 212 pen=e8b23a eraser=101418 'FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log'";
const char* const kDrawNoteFix = "print S 8 212 pen=e8b23a eraser=101418 'FIX YOUR TYPED DRIVE PORTS -- see the log'";
const char* const kDrawNoteNotChecked = "print S 8 212 pen=e8b23a eraser=101418 'FRONT NOT CHECKED -- see the log'";

/// Robot two's facts with the probe's typed drive ports (case 8) — or `left`/`right` instead.
CalibrateFacts typedFacts(std::initializer_list<int> left = {-11, 12, -13, 14, -15},
                          std::initializer_list<int> right = {20, -19, 18, -17, 16}) {
    CalibrateFacts f = robotTwoFacts();
    f.typedDrive = {};
    std::size_t i = 0;
    for (const int p : left) {
        f.typedDrive.left[i++] = static_cast<std::int8_t>(p);
    }
    i = 0;
    for (const int p : right) {
        f.typedDrive.right[i++] = static_cast<std::int8_t>(p);
    }
    return f;
}

/// The record a clean, front-led run saves on an empty card (the seed of a "card holds a record"
/// case) — made by the program itself, REQUIRED to decode before use.
const std::vector<std::byte>& frontLedRecord() {
    static const std::vector<std::byte> bytes = [] {
        Scenario sc;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        return rig.fileBytes(kRecordFileName);
    }();
    REQUIRE(bytes.size() == shulib::calibration::kRecordBytes);
    DrivetrainDescription d{};
    REQUIRE(shulib::calibration::decodeRecord(std::span<const std::byte>{bytes.data(), bytes.size()}, d));
    return bytes;
}

std::vector<std::byte> garbage(std::size_t n) { return std::vector<std::byte>(n, std::byte{'x'}); }

bool has(const Rig& rig, const std::string& line) { return findLine(rig.io.lines, line) != npos; }

bool drew(const Rig& rig, const std::string& part) {
    for (const auto& l : rig.screen.log()) {
        if (l.find(part) != npos) {
            return true;
        }
    }
    return false;
}

std::string measured(const Rig& rig, int nth = 0) {
    int k = 0;
    for (const auto& l : rig.io.lines) {
        if (l.rfind("MEASURED ", 0) == 0 && k++ == nth) {
            return l;
        }
    }
    return "";
}

DrivetrainDescription decoded(const std::vector<std::byte>& b) {
    DrivetrainDescription d{};
    REQUIRE(shulib::calibration::decodeRecord(std::span<const std::byte>{b.data(), b.size()}, d));
    return d;
}

std::string describe(const DrivetrainDescription& d) {
    char out[128];
    d.describe(out, sizeof out);
    return out;
}

void checkAllStopped(Rig& rig) {
    for (int p = 11; p <= 20; ++p) {
        CAPTURE(p);
        CHECK(rig.rec(p).commandedVoltage().value() == 0.0);
        CHECK(rig.rec(p).brakeMode() == BrakeMode::Coast);
    }
}

/// The saved record is the `nth` run's measurement (as it printed it) on BOTH files, with `sides`.
void checkSaved(const Rig& rig, int nth, const char* sides) {
    const auto bin = rig.fileBytes(kRecordFileName);
    REQUIRE(bin.size() == shulib::calibration::kRecordBytes);
    const DrivetrainDescription d = decoded(bin);
    CHECK(describe(d) == measured(rig, nth));
    CHECK(sidesOf(d) == sides);
    CHECK(rig.fileBytes(kRecordBackupFileName) == bin);
}

/// The three typed check lines, consecutive, starting at the first of them (REQUIRED present).
std::size_t checkTypedLines(const Rig& rig, const char* opening, const char* list, const char* verdict) {
    const auto& L = rig.io.lines;
    const std::size_t at = findLine(L, opening);
    REQUIRE(at != npos);
    REQUIRE(at + 2 < L.size());
    CHECK(L[at + 1] == list);
    CHECK(L[at + 2] == verdict);
    return at;
}

/// A refusal before ANYTHING: no device asked for, nothing commanded, nothing written, the
/// refusal's two lines the run's last, its headline painted (calibrate_program_test.cpp's helper).
void checkRefusedBeforeAnything(Rig& rig, const std::string& headline, const std::string& why,
                                const std::string& fix) {
    REQUIRE_FALSE(rig.timedOut);
    CHECK(rig.counting.requests == 0);
    for (int p = 1; p <= 21; ++p) {
        if (rig.hasMotor(p)) {
            CAPTURE(p);
            CHECK(rig.rec(p).voltageCalls == 0);
            CHECK(rig.rec(p).brakeCalls == 0);
        }
    }
    CHECK(rig.replacesFrom(0) == 0);
    const std::size_t r = findLine(rig.io.lines, "REFUSED: " + headline + " -- " + why);
    REQUIRE(r != npos);
    CHECK(rig.io.lines[r + 1] == "  " + fix);
    CHECK(r + 2 == rig.io.lines.size());
    CHECK(drew(rig, "print L 8 34 pen=e2564a eraser=101418 '" + headline + "'"));
}

/// The index of the `nth` operation of kind `op` on `file` in a card's log, or npos
/// (calibrate_program_test.cpp's helper).
std::size_t opIndex(const FakeFileStore& card, FakeFileOp op, const char* file, int nth = 0) {
    int k = 0;
    const auto& log = card.log();
    for (std::size_t i = 0; i < log.size(); ++i) {
        if (log[i].op == op && log[i].file == file && k++ == nth) {
            return i;
        }
    }
    return npos;
}

}  // namespace

// ═══ T7. The typed drive ports as the reference (test 2's typed half) ══════════════════

TEST_CASE("CalibrateProgram T7a: an EMPTY card and typed ports that agree in every port -- SAME, saved, the "
          "NOTE FRONT MATCHES YOUR TYPED DRIVE PORTS (never FIRST RECORD)") {
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, kCardTyped));
    checkTypedLines(rig, kNoRecordTyped, kTypedList, kTypedSame);
    CHECK(countContaining(rig.io.lines, "SAME FRONT, but") == 0);  // exact: no per-port lines
    CHECK(countContaining(rig.io.lines, "this is the FIRST") == 0);
    CHECK(drew(rig, kDrawNoteSame));
    CHECK_FALSE(drew(rig, "FIRST RECORD"));
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "FRONT FLIPPED") == rig.headers().end());
    checkSaved(rig, 0, kFrontLedSides);
    int total = 0;
    CHECK(rig.forwardScore(decoded(rig.fileBytes(kRecordFileName)), total) == 10);
}

TEST_CASE("CalibrateProgram T7b: typed ports that agree in FRONT but not in every port -- SAME, one line per "
          "differing port (a sign, a typed port not in this push, a port not typed), the partial NOTE") {
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts({-11, -12, -13, 14, -15}, {20, -19, 18, -17, 9}));
    REQUIRE_FALSE(rig.timedOut);
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, "typed drive ports: LEFT -11 -12 -13 +14 -15 | RIGHT +20 -19 +18 -17 +9",
        "front check against your typed drive ports: 9 drive port(s) in both -- 8 agree, 0 flipped, 1 sign "
        "only, 0 side only; 1 in the typed ports only, 1 in this run's only: SAME FRONT");
    const auto& L = rig.io.lines;
    REQUIRE(at + 6 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 3 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 9: typed RIGHT +9, not in this push's record (LEFT OUT, declared, or did not turn)");
    CHECK(L[at + 5] == "  port 12: typed LEFT -12, this push LEFT +12 -- the SIGN differs");
    CHECK(L[at + 6] == "  port 16: this push RIGHT +16, not in your typed drive ports");
    CHECK(L[at + 7].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);  // nothing else between
    CHECK(drew(rig, kDrawNotePartial));
    CHECK_FALSE(drew(rig, kDrawNoteSame));
    checkSaved(rig, 0, kFrontLedSides);
}

TEST_CASE("CalibrateProgram T7b-2: the per-port lines name a SIDE difference and a side-AND-sign one apart "
          "from a sign -- each port's own words") {
    // 13 typed on the RIGHT with its own sign (the SIDE differs: side only), 14 typed on the RIGHT
    // with the other sign (both differ: flipped, by the check's own rule): still 8 of 10 agree, so
    // SAME, with two lines
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts({-11, 12, -15}, {-13, -14, 20, -19, 18, -17, 16}));
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, "typed drive ports: LEFT -11 +12 -15 | RIGHT -13 -14 +20 -19 +18 -17 +16",
        "front check against your typed drive ports: 10 drive port(s) in both -- 8 agree, 1 flipped, 0 sign "
        "only, 1 side only; 0 in the typed ports only, 0 in this run's only: SAME FRONT");
    REQUIRE(at + 5 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 2 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 13: typed RIGHT -13, this push LEFT -13 -- the SIDE differs");
    CHECK(L[at + 5] == "  port 14: typed RIGHT -14, this push LEFT +14 -- the side AND the sign differ");
    CHECK(drew(rig, kDrawNotePartial));
}

TEST_CASE("CalibrateProgram T7b-3: typed lists only SHORT a motor (every typed port agrees; one pushed port is "
          "not typed) -- still the PARTIAL note and that port's own line, never the exact NOTE") {
    // CAL1-A2b's review (2026-10-30): the pushed ports the team did not type, ALONE, are a difference —
    // a side short a motor is the checker's original case for C4. T7b and typed-partial only ever
    // combine them with other differences, so an exact NOTE painted whenever the only differences are
    // pushed-only ports survived the suite.
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts({-11, 12, -13, 14}, {20, -19, 18, -17, 16}));
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, "typed drive ports: LEFT -11 +12 -13 +14 | RIGHT +20 -19 +18 -17 +16",
        "front check against your typed drive ports: 9 drive port(s) in both -- 9 agree, 0 flipped, 0 sign "
        "only, 0 side only; 0 in the typed ports only, 1 in this run's only: SAME FRONT");
    REQUIRE(at + 5 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 1 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 15: this push LEFT -15, not in your typed drive ports");
    CHECK(L[at + 5].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);  // nothing else between
    CHECK(drew(rig, kDrawNotePartial));
    CHECK_FALSE(drew(rig, kDrawNoteSame));
    checkSaved(rig, 0, kFrontLedSides);
}

TEST_CASE("CalibrateProgram T7b-4: the per-port lines reach BOTH ends of the port range -- typed ports 1 and 21, "
          "in no push, are each named") {
    // CAL1-A2b's review (2026-10-30): every other typed case lies in ports 9..20, so a scan that
    // stopped at 20 or started at 2 named N-1 of the N ports its header counts.
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts({-11, 12, -13, 14, -15, 1}, {20, -19, 18, -17, 16, 21}));
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, "typed drive ports: LEFT -11 +12 -13 +14 -15 +1 | RIGHT +20 -19 +18 -17 +16 +21",
        "front check against your typed drive ports: 10 drive port(s) in both -- 10 agree, 0 flipped, 0 sign "
        "only, 0 side only; 2 in the typed ports only, 0 in this run's only: SAME FRONT");
    REQUIRE(at + 6 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 2 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 1: typed LEFT +1, not in this push's record (LEFT OUT, declared, or did not turn)");
    CHECK(L[at + 5] == "  port 21: typed RIGHT +21, not in this push's record (LEFT OUT, declared, or did not turn)");
    CHECK(L[at + 6].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);
    CHECK(drew(rig, kDrawNotePartial));
    checkSaved(rig, 0, kFrontLedSides);
}

TEST_CASE("CalibrateProgram T7c: the other end leads on an EMPTY card -- HELD on FRONT FLIPPED in the TYPED "
          "ports' words, nothing written; USE NEW saves as pushed, says FIX YOUR TYPED DRIVE PORTS") {
    Scenario sc;
    sc.pushDir[0] = -1.0;
    sc.heldFlip[0] = kLeftBox;
    Rig rig{sc};
    rig.run(typedFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(rig, kNoRecordTyped, kTypedList, kTypedFlipped);
    for (std::size_t k = 0; k < 5; ++k) {
        CAPTURE(k);
        CHECK(L[at + 3 + k] == kTypedFlip[k]);
    }
    CHECK(countContaining(L, "the card's record calls") == 0);  // never the card's words
    CHECK(has(rig, "screen: FRONT FLIPPED: NOTHING SAVED YET / typed ports: the end you led is BACK / RUN AGAIN: "
                   "you led with the BACK by mistake | [USE NEW] [RUN AGAIN]"));
    // the held screen, drawn in the typed ports' words — the card's three lines never
    CHECK(drew(rig, "print M 8 160 pen=e8b23a eraser=101418 'typed ports: the end you led is BACK'"));
    CHECK(drew(rig, "print S 8 196 pen=f0f0f0 eraser=101418 'USE NEW: you led with the FRONT -- fix typed ports'"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'USE NEW reverses autons tuned on the typed ports'"));
    CHECK_FALSE(drew(rig, "'the card calls the end you led BACK'"));
    CHECK_FALSE(drew(rig, "'USE NEW: you led with the FRONT (cables swapped?)'"));
    CHECK_FALSE(drew(rig, "'USE NEW after a wrong push reverses every auton'"));
    // NOTHING written while held: no file at all until the USE NEW tap
    const auto* use = rig.tap("FRONT FLIPPED1");
    REQUIRE(use != nullptr);
    CHECK(rig.replacesFrom(0) - rig.replacesFrom(use->cardOps) == 0);
    // USE NEW: as pushed, then the FIX line and NOTE
    const std::size_t u = findLine(L, "front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)");
    REQUIRE(u != npos);
    CHECK(L[u + 1] == "front: now FIX YOUR TYPED DRIVE PORTS -- they call this FRONT the BACK, and anything that "
                      "still drives from them drives this robot backwards");
    CHECK(L[u + 2].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);
    CHECK(drew(rig, kDrawNoteFix));
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED -- NEW FRONT") != rig.headers().end());
    checkSaved(rig, 0, kOtherEndSides);
    int total = 0;
    CHECK(rig.forwardScore(decoded(rig.fileBytes(kRecordFileName)), total) == 0);  // the warned-of case
}

TEST_CASE("CalibrateProgram T7d: FRONT FLIPPED against the typed ports, RUN AGAIN -- nothing written, 'the "
          "card is unchanged'; a front-led run 2 is SAME and saves") {
    Scenario sc;
    sc.runs = 2;
    sc.pushDir[0] = -1.0;
    sc.heldFlip[0] = kRunAgain;
    Rig rig{sc};
    rig.run(typedFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    checkTypedLines(rig, kNoRecordTyped, kTypedList, kTypedFlipped);
    CHECK(has(rig, std::string{"RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, "} + kUnchanged));
    CHECK(countContaining(L, "keeps its record") == 0);
    const auto* again = rig.tap("FRONT FLIPPED2");
    REQUIRE(again != nullptr);
    CHECK(rig.replacesFrom(0) - rig.replacesFrom(again->cardOps) == 0);  // nothing before the RUN AGAIN
    // run 2: the typed check again, SAME this time
    const std::size_t first = findLine(L, kNoRecordTyped);
    REQUIRE(first != npos);
    const std::size_t second = findLine(L, kNoRecordTyped, first + 1);
    REQUIRE(second != npos);
    CHECK(L[second + 2] == kTypedSame);
    checkSaved(rig, 1, kFrontLedSides);
    CHECK(drew(rig, kDrawNoteSame));
}

TEST_CASE("CalibrateProgram T7e: the field disables on a typed FRONT FLIPPED -- said with the card's own words "
          "(no usable record), and NO file on the card") {
    Scenario sc;
    sc.pushDir[0] = -1.0;
    sc.heldFlip[0] = kDisable;
    Rig rig{sc};
    rig.run(typedFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, std::string{"the field disabled the robot on the held FRONT FLIPPED screen -- NOTHING SAVED: "}
                       + kUnchanged));
    CHECK(rig.fileBytes(kRecordFileName).empty());
    CHECK(rig.fileBytes(kRecordBackupFileName).empty());
    CHECK(rig.replacesFrom(0) == 0);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T7f: typed ports half flipped (a 5/5 tie) and typed ports with no port in common "
          "-- CANNOT TELL, saved as measured, FRONT NOT CHECKED") {
    struct Case {
        const char* name;
        CalibrateFacts facts;
        const char* list;
        const char* verdict;
    };
    const Case cases[] = {
        {"tie", typedFacts({-11, 12, -13, -16, 17, -20}, {-14, 15, 18, -19}),
         "typed drive ports: LEFT -11 +12 -13 -16 +17 -20 | RIGHT -14 +15 +18 -19",
         "front check against your typed drive ports: 10 drive port(s) in both -- 5 agree, 5 flipped, 0 sign "
         "only, 0 side only; 0 in the typed ports only, 0 in this run's only: CANNOT TELL"},
        {"no port in common", typedFacts({-3, 4}, {5, -8}), "typed drive ports: LEFT -3 +4 | RIGHT +5 -8",
         "front check against your typed drive ports: 0 drive port(s) in both -- 0 agree, 0 flipped, 0 sign "
         "only, 0 side only; 4 in the typed ports only, 10 in this run's only: CANNOT TELL"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.name});
        Scenario sc;
        Rig rig{sc};
        rig.run(c.facts);
        REQUIRE_FALSE(rig.timedOut);
        const std::size_t at = checkTypedLines(rig, kNoRecordTyped, c.list, c.verdict);
        CHECK(rig.io.lines[at + 3] == kCannotTell);
        CHECK(drew(rig, kDrawNoteNotChecked));
        CHECK_FALSE(drew(rig, "FRONT MATCHES"));
        CHECK(countContaining(rig.io.lines, "SAME FRONT, but") == 0);
        checkSaved(rig, 0, kFrontLedSides);
    }
}

TEST_CASE("CalibrateProgram T7g: a record on the card AND typed ports flipped against it -- the CARD decides, "
          "and the whole run EQUALS the same run with no typed ports: every line, every draw, every byte") {
    Scenario sc;
    Rig typed{sc};
    typed.card.setFile(kRecordFileName, frontLedRecord());
    typed.card.setFile(kRecordBackupFileName, frontLedRecord());
    typed.run(typedFacts({-20, 19, -18, 17, -16}, {11, -12, 13, -14, 15}));
    Rig plain{sc};
    plain.card.setFile(kRecordFileName, frontLedRecord());
    plain.card.setFile(kRecordBackupFileName, frontLedRecord());
    plain.run(robotTwoFacts());
    REQUIRE_FALSE(typed.timedOut);
    REQUIRE_FALSE(plain.timedOut);
    CHECK(typed.io.lines == plain.io.lines);
    CHECK(typed.screen.log() == plain.screen.log());
    CHECK(typed.fileBytes(kRecordFileName) == plain.fileBytes(kRecordFileName));
    CHECK(typed.fileBytes(kRecordBackupFileName) == plain.fileBytes(kRecordBackupFileName));
    // ... and that run is the card's check, SAME
    CHECK(countContaining(typed.io.lines, "typed drive ports") == 0);
    CHECK(has(typed, "front check against shulib_drivetrain.bin: 10 drive port(s) in both -- 10 agree, 0 flipped, 0 "
                     "sign only, 0 side only; 0 in the card's only, 0 in this run's only: SAME FRONT"));
}

TEST_CASE("CalibrateProgram T7g-2: a damaged record file, a GOOD BACKUP, and typed ports flipped against it -- the "
          "record the BACKUP holds decides, and the run EQUALS the untyped run (A2a's T2i, typed)") {
    // CAL1-A2b's review (2026-10-30): "a record (either file decodes)" — T7g only ever had BOTH files
    // good, so typed ports taking over whenever the record FILE did not decode survived the suite.
    Scenario sc;
    Rig typed{sc};
    typed.card.setFile(kRecordFileName, garbage(180));
    typed.card.setFile(kRecordBackupFileName, frontLedRecord());
    typed.run(typedFacts({-20, 19, -18, 17, -16}, {11, -12, 13, -14, 15}));
    Rig plain{sc};
    plain.card.setFile(kRecordFileName, garbage(180));
    plain.card.setFile(kRecordBackupFileName, frontLedRecord());
    plain.run(robotTwoFacts());
    REQUIRE_FALSE(typed.timedOut);
    REQUIRE_FALSE(plain.timedOut);
    CHECK(typed.io.lines == plain.io.lines);
    CHECK(typed.screen.log() == plain.screen.log());
    CHECK(typed.fileBytes(kRecordFileName) == plain.fileBytes(kRecordFileName));
    CHECK(typed.fileBytes(kRecordBackupFileName) == plain.fileBytes(kRecordBackupFileName));
    // ... and that run is the BACKUP's check, SAME — no typed word, no hold
    CHECK(countContaining(typed.io.lines, "typed drive ports") == 0);
    CHECK(has(typed, "front check against shulib_drivetrain.bak: 10 drive port(s) in both -- 10 agree, 0 flipped, 0 "
                     "sign only, 0 side only; 0 in the card's only, 0 in this run's only: SAME FRONT"));
    CHECK(std::find(typed.headers().begin(), typed.headers().end(), "FRONT FLIPPED") == typed.headers().end());
}

TEST_CASE("CalibrateProgram T7h: files that do not decode, typed ports present -- each opened file's own line, "
          "then the TYPED ports decide (both files, the backup alone, an EMPTY record file); flipped, it holds") {
    SUBCASE("both files undecodable, typed ports agreeing: SAME, saved over them") {
        Scenario sc;
        Rig rig{sc};
        rig.card.setFile(kRecordFileName, garbage(180));
        rig.card.setFile(kRecordBackupFileName, garbage(180));
        rig.run(typedFacts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        CHECK(has(rig, kCardTyped));
        const std::size_t at = findLine(L, kUndecodedTyped);
        REQUIRE(at != npos);
        CHECK(L[at + 1] == "  shulib_drivetrain.bin: magic is not \"SHDC\" -- this is not a calibration record");
        CHECK(L[at + 2] == "  shulib_drivetrain.bak: magic is not \"SHDC\" -- this is not a calibration record");
        CHECK(L[at + 3] == kTypedList);
        CHECK(L[at + 4] == kTypedSame);
        CHECK(countContaining(L, "NOT DONE") == 0);
        CHECK(drew(rig, kDrawNoteSame));
        checkSaved(rig, 0, kFrontLedSides);
    }
    SUBCASE("both files undecodable, the other end leading: HELD in the typed words, USE NEW saves") {
        Scenario sc;
        sc.pushDir[0] = -1.0;
        sc.heldFlip[0] = kLeftBox;
        Rig rig{sc};
        rig.card.setFile(kRecordFileName, garbage(180));
        rig.card.setFile(kRecordBackupFileName, garbage(180));
        rig.run(typedFacts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        const std::size_t at = findLine(L, kUndecodedTyped);
        REQUIRE(at != npos);
        CHECK(L[at + 4] == kTypedFlipped);
        CHECK(L[at + 5] == kTypedFlip[0]);
        const auto* use = rig.tap("FRONT FLIPPED1");
        REQUIRE(use != nullptr);
        CHECK(rig.replacesFrom(0) - rig.replacesFrom(use->cardOps) == 0);
        CHECK(drew(rig, kDrawNoteFix));
        checkSaved(rig, 0, kOtherEndSides);
    }
    SUBCASE("only the backup, undecodable: its line alone, never an empty reason") {
        Scenario sc;
        Rig rig{sc};
        rig.card.setFile(kRecordBackupFileName, garbage(180));
        rig.run(typedFacts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        const std::size_t at = findLine(L, kUndecodedTyped);  // the backup OPENED: "does not decode"
        REQUIRE(at != npos);
        CHECK(L[at + 1] == "  shulib_drivetrain.bak: magic is not \"SHDC\" -- this is not a calibration record");
        CHECK(L[at + 2] == kTypedList);
        CHECK(countContaining(L, kNoRecordTyped) == 0);
        CHECK(countContaining(L, "()") == 0);
    }
    SUBCASE("an EMPTY record file: 'EMPTY (0 bytes read)', then the typed ports") {
        Scenario sc;
        Rig rig{sc};
        rig.card.setFile(kRecordFileName, {});
        rig.run(typedFacts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        const std::size_t at = findLine(L, kUndecodedTyped);
        REQUIRE(at != npos);
        CHECK(L[at + 1] == "  shulib_drivetrain.bin: EMPTY (0 bytes read)");
        CHECK(L[at + 2] == kTypedList);
        CHECK(L[at + 3] == kTypedSame);
    }
}

TEST_CASE("CalibrateProgram T7i: NO card, typed ports present -- no check, typed or not: the run EQUALS the "
          "untyped no-card run, and no typed line is printed") {
    Scenario sc;
    sc.card = false;
    Rig typed{sc};
    typed.run(typedFacts());
    Rig plain{sc};
    plain.run(robotTwoFacts());
    REQUIRE_FALSE(typed.timedOut);
    REQUIRE_FALSE(plain.timedOut);
    CHECK(typed.io.lines == plain.io.lines);
    CHECK(typed.screen.log() == plain.screen.log());
    CHECK(has(typed, "card       : NO SD CARD in the brain -- nothing can be saved, and the FRONT is not checked"));
    CHECK(has(typed, "front check: NOT DONE -- no SD card in the brain (nothing can be saved either)"));
    CHECK(countContaining(typed.io.lines, "typed drive ports") == 0);
    CHECK(std::find(typed.headers().begin(), typed.headers().end(), "FRONT FLIPPED") == typed.headers().end());
}

TEST_CASE("CalibrateProgram T7j: a DRIVE port (12) declared a mechanism, an empty card, typed ports that drive "
          "with it -- the MECHANISM PORT TURNED evidence names the TYPED ports; RUN AGAIN and a disable say "
          "the card is unchanged, and nothing is written") {
    Scenario sc;
    sc.runs = 2;
    sc.heldMech[0] = kRunAgain;
    sc.heldMech[1] = kDisable;
    Rig rig{sc};
    CalibrateFacts f = typedFacts();
    f.mechanisms[1].port = 12;  // claw pivot 7 typed as 12
    rig.run(f);
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, kTypedList,
        "front check against your typed drive ports: 9 drive port(s) in both -- 9 agree, 0 flipped, 0 sign "
        "only, 0 side only; 1 in the typed ports only, 0 in this run's only: SAME FRONT");
    REQUIRE(at + 6 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 1 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 12: typed LEFT +12, not in this push's record (LEFT OUT, declared, or did not turn)");
    CHECK(L[at + 5] == "MECHANISM PORT TURNED -- port 12 is declared a mechanism but turned with the push (port 12: "
                       "1692 deg, drive 1692 deg).");
    CHECK(L[at + 6] == "Your typed drive ports DRIVE with port 12: saving now makes a record with NO port 12.");
    CHECK(countContaining(L, "The card's record DRIVES") == 0);
    CHECK(has(rig, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 12: 1692 deg, drive 1692 deg / your typed "
                   "ports DRIVE with port 12 | [SAVE] [RUN AGAIN]"));
    CHECK(drew(rig, "print S 8 180 pen=f0f0f0 eraser=101418 'your typed ports DRIVE with port 12'"));
    CHECK(has(rig, std::string{"RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, "} + kUnchanged));
    CHECK(has(rig, std::string{"the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING "
                               "SAVED: "} + kUnchanged));
    CHECK(rig.fileBytes(kRecordFileName).empty());
    CHECK(rig.replacesFrom(0) == 0);
    // the typed FRONT FLIPPED words never reach the MECHANISM screen
    CHECK_FALSE(drew(rig, "typed ports: the end you led is BACK"));
}

TEST_CASE("CalibrateProgram T7j-2: a typed RIGHT drive port (18) declared a mechanism -- the evidence says the "
          "typed ports DRIVE with it, on the right side as on the left") {
    // CAL1-A2b's review (2026-10-30): T7j and typed-watch-typo both declare 12, typed LEFT +12, so a
    // rule that judged only the LEFT side ("sideOf < 0") survived — and on the right side it would
    // say "not in this record", inviting a SAVE that drops a drive motor.
    Scenario sc;
    sc.heldMech[0] = kDisable;
    Rig rig{sc};
    CalibrateFacts f = typedFacts();
    f.mechanisms[1].port = 18;  // claw pivot 7 typed as 18
    rig.run(f);
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, kTypedList,
        "front check against your typed drive ports: 9 drive port(s) in both -- 9 agree, 0 flipped, 0 sign "
        "only, 0 side only; 1 in the typed ports only, 0 in this run's only: SAME FRONT");
    REQUIRE(at + 6 < L.size());
    CHECK(L[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 1 port(s) -- "
                       "check each before anything drives from the typed ports:");
    CHECK(L[at + 4] == "  port 18: typed RIGHT +18, not in this push's record (LEFT OUT, declared, or did not turn)");
    CHECK(L[at + 5].rfind("MECHANISM PORT TURNED -- port 18 is declared a mechanism but turned with the push (port 18: ",
                          0)
          == 0);
    CHECK(L[at + 6] == "Your typed drive ports DRIVE with port 18: saving now makes a record with NO port 18.");
    CHECK(drew(rig, "print S 8 180 pen=f0f0f0 eraser=101418 'your typed ports DRIVE with port 18'"));
    CHECK_FALSE(drew(rig, "'it is declared a mechanism: not in this record'"));
    CHECK(has(rig, std::string{"the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING "
                               "SAVED: "} + kUnchanged));
    CHECK(rig.fileBytes(kRecordFileName).empty());
    CHECK(rig.replacesFrom(0) == 0);
}

TEST_CASE("CalibrateProgram T7k: FRONT FLIPPED against the typed ports AND a declared claw turned -- USE NEW "
          "says FIX, the MECHANISM question follows, and after its SAVE the NOTE is PORT 6 KEPT (the NOTE's "
          "order), the record as pushed") {
    Scenario sc;
    sc.pushDir[0] = -1.0;
    sc.clawTurns = 6;
    sc.heldFlip[0] = kLeftBox;
    sc.heldMech[0] = kLeftBox;
    Rig rig{sc};
    rig.run(typedFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t at = checkTypedLines(rig, kNoRecordTyped, kTypedList, kTypedFlipped);
    // the hold is in the TYPED ports' words, with a MECHANISM question still to come — on the serial
    // and on the held screen (CAL1-A2b's review, 2026-10-30: with the card's words here, or the card's
    // three screen lines, this run would claim a record the empty card does not hold — and both
    // survived the suite, because only T7c checked these words, with no question to come)
    REQUIRE(at + 7 < L.size());
    for (std::size_t k = 0; k < 5; ++k) {
        CAPTURE(k);
        CHECK(L[at + 3 + k] == kTypedFlip[k]);
    }
    CHECK(countContaining(L, "the card's record calls") == 0);
    CHECK(has(rig, "screen: FRONT FLIPPED: NOTHING SAVED YET / typed ports: the end you led is BACK / RUN AGAIN: "
                   "you led with the BACK by mistake | [USE NEW] [RUN AGAIN]"));
    CHECK(drew(rig, "print M 8 160 pen=e8b23a eraser=101418 'typed ports: the end you led is BACK'"));
    CHECK(drew(rig, "print S 8 196 pen=f0f0f0 eraser=101418 'USE NEW: you led with the FRONT -- fix typed ports'"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'USE NEW reverses autons tuned on the typed ports'"));
    CHECK_FALSE(drew(rig, "'the card calls the end you led BACK'"));
    CHECK_FALSE(drew(rig, "'USE NEW: you led with the FRONT (cables swapped?)'"));
    CHECK_FALSE(drew(rig, "'USE NEW after a wrong push reverses every auton'"));
    CHECK(has(rig, "touch: x=120 y=115 release=2 on USE NEW -> USE NEW -- the next question follows; nothing is "
                   "saved yet"));
    const std::size_t u = findLine(L, "front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)");
    REQUIRE(u != npos);
    CHECK(L[u + 1].rfind("front: now FIX YOUR TYPED DRIVE PORTS", 0) == 0);
    CHECK(L[u + 2].rfind("MECHANISM PORT TURNED -- port 6 is declared a mechanism", 0) == 0);
    // port 6 is not a typed drive port: the screen says so in the mechanism's own words (the claw
    // follows the LEFT wheel, which the other end's push turns backwards: -1692)
    CHECK(has(rig, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 6: -1692 deg, drive 1692 deg / it is "
                   "declared a mechanism: not in this record | [SAVE] [RUN AGAIN]"));
    CHECK(countContaining(L, "DRIVE with port 6") == 0);
    // nothing written until the last answer
    const auto* save = rig.tap("MECHANISM PORT TURNED1");
    REQUIRE(save != nullptr);
    CHECK(rig.replacesFrom(0) - rig.replacesFrom(save->cardOps) == 0);
    CHECK(has(rig, "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'PORT 6 KEPT AS A MECHANISM -- see the log'"));
    CHECK_FALSE(drew(rig, kDrawNoteFix));  // the NOTE's first that applies
    checkSaved(rig, 0, kOtherEndSides);
}

TEST_CASE("CalibrateProgram T7l: a typed check whose SAVE FAILS -- NOT SAVED, and the typed NOTE still drawn "
          "(it describes the comparison, true whether or not the save stuck); never FIRST RECORD") {
    // CAL1-A2b's ruling 2 (A2b.1): A2a drops only FIRST RECORD under a NOT SAVED header (it names a
    // record the run made); the typed NOTEs stay — FIX YOUR TYPED DRIVE PORTS is still true when the
    // save failed. CAL1-A2b's review (2026-10-30): no test ran a typed reference whose save failed, so
    // dropping the typed NOTEs there too survived the suite. The fault: a short write of the record
    // file's first replace on an empty card (A2a's T4c's).
    struct Case {
        const char* name;
        double pushDir;
        const char* note;
        const char* notHeader;  ///< a header this run must NOT show
    };
    const Case cases[] = {
        {"every typed port agrees: FRONT MATCHES YOUR TYPED DRIVE PORTS", 1.0, kDrawNoteSame, "SAVED"},
        {"flipped, USE NEW: FIX YOUR TYPED DRIVE PORTS", -1.0, kDrawNoteFix, "SAVED -- NEW FRONT"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.name});
        std::size_t target = npos;
        {
            Scenario sc;
            sc.pushDir[0] = c.pushDir;
            sc.heldFlip[0] = kLeftBox;
            Rig dry{sc};
            dry.run(typedFacts());
            REQUIRE_FALSE(dry.timedOut);
            REQUIRE(drew(dry, c.note));  // the control: saved, and noted
            target = opIndex(dry.card, FakeFileOp::Replace, kRecordFileName);
            REQUIRE(target != npos);
        }
        Scenario sc;
        sc.pushDir[0] = c.pushDir;
        sc.heldFlip[0] = kLeftBox;
        Rig rig{sc};
        FakeFileFaultPlan plan{};
        plan.atOperation = target;
        plan.fault = FakeFileFault::ShortWrite;
        plan.amount = 30;
        rig.card.plan(plan);
        rig.run(typedFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read back 30 of "
                       "180 bytes)."));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "NOT SAVED") != rig.headers().end());
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), c.notHeader) == rig.headers().end());
        CHECK(drew(rig, "print M 8 4 pen=101418 eraser=e2564a 'NOT SAVED"));
        CHECK(drew(rig, c.note));
        CHECK_FALSE(drew(rig, "FIRST RECORD"));
    }
}

// ═══ T6g-T6i. The typed lists in the facts (test 6's typed half) ═══════════════════════

TEST_CASE("CalibrateProgram T6g: the typed lists' refusals -- a port outside 1..21, twice on one side, once on "
          "each side, one side only (either) -- FACTS REFUSED naming the entry and its value, before ANY device") {
    struct Case {
        const char* name;
        CalibrateFacts facts;
        const char* why;
    };
    const Case cases[] = {
        {"-22 on the left", typedFacts({-11, 12, -22, 14, -15}),
         "a typed drive port is not a smart port (1..21): typedDrive.left[2] = -22"},
        {"22 on the right", typedFacts({-11, 12, -13, 14, -15}, {20, -19, 22}),
         "a typed drive port is not a smart port (1..21): typedDrive.right[2] = 22"},
        {"-128 on the left", typedFacts({-128}),
         "a typed drive port is not a smart port (1..21): typedDrive.left[0] = -128"},
        {"11 twice on the left (opposite signs)", typedFacts({-11, 12, -13, 14, 11}),
         "a typed drive port is listed twice (one side, or both): typedDrive.left[4] = 11"},
        {"11 twice on the left, in ADJACENT slots", typedFacts({-11, 11, -13, 14, -15}),
         "a typed drive port is listed twice (one side, or both): typedDrive.left[1] = 11"},
        {"11 on each side", typedFacts({-11, 12, -13, 14, -15}, {20, -19, 18, -17, 11}),
         "a typed drive port is listed twice (one side, or both): typedDrive.right[4] = 11"},
        {"the left only", typedFacts({-11, 12, -13, 14, -15}, {}),
         "typed drive ports must list BOTH sides -- the RIGHT is empty: typedDrive.right[0] = 0"},
        {"the right only", typedFacts({}, {20, -19, 18, -17, 16}),
         "typed drive ports must list BOTH sides -- the LEFT is empty: typedDrive.left[0] = 0"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.name});
        Scenario sc;
        Rig rig{sc};
        rig.run(c.facts);
        checkRefusedBeforeAnything(rig, "FACTS REFUSED", c.why, "correct the robot's stated facts and rebuild");
        CHECK(countContaining(rig.io.lines, "(declared)") == 0);
    }
    SUBCASE("a mechanism problem is reported first") {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = typedFacts({-11, 12, -13, 14, 11});
        f.mechanisms[2] = {0, "intake"};
        rig.run(f);
        checkRefusedBeforeAnything(
            rig, "FACTS REFUSED",
            "a mechanism is named but its port is 0 -- state its port: mechanisms[2] = {0, \"intake\"}",
            "correct the robot's stated facts and rebuild");
    }
}

TEST_CASE("CalibrateProgram T6h: unused (0) typed slots are skipped and the order kept -- the program compares "
          "the compacted list, and it agrees in every port") {
    Scenario sc;
    Rig rig{sc};
    rig.run(typedFacts({0, -11, 0, 12, 0, -13, 14, -15}, {20, -19, 0, 18, -17, 0, 16}));
    REQUIRE_FALSE(rig.timedOut);
    checkTypedLines(rig, kNoRecordTyped, kTypedList, kTypedSame);
    CHECK(drew(rig, kDrawNoteSame));
}

TEST_CASE("CalibrateProgram T6h-2: typed ports in the LAST slot (index 20) ALONE, on both sides -- typed, and "
          "checked: the scan reaches the last slot") {
    // CAL1-A2b's review (2026-10-30): a scan that stopped at slot 19 ran these facts UNTYPED — no
    // refusal, no typed check, a FIRST RECORD instead
    Scenario sc;
    Rig rig{sc};
    CalibrateFacts f = robotTwoFacts();
    f.typedDrive = {};
    f.typedDrive.left[20] = -11;
    f.typedDrive.right[20] = 20;
    rig.run(f);
    REQUIRE_FALSE(rig.timedOut);
    const std::size_t at = checkTypedLines(
        rig, kNoRecordTyped, "typed drive ports: LEFT -11 | RIGHT +20",
        "front check against your typed drive ports: 2 drive port(s) in both -- 2 agree, 0 flipped, 0 sign "
        "only, 0 side only; 0 in the typed ports only, 8 in this run's only: SAME FRONT");
    REQUIRE(at + 3 < rig.io.lines.size());
    CHECK(rig.io.lines[at + 3] == "front check: SAME FRONT, but your typed drive ports and this push differ at 8 "
                                  "port(s) -- check each before anything drives from the typed ports:");
    CHECK(drew(rig, kDrawNotePartial));
    CHECK_FALSE(drew(rig, "FIRST RECORD"));
}

TEST_CASE("CalibrateProgram T6i: the typed facts at COMPILE time -- factsProblem() naming the list and the entry, "
          "hasTypedDrive() on either side, typedDriveDescription() compacting in order, typedDrivePortsFrom() its "
          "inverse") {
    using shulib::calibration::factsProblem;
    using shulib::calibration::FactsList;
    using shulib::calibration::hasTypedDrive;
    using shulib::calibration::typedDriveDescription;
    using shulib::calibration::typedDrivePortsFrom;
    constexpr CalibrateFacts none{};
    static_assert(!hasTypedDrive(none));
    static_assert(factsProblem(none).why == nullptr && factsProblem(none).list == FactsList::Mechanisms);
    constexpr CalibrateFacts typed{.typedDrive = {.left = {-11, 12}, .right = {20, -19}}};
    static_assert(hasTypedDrive(typed) && factsProblem(typed).why == nullptr);
    constexpr CalibrateFacts leftOnly{.typedDrive = {.left = {-11, 12}}};
    static_assert(hasTypedDrive(leftOnly));
    static_assert(factsProblem(leftOnly).why != nullptr && factsProblem(leftOnly).list == FactsList::TypedRight
                  && factsProblem(leftOnly).entry == 0);
    constexpr CalibrateFacts rightOnly{.typedDrive = {.right = {0, 0, 7}}};  // a non-zero entry anywhere counts
    static_assert(hasTypedDrive(rightOnly));
    static_assert(factsProblem(rightOnly).why != nullptr && factsProblem(rightOnly).list == FactsList::TypedLeft);
    constexpr CalibrateFacts edge{.typedDrive = {.left = {-21, 1}, .right = {21 - 1, -2}}};
    static_assert(factsProblem(edge).why == nullptr);  // 1 and 21 are smart ports
    constexpr CalibrateFacts past{.typedDrive = {.left = {-11}, .right = {12, 22}}};
    static_assert(factsProblem(past).why != nullptr && factsProblem(past).list == FactsList::TypedRight
                  && factsProblem(past).entry == 1);
    constexpr CalibrateFacts across{.typedDrive = {.left = {-11, 12}, .right = {20, -12}}};
    static_assert(factsProblem(across).why != nullptr && factsProblem(across).list == FactsList::TypedRight
                  && factsProblem(across).entry == 1);  // by |port|: -12 is 12
    constexpr CalibrateFacts lastSlot{.typedDrive = {.left = {5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                              0, 0, -5},
                                                     .right = {6}}};
    static_assert(factsProblem(lastSlot).why != nullptr && factsProblem(lastSlot).list == FactsList::TypedLeft
                  && factsProblem(lastSlot).entry == 20);
    // the LAST slot ALONE counts, on either side — the scan reaches index 20 (CAL1-A2b's review,
    // 2026-10-30: lastSlot above is decided at index 0)
    constexpr CalibrateFacts lastOnly{
        .typedDrive = {.left = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -11},
                       .right = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20}}};
    static_assert(hasTypedDrive(lastOnly) && factsProblem(lastOnly).why == nullptr);
    constexpr auto dLast = typedDriveDescription(lastOnly);
    static_assert(dLast.left.count == 1 && dLast.left.signedPorts[0] == -11 && dLast.right.count == 1
                  && dLast.right.signedPorts[0] == 20);
    constexpr CalibrateFacts lastLeftOnly{
        .typedDrive = {.left = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -11}}};
    static_assert(hasTypedDrive(lastLeftOnly));
    static_assert(factsProblem(lastLeftOnly).why != nullptr && factsProblem(lastLeftOnly).list == FactsList::TypedRight
                  && factsProblem(lastLeftOnly).entry == 0);
    constexpr CalibrateFacts lastRightOnly{
        .typedDrive = {.right = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20}}};
    static_assert(hasTypedDrive(lastRightOnly));
    static_assert(factsProblem(lastRightOnly).why != nullptr
                  && factsProblem(lastRightOnly).list == FactsList::TypedLeft && factsProblem(lastRightOnly).entry == 0);
    // a duplicate in ADJACENT slots, on either side (the same-side scan starts at the NEXT slot)
    constexpr CalibrateFacts adjacent{.typedDrive = {.left = {-11, 11}, .right = {20}}};
    static_assert(factsProblem(adjacent).why != nullptr && factsProblem(adjacent).list == FactsList::TypedLeft
                  && factsProblem(adjacent).entry == 1);
    constexpr CalibrateFacts adjacentRight{.typedDrive = {.left = {-11}, .right = {20, -20}}};
    static_assert(factsProblem(adjacentRight).why != nullptr
                  && factsProblem(adjacentRight).list == FactsList::TypedRight && factsProblem(adjacentRight).entry == 1);
    constexpr CalibrateFacts mechFirst{.mechanisms = {{6, "a"}, {6, "b"}}, .typedDrive = {.left = {-22}}};
    static_assert(factsProblem(mechFirst).list == FactsList::Mechanisms && factsProblem(mechFirst).entry == 1);
    // the description: the sides only, zero slots skipped, the order kept
    constexpr CalibrateFacts gappy{.typedDrive = {.left = {0, -21, 0, 12}, .right = {0, 0, 20}}};
    static_assert(factsProblem(gappy).why == nullptr);
    constexpr auto d = typedDriveDescription(gappy);
    static_assert(d.left.count == 2 && d.left.signedPorts[0] == -21 && d.left.signedPorts[1] == 12);
    static_assert(d.right.count == 1 && d.right.signedPorts[0] == 20);
    static_assert(d.sideOf(21) == -1 && d.signedPortOf(21) == -21 && d.sideOf(20) == +1 && d.sideOf(5) == 0);
    // its inverse: the compacted lists back as typed drive ports
    constexpr auto t = typedDrivePortsFrom(d);
    static_assert(t.left[0] == -21 && t.left[1] == 12 && t.left[2] == 0 && t.right[0] == 20 && t.right[1] == 0);
    CHECK(std::string{factsProblem(across).why} == "a typed drive port is listed twice (one side, or both)");
}
