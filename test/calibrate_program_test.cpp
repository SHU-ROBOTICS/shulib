// calibrate_program_test.cpp — the standard Calibrate (calibration::CalibrateProgram, chunk CAL1
// Part A2a), RUN WHOLE on the A2 host test plant through the library's fakes
// (test/calibrate_program_rig.hpp says how, and why the operator reads only the screen).
//
// The CAL1 brief's §2.A tests, A2a's share (the team lead's split, 2026-10-08):
//   T1  the scenarios, robot two's facts written literally (no typed drive ports — CAL1 Part B
//       derives exactly these): one, two runs, a slipping port, a curved then a straight push, no
//       card, no IMU, the field disabling mid-pulses, unequal side scales, a program fault, touch
//       errors at start, two programs alive at once (each its own build time); since the review
//       round (A2a's review findings): 0 V + Coast before the IMU is asked for (T1l), AUTONOMOUS
//       ending driver control as a disable does — mid-pulses with and without the "connected" bit,
//       a field's disable with it, after a finished run, during a fault's idle (T1m, T1n); since the
//       SECOND review round: the touch baseline taken by a retry (T1j-2), a field CONNECTED from the
//       start (T1o), a PROGRAM FAULT while the motors are POWERED, every motor stopped before the
//       idle's first wait (T1p), a slow tick in the loop's worst gap (T1q), a tap on a live DONE the
//       calibrator no longer accepts (T1r), and the push-live throttle's two edges landed on exactly
//       — the function's (T1s) and, since the third verification round, the PROGRAM's own lines on
//       the brain's clock formula (T1t). Each asserts VERBATIM the serial lines that carry a decision (the config and loop
//       lines too), the saved record's bytes, and that the card's decoded record equals the printed
//       MEASURED line.
//   T2  the front holds, the UNTYPED half (A2b adds the typed reference): FRONT FLIPPED held,
//       RUN AGAIN, USE NEW, a field disable while held, the cable swap, nothing held left behind
//       by a discarded hold; the first record, no card, a record that does not decode (both files,
//       the backup alone, an EMPTY file), CANNOT TELL, a damaged record file with a good backup;
//       autonomous on the held screen (T2k). A held run's saved record is pinned FIELD BY FIELD to
//       what the run printed before the hold (checkRecordIsTheMeasurement) — never to its own bytes.
//   T4  the save is SAFE-SAVE B1's: the program's card operations are EXACTLY two loadRecord()
//       reads and one saveRecordSafely(), and its words exactly formatSaveReport(No)'s — under
//       every planted card fault — and the panel says SAVED iff the save measured it; a source
//       pin: one writer of the record, no stdio printer, no PROS; a FIRST record whose save fails
//       paints no FIRST RECORD note (T4c).
//   T5  the watch and its hold — the turn, its exact edge (the calibrator's own >=), a read that
//       fails or is NaN, the discard words per card (a record, none, no card), FRONT FLIPPED then
//       MECHANISM PORT TURNED with its box re-greyed, nothing declared, and in EVERY scenario the
//       declared ports never asked for or commanded; the TRAVEL judged, never a claw's reading, and
//       only from a baseline that was read (T5k); TWO declared ports turned — one question per port,
//       each with its own evidence, in the facts' order, re-greyed between, nothing saved before the
//       last SAVE (T5l-T5n, the review's reproduced hazard); a discard over files that do not decode
//       (T5o); a claw that settles BEFORE the push, read from the push's own baseline (T5p); a port
//       whose DONE read alone fails (T5k); T5b undeclared mechanism motors, PULSE_STILL and
//       PULSE_ROLLED.
//   T6  the facts: every refusal before ANY device is asked for (and before the declared-port drop
//       prints a line), each naming its entry; TOO FEW MOTORS naming the declared ports, and its
//       edge (exactly two starts); the IMU count's edge (two refused); the adapters' refusals (a
//       motor dropped by name — the LAST port, and a MIDDLE one run to its saved record —, MOTORS
//       REFUSED over an inherited powered motor, IMU REFUSED); a stated IMU port as a cross-check;
//       the drop in the census's order; factsProblem() at compile time.
// Test 8b (the PROS shell) is test/pros_calibrate_adapter_test.cpp; test 9 (the panel's defaults
// are today's screens) is test/calibrate_panel_additions_test.cpp.
//
// Every expected string is written in the test, never computed by the code under test. Nothing
// here has run on a robot.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "calibrate_program_rig.hpp"
#include "doctest.h"
#include "shulib/calibration/calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
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

/// The record a clean, front-led run saves on an empty card — the seed every "card holds a
/// record" scenario starts from. Made ONCE, by the program itself, and checked before use: a
/// seeded scenario never runs on an empty card by accident (the CAL1 check of D6 (a), C2).
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

/// `n` bytes of 'x' — 180 of them are the probe's "files that do not decode".
std::vector<std::byte> garbage(std::size_t n) { return std::vector<std::byte>(n, std::byte{'x'}); }

void seedBoth(Rig& rig, const std::vector<std::byte>& bytes) {
    rig.card.setFile(kRecordFileName, bytes);
    rig.card.setFile(kRecordBackupFileName, bytes);
}

bool has(const Rig& rig, const std::string& line) { return findLine(rig.io.lines, line) != npos; }

/// Some draw on the screen contains `part`.
bool drew(const Rig& rig, const std::string& part) {
    for (const auto& l : rig.screen.log()) {
        if (l.find(part) != npos) {
            return true;
        }
    }
    return false;
}

/// The `nth` (0-based) serial line that begins "MEASURED ".
std::string measured(const Rig& rig, int nth = 0) {
    int k = 0;
    for (const auto& l : rig.io.lines) {
        if (l.rfind("MEASURED ", 0) == 0 && k++ == nth) {
            return l;
        }
    }
    return "";
}

/// A description decoded from bytes (REQUIRE'd to decode).
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

/// A DECLARED port was never asked for, never commanded, never given a brake mode (T3's rule:
/// never geared, never set to Coast, never pulsed) — and the watch read it twice per push.
void checkDeclaredUntouched(Rig& rig, int port) {
    CAPTURE(port);
    CHECK(rig.provider().motorRequestCount(static_cast<std::uint8_t>(port)) == 0);
    CHECK(rig.rec(port).voltageCalls == 0);
    CHECK(rig.rec(port).brakeCalls == 0);
}

/// Every drive motor at 0 V and coasting — the state every exit must leave.
void checkAllStopped(Rig& rig) {
    for (int p = 11; p <= 20; ++p) {
        CAPTURE(p);
        CHECK(rig.rec(p).commandedVoltage().value() == 0.0);
        CHECK(rig.rec(p).brakeMode() == BrakeMode::Coast);
    }
}

/// The `nth` (0-based) serial line that begins with `prefix`, without the prefix; "" if none.
std::string nthAfter(const Rig& rig, const std::string& prefix, int nth) {
    int k = 0;
    for (const auto& l : rig.io.lines) {
        if (l.rfind(prefix, 0) == 0 && k++ == nth) {
            return l.substr(prefix.size());
        }
    }
    return "";
}

/// The record file holds the `nth` run's MEASUREMENT, field by field, against what that run PRINTED
/// before anything was saved — never against the saved bytes themselves (decode-then-encode agrees
/// with ANY bytes that decode): the kind, the cartridge, the IMU port, the source, the provenance,
/// both signed sides, both scales and the width to the printed digits.
void checkRecordIsTheMeasurement(const Rig& rig, int nth) {
    CAPTURE(nth);
    const auto bin = rig.fileBytes(kRecordFileName);
    REQUIRE(bin.size() == shulib::calibration::kRecordBytes);
    const DrivetrainDescription d = decoded(bin);
    CHECK(describe(d) == measured(rig, nth));
    CHECK(d.kind == shulib::hal::DrivetrainKind::Tank);
    CHECK(d.cartridge == MotorCartridge::Blue);
    CHECK(d.imuPort == 2);  // the registry's IMU, as the run's start line says
    CHECK(d.source == shulib::hal::DescriptionSource::RobotMeasured);
    CHECK(std::string{d.provenance}.rfind("shulib Calibrate Oct 18 2026 12:34:56 | ROBOT-MEASURED: ", 0) == 0);
    CHECK(std::string{d.provenance} == nthAfter(rig, "provenance: ", nth));
    // the signed sides, as the run printed them AS MEASURED
    std::string left = nthAfter(rig, "  LEFT  (signed, AS MEASURED):", nth);
    std::string right = nthAfter(rig, "  RIGHT (signed, AS MEASURED):", nth);
    left = left.substr(0, left.find("  ("));
    right = right.substr(0, right.find("  ("));
    CHECK(sidesOf(d) == "LEFT" + left + " | RIGHT" + right);
    // both scales to six places and the width to four, as printed
    char ipr[96];
    std::snprintf(ipr, sizeof ipr, "LEFT %.6f  RIGHT %.6f  (L/R ", d.left.inchesPerRadian.value(),
                  d.right.inchesPerRadian.value());
    CHECK(nthAfter(rig, "  inches per radian: ", nth).rfind(ipr, 0) == 0);
    char width[32];
    std::snprintf(width, sizeof width, "%.4f in ", d.trackWidth.value());
    CHECK(nthAfter(rig, "  effective track width: ", nth).rfind(width, 0) == 0);
    CHECK(rig.fileBytes(kRecordBackupFileName) == bin);
}

/// The index of the first header mark equal to `word` at or after `from`, or npos.
std::size_t markOf(const Rig& rig, const std::string& word, std::size_t from = 0) {
    for (std::size_t i = from; i < rig.marks.size(); ++i) {
        if (rig.marks[i].header == word) {
            return i;
        }
    }
    return npos;
}

const char* const kFrontLedSides = "LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20";
const char* const kOtherEndSides = "LEFT -16 +17 -18 +19 -20 | RIGHT +11 -12 +13 -14 +15";

const char* const kFlip1 = "FRONT FLIPPED -- the card's record calls the end that LED this push its BACK.";
const char* const kFlip2 = "Either the BACK led by mistake (touch RUN AGAIN: nothing is saved), or the FRONT led";
const char* const kFlip3 = "and the drive cables were swapped between the sides, or the front was changed on";
const char* const kFlip4 = "purpose (touch USE NEW: saved as pushed). The records cannot tell these apart.";
const char* const kFlip5 = "USE NEW after a wrong push reverses every auton. NOTHING IS SAVED YET.";

}  // namespace

// ═══ T1. The scenarios ═══════════════════════════════════════════════════════════════

TEST_CASE("CalibrateProgram T1a: one clean run on an empty card -- every decision line verbatim, the "
          "record's bytes, the FIRST RECORD note, the declared claws untouched, every motor stopped") {
    Scenario sc;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;

    // the banner and the card line: both files named, the order said; no card record yet
    CHECK(has(rig, "#  shulib CALIBRATE -- the robot MEASURES ITS OWN DRIVETRAIN"));
    CHECK(has(rig, "#  It POWERS THE MOTORS. The end that LEADS the push is the FRONT."));
    CHECK(has(rig, "BUILD STAMP: Oct 18 2026 12:34:56   <-- must match the build you just uploaded"));
    CHECK(has(rig, "robot      : 2026 TANK CHASSIS (robot two)"));
    CHECK(has(rig, "record file: /usd/shulib_drivetrain.bin (written only after a COMPLETE run, a "
                   "verified backup shulib_drivetrain.bak first)"));
    CHECK(has(rig, "card       : no usable record yet -- this run's push decides the FRONT"));
    // the census, from the registry; the claws dropped by the facts, by name
    CHECK(has(rig, "  PORT  device      (no typed port list is consulted for this)"));
    CHECK(has(rig, "     1  RADIO"));
    CHECK(has(rig, "     2  IMU"));
    CHECK(has(rig, "     6  MOTOR"));
    CHECK(has(rig, "  12 motor(s), 1 IMU(s)"));
    CHECK(has(rig, "port 6: claw pivot 6 (declared) -- not a candidate, not touched"));
    CHECK(has(rig, "port 7: claw pivot 7 (declared) -- not a candidate, not touched"));
    // both declared ports HAVE a motor: the drop's "not found" line is never printed for them
    CHECK(countContaining(L, "the census found NO MOTOR there") == 0);
    CHECK(has(rig, "CARTRIDGE BELIEF: BLUE 600 rpm -- stated by a human and UNVERIFIABLE from inside "
                   "(every cartridge spins the rotor the same)"));
    CHECK(has(rig, "IMU        : port 2, DISCOVERED from the registry (stated 2)"));
    CHECK(has(rig, "calibrator started: 10 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                   "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
    // the config the run uses, verbatim, each value in its own place (every threshold INVENTED,
    // HA-136): a capture is read against the thresholds that produced it
    CHECK(has(rig, "config: push 48 in along a tape; a port MOVED past 5.0 deg of shaft; the push straight within "
                   "3.0 deg and at least 6.28 rad (one wheel turn) of median travel"));
    CHECK(has(rig, "config: pulses ramp 3.0/6.0/9.0 V for 0.40 s each, a side needs 2.0 deg of yaw; a port whose "
                   "own shaft turns 20 deg with nothing else moving is LEFT OUT; rest = motors under 0.05 rad/s AND "
                   "yaw rate under 2.0 deg/s for 0.5 s"));
    CHECK(has(rig, "config: spin 2 turns each way, levels 3.0 then 6.0 V; must turn 5.0 deg within 2.0 s per level; "
                   "STOPS at 15.0 deg the wrong way; 20 s timeout; the two ways agree within 5.0 pct"));
    CHECK(countPrefix(L, "config: ") == 3);
    // the loop's own record at DONE, verbatim: every tick 10 ms apart on the plant (T1q stalls one)
    CHECK(has(rig, "loop: 5496 ticks, worst gap between two ticks 10 ms (a powered step stops past 100 ms); 218 "
                   "screen prints so far"));
    // the push, live: the first tick's count, a line when the count changes (the ports moving; the
    // push far enough, one wheel turn), a keepalive 5 s after the last line while only the travel
    // grows, and READY — each verbatim (the plant is deterministic)
    const std::size_t pushPhase = findLine(L, "phase: PUSH | 0 of 10 moved, drift +0.0 deg, travel 0.0 rad");
    REQUIRE(pushPhase != npos);
    CHECK(L[pushPhase + 1] == "push live: 0 of 10 moved, drift +0.0 deg, travel 0.0 rad");
    const std::size_t ready = findLine(L, "push live: 10 of 10 moved, drift +0.0 deg, travel 29.5 rad -- READY");
    REQUIRE(ready != npos);
    CHECK(L[ready + 1] == "screen: 2/4 PUSH: PUSH FRONT FIRST / 48 IN ALONG TAPE / 10 of 10 turned | [DONE] [ABORT]");
    CHECK(has(rig, "push live: 10 of 10 moved, drift +0.0 deg, travel 0.9 rad"));   // the count changed
    CHECK(has(rig, "push live: 10 of 10 moved, drift +0.0 deg, travel 6.3 rad"));   // far enough: changed
    CHECK(has(rig, "push live: 10 of 10 moved, drift +0.0 deg, travel 24.4 rad"));  // a keepalive
    CHECK(countPrefix(L, "push live: ") == 5);
    // the watch: armed, judged, nothing held
    CHECK(has(rig, "watch      : the declared mechanism ports are READ during the push, never commanded;"));
    CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+0.0 deg)"));
    CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+0.0 deg)"));
    // the front: the push's, never asked
    CHECK(has(rig, "front: the end that LED the push is the FRONT -- the signs stand as captured"));
    CHECK(has(rig, "drive ports, signed: -11 +12 -13 +14 -15 +16 -17 +18 -19 +20  (10 to pulse, one "
                   "at a time)"));
    CHECK(countContaining(L, "WHICH END") == 0);
    CHECK(countContaining(L, "ANSWER FRONT FIRST") == 0);  // the program answered it, unasked
    CHECK(countContaining(L, "[FRONT FIRST]") == 0);       // and never showed the question
    // the result and the check: the FIRST record
    CHECK(has(rig, "  LEFT  (signed, AS MEASURED): -11 +12 -13 +14 -15  (5 ports)"));
    CHECK(has(rig, "  RIGHT (signed, AS MEASURED): +16 -17 +18 -19 +20  (5 ports)"));
    CHECK(has(rig, "  LEFT OUT: none"));
    CHECK(has(rig, "front check: the card holds no record yet -- this is the FIRST: the end that led "
                   "the push is the FRONT"));
    // B1's words, and the follow-ups
    const std::size_t saved = findLine(L, "SAVED -- /usd/shulib_drivetrain.bin: written, RE-READ and "
                                          "compared byte for byte (180 bytes), and re-read at the end.");
    REQUIRE(saved != npos);
    CHECK(L[saved + 1] == "The backup /usd/shulib_drivetrain.bak re-reads identical. The next boot reads "
                         "the record file.");
    CHECK(L[saved + 2] == "COPY THE 'MEASURED tank ...' LINE ABOVE DOWN. Then touch RUN AGAIN and check");
    CHECK(has(rig, "the field disabled the robot after the run had finished -- nothing was powered"));

    // the record's bytes: decodes; the MEASURED line IS the card's record; the backup identical
    const auto bin = rig.fileBytes(kRecordFileName);
    const auto bak = rig.fileBytes(kRecordBackupFileName);
    REQUIRE(bin.size() == 180);
    CHECK(bak == bin);
    const DrivetrainDescription d = decoded(bin);
    CHECK(describe(d) == measured(rig));
    CHECK(measured(rig) == "MEASURED tank BLUE 600 rpm: L5/R5 ports, ipr L1.62500 R1.62500, W 12.500 in");
    CHECK(sidesOf(d) == kFrontLedSides);
    CHECK(d.cartridge == MotorCartridge::Blue);
    CHECK(d.imuPort == 2);
    CHECK(std::string{d.provenance}.rfind("shulib Calibrate Oct 18 2026 12:34:56", 0) == 0);
    CHECK(std::abs(d.left.inchesPerRadian.value() - kTrueIpr) < 0.005 * kTrueIpr);
    CHECK(std::abs(d.right.inchesPerRadian.value() - kTrueIpr) < 0.005 * kTrueIpr);
    CHECK(std::abs(d.trackWidth.value() - kTrack) < 0.01 * kTrack);
    CHECK(encoded(d) == bin);  // nothing the decode cannot carry
    checkRecordIsTheMeasurement(rig, 0);
    int total = 0;
    CHECK(rig.forwardScore(d, total) == 10);
    CHECK(total == 10);

    // the screen: the four steps, never WHICH END; SAVED green; the NOTE
    const std::vector<std::string> want = {"CALIBRATE", "1/4 IMU",  "2/4 PUSH",
                                           "3/4 PULSES", "4/4 SPIN", "SAVED",
                                           "STOPPED -- MOTORS OFF"};
    CHECK(rig.headers() == want);
    // the step that changed under the operator's finger goes up AT ONCE, never on the next tick: the
    // PULSES header is drawn at the DONE tap's own instant, its new boxes greyed from that frame
    const auto* doneTap = rig.tap("DONE");
    const std::size_t pulsesMark = markOf(rig, "3/4 PULSES");
    REQUIRE(doneTap != nullptr);
    REQUIRE(pulsesMark != npos);
    CHECK(rig.marks[pulsesMark].at == doneTap->at);
    CHECK(drew(rig, "print M 8 138 pen=f0f0f0 eraser=101418 'the end that leads the push is FRONT'"));
    CHECK(drew(rig, "print L 8 30 pen=f0f0f0 eraser=101418 'PUSH FRONT FIRST'"));
    CHECK(drew(rig, "print M 8 160 pen=39c36e eraser=101418 'touch DONE -- then stand clear"));
    CHECK(drew(rig, "fill 0 0 479 26 pen=39c36e"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'FIRST RECORD -- the end you pushed first is FRONT'"));
    CHECK_FALSE(drew(rig, "WHICH END"));

    // the claws: never asked for, never commanded; read twice (the push's start and DONE)
    checkDeclaredUntouched(rig, 6);
    checkDeclaredUntouched(rig, 7);
    CHECK(rig.io.watchReads[6] == 2);
    CHECK(rig.io.watchReads[7] == 2);
    for (int p = 11; p <= 20; ++p) {
        CAPTURE(p);
        CHECK(rig.provider().motorRequestCount(static_cast<std::uint8_t>(p)) == 1);
        CHECK(rig.provider().motorCartridge(static_cast<std::uint8_t>(p)) == MotorCartridge::Blue);
        CHECK(rig.provider().motorSignedPort(static_cast<std::uint8_t>(p)) == p);  // POSITIVE
        CHECK(rig.rec(p).nonCoastCalls == 0);  // never Brake, never Hold
    }
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1b: two runs (RUN AGAIN) -- run 2 is checked against run 1's record, SAME "
          "FRONT, and the card ends holding run 2's") {
    Scenario sc;
    sc.runs = 2;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "touch: x=360 y=135 release=2 on RUN AGAIN -> RUN AGAIN"));
    CHECK(has(rig, "===== RUN 2 -- RUN AGAIN: a NEW calibrator, nothing carried over ====="));
    // run 2 is NUMBERED 2 where it ends, and run 1 ended once
    CHECK(has(rig, "===== RUN 2 ENDED: DONE ====="));
    CHECK(countPrefix(rig.io.lines, "===== RUN 1 ENDED") == 1);
    // the new gate is based on the release that ASKED for run 2: that release is never read again
    // (a gate based on the start's baseline would log it a second time, on run 2's first screen)
    CHECK(countContaining(rig.io.lines, " release=2 ") == 1);
    CHECK(has(rig, "touch: x=153 y=207 release=3 on DONE -> ANSWER DONE"));
    CHECK(has(rig, "front check against shulib_drivetrain.bin: 10 drive port(s) in both -- 10 agree, 0 "
                   "flipped, 0 sign only, 0 side only; 0 in the card's only, 0 in this run's only: SAME "
                   "FRONT"));
    CHECK(countPrefix(rig.io.lines, "SAVED -- /usd/shulib_drivetrain.bin") == 2);
    CHECK(countPrefix(rig.io.lines, "front check: the card holds no record yet") == 1);  // run 1 only
    // a run-2 SAME check draws no NOTE (only run 1's FIRST did)
    CHECK(std::count_if(rig.screen.log().begin(), rig.screen.log().end(), [](const std::string& l) {
              return l.find("FIRST RECORD") != npos;
          }) == 1);
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig, 1));
    // the watch re-armed for run 2: two reads per push per claw
    CHECK(rig.io.watchReads[6] == 4);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1c: port 18 slips -- LEFT OUT with the mechanism words, the record without "
          "it") {
    Scenario sc;
    sc.decouple18 = true;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "LEFT OUT: port 18 turns but does not move the robot (a slipping gear? or a MECHANISM "
                   "motor -- declare it in the robot's facts) -- it is in no side and is never commanded "
                   "again"));
    CHECK(findPrefix(rig.io.lines, "  LEFT OUT: 18 -- in no side of the description (") != npos);
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == "LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 -19 +20");
    CHECK(describe(d) == measured(rig));
    // The finished screen: the calibrator's two-row warning keeps ONE row, cut with " ..." (the full
    // text is on serial, above), because the FIRST RECORD note takes the third row.
    CHECK(drew(rig, "print S 8 196 pen=e8b23a eraser=101418 'port 18 turns but does not move the robot -- ...'"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'FIRST RECORD -- the end you pushed first is FRONT'"));
}

TEST_CASE("CalibrateProgram T1d: a curved push is STOPPED and writes nothing; the straight run after it "
          "saves -- and a refused push gets no watch or still-candidate lines") {
    Scenario sc;
    sc.runs = 2;
    sc.curve[0] = 1.6;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t stopped = findPrefix(L, "STOPPED [PUSH_CURVED]: the push curved by ");
    REQUIRE(stopped != npos);
    CHECK(L[stopped + 1] == "Every motor is at 0 V and coasting. Nothing was written to the card by this run.");
    // run 1 wrote nothing: every replace on the card comes after run 2's DONE
    const std::size_t run2 = findLine(L, "===== RUN 2 -- RUN AGAIN: a NEW calibrator, nothing carried over =====");
    REQUIRE(run2 != npos);
    CHECK(rig.marks.size() > 0);
    const std::size_t stoppedMark = markOf(rig, "STOPPED -- MOTORS OFF");
    REQUIRE(stoppedMark != npos);
    CHECK(rig.replacesFrom(0) == rig.replacesFrom(rig.marks[stoppedMark].cardOps));
    CHECK(rig.card.log().size() > 0);
    for (std::size_t i = 0; i < rig.marks[stoppedMark].cardOps; ++i) {
        CHECK(rig.card.log()[i].op == FakeFileOp::Read);
    }
    // the watch judged only the accepted push (run 2)
    CHECK(countPrefix(L, "watch: port 6 (claw pivot 6)") == 1);
    CHECK(countContaining(L, "did not turn with the push: not a drive motor") == 0);
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T1e: no SD card -- said at start, NOT DONE at the check, NOT SAVED, nothing "
          "on the card") {
    Scenario sc;
    sc.card = false;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "card       : NO SD CARD in the brain -- nothing can be saved, and the FRONT is not "
                   "checked"));
    CHECK(countContaining(rig.io.lines, "no usable record") == 0);  // no card is not "no record"
    CHECK(has(rig, "front check: NOT DONE -- no SD card in the brain (nothing can be saved either)"));
    const std::size_t ns = findLine(rig.io.lines, "NOT SAVED -- no SD card in the brain.");
    REQUIRE(ns != npos);
    CHECK(rig.io.lines[ns + 1] == "Nothing on the card was opened or written by this run: whatever it held, "
                                  "it still holds.");
    CHECK(rig.io.lines[ns + 2] == "THE RESULT IS ON THE SCREEN AND IN THIS LOG ONLY. Copy it down NOW.");
    CHECK(rig.fileBytes(kRecordFileName).empty());
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "NOT SAVED") != rig.headers().end());
    CHECK(drew(rig, "fill 0 0 479 26 pen=e2564a"));
    CHECK_FALSE(drew(rig, "FIRST RECORD"));
}

TEST_CASE("CalibrateProgram T1f: no IMU -- refused by name before ANY device is asked for; nothing "
          "powered, nothing written") {
    Scenario sc;
    sc.imu = false;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    CHECK(has(rig, "REFUSED: IMU COUNT NOT 1 -- the registry reports 0 IMUs; this procedure needs exactly "
                   "one"));
    CHECK(has(rig, "  plug in exactly one inertial sensor and run again"));
    CHECK(rig.counting.requests == 0);
    CHECK(drew(rig, "print L 8 34 pen=e2564a eraser=101418 'IMU COUNT NOT 1'"));
    CHECK(drew(rig, "'nothing is powered -- fix it, then restart it'"));
    CHECK(rig.replacesFrom(0) == 0);
    for (int p = 11; p <= 20; ++p) {
        CHECK(rig.rec(p).voltageCalls == 0);
    }
}

TEST_CASE("CalibrateProgram T1g: the field disables mid-pulses -- STOPPED on the tick it is seen, every "
          "motor 0 V + Coast, nothing written") {
    Scenario sc;
    sc.disableAfterPulses = 8.0;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    REQUIRE(rig.disabledAt > 0.0);
    CHECK(rig.returnedAt - rig.disabledAt < kDt);  // the loop exits on the first tick that sees it
    CHECK(has(rig, "===== RUN 1 ENDED: STOPPED ====="));
    CHECK(has(rig, "STOPPED [ABORTED]: the field disabled the robot, or autonomous started"));
    CHECK(has(rig, "competition state changed: status bits 0x01 field=no disabled=yes auton=no -- "
                   "calibration runs ONLY in driver control, enabled"));
    CHECK(has(rig, "STOPPED by the field/competition state -- every motor at 0 V, coast."));
    CHECK(drew(rig, "'NOT DRIVER CONTROL'"));
    CHECK(rig.replacesFrom(0) == 0);
    CHECK(rig.fileBytes(kRecordFileName).empty());
    bool someonePowered = false;
    for (int p = 11; p <= 20; ++p) {
        someonePowered = someonePowered || rig.rec(p).nonZeroCalls > 0;
    }
    CHECK(someonePowered);  // the disable really landed mid-pulses
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1h: UNEQUAL side scales reach the card on their own sides (re-check R14) -- "
          "the MEASURED line, the record and the plant agree") {
    Scenario sc;
    sc.rightScale = kRobotTwoScaleRatio;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(describe(d) == measured(rig));
    CHECK(measured(rig) == "MEASURED tank BLUE 600 rpm: L5/R5 ports, ipr L1.62500 R1.49114, W 12.500 in");
    CHECK(std::abs(d.left.inchesPerRadian.value() - kTrueIpr) < 0.005 * kTrueIpr);
    CHECK(std::abs(d.right.inchesPerRadian.value() - kTrueIpr / kRobotTwoScaleRatio)
          < 0.005 * kTrueIpr);
    CHECK(sidesOf(d) == kFrontLedSides);
}

TEST_CASE("CalibrateProgram T1i: a PROGRAM FAULT inside the loop -- caught at its edge, every motor "
          "stopped, said on the screen and the log, idled until the field disables") {
    Scenario sc;
    sc.faultPort = 11;
    sc.disableAt = 60.0;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(rig.rec(11).thrown);
    CHECK(has(rig, "PROGRAM FAULT (a precondition broke inside the loop): rig: a planted fault on the "
                   "first powered command"));
    CHECK(has(rig, "every motor is at 0 V, coasting. This is a bug in the program, not the robot."));
    CHECK(drew(rig, "'PROGRAM FAULT'"));
    CHECK(rig.returnedAt >= 60.0);  // it idled until the disable, never returned early
    CHECK(rig.returnedAt - rig.disabledAt < 0.26);  // and noticed the disable within one idle wait
    CHECK(rig.replacesFrom(0) == 0);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1j: every touch read at start is an ERROR read -- said, and the first good "
          "read sets the baseline without acting") {
    Scenario sc;
    sc.touchErrorsUntil = 1.0;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "WARNING: every touch read at start was an ERROR read -- the first good read will set "
                   "the baseline and is never acted on"));
    CHECK(has(rig, "touch: x=153 y=207 release=1 on DONE -> ANSWER DONE"));
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T1j-2: touch reads that error for 30 ms at start -- the baseline is taken by a RETRY "
          "(no WARNING), and the first tap acts") {
    // The retry waits 5 ms between reads, 20 tries: 30 ms of errors ends inside its window — so the
    // baseline comes from a retry, never from the loop's first good read (T1j's 1 s of errors does).
    Scenario sc;
    sc.touchErrorsUntil = 0.03;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(countPrefix(rig.io.lines, "WARNING: every touch read at start") == 0);
    CHECK(has(rig, "touch: x=153 y=207 release=1 on DONE -> ANSWER DONE"));
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T1k: two programs alive at once each paint their OWN build time -- the "
          "header's time is the program's, never a buffer every program shares") {
    Scenario sc;
    sc.imu = false;  // each refuses at once, after its starting screen
    Rig a{sc};
    Rig b{sc};
    auto first = a.program(robotTwoFacts(), "Oct 01 2026 11:11:11");
    auto second = b.program(robotTwoFacts(), "Oct 02 2026 22:22:22");
    first.run();
    second.run();
    CHECK(drew(a, "'BUILD 11:11:11'"));
    CHECK_FALSE(drew(a, "22:22:22"));
    CHECK(drew(b, "'BUILD 22:22:22'"));
    CHECK(has(a, "BUILD STAMP: Oct 01 2026 11:11:11   <-- must match the build you just uploaded"));
}

TEST_CASE("CalibrateProgram T1l: 0 V + Coast is the DECLARED starting state -- a drive motor left running and "
          "holding by the last program is stopped and coasting BEFORE the IMU is asked for (its calibration "
          "starts as it is made, and wants the robot still)") {
    Scenario sc;
    sc.disableAt = 1.0;  // the IMU still warming: the run ends before any push
    Rig rig{sc};
    // what the last program left: port 11 driven at 5 V and held, port 16 at -7 V and braked
    rig.rec(11).setVoltage(Voltage{5.0});
    rig.rec(11).setBrakeMode(BrakeMode::Hold);
    rig.rec(16).setVoltage(Voltage{-7.0});
    rig.rec(16).setBrakeMode(BrakeMode::Brake);
    int asked = 0;
    std::vector<double> voltsAtImu;
    std::vector<BrakeMode> brakeAtImu;
    rig.counting.onImu = [&] {
        ++asked;
        for (int p = 11; p <= 20; ++p) {
            voltsAtImu.push_back(rig.rec(p).commandedVoltage().value());
            brakeAtImu.push_back(rig.rec(p).brakeMode());
        }
    };
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    REQUIRE(asked == 1);
    REQUIRE(voltsAtImu.size() == 10);
    for (std::size_t i = 0; i < voltsAtImu.size(); ++i) {
        CAPTURE(11 + i);
        CHECK(voltsAtImu[i] == 0.0);
        CHECK(brakeAtImu[i] == BrakeMode::Coast);
    }
    CHECK(has(rig, "STOPPED [ABORTED]: the field disabled the robot, or autonomous started"));
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1m: AUTONOMOUS ends driver control as a disable does -- mid-pulses with or without "
          "the 'connected' bit, and a field's disable WITH it: STOPPED on the tick it is seen, every motor 0 V + "
          "Coast, nothing written, the status bits said") {
    struct EndCase {
        std::uint8_t status;
        const char* bits;
    };
    const EndCase ends[] = {
        {0x02, "status bits 0x02 field=no disabled=no auton=yes"},   // autonomous, no field
        {0x06, "status bits 0x06 field=yes disabled=no auton=yes"},  // autonomous on a field/switch
        {0x05, "status bits 0x05 field=yes disabled=yes auton=no"},  // a field's disable
    };
    for (const EndCase& e : ends) {
        CAPTURE(static_cast<int>(e.status));
        Scenario sc;
        sc.disableAfterPulses = 8.0;
        sc.endStatus = e.status;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        REQUIRE(rig.disabledAt > 0.0);
        CHECK(rig.returnedAt - rig.disabledAt < kDt);  // the loop exits on the first tick that sees it
        CHECK(has(rig, "===== RUN 1 ENDED: STOPPED ====="));
        CHECK(has(rig, "STOPPED [ABORTED]: the field disabled the robot, or autonomous started"));
        CHECK(has(rig, std::string{"competition state changed: "} + e.bits
                           + " -- calibration runs ONLY in driver control, enabled"));
        CHECK(has(rig, "STOPPED by the field/competition state -- every motor at 0 V, coast."));
        CHECK(rig.replacesFrom(0) == 0);
        CHECK(rig.fileBytes(kRecordFileName).empty());
        bool someonePowered = false;
        for (int p = 11; p <= 20; ++p) {
            someonePowered = someonePowered || rig.rec(p).nonZeroCalls > 0;
        }
        CHECK(someonePowered);  // it really landed mid-pulses
        checkAllStopped(rig);
    }
}

TEST_CASE("CalibrateProgram T1n: autonomous after a run finished, and during a PROGRAM FAULT's idle -- the words "
          "say AUTONOMOUS (never a disable that did not happen), and the fault's idle returns as the loop does") {
    SUBCASE("finished: 'autonomous started after the run had finished'") {
        Scenario sc;
        sc.endStatus = 0x02;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "autonomous started after the run had finished -- nothing was powered"));
        CHECK(countContaining(rig.io.lines, "the field disabled the robot after") == 0);
        CHECK(has(rig, "competition state changed: status bits 0x02 field=no disabled=no auton=yes -- calibration "
                       "runs ONLY in driver control, enabled"));
    }
    SUBCASE("a PROGRAM FAULT idles until autonomous starts, then returns within one idle wait") {
        Scenario sc;
        sc.faultPort = 11;
        sc.disableAt = 60.0;
        sc.endStatus = 0x06;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(rig.rec(11).thrown);
        CHECK(has(rig, "PROGRAM FAULT (a precondition broke inside the loop): rig: a planted fault on the first "
                       "powered command"));
        CHECK(rig.returnedAt >= 60.0);
        CHECK(rig.returnedAt - rig.disabledAt < 0.26);
        checkAllStopped(rig);
    }
}

TEST_CASE("CalibrateProgram T1o: a field or switch CONNECTED from the start (status 0x04, enabled in driver "
          "control) -- the 'connected' bit alone never ends driver control: the run goes to SAVED, and a PROGRAM "
          "FAULT's idle waits for the field's disable (0x05)") {
    // Every other scenario starts from 0x00, so a program that ended driver control on ANY status bit
    // would pass them all — and stop every run at its first tick on a field or a competition switch.
    SUBCASE("a whole run under 0x04: SAVED; the field's disable (0x05) ends it") {
        Scenario sc;
        sc.startStatus = 0x04;
        sc.endStatus = 0x05;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "opcontrol entry; competition status 0x04"));
        CHECK(has(rig, "===== RUN 1 ENDED: DONE ====="));
        CHECK(findPrefix(rig.io.lines, "SAVED -- /usd/shulib_drivetrain.bin") != npos);
        CHECK(countPrefix(rig.io.lines, "STOPPED [") == 0);
        CHECK(has(rig, "the field disabled the robot after the run had finished -- nothing was powered"));
        CHECK(has(rig, "competition state changed: status bits 0x05 field=yes disabled=yes auton=no -- calibration "
                       "runs ONLY in driver control, enabled"));
        const std::vector<std::string> want = {"CALIBRATE", "1/4 IMU",  "2/4 PUSH",
                                               "3/4 PULSES", "4/4 SPIN", "SAVED",
                                               "STOPPED -- MOTORS OFF"};
        CHECK(rig.headers() == want);
        checkRecordIsTheMeasurement(rig, 0);
        CHECK(rig.returnedAt - rig.disabledAt < kDt);
        checkAllStopped(rig);
    }
    SUBCASE("a PROGRAM FAULT under 0x04: the idle does not end on the connected bit; it returns on 0x05") {
        Scenario sc;
        sc.startStatus = 0x04;
        sc.endStatus = 0x05;
        sc.faultPort = 11;
        sc.disableAt = 60.0;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(rig.rec(11).thrown);
        CHECK(findPrefix(rig.io.lines, "PROGRAM FAULT (a precondition broke inside the loop)") != npos);
        CHECK(rig.returnedAt >= 60.0);  // it idled under 0x04 until the disable, never returned early
        CHECK(rig.returnedAt - rig.disabledAt < 0.26);
        checkAllStopped(rig);
    }
}

TEST_CASE("CalibrateProgram T1p: a PROGRAM FAULT while the motors are POWERED (mid-spin) -- the catch stops "
          "every drive motor at 0 V + Coast BEFORE the fault's idle begins, not only when run() returns") {
    // T1i's fault fires on a motor's FIRST powered command, when no other motor is powered, and its
    // checks run after run() returned (the RAII guard has run by then): neither could see a catch
    // block that idled over a running robot. Here the fault fires mid-spin, and the world is read at
    // the idle's FIRST 250 ms wait.
    Scenario sc;
    sc.faultPortInSpin = 20;  // the last the spin commands: nine powered when it throws
    sc.disableAt = 200.0;
    Rig rig{sc};
    int poweredAtFault = -1;
    rig.rec(20).beforeThrow = [&] {
        poweredAtFault = 0;
        for (int p = 11; p <= 19; ++p) {
            poweredAtFault += rig.rec(p).commandedVoltage().value() != 0.0 ? 1 : 0;
        }
    };
    bool snapped = false;
    std::vector<double> voltsAtIdle;
    std::vector<BrakeMode> brakeAtIdle;
    rig.io.onDelay = [&](std::uint32_t ms) {
        if (ms == 250 && !snapped) {  // the fault's idle: its first wait
            snapped = true;
            for (int p = 11; p <= 20; ++p) {
                voltsAtIdle.push_back(rig.rec(p).commandedVoltage().value());
                brakeAtIdle.push_back(rig.rec(p).brakeMode());
            }
        }
    };
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    REQUIRE(rig.rec(20).thrown);
    CHECK(poweredAtFault == 9);  // the premise: the robot WAS moving when the fault fired
    CHECK(has(rig, "PROGRAM FAULT (a precondition broke inside the loop): rig: a planted fault on the first "
                   "powered command"));
    CHECK(has(rig, "every motor is at 0 V, coasting. This is a bug in the program, not the robot."));
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "4/4 SPIN") != rig.headers().end());
    REQUIRE(snapped);
    REQUIRE(voltsAtIdle.size() == 10);
    for (std::size_t i = 0; i < voltsAtIdle.size(); ++i) {
        CAPTURE(11 + i);
        CHECK(voltsAtIdle[i] == 0.0);
        CHECK(brakeAtIdle[i] == BrakeMode::Coast);
    }
    CHECK(rig.returnedAt >= 200.0);  // it idled until the disable
    CHECK(rig.replacesFrom(0) == 0);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T1q: one slow tick (50 ms) -- the loop's worst gap says it, verbatim") {
    Scenario sc;
    sc.stallAt = 3.0;  // in the push: the gap is measured while the run is under way
    sc.stallSteps = 4;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "loop: 5492 ticks, worst gap between two ticks 50 ms (a powered step stops past 100 ms); 218 "
                   "screen prints so far"));
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T1r: a tap on a LIVE DONE that the calibrator no longer accepts when the touch is read "
          "(the robot moved between the paint and the read) -- logged as such, does nothing; the next tap acts") {
    Scenario sc;
    Rig rig{sc};
    bool jostled = false;
    rig.seam.onTouchRead = [&] {
        if (!jostled && rig.tap("DONE") != nullptr) {
            jostled = true;
            rig.jostle(12, 20.0);  // a drive member turning: pushReady() is false at this read
        }
    };
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    REQUIRE(jostled);
    const std::size_t first = findLine(rig.io.lines, "touch: x=153 y=207 release=1 on DONE -> ignored -- the "
                                                     "calibrator no longer accepts it");
    const std::size_t second = findLine(rig.io.lines, "touch: x=153 y=207 release=2 on DONE -> ANSWER DONE");
    REQUIRE(first != npos);
    REQUIRE(second != npos);
    CHECK(first < second);
    CHECK(countContaining(rig.io.lines, "-> ANSWER DONE") == 1);
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T1s: the push-live throttle's two edges, landed on EXACTLY (0.25 s and 5 s are exact "
          "in binary): a changed count at the minimum is due, a hair under is not; nothing changed at the "
          "keepalive is due, a hair under is not") {
    using shulib::calibration::CalibrateProgram;
    using shulib::calibration::detail::programPushLogDue;
    constexpr double kMin = CalibrateProgram::kPushLogMinSeconds;
    constexpr double kKeep = CalibrateProgram::kPushLogKeepaliveSeconds;
    static_assert(kMin == 0.25 && kKeep == 5.0, "the edges below are these two numbers");
    CHECK(programPushLogDue(true, 0.25, kMin, kKeep));
    CHECK_FALSE(programPushLogDue(true, std::nextafter(0.25, 0.0), kMin, kKeep));
    CHECK(programPushLogDue(false, 5.0, kMin, kKeep));
    CHECK_FALSE(programPushLogDue(false, std::nextafter(5.0, 0.0), kMin, kKeep));
    CHECK_FALSE(programPushLogDue(false, 0.25, kMin, kKeep));  // nothing changed: the minimum is no reason
    CHECK(programPushLogDue(true, 5.0, kMin, kKeep));
    CHECK(programPushLogDue(false, 1.0e9, kMin, kKeep));  // the first line (the last at -1e9)
}

TEST_CASE("CalibrateProgram T1t: the PROGRAM's own push-live lines land on both edges EXACTLY -- on the brain's "
          "clock formula, hands off for 5 s: the keepalive at exactly 5 s, then a count that changed inside the "
          "minimum held until exactly 0.25 s") {
    // T1s pins programPushLogDue's edges; this pins that the program's lines keep them. On the
    // plant's clock (which ADDS 0.01 a step) no gap between two lines is ever exactly 0.25 s or
    // 5 s, so a program whose edges were strict printed the same lines (the third verification
    // round's W13d / W13e). Here the program reads ProsClock's formula (steps x 1e-6, exact at
    // every quarter second), the push begins at exactly 2 s, and the operator waits 5 s before
    // pushing — so the keepalive's gap is exactly 5 s and the change's exactly 0.25 s.
    using shulib::calibration::CalibrateProgram;
    static_assert(CalibrateProgram::kPushLogMinSeconds == 0.25
                      && CalibrateProgram::kPushLogKeepaliveSeconds == 5.0,
                  "the instants below are these two intervals");
    Scenario sc;
    sc.exactClockOffsetUs = 0;
    sc.pushWaitSec = 5.0;
    Rig rig{sc};
    double shownAt = -1.0;  // the program's clock at the first tick the panel showed 10 of 10
    std::size_t seen = 0;
    rig.seam.onTouchRead = [&] {  // each tick, after the paint, before the clock moves
        const auto& log = rig.screen.log();
        for (; seen < log.size(); ++seen) {
            if (shownAt < 0.0 && log[seen].find("10 of 10 turned") != std::string::npos) {
                shownAt = rig.programNow();
            }
        }
    };
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    REQUIRE(rig.io.lineAt.size() == L.size());
    std::vector<std::size_t> live;
    for (std::size_t i = 0; i < L.size(); ++i) {
        if (L[i].rfind("push live: ", 0) == 0) {
            live.push_back(i);
        }
    }
    REQUIRE(live.size() == 6);
    // the push's first tick, at exactly 2 s (the premise every instant below stands on)
    CHECK(L[live[0]] == "push live: 0 of 10 moved, drift +0.0 deg, travel 0.0 rad");
    REQUIRE(rig.io.lineAt[live[0]] == 2.0);
    // nothing changed (hands off): the keepalive at EXACTLY 5 s — not one tick later
    CHECK(L[live[1]] == "push live: 0 of 10 moved, drift +0.0 deg, travel 0.0 rad");
    CHECK(rig.io.lineAt[live[1]] == 7.0);
    // the count changed INSIDE the minimum (the premise: shown at 7.03, after the keepalive and
    // before 0.25 s had passed) ...
    REQUIRE(shownAt > 7.0);
    REQUIRE(shownAt < 7.25);
    // ... so its line waits for EXACTLY 0.25 s after the keepalive — not a tick before, not one after
    CHECK(L[live[2]] == "push live: 10 of 10 moved, drift +0.0 deg, travel 0.9 rad");
    CHECK(rig.io.lineAt[live[2]] == 7.25);
    // the rest of the push as T1a's, 5 s later: far enough (changed), a keepalive, READY
    CHECK(L[live[3]] == "push live: 10 of 10 moved, drift +0.0 deg, travel 6.3 rad");
    CHECK(L[live[4]] == "push live: 10 of 10 moved, drift +0.0 deg, travel 24.4 rad");
    CHECK(L[live[5]] == "push live: 10 of 10 moved, drift +0.0 deg, travel 29.5 rad -- READY");
    CHECK(has(rig, "===== RUN 1 ENDED: DONE ====="));
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
    checkAllStopped(rig);
}

// ═══ T2. The front holds (the untyped half) ══════════════════════════════════════════

TEST_CASE("CalibrateProgram T2a: the other end leads run 2 -- HELD on FRONT FLIPPED with NOTHING written; "
          "RUN AGAIN discards it; a front-led run 3 is SAME and saves") {
    Scenario sc;
    sc.runs = 3;
    sc.pushDir[1] = -1.0;
    sc.heldFlip[1] = kRunAgain;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t check = findLine(
        L, "front check against shulib_drivetrain.bin: 10 drive port(s) in both -- 0 agree, 10 flipped, "
           "0 sign only, 0 side only; 0 in the card's only, 0 in this run's only: FRONT FLIPPED");
    REQUIRE(check != npos);
    CHECK(L[check + 1] == kFlip1);
    CHECK(L[check + 2] == kFlip2);
    CHECK(L[check + 3] == kFlip3);
    CHECK(L[check + 4] == kFlip4);
    CHECK(L[check + 5] == kFlip5);
    CHECK(has(rig, "screen: FRONT FLIPPED: NOTHING SAVED YET / the card calls the end you led BACK / RUN "
                   "AGAIN: you led with the BACK by mistake | [USE NEW] [RUN AGAIN]"));
    CHECK(has(rig, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card keeps its "
                   "record"));
    // the held screen, drawn: its words, its two boxes, amber
    CHECK(drew(rig, "print L 8 30 pen=e8b23a eraser=101418 'NOTHING SAVED YET'"));
    CHECK(drew(rig, "print S 8 196 pen=f0f0f0 eraser=101418 'USE NEW: you led with the FRONT (cables swapped?)'"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'USE NEW after a wrong push reverses every auton'"));
    // NOTHING written while held: no replace between FRONT FLIPPED going up and run 3's first screen
    const std::size_t held = markOf(rig, "FRONT FLIPPED");
    REQUIRE(held != npos);
    REQUIRE(held + 1 < rig.marks.size());
    CHECK(rig.replacesFrom(rig.marks[held].cardOps) == rig.replacesFrom(rig.marks[held + 1].cardOps));
    // run 3: SAME against run 1's record, saved, and it drives forward
    CHECK(countContaining(L, "10 agree, 0 flipped") == 1);
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == kFrontLedSides);
    CHECK(describe(d) == measured(rig, 2));
    int total = 0;
    CHECK(rig.forwardScore(d, total) == 10);
}

TEST_CASE("CalibrateProgram T2b: USE NEW saves the run AS PUSHED -- the record is the measurement, the "
          "header says NEW FRONT, and after a wrong push it drives 0 of 10 forward (the warned-of case)") {
    Scenario sc;
    sc.runs = 2;
    sc.pushDir[1] = -1.0;
    sc.heldFlip[1] = kLeftBox;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "touch: x=120 y=115 release=4 on USE NEW -> SAVE AS PUSHED"));
    CHECK(has(rig, "front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)"));
    CHECK(has(rig, "  LEFT  (signed, AS MEASURED): -16 +17 -18 +19 -20  (5 ports)"));
    const auto bin = rig.fileBytes(kRecordFileName);
    const DrivetrainDescription d = decoded(bin);
    CHECK(describe(d) == measured(rig, 1));  // run 2's measurement, as pushed
    CHECK(sidesOf(d) == kOtherEndSides);
    CHECK(encoded(d) == bin);
    CHECK(rig.fileBytes(kRecordBackupFileName) == bin);
    checkRecordIsTheMeasurement(rig, 1);  // every field, against what run 2 printed before the hold
    int total = 0;
    CHECK(rig.forwardScore(d, total) == 0);
    CHECK(total == 10);
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED -- NEW FRONT") != rig.headers().end());
    CHECK(drew(rig, "fill 0 0 479 26 pen=e8b23a"));  // amber, not green
    // ... and it is THIS header's amber: the header word is printed on its own band's colour (the
    // fill alone is satisfied by the held FRONT FLIPPED screen's amber band earlier in the run)
    CHECK(drew(rig, "print M 8 4 pen=101418 eraser=e8b23a 'SAVED -- NEW FRONT"));
    CHECK_FALSE(drew(rig, "eraser=39c36e 'SAVED -- NEW FRONT"));
}

TEST_CASE("CalibrateProgram T2c: the field disables on the held FRONT FLIPPED screen -- said, and the card "
          "keeps run 1's record byte for byte") {
    Scenario sc;
    sc.runs = 2;
    sc.pushDir[1] = -1.0;
    sc.heldFlip[1] = kDisable;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "the field disabled the robot on the held FRONT FLIPPED screen -- NOTHING SAVED: the card "
                   "keeps its record"));
    const auto bin = rig.fileBytes(kRecordFileName);
    CHECK(describe(decoded(bin)) == measured(rig, 0));  // run 1's
    CHECK(rig.fileBytes(kRecordBackupFileName) == bin);
    const std::size_t held = markOf(rig, "FRONT FLIPPED");
    REQUIRE(held != npos);
    CHECK(rig.replacesFrom(rig.marks[held].cardOps) == 0);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T2k: AUTONOMOUS starts on the held FRONT FLIPPED screen -- said as autonomous (never "
          "'the field disabled'), nothing written, the card byte for byte as before") {
    Scenario sc;
    sc.runs = 2;
    sc.pushDir[1] = -1.0;
    sc.heldFlip[1] = kDisable;  // the end of driver control lands on the held screen ...
    sc.endStatus = 0x06;        // ... as autonomous, on a field
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "autonomous started on the held FRONT FLIPPED screen -- NOTHING SAVED: the card keeps its record"));
    CHECK(countContaining(rig.io.lines, "the field disabled the robot on the held") == 0);
    CHECK(has(rig, "competition state changed: status bits 0x06 field=yes disabled=no auton=yes -- calibration runs "
                   "ONLY in driver control, enabled"));
    const std::size_t held = markOf(rig, "FRONT FLIPPED");
    REQUIRE(held != npos);
    CHECK(rig.replacesFrom(rig.marks[held].cardOps) == 0);
    const auto bin = rig.fileBytes(kRecordFileName);
    CHECK(describe(decoded(bin)) == measured(rig, 0));  // run 1's, kept
    CHECK(rig.fileBytes(kRecordBackupFileName) == bin);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T2d: the LEFT and RIGHT cable bundles swapped, the front leading -- FRONT "
          "FLIPPED 10 of 10 (the records cannot tell it from a flip); USE NEW drives 10 of 10 forward") {
    Scenario sc;
    sc.swapBundles = true;
    sc.heldFlip[0] = kLeftBox;
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    // the start's card line names the file the record came from: the RECORD file (T2i: the backup)
    CHECK(has(rig, "card       : shulib_drivetrain.bin holds MEASURED tank BLUE 600 rpm: L5/R5 ports, ipr L1.62500 "
                   "R1.62500, W 12.500 in -- this run's FRONT is checked against it"));
    CHECK(has(rig, "front check against shulib_drivetrain.bin: 10 drive port(s) in both -- 0 agree, 10 "
                   "flipped, 0 sign only, 0 side only; 0 in the card's only, 0 in this run's only: FRONT "
                   "FLIPPED"));
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    // the same signed record as the other end leading (T2b): only the operator knows which happened
    CHECK(sidesOf(d) == kOtherEndSides);
    int total = 0;
    CHECK(rig.forwardScore(d, total) == 10);  // USE NEW is RIGHT here
    CHECK(total == 10);
}

TEST_CASE("CalibrateProgram T2j: a held run discarded by RUN AGAIN leaves NOTHING held behind -- the next "
          "run STOPPED, the field disables: no 'held' words, since nothing is held") {
    Scenario sc;
    sc.runs = 3;
    sc.pushDir[1] = -1.0;
    sc.heldFlip[1] = kRunAgain;
    sc.curve[2] = 1.6;  // run 3 curves: STOPPED, nothing to save
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(countPrefix(L, "RUN AGAIN on a held screen") == 1);  // run 2's, and only it
    CHECK(findPrefix(L, "STOPPED [PUSH_CURVED]") != npos);
    CHECK(has(rig, "the field disabled the robot after the run had finished -- nothing was powered"));
    CHECK(countContaining(L, "on the held") == 0);
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig, 0));  // run 1's, kept
}

TEST_CASE("CalibrateProgram T2e: files that do not decode -- NOT DONE, one line per file naming it, the "
          "FRONT NOT CHECKED note, saved over them") {
    Scenario sc;
    Rig rig{sc};
    rig.card.setFile(kRecordFileName, garbage(180));
    rig.card.setFile(kRecordBackupFileName, garbage(180));
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(has(rig, "card       : no usable record yet -- this run's push decides the FRONT"));
    const std::size_t nd = findLine(L, "front check: NOT DONE -- the card's record does not decode; the end "
                                       "that led the push is the FRONT -- check the robot drives the right "
                                       "way before an auton");
    REQUIRE(nd != npos);
    CHECK(L[nd + 1] == "  shulib_drivetrain.bin: magic is not \"SHDC\" -- this is not a calibration record");
    CHECK(L[nd + 2] == "  shulib_drivetrain.bak: magic is not \"SHDC\" -- this is not a calibration record");
    CHECK(L[nd + 3].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);
    CHECK(drew(rig, "'FRONT NOT CHECKED -- see the log'"));
    CHECK(describe(decoded(rig.fileBytes(kRecordFileName))) == measured(rig));
}

TEST_CASE("CalibrateProgram T2f: ONLY a backup, and it does not decode -- NOT DONE (never 'the FIRST'), its "
          "own line, never an empty reason (the check of D6 (a), C3)") {
    Scenario sc;
    Rig rig{sc};
    rig.card.setFile(kRecordBackupFileName, garbage(180));
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(countContaining(L, "this is the FIRST") == 0);
    const std::size_t nd = findPrefix(L, "front check: NOT DONE -- the card's record does not decode");
    REQUIRE(nd != npos);
    CHECK(L[nd + 1] == "  shulib_drivetrain.bak: magic is not \"SHDC\" -- this is not a calibration record");
    CHECK(L[nd + 2].rfind("SAVED -- ", 0) == 0);  // no line for the record file: it never opened
    CHECK(countContaining(L, "()") == 0);
    CHECK(drew(rig, "'FRONT NOT CHECKED -- see the log'"));
    CHECK_FALSE(drew(rig, "FIRST RECORD"));
}

TEST_CASE("CalibrateProgram T2g: an EMPTY record file (a crash mid-write) -- NOT DONE and 'EMPTY (0 bytes "
          "read)', never the decoder's empty reason") {
    Scenario sc;
    Rig rig{sc};
    rig.card.setFile(kRecordFileName, {});
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t nd = findPrefix(L, "front check: NOT DONE -- the card's record does not decode");
    REQUIRE(nd != npos);
    CHECK(L[nd + 1] == "  shulib_drivetrain.bin: EMPTY (0 bytes read)");
    CHECK(L[nd + 2].rfind("SAVED -- ", 0) == 0);
}

TEST_CASE("CalibrateProgram T2h: a card holding ANOTHER robot's record -- CANNOT TELL, saved as measured, "
          "the FRONT NOT CHECKED note") {
    DrivetrainDescription other = decoded(frontLedRecord());
    other.left.count = 2;
    other.left.signedPorts[0] = -3;
    other.left.signedPorts[1] = 4;
    other.right.count = 2;
    other.right.signedPorts[0] = 5;
    other.right.signedPorts[1] = -8;
    REQUIRE(other.valid());
    Scenario sc;
    Rig rig{sc};
    seedBoth(rig, encoded(other));
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "front check against shulib_drivetrain.bin: 0 drive port(s) in both -- 0 agree, 0 "
                   "flipped, 0 sign only, 0 side only; 4 in the card's only, 10 in this run's only: CANNOT "
                   "TELL"));
    CHECK(has(rig, "front check: CANNOT TELL (no drive port in common, or no majority) -- saved as "
                   "measured; check the robot drives the right way before an auton"));
    CHECK(drew(rig, "'FRONT NOT CHECKED -- see the log'"));
    CHECK(sidesOf(decoded(rig.fileBytes(kRecordFileName))) == kFrontLedSides);
}

TEST_CASE("CalibrateProgram T2i: a damaged record file with a good backup -- the check reads the BACKUP, "
          "as the boot's load rule does, and names it") {
    Scenario sc;
    Rig rig{sc};
    rig.card.setFile(kRecordFileName, garbage(180));
    rig.card.setFile(kRecordBackupFileName, frontLedRecord());
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(findPrefix(rig.io.lines, "card       : shulib_drivetrain.bak holds MEASURED tank BLUE 600 rpm") != npos);
    CHECK(has(rig, "front check against shulib_drivetrain.bak: 10 drive port(s) in both -- 10 agree, 0 "
                   "flipped, 0 sign only, 0 side only; 0 in the card's only, 0 in this run's only: SAME "
                   "FRONT"));
}

// ═══ T4. The save is SAFE-SAVE B1's, under every planted fault ═══════════════════════

namespace {

/// The index of the `nth` operation of kind `op` on `file` in a card's log, or npos.
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

struct SaveCase {
    const char* name;
    std::vector<std::byte> bin;  ///< empty = no record file
    std::vector<std::byte> bak;  ///< empty = no backup
    FakeFileFault fault;         ///< planted on the target operation (None = no plan)
    FakeFileOp targetOp;
    const char* targetFile;
    std::size_t amount;
    const char* expectHeader;    ///< the finished screen's header word
    const char* expectLine;      ///< one line formatSaveReport() must have said
};

void seed(FakeFileStore& card, const SaveCase& c) {
    if (!c.bin.empty()) {
        card.setFile(kRecordFileName, c.bin);
    }
    if (!c.bak.empty()) {
        card.setFile(kRecordBackupFileName, c.bak);
    }
}

}  // namespace

TEST_CASE("CalibrateProgram T4: the save -- the program's card operations are EXACTLY two loadRecord() "
          "reads and saveRecordSafely(), its words EXACTLY formatSaveReport(No)'s, and the screen says "
          "SAVED iff the save measured it -- under every planted card fault") {
    // Records that DIFFER from what the run will measure (a seed equal to the new bytes would make
    // every failed write "save" them — the safe save classifies the file as holding the new bytes):
    // the front-led record with its track width changed, and a third with its scale changed.
    DrivetrainDescription olderD = decoded(frontLedRecord());
    olderD.trackWidth = shulib::units::Length{12.4};
    const std::vector<std::byte> older = encoded(olderD);
    DrivetrainDescription oldestD = decoded(frontLedRecord());
    oldestD.left.inchesPerRadian = shulib::units::Length{1.6};
    const std::vector<std::byte> oldest = encoded(oldestD);
    REQUIRE(older != frontLedRecord());
    REQUIRE(oldest != older);

    const std::vector<SaveCase> cases = {
        {"empty card, no fault", {}, {}, FakeFileFault::None, FakeFileOp::Replace, kRecordFileName, 0,
         "SAVED", "The backup /usd/shulib_drivetrain.bak re-reads identical. The next boot reads the record file."},
        {"a short write of the record file: restored", older, older, FakeFileFault::ShortWrite,
         FakeFileOp::Replace, kRecordFileName, 50, "NOT SAVED",
         "The earlier record was written back to the record file and re-read: it is still on the card, and "
         "the next boot uses it as before."},
        {"a false flush that kept every byte: SAVED", older, older, FakeFileFault::FlushFalse,
         FakeFileOp::Replace, kRecordFileName, 0, "SAVED",
         "(The card reported the flush of /usd/shulib_drivetrain.bin failing, yet it reads back whole.)"},
        {"a silent flip in the record file: restored", older, older, FakeFileFault::SilentFlip,
         FakeFileOp::Replace, kRecordFileName, 17, "NOT SAVED",
         "The earlier record was written back to the record file and re-read: it is still on the card, and "
         "the next boot uses it as before."},
        {"a crash at the record file's write, the card gone: UNCONFIRMED", older, older, FakeFileFault::Crash,
         FakeFileOp::Replace, kRecordFileName, 90, "NOT SAVED",
         "SAVE UNCONFIRMED -- the SD card is no longer in the brain."},
        {"the backup-first put fails: the record file never opened for writing", older, oldest,
         FakeFileFault::ShortWrite, FakeFileOp::Replace, kRecordBackupFileName, 30, "NOT SAVED",
         "so the record file was never opened for writing."},
        {"a damaged record file, a good backup, a short write: the boot does NOT read the backup", garbage(180),
         older, FakeFileFault::ShortWrite, FakeFileOp::Replace, kRecordFileName, 50, "NOT SAVED",
         "The drive program in this build does not read the backup: the next boot will REFUSE. Rerun shulib "
         "Calibrate, or on a laptop copy shulib_drivetrain.bak over shulib_drivetrain.bin."},
    };

    for (const SaveCase& c : cases) {
        CAPTURE(std::string{c.name});
        // 1. the dry run: where the target operation falls, and what the program asks to save
        std::size_t target = npos;
        std::vector<std::byte> newBytes;
        {
            Scenario sc;
            Rig dry{sc};
            seed(dry.card, c);
            dry.run(robotTwoFacts());
            REQUIRE_FALSE(dry.timedOut);
            const std::size_t firstP = opIndex(dry.card, FakeFileOp::Replace, kRecordFileName);
            REQUIRE(firstP != npos);
            newBytes = dry.card.log()[firstP].bytes;
            target = opIndex(dry.card, c.targetOp, c.targetFile);
            REQUIRE(target != npos);
        }
        // 2. the planted run
        Scenario sc;
        Rig rig{sc};
        seed(rig.card, c);
        FakeFileFaultPlan plan{};
        plan.atOperation = target;
        plan.fault = c.fault;
        plan.amount = c.amount;
        plan.leave = shulib::hal::fake::CrashLeave::Prefix;
        plan.after = shulib::hal::fake::AfterCrash::CardGone;
        if (c.fault != FakeFileFault::None) {
            rig.card.plan(plan);
        }
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        // 3. the replay: two loads, then the library's own save of the same bytes, on the same card
        FakeFileStore replay{"/usd/"};
        seed(replay, c);
        if (c.fault != FakeFileFault::None) {
            replay.plan(plan);
        }
        (void)shulib::calibration::loadRecord(replay);
        (void)shulib::calibration::loadRecord(replay);
        const auto report = shulib::calibration::saveRecordSafely(replay, decoded(newBytes));
        std::vector<std::string> words;
        shulib::calibration::formatSaveReport(report, shulib::calibration::BootReadsBackup::No,
                                              [&words](const char* l) { words.emplace_back(l); });
        // the card saw exactly the replay's operations, byte for byte
        const auto& a = rig.card.log();
        const auto& b = replay.log();
        REQUIRE(a.size() == b.size());
        for (std::size_t i = 0; i < a.size(); ++i) {
            CAPTURE(i);
            CHECK(a[i].op == b[i].op);
            CHECK(a[i].file == b[i].file);
            CHECK(a[i].bytes == b[i].bytes);
            CHECK(a[i].opened == b[i].opened);
            CHECK(a[i].count == b[i].count);
            CHECK(a[i].step == b[i].step);
            CHECK(a[i].faulted == b[i].faulted);
        }
        CHECK(rig.card.file(kRecordFileName) == replay.file(kRecordFileName));
        CHECK(rig.card.file(kRecordBackupFileName) == replay.file(kRecordBackupFileName));
        // the program said exactly the save's words, in order, together
        REQUIRE_FALSE(words.empty());
        const std::size_t at = findLine(rig.io.lines, words[0]);
        REQUIRE(at != npos);
        REQUIRE(at + words.size() < rig.io.lines.size());
        for (std::size_t k = 0; k < words.size(); ++k) {
            CHECK(rig.io.lines[at + k] == words[k]);
        }
        CHECK(findLine(words, c.expectLine) != npos);
        const bool saved = report.outcome == shulib::calibration::SaveResult::Saved;
        CHECK(rig.io.lines[at + words.size()]
              == (saved ? "COPY THE 'MEASURED tank ...' LINE ABOVE DOWN. Then touch RUN AGAIN and check"
                        : "THE RESULT IS ON THE SCREEN AND IN THIS LOG ONLY. Copy it down NOW."));
        // the panel: SAVED iff the save measured it
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), std::string{c.expectHeader})
              != rig.headers().end());
        CHECK((std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end()) == saved);
    }
}

TEST_CASE("CalibrateProgram T4b: a source pin -- ONE writer of the record (saveRecordSafely), no block "
          "sink or source of its own, no stdio printer, no PROS") {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/include/shulib/calibration/calibrate_program.hpp"};
    REQUIRE(in.good());
    std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    // code only: drop // comments
    std::string code;
    for (std::size_t i = 0; i < src.size(); ++i) {
        if (src[i] == '/' && i + 1 < src.size() && src[i + 1] == '/') {
            while (i < src.size() && src[i] != '\n') {
                ++i;
            }
        }
        if (i < src.size()) {
            code += src[i];
        }
    }
    auto count = [&code](const std::string& s) {
        int n = 0;
        for (std::size_t at = code.find(s); at != npos; at = code.find(s, at + 1)) {
            ++n;
        }
        return n;
    };
    CHECK(count("saveRecordSafely(") == 1);
    CHECK(count("formatSaveReport(report, BootReadsBackup::No") == 1);
    CHECK(count("BootReadsBackup::Yes") == 0);
    CHECK(count(".replace(") == 0);
    CHECK(count("ProsBlockSink") == 0);
    CHECK(count("ProsBlockSource") == 0);
    CHECK(count("saveAndReadBack") == 0);
    CHECK(count("printf(") == count("snprintf("));  // every printf is a buffer's (vsnprintf too)
    CHECK(count("puts(") == 0);
    CHECK(count("pros::") == 0);
    CHECK(count("#include \"pros/") == 0);
}

TEST_CASE("CalibrateProgram T4c: a FIRST record whose save FAILS -- NOT SAVED, and no FIRST RECORD note under it "
          "(the note names a record this run made; the card holds none, and the next boot will refuse)") {
    // where the run's first write falls, on an empty card
    std::size_t target = npos;
    {
        Scenario sc;
        Rig dry{sc};
        dry.run(robotTwoFacts());
        REQUIRE_FALSE(dry.timedOut);
        REQUIRE(drew(dry, "'FIRST RECORD -- the end you pushed first is FRONT'"));  // the control: saved, noted
        target = opIndex(dry.card, FakeFileOp::Replace, kRecordFileName);
        REQUIRE(target != npos);
    }
    Scenario sc;
    Rig rig{sc};
    FakeFileFaultPlan plan{};
    plan.atOperation = target;
    plan.fault = FakeFileFault::ShortWrite;
    plan.amount = 30;
    rig.card.plan(plan);
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    // the check still READ no record — the serial says the FIRST, as the brief words it ...
    CHECK(has(rig, "front check: the card holds no record yet -- this is the FIRST: the end that led the push is "
                   "the FRONT"));
    // ... the save's own words say it did not stick, and what the next boot will do
    CHECK(has(rig, "NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read back 30 of 180 "
                   "bytes)."));
    CHECK(has(rig, "Nothing usable is on the card (neither file decodes): the next boot will REFUSE until a run "
                   "saves."));
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "NOT SAVED") != rig.headers().end());
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") == rig.headers().end());
    // ... and the screen claims no record: no FIRST RECORD note under the red NOT SAVED header
    CHECK_FALSE(drew(rig, "FIRST RECORD"));
    CHECK(drew(rig, "print M 8 4 pen=101418 eraser=e2564a 'NOT SAVED"));
}

// ═══ T5. The watch and its hold ═══════════════════════════════════════════════════════

namespace {

const char* const kMechWarn6 = "WARNING: port 6 is declared as claw pivot 6 but TURNED WITH THE PUSH (+1692.4 deg; "
                               "the drive turned 1692 deg) -- the save waits for you";
const char* const kMechWarn12 = "WARNING: port 12 is declared as claw pivot 7 but TURNED WITH THE PUSH (+1692.4 deg; "
                                "the drive turned 1692 deg) -- the save waits for you";

/// Robot two's facts with drive port 12 typed where claw pivot 7 belongs (the probe's typo).
CalibrateFacts typo12Facts() {
    CalibrateFacts f = robotTwoFacts();
    f.mechanisms[1].port = 12;
    return f;
}

}  // namespace

TEST_CASE("CalibrateProgram T5a: a declared claw the push turns -- the WARNING with the drive's travel, "
          "MECHANISM PORT TURNED with NOTHING written while held; SAVE saves without it and paints the "
          "KEPT note") {
    Scenario sc;
    sc.clawTurns = 6;
    sc.heldMech[0] = kLeftBox;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(has(rig, kMechWarn6));
    CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+0.0 deg)"));
    const std::size_t m = findLine(L, "MECHANISM PORT TURNED -- port 6 is declared a mechanism but turned with "
                                      "the push (port 6: 1692 deg, drive 1692 deg).");
    REQUIRE(m != npos);
    CHECK(L[m + 1] == "If it is a mechanism the push turns (an intake on the floor), touch SAVE and keep it");
    CHECK(L[m + 2] == "declared. If it is a DRIVE motor, touch RUN AGAIN (nothing is saved), then stop: fix the");
    CHECK(L[m + 3] == "robot's facts or the wiring, rebuild, upload. NOTHING IS SAVED YET.");
    CHECK(countContaining(L, "DRIVES with port") == 0);  // an empty card: no record drives with it
    CHECK(has(rig, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 6: 1692 deg, drive 1692 deg / it is "
                   "declared a mechanism: not in this record | [SAVE] [RUN AGAIN]"));
    CHECK(has(rig, "touch: x=120 y=115 release=2 on SAVE -> SAVE AS PUSHED"));
    CHECK(has(rig, "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6"));
    // nothing written while held: no replace between the held screen going up and the SAVE tap,
    // and the whole save after it
    const std::size_t held = markOf(rig, "MECHANISM PORT TURNED");
    REQUIRE(held != npos);
    const auto* saveTap = rig.tap("MECHANISM PORT TURNED1");
    REQUIRE(saveTap != nullptr);
    CHECK(rig.marks[held].at < saveTap->at);
    CHECK(rig.replacesFrom(rig.marks[held].cardOps) == rig.replacesFrom(saveTap->cardOps));
    CHECK(rig.replacesFrom(0) == rig.replacesFrom(saveTap->cardOps));
    CHECK(rig.replacesFrom(saveTap->cardOps) > 0);
    REQUIRE(held + 1 < rig.marks.size());
    CHECK(rig.marks[held + 1].header == "SAVED");
    // the held screen, drawn
    CHECK(drew(rig, "print M 8 4 pen=101418 eraser=e8b23a 'MECHANISM PORT TURNED'"));
    CHECK(drew(rig, "print M 8 160 pen=e8b23a eraser=101418 'port 6: 1692 deg, drive 1692 deg'"));
    CHECK(drew(rig, "print S 8 180 pen=f0f0f0 eraser=101418 'it is declared a mechanism: not in this record'"));
    CHECK(drew(rig, "print S 8 196 pen=f0f0f0 eraser=101418 'SAVE: it is a mechanism the push turns -- keep it'"));
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'RUN AGAIN = save nothing: a drive motor? fix facts'"));
    // the NOTE after SAVE replaces the first record's: one question answered, one row
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 'PORT 6 KEPT AS A MECHANISM -- see the log'"));
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == kFrontLedSides);  // the claw is in no side
    CHECK(describe(d) == measured(rig));
    checkRecordIsTheMeasurement(rig, 0);  // every field, against what the run printed before the hold
    checkDeclaredUntouched(rig, 6);
    checkDeclaredUntouched(rig, 7);
}

TEST_CASE("CalibrateProgram T5b: a DRIVE port declared as a mechanism over a card whose record drives with it "
          "-- held, RUN AGAIN, held again, the field disables: the card byte-identical, the words say it kept "
          "its record") {
    Scenario sc;
    sc.runs = 2;
    sc.heldMech[0] = kRunAgain;
    sc.heldMech[1] = kDisable;
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(typo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(has(rig, "port 12: claw pivot 7 (declared) -- not a candidate, not touched"));
    CHECK(countPrefix(L, kMechWarn12) == 2);
    CHECK(countPrefix(L, "front check against shulib_drivetrain.bin: 9 drive port(s) in both -- 9 agree, 0 flipped, "
                         "0 sign only, 0 side only; 1 in the card's only, 0 in this run's only: SAME FRONT")
          == 2);
    CHECK(countPrefix(L, "The card's record DRIVES with port 12: saving now replaces that record with one that has "
                         "NO port 12.")
          == 2);
    CHECK(has(rig, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 12: 1692 deg, drive 1692 deg / the "
                   "card's record DRIVES with port 12 | [SAVE] [RUN AGAIN]"));
    CHECK(drew(rig, "'the card's record DRIVES with port 12'"));
    CHECK(has(rig, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card keeps its record"));
    CHECK(has(rig, "the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING SAVED: the "
                   "card keeps its record"));
    CHECK(rig.replacesFrom(0) == 0);
    CHECK(rig.fileBytes(kRecordFileName) == frontLedRecord());
    CHECK(rig.fileBytes(kRecordBackupFileName) == frontLedRecord());
    // the mistyped drive port: never asked for, never commanded — the watch only read it
    checkDeclaredUntouched(rig, 12);
    CHECK(rig.io.watchReads[12] == 4);
    // and port 7, no longer declared, is a candidate that does not turn: geared, never powered
    CHECK(rig.provider().motorRequestCount(7) == 1);
    CHECK(rig.rec(7).nonZeroCalls == 0);
    CHECK(has(rig, "port 7 did not turn with the push: not a drive motor, never powered -- if it is a mechanism, "
                   "declare it in the robot's facts"));
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T5c: the same typo on an EMPTY card -- the discard words follow the CARD's "
          "contents (it holds no usable record), never 'keeps its record'; no file appears") {
    Scenario sc;
    sc.runs = 2;
    sc.heldMech[0] = kRunAgain;
    sc.heldMech[1] = kDisable;
    Rig rig{sc};
    rig.run(typo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(countPrefix(L, "front check: the card holds no record yet -- this is the FIRST") == 2);
    CHECK(has(rig, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card is unchanged (it "
                   "holds no usable record)"));
    CHECK(has(rig, "the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING SAVED: the "
                   "card is unchanged (it holds no usable record)"));
    CHECK(countContaining(L, "keeps its record") == 0);
    CHECK(countContaining(L, "DRIVES with port") == 0);
    CHECK_FALSE(rig.card.file(kRecordFileName).has_value());
    CHECK_FALSE(rig.card.file(kRecordBackupFileName).has_value());
    CHECK(rig.replacesFrom(0) == 0);
}

TEST_CASE("CalibrateProgram T5d: the same typo with NO card -- both discards say there is no SD card, never "
          "'the card is unchanged'") {
    Scenario sc;
    sc.runs = 2;
    sc.card = false;
    sc.heldMech[0] = kRunAgain;
    sc.heldMech[1] = kDisable;
    Rig rig{sc};
    rig.run(typo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(has(rig, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, there is no SD card in "
                   "the brain"));
    CHECK(has(rig, "the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING SAVED: there "
                   "is no SD card in the brain"));
    CHECK(countContaining(L, "the card is unchanged") == 0);
    CHECK(countContaining(L, "keeps its record") == 0);
    CHECK(countPrefix(L, "front check: NOT DONE -- no SD card in the brain") == 2);
}

TEST_CASE("CalibrateProgram T5e: SAVE on the typo over a good card -- the operator's informed choice: the "
          "record without port 12 replaces the card's, and the note says so") {
    Scenario sc;
    sc.heldMech[0] = kLeftBox;
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(typo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "watch: SAVE -- port 12 stays a declared mechanism; this record has no port 12"));
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == "LEFT -11 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20");
    CHECK(describe(d) == measured(rig));
    checkRecordIsTheMeasurement(rig, 0);
    CHECK(drew(rig, "'PORT 12 KEPT AS A MECHANISM -- see the log'"));
}

TEST_CASE("CalibrateProgram T5f: FRONT FLIPPED AND a declared claw turned -- FRONT FLIPPED first, USE NEW, "
          "then MECHANISM PORT TURNED with its box RE-GREYED (a reflexive second tap lands on nothing); "
          "nothing written until both are answered") {
    Scenario sc;
    sc.pushDir[0] = -1.0;
    sc.clawTurns = 6;
    sc.heldFlip[0] = kLeftBox;
    sc.heldMech[0] = kLeftBox;
    sc.doubleTapLeftBox = true;
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t flipScreen = findPrefix(L, "screen: FRONT FLIPPED: NOTHING SAVED YET");
    // USE NEW asks the NEXT question here: it saves nothing, and its touch line says so (the review
    // round: "SAVE AS PUSHED" over a touch that saved nothing)
    const std::size_t useNew = findLine(L, "touch: x=120 y=115 release=2 on USE NEW -> USE NEW -- the next question "
                                           "follows; nothing is saved yet");
    const std::size_t mechScreen = findPrefix(L, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 6: "
                                                 "-1692 deg, drive 1692 deg");
    const std::size_t reflex = findLine(L, "touch: x=120 y=115 release=3 on nothing -> ignored -- the button was "
                                           "still GREYED (arming)");
    const std::size_t save = findLine(L, "touch: x=120 y=115 release=4 on SAVE -> SAVE AS PUSHED");
    REQUIRE(flipScreen != npos);
    REQUIRE(useNew != npos);
    REQUIRE(mechScreen != npos);
    REQUIRE(reflex != npos);
    REQUIRE(save != npos);
    CHECK(flipScreen < useNew);
    CHECK(useNew < mechScreen);
    CHECK(mechScreen < reflex);
    CHECK(reflex < save);
    CHECK(L[useNew + 1] == "front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)");
    CHECK(L[save + 1] == "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6");
    CHECK(countContaining(L, "-> SAVE AS PUSHED") == 1);  // the LAST question's SAVE only: the one that saves
    // the card holds a record, and it does NOT drive with 6: the screen says so, and nothing says
    // the record drives with it (only a card record that DOES drive with the port says that)
    CHECK(L[mechScreen] == "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 6: -1692 deg, drive 1692 deg / "
                           "it is declared a mechanism: not in this record | [SAVE] [RUN AGAIN]");
    CHECK(countContaining(L, "DRIVES with port") == 0);
    CHECK_FALSE(drew(rig, "DRIVES with port"));
    CHECK(rig.leftBoxTaps == 2);
    // nothing on the card changed until the SECOND question was answered: no replace before the
    // deliberate SAVE tap — not at USE NEW, not at the reflexive tap
    const std::size_t f = markOf(rig, "FRONT FLIPPED");
    const std::size_t mt = markOf(rig, "MECHANISM PORT TURNED");
    const std::size_t sv = markOf(rig, "SAVED -- NEW FRONT");
    REQUIRE(f != npos);
    REQUIRE(mt != npos);
    REQUIRE(sv != npos);
    CHECK(f < mt);
    CHECK(mt < sv);
    // the next question goes up AT ONCE, at the USE NEW tap's own instant — never a tick later
    const auto* useNewTap = rig.tap("FRONT FLIPPED1");
    REQUIRE(useNewTap != nullptr);
    CHECK(rig.marks[mt].at == useNewTap->at);
    const auto* saveTap = rig.tap("MECHANISM PORT TURNED1");
    const auto* reflexTap = rig.tap("REFLEX");
    REQUIRE(saveTap != nullptr);
    REQUIRE(reflexTap != nullptr);
    CHECK(reflexTap->at < saveTap->at);
    CHECK(rig.replacesFrom(0) == rig.replacesFrom(saveTap->cardOps));
    CHECK(rig.replacesFrom(saveTap->cardOps) > 0);
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == kOtherEndSides);  // as pushed
    CHECK(drew(rig, "'PORT 6 KEPT AS A MECHANISM -- see the log'"));
    checkDeclaredUntouched(rig, 6);
}

TEST_CASE("CalibrateProgram T5g: the watch's threshold is the calibrator's own test -- a claw that turns "
          "EXACTLY movedThreshold is named and holds; one a hair under it, or 2 deg, did not turn") {
    // The same expression the program evaluates, so the edge is landed on exactly, in binary.
    const double edge = shulib::calibration::CalibratorConfig{}.movedThreshold.value()
                        / shulib::calibration::kDegreesToRadians;
    SUBCASE("exactly the threshold: TURNED, held, SAVE") {
        Scenario sc;
        sc.clawTurns = 7;
        sc.clawExactDeg = edge;
        sc.heldMech[0] = kLeftBox;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "WARNING: port 7 is declared as claw pivot 7 but TURNED WITH THE PUSH (+5.0 deg; the drive "
                       "turned 1692 deg) -- the save waits for you"));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED")
              != rig.headers().end());
    }
    SUBCASE("a hair under: did not turn, nothing held") {
        Scenario sc;
        sc.clawTurns = 7;
        sc.clawExactDeg = std::nextafter(edge, 0.0);
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+5.0 deg)"));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED")
              == rig.headers().end());
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
    }
    SUBCASE("2 deg: did not turn (a jostled claw is not a drive motor)") {
        Scenario sc;
        sc.clawTurns = 6;
        sc.clawExactDeg = 2.0;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+2.0 deg)"));
        CHECK(countPrefix(rig.io.lines, "WARNING: port 6") == 0);
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
    }
}

TEST_CASE("CalibrateProgram T5h: a declared port the watch cannot read -- refused, or read as NaN -- is "
          "'not judged', never 'did not turn', and holds nothing") {
    for (const bool nan : {false, true}) {
        CAPTURE(nan);
        Scenario sc;
        (nan ? sc.nanPort : sc.unreadablePort) = 7;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 7 (claw pivot 7) could not be read -- not judged"));
        CHECK(countPrefix(rig.io.lines, "watch: port 7 (claw pivot 7) did not turn") == 0);
        CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+0.0 deg)"));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
        checkDeclaredUntouched(rig, 7);
    }
}

TEST_CASE("CalibrateProgram T5i: NO mechanism declared -- the banner says every motor is a drive candidate "
          "and one the push turns is POWERED; nothing is watched") {
    Scenario sc;
    sc.clawsPlugged = false;
    Rig rig{sc};
    CalibrateFacts f = robotTwoFacts();
    f.mechanisms[0] = {};
    f.mechanisms[1] = {};
    rig.run(f);
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t w = findLine(L, "watch      : no mechanism port declared -- nothing to watch, and EVERY motor "
                                      "is a");
    REQUIRE(w != npos);
    CHECK(L[w + 1] == "             drive candidate: one that turns with the push is POWERED (3, 6, 9 V).");
    CHECK(L[w + 2] == "             Declare every mechanism motor in the robot's facts, or unplug it first.");
    CHECK(countPrefix(L, "watch: ") == 0);
    CHECK(countContaining(L, "(declared)") == 0);
    int reads = 0;
    for (const int r : rig.io.watchReads) {
        reads += r;
    }
    CHECK(reads == 0);
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
}

TEST_CASE("CalibrateProgram T5j: in EVERY scenario the declared ports are never asked for, never commanded, "
          "never given a brake mode -- and each one is read once as a push begins and once more at an "
          "ACCEPTED push's DONE, never otherwise") {
    struct Sweep {
        const char* name;
        Scenario sc;
        bool seed;
        bool typo;
    };
    std::vector<Sweep> sweeps;
    auto add = [&sweeps](const char* name, auto&& edit, bool seed = false, bool typo = false) {
        Sweep w{name, Scenario{}, seed, typo};
        edit(w.sc);
        sweeps.push_back(w);
    };
    add("one", [](Scenario&) {});
    add("two runs", [](Scenario& s) { s.runs = 2; });
    add("slip18", [](Scenario& s) { s.decouple18 = true; });
    add("curved", [](Scenario& s) { s.runs = 2; s.curve[0] = 1.6; });
    add("no card", [](Scenario& s) { s.card = false; });
    add("disable mid-pulses", [](Scenario& s) { s.disableAfterPulses = 8.0; });
    add("flip, RUN AGAIN", [](Scenario& s) { s.runs = 3; s.pushDir[1] = -1.0; s.heldFlip[1] = kRunAgain; });
    add("flip, USE NEW", [](Scenario& s) { s.runs = 2; s.pushDir[1] = -1.0; s.heldFlip[1] = kLeftBox; });
    add("flip, disable", [](Scenario& s) { s.runs = 2; s.pushDir[1] = -1.0; s.heldFlip[1] = kDisable; });
    add("swap", [](Scenario& s) { s.swapBundles = true; s.heldFlip[0] = kLeftBox; }, true);
    add("claw 6 turned, SAVE", [](Scenario& s) { s.clawTurns = 6; s.heldMech[0] = kLeftBox; });
    add("claw 7 turned, RUN AGAIN", [](Scenario& s) { s.runs = 2; s.clawTurns = 7; s.heldMech[0] = kRunAgain; s.heldMech[1] = kDisable; });
    add("undeclared 8 and 9", [](Scenario& s) { s.undeclared8 = true; s.undeclared9 = true; });
    add("unequal scales", [](Scenario& s) { s.rightScale = kRobotTwoScaleRatio; });
    add("typo 12, RUN AGAIN", [](Scenario& s) { s.runs = 2; s.heldMech[0] = kRunAgain; s.heldMech[1] = kDisable; }, true, true);
    for (const Sweep& w : sweeps) {
        CAPTURE(std::string{w.name});
        Rig rig{w.sc};
        if (w.seed) {
            seedBoth(rig, frontLedRecord());
        }
        const CalibrateFacts facts = w.typo ? typo12Facts() : robotTwoFacts();
        rig.run(facts);
        REQUIRE_FALSE(rig.timedOut);
        int begun = 0;     // pushes begun: the watch's baseline read
        int accepted = 0;  // pushes the calibrator accepted: the watch's verdict read
        for (const auto& l : rig.io.lines) {
            begun += l.rfind("phase: PUSH | ", 0) == 0 ? 1 : 0;
            accepted += l.rfind("front: the end that LED the push is the FRONT", 0) == 0 ? 1 : 0;
        }
        CHECK(begun >= accepted);
        for (const auto& m : facts.mechanisms) {
            if (m.port == 0) {
                continue;
            }
            checkDeclaredUntouched(rig, m.port);
            CHECK(rig.io.watchReads[m.port] == begun + accepted);
        }
        checkAllStopped(rig);
    }
}

TEST_CASE("CalibrateProgram T5k: the watch judges TRAVEL since the push began, from a baseline it READ -- "
          "claws parked far from zero did not turn; a parked claw the push turns shows the same travel as "
          "from zero; a port whose push-start read was refused is not judged, though its DONE read answers") {
    // A claw's encoder holds whatever the last program left (robot two's claws HOLD where the driver
    // left them), so every rig scenario starting both claws at 0 deg could not tell a watch that judges
    // the READING, or one that judges against a baseline it never read, from the right one.
    SUBCASE("parked at +1234.5 and -987.25 deg, untouched by the push: did not turn, nothing held") {
        Scenario sc;
        sc.clawStartDeg[0] = 1234.5;
        sc.clawStartDeg[1] = -987.25;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+0.0 deg)"));
        CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+0.0 deg)"));
        CHECK(countPrefix(rig.io.lines, "WARNING: port") == 0);
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED")
              == rig.headers().end());
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
        CHECK(rig.io.watchReads[6] == 2);
        CHECK(rig.io.watchReads[7] == 2);
        checkDeclaredUntouched(rig, 6);
        checkDeclaredUntouched(rig, 7);
    }
    SUBCASE("the claw parked at +1234.5 deg turned by the push: T5a's travel, T5a's hold, SAVE") {
        Scenario sc;
        sc.clawStartDeg[0] = 1234.5;
        sc.clawStartDeg[1] = -987.25;
        sc.clawTurns = 6;
        sc.heldMech[0] = kLeftBox;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, kMechWarn6));  // +1692.4 deg: the travel, never 2926.9 (the reading)
        CHECK(has(rig, "MECHANISM PORT TURNED -- port 6 is declared a mechanism but turned with the push "
                       "(port 6: 1692 deg, drive 1692 deg)."));
        CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+0.0 deg)"));
        CHECK(has(rig, "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6"));
        checkDeclaredUntouched(rig, 6);
    }
    SUBCASE("port 7's push-start read refused, its DONE read answers -987.25: not judged, nothing held") {
        Scenario sc;
        sc.clawStartDeg[0] = 1234.5;
        sc.clawStartDeg[1] = -987.25;
        sc.unreadableAtPushPort = 7;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 7 (claw pivot 7) could not be read -- not judged"));
        CHECK(countPrefix(rig.io.lines, "watch: port 7 (claw pivot 7) did not turn") == 0);
        CHECK(countPrefix(rig.io.lines, "WARNING: port 7") == 0);
        CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+0.0 deg)"));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED")
              == rig.headers().end());
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
        checkDeclaredUntouched(rig, 7);
    }
    SUBCASE("port 6's push-start read answers +1234.5, its DONE read is refused: not judged -- never a turn of "
            "-1234.5 deg from a reading that never came") {
        Scenario sc;
        sc.clawStartDeg[0] = 1234.5;
        sc.unreadableAtDonePort = 6;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "watch: port 6 (claw pivot 6) could not be read -- not judged"));
        CHECK(countPrefix(rig.io.lines, "watch: port 6 (claw pivot 6) did not turn") == 0);
        CHECK(countPrefix(rig.io.lines, "WARNING: port 6") == 0);
        CHECK(has(rig, "watch: port 7 (claw pivot 7) did not turn with the push (+0.0 deg)"));
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED")
              == rig.headers().end());
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
        CHECK(rig.io.watchReads[6] == 2);  // both reads were MADE: the second one failed
        checkDeclaredUntouched(rig, 6);
    }
}

TEST_CASE("CalibrateProgram T5p: a declared claw that SETTLES during the IMU warm-up (+30 deg at 1 s, before any "
          "push) did not turn WITH the push -- the watch's baseline is read as the push begins, never earlier") {
    Scenario sc;
    sc.clawSettlesBeforePush = 6;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    CHECK(has(rig, "watch: port 6 (claw pivot 6) did not turn with the push (+0.0 deg)"));
    CHECK(countPrefix(rig.io.lines, "WARNING: port 6") == 0);
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "MECHANISM PORT TURNED") == rig.headers().end());
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
    CHECK(rig.io.watchReads[6] == 2);
    // the premise: the claw DID move, 30 deg — before the push, so a baseline taken at the IMU step
    // would have called it a turn with the push and held the save
    double deg = 0.0;
    REQUIRE(rig.io.readShaftDegrees(6, deg));
    CHECK(deg == 30.0);
    checkDeclaredUntouched(rig, 6);
}

namespace {

/// An intake on 6 (a mechanism the push legitimately turns) and DRIVE port 12 typed as claw pivot 7
/// — two declared ports that turn with the push, only one of them a mechanism.
CalibrateFacts intakeAndTypo12Facts() {
    CalibrateFacts f = robotTwoFacts();
    f.mechanisms[0] = {6, "intake"};
    f.mechanisms[1] = {12, "claw pivot 7"};
    return f;
}

const char* const kMechWarnIntake6 = "WARNING: port 6 is declared as intake but TURNED WITH THE PUSH (+1692.4 deg; the "
                                     "drive turned 1692 deg) -- the save waits for you";
const char* const kTwoTurned = "MECHANISM PORTS TURNED -- 2 declared ports turned with the push: each is asked on its "
                               "own screen, one at a time. NOTHING is saved until every one is answered SAVE; RUN "
                               "AGAIN on any of them saves nothing.";
const char* const kQ1Port6 = "MECHANISM PORT TURNED (1 of 2) -- port 6 is declared a mechanism but turned with the push "
                             "(port 6: 1692 deg, drive 1692 deg).";
const char* const kQ2Port12 = "MECHANISM PORT TURNED (2 of 2) -- port 12 is declared a mechanism but turned with the "
                              "push (port 12: 1692 deg, drive 1692 deg).";
const char* const kDrives12 = "The card's record DRIVES with port 12: saving now replaces that record with one that has "
                              "NO port 12.";
const char* const kScreenQ6 = "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 6: 1692 deg, drive 1692 deg / it "
                              "is declared a mechanism: not in this record | [SAVE] [RUN AGAIN]";
const char* const kScreenQ12 = "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 12: 1692 deg, drive 1692 deg / "
                               "the card's record DRIVES with port 12 | [SAVE] [RUN AGAIN]";

}  // namespace

TEST_CASE("CalibrateProgram T5l: TWO declared ports turned -- an intake (not in the card's record) and a DRIVE port "
          "typed as a mechanism (in it): each is its OWN question with its OWN evidence, so SAVE on the intake "
          "never hides that the card's record drives with 12; RUN AGAIN there, then a disable: the card "
          "byte-identical (the review's reproduced hazard)") {
    Scenario sc;
    sc.runs = 2;
    sc.clawTurns = 6;
    sc.heldMech[0] = kLeftBox;       // run 1: SAVE on the intake (right, for IT) ...
    sc.heldMechNext[0] = kRunAgain;  // ... then RUN AGAIN on 12, whose screen says the card drives with it
    sc.heldMech[1] = kDisable;       // run 2: the field disables on the first question
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(intakeAndTypo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(countPrefix(L, kMechWarnIntake6) == 2);
    CHECK(countPrefix(L, kMechWarn12) == 2);
    // run 1: the count, then port 6's question with ITS evidence
    const std::size_t lead = findLine(L, kTwoTurned);
    REQUIRE(lead != npos);
    CHECK(L[lead + 1] == kQ1Port6);
    CHECK(L[lead + 2] == "If it is a mechanism the push turns (an intake on the floor), touch SAVE and keep it");
    const std::size_t q1 = findLine(L, kScreenQ6, lead);
    REQUIRE(q1 != npos);
    // SAVE on 6 saves NOTHING yet: port 12's question comes next, with the card's DRIVES line
    const std::size_t save6 = findLine(L, "touch: x=120 y=115 release=2 on SAVE -> SAVE -- the next question follows; "
                                          "nothing is saved yet",
                                       q1);
    REQUIRE(save6 != npos);
    CHECK(L[save6 + 1] == "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6");
    CHECK(L[save6 + 2] == kQ2Port12);
    CHECK(L[save6 + 3] == kDrives12);
    const std::size_t q2 = findLine(L, kScreenQ12, save6);
    REQUIRE(q2 != npos);
    // drawn in place of port 6's line (padded with spaces over its longer text)
    CHECK(drew(rig, "print S 8 180 pen=f0f0f0 eraser=101418 'the card's record DRIVES with port 12"));
    CHECK(findLine(L, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card keeps its record",
                   q2)
          != npos);
    CHECK(countPrefix(L, "watch: SAVE -- port 12") == 0);
    CHECK(countContaining(L, "SAVE AS PUSHED") == 0);  // no touch in this run saved anything
    // run 2: the disable on the first question
    CHECK(has(rig, "the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING SAVED: the "
                   "card keeps its record"));
    CHECK(countPrefix(L, kQ1Port6) == 2);
    CHECK(countPrefix(L, kQ2Port12) == 1);  // run 2 never reached it
    // nothing was ever written: the card's good record, byte for byte, in both files
    CHECK(rig.replacesFrom(0) == 0);
    CHECK(rig.fileBytes(kRecordFileName) == frontLedRecord());
    CHECK(rig.fileBytes(kRecordBackupFileName) == frontLedRecord());
    checkDeclaredUntouched(rig, 6);
    checkDeclaredUntouched(rig, 12);
    CHECK(rig.io.watchReads[6] == 4);
    CHECK(rig.io.watchReads[12] == 4);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T5m: TWO declared ports turned, SAVE on both -- the second question's boxes RE-GREYED "
          "(a reflexive tap lands on nothing), nothing written until the LAST SAVE, the record as measured, and "
          "the NOTE counts the ports kept") {
    Scenario sc;
    sc.clawTurns = 6;
    sc.heldMech[0] = kLeftBox;  // SAVE on every question: the informed choice, port by port
    sc.doubleTapLeftBox = true;
    Rig rig{sc};
    seedBoth(rig, frontLedRecord());
    rig.run(intakeAndTypo12Facts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t q2 = findLine(L, kScreenQ12);
    const std::size_t reflex = findLine(L, "touch: x=120 y=115 release=3 on nothing -> ignored -- the button was "
                                           "still GREYED (arming)");
    const std::size_t save12 = findLine(L, "touch: x=120 y=115 release=4 on SAVE -> SAVE AS PUSHED");
    REQUIRE(q2 != npos);
    REQUIRE(reflex != npos);
    REQUIRE(save12 != npos);
    CHECK(q2 < reflex);
    CHECK(reflex < save12);
    CHECK(L[save12 + 1] == "watch: SAVE -- port 12 stays a declared mechanism; this record has no port 12");
    CHECK(has(rig, "touch: x=120 y=115 release=2 on SAVE -> SAVE -- the next question follows; nothing is saved yet"));
    CHECK(countContaining(L, "-> SAVE AS PUSHED") == 1);  // the last SAVE: the one that saves
    CHECK(L[save12 + 2].rfind("SAVED -- /usd/shulib_drivetrain.bin", 0) == 0);
    CHECK(has(rig, "watch: SAVE -- port 6 stays a declared mechanism; this record has no port 6"));
    CHECK(rig.leftBoxTaps == 2);
    // no replace before the second question's deliberate SAVE — not at the first SAVE, not at the reflex
    const auto* first = rig.tap("MECHANISM PORT TURNED1");
    const auto* reflexTap = rig.tap("REFLEX");
    const auto* last = rig.tap("MECHANISM PORT TURNED1#2");
    REQUIRE(first != nullptr);
    REQUIRE(reflexTap != nullptr);
    REQUIRE(last != nullptr);
    CHECK(first->at < reflexTap->at);
    CHECK(reflexTap->at < last->at);
    CHECK(rig.replacesFrom(0) == rig.replacesFrom(last->cardOps));
    CHECK(rig.replacesFrom(last->cardOps) > 0);
    // the record: as measured, with neither declared port in it
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == "LEFT -11 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20");
    checkRecordIsTheMeasurement(rig, 0);
    CHECK(drew(rig, "print S 8 212 pen=e8b23a eraser=101418 '2 PORTS KEPT AS MECHANISMS -- see the log'"));
    CHECK_FALSE(drew(rig, "PORT 6 KEPT"));
    CHECK_FALSE(drew(rig, "PORT 12 KEPT"));
    checkDeclaredUntouched(rig, 6);
    checkDeclaredUntouched(rig, 12);
}

TEST_CASE("CalibrateProgram T5n: the questions come in the FACTS' order, and RUN AGAIN on any ends them; two "
          "claws turned on an EMPTY card: both asked, neither 'drives', both kept") {
    SUBCASE("12 listed first: its question (the card drives with it) comes first; RUN AGAIN -- 6 is never asked") {
        Scenario sc;
        sc.runs = 2;
        sc.clawTurns = 6;
        sc.heldMech[0] = kRunAgain;
        sc.heldMech[1] = kDisable;
        Rig rig{sc};
        seedBoth(rig, frontLedRecord());
        CalibrateFacts f = robotTwoFacts();
        f.mechanisms[0] = {12, "claw pivot 7"};
        f.mechanisms[1] = {6, "intake"};
        rig.run(f);
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        const std::size_t q1 = findLine(L, "MECHANISM PORT TURNED (1 of 2) -- port 12 is declared a mechanism but "
                                           "turned with the push (port 12: 1692 deg, drive 1692 deg).");
        REQUIRE(q1 != npos);
        CHECK(L[q1 + 1] == kDrives12);
        CHECK(countPrefix(L, "MECHANISM PORT TURNED (2 of 2)") == 0);
        CHECK(countPrefix(L, "MECHANISM PORT TURNED (1 of 2) -- port 12") == 2);
        CHECK(rig.replacesFrom(0) == 0);
        CHECK(rig.fileBytes(kRecordFileName) == frontLedRecord());
    }
    SUBCASE("both claws turned on an empty card: two questions, no DRIVES line, SAVE twice, 2 PORTS KEPT") {
        Scenario sc;
        sc.clawTurns = 6;
        sc.clawTurnsToo = 7;
        sc.heldMech[0] = kLeftBox;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        CHECK(has(rig, kTwoTurned));
        CHECK(has(rig, "MECHANISM PORT TURNED (1 of 2) -- port 6 is declared a mechanism but turned with the push "
                       "(port 6: 1692 deg, drive 1692 deg)."));
        CHECK(has(rig, "MECHANISM PORT TURNED (2 of 2) -- port 7 is declared a mechanism but turned with the push "
                       "(port 7: 1692 deg, drive 1692 deg)."));
        CHECK(has(rig, "screen: MECHANISM PORT TURNED: NOTHING SAVED YET / port 7: 1692 deg, drive 1692 deg / it is "
                       "declared a mechanism: not in this record | [SAVE] [RUN AGAIN]"));
        CHECK(countContaining(L, "DRIVES with port") == 0);
        CHECK(sidesOf(decoded(rig.fileBytes(kRecordFileName))) == kFrontLedSides);
        CHECK(drew(rig, "'2 PORTS KEPT AS MECHANISMS -- see the log'"));
        checkDeclaredUntouched(rig, 6);
        checkDeclaredUntouched(rig, 7);
    }
}

TEST_CASE("CalibrateProgram T5o: a held run discarded over files that OPENED and do not decode -- 'the card is "
          "unchanged (it holds no usable record)', never 'keeps its record' (C2: no usable record is no "
          "record); the files byte for byte as they were") {
    for (const bool both : {true, false}) {
        CAPTURE(both);
        Scenario sc;
        sc.runs = 2;
        sc.heldMech[0] = kRunAgain;
        sc.heldMech[1] = kDisable;
        Rig rig{sc};
        if (both) {
            rig.card.setFile(kRecordFileName, garbage(180));
        }
        rig.card.setFile(kRecordBackupFileName, garbage(180));
        rig.run(typo12Facts());
        REQUIRE_FALSE(rig.timedOut);
        const auto& L = rig.io.lines;
        CHECK(countPrefix(L, "front check: NOT DONE -- the card's record does not decode") == 2);
        CHECK(has(rig, "RUN AGAIN on a held screen -- NOTHING SAVED: this run is discarded, the card is unchanged (it "
                       "holds no usable record)"));
        CHECK(has(rig, "the field disabled the robot on the held MECHANISM PORT TURNED screen -- NOTHING SAVED: the "
                       "card is unchanged (it holds no usable record)"));
        CHECK(countContaining(L, "keeps its record") == 0);
        CHECK(countContaining(L, "DRIVES with port") == 0);  // no usable record drives with anything
        CHECK(rig.replacesFrom(0) == 0);
        CHECK(rig.fileBytes(kRecordBackupFileName) == garbage(180));
        if (both) {
            CHECK(rig.fileBytes(kRecordFileName) == garbage(180));
        } else {
            CHECK_FALSE(rig.card.file(kRecordFileName).has_value());
        }
    }
}

// ═══ T5b. Undeclared mechanism motors (re-check R11) ═════════════════════════════════

TEST_CASE("CalibrateProgram T5b-1: an undeclared motor the push does not turn is geared and never powered; "
          "one the push turns is PULSED at 3, 6 and 9 V, then LEFT OUT -- each with the words that say "
          "declare it") {
    Scenario sc;
    sc.undeclared8 = true;
    sc.undeclared9 = true;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    CHECK(has(rig, "port 8 did not turn with the push: not a drive motor, never powered -- if it is a mechanism, "
                   "declare it in the robot's facts"));
    CHECK(countPrefix(L, "port 9 did not turn") == 0);
    // port 8: a candidate (the drive cartridge), 0 V and Coast only, never a non-zero command
    CHECK(rig.provider().motorRequestCount(8) == 1);
    CHECK(rig.provider().motorCartridge(8) == MotorCartridge::Blue);
    CHECK(rig.rec(8).voltageCalls > 0);
    CHECK(rig.rec(8).nonZeroCalls == 0);
    CHECK(rig.rec(8).maxAbs == 0.0);
    CHECK(rig.rec(8).nonCoastCalls == 0);
    // port 9: pulsed at exactly the three levels, then never again
    const std::vector<double> levels = {3.0, 6.0, 9.0};
    CHECK(rig.rec(9).levels == levels);
    CHECK(has(rig, "pulse port 9 (1 of 11) 9.0 V yaw +0.00 deg own +1051.5 deg others 0.0 deg: LEFT OUT (decoupled)"));
    CHECK(has(rig, "LEFT OUT: port 9 turns but does not move the robot (a slipping gear? or a MECHANISM motor -- "
                   "declare it in the robot's facts) -- it is in no side and is never commanded again"));
    // the record: ten drive ports, neither 8 nor 9
    const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
    CHECK(sidesOf(d) == kFrontLedSides);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T5b-2: the undeclared motor the push turns is LOADED (it cannot spin) -- the "
          "pulse refusal names it, and says to declare it if it is a mechanism") {
    Scenario sc;
    sc.undeclared9 = true;
    sc.loaded9 = true;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t st = findPrefix(L, "STOPPED [PULSE_STILL port 9]: port 9 could not move the robot at 9.0 V");
    REQUIRE(st != npos);
    CHECK(L[st + 1] == "if port 9 is a MECHANISM motor, declare it in the robot's facts: a motor that is not "
                       "declared is a drive candidate");
    CHECK(L[st + 2] == "Every motor is at 0 V and coasting. Nothing was written to the card by this run.");
    CHECK(rig.replacesFrom(0) == 0);
    checkAllStopped(rig);
}

TEST_CASE("CalibrateProgram T5b-3: a pulse that turns its train and not the robot (wheels up, from inside) -- "
          "PULSE_ROLLED names the port, and says to declare it if it is a mechanism, as PULSE_STILL does") {
    Scenario sc;
    sc.headingHeld = true;
    Rig rig{sc};
    rig.run(robotTwoFacts());
    REQUIRE_FALSE(rig.timedOut);
    const auto& L = rig.io.lines;
    const std::size_t st = findPrefix(L, "STOPPED [PULSE_ROLLED port 11]: port 11 moved its gear train (others ");
    REQUIRE(st != npos);
    CHECK(L[st + 1] == "if port 11 is a MECHANISM motor, declare it in the robot's facts: a motor that is not "
                       "declared is a drive candidate");
    CHECK(L[st + 2] == "Every motor is at 0 V and coasting. Nothing was written to the card by this run.");
    CHECK(rig.replacesFrom(0) == 0);
    checkAllStopped(rig);
}

// ═══ T6. The facts ════════════════════════════════════════════════════════════════════

namespace {

/// A refusal of the facts: before ANY device was asked for, nothing commanded, nothing written, the
/// headline painted and the two serial lines verbatim.
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
    CHECK(r + 2 == rig.io.lines.size());  // the last words: nothing ran after a refusal
    CHECK(drew(rig, "print L 8 34 pen=e2564a eraser=101418 '" + headline + "'"));
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "CANNOT START") != rig.headers().end());
    CHECK(std::find(rig.headers().begin(), rig.headers().end(), "1/4 IMU") == rig.headers().end());
}

}  // namespace

TEST_CASE("CalibrateProgram T6a: the mechanism list's refusals -- a NAMED port 0, a port past 21, a port listed "
          "twice -- each naming its entry, before ANY device is asked for and before the drop prints a line") {
    struct Case {
        const char* name;
        shulib::calibration::MechanismPort entry;
        std::size_t at;
        const char* why;
    };
    const Case cases[] = {
        {"a named port 0", {0, "intake"}, 2,
         "a mechanism is named but its port is 0 -- state its port: mechanisms[2] = {0, \"intake\"}"},
        {"port 22", {22, "lift"}, 2, "a mechanism port is not a smart port (1..21): mechanisms[2] = {22, \"lift\"}"},
        {"port 255", {255, nullptr}, 0, "a mechanism port is not a smart port (1..21): mechanisms[0] = {255, \"\"}"},
        {"6 twice", {6, "second claw"}, 2,
         "a mechanism port is listed twice: mechanisms[2] = {6, \"second claw\"}"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.name});
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.mechanisms[c.at] = c.entry;
        rig.run(f);
        checkRefusedBeforeAnything(rig, "FACTS REFUSED", c.why, "correct the robot's stated facts and rebuild");
        // the drop never ran on a list it could not trust: no "(declared)" line, true or false
        CHECK(countContaining(rig.io.lines, "(declared)") == 0);
        CHECK(has(rig, "  12 motor(s), 1 IMU(s)"));  // the census still printed
    }
}

TEST_CASE("CalibrateProgram T6b: the stated drive kind and cartridge -- unset, not a tank (x-drive, h-drive), "
          "a byte that is no cartridge -- each refused by name before ANY device is asked for") {
    {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.drive.reset();
        rig.run(f);
        checkRefusedBeforeAnything(rig, "DRIVE KIND UNSET", "the drive kind is not stated",
                                   "state it in the robot's facts (Tank), rebuild, upload");
    }
    for (const auto kind : {shulib::config::DrivetrainKind::XDrive, shulib::config::DrivetrainKind::HDrive}) {
        const std::string word = shulib::config::drivetrainKindWord(kind);
        CAPTURE(word);
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.drive = kind;
        rig.run(f);
        checkRefusedBeforeAnything(
            rig, "NOT A TANK",
            "the facts say " + std::string{kind == shulib::config::DrivetrainKind::XDrive ? "x-drive" : "h-drive"}
                + "; this procedure measures a TANK drivetrain only",
            "a holonomic drive's calibration is not built yet -- do not run this on it");
    }
    {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.cartridge.reset();
        rig.run(f);
        checkRefusedBeforeAnything(rig, "CARTRIDGE UNSET",
                                   "the cartridge cannot be measured from inside; read the insert colour off a motor",
                                   "state it in the robot's facts, rebuild, upload");
    }
    {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.cartridge = static_cast<MotorCartridge>(7);
        rig.run(f);
        checkRefusedBeforeAnything(rig, "CARTRIDGE UNSET", "the stated cartridge (byte 7) is not RED, GREEN or BLUE",
                                   "state RED, GREEN or BLUE in the robot's facts, rebuild, upload");
    }
}

TEST_CASE("CalibrateProgram T6c: drive motors declared as mechanisms until fewer than two remain -- TOO FEW "
          "MOTORS names every declared port the census found; without a declared port, the plain fix") {
    {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        for (int k = 0; k < 9; ++k) {
            f.mechanisms[2 + k] = {static_cast<std::uint8_t>(11 + k), "drive?"};
        }
        f.mechanisms[11] = {21, "not plugged"};  // declared, no motor: named by the census, not by the fix
        rig.run(f);
        checkRefusedBeforeAnything(
            rig, "TOO FEW MOTORS",
            "the registry found 1 motor(s) besides the declared mechanisms; a drivetrain needs at least two",
            "declared as mechanisms: 6 7 11 12 13 14 15 16 17 18 19 -- is any a DRIVE motor? else re-seat the drive "
            "cables");
        CHECK(has(rig, "port 21: not plugged (declared) -- the census found NO MOTOR there"));
    }
    {
        // nothing declared and one motor on the robot: the fix is the cables
        Scenario sc;
        sc.clawsPlugged = false;
        Rig rig{sc};
        for (int p = 12; p <= 20; ++p) {
            rig.registry.setKind(p, DeviceKind::None);
        }
        CalibrateFacts f = robotTwoFacts();
        f.mechanisms[0] = {};
        f.mechanisms[1] = {};
        rig.run(f);
        checkRefusedBeforeAnything(
            rig, "TOO FEW MOTORS",
            "the registry found 1 motor(s) besides the declared mechanisms; a drivetrain needs at least two",
            "re-seat the drive cables; the census is on the serial log");
    }
    {
        // the EDGE: exactly two motors besides the declared ones is a drivetrain — the calibrator starts
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        for (const int p : {12, 13, 14, 15, 17, 18, 19, 20}) {
            rig.registry.setKind(p, DeviceKind::None);
        }
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(countPrefix(rig.io.lines, "REFUSED: ") == 0);
        CHECK(has(rig, "calibrator started: 2 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
        CHECK(rig.provider().motorRequestCount(11) == 1);
        CHECK(rig.provider().motorRequestCount(16) == 1);
    }
}

TEST_CASE("CalibrateProgram T6c-2: the IMU count's edge (TWO IMUs refused, naming the count) and the adapters' "
          "refusals: a motor the provider will not make is NAMED and dropped; exactly two made still starts; one "
          "made is MOTORS REFUSED; an IMU the provider will not make is IMU REFUSED -- nothing ever powered") {
    SUBCASE("two IMUs: refused before ANY device") {
        Scenario sc;
        Rig rig{sc};
        rig.registry.setKind(3, DeviceKind::Imu);
        rig.run(robotTwoFacts());
        checkRefusedBeforeAnything(rig, "IMU COUNT NOT 1",
                                   "the registry reports 2 IMUs; this procedure needs exactly one",
                                   "plug in exactly one inertial sensor and run again");
    }
    SUBCASE("port 20 answers MOTOR but will not be made: a WARNING naming it, nine candidates") {
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        rig.provider().refusePort(20);
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "WARNING: port 20 answered MOTOR in the registry but could not be made: port 20 has no motor "
                       "-- dropped from the candidates"));
        CHECK(has(rig, "calibrator started: 9 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
        CHECK(rig.rec(20).voltageCalls == 0);  // never made, never commanded
    }
    SUBCASE("three in the census, one refused: exactly two made -- the calibrator starts (the edge)") {
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        for (const int p : {12, 13, 14, 15, 17, 18, 19}) {
            rig.registry.setKind(p, DeviceKind::None);
        }
        rig.provider().refusePort(20);
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "WARNING: port 20 answered MOTOR in the registry but could not be made: port 20 has no motor "
                       "-- dropped from the candidates"));
        CHECK(countPrefix(rig.io.lines, "REFUSED: ") == 0);
        CHECK(has(rig, "calibrator started: 2 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
    }
    SUBCASE("three in the census, two refused: one made -- MOTORS REFUSED, the IMU never asked for, nothing powered") {
        Scenario sc;
        Rig rig{sc};
        for (const int p : {12, 13, 14, 15, 17, 18, 19}) {
            rig.registry.setKind(p, DeviceKind::None);
        }
        rig.provider().refusePort(16);
        rig.provider().refusePort(20);
        // what the last program left on the one motor that will be made: 5 V and held (as T1l) — this
        // exit returns before the program's own all-stop, so only its RAII guard can stop it
        rig.rec(11).setVoltage(Voltage{5.0});
        rig.rec(11).setBrakeMode(BrakeMode::Hold);
        const int inheritedCalls = rig.rec(11).nonZeroCalls;
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        const std::size_t r = findLine(rig.io.lines, "REFUSED: MOTORS REFUSED -- fewer than two motor adapters would "
                                                     "construct; see the WARNING lines");
        REQUIRE(r != npos);
        CHECK(rig.io.lines[r + 1] == "  re-seat the drive cables; the census is on the serial log");
        CHECK(r + 2 == rig.io.lines.size());
        CHECK(countPrefix(rig.io.lines, "WARNING: port 16 answered MOTOR") == 1);
        CHECK(countPrefix(rig.io.lines, "WARNING: port 20 answered MOTOR") == 1);
        CHECK(drew(rig, "print L 8 34 pen=e2564a eraser=101418 'MOTORS REFUSED'"));
        CHECK(rig.counting.imuRequests == 0);
        CHECK(rig.rec(11).nonZeroCalls == inheritedCalls);  // the program powered nothing ...
        CHECK(rig.rec(11).commandedVoltage().value() == 0.0);  // ... and left nothing powered
        CHECK(rig.rec(11).brakeMode() == BrakeMode::Coast);
        CHECK(rig.replacesFrom(0) == 0);
    }
    SUBCASE("a MIDDLE port (13) answers MOTOR but will not be made: dropped by name, and the run SAVES a record "
            "that pairs every made motor with its OWN port") {
        // The drop compacts the port list beside the motors; the subcases above refuse the LAST port,
        // where a list never compacted reads the same, and end before any save.
        Scenario sc;
        Rig rig{sc};
        rig.provider().refusePort(13);
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "WARNING: port 13 answered MOTOR in the registry but could not be made: port 13 has no motor "
                       "-- dropped from the candidates"));
        CHECK(has(rig, "calibrator started: 9 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
        CHECK(has(rig, "drive ports, signed: -11 +12 +14 -15 +16 -17 +18 -19 +20  (9 to pulse, one at a time)"));
        CHECK(findPrefix(rig.io.lines, "SAVED -- /usd/shulib_drivetrain.bin") != npos);
        const DrivetrainDescription d = decoded(rig.fileBytes(kRecordFileName));
        CHECK(sidesOf(d) == "LEFT -11 +12 +14 -15 | RIGHT +16 -17 +18 -19 +20");
        checkRecordIsTheMeasurement(rig, 0);
        int total = 0;
        CHECK(rig.forwardScore(d, total) == 9);
        CHECK(total == 9);
        CHECK(rig.rec(13).voltageCalls == 0);  // never made, never commanded
        CHECK(rig.provider().motorRequestCount(13) == 1);
        checkAllStopped(rig);
    }
    SUBCASE("the IMU will not be made: IMU REFUSED with the provider's reason -- the motors made, never powered") {
        Scenario sc;
        Rig rig{sc};
        rig.provider().refusePort(2);
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        const std::size_t r = findLine(rig.io.lines, "REFUSED: IMU REFUSED -- port 2 has no IMU");
        REQUIRE(r != npos);
        CHECK(rig.io.lines[r + 1] == "  check the sensor and its cable");
        CHECK(r + 2 == rig.io.lines.size());
        CHECK(drew(rig, "print L 8 34 pen=e2564a eraser=101418 'IMU REFUSED'"));
        CHECK(rig.counting.imuRequests == 1);
        for (int p = 11; p <= 20; ++p) {
            CAPTURE(p);
            CHECK(rig.rec(p).nonZeroCalls == 0);
        }
        checkAllStopped(rig);
        CHECK(rig.replacesFrom(0) == 0);
    }
}

TEST_CASE("CalibrateProgram T6f: a STATED IMU port is a cross-check, never the truth -- one that differs is a "
          "WARNING and the registry's port is used; none stated says so; the same one says it agrees") {
    SUBCASE("stated 3, the IMU on 2") {
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.imuPortStated = 3;
        rig.run(f);
        REQUIRE_FALSE(rig.timedOut);
        const std::size_t w = findLine(rig.io.lines, "WARNING: the robot's facts say IMU port 3, the registry says "
                                                     "port 2 -- using the REGISTRY (a stated port is a cross-check, "
                                                     "not the truth)");
        REQUIRE(w != npos);
        CHECK(rig.io.lines[w + 1] == "IMU        : port 2, DISCOVERED from the registry (stated 3)");
        CHECK(has(rig, "calibrator started: 10 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
    }
    SUBCASE("none stated") {
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.imuPortStated = 0;
        rig.run(f);
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "IMU        : port 2, DISCOVERED from the registry"));
        CHECK(countPrefix(rig.io.lines, "WARNING: the robot's facts say IMU port") == 0);
    }
    SUBCASE("stated 2, the IMU on 2: no WARNING") {
        Scenario sc;
        sc.disableAt = 3.0;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "IMU        : port 2, DISCOVERED from the registry (stated 2)"));
        CHECK(countPrefix(rig.io.lines, "WARNING: the robot's facts say IMU port") == 0);
    }
}

TEST_CASE("CalibrateProgram T6d: the declared-port drop keeps the census order and reads the FACTS -- an empty "
          "list drops nothing; both pivots dropped; a declared port the census did not list is named, and the "
          "run goes on") {
    SUBCASE("an empty facts list: every motor is a candidate, in the census's order") {
        Scenario sc;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.mechanisms[0] = {};
        f.mechanisms[1] = {};
        rig.run(f);
        REQUIRE_FALSE(rig.timedOut);
        CHECK(countContaining(rig.io.lines, "(declared)") == 0);
        CHECK(has(rig, "calibrator started: 12 candidate(s), kind Tank, cartridge BLUE 600 rpm, IMU port 2, "
                       "provenance \"shulib Calibrate Oct 18 2026 12:34:56\""));
        CHECK(has(rig, "port 6 did not turn with the push: not a drive motor, never powered -- if it is a "
                       "mechanism, declare it in the robot's facts"));
        CHECK(has(rig, "port 7 did not turn with the push: not a drive motor, never powered -- if it is a "
                       "mechanism, declare it in the robot's facts"));
        // the push table lists the candidates in the census's order: 6, 7, then 11..20
        const std::size_t t = findPrefix(rig.io.lines, "PUSH TABLE at DONE");
        REQUIRE(t != npos);
        CHECK(rig.io.lines[t + 1].rfind("  push port 6 ", 0) == 0);
        CHECK(rig.io.lines[t + 2].rfind("  push port 7 ", 0) == 0);
        CHECK(rig.io.lines[t + 3].rfind("  push port 11 ", 0) == 0);
        CHECK(rig.io.lines[t + 12].rfind("  push port 20 ", 0) == 0);
        CHECK(rig.provider().motorRequestCount(6) == 1);  // geared as a candidate — and never powered
        CHECK(rig.rec(6).nonZeroCalls == 0);
    }
    SUBCASE("both pivots dropped, named by the facts, and the rest kept in order") {
        Scenario sc;
        Rig rig{sc};
        rig.run(robotTwoFacts());
        REQUIRE_FALSE(rig.timedOut);
        const std::size_t c = findLine(rig.io.lines, "  12 motor(s), 1 IMU(s)");
        REQUIRE(c != npos);
        CHECK(rig.io.lines[c + 1] == "port 6: claw pivot 6 (declared) -- not a candidate, not touched");
        CHECK(rig.io.lines[c + 2] == "port 7: claw pivot 7 (declared) -- not a candidate, not touched");
        const std::size_t t = findPrefix(rig.io.lines, "PUSH TABLE at DONE");
        REQUIRE(t != npos);
        CHECK(rig.io.lines[t + 1].rfind("  push port 11 ", 0) == 0);
        CHECK(rig.io.lines[t + 10].rfind("  push port 20 ", 0) == 0);
        CHECK(rig.io.lines[t + 11].rfind("push: 10 of 10 candidates moved", 0) == 0);
    }
    SUBCASE("a declared port with no motor: named by the census, the run goes on") {
        Scenario sc;
        sc.clawsPlugged = false;
        Rig rig{sc};
        CalibrateFacts f = robotTwoFacts();
        f.mechanisms[1] = {};
        f.mechanisms[0] = {6, nullptr};  // no name: "a mechanism"
        rig.run(f);
        REQUIRE_FALSE(rig.timedOut);
        CHECK(has(rig, "port 6: a mechanism (declared) -- the census found NO MOTOR there"));
        CHECK(has(rig, "watch: port 6 (a mechanism) could not be read -- not judged"));
        CHECK(rig.provider().motorRequestCount(6) == 0);
        CHECK(std::find(rig.headers().begin(), rig.headers().end(), "SAVED") != rig.headers().end());
    }
}

TEST_CASE("CalibrateProgram T6e: factsProblem() at COMPILE time -- the checks a build can static_assert, both "
          "ways") {
    using shulib::calibration::CalibrateFacts;
    using shulib::calibration::factsProblem;
    constexpr CalibrateFacts team{
        .drive = shulib::config::DrivetrainKind::Tank,
        .cartridge = MotorCartridge::Blue,
        .mechanisms = {{6, "left claw"}, {7, "right claw"}},
    };
    static_assert(factsProblem(team).why == nullptr);
    constexpr CalibrateFacts empty{};
    static_assert(factsProblem(empty).why == nullptr);  // nothing declared is a valid list
    constexpr CalibrateFacts namedZero{.mechanisms = {{6, "a"}, {0, "intake"}}};
    static_assert(factsProblem(namedZero).why != nullptr && factsProblem(namedZero).entry == 1);
    constexpr CalibrateFacts unnamedZero{.mechanisms = {{6, "a"}, {0, nullptr}, {7, "b"}}};
    static_assert(factsProblem(unnamedZero).why == nullptr);  // an unused slot, anywhere
    constexpr CalibrateFacts edge21{.mechanisms = {{21, "top port"}}};
    static_assert(factsProblem(edge21).why == nullptr);
    constexpr CalibrateFacts past21{.mechanisms = {{1, "a"}, {22, "b"}}};
    static_assert(factsProblem(past21).why != nullptr && factsProblem(past21).entry == 1);
    constexpr CalibrateFacts twice{.mechanisms = {{6, "a"}, {7, "b"}, {6, "c"}}};
    static_assert(factsProblem(twice).why != nullptr && factsProblem(twice).entry == 2);  // the SECOND entry
    constexpr CalibrateFacts lastSlot{.mechanisms = {{6, "a"}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
                                                     {}, {}, {}, {}, {}, {}, {6, "z"}}};
    static_assert(factsProblem(lastSlot).why != nullptr && factsProblem(lastSlot).entry == 20);
    static_assert(shulib::calibration::declaredMechanism(team, 6) == &team.mechanisms[0]);
    static_assert(shulib::calibration::declaredMechanism(team, 8) == nullptr);
    static_assert(shulib::calibration::declaredMechanism(team, 0) == nullptr);  // an unused slot is no port
    CHECK(std::string{factsProblem(twice).why} == "a mechanism port is listed twice");
}
