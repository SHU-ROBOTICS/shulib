// Tests for diag/motion_result.hpp — the §18.3 per-motion result line. What each targets:
//  * WIRE STABILITY: MotionOutcome numbers are §18.4's boundary vocabulary; a reorder
//    re-labels history.
//  * BYTE-EXACT SHAPE: the line is the §18.3 target; column drift or a token change
//    breaks the at-a-glance discipline the section exists for.
//  * HONESTY: the n/a path must render n/a — a fabricated 0.00 where no data flowed
//    is the lying-number failure the C5 brief bans outright.
//  * UGLY VALUES: NaN/±Inf/1e300 must render as the deterministic §18.3 tokens, not
//    libc's locale spellings or a 300-digit column explosion.
//  * THE xt COLUMN (TANK1 Part B): appended AFTER the duration only when the producer says so
//    AND path data flowed; every earlier byte of the §18.3 line unchanged (the goldens above
//    it are unedited — they are the "absent" half of this contract).

#include "doctest.h"

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/diag/motion_result.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::FaultCode;
using shulib::diag::MotionOutcome;
using shulib::diag::MotionResult;
using shulib::diag::TermSink;
using shulib::diag::emitResultLine;
using shulib::diag::motionOutcomeName;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::math::Angle;
using shulib::math::Pose2d;
namespace units = shulib::units;

namespace {
constexpr double kDegToRad = shulib::math::Angle::kPi / 180.0;

MotionResult settledResult() {
    MotionResult r;
    r.id = 7;
    r.name = "MoveToPose";
    r.outcome = MotionOutcome::Settled;
    r.duration = units::Time{1.16};
    r.hasPathData = true;
    r.finalPose = Pose2d{units::Length{24.1}, units::Length{36.0}, Angle::degrees(90.1)};
    r.overshoot = units::Length{0.2};
    r.drift = units::AngleDim{0.1 * kDegToRad};
    return r;
}

std::string render(const MotionResult& r, double t = 12.50) {
    FakeClock clock{units::Time{t}};
    FakeCharSink out;
    TermSink term{clock, out};
    emitResultLine(term, r);
    return out.text();
}
}  // namespace

TEST_CASE("MotionOutcome: §18.4 boundary vocabulary is wire-stable — a reorder turns "
          "this red") {
    CHECK(static_cast<std::uint8_t>(MotionOutcome::Settled) == 0);
    CHECK(static_cast<std::uint8_t>(MotionOutcome::TimedOut) == 1);
    CHECK(static_cast<std::uint8_t>(MotionOutcome::Cancelled) == 2);
    CHECK(static_cast<std::uint8_t>(MotionOutcome::FaultAbort) == 3);
    CHECK(static_cast<std::uint8_t>(MotionOutcome::Superseded) == 4);
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::Settled)} == "SETTLED");
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::TimedOut)} == "TIMEOUT");
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::Cancelled)} == "CANCELLED");
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::FaultAbort)} == "FAULT_ABORT");
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::Superseded)} == "SUPERSEDED");
    CHECK(std::string_view{motionOutcomeName(static_cast<MotionOutcome>(200))} == "UNKNOWN");
}

// Bug caught: ANY byte drift in the §18.3 result line — the golden that closes
// the "per-motion result line matches §18.3" DoD clause.
TEST_CASE("result line: the §18.3 golden bytes — ✓SETTLED with full path data") {
    CHECK(render(settledResult()) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s\n");
}

// Bug caught: a failure outcome rendering with the settle checkmark (or the abort
// line losing its causal code) — the glanceable pass/fail column lying.
TEST_CASE("result line: every non-settle outcome renders ✗, FAULT_ABORT carries its "
          "causal code") {
    MotionResult r = settledResult();

    r.outcome = MotionOutcome::TimedOut;
    CHECK(render(r).find(" ✗TIMEOUT final(") != std::string::npos);

    r.outcome = MotionOutcome::Cancelled;
    CHECK(render(r).find(" ✗CANCELLED final(") != std::string::npos);

    r.outcome = MotionOutcome::Superseded;
    CHECK(render(r).find(" ✗SUPERSEDED final(") != std::string::npos);

    r.outcome = MotionOutcome::FaultAbort;
    r.abortFault = FaultCode::OdoStuck;
    const std::string line = render(r);
    CHECK(line.find(" ✗FAULT_ABORT=ODO_STUCK final(") != std::string::npos);
    CHECK(line.find("✓") == std::string::npos);
}

// Bug caught: the no-data path fabricating "over  0.00\" drift  0.0°" — numbers
// with nothing behind them, the exact lie the brief bans. Final pose still renders
// (it is boundary-read, always real).
TEST_CASE("result line: hasPathData=false renders n/a for the derived fields — never "
          "a fabricated zero") {
    MotionResult r = settledResult();
    r.hasPathData = false;
    r.overshoot = units::Length{0.0};
    r.drift = units::AngleDim{0.0};
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over   n/a  drift  n/a    1.16s\n");
}

// Bug caught: non-finite result values reaching snprintf's locale-varying "nan"/
// "-nan(0x…)" spellings — the deterministic-token contract must hold on THIS
// formatter too, not only on the tick stream (three renderers, one definition).
TEST_CASE("result line: NaN / +Inf / huge magnitudes render the §18.3 tokens") {
    MotionResult r = settledResult();
    r.finalPose = Pose2d{units::Length{std::numeric_limits<double>::infinity()},
                         units::Length{36.0}, Angle::degrees(90.1)};
    r.overshoot = units::Length{std::numeric_limits<double>::quiet_NaN()};
    r.drift = units::AngleDim{std::numeric_limits<double>::quiet_NaN()};
    r.duration = units::Time{1e300};
    const std::string line = render(r);
    CHECK(line.find("final(  +Inf,  36.0,  90.1°)") != std::string::npos);
    CHECK(line.find("over   NaN\" drift  NaN°") != std::string::npos);
    CHECK(line.find("1e+300s") != std::string::npos);  // %.3g compaction, not 300 digits
    CHECK(line.find("nan(") == std::string::npos);
    CHECK(line.find("-nan") == std::string::npos);
}

// Bug caught: a hostile/over-long motion name (a Tier-3 custom IMotion) eating the
// line or splitting the framing — caller text is sanitized and bounded here as
// everywhere.
TEST_CASE("result line: an over-long or hostile name is truncated/sanitized, framing "
          "intact") {
    MotionResult r = settledResult();
    r.name = "AbsurdlyLongCustomTierThreeMotionName\nWithNewline";
    const std::string line = render(r);
    int newlines = 0;
    for (const char c : line) {
        newlines += (c == '\n') ? 1 : 0;
    }
    CHECK(newlines == 1);  // exactly the trailing frame
    CHECK(line.find("…") != std::string::npos);
    CHECK(line.find("WithNewline") == std::string::npos);
}

// Bug caught (DEFECTS1 item A7): MotionResult::outcome defaulted to Settled — the one value
// meaning success — and MotionOutcome had no unknown/unset enumerator to default to instead.
// A result line whose producer forgot the field rendered the checkmark and "SETTLED" for a
// motion that never happened. That is the opposite polarity to this same struct's hasPathData,
// which defaults false precisely so over/drift render "n/a" rather than a fabricated 0.00.
TEST_CASE("A7: an unpopulated result line reports UNSET, not a success") {
    const MotionResult fresh{};
    CHECK(fresh.outcome == MotionOutcome::Unset);
    CHECK(std::string_view{motionOutcomeName(MotionOutcome::Unset)} == "UNSET");
    CHECK(static_cast<std::uint8_t>(MotionOutcome::Unset) == 5);  // APPEND-ONLY: value pinned

    const std::string rendered = render(fresh);
    CHECK(rendered.find("UNSET") != std::string::npos);
    CHECK(rendered.find("✗") != std::string::npos);   // the pessimistic marker
    CHECK(rendered.find("✓") == std::string::npos);   // was the checkmark — the defect

    // NEGATIVE CONTROL: a genuinely settled motion still renders the checkmark, so the change
    // is about the DEFAULT and not about having broken the success rendering.
    CHECK(render(settledResult()).find("✓") != std::string::npos);
}

// ═══ TANK1 Part B — the ` xt` column ════════════════════════════════════════════════════

// Bug caught: the column appearing on a line whose producer never asked for it — a default that
// renders it, or a formatter that decides by the VALUE (crossTrack != 0) instead of the flag. Every
// X-drive and H-drive line, and every pre-2.3 producer, relies on this: the goldens above were
// written before the column existed and are deliberately unedited.
TEST_CASE("xt (TANK1 Part B): absent unless hasCrossTrack — a default MotionResult and a nonzero "
          "crossTrack with the flag off render the §18.3 line byte for byte") {
    const MotionResult fresh{};
    CHECK_FALSE(fresh.hasCrossTrack);
    CHECK(fresh.crossTrack.value() == 0.0);

    MotionResult r = settledResult();
    r.crossTrack = units::Length{-3.25};  // a value with the flag OFF: the flag decides, not the value
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s\n");
}

// Bug caught: the column in the wrong place (before the duration, or anywhere a §18.3 reader's
// field positions would move), the wrong width or precision, a missing inch mark, or the sign
// dropped. The golden is a TANK's version of the §18.3 sample: final (24.1, 36.0) against a
// target (24, 36, 90°) is 0.1 in to the RIGHT of the target heading, so xt is −0.10 (guide 11
// quotes this line).
TEST_CASE("xt (TANK1 Part B): appended after the duration, 6 wide, 2 decimals, signed, with an "
          "inch mark — the golden bytes") {
    MotionResult r = settledResult();
    r.hasCrossTrack = true;
    r.crossTrack = units::Length{-0.10};
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s xt  -0.10\"\n");

    // Width and sign: small positive, zero, a two-digit negative, a full 6-character value.
    struct Case {
        double value;
        const char* tail;
    };
    const Case cases[] = {{0.02, "1.16s xt   0.02\"\n"},
                          {0.0, "1.16s xt   0.00\"\n"},
                          {-12.81, "1.16s xt -12.81\"\n"},
                          {123.456, "1.16s xt 123.46\"\n"},
                          {5.096, "1.16s xt   5.10\"\n"}};
    for (const Case& c : cases) {
        CAPTURE(c.value);
        r.crossTrack = units::Length{c.value};
        const std::string line = render(r);
        const std::string tail{c.tail};
        REQUIRE(line.size() > tail.size());
        CHECK(line.substr(line.size() - tail.size()) == tail);
    }

    // Any outcome: the column follows the duration on a TIMEOUT, a CANCELLED, a SUPERSEDED and a
    // FAULT_ABORT too — the legs it exists for are the ones that did NOT arrive.
    r.crossTrack = units::Length{-5.10};
    r.outcome = MotionOutcome::TimedOut;
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✗TIMEOUT final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s xt  -5.10\"\n");
    r.outcome = MotionOutcome::Cancelled;
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✗CANCELLED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s xt  -5.10\"\n");
    r.outcome = MotionOutcome::Superseded;
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✗SUPERSEDED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s xt  -5.10\"\n");
    r.outcome = MotionOutcome::FaultAbort;
    r.abortFault = FaultCode::OdoStuck;
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✗FAULT_ABORT=ODO_STUCK final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s xt  -5.10\"\n");
}

// Bug caught: a number from the record stream on a line that says the stream did not flow — "over
// n/a drift n/a … xt 0.00" is the fabricated zero the C5 brief bans, one column over. A producer
// that sets hasCrossTrack without hasPathData contradicts itself; the formatter renders the n/a
// line exactly, with no column (the scheduler never produces that pair — CompletedMotion's rule
// needs path data — so this is the formatter's own guard).
TEST_CASE("xt (TANK1 Part B): no path data, no column — hasCrossTrack without hasPathData "
          "renders the n/a line exactly") {
    MotionResult r = settledResult();
    r.hasPathData = false;
    r.overshoot = units::Length{0.0};
    r.drift = units::AngleDim{0.0};
    r.hasCrossTrack = true;
    r.crossTrack = units::Length{0.0};
    CHECK(render(r) ==
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over   n/a  drift  n/a    1.16s\n");
}

// Bug caught: the new column rendering libc's locale spellings for a non-finite value, a
// 300-digit explosion for a huge one, or being CUT OFF — TermSink truncates a MESSAGE (the text
// after "[MOT] ") at 200 bytes, and the column is the last thing on the line. So the longest
// message the formatter can build must still end with the whole column: the 24-byte name bound,
// the largest id, the longest fault name (every code 0-255 is tried, so a longer name added later
// is covered), and every numeric column at its widest. appendNum emits at most 10 bytes a column:
// a %f rendering of up to 10 bytes is kept ("-123456.70"), a longer one is compacted to %.3g, at
// most "-1.23e+300" — so values of both kinds are tried. (Measured: the longest is 175 bytes, at
// -1.23e300 with MOTOR_GROUP_DISAGREE; an earlier version of this case built its "longest" from
// -1e300, whose "-1e+300" is 7 bytes, and so never reached it — the Part B review's finding 3.)
TEST_CASE("xt (TANK1 Part B): NaN / Inf / huge render the §18.3 tokens, and the column survives "
          "the longest possible message") {
    MotionResult r = settledResult();
    r.hasCrossTrack = true;
    r.crossTrack = units::Length{std::numeric_limits<double>::quiet_NaN()};
    std::string line = render(r);
    CHECK(line.substr(line.size() - 13) == "s xt    NaN\"\n");
    CHECK(line.find("nan") == std::string::npos);
    r.crossTrack = units::Length{std::numeric_limits<double>::infinity()};
    line = render(r);
    CHECK(line.substr(line.size() - 13) == "s xt   +Inf\"\n");
    r.crossTrack = units::Length{-1e300};
    line = render(r);
    CHECK(line.substr(line.size() - 14) == "s xt -1e+300\"\n");  // %.3g compaction

    std::size_t longest = 0;
    std::string longestMessage;
    int cut = 0;  // lines that do NOT end with the whole column (TermSink's cut would do that)
    for (const double v : {-1.23e300, -1e300, -999999.99, -123456.7}) {
        shulib::diag::lineformat::Line column;  // the formatter's own rendering of the column
        column.appendLiteral(" xt ");
        shulib::diag::lineformat::appendNum(column, v, 6, 2);
        column.appendLiteral("\"\n");
        const std::string tail{column.view()};
        for (int code = 0; code <= 255; ++code) {
            MotionResult worst = r;
            worst.name = "AbsurdlyLongCustomTierThreeMotionName";
            worst.id = std::numeric_limits<std::uint32_t>::max();
            worst.outcome = MotionOutcome::FaultAbort;
            worst.abortFault = static_cast<FaultCode>(code);
            worst.finalPose = Pose2d{units::Length{v}, units::Length{v}, Angle::degrees(-179.99)};
            worst.overshoot = units::Length{v};
            worst.drift = units::AngleDim{v};
            worst.duration = units::Time{v};
            worst.crossTrack = units::Length{v};
            line = render(worst, v);
            const std::size_t at = line.find("[MOT] ");
            REQUIRE(at != std::string::npos);
            REQUIRE(line.back() == '\n');
            const std::string message = line.substr(at + 6, line.size() - (at + 6) - 1);
            cut += (line.size() < tail.size()
                    || line.compare(line.size() - tail.size(), tail.size(), tail) != 0)
                       ? 1
                       : 0;
            if (message.size() > longest) {
                longest = message.size();
                longestMessage = message;
            }
        }
    }
    MESSAGE("longest result-line message: " << longest << " bytes (TermSink cuts at 200): "
                                            << longestMessage);
    CHECK(cut == 0);        // every one of the 1 024 lines ends with its whole column
    CHECK(longest >= 175);  // non-vacuity: the widest columns were reached (measured 175)
    CHECK(longest <= 200);  // within TermSink's 200 (a longer one is cut, and ends in "…")
    CHECK(longestMessage.find("#4294967295 ✗FAULT_ABORT=") != std::string::npos);
    CHECK(longestMessage.substr(longestMessage.size() - 26) == "-1.23e+300s xt -1.23e+300\"");
}
