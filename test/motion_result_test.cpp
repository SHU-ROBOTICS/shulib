// Tests for diag/motion_result.hpp — the §18.3 per-motion result line. What each targets:
//  * WIRE STABILITY: MotionOutcome numbers are §18.4's boundary vocabulary; a reorder
//    re-labels history.
//  * BYTE-EXACT SHAPE: the line is the §18.3 target; column drift or a token change
//    breaks the at-a-glance discipline the section exists for.
//  * HONESTY: the n/a path must render n/a — a fabricated 0.00 where no data flowed
//    is the lying-number failure the C5 brief bans outright.
//  * UGLY VALUES: NaN/±Inf/1e300 must render as the deterministic §18.3 tokens, not
//    libc's locale spellings or a 300-digit column explosion.

#include "doctest.h"

#include <cstdint>
#include <string>
#include <string_view>

#include "shulib/diag/fault.hpp"
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
