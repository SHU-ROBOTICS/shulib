// Tests for diag/run_summary.hpp + TermSink::summarize + the seam's new channel.
// What each targets:
//  * BYTE-EXACT BLOCK: the §18.3 run summary is the one-screen answer to "how did
//    it go" — column drift or a lost field degrades the headline deliverable.
//  * HONESTY: n/a heading when no data; MISSING build hash; drops ALWAYS shown.
//  * ADDITIVITY + DECORATORS: summarize() must be a no-op default (message-only
//    sinks keep compiling) and every shipped decorator must FORWARD it — a
//    decorator with the default body silently eats the run summary.
//  * VALUE SEMANTICS: RunSummary owns bounded copies of its strings — a sink that
//    retains summaries must never hold dangling views.

#include "doctest.h"

#include <limits>
#include <string>

#include "shulib/diag/level_filter_sink.hpp"
#include "shulib/diag/rate_limit_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::FaultCode;
using shulib::diag::LevelFilterSink;
using shulib::diag::RateLimitedSink;
using shulib::diag::RunSummary;
using shulib::diag::TermSink;
using shulib::hal::ITelemetrySink;
using shulib::hal::NullSink;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::motion::CommandIdStampSink;
using shulib::motion::MotionStatsSink;
namespace units = shulib::units;

namespace {
constexpr double kDegToRad = shulib::math::Angle::kPi / 180.0;

RunSummary cleanSummary() {
    RunSummary s;
    s.motionsStarted = 7;
    s.motionsSettled = 6;
    s.motionsTimedOut = 1;
    s.hasHeadingData = true;
    s.headingMax = units::AngleDim{0.7 * kDegToRad};
    s.headingFinal = units::AngleDim{0.3 * kDegToRad};
    s.gatingRejects = 4;
    s.worstLoopDt = units::Time{0.0112};
    s.setBuildHash("a1b2c3d");
    s.setRoutineId("redLeftTall");
    s.batteryStart = units::Voltage{12.4};
    s.batteryEnd = units::Voltage{11.6};
    return s;
}

std::string render(const RunSummary& s) {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};
    term.summarize(s);
    return out.text();
}
}  // namespace

// Bug caught: ANY drift in the §18.3 summary block — the golden that closes the
// "run summary fits one screen" DoD clause. Note the block is UNSTAMPED (a run
// artifact, not a timed event — the §18.3 sketch shows no [t=]).
TEST_CASE("run summary: the §18.3 golden block — clean run") {
    CHECK(render(cleanSummary()) ==
          "── RUN SUMMARY ───────────────────────────────────────────\n"
          " motions 7 · settled 6 · timeout 1 · cancelled 0 · aborted 0\n"
          " heading max  0.7° final  0.3° · gating rejects 4 · brownout no\n"
          " worst loop dt   11.2ms · first fault none · dropped 0 rec 0 ln\n"
          " build a1b2c3d · routine \"redLeftTall\" · batt 12.4→11.6V\n"
          "──────────────────────────────────────────────────────────\n");
}

// Bug caught: the pathological run rendering incompletely — the first fault
// losing its timestamp, brownout reading "no" after a latched collapse, or the
// drop counts vanishing exactly when they matter.
TEST_CASE("run summary: golden block — faulted, browned-out, throttled run") {
    RunSummary s = cleanSummary();
    s.motionsAborted = 1;
    s.brownout = true;
    s.firstFault = FaultCode::OdoStuck;
    s.firstFaultTime = units::Time{4.2};
    s.droppedRecords = 3;
    s.droppedLines = 47;
    CHECK(render(s) ==
          "── RUN SUMMARY ───────────────────────────────────────────\n"
          " motions 7 · settled 6 · timeout 1 · cancelled 0 · aborted 1\n"
          " heading max  0.7° final  0.3° · gating rejects 4 · brownout YES\n"
          " worst loop dt   11.2ms · first fault ODO_STUCK@  4.2s · dropped 3 rec 47 ln\n"
          " build a1b2c3d · routine \"redLeftTall\" · batt 12.4→11.6V\n"
          "──────────────────────────────────────────────────────────\n");
}

// Bug caught: heading fields fabricating " 0.0°" on a run with no heading data
// (nothing ran / records off) — the lying-zero failure class.
TEST_CASE("run summary: no heading data renders n/a, never a fabricated zero") {
    RunSummary s = cleanSummary();
    s.hasHeadingData = false;
    s.headingMax = units::AngleDim{0.0};
    s.headingFinal = units::AngleDim{0.0};
    const std::string text = render(s);
    CHECK(text.find(" heading max  n/a  final  n/a  · ") != std::string::npos);
    CHECK(text.find("0.0°") == std::string::npos);
}

// Bug caught: an empty hash rendering as an empty hole (or a placeholder) instead
// of the loud MISSING token — §18.5's rule applies to the summary too.
TEST_CASE("run summary: an empty build hash renders MISSING") {
    RunSummary s = cleanSummary();
    s.setBuildHash("");
    CHECK(render(s).find(" build MISSING · routine \"redLeftTall\"") != std::string::npos);
}

// Bug caught: a non-finite summary quantity reaching libc's locale spellings —
// the deterministic-token contract holds on the THIRD renderer too.
TEST_CASE("run summary: non-finite values render the deterministic tokens") {
    RunSummary s = cleanSummary();
    s.worstLoopDt = units::Time{std::numeric_limits<double>::quiet_NaN()};
    s.batteryEnd = units::Voltage{-std::numeric_limits<double>::infinity()};
    const std::string text = render(s);
    CHECK(text.find("worst loop dt    NaNms") != std::string::npos);
    CHECK(text.find("12.4→-InfV") != std::string::npos);
    CHECK(text.find("nan") == std::string::npos);
}

// Bug caught: RunSummary retaining views into caller storage (dangling once the
// caller's buffer dies), or the bounded copy overrunning/failing to terminate.
TEST_CASE("RunSummary: provenance strings are bounded owned copies — value semantics") {
    RunSummary s;
    {
        std::string transient(100, 'h');  // longer than the 47-byte hash bound
        s.setBuildHash(transient);
        s.setRoutineId("short");
    }  // the source storage is gone; the summary must not care
    CHECK(s.buildHash().size() == 47);
    CHECK(s.buildHash() == std::string(47, 'h'));
    CHECK(s.routineId() == "short");
    s.setRoutineId("");
    CHECK(s.routineId().empty());
}

// Bug caught: summarize() becoming pure (breaking every message-only sink — the
// F4-additivity regression, same shape as A1's emit() test) or NullSink acquiring
// cost. Also: FakeTelemetrySink must RECORD the channel for every test above the
// unit level.
TEST_CASE("summarize(): additive default no-op; FakeTelemetrySink records the channel") {
    // A sink written before C5 (log-only) must keep compiling and swallow quietly.
    NullSink null;
    ITelemetrySink& seam = null;
    seam.summarize(cleanSummary());  // no-op, no crash — the additivity proof

    FakeTelemetrySink fake;
    CHECK(fake.summaryCount() == 0);
    CHECK_THROWS_AS((void)fake.lastSummary(), shulib::PreconditionError);
    fake.summarize(cleanSummary());
    RunSummary second = cleanSummary();
    second.setRoutineId("skills");
    fake.summarize(second);
    REQUIRE(fake.summaryCount() == 2);
    CHECK(fake.summaryAt(0).routineId() == "redLeftTall");
    CHECK(fake.lastSummary().routineId() == "skills");
    CHECK_THROWS_AS((void)fake.summaryAt(2), shulib::PreconditionError);
    fake.clear();
    CHECK(fake.summaryCount() == 0);
}

// Bug caught: a shipped decorator keeping the DEFAULT no-op summarize — the run
// summary silently dying inside the sink chain (the exact decorator-swallows-it
// hazard the seam documents). Every shipped decorator must forward.
TEST_CASE("summarize(): every shipped decorator forwards — the summary survives the "
          "full chain") {
    FakeTelemetrySink fake;
    FakeClock clock;
    // The realistic full chain: stamp → stats → filter → limiter → sink.
    MotionStatsSink stats{fake};
    CommandIdStampSink stamp{stats};
    LevelFilterSink filter{stamp};
    RateLimitedSink limited{filter, clock};
    limited.summarize(cleanSummary());
    REQUIRE(fake.summaryCount() == 1);
    CHECK(fake.lastSummary().routineId() == "redLeftTall");
}
