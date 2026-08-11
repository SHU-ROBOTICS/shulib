// Tests for diag/rate_limit_sink.hpp — D-2, per-channel rate limiting. What each targets:
//  * NOTHING SILENT: under flood, drops must OCCUR, be COUNTED, be STAMPED onto the
//    surviving records, and be ANNOUNCED when a line channel resumes — a silent drop
//    reads as "nothing happened", the failure D-2 exists to prevent.
//  * THE ERROR PATH IS SACRED: Error/Warn lines and the run summary must NEVER be
//    throttled — a throttled fault line is a lost root cause.
//  * PER-CHANNEL ISOLATION: one chatty tag must not starve another.
//  * DETERMINISM: token refill runs on the injected clock — every case is exact.

#include "doctest.h"

#include <string>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/rate_limit_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::DebugRecord;
using shulib::diag::RateLimitConfig;
using shulib::diag::RateLimitedSink;
using shulib::diag::RunSummary;
using shulib::hal::LogLevel;
using shulib::hal::NullSink;
using shulib::hal::emitRecord;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;

// Bug caught: THE D-2 record-channel requirement — a flood dropping records
// SILENTLY: not counted, or counted but never visible on the wire. The stamped
// cumulative counters on every surviving record are the wire-level receipt.
TEST_CASE("D-2: a record flood drops, the drops are counted, and every surviving "
          "record carries the cumulative counts on the wire") {
    FakeTelemetrySink inner;
    FakeClock clock;
    RateLimitedSink limited{inner, clock, RateLimitConfig{.recordsPerSecond = 5.0}};

    // Burst capacity = one second's budget (5): the 6th..10th record this instant drop.
    for (int i = 0; i < 10; ++i) {
        DebugRecord r;
        r.activeCommandId = static_cast<std::uint32_t>(i + 1);
        limited.emit(r);
    }
    CHECK(inner.recordCount() == 5);
    CHECK(limited.droppedRecords() == 5);
    CHECK(inner.lastRecord().activeCommandId == 5);  // first-five-pass, not last-five
    CHECK(inner.lastRecord().droppedRecords == 0);   // nothing had dropped YET at #5

    // Refill: 0.4 s at 5/s = 2 tokens. Two more pass, stamped with the 5 drops.
    clock.advance(Time{0.4});
    for (int i = 0; i < 3; ++i) {
        limited.emit(DebugRecord{});
    }
    CHECK(inner.recordCount() == 7);
    CHECK(limited.droppedRecords() == 6);            // the 3rd of the batch dropped
    CHECK(inner.lastRecord().droppedRecords == 5);   // the wire explains its own gap
}

// Bug caught: line-channel drops that never ANNOUNCE themselves — the notice on
// resume is what tells a human scrolling the log that a gap is throttling, not
// silence from the robot.
TEST_CASE("D-2: a line flood is counted and announced with ONE notice when the "
          "channel resumes") {
    FakeTelemetrySink inner;
    FakeClock clock;
    RateLimitedSink limited{inner, clock,
                            RateLimitConfig{.linesPerSecondPerChannel = 2.0}};

    limited.log(LogLevel::Info, "MOT", "a");  // passes (burst 2)
    limited.log(LogLevel::Info, "MOT", "b");  // passes
    for (int i = 0; i < 47; ++i) {
        limited.log(LogLevel::Info, "MOT", "flood");  // all dropped
    }
    CHECK(inner.size() == 2);
    CHECK(limited.droppedLines() == 47);

    clock.advance(Time{1.0});                     // 2 tokens back
    limited.log(LogLevel::Info, "MOT", "resumed");
    REQUIRE(inner.size() == 4);                   // notice + the resuming line
    CHECK(inner.at(2).level == LogLevel::Warn);
    CHECK(inner.at(2).subsystem == "DIA");
    CHECK(inner.at(2).message == "throttled MOT: dropped 47 lines");
    CHECK(inner.at(3).message == "resumed");
    // Exactly ONE notice per episode — the next passing line has no echo.
    limited.log(LogLevel::Info, "MOT", "again");
    CHECK(inner.size() == 5);
    CHECK(inner.at(4).message == "again");
}

// Bug caught: the error path throttled — a FaultLatch line (Error) or a scheduler
// abort Warn eaten by a token bucket exactly when everything is going wrong and
// every line matters most.
TEST_CASE("D-2: Error and Warn lines are NEVER throttled, even mid-flood") {
    FakeTelemetrySink inner;
    FakeClock clock;
    RateLimitedSink limited{inner, clock,
                            RateLimitConfig{.linesPerSecondPerChannel = 1.0}};
    limited.log(LogLevel::Info, "MOT", "takes-the-token");
    for (int i = 0; i < 20; ++i) {
        limited.log(LogLevel::Error, "MOT", "fault line");
        limited.log(LogLevel::Warn, "MOT", "warn line");
    }
    CHECK(inner.size() == 41);  // 1 info + all 40 exempt lines
    CHECK(limited.droppedLines() == 0);
}

// Bug caught: channel cross-talk — one chatty tag draining a SHARED bucket and
// silencing an innocent tag (the per-channel in "per-channel rate limiting").
TEST_CASE("D-2: per-tag isolation — flooding MOT does not throttle LOC") {
    FakeTelemetrySink inner;
    FakeClock clock;
    RateLimitedSink limited{inner, clock,
                            RateLimitConfig{.linesPerSecondPerChannel = 3.0}};
    for (int i = 0; i < 50; ++i) {
        limited.log(LogLevel::Info, "MOT", "flood");
    }
    limited.log(LogLevel::Info, "LOC", "independent");
    CHECK(inner.at(inner.size() - 1).message == "independent");
    CHECK(limited.droppedLines() == 47);  // 50 - MOT's burst of 3; LOC dropped none
}

// Bug caught: the summary channel acquiring a bucket — the one-per-run block
// dying in the decorator exactly at the end of a heavily-throttled (= most in
// need of a summary) run.
TEST_CASE("D-2: summarize() is never throttled") {
    FakeTelemetrySink inner;
    FakeClock clock;
    RateLimitedSink limited{inner, clock, RateLimitConfig{.recordsPerSecond = 1.0}};
    for (int i = 0; i < 10; ++i) {
        limited.emit(DebugRecord{});  // exhaust everything
        RunSummary s;
        limited.summarize(s);
    }
    CHECK(inner.summaryCount() == 10);
}

// Bug caught: the pair rule broken one level up — wantsRecord() answering true
// over a NullSink inner (population cost paid for nothing) or false over a
// consuming inner (records never built, drops never counted, D-2 blind).
TEST_CASE("D-2: wantsRecord forwards the inner sink's answer; NullSink chains stay "
          "free") {
    FakeClock clock;
    NullSink null;
    RateLimitedSink overNull{null, clock};
    CHECK_FALSE(overNull.wantsRecord());
    int builds = 0;
    emitRecord(overNull, [&] {
        ++builds;
        return DebugRecord{};
    });
    CHECK(builds == 0);  // the competition chain never even populates

    FakeTelemetrySink fake;
    RateLimitedSink overFake{fake, clock};
    CHECK(overFake.wantsRecord());
}

// Bug caught: nonsense budgets accepted silently (a 0 rate = a sink that drops
// everything forever, quietly — the exact opposite of this decorator's contract).
TEST_CASE("D-2: nonsense budgets are loud preconditions") {
    FakeTelemetrySink inner;
    FakeClock clock;
    CHECK_THROWS_AS((RateLimitedSink{inner, clock, RateLimitConfig{.recordsPerSecond = 0.0}}),
                    PreconditionError);
    CHECK_THROWS_AS(
        (RateLimitedSink{inner, clock, RateLimitConfig{.linesPerSecondPerChannel = -1.0}}),
        PreconditionError);
}
