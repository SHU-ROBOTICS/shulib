// Tests for diag/level_filter_sink.hpp — D-1, per-subsystem log levels. What each targets:
//  * PRECISION: a level set for one subsystem must filter THAT subsystem and no
//    other — the whole point is watching [MOT] at Debug while [LOC] sits at Warn.
//  * SEVERITY SEMANTICS: "set to Warn" means Warn AND Error pass — inverting the
//    comparison would silence errors, the worst possible failure of a filter.
//  * DATA UNTOUCHED: records and summaries are not chatter; filtering them would
//    blind every downstream consumer because someone quieted a log tag.

#include "doctest.h"

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/level_filter_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"

using shulib::PreconditionError;
using shulib::diag::DebugRecord;
using shulib::diag::LevelFilterSink;
using shulib::diag::RunSummary;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;

// Bug caught: THE D-1 requirement — an override leaking onto other subsystems
// (every tag quieted because one was), or not filtering its own.
TEST_CASE("D-1: a per-subsystem level filters exactly that subsystem and no other") {
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};
    filter.setLevel("LOC", LogLevel::Warn);  // quiet the localizer to Warn+

    filter.log(LogLevel::Info, "LOC", "gone");     // filtered (below Warn)
    filter.log(LogLevel::Debug, "LOC", "gone");    // filtered
    filter.log(LogLevel::Warn, "LOC", "kept-1");   // at threshold: passes
    filter.log(LogLevel::Error, "LOC", "kept-2");  // above: passes
    filter.log(LogLevel::Info, "MOT", "kept-3");   // OTHER tag: untouched
    filter.log(LogLevel::Trace, "SEQ", "kept-4");  // untouched

    REQUIRE(inner.size() == 4);
    CHECK(inner.at(0).message == "kept-1");
    CHECK(inner.at(1).message == "kept-2");
    CHECK(inner.at(2).message == "kept-3");
    CHECK(inner.at(3).message == "kept-4");
}

// Bug caught: the global dial not applying to un-overridden tags, or stomping the
// per-tag overrides it must coexist with (the practical debugging setup: global
// Warn, one tag at Debug).
TEST_CASE("D-1: global level governs un-overridden tags; an override wins over global "
          "in BOTH directions") {
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};
    filter.setGlobalLevel(LogLevel::Warn);
    filter.setLevel("MOT", LogLevel::Debug);  // MORE verbose than global
    filter.setLevel("PWR", LogLevel::Error);  // LESS verbose than global

    filter.log(LogLevel::Info, "LOC", "gone");    // global Warn filters Info
    filter.log(LogLevel::Debug, "MOT", "kept");   // override opens MOT up
    filter.log(LogLevel::Trace, "MOT", "gone");   // …to Debug, not beyond
    filter.log(LogLevel::Warn, "PWR", "gone");    // override closes PWR down
    filter.log(LogLevel::Error, "PWR", "kept");

    REQUIRE(inner.size() == 2);
    CHECK(inner.at(0).subsystem == "MOT");
    CHECK(inner.at(1).subsystem == "PWR");
}

// Bug caught: re-setting a tag appending a DUPLICATE entry (first match wins →
// the dial stops responding), and clearLevels not restoring transparency.
TEST_CASE("D-1: re-setting a tag updates in place; clearLevels restores transparency") {
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};
    filter.setLevel("LOC", LogLevel::Error);
    filter.log(LogLevel::Warn, "LOC", "gone");
    filter.setLevel("LOC", LogLevel::Trace);  // reopen the same tag
    filter.log(LogLevel::Warn, "LOC", "kept");
    filter.clearLevels();
    filter.setGlobalLevel(LogLevel::Trace);
    filter.log(LogLevel::Trace, "LOC", "kept-2");
    REQUIRE(inner.size() == 2);
    CHECK(inner.at(0).message == "kept");
    CHECK(inner.at(1).message == "kept-2");
}

// Bug caught: the filter touching the DATA channels — a record or summary dropped
// because a tag was quieted (filtering is for chatter; data has its own dials),
// or the wantsRecord pair rule broken (population silently stops).
TEST_CASE("D-1: records and summaries pass a filtering sink untouched; wantsRecord "
          "forwards") {
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};
    filter.setGlobalLevel(LogLevel::Error);  // maximally quiet chatter
    CHECK(filter.wantsRecord());             // pair rule: inner (Fake) wants records

    DebugRecord r;
    r.activeCommandId = 9;
    filter.emit(r);
    RunSummary s;
    s.setRoutineId("still-here");
    filter.summarize(s);

    REQUIRE(inner.recordCount() == 1);
    CHECK(inner.lastRecord().activeCommandId == 9);
    REQUIRE(inner.summaryCount() == 1);
    CHECK(inner.lastSummary().routineId() == "still-here");
}

// Bug caught: a silently IGNORED setLevel — a full table or a bad tag must be
// loud (a dial that quietly does nothing is how a debug session chases ghosts).
TEST_CASE("D-1: misuse is loud — bad tags and a full table are preconditions") {
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};
    CHECK_THROWS_AS(filter.setLevel("", LogLevel::Warn), PreconditionError);
    CHECK_THROWS_AS(filter.setLevel("SEVENTEEN-BYTES-X", LogLevel::Warn), PreconditionError);
    for (int i = 0; i < 16; ++i) {
        char tag[8];
        std::snprintf(tag, sizeof tag, "T%d", i);
        filter.setLevel(tag, LogLevel::Warn);  // fills all 16 slots
    }
    filter.setLevel("T3", LogLevel::Error);  // updating an existing tag still fine
    CHECK_THROWS_AS(filter.setLevel("T16", LogLevel::Warn), PreconditionError);
}

// Bug caught (DEFECTS1 item A4): Override::tag() rebuilt a string_view from the NUL-terminated
// buffer, so it stopped at the first NUL. A tag containing an embedded NUL was STORED in full
// but MATCHED only by its truncated prefix: setLevel("A\0B") then setLevel("A") found the
// first slot by its truncated name and overwrote it, while passes("A\0B") compared "A\0B"
// against "A" and failed — the original tag's own dial unreachable by its own name, and
// silently steering a different channel. The class doc promises this table is LOUD about
// every failure mode; that one was silent.
TEST_CASE("A4: a tag with an embedded NUL keeps its own dial (matched by length)") {
    using namespace std::string_view_literals;
    FakeTelemetrySink inner;
    LevelFilterSink filter{inner};

    const auto embedded = "A\0B"sv;
    REQUIRE(embedded.size() == 3);
    filter.setLevel(embedded, LogLevel::Error);   // this tag: errors only
    filter.setLevel("A"sv, LogLevel::Debug);      // a DIFFERENT tag: chatty

    // Two distinct slots, not one overwritten slot: each tag steers only itself.
    filter.log(LogLevel::Info, embedded, "blocked");   // below Error for THIS tag
    filter.log(LogLevel::Debug, "A"sv, "kept-1");      // Debug allowed for "A"
    filter.log(LogLevel::Error, embedded, "kept-2");   // at threshold for the NUL tag

    REQUIRE(inner.size() == 2);
    CHECK(inner.at(0).message == "kept-1");
    CHECK(inner.at(1).message == "kept-2");
}
