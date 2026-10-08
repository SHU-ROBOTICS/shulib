// log_copy_line_test.cpp — the ONE boot line that says where a program's SD-card copy of its log is,
// or that it did not open (src/common/log_copy_line.hpp; chunk T4 Part 3, the team lead's N5 (a),
// A (a) and B (a)). "shulib Drive" (a new file each boot) and the competition program (APPENDED
// across boots) both say it; their source pins (test/drive_program_source_test.cpp,
// test/competition_source_test.cpp) prove each calls this with what its provider reports. Every case
// names the bug it would catch.

#include "doctest.h"

#include <cstdint>
#include <string>

#include "common/log_copy_line.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/ui/touch_panel.hpp"

using shulib::bench::kLogCopyLineMaxBytes;
using shulib::bench::kRefusalPieceCols;
using shulib::bench::LogCopyLine;
using shulib::bench::logBootRefusal;
using shulib::bench::logCopyBootLine;
using shulib::bench::LogCopyOpen;
using shulib::bench::logCopyOpenFrom;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;

// (LCL1) Bug caught: the boot claiming a copy that never opened (a practice run with no card would
// be believed recorded), a refusing card said as an Info line (missed in a scroll), the mode said
// wrong (an appended file believed fresh: a reader would take an earlier boot's cut as this one's),
// or the line promising per-tick records the copy never holds (A (a): events and summaries only).
TEST_CASE("log copy line (LCL1): each way the copy opened, in its own words") {
    const LogCopyLine fresh = logCopyBootLine(LogCopyOpen::Truncated, "/usd/drive_log.txt", 0);
    CHECK(std::string{fresh.text}
          == "log copy: /usd/drive_log.txt (one file per boot) -- its events and summaries, no "
             "per-tick record, from the provider on (earlier lines: serial only)");
    CHECK_FALSE(fresh.warn);
    const LogCopyLine created = logCopyBootLine(LogCopyOpen::Created, "/usd/comp_log.txt", 0);
    CHECK(std::string{created.text}.rfind(
              "log copy: /usd/comp_log.txt (CREATED, kept across boots) -- its events", 0)
          == 0);
    CHECK_FALSE(created.warn);
    const LogCopyLine appended = logCopyBootLine(LogCopyOpen::Appended, "/usd/comp_log.txt", 0);
    CHECK(std::string{appended.text}.rfind(
              "log copy: /usd/comp_log.txt (APPENDED, kept across boots) -- its events", 0)
          == 0);
    CHECK_FALSE(appended.warn);
    // Did not open: WHY, in the words a person holding the card needs (the T4-3 review, re-pinned
    // 2026-10-08) -- no card at all, or a card that is in and would still not take the file.
    const LogCopyLine noCard = logCopyBootLine(LogCopyOpen::NoCard, "/usd/drive_log.txt", 7);
    CHECK(std::string{noCard.text}
          == "log copy: /usd/drive_log.txt did NOT open: NO SD CARD -- serial only; 7 line(s) lost "
             "so far");
    CHECK(noCard.warn);
    const LogCopyLine none = logCopyBootLine(LogCopyOpen::NotOpen, "/usd/drive_log.txt", 7);
    CHECK(std::string{none.text}
          == "log copy: /usd/drive_log.txt did NOT open, though an SD card is in (full? "
             "write-protected? a folder of that name?) -- serial only; 7 line(s) lost so far");
    CHECK(none.warn);
    CHECK(std::string{none.text}.find("NO SD CARD") == std::string::npos);
    for (const LogCopyLine* l : {&fresh, &created, &appended}) {
        CHECK(std::string{l->text}.find("per-tick record") != std::string::npos);
        CHECK(std::string{l->text}.find("did NOT open") == std::string::npos);
    }
}

// (LCL2) Bug caught: a line TermSink cuts at 200 bytes (the reader loses where the copy is), a
// null path crashing the boot, or a character the terminal or the screen rules cannot carry — at
// the worst case: a path longer than the line prints, the largest drop count.
TEST_CASE("log copy line (LCL2): every line within 200 bytes, ASCII, null-safe, at the worst case") {
    const std::string longPath = "/usd/" + std::string(300, 'x');
    for (const LogCopyOpen how : {LogCopyOpen::NotOpen, LogCopyOpen::NoCard, LogCopyOpen::Truncated,
                                  LogCopyOpen::Created, LogCopyOpen::Appended}) {
        for (const char* path : {longPath.c_str(), static_cast<const char*>(nullptr), ""}) {
            const LogCopyLine l = logCopyBootLine(how, path, UINT32_MAX);
            CAPTURE(l.text);
            CHECK(std::string{l.text}.size() <= kLogCopyLineMaxBytes);
            CHECK(shulib::ui::screenText(l.text, kLogCopyLineMaxBytes));
            CHECK(std::string{l.text}.rfind("log copy: ", 0) == 0);
        }
    }
    // The path is cut at 40 characters, never past it.
    const LogCopyLine cut = logCopyBootLine(LogCopyOpen::Truncated, longPath.c_str(), 0);
    CHECK(std::string{cut.text}.rfind("log copy: " + longPath.substr(0, 40) + " (", 0) == 0);
}

// (LCL3) Bug caught (the T4-3 review): a copy that failed to open with the card IN said as "no card"
// (the person holding it is sent to look for a missing card), the reverse, or the two programs'
// mappings drifting (Drive truncates, the competition program appends) -- one function maps both.
TEST_CASE("log copy line (LCL3): the open state from what the provider and the card say") {
    CHECK(logCopyOpenFrom(false, false, false, false) == LogCopyOpen::NoCard);
    CHECK(logCopyOpenFrom(false, true, false, false) == LogCopyOpen::NotOpen);
    CHECK(logCopyOpenFrom(false, true, true, true) == LogCopyOpen::NotOpen);
    CHECK(logCopyOpenFrom(false, false, true, false) == LogCopyOpen::NoCard);
    CHECK(logCopyOpenFrom(true, true, false, false) == LogCopyOpen::Truncated);
    CHECK(logCopyOpenFrom(true, true, false, true) == LogCopyOpen::Truncated);  // Truncate never creates
    CHECK(logCopyOpenFrom(true, true, true, true) == LogCopyOpen::Created);
    CHECK(logCopyOpenFrom(true, true, true, false) == LogCopyOpen::Appended);
}

// (LCL4) Bug caught (the T4-3 review): a boot refused AFTER the provider opened the copy that never
// says why on the card (an empty drive_log.txt, the reason on serial only); a refusal line TermSink
// cuts at 200 bytes (the reason's end lost); a refusal said below ERROR; a word split or dropped.
TEST_CASE("log copy line (LCL4): a boot refusal through the provider's sink -- ERROR, whole, in "
          "pieces TermSink never cuts") {
    FakeTelemetrySink sink;
    logBootRefusal(sink, "DRV", "the driver file REFUSED this robot", "port 6 is not a motor");
    REQUIRE(sink.size() == 1);
    CHECK(sink.at(0).level == LogLevel::Error);
    CHECK(sink.at(0).subsystem == "DRV");
    CHECK(sink.at(0).message
          == "REFUSED: the driver file REFUSED this robot -- port 6 is not a motor");

    // The worst case the programs can hand in: a 63-character headline and a 191-character reason.
    const std::string headline(63, 'h');
    std::string why;
    for (int i = 0; i < 24; ++i) {
        why += "word" + std::to_string(i % 10) + "xx ";  // 8 characters a word
    }
    why.resize(191);
    FakeTelemetrySink pieces;
    logBootRefusal(pieces, "CMP", headline.c_str(), why.c_str());
    REQUIRE(pieces.size() >= 2);
    std::string joined;
    for (int i = 0; i < pieces.size(); ++i) {
        const std::string& m = pieces.at(i).message;
        CAPTURE(m);
        CHECK(pieces.at(i).level == LogLevel::Error);
        CHECK(m.size() <= kRefusalPieceCols + 4);
        CHECK(m.size() <= 200U);
        CHECK(shulib::ui::screenText(m, 200));
        if (i > 0) {
            REQUIRE(m.rfind("... ", 0) == 0);
            joined += " " + m.substr(4);
        } else {
            joined += m;
        }
    }
    std::string expected = "REFUSED: " + headline + " -- " + why;
    while (!expected.empty() && expected.back() == ' ') {
        expected.pop_back();
    }
    CHECK(joined == expected);  // every word, in order, none split

    // Null texts print as "" and never crash.
    FakeTelemetrySink nulls;
    logBootRefusal(nulls, "DRV", nullptr, nullptr);
    REQUIRE(nulls.size() == 1);
    CHECK(nulls.at(0).message == "REFUSED: --");  // wrapWords collapses the empty fields' spaces
}

// (LCL5) Bug caught: a reason holding one word longer than a log piece (an exception's text with
// no spaces -- the competition program hands its raw reason in) losing the rest of the refusal
// SILENTLY, or the word split, or cut by TermSink. The words before it are said whole, then one
// line that says the rest did not fit; every line ERROR.
TEST_CASE("log copy line (LCL5): a word longer than a piece stops the refusal, and says so") {
    FakeTelemetrySink sink;
    const std::string word(220, 'x');
    const std::string why = "bad: " + word + " never reached";
    logBootRefusal(sink, "CMP", "the competition program REFUSED to start", why.c_str());
    REQUIRE(sink.size() == 2);
    CHECK(sink.at(0).message == "REFUSED: the competition program REFUSED to start -- bad:");
    CHECK(sink.at(1).message == "... (the rest of this refusal did not fit three log lines)");
    for (int i = 0; i < sink.size(); ++i) {
        CHECK(sink.at(i).level == LogLevel::Error);
        CHECK(sink.at(i).subsystem == "CMP");
        CHECK(sink.at(i).message.size() <= 200U);
        CHECK(sink.at(i).message.find(word.substr(0, 20)) == std::string::npos);  // never split in
    }
}
