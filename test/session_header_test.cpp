// Tests for diag/session_info.hpp + diag/build_info.hpp — the §18.5 session header.
// What each targets:
//  * REPRODUCIBILITY: the header's job is to say WHICH BINARY ran — the hash must
//    appear verbatim, and the plumbing (build system → macro → header) must be intact
//    in THIS very build.
//  * LOUD MISSING: an absent hash must produce an [ERROR] line and the literal token
//    MISSING — never a plausible-looking placeholder (a wrong hash sends the 2am
//    investigation to the wrong commit with full confidence).
//  * FRAMING SAFETY: header fields are caller text; hostile bytes must not break the
//    one-line framing (the legacy escapeJSONString failure class).

#include "doctest.h"

#include <cctype>
#include <string>
#include <string_view>

#include "shulib/diag/build_info.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::SessionInfo;
using shulib::diag::TermSink;
using shulib::diag::compiledBuildHash;
using shulib::diag::emitSessionHeader;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {
SessionInfo fullInfo() {
    return SessionInfo{.buildHash = "a1b2c3d-dirty",
                       .routineId = "redLeftTall",
                       .alliance = "red",
                       .side = "left",
                       .portMap = "L1,2,3 R4,5,6 IMU10"};
}
}  // namespace

// Bug caught: any drift in the §18.5 header shape — a field dropped, an order
// swap, a stamp/tag change — silently degrading the provenance line every future
// log depends on.
TEST_CASE("session header: the §18.5 golden bytes, hash present") {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};
    emitSessionHeader(term, fullInfo(), Voltage{12.4});
    CHECK(out.text() ==
          "[t=   0.00] [SES] run start · build a1b2c3d-dirty · routine \"redLeftTall\"\n"
          "[t=   0.00] [SES] alliance red · side left · batt 12.40V\n"
          "[t=   0.00] [SES] ports L1,2,3 R4,5,6 IMU10\n");
}

// Bug caught: the missing-hash path quietly emitting something plausible (or
// nothing at all) instead of the loud contract: an [ERROR][SES] line FIRST, then
// the literal token MISSING in the header line.
TEST_CASE("session header: a MISSING hash is loud — ERROR first, the MISSING token, "
          "never a plausible placeholder") {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};
    SessionInfo info = fullInfo();
    info.buildHash = {};
    emitSessionHeader(term, info, Voltage{12.4});
    CHECK(out.text() ==
          "[t=   0.00] [ERROR][SES] build hash MISSING — define SHULIB_BUILD_HASH at "
          "build time; a wrong hash is worse than none, so nothing is invented\n"
          "[t=   0.00] [SES] run start · build MISSING · routine \"redLeftTall\"\n"
          "[t=   0.00] [SES] alliance red · side left · batt 12.40V\n"
          "[t=   0.00] [SES] ports L1,2,3 R4,5,6 IMU10\n");
    // The anti-placeholder pin: nothing that LOOKS like a hash may appear.
    CHECK(out.text().find("unknown") == std::string::npos);
    CHECK(out.text().find("0000000") == std::string::npos);
    CHECK(out.text().find("deadbeef") == std::string::npos);
}

// Bug caught: the ERROR line arriving on a message-only sink path with the wrong
// level/tag (a Warn would not read as the failure it is), or the header silently
// skipping fields on the leveled channel.
TEST_CASE("session header: channel semantics — Info lines tagged SES; the missing-hash "
          "line is Error level") {
    FakeTelemetrySink sink;
    SessionInfo info = fullInfo();
    info.buildHash = {};
    emitSessionHeader(sink, info, Voltage{11.9});
    REQUIRE(sink.size() == 4);
    CHECK(sink.at(0).level == LogLevel::Error);
    CHECK(sink.at(0).subsystem == "SES");
    CHECK(sink.at(1).level == LogLevel::Info);
    CHECK(sink.at(2).level == LogLevel::Info);
    CHECK(sink.at(3).level == LogLevel::Info);
    CHECK(sink.at(1).message.find("build MISSING") != std::string::npos);
    CHECK(sink.at(2).message.find("batt 11.90V") != std::string::npos);
}

// Bug caught: empty alliance/side/port fields VANISHING from the line — a blank
// must render as a visible "-" so the column survives and the omission is seen.
TEST_CASE("session header: empty optional fields render '-' — a blank never silently "
          "disappears") {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};
    emitSessionHeader(term, SessionInfo{.buildHash = "abc1234"}, Voltage{12.4});
    CHECK(out.text() ==
          "[t=   0.00] [SES] run start · build abc1234 · routine \"\"\n"
          "[t=   0.00] [SES] alliance - · side - · batt 12.40V\n"
          "[t=   0.00] [SES] ports -\n");
}

// Bug caught: hostile header text (an embedded newline in a routine id read from a
// future file, an over-long port map) breaking the one-line framing or flooding
// the line — the legacy sanitization failure class, at the header's fields.
TEST_CASE("session header: hostile field text is sanitized and bounded") {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};
    SessionInfo info = fullInfo();
    info.routineId = "bad\nroutine";
    const std::string longPorts(300, 'P');
    info.portMap = longPorts;
    emitSessionHeader(term, info, Voltage{12.4});
    // Exactly 3 lines (no injected newline splits a line into four).
    int newlines = 0;
    for (const char c : out.text()) {
        newlines += (c == '\n') ? 1 : 0;
    }
    CHECK(newlines == 3);
    CHECK(out.text().find("bad?routine") != std::string::npos);  // control byte -> '?'
    CHECK(out.text().find("…") != std::string::npos);            // over-long -> truncated
    CHECK(out.text().find(std::string(200, 'P')) == std::string::npos);
}

// Bug caught: the build-system plumbing rotting — the suite building WITHOUT a
// hash (this environment has git, so an empty hash here means the CMake injection
// broke) or the macro not reaching code. This is the "missing is loud" contract
// applied to our own build: the failure happens HERE, visibly, never as a quietly
// hash-less session header down the road.
TEST_CASE("build hash plumbing: THIS build carries a real, sane git identity") {
    const std::string_view hash = compiledBuildHash();
#ifndef SHULIB_BUILD_HASH
    FAIL("SHULIB_BUILD_HASH is not defined — the test build lost its git identity; "
         "fix the CMake injection, do NOT fake a value");
#endif
    REQUIRE_FALSE(hash.empty());
    // Shape sanity: git describe --always [--dirty] output. 7..47 bytes, from the
    // printable describe alphabet (alnum, '.', '-', '_'), and it must NOT be a
    // placeholder someone hard-coded to appease the emptiness check.
    CHECK(hash.size() >= 7);
    CHECK(hash.size() <= 47);
    for (const char c : hash) {
        const bool ok = (std::isalnum(static_cast<unsigned char>(c)) != 0) || c == '.'
                        || c == '-' || c == '_';
        CHECK(ok);
    }
    CHECK(hash != "unknown");
    CHECK(hash != "0000000");
}
