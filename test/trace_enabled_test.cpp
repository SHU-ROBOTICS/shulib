// Tests for diag/trace.hpp in the DEV configuration: this TU defines
// SHULIB_ENABLE_TRACE before including the header, so SHULIB_TRACE must be a real,
// argument-evaluating call. What this targets:
//  * the enabled path actually LOGS at Trace level with the given tag/message — a strip
//    mechanism that accidentally stripped both configurations would pass every
//    trace_strip_test and silently kill dev tracing; this is the test that catches it.
//  * arguments are evaluated EXACTLY ONCE (a naive macro could double-evaluate).

#define SHULIB_ENABLE_TRACE
#include "doctest.h"

#include <string_view>

#include "shulib/diag/trace.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"

using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;

namespace {
int g_enabledMessageBuilds = 0;
std::string_view buildMessage() {
    ++g_enabledMessageBuilds;
    return "hot-path detail";
}
}  // namespace

TEST_CASE("SHULIB_TRACE (enabled): logs at Trace level, evaluating each argument once") {
    FakeTelemetrySink sink;
    g_enabledMessageBuilds = 0;

    SHULIB_TRACE(sink, "MOT", buildMessage());

    CHECK(g_enabledMessageBuilds == 1);  // exactly once — no double evaluation
    REQUIRE(sink.size() == 1);
    CHECK(sink.at(0).level == LogLevel::Trace);
    CHECK(sink.at(0).subsystem == "MOT");
    CHECK(sink.at(0).message == "hot-path detail");
}
