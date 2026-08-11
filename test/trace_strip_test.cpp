// Tests for diag/trace.hpp in its DEFAULT (competition) configuration: this TU does NOT
// define SHULIB_ENABLE_TRACE, so SHULIB_TRACE must be compiled out. What this targets:
//  * THE ZERO-COST PROOF (§18.3): the macro's argument expressions must never be
//    EVALUATED — a side-effecting message builder must not run and the sink must see
//    nothing. This is stronger than "the sink drops it": a runtime level-check would
//    pass a sink-sees-nothing test while still paying to build the message every tick.
//  * Arguments remain TYPE-CHECKED even when stripped (the `true ? … : …` mechanism):
//    this file compiles the stripped form against real sink/args, proving a trace call
//    cannot bit-rot in competition builds.
// The enabled counterpart lives in trace_enabled_test.cpp (which defines the flag
// before including the header — the flag is per-TU at preprocessing time, which is what
// lets one binary prove both behaviors).

#include "doctest.h"

#include <string_view>

#include "shulib/diag/trace.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"

using shulib::hal::fake::FakeTelemetrySink;

namespace {
int g_messageBuilds = 0;
std::string_view expensiveMessage() {
    ++g_messageBuilds;  // stands in for the real cost: formatting on the hot path
    return "expensive";
}
int g_sinkLookups = 0;
FakeTelemetrySink& lookupSink(FakeTelemetrySink& s) {
    ++g_sinkLookups;
    return s;
}
}  // namespace

TEST_CASE("SHULIB_TRACE (stripped): argument expressions are NEVER evaluated — the "
          "compile-time strip is real, not a runtime skip") {
    FakeTelemetrySink sink;
    g_messageBuilds = 0;
    g_sinkLookups = 0;

    SHULIB_TRACE(lookupSink(sink), "MOT", expensiveMessage());

    CHECK(g_messageBuilds == 0);  // the message was never built…
    CHECK(g_sinkLookups == 0);    // …the sink expression never ran…
    CHECK(sink.empty());          // …and nothing reached the sink
}

TEST_CASE("SHULIB_TRACE (stripped): safe in single-statement contexts (an unbraced if) — "
          "the expansion is one expression, not a naked block") {
    FakeTelemetrySink sink;
    g_messageBuilds = 0;
    const bool condition = true;
    if (condition)
        SHULIB_TRACE(sink, "MOT", expensiveMessage());
    else
        sink.log(shulib::hal::LogLevel::Info, "MOT", "else-arm intact");
    CHECK(g_messageBuilds == 0);
    CHECK(sink.empty());  // neither arm fired: the strip didn't eat the else
}
