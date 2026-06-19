// Tests for the ITelemetrySink seam: NullSink drops silently; FakeTelemetrySink
// records leveled/tagged messages in order and bounds-checks access (so a test that
// asserts "fault X was logged" can't read past the end and pass by accident).

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"

using shulib::PreconditionError;
using shulib::hal::ITelemetrySink;
using shulib::hal::LogLevel;
using shulib::hal::NullSink;
using shulib::hal::fake::FakeTelemetrySink;

TEST_CASE("NullSink: drops everything and never throws, through the interface") {
    NullSink s;
    ITelemetrySink& sink = s;
    sink.log(LogLevel::Error, "TEST", "anything");  // no observable effect, no throw
    CHECK(true);
}

TEST_CASE("FakeTelemetrySink: records leveled, subsystem-tagged messages in order") {
    FakeTelemetrySink s;
    CHECK(s.empty());

    s.log(LogLevel::Warn, "SEQ", "retry 1/3");
    s.log(LogLevel::Error, "EKF", "gps reject");

    REQUIRE(s.size() == 2);
    CHECK(s.at(0).level == LogLevel::Warn);
    CHECK(s.at(0).subsystem == "SEQ");
    CHECK(s.at(0).message == "retry 1/3");
    CHECK(s.last().level == LogLevel::Error);
    CHECK(s.last().subsystem == "EKF");
    CHECK(s.last().message == "gps reject");
}

TEST_CASE("FakeTelemetrySink: access is bounds-checked; last() on empty throws") {
    FakeTelemetrySink s;
    CHECK_THROWS_AS((void)s.last(), PreconditionError);  // nothing logged yet

    s.log(LogLevel::Info, "A", "x");
    CHECK_NOTHROW((void)s.at(0));
    CHECK_THROWS_AS((void)s.at(-1), PreconditionError);
    CHECK_THROWS_AS((void)s.at(1), PreconditionError);
}

TEST_CASE("FakeTelemetrySink: usable through the interface; clear resets history") {
    FakeTelemetrySink s;
    ITelemetrySink& sink = s;
    sink.log(LogLevel::Debug, "MOT", "tick");
    CHECK(s.size() == 1);
    s.clear();
    CHECK(s.empty());
}
