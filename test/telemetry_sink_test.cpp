// Tests for the ITelemetrySink seam: NullSink drops silently; FakeTelemetrySink
// records leveled/tagged messages in order and bounds-checks access (so a test that
// asserts "fault X was logged" can't read past the end and pass by accident).
//
// Since chunk A1 this file also pins the seam's ADDITIVITY contract: emit() is
// non-pure with a default no-op body, so a sink implementing only log() must keep
// compiling and running forever. The MessageOnlySink case below is the test that
// would go red if emit() (or wantsRecord()) ever became pure virtual — the exact
// F4-breaking regression the header warns against.

#include "doctest.h"

#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"

using shulib::PreconditionError;
using shulib::diag::DebugRecord;
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

namespace {
/// A sink written against the M1 seam: log() ONLY — no emit(), no wantsRecord().
/// Its very COMPILATION is the additivity proof; a pure-virtual emit() would stop
/// this file building (the F4-safety regression the seam header forbids).
class MessageOnlySink final : public ITelemetrySink {
public:
    void log(LogLevel, std::string_view, std::string_view message) override {
        lastMessage = std::string_view{message};
    }
    std::string_view lastMessage;
};
}  // namespace

TEST_CASE("Additivity (F4 safety): a sink implementing ONLY log() compiles, runs, and "
          "inherits the record defaults — no records wanted, emit() a harmless no-op") {
    MessageOnlySink s;
    ITelemetrySink& sink = s;

    sink.log(LogLevel::Info, "MOT", "still works");
    CHECK(s.lastMessage == "still works");

    CHECK_FALSE(sink.wantsRecord());  // the default matches the default no-op emit()
    const DebugRecord r{};
    sink.emit(r);                     // must be callable and do nothing — no throw
    CHECK(s.lastMessage == "still works");
}

TEST_CASE("NullSink: wants no records and drops an emitted record silently, through "
          "the interface") {
    NullSink s;
    ITelemetrySink& sink = s;
    CHECK_FALSE(sink.wantsRecord());
    const DebugRecord r{};
    sink.emit(r);  // no observable effect, no throw
    CHECK(true);
}
