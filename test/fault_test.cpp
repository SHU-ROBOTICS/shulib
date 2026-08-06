// Tests for diag/fault.hpp — the §18.4 fault discipline. What each targets:
//  * NUMERIC STABILITY: the FaultCode values go on the F9 wire and into SdSink files;
//    a silent reorder of the enum must turn this suite red, not re-label history.
//  * FIRST-FAULT LATCHING: a cascade must retain the FIRST fault (the root cause), not
//    the last (the loudest) — the exact bug a "latest fault" implementation would have.
//  * NEVER-CRASH: raise() must survive even a contract-violating (throwing) sink, and
//    raising None must be a no-op, because the error path is the one path that can
//    never be allowed to bring the run down.

#include "doctest.h"

#include <cstdint>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::faultCodeName;
using shulib::hal::ITelemetrySink;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;

TEST_CASE("FaultCode: numeric values are wire-stable (F9) — a reorder turns this red") {
    // These exact numbers are serialized at H1 and written into E1 blackbox files.
    // If this test failed because you reordered/renumbered the enum: DON'T. Append.
    CHECK(static_cast<std::uint16_t>(FaultCode::None) == 0);
    CHECK(static_cast<std::uint16_t>(FaultCode::Precondition) == 1);
    CHECK(static_cast<std::uint16_t>(FaultCode::NanPose) == 2);
    CHECK(static_cast<std::uint16_t>(FaultCode::LoopOverrun) == 3);
    CHECK(static_cast<std::uint16_t>(FaultCode::OdoStuck) == 4);
    CHECK(static_cast<std::uint16_t>(FaultCode::ImuLost) == 5);
    CHECK(static_cast<std::uint16_t>(FaultCode::GpsGateReject) == 6);
    CHECK(static_cast<std::uint16_t>(FaultCode::Brownout) == 7);
    CHECK(static_cast<std::uint16_t>(FaultCode::MotionTimeout) == 8);
    CHECK(static_cast<std::uint16_t>(FaultCode::MotorOverTemp) == 9);  // appended at A3
    // Wire width is part of the freeze: one uint16 slot.
    static_assert(std::is_same_v<std::underlying_type_t<FaultCode>, std::uint16_t>);
}

TEST_CASE("faultCodeName: §18.4 spellings; out-of-range renders, never crashes") {
    CHECK(std::string_view{faultCodeName(FaultCode::None)} == "NONE");
    CHECK(std::string_view{faultCodeName(FaultCode::Precondition)} == "PRECONDITION");
    CHECK(std::string_view{faultCodeName(FaultCode::NanPose)} == "NAN_POSE");
    CHECK(std::string_view{faultCodeName(FaultCode::LoopOverrun)} == "LOOP_OVERRUN");
    CHECK(std::string_view{faultCodeName(FaultCode::OdoStuck)} == "ODO_STUCK");
    CHECK(std::string_view{faultCodeName(FaultCode::ImuLost)} == "IMU_LOST");
    CHECK(std::string_view{faultCodeName(FaultCode::GpsGateReject)} == "GPS_GATE_REJECT");
    CHECK(std::string_view{faultCodeName(FaultCode::Brownout)} == "BROWNOUT");
    CHECK(std::string_view{faultCodeName(FaultCode::MotionTimeout)} == "MOTION_TIMEOUT");
    CHECK(std::string_view{faultCodeName(FaultCode::MotorOverTemp)} == "MOTOR_OVER_TEMP");
    // A corrupted/future code must render as a token, not read off the end of anything.
    CHECK(std::string_view{faultCodeName(static_cast<FaultCode>(999))} == "UNKNOWN");
}

TEST_CASE("FaultLatch: a cascade latches the FIRST fault (root cause), not the last") {
    FakeTelemetrySink sink;
    FakeClock clock{Time{1.0}};
    FaultLatch latch{sink, clock};

    CHECK_FALSE(latch.hasFault());
    CHECK(latch.firstFault() == FaultCode::None);
    CHECK(latch.faultCount() == 0);

    latch.raise(FaultCode::NanPose, "LOC", "pose.x");   // the root cause, t=1.0
    clock.advance(Time{0.5});
    latch.raise(FaultCode::LoopOverrun, "DIAG", "");    // cascade, t=1.5
    latch.raise(FaultCode::ImuLost, "IMU", "");         // cascade

    CHECK(latch.hasFault());
    CHECK(latch.firstFault() == FaultCode::NanPose);        // NOT ImuLost
    CHECK(latch.firstFaultTime().value() == doctest::Approx(1.0));  // NOT 1.5
    CHECK(latch.lastFault() == FaultCode::ImuLost);
    CHECK(latch.faultCount() == 3);
}

TEST_CASE("FaultLatch: every raise logs one Error line; FIRST is marked distinctly") {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};

    latch.raise(FaultCode::NanPose, "LOC", "pose.x");
    latch.raise(FaultCode::GpsGateReject, "EKF", "mahal=9.1");
    latch.raise(FaultCode::Brownout, "PWR", "");  // empty detail: no trailing space

    REQUIRE(sink.size() == 3);
    CHECK(sink.at(0).level == LogLevel::Error);
    CHECK(sink.at(0).subsystem == "LOC");
    CHECK(sink.at(0).message == "fault=NAN_POSE n=1 FIRST pose.x");
    CHECK(sink.at(1).level == LogLevel::Error);
    CHECK(sink.at(1).subsystem == "EKF");
    CHECK(sink.at(1).message == "fault=GPS_GATE_REJECT n=2 mahal=9.1");  // no FIRST marker
    CHECK(sink.at(2).message == "fault=BROWNOUT n=3");
}

TEST_CASE("FaultLatch: raising None is a defensive no-op (the error path never crashes)") {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};

    latch.raise(FaultCode::None, "X", "should be ignored");
    CHECK_FALSE(latch.hasFault());
    CHECK(latch.faultCount() == 0);
    CHECK(sink.empty());

    // And None mid-cascade neither latches nor counts.
    latch.raise(FaultCode::OdoStuck, "ODO", "");
    latch.raise(FaultCode::None, "X", "");
    CHECK(latch.faultCount() == 1);
    CHECK(latch.lastFault() == FaultCode::OdoStuck);
}

TEST_CASE("FaultLatch: clear() opens a new run — the next fault is a new FIRST") {
    FakeTelemetrySink sink;
    FakeClock clock{Time{2.0}};
    FaultLatch latch{sink, clock};

    latch.raise(FaultCode::ImuLost, "IMU", "");
    latch.clear();
    CHECK_FALSE(latch.hasFault());
    CHECK(latch.firstFault() == FaultCode::None);
    CHECK(latch.lastFault() == FaultCode::None);
    CHECK(latch.firstFaultTime().value() == doctest::Approx(0.0));
    CHECK(latch.faultCount() == 0);

    clock.advance(Time{1.0});
    latch.raise(FaultCode::Brownout, "PWR", "");
    CHECK(latch.firstFault() == FaultCode::Brownout);
    CHECK(latch.firstFaultTime().value() == doctest::Approx(3.0));
    CHECK(latch.faultCount() == 1);
}

// Bug caught: the per-code tally (added at C2 for the scheduler's fault
// policy) miscounting — a re-raise the policy must see would be invisible, or
// a stale count would abort a healthy motion. Also pins that clear() resets
// the tallies with the rest of the run state.
TEST_CASE("FaultLatch: raiseCount tallies per code, survives cascades, resets on clear()") {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};

    CHECK(latch.raiseCount(FaultCode::OdoStuck) == 0);
    latch.raise(FaultCode::OdoStuck, "LOC", "first episode");
    latch.raise(FaultCode::Brownout, "PWR", "");
    latch.raise(FaultCode::OdoStuck, "LOC", "second episode");  // the re-raise the
                                                                // C2 policy must see
    CHECK(latch.raiseCount(FaultCode::OdoStuck) == 2);
    CHECK(latch.raiseCount(FaultCode::Brownout) == 1);
    CHECK(latch.raiseCount(FaultCode::ImuLost) == 0);
    CHECK(latch.raiseCount(FaultCode::None) == 0);  // None is never raisable
    latch.raise(FaultCode::None, "X", "");          // …and the no-op stays a no-op
    CHECK(latch.raiseCount(FaultCode::None) == 0);
    // An out-of-tally-range cast must read 0, never crash or index wild.
    CHECK(latch.raiseCount(static_cast<FaultCode>(200)) == 0);

    latch.clear();
    CHECK(latch.raiseCount(FaultCode::OdoStuck) == 0);
    CHECK(latch.raiseCount(FaultCode::Brownout) == 0);
}

namespace {
/// A sink that violates its MUST-NOT-THROW contract — the hostile case raise() must survive.
class ThrowingSink final : public ITelemetrySink {
public:
    void log(LogLevel, std::string_view, std::string_view) override { throw 42; }
};
}  // namespace

TEST_CASE("FaultLatch: a contract-violating (throwing) sink cannot crash raise(); "
          "the latch still latches") {
    ThrowingSink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};

    // raise() is noexcept: if the throw escaped, this would std::terminate the suite —
    // reaching the CHECKs below IS the proof it was swallowed.
    latch.raise(FaultCode::NanPose, "LOC", "hostile sink");
    CHECK(latch.hasFault());
    CHECK(latch.firstFault() == FaultCode::NanPose);
    CHECK(latch.faultCount() == 1);
}
