// Tests for diag/health_monitor.hpp — the A3 pathology→fault policy. What each targets:
//  * EDGE TRIGGERING: a persistent pathology is ONE episode, not one fault per tick —
//    the anti-spam contract that keeps the first-fault latch legible.
//  * THE BOOT RULE: never-ready IMU (calibration) is NOT a loss; ready→lost IS.
//  * HYSTERESIS: a pack chattering around the brownout threshold cannot re-trip until
//    it genuinely recovers; the brownedOut() marker stays latched for the run.
//  * LATCH INTERACTION: episodes cascade into the A1 latch without usurping the first
//    fault (root-cause discipline survives the monitor).

#include "doctest.h"

#include <limits>
#include <string>

#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::HealthMonitor;
using shulib::diag::HealthMonitorConfig;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {
struct Rig {
    FakeTelemetrySink sink;
    FakeClock clock{Time{1.0}};
    FaultLatch latch{sink, clock};
    HealthMonitor monitor{latch};
};
}  // namespace

TEST_CASE("HealthMonitor: a healthy tick raises nothing") {
    Rig r;
    for (int i = 0; i < 100; ++i) {
        r.monitor.tick({});  // all defaults are the healthy values
    }
    CHECK_FALSE(r.latch.hasFault());
    CHECK(r.sink.empty());
}

TEST_CASE("HealthMonitor: never-ready IMU (boot calibration) is not a loss") {
    Rig r;
    HealthMonitor::Observations o;
    o.imuReady = false;  // calibrating from t=0
    for (int i = 0; i < 200; ++i) {
        r.monitor.tick(o);
    }
    CHECK_FALSE(r.latch.hasFault());
    CHECK_FALSE(r.monitor.imuLost());
}

TEST_CASE("HealthMonitor: ready→lost raises IMU_LOST once per episode, re-arms on recovery") {
    Rig r;
    HealthMonitor::Observations ok;
    HealthMonitor::Observations lost;
    lost.imuReady = false;

    r.monitor.tick(ok);  // seen ready
    for (int i = 0; i < 50; ++i) {
        r.monitor.tick(lost);  // one persistent episode
    }
    CHECK(r.latch.faultCount() == 1);  // ONE fault, not 50
    CHECK(r.latch.firstFault() == FaultCode::ImuLost);
    CHECK(r.monitor.imuLost());

    r.monitor.tick(ok);  // recovery re-arms
    CHECK_FALSE(r.monitor.imuLost());
    r.monitor.tick(lost);  // a NEW episode
    CHECK(r.latch.faultCount() == 2);
}

TEST_CASE("HealthMonitor: implausible odometry and the stall cross-check raise ODO_STUCK") {
    Rig r;
    SUBCASE("implausible tick") {
        HealthMonitor::Observations o;
        o.odomImplausible = true;
        r.monitor.tick(o);
        r.monitor.tick(o);
        CHECK(r.latch.faultCount() == 1);
        CHECK(r.latch.firstFault() == FaultCode::OdoStuck);
        CHECK(r.sink.last().message.find("implausible") != std::string::npos);
    }
    SUBCASE("caller-computed stall (frozen encoder containment)") {
        HealthMonitor::Observations o;
        o.odomStalled = true;
        r.monitor.tick(o);
        CHECK(r.latch.firstFault() == FaultCode::OdoStuck);
        CHECK(r.sink.last().message.find("no motion") != std::string::npos);
    }
}

TEST_CASE("HealthMonitor: a gated fix raises GPS_GATE_REJECT per episode") {
    Rig r;
    HealthMonitor::Observations o;
    o.fixGated = true;
    for (int i = 0; i < 10; ++i) {
        r.monitor.tick(o);
    }
    CHECK(r.latch.faultCount() == 1);
    CHECK(r.latch.firstFault() == FaultCode::GpsGateReject);
    o.fixGated = false;
    r.monitor.tick(o);
    o.fixGated = true;
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 2);
}

TEST_CASE("HealthMonitor: brownout trips at the threshold, latches, and uses hysteresis") {
    Rig r;
    HealthMonitor::Observations o;

    o.batteryVolts = Voltage{10.6};  // above 10.5: healthy
    r.monitor.tick(o);
    CHECK_FALSE(r.latch.hasFault());
    CHECK_FALSE(r.monitor.brownedOut());

    o.batteryVolts = Voltage{10.5};  // AT the threshold: trips (inclusive)
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 1);
    CHECK(r.latch.firstFault() == FaultCode::Brownout);
    CHECK(r.monitor.brownedOut());
    CHECK(r.sink.last().message.find("battery=10.50V") != std::string::npos);

    // Chatter INSIDE the hysteresis band must NOT start a new episode…
    o.batteryVolts = Voltage{10.6};  // above trip, below recover (10.8)
    r.monitor.tick(o);
    o.batteryVolts = Voltage{10.4};
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 1);  // still the one episode

    // …but a genuine recovery re-arms, and a second collapse is a second episode.
    o.batteryVolts = Voltage{11.5};
    r.monitor.tick(o);
    o.batteryVolts = Voltage{10.2};
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 2);
    CHECK(r.monitor.brownedOut());  // the marker never un-latches within a run
}

TEST_CASE("HealthMonitor: motor over-temp trips at 55C inclusive, once per episode") {
    Rig r;
    HealthMonitor::Observations o;
    o.maxMotorTempC = 54.9;
    r.monitor.tick(o);
    CHECK_FALSE(r.latch.hasFault());
    o.maxMotorTempC = 55.0;  // AT the documented throttle step
    r.monitor.tick(o);
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 1);
    CHECK(r.latch.firstFault() == FaultCode::MotorOverTemp);
    CHECK(r.sink.last().message.find("temp=55.0C") != std::string::npos);
    o.maxMotorTempC = 40.0;  // cooled → re-armed
    r.monitor.tick(o);
    o.maxMotorTempC = 60.0;
    r.monitor.tick(o);
    CHECK(r.latch.faultCount() == 2);
}

TEST_CASE("HealthMonitor: cascades never usurp the first fault (root-cause discipline)") {
    Rig r;
    HealthMonitor::Observations o;
    o.imuReady = true;
    r.monitor.tick(o);

    o.imuReady = false;  // root cause: IMU lost…
    r.monitor.tick(o);
    o.odomImplausible = true;            // …then odometry degrades…
    o.batteryVolts = Voltage{10.0};      // …then the battery collapses
    r.monitor.tick(o);

    CHECK(r.latch.firstFault() == FaultCode::ImuLost);  // the 2am answer
    CHECK(r.latch.faultCount() == 3);
    CHECK(r.latch.lastFault() == FaultCode::Brownout);
}

TEST_CASE("HealthMonitor: reset() opens a new run (episodes, marker, boot rule)") {
    Rig r;
    HealthMonitor::Observations o;
    o.imuReady = true;
    r.monitor.tick(o);
    o.imuReady = false;
    o.batteryVolts = Voltage{9.0};
    r.monitor.tick(o);
    CHECK(r.monitor.brownedOut());
    CHECK(r.latch.faultCount() == 2);

    r.monitor.reset();
    r.latch.clear();
    CHECK_FALSE(r.monitor.brownedOut());
    // Post-reset, a not-ready IMU is a BOOT window again, not a loss.
    o.batteryVolts = Voltage{12.6};
    r.monitor.tick(o);
    CHECK_FALSE(r.latch.hasFault());
}

TEST_CASE("HealthMonitor: rejects an out-of-range config") {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};
    CHECK_THROWS_AS(
        (HealthMonitor{latch, HealthMonitorConfig{.brownoutVolts = Voltage{0.0}}}),
        PreconditionError);
    CHECK_THROWS_AS((HealthMonitor{latch, HealthMonitorConfig{.brownoutVolts = Voltage{11.0},
                                                              .brownoutRecoverVolts = Voltage{10.0}}}),
                    PreconditionError);
    CHECK_THROWS_AS((HealthMonitor{latch, HealthMonitorConfig{.maxMotorTempC = 0.0}}),
                    PreconditionError);
}

// Bug caught (DEFECTS1 item E9): brownoutRecoverVolts was ordered against brownoutVolts but
// never checked for finiteness, and +Inf passes the ordering. The re-arm branch
// (`v >= brownoutRecoverVolts`) could then never be true, so brownoutActive_ latched on the
// first trip and every later collapse was silently swallowed — the E1 anti-spam edge trigger
// turned into a permanent mute on the one signal it exists to report. At most one brownout
// episode per run, however many times the pack died.
TEST_CASE("HealthMonitor: a non-finite brownoutRecoverVolts is rejected at construction (E9)") {
    Rig r;
    const double inf = std::numeric_limits<double>::infinity();
    HealthMonitorConfig bad;
    bad.brownoutRecoverVolts = Voltage{inf};
    CHECK_THROWS_AS((HealthMonitor{r.latch, bad}), PreconditionError);

    // NEGATIVE CONTROL: the ordering rule still holds on its own, and a finite recover level
    // above the trip level still constructs — so the throw above is about finiteness, not
    // about having changed the ordering check.
    HealthMonitorConfig backwards;
    backwards.brownoutVolts = Voltage{7.0};
    backwards.brownoutRecoverVolts = Voltage{6.0};
    CHECK_THROWS_AS((HealthMonitor{r.latch, backwards}), PreconditionError);
    HealthMonitorConfig good;
    good.brownoutVolts = Voltage{7.0};
    good.brownoutRecoverVolts = Voltage{8.0};
    CHECK_NOTHROW((HealthMonitor{r.latch, good}));
}

// The behaviour the E9 precondition protects, pinned end to end: with a FINITE recover level
// a pack that collapses, recovers and collapses again raises TWO episodes. This is what
// +Inf silently reduced to one.
TEST_CASE("HealthMonitor: two brownout episodes are two faults, not one (E9's payoff)") {
    Rig r;
    HealthMonitor::Observations o;
    o.batteryVolts = Voltage{6.0};
    r.monitor.tick(o);                       // episode 1
    CHECK(r.latch.raiseCount(FaultCode::Brownout) == 1);
    o.batteryVolts = Voltage{12.0};
    r.monitor.tick(o);                       // genuine recovery re-arms
    o.batteryVolts = Voltage{6.0};
    r.monitor.tick(o);                       // episode 2
    CHECK(r.latch.raiseCount(FaultCode::Brownout) == 2);
}
