// Tests for diag/finite_guard.hpp — the §18.4 log-and-recover NaN/Inf invariants.
// What each targets:
//  * THE ABSOLUTE GUARANTEE: recoverFinite* returns a finite value UNCONDITIONALLY —
//    even against a broken (non-finite) fallback. A guard with any bypass path lets a
//    NaN into the pose estimate exactly once, which is once too many.
//  * LOG-AND-RECOVER, NOT CRASH: the deliberately injected NaN is caught, raises a
//    fault through the latch, gets logged, and execution continues — the DoD case.
//  * NO STICKY STATE: recovery must not poison subsequent healthy ticks.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/diag/fault.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::isFinitePose;
using shulib::diag::recoverFinite;
using shulib::diag::recoverFinitePose;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

namespace {
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Rig {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};
};

Pose2d makePose(double x, double y) {
    return Pose2d{Length{x}, Length{y}, Angle::degrees(45.0)};
}
}  // namespace

TEST_CASE("isFinitePose: flags NaN/Inf in either position axis") {
    CHECK(isFinitePose(makePose(1.0, -2.0)));
    CHECK_FALSE(isFinitePose(makePose(kNan, 0.0)));
    CHECK_FALSE(isFinitePose(makePose(0.0, kNan)));
    CHECK_FALSE(isFinitePose(makePose(kInf, 0.0)));
    CHECK_FALSE(isFinitePose(makePose(0.0, -kInf)));
    // Heading cannot be non-finite BY CONSTRUCTION (Angle rejects it) — the reason
    // isFinitePose checks two axes, not three. Pinned in angle_test.cpp; asserted here
    // so a future Angle change that weakens this shows up next to its consumer.
    CHECK_THROWS((void)Angle::radians(kNan));
}

TEST_CASE("recoverFinite(double): a finite candidate passes through untouched, no fault") {
    Rig rig;
    const double out = recoverFinite(-3.25, 99.0, rig.latch, FaultCode::NanPose, "LOC", "vx");
    CHECK(out == -3.25);  // exact — the guard must not perturb healthy data
    CHECK_FALSE(rig.latch.hasFault());
    CHECK(rig.sink.empty());
}

TEST_CASE("recoverFinite(double): NaN and ±Inf are caught, logged with the GIVEN code, "
          "and replaced by the fallback") {
    Rig rig;
    // The code parameter must be respected (not hard-wired to NAN_POSE) — pass another.
    const double out = recoverFinite(kNan, 1.5, rig.latch, FaultCode::GpsGateReject, "EKF", "resid");
    CHECK(out == 1.5);
    CHECK(rig.latch.firstFault() == FaultCode::GpsGateReject);
    REQUIRE(rig.sink.size() == 1);
    CHECK(rig.sink.at(0).subsystem == "EKF");
    CHECK(rig.sink.at(0).message == "fault=GPS_GATE_REJECT n=1 FIRST resid");

    CHECK(recoverFinite(kInf, 2.5, rig.latch, FaultCode::NanPose, "LOC", "") == 2.5);
    CHECK(recoverFinite(-kInf, -2.5, rig.latch, FaultCode::NanPose, "LOC", "") == -2.5);
    CHECK(rig.latch.faultCount() == 3);
}

TEST_CASE("recoverFinite(double): even a NON-FINITE FALLBACK cannot leak — degrades to 0") {
    Rig rig;
    const double out = recoverFinite(kNan, kNan, rig.latch, FaultCode::NanPose, "LOC", "both bad");
    CHECK(out == 0.0);
    CHECK(std::isfinite(out));
    CHECK(rig.latch.hasFault());  // still raised — the pathology is not hidden
}

TEST_CASE("recoverFinitePose: the DoD case — an injected NaN pose is caught, logged as "
          "NAN_POSE, recovered to last-known-good, and the run continues") {
    Rig rig;
    const Pose2d lastGood = makePose(24.0, 36.0);

    const Pose2d out = recoverFinitePose(makePose(kNan, 36.0), lastGood, rig.latch, "LOC");
    CHECK(out.approxEqual(lastGood));                       // recovered, not propagated
    CHECK(rig.latch.firstFault() == FaultCode::NanPose);    // logged as a fault
    REQUIRE(rig.sink.size() == 1);
    CHECK(rig.sink.at(0).message == "fault=NAN_POSE n=1 FIRST non-finite pose -> fallback");

    // …and the run continues: the next healthy tick passes clean (no sticky state).
    const Pose2d next = makePose(24.5, 36.1);
    CHECK(recoverFinitePose(next, out, rig.latch, "LOC").approxEqual(next));
    CHECK(rig.latch.faultCount() == 1);  // no new fault
}

TEST_CASE("recoverFinitePose: every non-finite axis combination recovers; Inf too") {
    Rig rig;
    const Pose2d good = makePose(1.0, 2.0);
    CHECK(recoverFinitePose(makePose(0.0, kNan), good, rig.latch, "LOC").approxEqual(good));
    CHECK(recoverFinitePose(makePose(kInf, 0.0), good, rig.latch, "LOC").approxEqual(good));
    CHECK(recoverFinitePose(makePose(-kInf, kNan), good, rig.latch, "LOC").approxEqual(good));
    CHECK(rig.latch.faultCount() == 3);
}

TEST_CASE("recoverFinitePose: a non-finite fallback degrades to the origin — the return "
          "is finite UNCONDITIONALLY") {
    Rig rig;
    const Pose2d out =
        recoverFinitePose(makePose(kNan, 0.0), makePose(kInf, kNan), rig.latch, "LOC");
    CHECK(isFinitePose(out));
    CHECK(out.x().value() == 0.0);
    CHECK(out.y().value() == 0.0);
    CHECK(rig.latch.hasFault());
}

TEST_CASE("recoverFinitePose: a finite pose is never altered and never faults") {
    Rig rig;
    const Pose2d p = makePose(-71.99, 71.99);  // field-edge extremes are still healthy
    CHECK(recoverFinitePose(p, makePose(0.0, 0.0), rig.latch, "LOC").approxEqual(p));
    CHECK_FALSE(rig.latch.hasFault());
    CHECK(rig.sink.empty());
}
