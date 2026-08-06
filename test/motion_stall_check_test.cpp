// OdoStallCheck unit tests — the spin-vs-motion cross-check in isolation.
// Each case names its bug (C1 test-bar rule). The integrated behaviour (a real
// frozen encoder through the full stack) lives in motion_hostile_test.cpp;
// here the verdict logic itself is pinned against hand-built observables.

#include "doctest.h"

#include <array>
#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::IMotor;
using shulib::hal::fake::FakeMotor;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::OdoStallCheck;
using shulib::motion::OdoStallCheckConfig;
using shulib::units::AngleDim;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Four drive motors + a pose the test scripts by hand. Wheel radius 1.625 in
/// (the config default), window 0.3 s, ticks every 10 ms.
struct Rig {
    std::array<FakeMotor, 4> motors{};
    std::array<IMotor*, 4> ptrs{&motors[0], &motors[1], &motors[2], &motors[3]};
    OdoStallCheck check{};

    [[nodiscard]] std::span<IMotor* const> span() const {
        return std::span<IMotor* const>{ptrs.data(), ptrs.size()};
    }

    /// Advance every shaft by `shaftRad` and feed `pose` at time t.
    bool feed(double t, double shaftRad, const Pose2d& pose) {
        for (FakeMotor& m : motors) {
            m.setPosition(AngleDim{m.position().value() + shaftRad});
        }
        return check.update(Time{t}, span(), pose);
    }
};

constexpr double kWheelR = 3.25 / 2.0;

}  // namespace

// Bug caught: a check that never fires — wheels spinning hard with a frozen
// pose MUST trip after one full window. (Mutation #4's unit-level home.)
TEST_CASE("OdoStallCheck: spin with zero observed motion trips after one window") {
    Rig rig;
    const Pose2d frozen{};
    double t = 0.0;
    bool stalled = false;
    // 2 in of wheel travel per window (≥ minSpinTravel 1.0), pose frozen:
    for (int i = 0; i < 40; ++i) {
        t += 0.01;
        stalled = rig.feed(t, (2.0 / kWheelR) / 30.0, frozen);
    }
    CHECK(stalled);
    CHECK(rig.check.stalled());
}

// Bug caught: false positives on honest driving (spin ≈ motion) — a check that
// faults a healthy robot would poison every routine with ODO_STUCK spam.
TEST_CASE("OdoStallCheck: honest motion (observed ~= spin) never trips") {
    Rig rig;
    double t = 0.0;
    double x = 0.0;
    for (int i = 0; i < 120; ++i) {
        t += 0.01;
        const double travel = 0.15;  // 15 in/s
        x += travel;
        CHECK_FALSE(rig.feed(t, travel / kWheelR, Pose2d{Length{x}, Length{0.0}, Angle{}}));
    }
}

// Bug caught: the rotation term missing — a pure turn (wheels spin, position
// still, heading moves) would false-fault. (The rotation-term mutation home.)
TEST_CASE("OdoStallCheck: a pure rotation is MOTION, not a stall (the rotation term)") {
    Rig rig;
    double t = 0.0;
    double heading = 0.0;
    for (int i = 0; i < 120; ++i) {
        t += 0.01;
        const double dTheta = 0.03;  // 3 rad/s spin in place
        heading += dTheta;
        // wheels travel ≈ R·dθ each (7 in drive radius): genuinely spinning
        CHECK_FALSE(rig.feed(t, 7.0 * dTheta / kWheelR,
                             Pose2d{Length{0.0}, Length{0.0}, Angle::radians(heading)}));
    }
}

// Bug caught: a raw (non-shortest) heading delta across the ±180° seam would
// read ~358° of phantom motion and MASK a real stall exactly there.
TEST_CASE("OdoStallCheck: a stall is still caught when the heading sits on the ±180° seam") {
    Rig rig;
    double t = 0.0;
    bool stalled = false;
    // heading wobbles 179° ↔ −179° (2° of real rotation) while wheels spin
    // 3 in per window and the position is frozen:
    for (int i = 0; i < 40; ++i) {
        t += 0.01;
        const double h = (i % 2 == 0) ? 179.0 : -179.0;
        stalled = rig.feed(t, (3.0 / kWheelR) / 30.0,
                           Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(h)});
    }
    CHECK(stalled);  // raw |Δ| = 358° would have read 43 in of motion and missed it
}

// Bug caught: slip-margin regression — A3's slip still propels ≈70% of spin,
// far above the 25% ratio; a check that trips on ordinary slip is unusable.
TEST_CASE("OdoStallCheck: 70% slip (A3's traction model) does not trip") {
    Rig rig;
    double t = 0.0;
    double x = 0.0;
    for (int i = 0; i < 120; ++i) {
        t += 0.01;
        const double spinTravel = 0.20;          // 20 in/s of wheel spin
        x += spinTravel * 0.70;                  // 70% reaches the floor
        CHECK_FALSE(rig.feed(t, spinTravel / kWheelR, Pose2d{Length{x}, Length{0.0}, Angle{}}));
    }
}

// Bug caught: verdict not recovering — after real motion resumes, the flag
// must clear so HealthMonitor's episode can re-arm.
TEST_CASE("OdoStallCheck: the verdict clears when motion resumes") {
    Rig rig;
    double t = 0.0;
    for (int i = 0; i < 40; ++i) {  // stall phase
        t += 0.01;
        (void)rig.feed(t, (2.0 / kWheelR) / 30.0, Pose2d{});
    }
    REQUIRE(rig.check.stalled());
    double x = 0.0;
    bool last = true;
    for (int i = 0; i < 40; ++i) {  // healthy phase
        t += 0.01;
        x += 0.15;
        last = rig.feed(t, 0.15 / kWheelR, Pose2d{Length{x}, Length{0.0}, Angle{}});
    }
    CHECK_FALSE(last);
    CHECK_FALSE(rig.check.stalled());
}

// Bug caught: sub-threshold spin counted as a stall — a robot COMMANDED to sit
// still (zero volts, zero spin) must never read stalled no matter how long.
TEST_CASE("OdoStallCheck: no spin, no stall — stationary is not stuck") {
    Rig rig;
    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        t += 0.01;
        CHECK_FALSE(rig.feed(t, 0.0, Pose2d{}));
    }
}

// Bug caught: reset() leaking the previous window's baseline across a motion
// boundary — a setPose/teleport between motions would read as huge motion (or
// its absence as a stall).
TEST_CASE("OdoStallCheck: reset() re-baselines — a teleport between motions is not motion") {
    Rig rig;
    double t = 0.0;
    for (int i = 0; i < 15; ++i) {
        t += 0.01;
        (void)rig.feed(t, (2.0 / kWheelR) / 30.0, Pose2d{});
    }
    rig.check.reset();  // motion boundary; the pose then JUMPS 50 in (setPose)
    bool stalled = false;
    for (int i = 0; i < 40; ++i) {
        t += 0.01;
        stalled = rig.feed(t, (2.0 / kWheelR) / 30.0,
                           Pose2d{Length{50.0}, Length{0.0}, Angle{}});
    }
    // Post-reset baseline starts AT the teleported pose, so the jump itself
    // contributed nothing — the ongoing spin-without-motion still trips.
    CHECK(stalled);
}

// Bug caught: config validation holes.
TEST_CASE("OdoStallCheck: invalid configs are rejected loudly") {
    OdoStallCheckConfig bad;
    bad.window = 0.0;
    CHECK_THROWS_AS((OdoStallCheck{bad}), shulib::PreconditionError);
    bad = {};
    bad.motionRatio = 1.0;
    CHECK_THROWS_AS((OdoStallCheck{bad}), shulib::PreconditionError);
    bad = {};
    bad.minSpinTravel = Length{0.0};
    CHECK_THROWS_AS((OdoStallCheck{bad}), shulib::PreconditionError);
    bad = {};
    bad.wheelRadius = Length{-1.0};
    CHECK_THROWS_AS((OdoStallCheck{bad}), shulib::PreconditionError);
    bad = {};
    bad.rotationRadius = Length{0.0};
    CHECK_THROWS_AS((OdoStallCheck{bad}), shulib::PreconditionError);
}
