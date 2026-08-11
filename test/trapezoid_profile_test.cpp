// Adversarial tests for TrapezoidProfile. Targets: the trapezoid phase boundaries and
// total time, the triangular degradation (short move never reaches maxVelocity),
// symmetry, endpoints (clamped + at-rest), signed distance, zero move, and preconditions.

#include "doctest.h"

#include <algorithm>
#include <cmath>

#include "shulib/control/trapezoid_profile.hpp"
#include "shulib/core/check.hpp"

using shulib::PreconditionError;
using shulib::control::ProfileState;
using shulib::control::TrapezoidProfile;

TEST_CASE("TrapezoidProfile: a long move cruises — phases and duration are correct") {
    // D=10, vMax=2, aMax=1: dAccel=2, cruise dist=6 → tAccel=2, tCruise=3, duration=7.
    const TrapezoidProfile p{10.0, {.maxVelocity = 2.0, .maxAcceleration = 1.0}};
    CHECK(p.duration() == doctest::Approx(7.0));

    CHECK(p.sample(0.0).acceleration == doctest::Approx(1.0));  // starts accelerating
    CHECK(p.sample(1.0).velocity == doctest::Approx(1.0));      // v = a·t
    CHECK(p.sample(1.0).position == doctest::Approx(0.5));      // ½·a·t²

    const ProfileState cruise = p.sample(3.5);  // mid-cruise (and the time midpoint)
    CHECK(cruise.velocity == doctest::Approx(2.0));            // at cruise speed
    CHECK(cruise.acceleration == doctest::Approx(0.0));
    CHECK(cruise.position == doctest::Approx(5.0));            // half the distance at the midpoint

    const ProfileState decel = p.sample(6.0);  // in the decel ramp
    CHECK(decel.acceleration == doctest::Approx(-1.0));
    CHECK(decel.velocity == doctest::Approx(1.0));             // 2 − 1·(6−5)
}

TEST_CASE("TrapezoidProfile: a short move is triangular — peak speed below maxVelocity") {
    // D=2, vMax=10 (unreachable), aMax=1: vPeak=√(2·1)=√2, no cruise, duration=2√2.
    const TrapezoidProfile p{2.0, {.maxVelocity = 10.0, .maxAcceleration = 1.0}};
    const double vPeak = std::sqrt(2.0);
    CHECK(p.duration() == doctest::Approx(2.0 * vPeak));
    CHECK(p.sample(vPeak).velocity == doctest::Approx(vPeak));  // peak at the apex (t=tAccel=√2)
    CHECK(p.sample(vPeak).velocity < 10.0);                     // never reaches maxVelocity
    CHECK(p.sample(vPeak).position == doctest::Approx(1.0));    // half the distance at the apex
}

TEST_CASE("TrapezoidProfile: arrives exactly at the target, at rest") {
    const TrapezoidProfile p{10.0, {.maxVelocity = 2.0, .maxAcceleration = 1.0}};
    const ProfileState end = p.sample(p.duration());
    CHECK(end.position == doctest::Approx(10.0));
    CHECK(end.velocity == doctest::Approx(0.0));
    CHECK(end.acceleration == doctest::Approx(0.0));
    // and stays there past the end (clamped)
    CHECK(p.sample(100.0).position == doctest::Approx(10.0));
    CHECK(p.sample(100.0).velocity == doctest::Approx(0.0));
    CHECK(p.isDone(p.duration()));
    CHECK_FALSE(p.isDone(p.duration() - 0.1));
}

TEST_CASE("TrapezoidProfile: a negative move mirrors position/velocity/acceleration") {
    const TrapezoidProfile p{-10.0, {.maxVelocity = 2.0, .maxAcceleration = 1.0}};
    CHECK(p.duration() == doctest::Approx(7.0));         // same timing
    CHECK(p.sample(1.0).velocity == doctest::Approx(-1.0));
    CHECK(p.sample(1.0).position == doctest::Approx(-0.5));
    CHECK(p.sample(0.0).acceleration == doctest::Approx(-1.0));
    CHECK(p.sample(p.duration()).position == doctest::Approx(-10.0));
}

TEST_CASE("TrapezoidProfile: a zero move is instantaneous and always at rest") {
    const TrapezoidProfile p{0.0, {.maxVelocity = 2.0, .maxAcceleration = 1.0}};
    CHECK(p.duration() == doctest::Approx(0.0));
    CHECK(p.sample(0.0).position == doctest::Approx(0.0));
    CHECK(p.sample(5.0).velocity == doctest::Approx(0.0));
    CHECK(p.isDone(0.0));
}

TEST_CASE("TrapezoidProfile: velocity is non-negative across a forward move and peaks once") {
    const TrapezoidProfile p{10.0, {.maxVelocity = 2.0, .maxAcceleration = 1.0}};
    double prevPos = p.sample(0.0).position;
    double maxV = 0.0;
    for (double t = 0.0; t <= p.duration(); t += 0.05) {
        const ProfileState s = p.sample(t);
        CHECK(s.velocity >= -1e-9);            // never reverses on a forward move
        CHECK(s.position >= prevPos - 1e-9);   // position monotonically increases
        prevPos = s.position;
        maxV = std::max(maxV, s.velocity);
    }
    CHECK(maxV == doctest::Approx(2.0));       // tops out at maxVelocity
}

TEST_CASE("TrapezoidProfile: construction rejects non-positive limits") {
    CHECK_THROWS_AS((TrapezoidProfile{10.0, {.maxVelocity = 0.0, .maxAcceleration = 1.0}}),
                    PreconditionError);
    CHECK_THROWS_AS((TrapezoidProfile{10.0, {.maxVelocity = 2.0, .maxAcceleration = -1.0}}),
                    PreconditionError);
}
