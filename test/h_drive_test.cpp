// Physical-geometry tests for the H-drive preset (chunk C3) — the drive the
// pseudo-inverse exists for. These pin the SIGNATURES of an H-drive, each
// derived INDEPENDENTLY of MatrixKinematics (rigid-body kinematics computed
// from scratch in-test), because the sim plant and the motion layer share the
// same IKinematics object: a sign error in the preset's rows would CANCEL
// end-to-end in closed loop (toWheels and forward invert each other's mistake)
// and only an independent oracle can see it. drive_plant.hpp's header names
// exactly this hazard; these tests are the oracle for the H rows.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/h_drive.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::hDrive;
using shulib::kinematics::HDriveConfig;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {
// A representative off-centre geometry (NOT the HA-55 stand-in bot — synthetic
// numbers so a geometry-file bug and a test-rig bug cannot share a source).
HDriveConfig cfg(double track = 10.0, double offset = -3.0) {
    return HDriveConfig{.trackWidth = Length{track}, .strafeWheelOffset = Length{offset}};
}
}  // namespace

TEST_CASE("hDrive: shape — 3 kinematic wheels (left, right, strafe), derived authority") {
    const MatrixKinematics k = hDrive(cfg());
    CHECK(k.wheelCount() == 3);
    // authority = strafeSpeedRatio (1.0 default) × strafeTractionDerate (0.35
    // provisional HA-54) — the master plan's locked "~0.35" default, derived.
    CHECK(k.strafeAuthority() == doctest::Approx(0.35));
}

// ── The independent rigid-body oracle (the whole point of this file) ───────────────
// Wheel surface speed = d̂ · (v_body + ω × r), computed HERE from each wheel's
// position and roll direction, never from the preset's rows. Randomized over
// geometry AND twist so no lucky cancellation survives the sweep.
// Bug caught: any row error — swapped left/right lever signs, the strafe wheel's
// ω-coupling sign (vy − ω·a instead of vy + ω·a), offset applied to the wrong
// wheel — including the class that cancels in closed loop through the shared
// plant.
TEST_CASE("hDrive: toWheels matches from-scratch rigid-body projection over random "
          "geometry and twists") {
    shulib::sim::Rng rng{7100ULL};
    for (int trial = 0; trial < 200; ++trial) {
        CAPTURE(trial);
        const double w = rng.uniform(6.0, 16.0);        // track width
        const double a = rng.uniform(-8.0, 8.0);        // strafe wheel fwd coord
        const MatrixKinematics k = hDrive(cfg(w, a));
        const double vx = rng.uniform(-60.0, 60.0);
        const double vy = rng.uniform(-25.0, 25.0);
        const double om = rng.uniform(-6.0, 6.0);
        const WheelSpeeds ws =
            k.toWheels(ChassisSpeeds{Velocity{vx}, Velocity{vy}, AngularVelocity{om}});
        // From scratch: v_point = (vx − ω·py, vy + ω·px), dotted with roll dir.
        // left  wheel: p = (0, +w/2), rolls (1,0)  → vx − ω·(w/2)
        // right wheel: p = (0, −w/2), rolls (1,0)  → vx + ω·(w/2)
        // strafe wheel: p = (a, 0),   rolls (0,1)  → vy + ω·a
        REQUIRE(ws[0].value() == doctest::Approx(vx - om * (w / 2.0)).epsilon(1e-12));
        REQUIRE(ws[1].value() == doctest::Approx(vx + om * (w / 2.0)).epsilon(1e-12));
        REQUIRE(ws[2].value() == doctest::Approx(vy + om * a).epsilon(1e-12));
    }
}

// ── The tank base IS the proven tank closed form ───────────────────────────────────
// Bug caught: the H-drive's drive lines diverging from TankKinematics (a second,
// subtly different differential in the tree — the two-implementations drift bug).
TEST_CASE("hDrive: left/right wheels equal TankKinematics of the same track, for every "
          "twist on a swept grid") {
    const double track = 11.0;
    const MatrixKinematics h = hDrive(cfg(track, -4.0));
    const TankKinematics tank{Length{track}};
    for (int a = -3; a <= 3; ++a) {
        for (int c = -3; c <= 3; ++c) {
            const ChassisSpeeds cmd{Velocity{static_cast<double>(a) * 9.0},
                                    Velocity{13.0},  // tank ignores vy; H routes it to
                                                     // wheel 2 only — drive lines match
                                    AngularVelocity{static_cast<double>(c) * 0.9}};
            const WheelSpeeds hw = h.toWheels(cmd);
            const WheelSpeeds tw = tank.toWheels(cmd);
            CHECK(hw[0].value() == doctest::Approx(tw[0].value()).epsilon(1e-12));
            CHECK(hw[1].value() == doctest::Approx(tw[1].value()).epsilon(1e-12));
        }
    }
}

// ── Signatures a wrong table cannot fake ───────────────────────────────────────────
TEST_CASE("hDrive: pure strafe drives ONLY the strafe wheel; pure rotation drives all "
          "three (the off-centre wheel must roll or drag)") {
    const MatrixKinematics k = hDrive(cfg(10.0, -3.0));
    const WheelSpeeds s =
        k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{14.0}, AngularVelocity{0.0}});
    CHECK(s[0].value() == doctest::Approx(0.0));
    CHECK(s[1].value() == doctest::Approx(0.0));
    CHECK(s[2].value() == doctest::Approx(14.0));

    const WheelSpeeds r =
        k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{2.0}});
    CHECK(r[0].value() == doctest::Approx(-10.0));  // −(w/2)·ω = −5·2
    CHECK(r[1].value() == doctest::Approx(+10.0));
    // The off-centre strafe wheel translates during rotation (a·ω = −3·2): a
    // table that zeroed this would command the wheel to DRAG across the foam and
    // mis-attribute the motion in forward().
    CHECK(r[2].value() == doctest::Approx(-6.0));
}

// ── forward(): exact inverse, proven against a from-scratch closed form ────────────
// The 3×3 H table is square full-rank, so forward() must be the EXACT inverse:
//   vx = (l+r)/2,  ω = (r−l)/w,  vy = s − ω·a   (derived by hand from the rows).
// Bug caught: a wrong pseudo-inverse for the square case (the H-drive's own
// odometry path — this feeds the plant's truth).
TEST_CASE("hDrive: forward equals the hand-derived closed-form inverse and round-trips") {
    shulib::sim::Rng rng{7200ULL};
    for (int trial = 0; trial < 200; ++trial) {
        CAPTURE(trial);
        const double w = rng.uniform(6.0, 16.0);
        const double a = rng.uniform(-8.0, 8.0);
        const MatrixKinematics k = hDrive(cfg(w, a));
        WheelSpeeds ws{3};
        ws.set(0, Velocity{rng.uniform(-60.0, 60.0)});
        ws.set(1, Velocity{rng.uniform(-60.0, 60.0)});
        ws.set(2, Velocity{rng.uniform(-30.0, 30.0)});
        const Twist2d t = k.forward(ws);
        const double l = ws[0].value(), r = ws[1].value(), s = ws[2].value();
        const double om = (r - l) / w;
        REQUIRE(t.vx().value() == doctest::Approx((l + r) / 2.0).epsilon(1e-10));
        REQUIRE(t.omega().value() == doctest::Approx(om).epsilon(1e-10));
        REQUIRE(t.vy().value() == doctest::Approx(s - om * a).epsilon(1e-10));
        // and the round-trip closes:
        const Twist2d back = k.forward(k.toWheels(ChassisSpeeds{t.vx(), t.vy(), t.omega()}));
        REQUIRE(back.approxEqual(t, 1e-9));
    }
}

// ── Authority is derived, and from the RIGHT inputs ────────────────────────────────
// Bug caught: authority invented (a constant that ignores the config), or
// accidentally coupled to wheel PLACEMENT (offset changes the ω coupling, never
// the sustainable lateral speed).
TEST_CASE("hDrive: authority = ratio × derate; independent of strafe wheel placement") {
    auto k1 = cfg();
    k1.strafeSpeedRatio = 0.5;
    k1.strafeTractionDerate = 0.4;
    CHECK(hDrive(k1).strafeAuthority() == doctest::Approx(0.2));

    auto k2 = cfg();
    k2.strafeTractionDerate = 1.0;
    CHECK(hDrive(k2).strafeAuthority() == doctest::Approx(1.0));  // symmetric limit

    // Placement sweep: the authority number must not move.
    for (const double a : {-8.0, -3.0, 0.0, 2.5, 8.0}) {
        CHECK(hDrive(cfg(10.0, a)).strafeAuthority() == doctest::Approx(0.35));
    }
}

TEST_CASE("hDrive: derate 0 is the honest dead-strafe limit — authority 0, kinematics "
          "still rank-3") {
    auto c = cfg();
    c.strafeTractionDerate = 0.0;
    const MatrixKinematics k = hDrive(c);
    CHECK(k.strafeAuthority() == doctest::Approx(0.0));
    // The wheel still EXISTS kinematically (forward() must keep inverting its
    // encoder), even though the motion layer will clamp vy to zero (C1 D12's
    // tank-honesty behaviour takes over).
    CHECK(k.wheelCount() == 3);
    const Twist2d back = k.forward(
        k.toWheels(ChassisSpeeds{Velocity{10.0}, Velocity{5.0}, AngularVelocity{1.0}}));
    CHECK(back.approxEqual(Twist2d{Velocity{10.0}, Velocity{5.0}, AngularVelocity{1.0}}, 1e-9));
}

// ── The brief's degenerate case: strafe wheel EXACTLY on centre ────────────────────
// a = 0 makes the table orthogonal — MatrixKinematics runs its historical exact
// path. Everything must behave identically to the off-centre case except the ω
// coupling: same authority, exact round-trip, ω never leaks into vy.
// Bug caught: an on-centre special case that diverges (the preset must be ONE
// code path for both mountings), or authority silently depending on a.
TEST_CASE("hDrive: strafe wheel exactly on centre — orthogonal table, sensible limits") {
    const MatrixKinematics k = hDrive(cfg(10.0, 0.0));
    CHECK(k.strafeAuthority() == doctest::Approx(0.35));
    // Pure rotation no longer moves the strafe wheel (a·ω = 0):
    const WheelSpeeds r =
        k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{2.0}});
    CHECK(r[2].value() == doctest::Approx(0.0));
    // Round-trip stays exact:
    for (int a = -2; a <= 2; ++a) {
        for (int c = -2; c <= 2; ++c) {
            const ChassisSpeeds cmd{Velocity{static_cast<double>(a) * 11.0}, Velocity{7.5},
                                    AngularVelocity{static_cast<double>(c) * 1.1}};
            CHECK(k.forward(k.toWheels(cmd))
                      .approxEqual(Twist2d{cmd.vx(), cmd.vy(), cmd.omega()}, 1e-9));
        }
    }
}

// ── Config preconditions: malformed geometry throws, never mis-builds ──────────────
TEST_CASE("hDrive: config preconditions reject bad geometry and bad ratios") {
    CHECK_THROWS_AS(hDrive({.trackWidth = Length{0.0}, .strafeWheelOffset = Length{1.0}}),
                    PreconditionError);
    CHECK_THROWS_AS(hDrive({.trackWidth = Length{-5.0}, .strafeWheelOffset = Length{1.0}}),
                    PreconditionError);
    CHECK_THROWS_AS(
        hDrive({.trackWidth = Length{std::numeric_limits<double>::quiet_NaN()},
                .strafeWheelOffset = Length{1.0}}),
        PreconditionError);
    CHECK_THROWS_AS(
        hDrive({.trackWidth = Length{10.0},
                .strafeWheelOffset = Length{std::numeric_limits<double>::infinity()}}),
        PreconditionError);
    {
        auto c = cfg();
        c.strafeSpeedRatio = 0.0;
        CHECK_THROWS_AS(hDrive(c), PreconditionError);
    }
    {
        auto c = cfg();
        c.strafeSpeedRatio = -1.0;
        CHECK_THROWS_AS(hDrive(c), PreconditionError);
    }
    {
        auto c = cfg();
        c.strafeTractionDerate = -0.1;
        CHECK_THROWS_AS(hDrive(c), PreconditionError);
    }
    {
        auto c = cfg();
        c.strafeTractionDerate = 1.1;
        CHECK_THROWS_AS(hDrive(c), PreconditionError);
    }
    {
        auto c = cfg();
        c.strafeTractionDerate = std::numeric_limits<double>::quiet_NaN();
        CHECK_THROWS_AS(hDrive(c), PreconditionError);
    }
}
