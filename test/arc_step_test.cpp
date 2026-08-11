// Adversarial tests for arcStep — the SE(2) constant-curvature integrator (localization
// backbone). These are built to FAIL if any term is wrong: each pins an INDEPENDENTLY known
// endpoint (closed-form arc geometry, or the raw-integral oracle in arc_expected.py), not a
// restatement of arcStep's own half-angle form. The keystone is the quarter-circle case,
// which is exactly where the legacy odom's rotate-by-new-heading bug shows up.
//
// Frame (F1): +X forward (body), +Y left (body); field +X/CCW. arcStep returns the FIELD
// displacement for one tick given the tracking center's body travel + the heading samples.

#include <cmath>

#include "doctest.h"

#include "shulib/localization/arc_step.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::localization::arcStep;
using shulib::localization::BodyTravel;
using shulib::localization::FieldDelta;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::robotToField;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {
constexpr double kPi = Angle::kPi;
}  // namespace

// ── Straight-line (Δθ = 0): pure rotate-by-heading ──────────────────────────────────────
TEST_CASE("arcStep: straight forward at heading 0 moves along +X") {
    const FieldDelta d = arcStep({.forward = Length{5.0}, .lateral = Length{0.0}}, Angle{}, Angle{});
    CHECK(d.dx.value() == doctest::Approx(5.0));
    CHECK(d.dy.value() == doctest::Approx(0.0));
}

TEST_CASE("arcStep: straight forward at heading 90° moves along +Y") {
    const Angle h = Angle::degrees(90.0);
    const FieldDelta d = arcStep({.forward = Length{5.0}, .lateral = Length{0.0}}, h, h);  // Δθ = 0
    CHECK(d.dx.value() == doctest::Approx(0.0));
    CHECK(d.dy.value() == doctest::Approx(5.0));
}

TEST_CASE("arcStep: pure lateral (strafe) sign is +Y/left at heading 0") {
    const FieldDelta d = arcStep({.forward = Length{0.0}, .lateral = Length{3.0}}, Angle{}, Angle{});
    CHECK(d.dx.value() == doctest::Approx(0.0));
    CHECK(d.dy.value() == doctest::Approx(3.0));  // left == +Y when facing +X
}

TEST_CASE("arcStep: strafing left while facing +Y moves toward -X") {
    const Angle h = Angle::degrees(90.0);
    const FieldDelta d = arcStep({.forward = Length{0.0}, .lateral = Length{3.0}}, h, h);
    CHECK(d.dx.value() == doctest::Approx(-3.0));  // left of +Y is -X
    CHECK(d.dy.value() == doctest::Approx(0.0));
}

// ── KEYSTONE: a known quarter circle. Catches the rotate-by-AVERAGE-heading requirement. ──
// Drive a left (CCW) quarter circle of radius 10 from heading 0. The tracking center's
// forward travel is the arc length 10·(π/2); end heading is +90°. Exact endpoint: (10, 10).
// The legacy bug (rotate by the NEW heading) would land this at (0, 14.14).
TEST_CASE("arcStep: quarter-circle CCW arc lands at the exact (10,10) endpoint") {
    const double rho = 10.0;
    const FieldDelta d = arcStep({.forward = Length{rho * kPi / 2.0}, .lateral = Length{0.0}},
                                 Angle{}, Angle::degrees(90.0));
    CHECK(d.dx.value() == doctest::Approx(10.0));
    CHECK(d.dy.value() == doctest::Approx(10.0));
}

TEST_CASE("arcStep: clockwise quarter-circle mirrors to (10,-10)") {
    const double rho = 10.0;
    const FieldDelta d = arcStep({.forward = Length{rho * kPi / 2.0}, .lateral = Length{0.0}},
                                 Angle{}, Angle::degrees(-90.0));
    CHECK(d.dx.value() == doctest::Approx(10.0));
    CHECK(d.dy.value() == doctest::Approx(-10.0));  // sign of Δθ flips Δy only
}

// Half circle exercises the antipodal Δθ (errorTo(π) resolves to +π, never -π).
TEST_CASE("arcStep: half-circle CCW lands at (0, 20)") {
    const double rho = 10.0;
    const FieldDelta d = arcStep({.forward = Length{rho * kPi}, .lateral = Length{0.0}},
                                 Angle{}, Angle::degrees(180.0));
    CHECK(d.dx.value() == doctest::Approx(0.0));
    CHECK(d.dy.value() == doctest::Approx(20.0));
}

// ── Holonomic arc (forward + lateral + rotation). Expected from the raw-integral oracle,
// which is algebraically independent of arcStep's half-angle form. ───────────────────────
TEST_CASE("arcStep: combined forward+lateral+rotation matches the raw-integral oracle") {
    const FieldDelta d = arcStep({.forward = Length{4.0}, .lateral = Length{2.0}},
                                 Angle::degrees(30.0), Angle::degrees(70.0));
    CHECK(d.dx.value() == doctest::Approx(1.0180886521));   // /tmp/arc_expected.py
    CHECK(d.dy.value() == doctest::Approx(4.2619555594));
}

// Second holonomic case in Q3 with NEGATIVE lateral (red-team gap: the Q1 case above can let a
// lateral-under-rotation sign flip partially cancel; this one is in a quadrant where it can't).
TEST_CASE("arcStep: holonomic arc in Q3 with negative lateral matches the oracle") {
    const FieldDelta d = arcStep({.forward = Length{4.0}, .lateral = Length{-2.0}},
                                 Angle::degrees(200.0), Angle::degrees(245.0));
    CHECK(d.dx.value() == doctest::Approx(-4.1906124133));  // /tmp/arc_expected.py
    CHECK(d.dy.value() == doctest::Approx(-1.1964914009));
}

// ── Equivalence pin: with Δθ = 0, arcStep IS body→field rotation. Locks the handedness to the
// one blessed frame transform (math::robotToField) so the two can never silently diverge. ──
TEST_CASE("arcStep: at Δθ=0 equals robotToField across all quadrants") {
    const double f = 3.3, l = -1.7;
    for (double deg : {0.0, 90.0, 135.0, 200.0, 300.0, -45.0}) {
        const Angle h = Angle::degrees(deg);
        const FieldDelta d = arcStep({.forward = Length{f}, .lateral = Length{l}}, h, h);
        const ChassisSpeeds rf =
            robotToField(ChassisSpeeds{Velocity{f}, Velocity{l}, AngularVelocity{0.0}}, h);
        CHECK(d.dx.value() == doctest::Approx(rf.vx().value()));
        CHECK(d.dy.value() == doctest::Approx(rf.vy().value()));
    }
}

// ── Δθ → 0 limit: no divide-by-zero, and continuity across the small-angle guard ─────────
TEST_CASE("arcStep: exact Δθ == 0 is finite (no 0/0), equals straight travel") {
    const FieldDelta d = arcStep({.forward = Length{7.0}, .lateral = Length{0.0}}, Angle{}, Angle{});
    REQUIRE(std::isfinite(d.dx.value()));
    REQUIRE(std::isfinite(d.dy.value()));
    CHECK(d.dx.value() == doctest::Approx(7.0));
}

TEST_CASE("arcStep: chord factor is continuous across the straight-line guard") {
    // Just inside the guard (k forced to 1) vs just outside (k computed). For a tiny Δθ the
    // forward travel is ~unchanged and the heading barely moves, so both ≈ straight (7, 0).
    const FieldDelta inside = arcStep({.forward = Length{7.0}, .lateral = Length{0.0}},
                                      Angle{}, Angle::radians(1e-12));   // < eps → k = 1
    const FieldDelta outside = arcStep({.forward = Length{7.0}, .lateral = Length{0.0}},
                                       Angle{}, Angle::radians(1e-6));   // > eps → k computed
    CHECK(inside.dx.value() == doctest::Approx(7.0));
    CHECK(outside.dx.value() == doctest::Approx(7.0));
    CHECK(outside.dx.value() == doctest::Approx(inside.dx.value()));
    CHECK(std::abs(outside.dy.value()) < 1e-4);  // negligible lateral creep at Δθ = 1e-6
}

// ── Contract pin: arcStep operates purely on the WRAPPED heading. 270° and −90° are the same
// Angle, so the >π-per-tick aliasing is upstream of arcStep — the plausibility gate belongs to
// PilonsOdometry (which sees consecutive ticks), exactly as the header PRECONDITION states. ──
TEST_CASE("arcStep: a wrapped 270° end-heading is identical to -90° (gate is upstream)") {
    const BodyTravel t{.forward = Length{2.0}, .lateral = Length{1.0}};
    const FieldDelta viaPos = arcStep(t, Angle{}, Angle::degrees(270.0));
    const FieldDelta viaNeg = arcStep(t, Angle{}, Angle::degrees(-90.0));
    CHECK(viaPos.dx.value() == doctest::Approx(viaNeg.dx.value()));
    CHECK(viaPos.dy.value() == doctest::Approx(viaNeg.dy.value()));
}

// ── Chord < arc: a curved step's straight-line displacement is shorter than the travel.
// Distinguishes the chord factor k = 2sin(Δθ/2)/Δθ from a stray k = 1 (would over-shoot). ──
TEST_CASE("arcStep: a 90° arc's net displacement is the chord (shorter than the arc length)") {
    const double rho = 10.0;
    const double arcLen = rho * kPi / 2.0;  // ≈ 15.708
    const FieldDelta d = arcStep({.forward = Length{arcLen}, .lateral = Length{0.0}},
                                 Angle{}, Angle::degrees(90.0));
    const double netDisp = std::hypot(d.dx.value(), d.dy.value());
    const double chord = 2.0 * rho * std::sin(kPi / 4.0);  // ≈ 14.142
    CHECK(netDisp == doctest::Approx(chord));
    CHECK(netDisp < arcLen);  // chord is strictly shorter than the arc it subtends
}
