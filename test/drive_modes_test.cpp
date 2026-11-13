// Adversarial tests for the drive modes (chunk T2, brief tests 4, 9, 10).
//
// The load-bearing claim here is a ROUND TRIP, not a formula: tank mode exists
// only because one DriveRequest can still carry it, and that is true exactly
// when tankSideVolts() recovers the two side commands the driver asked for. The
// legacy tree's catalogued failure (C6: three data representations of one
// command vocabulary) is what a second request type would re-create, so the
// inverse is pinned rather than asserted in prose.
//
// The disconnected-controller case is swept through EVERY mode, because HA-103
// says a dropped controller reads 0 on every channel anyway — which makes the
// isConnected() guard look redundant in whichever mode it was forgotten.
//
// Every expected value that is not swept is a hand-computed literal. The
// mutations the T2 brief requires were RUN and each observed red; T2-PROGRESS.md
// records the messages.

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/field_centric.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::teleop::arcadeRequest;
using shulib::teleop::DriveMode;
using shulib::teleop::DriveRequest;
using shulib::teleop::FieldCentric;
using shulib::teleop::FieldCentricReason;
using shulib::teleop::mapSticks;
using shulib::teleop::modeRequest;
using shulib::teleop::SideVolts;
using shulib::teleop::StickInput;
using shulib::teleop::tankRequest;
using shulib::teleop::tankSideVolts;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

namespace {

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A grid of EXACT binary fractions (sixteenths) spanning the unit range. Chosen so the
/// tank round trip is an exact arithmetic claim rather than a lucky one: for multiples of
/// 1/16 with magnitude <= 1, `l + r` and `r - l` are representable without rounding, halving
/// is exact, and the recombination inside tankSideVolts is exact too.
[[nodiscard]] std::vector<double> dyadicGrid() {
    std::vector<double> v;
    for (int i = -16; i <= 16; ++i) {
        v.push_back(static_cast<double>(i) / 16.0);
    }
    return v;
}

/// A grid of values that are NOT binary fractions — where the round trip is correct to a
/// couple of ulps rather than bit-perfect, which is a documented property and not a defect.
[[nodiscard]] std::vector<double> awkwardGrid() {
    std::vector<double> v;
    for (int i = -20; i <= 20; ++i) {
        v.push_back(static_cast<double>(i) * 0.05);
    }
    v.push_back(0.1);
    v.push_back(-0.3);
    v.push_back(0.7);
    v.push_back(1.0 / 3.0);
    return v;
}

const std::vector<double>& modeSweepAxes() {
    static const std::vector<double> v = {-1.0, -0.6, -0.05, -0.02, 0.0, -0.0,
                                          0.02, 0.05, 0.6,   1.0,   kInf, -kInf};
    return v;
}

}  // namespace

// Constexpr-ness is part of the contract. Pinned on inputs no sign or half-swap mutation can
// satisfy accidentally at COMPILE time is deliberately NOT done here — the signs are pinned by
// the runtime cases, so a flipped one reads as a failing test with its inputs, not as a build
// that stops before any case runs (the R1a precedent).
static_assert(arcadeRequest(StickInput{}).forward == 0.0);
static_assert(tankRequest(StickInput{}).forward == 0.0);

// ═══ Test 4 — the tank inverse ═════════════════════════════════════════════════════

TEST_CASE("T2 tank: tankSideVolts recovers the two sides EXACTLY — the reason there is one request") {
    // BUG CAUGHT: the brief's named mutation, the halves swapped — forward = (l - r)/2 and
    // yaw = (l + r)/2, or the yaw sign flipped. Either turns the left stick into something
    // that is not the left side, and a single-stick test would still look plausible. The
    // round trip over a full 33x33 grid leaves it nowhere: only the true inverse recovers
    // BOTH sides at EVERY point.
    //
    // Exactness is arithmetic, not luck: on multiples of 1/16 in [-1, 1] every operation in
    // the round trip is representable, so `==` is the right comparison and a tolerance would
    // be hiding something.
    const double maxVolts = 12.0;
    int compared = 0;
    for (const double l : dyadicGrid()) {
        for (const double r : dyadicGrid()) {
            const SideVolts got = tankSideVolts(tankRequest(l, r), maxVolts);
            if (got.left != maxVolts * l || got.right != maxVolts * r) {
                CAPTURE(l);
                CAPTURE(r);
                CAPTURE(got.left);
                CAPTURE(got.right);
                CHECK(got.left == maxVolts * l);
                CHECK(got.right == maxVolts * r);
            }
            ++compared;
        }
    }
    CHECK(compared == 33 * 33);  // the grid actually ran

    // The strafe axis is zero in tank mode, always — a tank drivetrain cannot honour one,
    // and quietly asking for one would be a lie the kinematics would have to clean up.
    for (const double l : dyadicGrid()) {
        CHECK(tankRequest(l, -l).left == 0.0);
    }

    // Off the dyadic grid the round trip is correct to a couple of ulps, which is a property
    // of halving-and-recombining doubles, not a defect. Pinned so a real regression (a
    // swapped half, a sign) still fails here and cannot hide behind "floating point".
    int loose = 0;
    for (const double l : awkwardGrid()) {
        for (const double r : awkwardGrid()) {
            const SideVolts got = tankSideVolts(tankRequest(l, r), maxVolts);
            if (std::abs(got.left - maxVolts * l) > 1e-14
                || std::abs(got.right - maxVolts * r) > 1e-14) {
                CAPTURE(l);
                CAPTURE(r);
                CHECK(std::abs(got.left - maxVolts * l) <= 1e-14);
                CHECK(std::abs(got.right - maxVolts * r) <= 1e-14);
            }
            ++loose;
        }
    }
    CHECK(loose > 1900);
}

TEST_CASE("T2 tank: no clamp engages anywhere inside the unit square") {
    // BUG CAUGHT: a request scaled so that a legal stick position saturates a side. If
    // tankSideVolts' clamp ever fired inside the unit square the inverse above would be
    // silently approximate at the corners, and the test would still pass at the centre.
    const double maxVolts = 12.0;
    for (const double l : dyadicGrid()) {
        for (const double r : dyadicGrid()) {
            const SideVolts got = tankSideVolts(tankRequest(l, r), maxVolts);
            if (std::abs(got.left) > maxVolts || std::abs(got.right) > maxVolts) {
                CAPTURE(l);
                CAPTURE(r);
                CHECK(std::abs(got.left) <= maxVolts);
            }
        }
    }
    // The four corners, hand-computed: both sticks full forward is straight ahead at full
    // authority and zero yaw; opposite rails are a pure spin at full yaw and zero forward.
    CHECK(tankRequest(1.0, 1.0).forward == 1.0);
    CHECK(tankRequest(1.0, 1.0).yawCcw == 0.0);
    CHECK(tankRequest(-1.0, -1.0).forward == -1.0);
    CHECK(tankRequest(-1.0, 1.0).forward == 0.0);
    CHECK(tankRequest(-1.0, 1.0).yawCcw == 1.0);   // right side faster = CCW = left turn
    CHECK(tankRequest(1.0, -1.0).yawCcw == -1.0);  // left side faster = CW
}

TEST_CASE("T2 tank: the left stick is the LEFT side and the right stick the RIGHT side") {
    // BUG CAUGHT: the sticks swapped, or rightY read from the wrong axis. Single-stick
    // inputs make the assignment unambiguous, which the symmetric grid above cannot.
    const StickInput leftOnly{.leftY = 0.5, .rightY = 0.0, .connected = true};
    CHECK(tankRequest(leftOnly).forward == 0.25);    // (0.5 + 0)/2
    CHECK(tankRequest(leftOnly).yawCcw == -0.25);    // (0 - 0.5)/2 — left side faster = CW
    const StickInput rightOnly{.rightY = 0.5, .connected = true};
    CHECK(tankRequest(rightOnly).forward == 0.25);
    CHECK(tankRequest(rightOnly).yawCcw == 0.25);    // right side faster = CCW
    // Through the volts, hand-computed at 12 V: only the commanded side moves.
    CHECK(tankSideVolts(tankRequest(leftOnly), 12.0).left == 6.0);
    CHECK(tankSideVolts(tankRequest(leftOnly), 12.0).right == 0.0);
    CHECK(tankSideVolts(tankRequest(rightOnly), 12.0).left == 0.0);
    CHECK(tankSideVolts(tankRequest(rightOnly), 12.0).right == 6.0);
    // leftX is IGNORED in tank mode — a tank drivetrain cannot strafe.
    const StickInput withStrafe{.leftY = 0.5, .leftX = -1.0, .rightY = 0.5, .connected = true};
    CHECK(tankRequest(withStrafe).left == 0.0);
    CHECK(tankRequest(withStrafe).forward == 0.5);
}

// ═══ ArcadeBody — the mapping, and its relationship to today's ═════════════════════

TEST_CASE("T2 arcade: the same axis signs as the shipped mapping, unshaped") {
    // BUG CAUGHT: a sign flipped in the new mapping while the old one stays right — the
    // robot would drive backwards only once a program switched to the shaper. Locked frame
    // F1: +X forward, +Y LEFT, yaw CCW-positive; hal's LeftX/RightX are + when pushed RIGHT.
    const StickInput up{.leftY = 0.5, .connected = true};
    CHECK(arcadeRequest(up).forward == 0.5);
    CHECK(arcadeRequest(up).left == 0.0);
    CHECK(arcadeRequest(up).yawCcw == 0.0);
    const StickInput pushedLeft{.leftX = -0.5, .connected = true};
    CHECK(arcadeRequest(pushedLeft).left == 0.5);
    const StickInput yawRight{.rightX = 0.5, .connected = true};
    CHECK(arcadeRequest(yawRight).yawCcw == -0.5);  // right = clockwise = NEGATIVE
    const StickInput down{.leftY = -0.75, .connected = true};
    CHECK(arcadeRequest(down).forward == -0.75);

    // Outside the old deadband, the unshaped mapping and mapSticks() agree exactly — the
    // rescale is the ONLY difference the feel layer introduces, and it lives in the shaping
    // step, not here.
    int agreed = 0;
    for (const double ly : {-1.0, -0.6, -0.051, 0.051, 0.6, 1.0}) {
        for (const double lx : {-1.0, -0.2, 0.2, 1.0}) {
            for (const double rx : {-1.0, 0.3, 1.0}) {
                const StickInput in{.leftY = ly, .leftX = lx, .rightX = rx, .connected = true};
                CHECK(arcadeRequest(in).forward == mapSticks(in).forward);
                CHECK(arcadeRequest(in).left == mapSticks(in).left);
                CHECK(arcadeRequest(in).yawCcw == mapSticks(in).yawCcw);
                ++agreed;
            }
        }
    }
    CHECK(agreed == 72);
}

TEST_CASE("T2 modeRequest: ArcadeField maps exactly like ArcadeBody — the rotation is elsewhere") {
    // BUG CAUGHT: a rotation smuggled into the mapping step, which would put field-centric
    // BEFORE the slew limiter and make a spinning robot fight its own rate limit under a
    // still stick (the documented order says rotation is last).
    const StickInput in{.leftY = 0.4, .leftX = -0.2, .rightX = 0.7, .connected = true};
    CHECK(modeRequest(DriveMode::ArcadeBody, in).forward
          == modeRequest(DriveMode::ArcadeField, in).forward);
    CHECK(modeRequest(DriveMode::ArcadeBody, in).left
          == modeRequest(DriveMode::ArcadeField, in).left);
    CHECK(modeRequest(DriveMode::ArcadeBody, in).yawCcw
          == modeRequest(DriveMode::ArcadeField, in).yawCcw);
    // And TankSticks really is a different mapping, so the dispatch is not a no-op.
    const StickInput tank{.leftY = 0.4, .rightY = -0.4, .connected = true};
    CHECK(modeRequest(DriveMode::TankSticks, tank).forward == 0.0);
    CHECK(modeRequest(DriveMode::ArcadeBody, tank).forward == 0.4);
}

// ═══ Test 9 — the disconnected controller, through EVERY mode ══════════════════════

TEST_CASE("T2 HA-103: a DISCONNECTED controller is all zero in every mode, whatever the axes") {
    // BUG CAUGHT: the isConnected() guard dropped in ONE mode. It looks redundant — a
    // controller that drops reads 0 on every channel anyway (HA-103) — and it is not: it is
    // what makes a stale or garbage axis read on a lost link command NOTHING. A per-mode
    // sweep is the only shape that catches a guard present in two modes and missing in the
    // third, and the axes are driven to the rails and past them (inf) so a missing guard
    // cannot hide behind zeros.
    int checked = 0;
    for (const DriveMode mode : {DriveMode::ArcadeBody, DriveMode::ArcadeField,
                                 DriveMode::TankSticks}) {
        for (const double a : modeSweepAxes()) {
            const StickInput in{.leftY = a, .leftX = -a, .rightX = a, .rightY = -a,
                                .connected = false};
            const DriveRequest r = modeRequest(mode, in);
            // Exactly zero, on BITS: a +0.0 and a -0.0 are both "zero" to ==, and the
            // library's sign-of-zero rule is what a later bit-identity pin will compare.
            if (!sameBits(r.forward, 0.0) || !sameBits(r.left, 0.0)
                || !sameBits(r.yawCcw, 0.0)) {
                CAPTURE(static_cast<int>(mode));
                CAPTURE(a);
                CHECK(sameBits(r.forward, 0.0));
                CHECK(sameBits(r.left, 0.0));
                CHECK(sameBits(r.yawCcw, 0.0));
            }
            ++checked;
        }
    }
    CHECK(checked == 3 * 12);

    // And the same sticks CONNECTED are not zero — so the case above is testing the guard,
    // not an all-zero input.
    const StickInput live{.leftY = 1.0, .leftX = -1.0, .rightX = 1.0, .rightY = -1.0,
                          .connected = true};
    CHECK(modeRequest(DriveMode::ArcadeBody, live).forward == 1.0);
    CHECK(modeRequest(DriveMode::ArcadeField, live).left == 1.0);
    CHECK(modeRequest(DriveMode::TankSticks, live).yawCcw == -1.0);
}

// ═══ Test 10 — totality ════════════════════════════════════════════════════════════

TEST_CASE("T2 modes are TOTAL: a non-finite axis produces a value, and is rejected downstream") {
    // BUG CAUGHT: a precondition added to the mapping layer, which would move the home of
    // the finiteness contract. It lives at Chassis::drive and at the motor adapter, and it
    // must stay there: a teleop tick that can abort the program is worse than one that
    // commands a NaN, because the abort takes the whole robot with it.
    const StickInput bad{.leftY = kNaN, .leftX = kInf, .rightX = -kInf, .rightY = kNaN,
                         .connected = true};
    CHECK(std::isnan(arcadeRequest(bad).forward));
    CHECK(arcadeRequest(bad).left == -kInf);   // the sign flip is all the mapping does
    CHECK(arcadeRequest(bad).yawCcw == kInf);
    CHECK(std::isnan(tankRequest(bad).forward));
    // A non-finite axis survives the mapping rather than being silently swallowed — which is
    // what makes it visible at the precondition that owns it (pinned in driver_control_test).
    CHECK_FALSE(std::isfinite(modeRequest(DriveMode::TankSticks, bad).forward));
}

// ═══ Test 5 — ArcadeField: the rotation, and the honest failure mode ═══════════════
//
// The rotation direction is the landmine: the body frame is CCW-positive and the
// driver's stick vector must be rotated by MINUS (heading - zero). A test at 90
// degrees alone can pass with the sign wrong once the other bugs are excluded, so
// the cases below carry two ODD angles as well as the four cardinals, and both
// components of every result. The oracle is hand-derived from R(-phi), and each
// case is ALSO cross-checked against the frozen math::fieldToRobot — a second,
// independently tested implementation of the same rotation that the code under
// test does not call.

TEST_CASE("T2 field-centric: the translation rotates by MINUS the heading, both components") {
    // BUG CAUGHT: the brief's named mutation — rotating by +heading instead of -heading.
    // At 0 and 180 the two agree, so only the 90/270 pair and the odd angles separate
    // them; checking only `forward` would miss it at 37 degrees too, because the forward
    // component of R(+phi) and R(-phi) is identical (cos is even). It is the LATERAL
    // component that carries the sign, which is why every case below checks both.
    struct Case {
        double headingDeg;
        double wantForward;  // stick UP (forward = 1, left = 0) -> body
        double wantLeft;
    };
    // Hand-derived: R(-phi) * (1, 0) = (cos phi, -sin phi).
    const Case cases[] = {
        {0.0, 1.0, 0.0},
        {90.0, 6.123233995736766e-17, -1.0},
        {180.0, -1.0, -1.2246467991473532e-16},
        {270.0, -1.8369701987210297e-16, 1.0},
        {37.0, 0.7986355100472928, -0.6018150231520483},
        {-53.0, 0.6018150231520484, 0.7986355100472928},
    };
    for (const Case& c : cases) {
        CAPTURE(c.headingDeg);
        FieldCentric fc;
        const double h = c.headingDeg * Angle::kPi / 180.0;
        const FieldCentric::Result got = fc.request(DriveRequest{1.0, 0.0, 0.25}, h, true);
        CHECK(got.request.forward == doctest::Approx(c.wantForward).epsilon(1e-14));
        CHECK(got.request.left == doctest::Approx(c.wantLeft).epsilon(1e-14));
        CHECK(got.state.active);
        CHECK_FALSE(got.state.fellBack);
        CHECK(got.state.reason == FieldCentricReason::Active);
        // Yaw is frame-invariant: it survives the rotation untouched, bit for bit.
        CHECK(sameBits(got.request.yawCcw, 0.25));

        // Independent cross-check: the frozen F1 rotation, which this code does not call.
        const ChassisSpeeds oracle = shulib::math::fieldToRobot(
            ChassisSpeeds{Velocity{1.0}, Velocity{0.0}, AngularVelocity{0.25}},
            Angle::radians(h));
        CHECK(got.request.forward == doctest::Approx(oracle.vx().value()).epsilon(1e-14));
        CHECK(got.request.left == doctest::Approx(oracle.vy().value()).epsilon(1e-14));
    }
}

TEST_CASE("T2 field-centric: stick UP at 90 degrees CCW drives the robot's RIGHT, not its left") {
    // BUG CAUGHT: the rotation direction, stated once in plain physical terms so a reader
    // can check it without trigonometry. At heading 90 the robot's nose points along field
    // +Y. The driver pushes "away from me", which is field +X. Field +X is 90 degrees
    // CLOCKWISE from the nose — the robot's RIGHT — and the body frame's +Y is LEFT, so the
    // request must be left = -1.
    FieldCentric fc;
    const FieldCentric::Result got =
        fc.request(DriveRequest{1.0, 0.0, 0.0}, Angle::kPi / 2.0, true);
    CHECK(got.request.left == doctest::Approx(-1.0).epsilon(1e-14));
    CHECK(std::abs(got.request.forward) < 1e-14);
    // And the lateral stick at the same heading drives the robot FORWARD.
    FieldCentric fc2;
    const FieldCentric::Result lateral =
        fc2.request(DriveRequest{0.0, 1.0, 0.0}, Angle::kPi / 2.0, true);
    CHECK(lateral.request.forward == doctest::Approx(1.0).epsilon(1e-14));
    CHECK(std::abs(lateral.request.left) < 1e-14);
}

TEST_CASE("T2 field-centric: an untrusted heading falls back UNROTATED, and the fallback LATCHES") {
    // BUG CAUGHT: the brief's named mutation — the latch dropped, so the controls flicker
    // between two frames the instant heading quality wobbles. A driver can adapt to a robot
    // that is robot-centric; a driver cannot adapt to a robot that changes its mind
    // mid-match, and the flicker is invisible to any test that only checks the untrusted
    // tick itself. The sequence below is the whole property: fall back, STAY fallen back
    // through trusted ticks, and recover only on the driver's re-zero.
    FieldCentric fc;
    const DriveRequest stick{1.0, 0.0, 0.0};
    const double h = Angle::kPi / 2.0;  // 90 degrees — rotated and unrotated differ most here

    const FieldCentric::Result untrusted = fc.request(stick, h, false);
    CHECK(untrusted.request.forward == 1.0);  // unrotated
    CHECK(untrusted.request.left == 0.0);
    CHECK(untrusted.state.fellBack);
    CHECK_FALSE(untrusted.state.active);
    CHECK(untrusted.state.reason == FieldCentricReason::HeadingNotTrusted);
    CHECK(fc.fellBack());

    // Trust returns — and the controls DO NOT silently rotate back.
    for (int tick = 0; tick < 5; ++tick) {
        const FieldCentric::Result still = fc.request(stick, h, true);
        CAPTURE(tick);
        CHECK(still.request.forward == 1.0);
        CHECK(still.request.left == 0.0);
        CHECK(still.state.fellBack);
        CHECK_FALSE(still.state.active);
        CHECK(still.state.reason == FieldCentricReason::LatchedUntilRezero);
    }

    // Only the driver clears it.
    fc.rezero(0.0);
    CHECK_FALSE(fc.fellBack());
    const FieldCentric::Result back = fc.request(stick, h, true);
    CHECK(back.state.active);
    CHECK_FALSE(back.state.fellBack);
    CHECK(back.request.left == doctest::Approx(-1.0).epsilon(1e-14));
}

TEST_CASE("T2 field-centric: rezero(h) makes 'stick UP at heading h' mean 'body forward'") {
    // BUG CAUGHT: the zero offset dropped from the rotation (phi computed from the raw
    // heading), or applied with the wrong sign. The driver's re-zero is the whole reason
    // the offset exists: after pressing it, the way the robot is facing IS "away from me".
    for (const double zeroDeg : {0.0, 37.0, 90.0, -120.0, 179.0}) {
        CAPTURE(zeroDeg);
        FieldCentric fc;
        const double zero = zeroDeg * Angle::kPi / 180.0;
        fc.rezero(zero);
        CHECK(fc.zeroRadians() == zero);
        const FieldCentric::Result got = fc.request(DriveRequest{1.0, 0.0, 0.0}, zero, true);
        CHECK(got.request.forward == doctest::Approx(1.0).epsilon(1e-14));
        CHECK(std::abs(got.request.left) < 1e-14);
        // And a quarter turn CCW from the new zero is still the robot's right.
        const FieldCentric::Result quarter =
            fc.request(DriveRequest{1.0, 0.0, 0.0}, zero + Angle::kPi / 2.0, true);
        CHECK(quarter.request.left == doctest::Approx(-1.0).epsilon(1e-14));
    }
}

TEST_CASE("T2 field-centric: a NON-FINITE heading is treated as untrusted, not propagated") {
    // BUG CAUGHT: a NaN heading multiplied into the request, which turns the whole command
    // into NaN and surfaces as a precondition abort inside Chassis::drive with nothing on
    // the panel to say why. "Trusted" is exactly the claim such a number fails, so it takes
    // the honest path the mode already has — and it LATCHES like any other loss of trust.
    for (const double bad : {kNaN, kInf, -kInf}) {
        CAPTURE(bad);
        FieldCentric fc;
        const FieldCentric::Result got = fc.request(DriveRequest{1.0, 0.0, 0.5}, bad, true);
        CHECK(got.request.forward == 1.0);
        CHECK(got.request.left == 0.0);
        CHECK(got.request.yawCcw == 0.5);
        CHECK(got.state.fellBack);
        CHECK(got.state.reason == FieldCentricReason::HeadingNotTrusted);
        CHECK(fc.fellBack());
    }
}

TEST_CASE("T2 field-centric: nothing is tared — the zero lives in this object and nowhere else") {
    // BUG CAUGHT: a re-zero implemented by moving the estimate (HA-05: a tare under a live
    // boot-heading offset silently corrupts it). Structurally impossible here — this object
    // holds two doubles and a bool and has a reference to nothing — and this case pins the
    // observable half: rezero changes THIS object's offset, and `active` is a property of a
    // tick rather than of the object.
    FieldCentric fc;
    CHECK(fc.zeroRadians() == 0.0);
    CHECK_FALSE(fc.fellBack());
    CHECK(fc.state().reason == FieldCentricReason::NotSelected);
    fc.rezero(1.25);
    CHECK(fc.zeroRadians() == 1.25);
    (void)fc.request(DriveRequest{}, 0.0, false);
    CHECK(fc.state().reason == FieldCentricReason::LatchedUntilRezero);
    CHECK(fc.state().fellBack);
    CHECK_FALSE(fc.state().active);
    fc.reset();
    CHECK(fc.zeroRadians() == 0.0);
    CHECK_FALSE(fc.fellBack());
}
