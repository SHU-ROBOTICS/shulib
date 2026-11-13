// Adversarial tests for the feel layer's pure shaping (chunk T2, brief tests 1-3).
//
// These three functions are where "twitchy, I can't place it" is either fixed or
// shipped, so the cases below are written against the PROPERTIES that make them
// a fix — continuity at the threshold, monotonicity everywhere, the two ends
// exact, and OFF being off BIT FOR BIT — rather than against a handful of
// values. Every expected number that is not swept is a hand-computed literal
// (E2's lesson), and each case names the bug it catches.
//
// The mutations the T2 brief requires were RUN against these cases and each
// observed red; T2-PROGRESS.md records the message the runner printed.

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/slew_limiter.hpp"
#include "shulib/teleop/stick_mapping.hpp"

using shulib::teleop::AxisShaping;
using shulib::teleop::continuousDeadband;
using shulib::teleop::deadbanded;
using shulib::teleop::DriveMode;
using shulib::teleop::DriverProfile;
using shulib::teleop::expoCurve;
using shulib::teleop::kDefaultDeadband;
using shulib::teleop::kDefaultExpo;
using shulib::teleop::kStickDeadband;
using shulib::teleop::shapeAxis;
using shulib::teleop::SlewLimiter;

namespace {

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A fine sweep of the whole axis range, plus both zeros and both rails exactly.
[[nodiscard]] std::vector<double> fineSweep() {
    std::vector<double> v;
    v.push_back(-1.0);
    for (int i = -2000; i <= 2000; ++i) {
        v.push_back(static_cast<double>(i) / 2000.0);
    }
    v.push_back(1.0);
    return v;
}

}  // namespace

// The constexpr-ness is part of the contract: a shaping function that can be evaluated at
// compile time has no hidden state and no hidden clock. Pinned ONLY on inputs whose answer no
// plausible arithmetic mutation changes (the R1a precedent in stick_mapping_test.cpp) — the
// ends, the rescale and the curve are pinned by the runtime cases below on purpose, so that a
// broken one is observed as a failing TEST with its inputs in the message rather than as a
// compile error that hides every other case in the file.
static_assert(continuousDeadband(0.0, 0.05) == 0.0);
static_assert(expoCurve(0.0, 0.5) == 0.0);
static_assert(shapeAxis(0.0, AxisShaping{}) == 0.0);
static_assert(AxisShaping{}.valid());
static_assert(SlewLimiter{}.isOff());

// ═══ Test 1 — continuousDeadband ═══════════════════════════════════════════════════

TEST_CASE("T2 deadband: CONTINUOUS at the threshold — the whole reason the function exists") {
    // BUG CAUGHT: the naive cut (`|x| < d ? 0 : x`, i.e. stick_mapping's deadbanded())
    // shipped under the new name. It is invisible to a test that only checks "inside is
    // zero" and "1 maps to 1" — both are true of the naive form. What separates them is
    // the value JUST OUTSIDE the threshold: the rescale leaves zero smoothly (a few
    // 1e-17), the naive cut jumps straight to 0.05, which is 5% of the robot's authority
    // arriving in one stick count. That jump IS the "twitchy" complaint.
    const double d = 0.05;
    CHECK(continuousDeadband(std::nextafter(d, 1.0), d) < 1e-12);
    CHECK(continuousDeadband(std::nextafter(-d, -1.0), d) > -1e-12);
    // And a small, real step past the threshold is small: (0.051 - 0.05)/0.95.
    CHECK(continuousDeadband(0.051, d) == doctest::Approx(0.001 / 0.95).epsilon(1e-12));
    CHECK(continuousDeadband(0.051, d) < 0.002);  // the naive cut would answer 0.051

    // Continuity, stated as a limit: the gap across the threshold shrinks with the step.
    for (const double eps : {1e-3, 1e-6, 1e-9}) {
        CAPTURE(eps);
        CHECK(std::abs(continuousDeadband(d + eps, d) - continuousDeadband(d, d))
              < eps / (1.0 - d) * 1.000001);
    }
}

TEST_CASE("T2 deadband: zero inside, EXACTLY zero at the threshold, and POSITIVE zero always") {
    // BUG CAUGHT: (a) the dead zone moved or resized; (b) the threshold itself falling
    // through the rescale branch, which would return NEGATIVE zero at -d (a negated 0.0)
    // and break a later bit-identity pin for a reason nobody can see — stick_mapping pins
    // +0.0 for a centred stick and this layer keeps that rule; (c) an asymmetric cut.
    const double d = 0.05;
    CHECK(continuousDeadband(0.0, d) == 0.0);
    CHECK(continuousDeadband(0.049, d) == 0.0);
    CHECK(continuousDeadband(-0.049, d) == 0.0);
    CHECK(continuousDeadband(std::nextafter(d, 0.0), d) == 0.0);
    CHECK(continuousDeadband(d, d) == 0.0);
    CHECK(continuousDeadband(-d, d) == 0.0);

    // The sign of zero, on bits — the landmine this layer inherited from R1a.
    CHECK(sameBits(continuousDeadband(0.0, d), 0.0));
    CHECK(sameBits(continuousDeadband(-0.0, d), 0.0));
    CHECK(sameBits(continuousDeadband(-0.02, d), 0.0));
    CHECK(sameBits(continuousDeadband(-d, d), 0.0));
    CHECK(sameBits(continuousDeadband(d, d), 0.0));
    CHECK_FALSE(sameBits(continuousDeadband(-0.02, d), -0.0));
    CHECK_FALSE(sameBits(continuousDeadband(-d, d), -0.0));
    // The same property through the whole composition, which is what a caller uses.
    CHECK(sameBits(shapeAxis(-0.02, AxisShaping{}), 0.0));
    CHECK(sameBits(shapeAxis(-0.02, AxisShaping{.deadband = 0.05, .expo = 1.0}), 0.0));
}

TEST_CASE("T2 deadband: full deflection still asks for EVERYTHING — (±1) -> ±1 exactly") {
    // BUG CAUGHT: rescaling by the wrong denominator (dividing by 1 instead of 1-d, or by
    // 1+d), which silently costs the driver top speed — a robot that cannot be asked for
    // 100%. Exact equality on purpose: "about 1" is a different contract.
    for (const double d : {0.0, 0.01, 0.05, 0.1, 0.25, 0.5, 0.9}) {
        CAPTURE(d);
        CHECK(continuousDeadband(1.0, d) == 1.0);
        CHECK(continuousDeadband(-1.0, d) == -1.0);
    }
    // Hand-computed interior values for the shipped deadband: (|x| - 0.05)/0.95.
    CHECK(continuousDeadband(0.1, 0.05) == doctest::Approx(0.05 / 0.95).epsilon(1e-15));
    CHECK(continuousDeadband(0.5, 0.05) == doctest::Approx(0.45 / 0.95).epsilon(1e-15));
    CHECK(continuousDeadband(-0.5, 0.05) == doctest::Approx(-0.45 / 0.95).epsilon(1e-15));
}

TEST_CASE("T2 deadband: MONOTONIC across a fine sweep, both signs, and odd") {
    // BUG CAUGHT: any non-monotonic rescale (an abs() dropped, a sign applied before the
    // subtraction, a clamp in the wrong place) — "pushing further slows the robot down",
    // which is not a feel preference but a control system that lies.
    const std::vector<double> xs = fineSweep();
    for (const double d : {0.0, 0.05, 0.2, 0.5}) {
        CAPTURE(d);
        double previous = continuousDeadband(xs.front(), d);
        int steps = 0;
        for (std::size_t i = 1; i < xs.size(); ++i) {
            const double y = continuousDeadband(xs[i], d);
            if (!(y >= previous)) {
                CAPTURE(xs[i]);
                CAPTURE(y);
                CAPTURE(previous);
                CHECK(y >= previous);
            }
            previous = y;
            ++steps;
        }
        CHECK(steps > 4000);  // the sweep actually ran, or it proves nothing
        // Odd symmetry: f(-x) == -f(x) for every swept point.
        for (const double x : xs) {
            if (continuousDeadband(-x, d) != -continuousDeadband(x, d)) {
                CAPTURE(x);
                CHECK(continuousDeadband(-x, d) == -continuousDeadband(x, d));
            }
        }
    }
}

TEST_CASE("T2 deadband: d = 0 is the IDENTITY, bit for bit — and a nonsensical d is too") {
    // BUG CAUGHT: "off" implemented as arithmetic that happens to be the identity. It is
    // not: (|x| - 0)/(1 - 0) loses the sign of a negative zero, and the shipped contract
    // is that turning a feature off changes no bit at all. Also pins the total-function
    // rule for a deadband at or above 1, which would otherwise divide by zero or by a
    // negative and hand the driver a silently INVERTED axis.
    int swept = 0;
    for (const double x : fineSweep()) {
        if (!sameBits(continuousDeadband(x, 0.0), x)) {
            CAPTURE(x);
            CHECK(sameBits(continuousDeadband(x, 0.0), x));
        }
        ++swept;
    }
    CHECK(swept > 4000);  // the sweep actually ran, or it proves nothing
    CHECK(sameBits(continuousDeadband(-0.0, 0.0), -0.0));
    CHECK(sameBits(continuousDeadband(0.0, 0.0), 0.0));
    for (const double bad : {-0.1, 1.0, 1.5, kNaN}) {
        CAPTURE(bad);
        CHECK(sameBits(continuousDeadband(0.42, bad), 0.42));
        CHECK(sameBits(continuousDeadband(-0.42, bad), -0.42));
    }
    // AxisShaping::valid() is where that nonsense is meant to be caught, loudly.
    CHECK(AxisShaping{}.valid());
    CHECK_FALSE(AxisShaping{.deadband = 1.0, .expo = 0.0}.valid());
    CHECK_FALSE(AxisShaping{.deadband = -0.1, .expo = 0.0}.valid());
    CHECK_FALSE(AxisShaping{.deadband = 0.05, .expo = 1.5}.valid());
    CHECK_FALSE(AxisShaping{.deadband = 0.05, .expo = -0.1}.valid());
}

// ═══ Test 2 — expoCurve ════════════════════════════════════════════════════════════

TEST_CASE("T2 expo: expo = 0 is the IDENTITY, bit for bit, on every input including inf") {
    // BUG CAUGHT: "no curve" written as a multiply by one. x*(0*x*x + 1) is NOT the
    // identity — at x = inf the 0*inf term is NaN, so an axis that was merely out of
    // range becomes un-diagnosable garbage. The early return is the contract.
    int swept = 0;
    for (const double x : fineSweep()) {
        if (!sameBits(expoCurve(x, 0.0), x)) {
            CAPTURE(x);
            CHECK(sameBits(expoCurve(x, 0.0), x));
        }
        ++swept;
    }
    CHECK(swept > 4000);  // the sweep actually ran, or it proves nothing
    CHECK(sameBits(expoCurve(-0.0, 0.0), -0.0));
    CHECK(sameBits(expoCurve(kInf, 0.0), kInf));
    CHECK(sameBits(expoCurve(-kInf, 0.0), -kInf));
    CHECK(std::isnan(expoCurve(kNaN, 0.0)));
    CHECK(kDefaultExpo == 0.0);  // the shipped profile therefore has no curve at all
}

TEST_CASE("T2 expo: through (0,0) and (±1,±1) EXACTLY for every expo on a grid") {
    // BUG CAUGHT: the mutation the brief names — x*(expo*x^2 + 1) instead of
    // x*(expo*x^2 + (1 - expo)). It is monotonic, it is odd, it passes through the
    // origin, and it is WRONG: at full deflection it asks for 1 + expo, i.e. up to
    // double authority, which desaturation then silently eats. Only the ±1 anchor
    // catches it.
    for (const double e : {0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0}) {
        CAPTURE(e);
        CHECK(expoCurve(0.0, e) == 0.0);
        CHECK(expoCurve(1.0, e) == 1.0);
        CHECK(expoCurve(-1.0, e) == -1.0);
    }
    // Hand-computed interior values: x*(e*x^2 + 1 - e).
    CHECK(expoCurve(0.5, 0.5) == doctest::Approx(0.3125).epsilon(1e-15));   // .5*(.125+.5)
    CHECK(expoCurve(0.5, 1.0) == doctest::Approx(0.125).epsilon(1e-15));    // .5*.25
    CHECK(expoCurve(0.8, 0.25) == doctest::Approx(0.728).epsilon(1e-15));   // .8*(.16+.75)
    CHECK(expoCurve(-0.8, 0.25) == doctest::Approx(-0.728).epsilon(1e-15));
    // Larger expo really is softer in the middle (the property a driver buys).
    CHECK(expoCurve(0.3, 1.0) < expoCurve(0.3, 0.5));
    CHECK(expoCurve(0.3, 0.5) < expoCurve(0.3, 0.0));
}

TEST_CASE("T2 expo: MONOTONIC on a fine sweep for every expo in [0, 1], and odd") {
    // BUG CAUGHT: a curve shape that dips — pushing the stick further slowing the robot
    // down. The derivative is 3*e*x^2 + (1 - e), non-negative exactly on [0, 1], so this
    // sweep is the property and not a sample of it.
    const std::vector<double> xs = fineSweep();
    for (const double e : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        CAPTURE(e);
        double previous = expoCurve(xs.front(), e);
        int steps = 0;
        for (std::size_t i = 1; i < xs.size(); ++i) {
            const double y = expoCurve(xs[i], e);
            if (!(y >= previous)) {
                CAPTURE(xs[i]);
                CAPTURE(y);
                CAPTURE(previous);
                CHECK(y >= previous);
            }
            previous = y;
            ++steps;
        }
        CHECK(steps > 4000);  // the sweep actually ran, or it proves nothing
        for (const double x : xs) {
            if (expoCurve(-x, e) != -expoCurve(x, e)) {
                CAPTURE(x);
                CHECK(expoCurve(-x, e) == -expoCurve(x, e));
            }
        }
        // Never asks for more than the rail, anywhere.
        for (const double x : xs) {
            if (!(std::abs(expoCurve(x, e)) <= 1.0)) {
                CAPTURE(x);
                CHECK(std::abs(expoCurve(x, e)) <= 1.0);
            }
        }
    }
}

// ═══ shapeAxis — the composition, and the shipped default ══════════════════════════

TEST_CASE("T2 shapeAxis: deadband FIRST, then the curve — and the order is visible") {
    // BUG CAUGHT: the composition written the other way round. Curve-then-deadband
    // compresses the dead zone itself, so the threshold's position starts depending on
    // the expo setting and tuning becomes guesswork. The two orders are pinned apart
    // here with a hand-computed pair.
    const AxisShaping s{.deadband = 0.05, .expo = 1.0};
    // deadband first: (0.5 - 0.05)/0.95 = 0.473684210526315790; cubed.
    const double rescaled = 0.45 / 0.95;
    CHECK(shapeAxis(0.5, s) == doctest::Approx(rescaled * rescaled * rescaled).epsilon(1e-14));
    // curve first would be 0.125, then the deadband: (0.125-0.05)/0.95 = 0.078947…
    CHECK(shapeAxis(0.5, s) != doctest::Approx(0.075 / 0.95).epsilon(1e-9));
    // Both ends survive the composition exactly, for every expo.
    for (const double e : {0.0, 0.5, 1.0}) {
        CAPTURE(e);
        const AxisShaping a{.deadband = 0.05, .expo = e};
        CHECK(shapeAxis(1.0, a) == 1.0);
        CHECK(shapeAxis(-1.0, a) == -1.0);
        CHECK(shapeAxis(0.0, a) == 0.0);
        CHECK(shapeAxis(0.03, a) == 0.0);
    }
}

TEST_CASE("T2 default profile: the shipped numbers are the ones the header claims") {
    // BUG CAUGHT: a default quietly edited away from "reproduce today's mapping as
    // closely as a continuous deadband allows". Every one of these is INVENTED
    // (PROVISIONAL, A4: HA-112) and pinned as a literal so a change has to be deliberate.
    const DriverProfile p{};
    CHECK(kDefaultDeadband == 0.05);
    CHECK(kDefaultDeadband == kStickDeadband);  // the same dead zone the library shipped
    CHECK(kDefaultExpo == 0.0);
    CHECK(p.forward.deadband == 0.05);
    CHECK(p.left.deadband == 0.05);
    CHECK(p.yaw.deadband == 0.05);
    CHECK(p.forward.expo == 0.0);
    CHECK(p.left.expo == 0.0);
    CHECK(p.yaw.expo == 0.0);
    CHECK(p.slew.ratePerSecond == 0.0);   // OFF: lag is the enemy of feel
    CHECK(p.mode == DriveMode::ArcadeBody);
    CHECK(p.valid());
}

// ═══ Test 3 — SlewLimiter ══════════════════════════════════════════════════════════

TEST_CASE("T2 slew: rate 0 is OFF — output == target, BIT FOR BIT") {
    // BUG CAUGHT: "off" implemented as a limit with a huge rate, or as arithmetic that
    // rounds to the target. Neither is off. The shipped profile has slew off, so this is
    // the property that makes the default add exactly zero lag — and a rate expressed as
    // a step would turn a -0.0 into a +0.0 here and break a later bit pin.
    SlewLimiter off{};
    CHECK(off.isOff());
    CHECK(off.ratePerSecond() == 0.0);
    int swept = 0;
    for (const double x : fineSweep()) {
        if (!sameBits(off.update(x, 0.01), x)) {
            CAPTURE(x);
            CHECK(sameBits(off.update(x, 0.01), x));
        }
        ++swept;
    }
    CHECK(swept > 4000);  // the sweep actually ran, or it proves nothing
    CHECK(sameBits(off.update(-0.0, 0.01), -0.0));
    CHECK(sameBits(off.update(kInf, 0.01), kInf));
    // A jump the full width of the range, in one tick, unmodified.
    off.reset(1.0);
    CHECK(sameBits(off.update(-1.0, 0.001), -1.0));
    // A NEGATIVE rate is read as OFF, not as a backwards limit (a silent control
    // inversion would be undebuggable; "off" is a failure a driver can name).
    SlewLimiter negative{-5.0};
    CHECK(negative.isOff());
    CHECK(sameBits(negative.update(0.7, 0.01), 0.7));
}

TEST_CASE("T2 slew: the limit IS the limit — |delta| <= rate*dt every tick") {
    // BUG CAUGHT: the brief's two named mutations — dt ignored (so the step becomes the
    // rate itself, 64x too large here) and the clamp skipped on the negative side (so a
    // stick slammed backwards is not limited at all). Swept over rates and dts so a
    // single lucky arithmetic identity cannot hide either.
    for (const double rate : {0.5, 1.0, 4.0, 20.0}) {
        for (const double dt : {0.001, 0.01, 0.02, 1.0 / 64.0}) {
            CAPTURE(rate);
            CAPTURE(dt);
            SlewLimiter s{rate};
            double previous = 0.0;
            // A deliberately hostile target stream: rails, reversals, small moves.
            for (const double target : {1.0, 1.0, -1.0, -1.0, 0.0, 0.3, -0.3, 1.0, -1.0}) {
                for (int tick = 0; tick < 20; ++tick) {
                    const double y = s.update(target, dt);
                    const double moved = std::abs(y - previous);
                    if (!(moved <= rate * dt * (1.0 + 1e-12))) {
                        CAPTURE(target);
                        CAPTURE(y);
                        CAPTURE(previous);
                        CHECK(moved <= rate * dt * (1.0 + 1e-12));
                    }
                    previous = y;
                }
            }
        }
    }
}

TEST_CASE("T2 slew: reaches the target in the hand-computed number of ticks, exactly") {
    // BUG CAUGHT: an off-by-one in the step, or a limiter that creeps toward its target
    // forever (delta scaled rather than clamped) so a held stick never delivers full
    // authority. Rate and dt are binary-exact on purpose (4.0 per second at 1/64 s =
    // exactly 1/16 per tick), so every value below is an exact arithmetic claim.
    SlewLimiter s{4.0};
    const double dt = 1.0 / 64.0;  // step = 0.0625 exactly
    CHECK(s.value() == 0.0);
    for (int tick = 1; tick <= 16; ++tick) {
        const double y = s.update(1.0, dt);
        CAPTURE(tick);
        CHECK(y == static_cast<double>(tick) * 0.0625);
    }
    CHECK(s.value() == 1.0);
    // And it STAYS there — no overshoot past a reached target.
    CHECK(s.update(1.0, dt) == 1.0);
    CHECK(s.update(1.0, dt) == 1.0);
    // Within reach it lands EXACTLY on the target rather than stepping past it.
    s.reset(0.0);
    CHECK(s.update(0.01, dt) == 0.01);
}

TEST_CASE("T2 slew: a sign reversal is limited THROUGH zero, not across it") {
    // BUG CAUGHT: the negative-side clamp skipped (the brief's mutation), or a limit
    // applied to |target| so a full reversal jumps the full 2.0 span in one tick — the
    // exact current spike a slew limit exists to prevent.
    SlewLimiter s{4.0};
    const double dt = 1.0 / 64.0;  // step 0.0625
    s.reset(1.0);
    for (int tick = 1; tick <= 16; ++tick) {
        const double y = s.update(-1.0, dt);
        CAPTURE(tick);
        CHECK(y == 1.0 - static_cast<double>(tick) * 0.0625);
    }
    CHECK(s.value() == 0.0);              // it passed THROUGH zero at tick 16
    for (int tick = 1; tick <= 16; ++tick) {
        CHECK(s.update(-1.0, dt) == -static_cast<double>(tick) * 0.0625);
    }
    CHECK(s.value() == -1.0);             // 32 ticks for the full span, as hand-computed
}

TEST_CASE("T2 slew: reset(), and a dt that has not passed cannot move the output") {
    // BUG CAUGHT: reset() forgetting to clear (so a limiter carries a stale command
    // across a change of control), or dt <= 0 being treated as "take the whole step",
    // which would make a paused or mis-measured clock look like an unlimited axis.
    SlewLimiter s{4.0};
    (void)s.update(1.0, 1.0 / 64.0);
    CHECK(s.value() == 0.0625);
    s.reset();
    CHECK(s.value() == 0.0);
    s.reset(0.5);
    CHECK(s.value() == 0.5);
    CHECK(s.update(1.0, 0.0) == 0.5);     // no time has passed
    CHECK(s.update(1.0, -0.01) == 0.5);   // nor backwards
    CHECK(s.update(1.0, kNaN) == 0.5);    // nor un-measurably
    CHECK(s.value() == 0.5);
    CHECK(s.update(1.0, 1.0 / 64.0) == 0.5625);
}

TEST_CASE("T2 shaping + slew are TOTAL: a hostile axis produces a value, never an abort") {
    // BUG CAUGHT: a precondition added to the feel layer. Finiteness is checked where it
    // always was — Chassis::drive and the motor adapter — and a teleop tick must never be
    // able to abort the program. These calls exist to prove they return.
    CHECK(std::isnan(continuousDeadband(kNaN, 0.05)));
    CHECK(continuousDeadband(kInf, 0.05) == kInf);
    CHECK(continuousDeadband(-kInf, 0.05) == -kInf);
    CHECK(std::isnan(expoCurve(kNaN, 0.5)));
    CHECK(std::isnan(shapeAxis(kNaN, AxisShaping{})));
    CHECK(shapeAxis(kInf, AxisShaping{}) == kInf);  // expo 0 short-circuits, so inf survives
    SlewLimiter s{4.0};
    (void)s.update(kNaN, 1.0 / 64.0);
    CHECK(std::isnan(s.value()));
    s.reset();
    CHECK(s.value() == 0.0);
}
