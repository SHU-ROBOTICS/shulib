// Adversarial tests for the teleop stick mapping (chunk R3b Session 2, brief test 14).
//
// The mapping was EXTRACTED from src/main.cpp's R1a loop with the promise of no
// behaviour change. A promise is not a measurement, so the first case here
// carries the ORIGINAL code — `shaped()` and the ChassisSpeeds construction,
// copied verbatim from the pre-change main.cpp — and compares the two BIT FOR
// BIT across a sweep. Every other expected value is a hand-computed literal
// (E2's lesson). Each case names the bug it catches, and the sign-flip
// mutations the brief requires were run against these cases (the R3b log
// records each observed red).

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::ChassisSpeeds;
using shulib::teleop::deadbanded;
using shulib::teleop::DriveRequest;
using shulib::teleop::kStickDeadband;
using shulib::teleop::mapSticks;
using shulib::teleop::mapSticksToChassisSpeeds;
using shulib::teleop::StickInput;
using shulib::teleop::toChassisSpeeds;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

namespace {

// ── THE ORACLE: src/main.cpp's mapping BEFORE the extraction, verbatim ─────────
// (commit d92a9fe, `shaped()` at main.cpp:275-277 and the opcontrol() body at
// main.cpp:401-409). Do not "improve" this block: its only value is that it is
// the old code, character for character where the arithmetic is concerned.
namespace legacy {
constexpr double kTeleopDeadband = 0.05;

[[nodiscard]] double shaped(double axis) {
    return (axis > -kTeleopDeadband && axis < kTeleopDeadband) ? 0.0 : axis;
}

[[nodiscard]] ChassisSpeeds loopBody(bool connected, double leftY, double leftX, double rightX,
                                     const shulib::motion::MotionConfig& cfg) {
    ChassisSpeeds command{};  // zero twist unless the driver says otherwise
    if (connected) {
        const double fwd = shaped(leftY);
        const double left = shaped(-leftX);
        const double yaw = shaped(-rightX);
        command = ChassisSpeeds{fwd * cfg.maxLinearSpeed, left * cfg.maxLinearSpeed,
                                yaw * cfg.maxAngularSpeed};
    }
    return command;
}
}  // namespace legacy

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

[[nodiscard]] bool sameBits(const ChassisSpeeds& a, const ChassisSpeeds& b) {
    return sameBits(a.vx().value(), b.vx().value()) && sameBits(a.vy().value(), b.vy().value())
           && sameBits(a.omega().value(), b.omega().value());
}

/// The axis values that matter: both zeros, the deadband threshold and its two
/// floating-point neighbours on each side, the rails, and a coarse grid.
[[nodiscard]] std::vector<double> interestingAxisValues() {
    std::vector<double> v = {0.0, -0.0, kStickDeadband, -kStickDeadband,
                             std::nextafter(kStickDeadband, 0.0),
                             std::nextafter(-kStickDeadband, 0.0),
                             std::nextafter(kStickDeadband, 1.0),
                             std::nextafter(-kStickDeadband, -1.0), 1.0, -1.0};
    for (double x = -1.0; x <= 1.0001; x += 0.05) {
        v.push_back(x);
    }
    return v;
}

}  // namespace

// The constexpr-ness is part of the contract (a mapping evaluable at compile time has no
// hidden state). Pinned on inputs whose answer no sign/deadband mutation changes — the
// SIGNS are pinned by the runtime cases below on purpose, so that a flipped sign is
// observed as a failing TEST (with its inputs in the message) rather than as a compile
// error that hides every other case.
static_assert(mapSticks(StickInput{}).forward == 0.0);
static_assert(deadbanded(1.0) == 1.0);
static_assert(toChassisSpeeds(DriveRequest{1.0, 0.0, 0.0}, Velocity{60.0}, AngularVelocity{6.0})
                  .vx()
                  .value()
              == 60.0);

TEST_CASE("stick mapping: BIT-IDENTICAL to the R1a loop it replaced (the oracle sweep)") {
    // BUG CAUGHT: any change at all in the extracted mapping — a flipped sign, a
    // deadband-then-negate reorder (which changes the SIGN OF ZERO inside the
    // deadband), a boundary moved from < to <=, a budget applied to the wrong
    // axis. The oracle is the old code; the comparison is on bits, not Approx.
    const shulib::motion::MotionConfig defaults{};
    shulib::motion::MotionConfig odd{};
    odd.maxLinearSpeed = Velocity{37.5};
    odd.maxAngularSpeed = AngularVelocity{2.25};
    // A NEGATIVE budget is not a valid MotionConfig (validate() rejects it), but the
    // sweep includes one on purpose: it is the only input that separates "return
    // ChassisSpeeds{} when disconnected" from "return 0·budget" — the latter yields
    // −0.0 here, and the old loop returned +0.0. The mapping must not have grown a
    // dependence on the budgets' values in the disconnected case.
    shulib::motion::MotionConfig negative{};
    negative.maxLinearSpeed = Velocity{-60.0};
    negative.maxAngularSpeed = AngularVelocity{-6.0};

    const std::vector<double> axis = interestingAxisValues();
    const shulib::motion::MotionConfig* const configs[] = {&defaults, &odd, &negative};
    int compared = 0;
    for (const shulib::motion::MotionConfig* cfg : configs) {
        for (bool connected : {true, false}) {
            // Full 3-D grid over the interesting values (~50^3 combinations per config).
            for (double ly : axis) {
                for (double lx : axis) {
                    for (double rx : axis) {
                        const ChassisSpeeds want =
                            legacy::loopBody(connected, ly, lx, rx, *cfg);
                        const ChassisSpeeds got = mapSticksToChassisSpeeds(
                            StickInput{.leftY = ly, .leftX = lx, .rightX = rx,
                                       .connected = connected},
                            cfg->maxLinearSpeed, cfg->maxAngularSpeed);
                        if (!sameBits(want, got)) {
                            // One failing CHECK per divergence, with the inputs in the message.
                            CAPTURE(ly);
                            CAPTURE(lx);
                            CAPTURE(rx);
                            CAPTURE(connected);
                            CHECK(sameBits(want, got));
                        }
                        ++compared;
                    }
                }
            }
        }
    }
    // The sweep must have actually run at scale, or a broken loop proves nothing.
    CHECK(compared > 3 * 2 * 40 * 40 * 40);
}

TEST_CASE("stick mapping: axis signs — up = +forward, LEFT = +left, RIGHT = −yaw (CCW-positive)") {
    // BUG CAUGHT: any one of the three signs flipped — the robot drives backward
    // on "forward", strafes right on "left", or turns the wrong way on the right
    // stick. Locked frame F1: +X forward, +Y LEFT, yaw CCW-positive; hal's LeftX
    // and RightX are + when pushed RIGHT, so "left" and "CCW" are both negations.
    const StickInput up{.leftY = 0.5, .connected = true};
    CHECK(mapSticks(up).forward == 0.5);
    CHECK(mapSticks(up).left == 0.0);
    CHECK(mapSticks(up).yawCcw == 0.0);

    const StickInput pushedLeft{.leftX = -0.5, .connected = true};   // − raw = pushed LEFT
    CHECK(mapSticks(pushedLeft).left == 0.5);
    const StickInput pushedRight{.leftX = 0.5, .connected = true};   // + raw = pushed RIGHT
    CHECK(mapSticks(pushedRight).left == -0.5);

    const StickInput yawRight{.rightX = 0.5, .connected = true};     // right stick pushed RIGHT
    CHECK(mapSticks(yawRight).yawCcw == -0.5);                       // → clockwise = NEGATIVE
    const StickInput yawLeft{.rightX = -0.5, .connected = true};
    CHECK(mapSticks(yawLeft).yawCcw == 0.5);

    const StickInput down{.leftY = -0.75, .connected = true};
    CHECK(mapSticks(down).forward == -0.75);

    // Through the budgets, hand-computed: 0.5 × 60 in/s = 30 in/s; 0.5 × 6 rad/s = 3 rad/s.
    const Velocity lin{60.0};
    const AngularVelocity ang{6.0};
    CHECK(toChassisSpeeds(mapSticks(up), lin, ang).vx().value() == 30.0);
    CHECK(toChassisSpeeds(mapSticks(pushedLeft), lin, ang).vy().value() == 30.0);
    CHECK(toChassisSpeeds(mapSticks(yawRight), lin, ang).omega().value() == -3.0);
    // The budgets go to the right axes: yaw never scales by the linear budget.
    const StickInput all{.leftY = 1.0, .leftX = -1.0, .rightX = 1.0, .connected = true};
    const ChassisSpeeds s = toChassisSpeeds(mapSticks(all), lin, ang);
    CHECK(s.vx().value() == 60.0);
    CHECK(s.vy().value() == 60.0);
    CHECK(s.omega().value() == -6.0);
}

TEST_CASE("stick mapping: axis ASSIGNMENT — the left stick translates, the right stick yaws") {
    // BUG CAUGHT: the sticks swapped (right stick driving forward) or an axis
    // cross-wired (LeftX feeding forward) — every other case above passes a
    // single-axis input, which cannot see a swap that keeps the signs right.
    const StickInput onlyRight{.rightX = 0.8, .connected = true};
    CHECK(mapSticks(onlyRight).forward == 0.0);
    CHECK(mapSticks(onlyRight).left == 0.0);
    CHECK(mapSticks(onlyRight).yawCcw == -0.8);
    const StickInput onlyLeftX{.leftX = 0.8, .connected = true};
    CHECK(mapSticks(onlyLeftX).forward == 0.0);
    CHECK(mapSticks(onlyLeftX).left == -0.8);
    CHECK(mapSticks(onlyLeftX).yawCcw == 0.0);
    const StickInput onlyLeftY{.leftY = 0.8, .connected = true};
    CHECK(mapSticks(onlyLeftY).forward == 0.8);
    CHECK(mapSticks(onlyLeftY).left == 0.0);
    CHECK(mapSticks(onlyLeftY).yawCcw == 0.0);
}

TEST_CASE("stick mapping: deadband is a STRICT interior cut — the threshold itself passes") {
    // BUG CAUGHT: the boundary moved (< to <=) so a stick held exactly at the
    // threshold reads 0; or the deadband widened/narrowed; or the cut applied
    // asymmetrically. 0.05 is HA-112's invented constant, pinned as a literal.
    CHECK(kStickDeadband == 0.05);
    CHECK(deadbanded(0.049) == 0.0);
    CHECK(deadbanded(-0.049) == 0.0);
    CHECK(deadbanded(std::nextafter(0.05, 0.0)) == 0.0);
    CHECK(deadbanded(std::nextafter(-0.05, 0.0)) == 0.0);
    CHECK(deadbanded(0.05) == 0.05);
    CHECK(deadbanded(-0.05) == -0.05);
    CHECK(deadbanded(0.0500001) == 0.0500001);
    CHECK(deadbanded(1.0) == 1.0);
    CHECK(deadbanded(-1.0) == -1.0);
    // Inside the band the result is POSITIVE zero regardless of the input's sign —
    // the "negate first, then deadband" order makes mapSticks yield +0.0 for a
    // slightly-right stick, exactly as the old loop did (bit-pinned in the oracle
    // sweep; stated here as a readable fact).
    CHECK(sameBits(deadbanded(-0.02), 0.0));
    CHECK(sameBits(mapSticks(StickInput{.leftX = 0.02, .connected = true}).left, 0.0));
    CHECK_FALSE(sameBits(deadbanded(-0.02), -0.0));
}

TEST_CASE("stick mapping: a DISCONNECTED controller yields exactly zero, whatever the sticks") {
    // BUG CAUGHT: the isConnected() guard dropped — a controller that drops
    // mid-match reads 0 on every channel anyway (HA-103), so the guard looks
    // redundant and is not: it is what makes a stale/garbage axis read on a lost
    // link command NOTHING rather than something, and it must not depend on the
    // budgets (a 0·budget with a negative budget is −0.0, not the +0.0 a default
    // ChassisSpeeds carries — the oracle sweep pins that bit too).
    const StickInput rails{.leftY = 1.0, .leftX = -1.0, .rightX = 1.0, .connected = false};
    const DriveRequest r = mapSticks(rails);
    CHECK(sameBits(r.forward, 0.0));
    CHECK(sameBits(r.left, 0.0));
    CHECK(sameBits(r.yawCcw, 0.0));
    const ChassisSpeeds s = mapSticksToChassisSpeeds(rails, Velocity{60.0}, AngularVelocity{6.0});
    CHECK(sameBits(s, ChassisSpeeds{}));
    const ChassisSpeeds neg =
        mapSticksToChassisSpeeds(rails, Velocity{-60.0}, AngularVelocity{-6.0});
    CHECK(sameBits(neg, ChassisSpeeds{}));
    // And the same sticks CONNECTED are not zero — so the case above is testing the
    // guard, not an all-zero input.
    const StickInput live{.leftY = 1.0, .leftX = -1.0, .rightX = 1.0, .connected = true};
    CHECK(mapSticks(live).forward == 1.0);
}

TEST_CASE("stick mapping: the one-call form equals the two-layer composition when connected") {
    // BUG CAUGHT: the two entry points drifting apart — the library loop uses the
    // one-call form and the bench DRIVE station uses mapSticks() directly; if they
    // ever disagreed, the driver would feel two different robots in two programs,
    // which is the whole reason the mapping was extracted (brief §3.3, §6.2).
    const Velocity lin{60.0};
    const AngularVelocity ang{6.0};
    for (double ly : interestingAxisValues()) {
        for (double rx : {-1.0, -0.3, 0.0, 0.04, 0.05, 0.6}) {
            const StickInput in{.leftY = ly, .leftX = -0.2, .rightX = rx, .connected = true};
            CHECK(sameBits(mapSticksToChassisSpeeds(in, lin, ang),
                           toChassisSpeeds(mapSticks(in), lin, ang)));
        }
    }
}
