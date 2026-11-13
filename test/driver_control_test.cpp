// Adversarial tests for DriveShaper — the composed feel layer (chunk T2, brief
// tests 6, 7, 8 and the second half of 10).
//
// Three properties carry this file:
//
//   * THE ORDER IS A CONTRACT (mode -> shaping -> slew -> rotation). Adjacent
//     steps do not commute, so the pinned numbers below are only reachable by the
//     documented order; swapping any pair moves them by a visible amount.
//   * THE DEFAULT PROFILE DOES NOT SURPRISE A DRIVER WHO DROVE YESTERDAY. It is
//     identical to mapSticks() at centre and at both rails, and inside that range
//     it differs by exactly the deadband rescale — a hand-computed bound, not a
//     tolerance picked to make a test pass.
//   * INPUT REACHES THE MOTORS IN THE SAME TICK. Pinned through the A2 plant with
//     a real Chassis, because "no buffering" is a claim about the whole path and
//     not about one function.
//
// PERCEIVED latency is NOT claimed anywhere: real input-to-motion latency is PROS
// call cost plus loop rate, and neither has been measured. Nothing in this file
// has been felt by a driver either — every default it exercises is invented.
//
// The mutations the T2 brief requires were RUN and each observed red;
// T2-PROGRESS.md records the messages.

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::teleop::AxisShaping;
using shulib::teleop::continuousDeadband;
using shulib::teleop::DriveMode;
using shulib::teleop::DriveRequest;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::expoCurve;
using shulib::teleop::FieldCentricReason;
using shulib::teleop::mapSticks;
using shulib::teleop::shapeAxis;
using shulib::teleop::SlewLimiter;
using shulib::teleop::StickInput;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A profile whose shaping and slew are BOTH on, so the order of the two steps is visible.
[[nodiscard]] DriverProfile orderingProfile() {
    DriverProfile p;
    p.forward = AxisShaping{.deadband = 0.05, .expo = 1.0};
    p.left = p.forward;
    p.yaw = p.forward;
    p.slew.ratePerSecond = 1.0;
    return p;
}

}  // namespace

// ═══ Test 6 — the order inside shape() ═════════════════════════════════════════════

TEST_CASE("T2 order: shaping happens BEFORE slew, and the two orders are far apart") {
    // BUG CAUGHT: the brief's named mutation — the two steps swapped. With expo on, a
    // slew-limited signal pushed through the curve is limited at a rate the driver never
    // asked for: the curve scales the step by a factor that depends on where the stick is.
    // The single tick below separates the orders by a factor of ~700, so no tolerance can
    // hide the swap.
    //
    // Hand-computed, stick slammed from centre to full forward, dt = 0.1 s, rate 1.0/s:
    //   documented order   shape(1.0) = expo(deadband(1.0)) = 1.0, then slew 0 -> 0.1
    //   swapped order      slew(1.0) = 0.1, then shape(0.1) = ((0.1-0.05)/0.95)^3
    //                                                        = 0.0526315789...^3
    //                                                        = 1.4579e-4
    DriveShaper shaper{orderingProfile()};
    const StickInput full{.leftY = 1.0, .connected = true};
    const DriveRequest got = shaper.shape(full, 0.1, 0.0, true);
    CHECK(got.forward == doctest::Approx(0.1).epsilon(1e-12));

    const double rescaled = (0.1 - 0.05) / 0.95;
    const double swapped = rescaled * rescaled * rescaled;
    CHECK(swapped < 2.0e-4);                 // the other order's answer, hand-computed
    CHECK(got.forward > 400.0 * swapped);    // and they are nowhere near each other

    // The slew's own output is reported, and it IS the returned request in a body mode —
    // so a panel can show the cost of the rate limit without recomputing anything.
    CHECK(sameBits(shaper.state().slewOutput.forward, got.forward));
    CHECK(shaper.state().fieldCentric.reason == FieldCentricReason::NotSelected);
}

TEST_CASE("T2 order: rotation happens LAST — the limiter sees the driver's frame, not the robot's") {
    // BUG CAUGHT: rotating before the slew limit. A robot spinning under a perfectly still
    // stick then generates a rotating target, and the limiter spends every tick chasing it:
    // "hold the stick still" becomes a drifting, heading-dependent command. Here the stick
    // is held at full forward while the heading sweeps a full turn; the pre-rotation
    // (slew) output must be completely heading-independent, and it is only that if the
    // rotation is last.
    DriverProfile p = orderingProfile();
    p.mode = DriveMode::ArcadeField;
    DriveShaper shaper{p};
    const StickInput full{.leftY = 1.0, .connected = true};
    // Settle the limiter at the rail first (rate 1.0/s, dt 0.1 s -> 10 ticks).
    for (int i = 0; i < 12; ++i) {
        (void)shaper.shape(full, 0.1, 0.0, true);
    }
    CHECK(shaper.state().slewOutput.forward == doctest::Approx(1.0).epsilon(1e-12));

    for (const double deg : {0.0, 45.0, 90.0, 137.0, 180.0, -90.0, -33.0}) {
        CAPTURE(deg);
        const DriveRequest out = shaper.shape(full, 0.1, deg * Angle::kPi / 180.0, true);
        // The DRIVER-frame signal never moved, whatever the robot did.
        CHECK(shaper.state().slewOutput.forward == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(shaper.state().slewOutput.left == 0.0);
        // The BODY-frame answer did, and by exactly the rotation.
        CHECK(out.forward == doctest::Approx(std::cos(deg * Angle::kPi / 180.0)).epsilon(1e-12));
        CHECK(out.left == doctest::Approx(-std::sin(deg * Angle::kPi / 180.0)).epsilon(1e-12));
        CHECK(shaper.state().fieldCentric.active);
    }
}

// ═══ Test 7 — the default profile against today's mapping ══════════════════════════

TEST_CASE("T2 default: identical to mapSticks() at centre and at both rails, EXACTLY") {
    // BUG CAUGHT: a default that quietly changes what a driver who drove yesterday feels
    // at the two places a driver notices immediately — a centred stick (the robot must not
    // creep, and the sign of that zero is pinned) and a stick at the rail (the robot must
    // still deliver everything). Exact equality, on bits at zero.
    DriveShaper shaper{DriverProfile{}};
    const StickInput centred{.connected = true};
    const DriveRequest zero = shaper.shape(centred, 0.01, 0.0, true);
    CHECK(sameBits(zero.forward, mapSticks(centred).forward));
    CHECK(sameBits(zero.left, mapSticks(centred).left));
    CHECK(sameBits(zero.yawCcw, mapSticks(centred).yawCcw));
    CHECK(sameBits(zero.forward, 0.0));  // POSITIVE zero, as R1a pinned

    for (const double rail : {1.0, -1.0}) {
        CAPTURE(rail);
        const StickInput rails{.leftY = rail, .leftX = rail, .rightX = rail, .connected = true};
        const DriveRequest got = shaper.shape(rails, 0.01, 0.0, true);
        CHECK(got.forward == mapSticks(rails).forward);
        CHECK(got.left == mapSticks(rails).left);
        CHECK(got.yawCcw == mapSticks(rails).yawCcw);
        CHECK(std::abs(got.forward) == 1.0);
    }
}

TEST_CASE("T2 default: everywhere else it differs from mapSticks() by exactly the rescale") {
    // BUG CAUGHT: the default profile drifting away from "reproduce today's feel as closely
    // as a continuous deadband allows". The difference is not a tolerance, it is an
    // equation: for |x| >= d the shaped magnitude is (|x| - d)/(1 - d), so
    //
    //     |naive| - |shaped| = d * (1 - |x|) / (1 - d)
    //
    // which is 0 at the rails, is largest AT the threshold (where it equals d itself), and
    // is never negative — the shaping never asks for MORE authority than the old mapping
    // did. All three facts are pinned, plus the bound 0.05.
    DriveShaper shaper{DriverProfile{}};
    const double d = 0.05;
    double worst = 0.0;
    int swept = 0;
    for (int i = -1000; i <= 1000; ++i) {
        const double x = static_cast<double>(i) / 1000.0;
        const StickInput in{.leftY = x, .connected = true};
        const double shaped = shaper.shape(in, 0.01, 0.0, true).forward;
        const double naive = mapSticks(in).forward;
        const double gap = std::abs(naive) - std::abs(shaped);
        const double predicted = std::abs(x) < d ? std::abs(naive)
                                                 : d * (1.0 - std::abs(x)) / (1.0 - d);
        if (gap < -1e-15 || std::abs(gap - predicted) > 1e-12) {
            CAPTURE(x);
            CAPTURE(shaped);
            CAPTURE(naive);
            CAPTURE(gap);
            CAPTURE(predicted);
            CHECK(gap >= -1e-15);
            CHECK(std::abs(gap - predicted) <= 1e-12);
        }
        worst = std::max(worst, gap);
        ++swept;
    }
    CHECK(swept == 2001);
    CHECK(worst <= d + 1e-15);                      // the hand-computed bound
    CHECK(worst == doctest::Approx(d).epsilon(1e-9));  // and it is REACHED, at the threshold
}

// ═══ Test 8 — one-tick latency, through the plant ══════════════════════════════════

TEST_CASE("T2 latency: a stick step at tick k changes the commanded volts at tick k") {
    // BUG CAUGHT: a buffer, a queue, or any "apply next tick" anywhere between the sticks
    // and the motors. This is the whole path — DriveShaper, toChassisSpeeds, Chassis::drive,
    // the command pipeline, the fake motors — driven by the A2 plant one tick at a time,
    // with the commanded voltage sampled immediately after each drive() call. A one-tick
    // buffer would move the first nonzero sample by exactly one index, which is what the
    // recorded step index below pins.
    const auto kin = shulib::kinematics::TankKinematics(Length{11.0});
    motion_rig::ChassisRig c{kin};
    DriveShaper shaper{DriverProfile{}};
    const Velocity maxLinear{60.0};
    const AngularVelocity maxAngular{6.0};

    constexpr int kStepTick = 5;
    constexpr int kTicks = 12;
    std::vector<double> volts;
    for (int tick = 0; tick < kTicks; ++tick) {
        const StickInput in{.leftY = tick >= kStepTick ? 1.0 : 0.0, .connected = true};
        const DriveRequest req = shaper.shape(in, 0.01, 0.0, true);
        c.chassis.drive(toChassisSpeeds(req, maxLinear, maxAngular), Frame::Body);
        volts.push_back(c.rig.h.motor(0).commandedVoltage().value());
        c.pacer.pace();
    }
    REQUIRE(volts.size() == static_cast<std::size_t>(kTicks));

    // Before the step: the drive is commanded nothing.
    for (int tick = 0; tick < kStepTick; ++tick) {
        CAPTURE(tick);
        CHECK(volts[static_cast<std::size_t>(tick)] == doctest::Approx(0.0).epsilon(1e-12));
    }
    // ON the step tick — not the one after it — the volts move, and substantially.
    CHECK(std::abs(volts[static_cast<std::size_t>(kStepTick)]) > 1.0);
    // And the FIRST tick that moved is the step tick itself.
    int firstMoved = -1;
    for (std::size_t i = 0; i < volts.size(); ++i) {
        if (std::abs(volts[i]) > 1e-9) {
            firstMoved = static_cast<int>(i);
            break;
        }
    }
    CHECK(firstMoved == kStepTick);
}

TEST_CASE("T2 latency: the release is in the same tick too — no lag getting back to zero") {
    // BUG CAUGHT: a buffer that only shows on the rising edge (a "last command" cache that
    // is written after it is read). A driver noticing lag on the STOP is the same defect
    // and the more dangerous half of it.
    const auto kin = shulib::kinematics::TankKinematics(Length{11.0});
    motion_rig::ChassisRig c{kin};
    DriveShaper shaper{DriverProfile{}};
    const Velocity maxLinear{60.0};
    const AngularVelocity maxAngular{6.0};

    constexpr int kReleaseTick = 6;
    std::vector<double> volts;
    for (int tick = 0; tick < 12; ++tick) {
        const StickInput in{.leftY = tick < kReleaseTick ? 1.0 : 0.0, .connected = true};
        const DriveRequest req = shaper.shape(in, 0.01, 0.0, true);
        c.chassis.drive(toChassisSpeeds(req, maxLinear, maxAngular), Frame::Body);
        volts.push_back(c.rig.h.motor(0).commandedVoltage().value());
        c.pacer.pace();
    }
    CHECK(std::abs(volts[static_cast<std::size_t>(kReleaseTick) - 1]) > 1.0);
    // Zero speed commanded on the release tick: the feedforward's kS is gated by the
    // commanded velocity, so a zero request is a zero command in the same tick.
    CHECK(volts[static_cast<std::size_t>(kReleaseTick)]
          == doctest::Approx(0.0).epsilon(1e-9));
}

// ═══ Test 9 (composed) and the disconnected short-circuit ══════════════════════════

TEST_CASE("T2 shaper: a disconnected controller is zero NOW, and the limiters are reset") {
    // BUG CAUGHT: a rate limit that keeps the robot rolling after its driver is gone. With
    // slew on, ramping down is the "natural" implementation and it is wrong: a controller
    // that drops mid-match must stop the robot in the same tick. This also pins that the
    // limiter's HISTORY is cleared, so the tick after a reconnect starts from zero rather
    // than from the command the lost driver last held.
    DriveShaper shaper{orderingProfile()};
    const StickInput full{.leftY = 1.0, .connected = true};
    for (int i = 0; i < 12; ++i) {
        (void)shaper.shape(full, 0.1, 0.0, true);
    }
    CHECK(shaper.state().slewOutput.forward == doctest::Approx(1.0).epsilon(1e-12));

    const StickInput dropped{.leftY = 1.0, .leftX = -1.0, .rightX = 1.0, .rightY = -1.0,
                             .connected = false};
    const DriveRequest out = shaper.shape(dropped, 0.1, 0.0, true);
    CHECK(sameBits(out.forward, 0.0));   // NOW, not over ten ticks
    CHECK(sameBits(out.left, 0.0));
    CHECK(sameBits(out.yawCcw, 0.0));
    CHECK(shaper.state().controllerAbsent);
    CHECK(sameBits(shaper.state().slewOutput.forward, 0.0));

    // Reconnecting starts from zero, so the first tick back is rate-limited from rest.
    const DriveRequest back = shaper.shape(full, 0.1, 0.0, true);
    CHECK(back.forward == doctest::Approx(0.1).epsilon(1e-12));
}

TEST_CASE("T2 shaper: field-centric's latch survives a controller drop") {
    // BUG CAUGHT: clearing the fallback latch on reconnect. A lost radio link says nothing
    // about the heading, and silently restoring field-centric behind the driver's back is
    // exactly the frame flicker the latch exists to prevent.
    DriverProfile p;
    p.mode = DriveMode::ArcadeField;
    DriveShaper shaper{p};
    const StickInput full{.leftY = 1.0, .connected = true};
    (void)shaper.shape(full, 0.01, Angle::kPi / 2.0, false);  // heading lost -> latched
    CHECK(shaper.state().fieldCentric.fellBack);

    (void)shaper.shape(StickInput{.connected = false}, 0.01, Angle::kPi / 2.0, true);
    CHECK(shaper.state().fieldCentric.fellBack);   // the drop did not clear it

    const DriveRequest still = shaper.shape(full, 0.01, Angle::kPi / 2.0, true);
    CHECK(still.forward == 1.0);                   // still robot-centric
    CHECK(shaper.state().fieldCentric.reason == FieldCentricReason::LatchedUntilRezero);

    shaper.rezero(Angle::kPi / 2.0);               // the driver's button
    const DriveRequest rotated = shaper.shape(full, 0.01, Angle::kPi / 2.0, true);
    CHECK(rotated.forward == doctest::Approx(1.0).epsilon(1e-14));  // the new zero IS here
    CHECK(shaper.state().fieldCentric.active);
}

TEST_CASE("T2 shaper: setProfile() swaps the tuning and resets the limiters; reset() clears all") {
    // BUG CAUGHT: a limiter carrying its history across a rate change, so the first tick
    // after a driver swap obeys neither driver's rate.
    DriveShaper shaper{orderingProfile()};
    const StickInput full{.leftY = 1.0, .connected = true};
    for (int i = 0; i < 12; ++i) {
        (void)shaper.shape(full, 0.1, 0.0, true);
    }
    CHECK(shaper.state().slewOutput.forward == doctest::Approx(1.0).epsilon(1e-12));

    DriverProfile slower = orderingProfile();
    slower.slew.ratePerSecond = 0.5;
    shaper.setProfile(slower);
    CHECK(shaper.profile().slew.ratePerSecond == 0.5);
    CHECK(shaper.shape(full, 0.1, 0.0, true).forward == doctest::Approx(0.05).epsilon(1e-12));

    shaper.reset();
    CHECK(shaper.state().slewOutput.forward == 0.0);
    CHECK(shaper.fieldCentric().zeroRadians() == 0.0);
    CHECK_FALSE(shaper.fieldCentric().fellBack());
}

// ═══ Test 10 — the precondition's home does not move ═══════════════════════════════

TEST_CASE("T2 finiteness: the shaper is total; Chassis::drive is still where the check lives") {
    // BUG CAUGHT: the finiteness contract migrating into the feel layer. It belongs at
    // Chassis::drive and at the motor adapter, and it must stay there — a teleop tick that
    // can abort the program is worse than one that commands a NaN, because the abort takes
    // the whole robot with it. Two halves: the shaper RETURNS on hostile input (no
    // precondition of its own), and the precondition downstream still fires on what it
    // hands over.
    DriveShaper shaper{DriverProfile{}};
    const StickInput bad{.leftY = kNaN, .leftX = kInf, .rightX = -kInf, .connected = true};
    const DriveRequest out = shaper.shape(bad, 0.01, 0.0, true);
    CHECK(std::isnan(out.forward));
    CHECK_FALSE(std::isfinite(out.left));
    CHECK_FALSE(std::isfinite(out.yawCcw));

    // Finite sticks in, finite request out — for the whole sweep and every mode.
    for (const DriveMode mode : {DriveMode::ArcadeBody, DriveMode::ArcadeField,
                                 DriveMode::TankSticks}) {
        DriverProfile p;
        p.mode = mode;
        DriveShaper s{p};
        for (int i = -20; i <= 20; ++i) {
            const double a = static_cast<double>(i) / 20.0;
            const StickInput in{.leftY = a, .leftX = -a, .rightX = a, .rightY = -a,
                                .connected = true};
            const DriveRequest r = s.shape(in, 0.01, 1.1, true);
            if (!std::isfinite(r.forward) || !std::isfinite(r.left)
                || !std::isfinite(r.yawCcw)) {
                CAPTURE(static_cast<int>(mode));
                CAPTURE(a);
                CHECK(std::isfinite(r.forward));
                CHECK(std::isfinite(r.left));
                CHECK(std::isfinite(r.yawCcw));
            }
        }
    }

    // The precondition still fires where it always did, on the value the shaper handed over.
    const auto kin = shulib::kinematics::TankKinematics(Length{11.0});
    motion_rig::ChassisRig c{kin};
    CHECK_THROWS(c.chassis.drive(toChassisSpeeds(out, Velocity{60.0}, AngularVelocity{6.0}),
                                 Frame::Body));
}
