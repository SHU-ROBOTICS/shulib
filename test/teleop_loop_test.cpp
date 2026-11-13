// teleop::runDriverControl — the library's driver-control loop, on the A2 host plant (chunk G1
// Part 2a, brief §4 test 4 and the loop half of test 5).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// The loop's properties, each as a behaviour of the WHOLE path — controller → DriveShaper →
// budgets → Chassis::drive → the plant's motors — rather than as a claim about one function:
//
//   * one-tick latency: a stick step read at tick k is in the commanded volts of tick k;
//   * disabled() ⇒ cancel() — 0 V and Brake at the motors — and return;
//   * disconnected ⇒ a zero command whatever the axes claim, and NOTHING read while
//     disconnected (the controller below lies about its axes when disconnected, and counts);
//   * the re-zero is the loop's OWN edge: once per press, and before this tick is shaped;
//   * an untrusted heading ⇒ field-centric falls back, visibly, in the state perTick receives;
//   * tank sticks move the RIGHT side from RightY (the axis the old loop never read);
//   * the configured tick period reaches the shaper; an unstated period never runs;
//   * the command is always Body-frame;
//   * with the DEFAULT profile, the loop differs from the old mapping by exactly the continuous
//     deadband's rescale — identical at 0 and at both rails — on a full sweep of every axis.
//
// NO HARDWARE CLAIM. The plant is the only thing this loop has driven; no driver has felt it.

#include "doctest.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/field_centric.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::BrakeMode;
using shulib::hal::ControllerAxis;
using shulib::hal::ControllerButton;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::teleop::DriveMode;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::FieldCentricReason;
using shulib::teleop::HeadingSample;
using shulib::teleop::runDriverControl;
using shulib::teleop::StickInput;
using shulib::units::Length;
using shulib::units::Time;

namespace {

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

/// A controller that COUNTS every read the loop makes and LIES while disconnected: it keeps
/// reporting whatever axes and buttons the test set, connected or not. A real V5 controller reads
/// 0 when disconnected (HA-103), which would hide a loop that polled it anyway; this one makes
/// such a loop visible twice — in the read counters, and in any command it would produce.
class CountingController final : public shulib::hal::IController {
public:
    [[nodiscard]] double axis(ControllerAxis a) const override {
        const auto i = static_cast<std::size_t>(a);
        ++reads[i];
        if (!connected) {
            ++readsWhileDisconnected;
        }
        return axes[i];
    }
    [[nodiscard]] bool pressed(ControllerButton b) const override {
        ++buttonReads;
        if (!connected) {
            ++readsWhileDisconnected;
        }
        return buttons[static_cast<std::size_t>(b)];
    }
    [[nodiscard]] bool isConnected() const override { return connected; }

    void set(ControllerAxis a, double v) { axes[static_cast<std::size_t>(a)] = v; }
    void hold(ControllerButton b, bool down) { buttons[static_cast<std::size_t>(b)] = down; }
    void centre() { axes = {}; }

    std::array<double, 4> axes{};
    std::array<bool, 12> buttons{};
    bool connected = true;
    mutable std::array<int, 4> reads{};
    mutable int buttonReads = 0;
    mutable int readsWhileDisconnected = 0;
};

/// What one tick produced, as perTick saw it — plus the plant motors' commanded volts at that
/// instant (after drive(), before pace(): exactly the volts this tick commanded).
struct Tick {
    ChassisSpeeds command{};
    bool connected = false;
    DriveShaperState state{};
    double leftVolts = 0.0;
    double rightVolts = 0.0;
    BrakeMode leftBrake = BrakeMode::Coast;  ///< the left motor's brake mode at that instant
    double zero = 0.0;  ///< the shaper's field-centric zero after this tick
};

constexpr DriverLoopConfig kLoopCfg{.tickPeriod = Time{0.01}, .rezeroButton = ControllerButton::A};

/// Run the loop for `ticks` ticks. `script(k)` runs at the TOP of tick k (from inside the
/// disabled predicate, which the loop polls first) to set the controller for that tick; the
/// predicate reports "disabled" once `ticks` have run. `heading(k)` is tick k's heading sample.
template <typename Script, typename Heading>
std::vector<Tick> runTicks(motion_rig::ChassisRig& c, CountingController& pad,
                           DriveShaper& shaper, int ticks, Script&& script, Heading&& heading,
                           const DriverLoopConfig& cfg = kLoopCfg) {
    std::vector<Tick> log;
    int k = 0;
    runDriverControl(
        c.chassis, pad, c.pacer, shaper, cfg,
        [&] {
            if (k >= ticks) {
                return true;
            }
            script(k);
            return false;
        },
        [&] { return heading(k); },
        [&](const ChassisSpeeds& command, bool connected, const DriveShaperState& state) {
            log.push_back(Tick{.command = command,
                               .connected = connected,
                               .state = state,
                               .leftVolts = c.rig.h.motor(0).commandedVoltage().value(),
                               .rightVolts = c.rig.h.motor(1).commandedVoltage().value(),
                               .leftBrake = c.rig.h.motor(0).brakeMode(),
                               .zero = shaper.fieldCentric().zeroRadians()});
            ++k;
        });
    return log;
}

HeadingSample trustedZero(int /*tick*/) { return HeadingSample{.radians = 0.0, .trusted = true}; }

const shulib::kinematics::TankKinematics kTank{Length{11.0}};

}  // namespace

// ═══ 4a. One-tick latency, through the loop ═════════════════════════════════════════
TEST_CASE("G1 2a loop 4a: a stick step read at tick k is in the commanded volts of tick k — and "
          "the release too") {
    // BUG CAUGHT (the brief's mutation "add a one-tick buffer"): the loop commanding the
    // PREVIOUS tick's request — "apply next tick" anywhere between the controller and the
    // motors. The first moving tick would slip by exactly one index.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriveShaper shaper{DriverProfile{}};
    constexpr int kStep = 5;
    constexpr int kRelease = 9;
    const auto log = runTicks(
        c, pad, shaper, 14,
        [&](int k) { pad.set(ControllerAxis::LeftY, (k >= kStep && k < kRelease) ? 1.0 : 0.0); },
        trustedZero);
    REQUIRE(log.size() == 14U);
    int firstMoved = -1;
    for (std::size_t i = 0; i < log.size(); ++i) {
        if (std::abs(log[i].leftVolts) > 1e-9) {
            firstMoved = static_cast<int>(i);
            break;
        }
    }
    CHECK(firstMoved == kStep);
    CHECK(log[static_cast<std::size_t>(kStep)].leftVolts > 1.0);
    CHECK(log[static_cast<std::size_t>(kStep)].rightVolts > 1.0);
    CHECK(log[static_cast<std::size_t>(kRelease) - 1].leftVolts > 1.0);
    CHECK(log[static_cast<std::size_t>(kRelease)].leftVolts == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(log[static_cast<std::size_t>(kStep)].command.vx().value()
          == c.chassis.motionConfig().maxLinearSpeed.value());
}

// ═══ 4b. A field disable: safe, then gone ═══════════════════════════════════════════
TEST_CASE("G1 2a loop 4b: disabled() ⇒ cancel() — 0 V and Brake at the motors — and the loop "
          "RETURNS") {
    // BUG CAUGHT (the brief's mutation "skip the cancel"): leaving on a disable with the last
    // command still on the motors. PROS deletes the opcontrol task on a field disable, so this
    // is the ONE place the drive is put in its safe state before VEXos cuts output itself.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriveShaper shaper{DriverProfile{}};

    SUBCASE("after five ticks at full stick") {
        const auto log = runTicks(
            c, pad, shaper, 5, [&](int) { pad.set(ControllerAxis::LeftY, 1.0); }, trustedZero);
        REQUIRE(log.size() == 5U);
        CHECK(log.back().leftVolts > 1.0);                // it WAS driving…
        CHECK(log.back().leftBrake != BrakeMode::Brake);  // …and not braked while it did
        // …and the loop has returned (we are here), with the drive safed.
        CHECK(c.rig.h.motor(0).commandedVoltage().value() == 0.0);
        CHECK(c.rig.h.motor(1).commandedVoltage().value() == 0.0);
        CHECK(c.rig.h.motor(0).brakeMode() == BrakeMode::Brake);
        CHECK(c.rig.h.motor(1).brakeMode() == BrakeMode::Brake);
        CHECK(c.pacer.paces == 5);  // the disabled tick paced nothing
    }
    SUBCASE("disabled on the very first tick: nothing driven, nothing read, still safed") {
        int polled = 0;
        runDriverControl(
            c.chassis, pad, c.pacer, shaper, kLoopCfg,
            [&] {
                ++polled;
                return true;
            },
            [&] { return trustedZero(0); },
            [&](const ChassisSpeeds&, bool, const DriveShaperState&) {
                FAIL("perTick ran on a disabled tick");
            });
        CHECK(polled == 1);
        CHECK(pad.reads == std::array<int, 4>{});
        CHECK(c.pacer.paces == 0);
        CHECK(c.rig.h.motor(0).brakeMode() == BrakeMode::Brake);
    }
}

// ═══ 4c. A dropped controller commands NOTHING, and is not read ════════════════════
TEST_CASE("G1 2a loop 4c: disconnected ⇒ an exactly-zero command whatever the axes claim, and no "
          "channel is read while disconnected; connected ⇒ all FOUR axes, once per tick") {
    // BUG CAUGHT (the brief's mutation "poll axes while disconnected"): HA-103 — a disconnected
    // V5 controller reads 0 on every channel, so reading it proves nothing and the connected
    // flag is the only truth. This controller does NOT read 0: it keeps claiming full deflection
    // on every axis and a held re-zero button, so a loop that read it would be counted.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriveShaper shaper{DriverProfile{}};
    const auto log = runTicks(
        c, pad, shaper, 12,
        [&](int k) {
            pad.connected = k < 4 || k >= 8;  // drops for ticks 4..7
            pad.set(ControllerAxis::LeftY, 1.0);
            pad.set(ControllerAxis::LeftX, -1.0);
            pad.set(ControllerAxis::RightX, 1.0);
            pad.set(ControllerAxis::RightY, -1.0);
            pad.hold(ControllerButton::A, true);
        },
        trustedZero);
    REQUIRE(log.size() == 12U);
    for (std::size_t k = 4; k < 8; ++k) {
        CAPTURE(k);
        CHECK_FALSE(log[k].connected);
        CHECK(sameBits(log[k].command.vx().value(), 0.0));
        CHECK(sameBits(log[k].command.vy().value(), 0.0));
        CHECK(sameBits(log[k].command.omega().value(), 0.0));
        CHECK(log[k].state.controllerAbsent);
    }
    CHECK(pad.readsWhileDisconnected == 0);
    // Eight connected ticks, and on each one every axis — RightY included — read exactly once.
    for (std::size_t a = 0; a < 4; ++a) {
        CAPTURE(a);
        CHECK(pad.reads[a] == 8);
    }
    CHECK(pad.buttonReads == 8);
    CHECK(log[3].command.vx().value() > 0.0);  // connected: the stick is honoured
    CHECK(log[8].command.vx().value() > 0.0);  // and again after the reconnect
}

// ═══ 4d. The re-zero: the loop's own edge, once per press, before shaping ═══════════
TEST_CASE("G1 2a loop 4d: the re-zero button calls rezero() ONCE per press, with that tick's "
          "heading, and governs the same tick") {
    // BUG CAUGHT: re-zeroing on the button LEVEL (the zero would chase the heading for as long
    // as the button is held), sharing an edge with another consumer (HA-104), and re-zeroing
    // AFTER the tick is shaped (the press would take effect one tick late, and the latch would
    // still show on the tick the driver pressed).
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriverProfile p;
    p.mode = DriveMode::ArcadeField;
    DriveShaper shaper{p};
    // Heading: untrusted for ticks 0..2 (the fallback latches), then trusted and turning 0.1 rad
    // per tick.
    const auto heading = [](int k) {
        return HeadingSample{.radians = 0.1 * static_cast<double>(k), .trusted = k >= 3};
    };
    const auto log = runTicks(
        c, pad, shaper, 14,
        [&](int k) {
            pad.set(ControllerAxis::LeftY, 0.5);
            // Held for ticks 5..8, released 9..10, pressed again 11..12, released 13.
            pad.hold(ControllerButton::A, (k >= 5 && k <= 8) || (k >= 11 && k <= 12));
        },
        heading);
    REQUIRE(log.size() == 14U);
    CHECK(log[2].state.fieldCentric.reason == FieldCentricReason::HeadingNotTrusted);
    CHECK(log[4].state.fieldCentric.reason == FieldCentricReason::LatchedUntilRezero);
    // The press at tick 5 set the zero to tick 5's heading, and THAT tick was already rotated.
    CHECK(log[5].zero == doctest::Approx(0.5).epsilon(1e-12));
    CHECK(log[5].state.fieldCentric.reason == FieldCentricReason::Active);
    CHECK(log[5].state.fieldCentric.active);
    // Held through tick 8: the zero did NOT follow the heading.
    CHECK(log[8].zero == doctest::Approx(0.5).epsilon(1e-12));
    // The second press, at tick 11, moved it once more.
    CHECK(log[10].zero == doctest::Approx(0.5).epsilon(1e-12));
    CHECK(log[11].zero == doctest::Approx(1.1).epsilon(1e-12));
    CHECK(log[13].zero == doctest::Approx(1.1).epsilon(1e-12));
}

TEST_CASE("G1 2a loop 4d': a re-zero pressed on a heading that is NOT A NUMBER keeps the old zero "
          "and latches the fallback — it never poisons a later tick") {
    // BUG CAUGHT (found while wiring the button; fixed where it lives, FieldCentric::rezero): a
    // non-finite heading handed to rezero() used to become the ZERO, with the latch cleared — and
    // the next TRUSTED tick then rotated by `heading - NaN`: a NaN command that Chassis::drive
    // rejects by precondition, i.e. the teleop task dying ticks after the press with nothing on
    // the panel to say why.
    //
    // Two halves. The CLASS half is the discriminating one: it calls rezero() and then a trusted
    // request directly, which is what any caller of DriveShaper::rezero can do. The LOOP half is
    // belt and braces: this loop samples the heading once and re-zeroes BEFORE shaping, so the
    // same tick's shape() already re-latches on the same bad heading — but a loop that re-zeroed
    // AFTER shaping (the order the brief listed) would clear that latch and carry the NaN zero
    // into the next tick, and it is the class fix that makes that order safe too.
    shulib::teleop::FieldCentric fc;
    fc.rezero(0.7);
    for (const double bad : {std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(), std::nan("")}) {
        CAPTURE(bad);
        fc.rezero(bad);
        CHECK(fc.zeroRadians() == 0.7);  // the old zero survived
        CHECK(fc.fellBack());            // and the controls fell back, visibly
        const auto r = fc.request(shulib::teleop::DriveRequest{1.0, 0.0, 0.0}, 0.3, true);
        CHECK(std::isfinite(r.request.forward));
        CHECK(std::isfinite(r.request.left));
        CHECK(r.state.reason == FieldCentricReason::LatchedUntilRezero);
    }
    fc.rezero(-0.2);  // a real heading clears it, as always
    CHECK(fc.zeroRadians() == -0.2);
    CHECK_FALSE(fc.fellBack());

    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriverProfile p;
    p.mode = DriveMode::ArcadeField;
    DriveShaper shaper{p};
    shaper.rezero(0.25);  // a good zero, before the loop starts
    const double nan = std::nan("");
    const auto heading = [nan](int k) {
        return k == 2 ? HeadingSample{.radians = nan, .trusted = true}  // a LYING source
                      : HeadingSample{.radians = 0.3, .trusted = true};
    };
    std::vector<Tick> log;
    CHECK_NOTHROW(log = runTicks(
                      c, pad, shaper, 8,
                      [&](int k) {
                          pad.set(ControllerAxis::LeftY, 1.0);
                          pad.hold(ControllerButton::A, k == 2);
                      },
                      heading));
    REQUIRE(log.size() == 8U);
    for (const Tick& t : log) {
        CHECK(std::isfinite(t.command.vx().value()));
        CHECK(std::isfinite(t.command.vy().value()));
        CHECK(t.zero == 0.25);
    }
    CHECK(log[2].state.fieldCentric.fellBack);
    CHECK(log[7].state.fieldCentric.reason == FieldCentricReason::LatchedUntilRezero);
}

// ═══ 4e. An untrusted heading: field-centric falls back, visibly ═════════════════════
TEST_CASE("G1 2a loop 4e: an untrusted heading ⇒ state().fieldCentric.fellBack, the command "
          "UNROTATED, and the latch holds when trust returns") {
    // BUG CAUGHT: a loop that dropped the trust verdict (passing `true`, or never calling the
    // source) — the controls would silently rotate on a heading nobody vouched for.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriverProfile p;
    p.mode = DriveMode::ArcadeField;
    DriveShaper shaper{p};
    const double quarter = Angle::kPi / 2.0;
    const auto log = runTicks(
        c, pad, shaper, 6, [&](int) { pad.set(ControllerAxis::LeftY, 1.0); },
        [quarter](int k) { return HeadingSample{.radians = quarter, .trusted = k >= 3}; });
    REQUIRE(log.size() == 6U);
    for (std::size_t k = 0; k < 3; ++k) {
        CAPTURE(k);
        CHECK(log[k].state.fieldCentric.fellBack);
        CHECK(log[k].state.fieldCentric.reason == FieldCentricReason::HeadingNotTrusted);
        // Unrotated: full forward stays full FORWARD. Rotated by a quarter turn it would be
        // (almost) all sideways.
        CHECK(log[k].command.vx().value() == c.chassis.motionConfig().maxLinearSpeed.value());
    }
    CHECK(log[5].state.fieldCentric.fellBack);
    CHECK(log[5].state.fieldCentric.reason == FieldCentricReason::LatchedUntilRezero);
}

// ═══ 4f. Tank sticks: the RIGHT stick's Y drives the RIGHT side ══════════════════════
TEST_CASE("G1 2a loop 4f: in tank-stick mode the RIGHT stick's Y moves the RIGHT side — the axis "
          "the old loop never read") {
    // BUG CAUGHT (the brief's mutation "never read RightY"): runTeleopLoop read LeftY, LeftX and
    // RightX only, so a tank-stick driver's right hand would move nothing at all.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriverProfile p;
    p.mode = DriveMode::TankSticks;
    DriveShaper shaper{p};
    const auto log = runTicks(
        c, pad, shaper, 8,
        [&](int k) {
            pad.centre();
            if (k < 4) {
                pad.set(ControllerAxis::RightY, 1.0);  // right hand only
            } else {
                pad.set(ControllerAxis::LeftY, 1.0);   // left hand only
            }
        },
        trustedZero);
    REQUIRE(log.size() == 8U);
    // Right stick alone: the right side is driven forward, harder than the left.
    CHECK(log[3].rightVolts > 1.0);
    CHECK(log[3].rightVolts > log[3].leftVolts + 1.0);
    CHECK(log[3].command.omega().value() > 0.0);  // right side faster = a left (CCW) turn
    // Left stick alone: the mirror image.
    CHECK(log[7].leftVolts > 1.0);
    CHECK(log[7].leftVolts > log[7].rightVolts + 1.0);
    CHECK(log[7].command.omega().value() < 0.0);
    CHECK(pad.reads[static_cast<std::size_t>(ControllerAxis::RightY)] == 8);
}

// ═══ 4g. The period, the frame, and a config nobody wrote ═══════════════════════════
TEST_CASE("G1 2a loop 4g: the configured tick period reaches the shaper; the command is always "
          "BODY-frame; an unstated period never runs") {
    SUBCASE("the period is the shaper's dt (a slew limit makes it visible)") {
        // BUG CAUGHT: a loop that handed the shaper a constant, zero, or its own guess instead of
        // the program's stated period — invisible with the shipped slew-off profile, and a
        // silently wrong rate limit the day a driver turns slew on.
        for (const double period : {0.01, 0.02}) {
            CAPTURE(period);
            motion_rig::ChassisRig c{kTank};
            CountingController pad;
            DriverProfile p;
            p.slew.ratePerSecond = 1.0;
            DriveShaper shaper{p};
            const DriverLoopConfig cfg{.tickPeriod = Time{period},
                                       .rezeroButton = ControllerButton::A};
            const auto log = runTicks(
                c, pad, shaper, 1, [&](int) { pad.set(ControllerAxis::LeftY, 1.0); },
                trustedZero, cfg);
            REQUIRE(log.size() == 1U);
            CHECK(log[0].state.slewOutput.forward == doctest::Approx(period).epsilon(1e-12));
        }
    }
    SUBCASE("always Body: a robot facing +Y still drives FORWARD on a forward stick") {
        // BUG CAUGHT: commanding Frame::Field. At a 90 degree heading a field-frame "+X" is the
        // tank's SIDEWAYS, which it cannot do — both sides would sit near zero. In Body frame the
        // two sides are driven forward, equally, from the first tick.
        shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
        cfg.plant.initialPose = shulib::math::Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(90.0)};
        motion_rig::ChassisRig c{kTank, cfg};
        CountingController pad;
        DriveShaper shaper{DriverProfile{}};
        // 20 ticks: past the Localizer's 0.1 s boot settle window, so both of a Field-frame
        // loop's failure modes are covered — zero volts while the estimate is not yet live, and
        // a sideways (untrackable) command once it is.
        const auto log = runTicks(
            c, pad, shaper, 20, [&](int) { pad.set(ControllerAxis::LeftY, 1.0); }, trustedZero);
        REQUIRE(log.size() == 20U);
        //
        // The threshold is 5 V, not "non-zero", and that was MEASURED: under a Field-frame
        // mutant the command pipeline still puts kS = 1.2 V on both sides, because the rotated
        // forward component is cos(90 deg) * 60 ~ 4e-15 in/s rather than an exact zero and kS is
        // gated on the sign of the commanded wheel speed. Full stick forward in Body frame is
        // ~11 V; 5 V separates "driving" from "static-friction feedforward on a rounding residual".
        int driven = 0;
        for (const Tick& t : log) {
            CHECK(t.leftVolts == doctest::Approx(t.rightVolts).epsilon(1e-9));
            if (t.leftVolts > 5.0) {
                ++driven;
            }
        }
        CHECK(driven == 20);
    }
    SUBCASE("DriverLoopConfig{} — a zero period — is refused before a single tick") {
        motion_rig::ChassisRig c{kTank};
        CountingController pad;
        DriveShaper shaper{DriverProfile{}};
        int polled = 0;
        const auto never = [&](const ChassisSpeeds&, bool, const DriveShaperState&) {};
        CHECK_THROWS_AS(runDriverControl(
                            c.chassis, pad, c.pacer, shaper, DriverLoopConfig{},
                            [&] {
                                ++polled;
                                return false;
                            },
                            [] { return trustedZero(0); }, never),
                        shulib::PreconditionError);
        const DriverLoopConfig nanPeriod{.tickPeriod = Time{std::nan("")},
                                         .rezeroButton = ControllerButton::A};
        CHECK_THROWS_AS(runDriverControl(
                            c.chassis, pad, c.pacer, shaper, nanPeriod, [&] { return false; },
                            [] { return trustedZero(0); }, never),
                        shulib::PreconditionError);
        CHECK(polled == 0);
        CHECK(c.pacer.paces == 0);
    }
}

// ═══ 5b. The default profile vs the old mapping, on a sweep, through the loop ═══════
TEST_CASE("G1 2a loop 5: with the DEFAULT profile the loop's command equals the old loop's "
          "mapping at 0 and at both rails, and differs elsewhere by exactly the deadband rescale") {
    // BUG CAUGHT: a Teleop whose feel changed by MORE than the one thing T2 changed. The old
    // loop (src/main.cpp runTeleopLoop) sent mapSticksToChassisSpeeds(sticks, budgets); this one
    // sends toChassisSpeeds(shaper.shape(...), budgets). With the shipped profile the only
    // difference is the continuous deadband, whose cost is an equation, not a tolerance:
    //     |old| - |new| = budget * d * (1 - |x|) / (1 - d)   for |x| >= d, and |old| inside it,
    // never negative, zero at the rails, and at most budget * d. All three axes are swept from
    // -1 to +1 in steps of 0.01, one axis at a time, through the WHOLE loop.
    motion_rig::ChassisRig c{kTank};
    CountingController pad;
    DriveShaper shaper{DriverProfile{}};
    const double d = shulib::teleop::kDefaultDeadband;
    const double maxLinear = c.chassis.motionConfig().maxLinearSpeed.value();
    const double maxAngular = c.chassis.motionConfig().maxAngularSpeed.value();
    constexpr int kSteps = 201;
    const ControllerAxis axes[3] = {ControllerAxis::LeftY, ControllerAxis::LeftX,
                                    ControllerAxis::RightX};
    std::vector<StickInput> fed;
    const auto log = runTicks(
        c, pad, shaper, 3 * kSteps,
        [&](int k) {
            pad.centre();
            const double x = -1.0 + 0.01 * static_cast<double>(k % kSteps);
            pad.set(axes[k / kSteps], x);
            fed.push_back(StickInput{.leftY = pad.axes[1],
                                     .leftX = pad.axes[0],
                                     .rightX = pad.axes[2],
                                     .rightY = pad.axes[3],
                                     .connected = true});
        },
        trustedZero);
    REQUIRE(log.size() == static_cast<std::size_t>(3 * kSteps));
    int exactAtRailsAndCentre = 0;
    for (std::size_t k = 0; k < log.size(); ++k) {
        const ChassisSpeeds old = shulib::teleop::mapSticksToChassisSpeeds(
            fed[k], c.chassis.motionConfig().maxLinearSpeed,
            c.chassis.motionConfig().maxAngularSpeed);
        const std::size_t axis = k / kSteps;
        const double x = axis == 0 ? fed[k].leftY : axis == 1 ? fed[k].leftX : fed[k].rightX;
        const double budget = axis == 2 ? maxAngular : maxLinear;
        const double oldV = axis == 0   ? old.vx().value()
                            : axis == 1 ? old.vy().value()
                                        : old.omega().value();
        const double newV = axis == 0   ? log[k].command.vx().value()
                            : axis == 1 ? log[k].command.vy().value()
                                        : log[k].command.omega().value();
        const double ax = std::abs(x);
        if (ax < 1e-12 || std::abs(ax - 1.0) < 1e-12) {
            // Centre and rails: identical to the bit.
            CAPTURE(k);
            CHECK(sameBits(oldV, newV));
            ++exactAtRailsAndCentre;
            continue;
        }
        const double gap = std::abs(oldV) - std::abs(newV);
        const double predicted =
            ax < d ? std::abs(oldV) : budget * d * (1.0 - ax) / (1.0 - d);
        if (gap < -1e-9 || std::abs(gap - predicted) > 1e-9 || gap > budget * d + 1e-9) {
            CAPTURE(k);
            CAPTURE(x);
            CAPTURE(oldV);
            CAPTURE(newV);
            CHECK(gap >= -1e-9);
            CHECK(std::abs(gap - predicted) <= 1e-9);
            CHECK(gap <= budget * d + 1e-9);
        }
        // The two never disagree on DIRECTION.
        CHECK((oldV == 0.0 || newV == 0.0 || (oldV > 0.0) == (newV > 0.0)));
    }
    CHECK(exactAtRailsAndCentre == 9);  // -1, 0, +1 on each of three axes
}
