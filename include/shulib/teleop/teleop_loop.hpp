#pragma once
//
// runDriverControl — the library's driver-control loop (chunk G1 Part 2a, 2026-09-24; T2's
// integration contract, T2 brief §7).
//
// ── Why it is library code ──────────────────────────────────────────────────────────
// Until this file, the only teleop loop in the tree was `runTeleopLoop`, a 22-line function in
// src/main.cpp's anonymous namespace. It could not be host-tested (it called PROS's
// competition_is_disabled() directly), it read three of the four stick axes (not RightY, which
// T2's tank-stick mode needs), and it mapped the sticks through the raw R1a mapping rather than
// T2's feel layer. This is that loop, rebuilt on teleop::DriveShaper, with its three PROS
// dependencies INJECTED — the disabled predicate, the tick period and the heading source — so
// teleop/ stays PROS-free and the whole loop runs on the A2 host plant under a FakeController
// (the G1 Part 2 brief's ruling M3). Rejected: leaving the loop in src/ (untestable); adding a
// period to motion::ITickPacer (it is in the frozen Chassis constructor's signature, and a new
// pure virtual would break every implementer) — so the period is a config value instead.
//
// ── ONE TICK, IN THIS ORDER ─────────────────────────────────────────────────────────
//   1. disabled()      true ⇒ chassis.cancel() — 0 V + Brake, the facade's panic stop — and
//                      RETURN. PROS deletes the opcontrol task on a field disable, and a deleted
//                      task runs no destructor, so this is the one path on which the drive is
//                      put into its safe state before VEXos cuts the motors itself (R3b Part 0b's
//                      finding, carried over from runTeleopLoop). PROS calls opcontrol() again
//                      when driver control resumes; the caller simply calls this again.
//                      Whether that stop — a PROS brake() since DEFECTS2 — lands in time and
//                      outlasts the cut is A4 register HA-145 (unknown).
//   2. the sticks      isConnected() first; ALL FOUR axes, and the re-zero button, are read
//                      ONLY while connected. A disconnected V5 controller reads 0 on every
//                      channel (HA-103), so the connected flag is the only thing that separates
//                      "driver unplugged" from "sticks centred", and nothing is read that could
//                      pretend otherwise.
//   3. heading()       sampled ONCE per tick, used for both the re-zero and the shaping. It is
//                      called BEFORE this tick's chassis.drive(), which is where the Localizer
//                      updates — so a source that reads the fused estimate sees last tick's
//                      heading (10 ms old); one that reads the IMU directly does not.
//   4. re-zero         the loop's OWN hal::ButtonEdge on cfg.rezeroButton (a per-consumer edge,
//                      HA-104) calls shaper.rezero(heading) ONCE per press, and BEFORE shaping,
//                      so "the way the robot faces now is away from me" governs this very tick.
//   5. shape           shaper.shape(sticks, cfg.tickPeriod, heading.radians, heading.trusted) —
//                      mode → shaping → slew → rotation (driver_control.hpp).
//   6. scale + drive   toChassisSpeeds(request, maxLinearSpeed, maxAngularSpeed) with the
//                      budgets from chassis.motionConfig(), then chassis.drive(…, Frame::Body)
//                      — ALWAYS Body, field-centric included: the rotation already happened
//                      inside shape().
//   6b. release mode   cfg.releaseBrakeMode (default Coast) on every drive motor — also once
//                      at ENTRY, before the first tick (chunk DEFECTS2). drive() pre-empting a
//                      motion cancels it into Brake, and since DEFECTS2 a Brake at 0 V goes to
//                      a V5 as PROS brake(); re-asserting after EVERY drive() — not only after a
//                      pre-emption — keeps the invariant independent of which library path
//                      set another mode, for one set_brake_mode per motor per tick (believed
//                      harmless to a running motor: A4 register HA-146).
//   7. perTick(command, connected, shaper.state())  the caller's panel.
//   8. pacer.pace()    the tick boundary.
//
// No buffer, no queue, no "apply next tick": a stick step read at tick k is in the commanded
// volts of tick k (pinned through the host plant). PERCEIVED latency is not claimed — real
// input-to-motion latency is PROS call cost plus loop rate, and neither is measured.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Host-proven on the A2 plant only. Since G1 Part 2b "shulib Teleop" (src/teleop_program.cpp)
// drives through it, with the default DriverProfile (ArcadeBody) and field-centric selectable but
// OFF — and that version of the program has not run on hardware, so nothing in the feel layer it
// runs has been felt by a driver (HA-112).
//
// PROS-free, sim-free, allocation-free.

#include <cmath>

#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::teleop {

/// What the loop needs that neither the chassis nor the controller can supply.
///
/// State the first TWO, with designated initializers — `{.tickPeriod = …, .rezeroButton = …}`.
/// The period's default is deliberately not a working configuration: a zero period trips
/// `runDriverControl`'s precondition before anything moves, so a config nobody wrote can never
/// run. The button's default is NOT so guarded: a config that leaves `rezeroButton` unset
/// quietly gets `L1` — a shoulder button, which the planned claw bindings take — so a program
/// must name its button ("shulib Teleop" names `A`). Making this default refuse too is owed
/// before any program binds a mechanism to a shoulder button. The third, `releaseBrakeMode`,
/// has a working default (Coast, the team lead's ruling).
struct DriverLoopConfig {
    /// How long one tick is — the pacer's period (on a brain, `ProsTickPacer::kTickMs` = 10 ms,
    /// HA-32). Handed to the shaper as its dt, where only the slew limiter reads it. Must be
    /// finite and > 0. `motion::ITickPacer` has no period of its own, and cannot gain one (M3),
    /// which is why the program states it here.
    units::Time tickPeriod{0.0};
    /// The driver's field-centric re-zero button (`DriveShaper::rezero`). Harmless in a body
    /// mode, where a press only pre-arms the zero for a later switch to `ArcadeField`. Which
    /// button is the PROGRAM's choice, so state it.
    hal::ControllerButton rezeroButton = hal::ControllerButton::L1;
    /// What the drive does when the sticks are released (chunk DEFECTS2): the brake mode the
    /// loop puts on every drive motor at entry and again after every `drive()`, so a library
    /// stop inside the loop — `drive()` pre-empting a motion cancels it into Brake — never
    /// outlives its tick. Default `Coast`, the team lead's ruling (2026-09-26): the drive rolls
    /// free on release, the feel shulib Teleop was driven with on 2026-09-25. `Brake` or `Hold`
    /// make a release stop the drive; since DEFECTS2 the V5 adapter sends them to the motor as
    /// PROS brake() at a zero command (`hal/pros/motor.hpp`, THE STOP — not yet seen on a robot).
    /// The disable path is NOT governed by this: a disable still cancels into Brake.
    hal::BrakeMode releaseBrakeMode = hal::BrakeMode::Coast;
};

/// One tick's heading, as the loop hands it to the shaper: the robot's estimated heading and
/// the caller's verdict on it. What a `runDriverControl` heading source returns.
struct HeadingSample {
    /// The estimated heading, radians, CCW-positive, in the same frame the driver's zero is
    /// captured in. Ignored unless the profile's mode is `ArcadeField`.
    double radians = 0.0;
    /// True only when the estimate is live and the heading is believable (for a robot::Robot:
    /// the Localizer is past `Quality::Uninitialized` and the IMU reports ready). False makes
    /// field-centric fall back to robot-centric and LATCH until the driver re-zeroes.
    bool trusted = false;
};

/// Run driver control until `disabled()` says stop: sticks → `shaper` → budgets → the chassis,
/// once per tick, in the order the file header lists (disabled check, sticks while connected,
/// one heading sample, the re-zero edge, shape, `drive(Body)`, the release mode, `perTick`,
/// `pace`).
///
/// * `chassis` — the facade to drive; its `motionConfig()` supplies the speed budgets
///   (`maxLinearSpeed`, `maxAngularSpeed` — HA-50, provisional).
/// * `controller` — the driver's controller (VEX U's partner is a second call with a second
///   shaper).
/// * `pacer` — the tick boundary: on a brain the provider's `ProsTickPacer`; in a host test the
///   plant-stepping pacer.
/// * `shaper` — the driver's feel (`DriveShaper`, one per driver, held across calls so its
///   field-centric zero and latch survive a disable).
/// * `cfg` — the tick period, the re-zero button and the release brake mode (applied at entry
///   and after every `drive()`). PRECONDITION: `cfg.tickPeriod` finite and > 0, checked before
///   the first tick.
/// * `disabled` — `bool()`: polled FIRST every tick; true ⇒ `chassis.cancel()` (0 V + Brake)
///   and return. The only way out.
/// * `heading` — `HeadingSample()`: called once per tick, after the sticks, before shaping.
/// * `perTick` — `void(const math::ChassisSpeeds& command, bool connected,
///   const DriveShaperState& state)`: called after `drive()`, before `pace()` — the panel's hook.
///
/// Nothing is read from the controller while it is disconnected; the command is then exactly
/// zero whatever the axes claim (HA-103).
template <typename IsDisabled, typename HeadingSource, typename PerTick>
void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller,
                      motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg,
                      IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick) {
    SHULIB_PRECONDITION(std::isfinite(cfg.tickPeriod.value()) && cfg.tickPeriod.value() > 0.0,
                        "runDriverControl: DriverLoopConfig::tickPeriod must be finite and > 0 "
                        "(state it: the pacer's period, 10 ms on a V5)");
    const motion::MotionConfig& budgets = chassis.motionConfig();
    const double dtSeconds = cfg.tickPeriod.value();
    hal::ButtonEdge rezeroEdge{};  // the loop's OWN edge (HA-104: one per consumer)
    // DEFECTS2: the release mode on every drive motor — at entry, and after every drive().
    const auto applyReleaseMode = [&chassis, &cfg] {
        for (hal::IMotor* m : chassis.deps().ctx->driveMotors()) {
            m->setBrakeMode(cfg.releaseBrakeMode);
        }
    };
    applyReleaseMode();
    while (true) {
        // 1. A field disable: safe the drive, then leave.
        if (disabled()) {
            chassis.cancel();
            return;
        }
        // 2. The sticks — every channel read only while the controller is there.
        StickInput sticks{.connected = controller.isConnected()};
        bool rezeroDown = false;
        if (sticks.connected) {
            sticks.leftY = controller.axis(hal::ControllerAxis::LeftY);
            sticks.leftX = controller.axis(hal::ControllerAxis::LeftX);
            sticks.rightX = controller.axis(hal::ControllerAxis::RightX);
            sticks.rightY = controller.axis(hal::ControllerAxis::RightY);  // tank sticks' RIGHT
            rezeroDown = controller.pressed(cfg.rezeroButton);
        }
        // 3. One heading sample for the whole tick.
        const HeadingSample h = heading();
        // 4. The re-zero, once per press, before this tick is shaped.
        if (rezeroEdge.update(rezeroDown)) {
            shaper.rezero(h.radians);
        }
        // 5–6. Shape, scale to the budgets, command in the BODY frame.
        const DriveRequest request = shaper.shape(sticks, dtSeconds, h.radians, h.trusted);
        const math::ChassisSpeeds command =
            toChassisSpeeds(request, budgets.maxLinearSpeed, budgets.maxAngularSpeed);
        chassis.drive(command, math::Frame::Body);
        // 6b. The release mode again, in this same tick: undoes a pre-emption's Brake (DEFECTS2).
        applyReleaseMode();
        // 7–8. The caller's panel, then the tick boundary.
        perTick(command, sticks.connected, shaper.state());
        pacer.pace();
    }
}

}  // namespace shulib::teleop
