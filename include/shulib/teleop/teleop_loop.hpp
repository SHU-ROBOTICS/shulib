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
//                      inside shape(). The DRIVE PROTECTIONS (chunk T4 Part 2, below) wrap it:
//                      their beforeDrive() runs just before it, and while a cut or a refusal is
//                      in force drive() is handed a ZERO command instead of this one; their
//                      afterDrive() runs just after it, and may CUT in this same tick.
//   6b. release mode   cfg.releaseBrakeMode (default Coast) on every drive motor — also once
//                      at ENTRY, before the first tick (chunk DEFECTS2). drive() pre-empting a
//                      motion cancels it into Brake, and since DEFECTS2 a Brake at 0 V goes to
//                      a V5 as PROS brake(); re-asserting after EVERY drive() — not only after a
//                      pre-emption — keeps the invariant independent of which library path
//                      set another mode, for one set_brake_mode per motor per tick (believed
//                      harmless to a running motor: A4 register HA-146).
//   7. perTick(command, connected, shaper.state())  the caller's panel — or, when the caller's
//                      perTick takes a FOURTH argument, perTick(…, protection.state()), the
//                      drive protections' state (DriveProtectionState). The three-argument form
//                      compiles and sees exactly what it saw before.
//   8. pacer.pace()    the tick boundary.
//
// THE DRIVE PROTECTIONS (chunk T4 Part 2, 2026-09-29; the team lead's ruling R4): the old
// "shulib Drive"'s mid-match absence rule and its two re-arming cuts, applied HERE, for every
// program that drives through this loop — teleop/drive_protection.hpp has the rules. After the
// precondition checks (the protections' numbers are one of them: validProtectionConfig, so a bad
// DriverLoopConfig::protection commands NOTHING), ONE DriveProtection is built and enter()'d —
// it REFUSES from tick 1 a drive whose ABSENT marks, left in the groups by an earlier entry, are
// more than two in total. Each tick: beforeDrive() → drive() (the stick, or zero while cut or
// refused) → afterDrive() → 6b and the rest as above. On the disable, after cancel(), its one
// summary line per group. Its destructor clears every group's exclusion mask on every way out of
// this function — the disable return and a throw — so motion code after driver control sees the
// groups as it expects. The way out it cannot cover is step 1's: when PROS deletes the task
// before the loop polls the disable, no destructor runs, and the mask holds the last tick's
// FROZEN members until the next entry's enter() clears it (drive_protection.hpp — stated, not
// solved). A cut never delays the disable: step 1 still runs first on every tick.
//
// With MECHANISM BINDINGS (the overload taking a span; chunk T3, teleop/mechanism_binding.hpp),
// three things are added and nothing above moves: BEFORE tick 1 every binding is reset() and
// makeSafe()'d (after the same precondition checks, and `validBindings()` is one of them — and,
// between the two, a claim still on a bound mechanism is RELEASED and said: the team lead's ruling
// of 2026-09-26, T3 Part 2, below); in step
// 1 every binding is makeSafe()'d right AFTER chassis.cancel(); and after step 6b's release mode
// (so after drive()), before step 7's perTick, each binding's two buttons are read — only while
// connected, a disconnected
// controller being a release — and the binding ticks, so a press read at tick k is commanded in
// tick k. These three are placed by position, not numbered: they touch only the mechanisms, never
// the drive. The overload without a span is exactly this one with an empty span: one body.
//
// TOGGLE BINDINGS (a second span; T3's addendum B2 — a pneumatic claw on one button) ride the
// same three places with ONE difference, made where it happens: BEFORE tick 1 every toggle is
// enter()'d (its stated initial level, once in its life); after drive() each toggle's button is
// read only while connected and the toggle ticks, after the hold bindings; and on the disable the
// hold bindings are made safe and the toggles are left UNTOUCHED — a claw keeps its level through
// a disable, because it must not drop what it holds. `validBindings()` checks both spans as one
// set. The two-span overload is the body; the one-span and no-span overloads forward to it.
//
// A CLAIM LEFT OVER AT ENTRY IS RELEASED (the team lead's ruling, 2026-09-26; T3 Part 2). PROS
// deletes the autonomous task when the field ends it, so an operation running then keeps its
// mechanism CLAIMED into driver control — and a binding never commands a claimed mechanism, so the
// driver would be locked out of it for the whole period. At entry, after the set check and before
// the entry safe state and enter(), every binding's and toggle's releaseLeftoverClaim() releases a
// claim still on its mechanism (never calling the claimant's cancel(): it may have lived on the
// deleted task's stack), and the loop SAYS so: one Warn line per released claim, tag `TEL`, through
// the chassis's own sink (`chassis.deps().ctx->telemetry()`), and the binding remembers it for a
// panel (`releasedLeftoverClaim()`). The mechanism is then made safe at entry like any other. A
// claim taken DURING the loop is not touched: the binding still defers to it. The cost of the
// ruling is stated in mechanism_binding.hpp (an operation started before entry loses its claim).
//
// No buffer, no queue, no "apply next tick": a stick step read at tick k is in the commanded
// volts of tick k (pinned through the host plant). PERCEIVED latency is not claimed — real
// input-to-motion latency is PROS call cost plus loop rate, and neither is measured.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Host-proven on the A2 plant only. Two callers: robot two's driver file
// (src/team/robot_two_driver.hpp, T4 Part 1a), with the default DriverProfile (ArcadeBody) and
// field-centric OFF — "shulib Drive" runs that file (T4 Part 3, src/programs/drive_program.cpp),
// and so does the competition program's opcontrol() (chunk COMP-b) — and the invented X-drive's
// program (src/programs/xdrive_program.cpp, since G1 Part 2c), which refuses at boot on every
// robot that exists. ("shulib Teleop" called it by its own call from G1 Part 2b until T4 Part 3
// retired it.) Every one of them gets the drive protections from DriverLoopConfig::protection's
// defaults, since none names the field. No version has run on
// hardware, so nothing in the feel layer it runs has been felt by a driver (HA-112), and no
// protection has cut or marked a real motor (HA-175, HA-176).
//
// PROS-free, sim-free, allocation-free.

#include <cmath>
#include <cstdio>
#include <span>
#include <type_traits>
#include <utility>

#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::teleop {

/// What the loop needs that neither the chassis nor the controller can supply.
///
/// State the first TWO, with designated initializers — `{.tickPeriod = …, .rezeroButton = …}`.
/// The period's default is deliberately not a working configuration: a zero period trips
/// `runDriverControl`'s precondition before anything moves, so a config nobody wrote can never
/// run. The button's default is NOT a refusal by itself: a config that leaves `rezeroButton`
/// unset quietly gets `L1` — a shoulder button, which robot two's claw pivot 7 takes. What stops
/// that collision running is `validBindings()` (chunk T3): a binding on the re-zero button is
/// refused before the first tick, by `runDriverControl`'s precondition. A program should still
/// name its button, and robot two's driver file does (its kLoop names Y). The third,
/// `releaseBrakeMode`, has a working default (Coast, the team lead's ruling). The fourth, `protection`, is APPENDED LAST so every
/// existing designated initializer still compiles, and its defaults are the old "shulib Drive"'s
/// numbers: a program that names no `protection` is protected (chunk T4 Part 2).
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
    /// The drive protections' numbers (chunk T4 Part 2; teleop/drive_protection.hpp): the
    /// frozen-reading rule's read count, the over-current limit and window, the cut's length, the
    /// log bound, and the one switch that turns the over-current CUT into a warning. The defaults
    /// are the old "shulib Drive"'s — every one of them unmeasured (HA-175, HA-176) — and apply
    /// to every program that leaves this field out. Checked before the first tick
    /// (validProtectionConfig).
    DriveProtectionConfig protection{};
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
/// one heading sample, the re-zero edge, shape, the drive protections' check before `drive`,
/// `drive(Body)`, their check after it, the release mode, the hold bindings, the toggles,
/// `perTick`, `pace`).
///
/// * `chassis` — the facade to drive; its `motionConfig()` supplies the speed budgets
///   (`maxLinearSpeed`, `maxAngularSpeed` — HA-50, provisional).
/// * `controller` — the driver's controller (VEX U's partner is a second call with a second
///   shaper).
/// * `pacer` — the tick boundary: on a brain the provider's `ProsTickPacer`; in a host test the
///   plant-stepping pacer.
/// * `shaper` — the driver's feel (`DriveShaper`, one per driver, held across calls so its
///   field-centric zero and latch survive a disable).
/// * `cfg` — the tick period, the re-zero button, the release brake mode (applied at entry
///   and after every `drive()`) and the drive protections' numbers. PRECONDITIONS, checked before
///   the first tick and before anything is commanded: `cfg.tickPeriod` finite and > 0, and
///   `validProtectionConfig(cfg.protection, chassis.deps().motorGroups)` — together, every
///   precondition of the DriveProtection the loop builds, so nothing is refused after the entry
///   has commanded something.
/// * `bindings` — the driver's mechanism buttons (chunk T3), each ticked after `drive()` with its
///   two buttons read only while connected. PRECONDITION, checked before the first tick:
///   `validBindings(bindings, cfg.rezeroButton)` — no null, no button used twice, none equal to
///   the re-zero button, no mechanism bound twice. A claim still on a bound mechanism at entry is
///   RELEASED (`releaseLeftoverClaim()`) and logged — one Warn line each, tag `TEL`, on the
///   chassis's sink (the team lead's ruling, 2026-09-26). Every binding is `reset()` and
///   `makeSafe()`'d before the first tick, and `makeSafe()`'d again on the disable. BORROWED for the call; for a
///   PROS program the bindings (and their mechanisms and motors) live in static storage, because a
///   field disable deletes the task. The overload without this argument passes an empty span.
/// * `toggles` — the driver's TOGGLE buttons (addendum B2: one button flipping one pneumatic
///   mechanism per press), ticked after the hold bindings with their buttons read only while
///   connected. A claim still on a toggle's mechanism at entry is released and logged, as for
///   `bindings`, BEFORE its `enter()`. Every toggle is `enter()`'d before the first tick (its
///   initial level, once in its life) and is NOT touched on the disable: the claw keeps its level.
///   Checked with `bindings` as
///   ONE set by the same precondition, `validBindings(bindings, toggles, cfg.rezeroButton)`.
///   BORROWED, static storage on a brain, exactly as `bindings`. The overloads without this
///   argument pass an empty span.
/// * `disabled` — `bool()`: polled FIRST every tick; true ⇒ `chassis.cancel()` (0 V + Brake),
///   every hold binding's `makeSafe()` — no toggle is touched — and return. The only way out.
/// * `heading` — `HeadingSample()`: called once per tick, after the sticks, before shaping.
/// * `perTick` — `void(const math::ChassisSpeeds& command, bool connected,
///   const DriveShaperState& state)`: called after `drive()`, before `pace()` — the panel's hook.
///   One that ALSO takes `const DriveProtectionState&` as a fourth argument is handed the drive
///   protections' state (detected at compile time; the three-argument form is unchanged).
///   `command` is always the driver's request, also while a cut sends the drive zero.
///
/// Nothing is read from the controller while it is disconnected; the command is then exactly
/// zero whatever the axes claim (HA-103).
///
/// The DRIVE PROTECTIONS run inside it for every caller (the file header; drive_protection.hpp):
/// a drive member whose readings freeze is kept out of the fight monitor and marked ABSENT after
/// 250 ms; more than two dead in total REFUSES the drive for the rest of the run; a fight or a
/// member over 2.4 A for 250 ms while driven cuts the drive to 0 V for 1 s, then re-arms.
template <typename IsDisabled, typename HeadingSource, typename PerTick>
void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller,
                      motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg,
                      std::span<MechanismBinding* const> bindings,
                      std::span<ToggleBinding* const> toggles, IsDisabled&& disabled,
                      HeadingSource&& heading, PerTick&& perTick) {
    SHULIB_PRECONDITION(std::isfinite(cfg.tickPeriod.value()) && cfg.tickPeriod.value() > 0.0,
                        "runDriverControl: DriverLoopConfig::tickPeriod must be finite and > 0 "
                        "(state it: the pacer's period, 10 ms on a V5)");
    char bindingWhy[192] = "";
    SHULIB_PRECONDITION(
        validBindings(bindings, toggles, cfg.rezeroButton, bindingWhy, sizeof bindingWhy),
        bindingWhy);
    // The protections' numbers, checked with the rest — BEFORE the claim release, the entry safe
    // state, the toggles' enter and the release mode below, each of which commands something: a
    // bad config commands nothing at all (T4 Part 2).
    char protectionWhy[192] = "";
    SHULIB_PRECONDITION(validProtectionConfig(cfg.protection, chassis.deps().motorGroups,
                                              protectionWhy, sizeof protectionWhy),
                        protectionWhy);
    // A claim still on a bound mechanism at entry is RELEASED, and said (the team lead's ruling,
    // 2026-09-26): a claim left by a deleted autonomous task must never lock the driver out. After
    // the set check (a refused set touches nothing), before the entry safe state below.
    const auto sayReleased = [&chassis](const char* mechanismName) {
        char line[192];
        std::snprintf(line, sizeof line,
                      "driver control: RELEASED a claim left on mechanism '%s' at entry (an "
                      "operation that outlived its task) -- its buttons work from the first tick",
                      mechanismName);
        chassis.deps().ctx->telemetry().log(hal::LogLevel::Warn, "TEL", line);
    };
    for (MechanismBinding* binding : bindings) {
        if (binding->releaseLeftoverClaim()) {
            sayReleased(binding->mechanism().name());
        }
    }
    for (ToggleBinding* toggle : toggles) {
        if (toggle->releaseLeftoverClaim()) {
            sayReleased(toggle->mechanism().name());
        }
    }
    // Bindings, before tick 1: every one starts from nothing, in its declared safe state.
    for (MechanismBinding* binding : bindings) {
        binding->reset();
        binding->makeSafe();
    }
    // …and every toggle is entered: its initial level, once in its life; its button not yet seen.
    for (ToggleBinding* toggle : toggles) {
        toggle->enter();
    }
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
    // The drive protections for this entry (T4 Part 2): judged from the groups as they stand, so a
    // drive left with more than two dead by an earlier entry is REFUSED from tick 1. Its destructor
    // clears every group's exclusion mask on every way out of this function.
    DriveProtection protection{chassis, cfg.protection, cfg.tickPeriod};
    protection.enter();
    const math::ChassisSpeeds zeroCommand{};
    while (true) {
        // 1. A field disable: safe the drive, then leave.
        if (disabled()) {
            chassis.cancel();
            for (MechanismBinding* binding : bindings) {
                binding->makeSafe();  // …and every mechanism, in the same tick
            }
            // No toggle is touched: a pneumatic claw keeps its level through a disable (whether
            // the BRAIN keeps a 3-wire output's level while disabled: A4 register HA-157).
            protection.onDisable();  // the summary, after the drive is already safe
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
        // The protections wrap drive(): the frozen-reading rule BEFORE it (the group monitors are
        // judged inside it), a ZERO command while a cut or a refusal is in force, the cuts AFTER.
        protection.beforeDrive();
        chassis.drive(protection.commandAllowed() ? command : zeroCommand, math::Frame::Body);
        protection.afterDrive();
        // 6b. The release mode again, in this same tick: undoes a pre-emption's Brake (DEFECTS2).
        //     Drive motors only (driveMotors()) — a mechanism's motor is its binding's alone.
        applyReleaseMode();
        // Bindings, after drive() and the release mode, before the panel: two buttons each, read
        // only while connected.
        for (MechanismBinding* binding : bindings) {
            binding->tick(sticks.connected,
                          sticks.connected && controller.pressed(binding->forwardButton()),
                          sticks.connected && controller.pressed(binding->reverseButton()));
        }
        for (ToggleBinding* toggle : toggles) {  // …then each toggle: one button, same rule
            toggle->tick(sticks.connected,
                         sticks.connected && controller.pressed(toggle->button()));
        }
        // 7–8. The caller's panel, then the tick boundary. A panel that takes a fourth argument is
        //      handed the protections' state; the three-argument form is unchanged.
        if constexpr (std::is_invocable_v<PerTick&, const math::ChassisSpeeds&, bool,
                                          const DriveShaperState&, const DriveProtectionState&>) {
            perTick(command, sticks.connected, shaper.state(), protection.state());
        } else {
            perTick(command, sticks.connected, shaper.state());
        }
        pacer.pace();
    }
}

/// `runDriverControl` with hold bindings and NO toggles — the chunk-T3 signature, forwarding to
/// the overload above with an empty toggle span, so there is one loop body and every rule above
/// holds unchanged.
template <typename IsDisabled, typename HeadingSource, typename PerTick>
void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller,
                      motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg,
                      std::span<MechanismBinding* const> bindings, IsDisabled&& disabled,
                      HeadingSource&& heading, PerTick&& perTick) {
    runDriverControl(chassis, controller, pacer, shaper, cfg, bindings,
                     std::span<ToggleBinding* const>{}, std::forward<IsDisabled>(disabled),
                     std::forward<HeadingSource>(heading), std::forward<PerTick>(perTick));
}

/// `runDriverControl` with NO mechanism bindings — the 2a signature, forwarding to the overload
/// above with an empty span, so there is one loop body and every rule above holds unchanged.
template <typename IsDisabled, typename HeadingSource, typename PerTick>
void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller,
                      motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg,
                      IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick) {
    runDriverControl(chassis, controller, pacer, shaper, cfg,
                     std::span<MechanismBinding* const>{}, std::forward<IsDisabled>(disabled),
                     std::forward<HeadingSource>(heading), std::forward<PerTick>(perTick));
}

}  // namespace shulib::teleop
