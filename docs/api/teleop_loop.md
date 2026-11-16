<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/teleop_loop.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `teleop_loop.hpp`

runDriverControl — the library's driver-control loop.

This header declares **2** types (6 members), **4** free functions, **3** constants, and **3** type aliass.

Extracted from [`include/shulib/teleop/teleop_loop.hpp`](../../include/shulib/teleop/teleop_loop.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriverLoopConfig`](#struct-driverloopconfig)
  - [`tickPeriod`](#driverloopconfig-tickperiod)
  - [`rezeroButton`](#driverloopconfig-rezerobutton)
  - [`releaseBrakeMode`](#driverloopconfig-releasebrakemode)
  - [`protection`](#driverloopconfig-protection)
- [`struct HeadingSample`](#struct-headingsample)
  - [`radians`](#headingsample-radians)
  - [`trusted`](#headingsample-trusted)
- [`runDriverControl`](#rundrivercontrol) — *free function*
- [`runDriverControl (overload 2)`](#rundrivercontrol-2) — *free function*
- [`runDriverControl (overload 3)`](#rundrivercontrol-3) — *free function*
- [`runDriverControl (overload 4)`](#rundrivercontrol-4) — *free function*
- [`Button`](#button) — *type alias*
- [`DriverProfile`](#driverprofile) — *type alias*
- [`DriverLoopConfig`](#driverloopconfig) — *type alias*
- [`ArcadeBody`](#arcadebody) — *constant*
- [`ArcadeField`](#arcadefield) — *constant*
- [`TankSticks`](#tanksticks) — *constant*

<a id="struct-driverloopconfig"></a>

## `struct DriverLoopConfig`

```cpp
struct DriverLoopConfig
```

What the loop needs that neither the chassis nor the controller can supply.  State the first TWO, with designated initializers — `{.tickPeriod = …, .rezeroButton = …}`. The period's default is deliberately not a working configuration: a zero period trips `runDriverControl`'s precondition before anything moves, so a config nobody wrote can never run. The button's default is NOT a refusal by itself: a config that leaves `rezeroButton` unset quietly gets `L1` — a shoulder button, which robot two's claw pivot 7 takes. What stops that collision running is `runDriverControl`'s precondition, before the first tick: a binding on the re-zero button is refused by `validBindings()` (chunk T3), a mechanism service's control on it by `mechanism::DriverInputs::avoids()` (chunk MECH-A4). A program should still name its button, and robot two's driver file does (its kLoop names Y — and since MECH-A5 a driver file whose control takes its re-zero button does not build). The third, `releaseBrakeMode`, has a working default (Coast, the team lead's ruling). The fourth, `protection`, is APPENDED LAST so every existing designated initializer still compiles, and its defaults are the old "shulib Drive"'s numbers: a program that names no `protection` is protected (chunk T4 Part 2).

*struct, declared at [`include/shulib/teleop/teleop_loop.hpp:196`](../../include/shulib/teleop/teleop_loop.hpp#L196).*

<a id="driverloopconfig-tickperiod"></a>

### `DriverLoopConfig::tickPeriod`

```cpp
units::Time tickPeriod{0.0}
```

How long one tick is — the pacer's period (on a brain, `ProsTickPacer::kTickMs` = 10 ms, HA-32). Handed to the shaper as its dt, where only the slew limiter reads it. Must be finite and > 0. `motion::ITickPacer` has no period of its own, and cannot gain one (M3), which is why the program states it here.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:201`](../../include/shulib/teleop/teleop_loop.hpp#L201).*

<a id="driverloopconfig-rezerobutton"></a>

### `DriverLoopConfig::rezeroButton`

```cpp
hal::ControllerButton rezeroButton = hal::ControllerButton::L1
```

The driver's field-centric re-zero button (`DriveShaper::rezero`). Harmless in a body mode, where a press only pre-arms the zero for a later switch to `ArcadeField`. Which button is the PROGRAM's choice, so state it.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:205`](../../include/shulib/teleop/teleop_loop.hpp#L205).*

<a id="driverloopconfig-releasebrakemode"></a>

### `DriverLoopConfig::releaseBrakeMode`

```cpp
hal::BrakeMode releaseBrakeMode = hal::BrakeMode::Coast
```

What the drive does when the sticks are released (chunk DEFECTS2): the brake mode the loop puts on every drive motor at entry and again after every `drive()`, so a library stop inside the loop — `drive()` pre-empting a motion cancels it into Brake — never outlives its tick. Default `Coast`, the team lead's ruling (2026-09-26): the drive rolls free on release, the feel shulib Teleop was driven with on 2026-09-25. `Brake` or `Hold` make a release stop the drive; since DEFECTS2 the V5 adapter sends them to the motor as PROS brake() at a zero command (`hal/pros/motor.hpp`, THE STOP — not yet seen on a robot). The disable path is NOT governed by this: a disable still cancels into Brake.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:214`](../../include/shulib/teleop/teleop_loop.hpp#L214).*

<a id="driverloopconfig-protection"></a>

### `DriverLoopConfig::protection`

```cpp
DriveProtectionConfig protection{}
```

The drive protections' numbers (chunk T4 Part 2; teleop/drive_protection.hpp): the frozen-reading rule's read count, the over-current limit and window, the cut's length, the log bound, and the one switch that turns the over-current CUT into a warning. The defaults are the old "shulib Drive"'s — every one of them unmeasured (HA-175, HA-176) — and apply to every program that leaves this field out. Checked before the first tick (validProtectionConfig).

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:221`](../../include/shulib/teleop/teleop_loop.hpp#L221).*

<a id="struct-headingsample"></a>

## `struct HeadingSample`

```cpp
struct HeadingSample
```

One tick's heading, as the loop hands it to the shaper: the robot's estimated heading and the caller's verdict on it. What a `runDriverControl` heading source returns.

*struct, declared at [`include/shulib/teleop/teleop_loop.hpp:226`](../../include/shulib/teleop/teleop_loop.hpp#L226).*

<a id="headingsample-radians"></a>

### `HeadingSample::radians`

```cpp
double radians = 0.0
```

The estimated heading, radians, CCW-positive, in the same frame the driver's zero is captured in. Ignored unless the profile's mode is `ArcadeField`.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:229`](../../include/shulib/teleop/teleop_loop.hpp#L229).*

<a id="headingsample-trusted"></a>

### `HeadingSample::trusted`

```cpp
bool trusted = false
```

True only when the estimate is live and the heading is believable (for a robot::Robot: the Localizer is past `Quality::Uninitialized` and the IMU reports ready). False makes field-centric fall back to robot-centric and LATCH until the driver re-zeroes.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:233`](../../include/shulib/teleop/teleop_loop.hpp#L233).*

<a id="rundrivercontrol"></a>

## `runDriverControl`

```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick> void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller, motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg, std::span<MechanismBinding* const> bindings, std::span<ToggleBinding* const> toggles, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)
```

Run driver control until `disabled()` says stop: sticks → `shaper` → budgets → the chassis, once per tick, in the order the file header lists (disabled check, sticks while connected, one heading sample, the re-zero edge, shape, the drive protections' check before `drive`, `drive(Body)`, their check after it, the release mode, the hold bindings, the toggles, `perTick`, `pace`).  * `chassis` — the facade to drive; its `motionConfig()` supplies the speed budgets (`maxLinearSpeed`, `maxAngularSpeed` — HA-50, provisional). * `controller` — the driver's controller (VEX U's partner is a second call with a second shaper). * `pacer` — the tick boundary: on a brain the provider's `ProsTickPacer`; in a host test the plant-stepping pacer. * `shaper` — the driver's feel (`DriveShaper`, one per driver, held across calls so its field-centric zero and latch survive a disable). * `cfg` — the tick period, the re-zero button, the release brake mode (applied at entry and after every `drive()`) and the drive protections' numbers. PRECONDITIONS, checked before the first tick and before anything is commanded: `cfg.tickPeriod` finite and > 0, and `validProtectionConfig(cfg.protection, chassis.deps().motorGroups)` — together, every precondition of the DriveProtection the loop builds, so nothing is refused after the entry has commanded something. * `bindings` — the driver's mechanism buttons (chunk T3), each ticked after `drive()` with its two buttons read only while connected. PRECONDITION, checked before the first tick: `validBindings(bindings, cfg.rezeroButton)` — no null, no button used twice, none equal to the re-zero button, no mechanism bound twice. A claim still on a bound mechanism at entry is RELEASED (`releaseLeftoverClaim()`) and logged — one Warn line each, tag `TEL`, on the chassis's sink (the team lead's ruling, 2026-09-26). Every binding is `reset()` and `makeSafe()`'d before the first tick, and `makeSafe()`'d again on the disable. BORROWED for the call; for a PROS program the bindings (and their mechanisms and motors) live in static storage, because a field disable deletes the task. The overload without this argument passes an empty span. * `toggles` — the driver's TOGGLE buttons (addendum B2: one button flipping one pneumatic mechanism per press), ticked after the hold bindings with their buttons read only while connected. A claim still on a toggle's mechanism at entry is released and logged, as for `bindings`, BEFORE its `enter()`. Every toggle is `enter()`'d before the first tick (its initial level, once in its life) and is NOT touched on the disable: the claw keeps its level. Checked with `bindings` as ONE set by the same precondition, `validBindings(bindings, toggles, cfg.rezeroButton)`. BORROWED, static storage on a brain, exactly as `bindings`. The overloads without this argument pass an empty span. * `disabled` — `bool()`: polled FIRST every tick; true ⇒ `chassis.cancel()` (0 V + Brake), every hold binding's `makeSafe()` — no toggle is touched — and return. The only way out. * `heading` — `HeadingSample()`: called once per tick, after the sticks, before shaping. * `perTick` — `void(const math::ChassisSpeeds& command, bool connected, const DriveShaperState& state)`: called after `drive()`, before `pace()` — the panel's hook. One that ALSO takes `const DriveProtectionState&` as a fourth argument is handed the drive protections' state (detected at compile time; the three-argument form is unchanged). `command` is always the driver's request, also while a cut sends the drive zero.  Nothing is read from the controller while it is disconnected; the command is then exactly zero whatever the axes claim (HA-103).  The DRIVE PROTECTIONS run inside it for every caller (the file header; drive_protection.hpp): a drive member whose readings freeze is kept out of the fight monitor and marked ABSENT after 250 ms; more than two dead in total REFUSES the drive for the rest of the run; a fight or a member over 2.4 A for 250 ms while driven cuts the drive to 0 V for 1 s, then re-arms.  SUPERSEDED by the mechanism service (the file header, N4): this T3 overload is for a program with NO mechanism service; a program with one drives its mechanisms through the service overload below.

*free function, declared at [`include/shulib/teleop/teleop_loop.hpp:465`](../../include/shulib/teleop/teleop_loop.hpp#L465).*

<a id="rundrivercontrol-2"></a>

## `runDriverControl (overload 2)`

```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick> void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller, motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg, mechanism::MechanismService& mechanisms, mechanism::DriverInputs& inputs, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)
```

`runDriverControl` with the robot's MECHANISM SERVICE (chunk MECH-A4, API 2.5; the file header's THE MECHANISM SERVICE): the driver file's controls (`inputs`, bound to `mechanisms`) run the robot's declared mechanisms through the service, which ticks EXACTLY ONCE per tick at step 6½ — `pacer` may be the raw tick pacer or a chain that contains the service. Everything the overload above says of the drive — the order of a tick, the protections, the release mode, the disable — holds unchanged; there are no T3 bindings. PRECONDITIONS, checked before anything is commanded: `inputs` bound to `mechanisms` (at boot); no control on `cfg.rezeroButton`; and every precondition of the overload above. BORROWED for the call; a PROS program keeps the service and the inputs in static storage.

*free function, declared at [`include/shulib/teleop/teleop_loop.hpp:485`](../../include/shulib/teleop/teleop_loop.hpp#L485).*

<a id="rundrivercontrol-3"></a>

## `runDriverControl (overload 3)`

```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick> void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller, motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg, std::span<MechanismBinding* const> bindings, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)
```

`runDriverControl` with hold bindings and NO toggles — the chunk-T3 signature, forwarding to the two-span overload with an empty toggle span, so there is one loop body and every rule above holds unchanged. SUPERSEDED by the mechanism service, like the two-span overload (N4).

*free function, declared at [`include/shulib/teleop/teleop_loop.hpp:498`](../../include/shulib/teleop/teleop_loop.hpp#L498).*

<a id="rundrivercontrol-4"></a>

## `runDriverControl (overload 4)`

```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick> void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller, motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)
```

`runDriverControl` with NO mechanism bindings — the 2a signature, forwarding to the hold-binding overload with an empty span, so there is one loop body and every rule above holds unchanged.

*free function, declared at [`include/shulib/teleop/teleop_loop.hpp:510`](../../include/shulib/teleop/teleop_loop.hpp#L510).*

<a id="button"></a>

## `Button`

```cpp
using Button = hal::ControllerButton
```

The controller's buttons: `Button::R1`.

*type alias, declared at [`include/shulib/teleop/teleop_loop.hpp:527`](../../include/shulib/teleop/teleop_loop.hpp#L527).*

<a id="driverprofile"></a>

## `DriverProfile`

```cpp
using teleop::DriverProfile
```

The driver's feel (teleop/driver_control.hpp).

*type alias, declared at [`include/shulib/teleop/teleop_loop.hpp:529`](../../include/shulib/teleop/teleop_loop.hpp#L529).*

<a id="driverloopconfig"></a>

## `DriverLoopConfig`

```cpp
using teleop::DriverLoopConfig
```

The loop's numbers: the re-zero button, the release brake mode, the drive protections.

*type alias, declared at [`include/shulib/teleop/teleop_loop.hpp:531`](../../include/shulib/teleop/teleop_loop.hpp#L531).*

<a id="arcadebody"></a>

## `ArcadeBody`

```cpp
inline constexpr teleop::DriveMode ArcadeBody = teleop::DriveMode::ArcadeBody
```

Arcade, robot-centric: the left stick drives forward and back, the right turns.

*constant, declared at [`include/shulib/teleop/teleop_loop.hpp:533`](../../include/shulib/teleop/teleop_loop.hpp#L533).*

<a id="arcadefield"></a>

## `ArcadeField`

```cpp
inline constexpr teleop::DriveMode ArcadeField = teleop::DriveMode::ArcadeField
```

Arcade, field-centric ("away from me" is away, whichever way the robot faces).

*constant, declared at [`include/shulib/teleop/teleop_loop.hpp:535`](../../include/shulib/teleop/teleop_loop.hpp#L535).*

<a id="tanksticks"></a>

## `TankSticks`

```cpp
inline constexpr teleop::DriveMode TankSticks = teleop::DriveMode::TankSticks
```

Tank sticks: the left stick drives the left side, the right stick the right.

*constant, declared at [`include/shulib/teleop/teleop_loop.hpp:537`](../../include/shulib/teleop/teleop_loop.hpp#L537).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 153 lines, click to expand</summary>

```text

 runDriverControl — the library's driver-control loop (chunk G1 Part 2a, 2026-09-24; T2's
 integration contract, T2 brief §7).

 ── Why it is library code ──────────────────────────────────────────────────────────
 Until this file, the only teleop loop in the tree was `runTeleopLoop`, a 22-line function in
 src/main.cpp's anonymous namespace. It could not be host-tested (it called PROS's
 competition_is_disabled() directly), it read three of the four stick axes (not RightY, which
 T2's tank-stick mode needs), and it mapped the sticks through the raw R1a mapping rather than
 T2's feel layer. This is that loop, rebuilt on teleop::DriveShaper, with its three PROS
 dependencies INJECTED — the disabled predicate, the tick period and the heading source — so
 teleop/ stays PROS-free and the whole loop runs on the A2 host plant under a FakeController
 (the G1 Part 2 brief's ruling M3). Rejected: leaving the loop in src/ (untestable); adding a
 period to motion::ITickPacer (it is in the frozen Chassis constructor's signature, and a new
 pure virtual would break every implementer) — so the period is a config value instead.

 ── ONE TICK, IN THIS ORDER ─────────────────────────────────────────────────────────
   1. disabled()      true ⇒ chassis.cancel() — 0 V + Brake, the facade's panic stop — and
                      RETURN. PROS deletes the opcontrol task on a field disable, and a deleted
                      task runs no destructor, so this is the one path on which the drive is
                      put into its safe state before VEXos cuts the motors itself (R3b Part 0b's
                      finding, carried over from runTeleopLoop). PROS calls opcontrol() again
                      when driver control resumes; the caller simply calls this again.
                      Whether that stop — a PROS brake() since DEFECTS2 — lands in time and
                      outlasts the cut is A4 register HA-145 (unknown).
   2. the sticks      isConnected() first; ALL FOUR axes, and the re-zero button, are read
                      ONLY while connected. A disconnected V5 controller reads 0 on every
                      channel (HA-103), so the connected flag is the only thing that separates
                      "driver unplugged" from "sticks centred", and nothing is read that could
                      pretend otherwise.
   3. heading()       sampled ONCE per tick, used for both the re-zero and the shaping. It is
                      called BEFORE this tick's chassis.drive(), which is where the Localizer
                      updates — so a source that reads the fused estimate sees last tick's
                      heading (10 ms old); one that reads the IMU directly does not.
   4. re-zero         the loop's OWN hal::ButtonEdge on cfg.rezeroButton (a per-consumer edge,
                      HA-104) calls shaper.rezero(heading) ONCE per press, and BEFORE shaping,
                      so "the way the robot faces now is away from me" governs this very tick.
   5. shape           shaper.shape(sticks, cfg.tickPeriod, heading.radians, heading.trusted) —
                      mode → shaping → slew → rotation (driver_control.hpp).
   6. scale + drive   toChassisSpeeds(request, maxLinearSpeed, maxAngularSpeed) with the
                      budgets from chassis.motionConfig(), then chassis.drive(…, Frame::Body)
                      — ALWAYS Body, field-centric included: the rotation already happened
                      inside shape(). The DRIVE PROTECTIONS (chunk T4 Part 2, below) wrap it:
                      their beforeDrive() runs just before it, and while a cut or a refusal is
                      in force drive() is handed a ZERO command instead of this one; their
                      afterDrive() runs just after it, and may CUT in this same tick.
   6b. release mode   cfg.releaseBrakeMode (default Coast) on every drive motor — also once
                      at ENTRY, before the first tick (chunk DEFECTS2). drive() pre-empting a
                      motion cancels it into Brake, and since DEFECTS2 a Brake at 0 V goes to
                      a V5 as PROS brake(); re-asserting after EVERY drive() — not only after a
                      pre-emption — keeps the invariant independent of which library path
                      set another mode, for one set_brake_mode per motor per tick (believed
                      harmless to a running motor: A4 register HA-146).
   6½. mechanisms     (the MECHANISM SERVICE overload only — chunk MECH-A4, below) the driver
                      file's controls read their buttons and command the service
                      (mechanism::DriverInputs::tick, only while connected), THEN the service
                      ticks every mechanism ONCE (tickFromDriverLoop) — after the drive and the
                      T3 bindings' place, BEFORE perTick, so a press read at tick k is commanded
                      and ticked in tick k and a panel sees this tick's command.
   7. perTick(command, connected, shaper.state())  the caller's panel — or, when the caller's
                      perTick takes a FOURTH argument, perTick(…, protection.state()), the
                      drive protections' state (DriveProtectionState). The three-argument form
                      compiles and sees exactly what it saw before.
   8. pacer.pace()    the tick boundary.

 THE DRIVE PROTECTIONS (chunk T4 Part 2, 2026-09-29; the team lead's ruling R4): the old
 "shulib Drive"'s mid-match absence rule and its two re-arming cuts, applied HERE, for every
 program that drives through this loop — teleop/drive_protection.hpp has the rules. After the
 precondition checks (the protections' numbers are one of them: validProtectionConfig, so a bad
 DriverLoopConfig::protection commands NOTHING), ONE DriveProtection is built and enter()'d —
 it REFUSES from tick 1 a drive whose ABSENT marks, left in the groups by an earlier entry, are
 more than two in total. Each tick: beforeDrive() → drive() (the stick, or zero while cut or
 refused) → afterDrive() → 6b and the rest as above. On the disable, after cancel(), its one
 summary line per group. Its destructor clears every group's exclusion mask on every way out of
 this function — the disable return and a throw — so motion code after driver control sees the
 groups as it expects. The way out it cannot cover is step 1's: when PROS deletes the task
 before the loop polls the disable, no destructor runs, and the mask holds the last tick's
 FROZEN members until the next entry's enter() clears it (drive_protection.hpp — stated, not
 solved). A cut never delays the disable: step 1 still runs first on every tick.

 With MECHANISM BINDINGS (the overload taking a span; chunk T3, teleop/mechanism_binding.hpp),
 three things are added and nothing above moves: BEFORE tick 1 every binding is reset() and
 makeSafe()'d (after the same precondition checks, and `validBindings()` is one of them — and,
 between the two, a claim still on a bound mechanism is RELEASED and said: the team lead's ruling
 of 2026-09-26, T3 Part 2, below); in step
 1 every binding is makeSafe()'d right AFTER chassis.cancel(); and after step 6b's release mode
 (so after drive()), before step 7's perTick, each binding's two buttons are read — only while
 connected, a disconnected
 controller being a release — and the binding ticks, so a press read at tick k is commanded in
 tick k. These three are placed by position, not numbered: they touch only the mechanisms, never
 the drive. The overload without a span is exactly this one with an empty span: one body.

 TOGGLE BINDINGS (a second span; T3's addendum B2 — a pneumatic claw on one button) ride the
 same three places with ONE difference, made where it happens: BEFORE tick 1 every toggle is
 enter()'d (its stated initial level, once in its life); after drive() each toggle's button is
 read only while connected and the toggle ticks, after the hold bindings; and on the disable the
 hold bindings are made safe and the toggles are left UNTOUCHED — a claw keeps its level through
 a disable, because it must not drop what it holds. `validBindings()` checks both spans as one
 set. The two-span overload is the body; the one-span and no-span overloads forward to it.

 A CLAIM LEFT OVER AT ENTRY IS RELEASED (the team lead's ruling, 2026-09-26; T3 Part 2). PROS
 deletes the autonomous task when the field ends it, so an operation running then keeps its
 mechanism CLAIMED into driver control — and a binding never commands a claimed mechanism, so the
 driver would be locked out of it for the whole period. At entry, after the set check and before
 the entry safe state and enter(), every binding's and toggle's releaseLeftoverClaim() releases a
 claim still on its mechanism (never calling the claimant's cancel(): it may have lived on the
 deleted task's stack), and the loop SAYS so: one Warn line per released claim, tag `TEL`, through
 the chassis's own sink (`chassis.deps().ctx->telemetry()`), and the binding remembers it for a
 panel (`releasedLeftoverClaim()`). The mechanism is then made safe at entry like any other. A
 claim taken DURING the loop is not touched: the binding still defers to it. The cost of the
 ruling is stated in mechanism_binding.hpp (an operation started before entry loses its claim).

 THE MECHANISM SERVICE (chunk MECH-A4, 2026-10-22; the MECH brief's §1.10). The overload taking a
 mechanism::MechanismService and its bound mechanism::DriverInputs runs a robot's DECLARED mechanisms
 — the driver file's whileHeld / toggle / preset / homing controls (mechanism/driver_inputs.hpp) —
 through the one per-tick service that also runs them in autons. Nothing above moves (every public
 overload runs ONE body, detail::driverLoop); three places are added: at ENTRY, after the checks,
 the service's enterDriverControl() (a claim another owner left released WITHOUT its cancel() and
 said by name; every running step CANCELLED; motor mechanisms safe; a pneumatic sent its START
 position on the program's FIRST entry only) and then the inputs' enter() (a press must be SEEN;
 homeAtDriverStart homes what nothing homed); on the DISABLE, after chassis.cancel(), the service's
 leaveDriverControl() (motor mechanisms safe, claims released; a pneumatic KEEPS its level — HA-157);
 and step 6½ each tick. The service ticks EXACTLY ONCE per tick whichever pacer the loop is handed —
 the raw tick pacer, the service itself, or a guard around it (a pace() right after
 tickFromDriverLoop() does not tick it again). TWO PRECONDITIONS, checked first: the service and
 its BOUND inputs come together and replace the T3 bindings — one driver path per mechanism — and
 no control takes the re-zero button.

 T3'S BINDINGS ARE SUPERSEDED, NOT REMOVED (the brief's N4, ruled (a)): MechanismBinding and
 ToggleBinding and the overloads taking them stay, documented "superseded by the mechanism service;
 for a program with no service". A program with a mechanism service drives its mechanisms through
 the service overload; one with none keeps the T3 path exactly as it was.

 No buffer, no queue, no "apply next tick": a stick step read at tick k is in the commanded
 volts of tick k (pinned through the host plant). PERCEIVED latency is not claimed — real
 input-to-motion latency is PROS call cost plus loop rate, and neither is measured.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-proven on the A2 plant only. Two callers: robot two's driver file
 (src/team/robot_two_driver.hpp, T4 Part 1a), with the default DriverProfile (ArcadeBody) and
 field-centric OFF, through the call every driver file shares — src/common/driver_file.hpp's
 DriverFile::run(), the mechanism-service overload, since MECH-A5 (the driver file made the call
 itself, over T3's bindings, until then) — "shulib Drive" runs that file (T4 Part 3,
 src/programs/drive_program.cpp), and so does the competition program's opcontrol() (chunk COMP-b) —
 and the invented X-drive's
 program (src/programs/xdrive_program.cpp, since G1 Part 2c), which refuses at boot on every
 robot that exists. ("shulib Teleop" called it by its own call from G1 Part 2b until T4 Part 3
 retired it.) Every one of them gets the drive protections from DriverLoopConfig::protection's
 defaults, since none names the field. No version has run on
 hardware, so nothing in the feel layer it runs has been felt by a driver (HA-112), and no
 protection has cut or marked a real motor (HA-175, HA-176).

 PROS-free, sim-free, allocation-free.
```

</details>
