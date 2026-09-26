<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/teleop_loop.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `teleop_loop.hpp`

runDriverControl — the library's driver-control loop.

This header declares **2** types (4 members) and **1** free function.

Extracted from [`include/shulib/teleop/teleop_loop.hpp`](../../include/shulib/teleop/teleop_loop.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriverLoopConfig`](#struct-driverloopconfig)
  - [`tickPeriod`](#driverloopconfig-tickperiod)
  - [`rezeroButton`](#driverloopconfig-rezerobutton)
- [`struct HeadingSample`](#struct-headingsample)
  - [`radians`](#headingsample-radians)
  - [`trusted`](#headingsample-trusted)
- [`runDriverControl`](#rundrivercontrol) — *free function*

<a id="struct-driverloopconfig"></a>

## `struct DriverLoopConfig`

```cpp
struct DriverLoopConfig
```

The two numbers the loop needs that neither the chassis nor the controller can supply.  State BOTH, with designated initializers — `{.tickPeriod = …, .rezeroButton = …}`. The period's default is deliberately not a working configuration: a zero period trips `runDriverControl`'s precondition before anything moves, so a config nobody wrote can never run. The button's default is NOT so guarded: a config that leaves `rezeroButton` unset quietly gets `L1` — a shoulder button, which the planned claw bindings take — so a program must name its button ("shulib Teleop" names `A`). Making this default refuse too is owed before any program binds a mechanism to a shoulder button.

*struct, declared at [`include/shulib/teleop/teleop_loop.hpp:82`](../../include/shulib/teleop/teleop_loop.hpp#L82).*

<a id="driverloopconfig-tickperiod"></a>

### `DriverLoopConfig::tickPeriod`

```cpp
units::Time tickPeriod{0.0}
```

How long one tick is — the pacer's period (on a brain, `ProsTickPacer::kTickMs` = 10 ms, HA-32). Handed to the shaper as its dt, where only the slew limiter reads it. Must be finite and > 0. `motion::ITickPacer` has no period of its own, and cannot gain one (M3), which is why the program states it here.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:87`](../../include/shulib/teleop/teleop_loop.hpp#L87).*

<a id="driverloopconfig-rezerobutton"></a>

### `DriverLoopConfig::rezeroButton`

```cpp
hal::ControllerButton rezeroButton = hal::ControllerButton::L1
```

The driver's field-centric re-zero button (`DriveShaper::rezero`). Harmless in a body mode, where a press only pre-arms the zero for a later switch to `ArcadeField`. Which button is the PROGRAM's choice, so state it.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:91`](../../include/shulib/teleop/teleop_loop.hpp#L91).*

<a id="struct-headingsample"></a>

## `struct HeadingSample`

```cpp
struct HeadingSample
```

One tick's heading, as the loop hands it to the shaper: the robot's estimated heading and the caller's verdict on it. What a `runDriverControl` heading source returns.

*struct, declared at [`include/shulib/teleop/teleop_loop.hpp:96`](../../include/shulib/teleop/teleop_loop.hpp#L96).*

<a id="headingsample-radians"></a>

### `HeadingSample::radians`

```cpp
double radians = 0.0
```

The estimated heading, radians, CCW-positive, in the same frame the driver's zero is captured in. Ignored unless the profile's mode is `ArcadeField`.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:99`](../../include/shulib/teleop/teleop_loop.hpp#L99).*

<a id="headingsample-trusted"></a>

### `HeadingSample::trusted`

```cpp
bool trusted = false
```

True only when the estimate is live and the heading is believable (for a robot::Robot: the Localizer is past `Quality::Uninitialized` and the IMU reports ready). False makes field-centric fall back to robot-centric and LATCH until the driver re-zeroes.

*field, declared at [`include/shulib/teleop/teleop_loop.hpp:103`](../../include/shulib/teleop/teleop_loop.hpp#L103).*

<a id="rundrivercontrol"></a>

## `runDriverControl`

```cpp
template <typename IsDisabled, typename HeadingSource, typename PerTick> void runDriverControl(chassis::Chassis& chassis, const hal::IController& controller, motion::ITickPacer& pacer, DriveShaper& shaper, const DriverLoopConfig& cfg, IsDisabled&& disabled, HeadingSource&& heading, PerTick&& perTick)
```

Run driver control until `disabled()` says stop: sticks → `shaper` → budgets → the chassis, once per tick, in the order the file header lists (disabled check, sticks while connected, one heading sample, the re-zero edge, shape, `drive(Body)`, `perTick`, `pace`).  * `chassis` — the facade to drive; its `motionConfig()` supplies the speed budgets (`maxLinearSpeed`, `maxAngularSpeed` — HA-50, provisional). * `controller` — the driver's controller (VEX U's partner is a second call with a second shaper). * `pacer` — the tick boundary: on a brain the provider's `ProsTickPacer`; in a host test the plant-stepping pacer. * `shaper` — the driver's feel (`DriveShaper`, one per driver, held across calls so its field-centric zero and latch survive a disable). * `cfg` — the tick period and the re-zero button. PRECONDITION: `cfg.tickPeriod` finite and > 0, checked before the first tick. * `disabled` — `bool()`: polled FIRST every tick; true ⇒ `chassis.cancel()` (0 V + Brake) and return. The only way out. * `heading` — `HeadingSample()`: called once per tick, after the sticks, before shaping. * `perTick` — `void(const math::ChassisSpeeds& command, bool connected, const DriveShaperState& state)`: called after `drive()`, before `pace()` — the panel's hook.  Nothing is read from the controller while it is disconnected; the command is then exactly zero whatever the axes claim (HA-103).

*free function, declared at [`include/shulib/teleop/teleop_loop.hpp:129`](../../include/shulib/teleop/teleop_loop.hpp#L129).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

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
                      inside shape().
   7. perTick(command, connected, shaper.state())  the caller's panel.
   8. pacer.pace()    the tick boundary.

 No buffer, no queue, no "apply next tick": a stick step read at tick k is in the commanded
 volts of tick k (pinned through the host plant). PERCEIVED latency is not claimed — real
 input-to-motion latency is PROS call cost plus loop rate, and neither is measured.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-proven on the A2 plant only. Since G1 Part 2b "shulib Teleop" (src/teleop_program.cpp)
 drives through it, with the default DriverProfile (ArcadeBody) and field-centric selectable but
 OFF — and that version of the program has not run on hardware, so nothing in the feel layer it
 runs has been felt by a driver (HA-112).

 PROS-free, sim-free, allocation-free.
```

</details>
