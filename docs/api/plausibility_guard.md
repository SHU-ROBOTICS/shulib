<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/plausibility_guard.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `plausibility_guard.hpp`

Physical-plausibility invariants (diagnostics-plan D-5; WS13, chunk C5) — FiniteGuard's log-and-recover posture, extended beyond finiteness.

This header declares **2** types (7 members), **2** free functions, and **1** constant.

Extracted from [`include/shulib/diag/plausibility_guard.hpp`](../../include/shulib/diag/plausibility_guard.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PlausibilityConfig`](#struct-plausibilityconfig)
  - [`maxSpeed`](#plausibilityconfig-maxspeed)
  - [`maxYawRate`](#plausibilityconfig-maxyawrate)
  - [`margin`](#plausibilityconfig-margin)
  - [`validate`](#plausibilityconfig-validate)
- [`class PoseDeltaGuard`](#class-posedeltaguard)
  - [`PoseDeltaGuard`](#posedeltaguard-posedeltaguard)
  - [`check`](#posedeltaguard-check)
  - [`reset`](#posedeltaguard-reset)
- [`kCommandAuditMargin`](#kcommandauditmargin) — *constant*
- [`commandWithinCapability`](#commandwithincapability) — *free function*
- [`recoverWheelVoltage`](#recoverwheelvoltage) — *free function*

<a id="struct-plausibilityconfig"></a>

## `struct PlausibilityConfig`

```cpp
struct PlausibilityConfig
```

The physical envelope PoseDeltaGuard judges a tick's pose delta against. The defaults are deliberately GENEROUS hardware claims, not a tuned trip point: they sit far above anything a VEX drivetrain reaches, so a false positive requires the estimate to be wrong by construction. That makes this a bug detector, not a performance limit — tightening it toward the real envelope trades that guarantee for sensitivity. Nothing here bounds a COMMAND.

*struct, declared at [`include/shulib/diag/plausibility_guard.hpp:60`](../../include/shulib/diag/plausibility_guard.hpp#L60).*

<a id="plausibilityconfig-maxspeed"></a>

### `PlausibilityConfig::maxSpeed`

```cpp
units::Velocity maxSpeed{150.0}
```

Physical maximum linear speed the robot could conceivably reach. PROVISIONAL (A4: HA-56) — a 600 rpm 4" drive tops out near 125 in/s.

*field, declared at [`include/shulib/diag/plausibility_guard.hpp:63`](../../include/shulib/diag/plausibility_guard.hpp#L63).*

<a id="plausibilityconfig-maxyawrate"></a>

### `PlausibilityConfig::maxYawRate`

```cpp
units::AngularVelocity maxYawRate{20.0}
```

Physical maximum yaw rate. PROVISIONAL (A4: HA-56) — ~3 rev/s is far past any real chassis.

*field, declared at [`include/shulib/diag/plausibility_guard.hpp:66`](../../include/shulib/diag/plausibility_guard.hpp#L66).*

<a id="plausibilityconfig-margin"></a>

### `PlausibilityConfig::margin`

```cpp
double margin = 1.5
```

Headroom multiplier over the physical maxima (fusion nudges, discretization — header note). Logic constant. Must be >= 1.

*field, declared at [`include/shulib/diag/plausibility_guard.hpp:69`](../../include/shulib/diag/plausibility_guard.hpp#L69).*

<a id="plausibilityconfig-validate"></a>

### `PlausibilityConfig::validate`

```cpp
void validate() const
```

Raise a LOUD precondition if any field is unusable (non-finite or non-positive maxima, margin < 1). Note the polarity — and note what it is NOT: SHULIB_PRECONDITION does not crash. It throws a catchable PreconditionError, and the on-robot policy raises FaultCode::Precondition on the latch BEFORE throwing (core/check.hpp). What separates this from the header's three invariants is RECOVERY, not loudness: they raise Implausible mid-tick and the run continues on a safe value, while this runs at CONSTRUCTION — PoseDeltaGuard's ctor, so in practice while the scheduler is being built at setup — when no motion is in flight for the scheduler's task boundary to convert the throw into a FAULT_ABORT. It therefore leaves the constructor rather than costing one motion, which is the intent: a nonsense envelope is a programming error in the setup, not a runtime anomaly the guard is here to survive. PoseDeltaGuard's constructor already calls it; call it yourself only when you build a config without one.

*function, declared at [`include/shulib/diag/plausibility_guard.hpp:83`](../../include/shulib/diag/plausibility_guard.hpp#L83).*

<a id="class-posedeltaguard"></a>

## `class PoseDeltaGuard`

```cpp
class PoseDeltaGuard
```

Invariant 1 (header): per-tick pose delta within the physical envelope.

*class, declared at [`include/shulib/diag/plausibility_guard.hpp:94`](../../include/shulib/diag/plausibility_guard.hpp#L94).*

<a id="posedeltaguard-posedeltaguard"></a>

### `PoseDeltaGuard::PoseDeltaGuard`

```cpp
explicit PoseDeltaGuard(const PlausibilityConfig& config = {})
```

COPIES `config` and validates it (loud on a nonsense envelope), so later edits to the caller's config never reach this guard. Starts with NO baseline: the first check() only records a pose and returns false, because one sample is not yet a delta.

*function, declared at [`include/shulib/diag/plausibility_guard.hpp:99`](../../include/shulib/diag/plausibility_guard.hpp#L99).*

<a id="posedeltaguard-check"></a>

### `PoseDeltaGuard::check`

```cpp
bool check(const math::Pose2d& pose, units::Time dt, FaultLatch& faults)
```

Feed one tick's estimate. `dt` is the measured tick dt; dt <= 0 (the baseline tick after construction/reset) only re-baselines — there is no interval to judge. Returns true iff THIS tick's delta is implausible (raising Implausible on the latch only on a NEW episode — header note).

*function, declared at [`include/shulib/diag/plausibility_guard.hpp:107`](../../include/shulib/diag/plausibility_guard.hpp#L107).*

<a id="posedeltaguard-reset"></a>

### `PoseDeltaGuard::reset`

```cpp
void reset() noexcept
```

Forget the baseline before a DELIBERATE teleport (e.g. setPose re-seeding between runs) so intent is not reported as pathology.

*function, declared at [`include/shulib/diag/plausibility_guard.hpp:140`](../../include/shulib/diag/plausibility_guard.hpp#L140).*

<a id="kcommandauditmargin"></a>

## `kCommandAuditMargin`

```cpp
inline constexpr double kCommandAuditMargin = 1.01
```

Float headroom for the pipeline self-checks (invariants 2/3): the values were clamped by the same arithmetic that audits them, so anything past 1% is a real regression, not rounding.

*constant, declared at [`include/shulib/diag/plausibility_guard.hpp:152`](../../include/shulib/diag/plausibility_guard.hpp#L152).*

<a id="commandwithincapability"></a>

## `commandWithinCapability`

```cpp
[[nodiscard]] inline bool commandWithinCapability(const math::ChassisSpeeds& body, units::Velocity maxLinear, units::AngularVelocity maxAngular, FaultLatch& faults, std::string_view subsystem) noexcept
```

Invariant 2 (header): the FINAL body command respects the configured budgets. True = plausible. False = Implausible raised (caller decides recovery; the shipped pipeline's own clamps make reaching false a pipeline regression).

*free function, declared at [`include/shulib/diag/plausibility_guard.hpp:157`](../../include/shulib/diag/plausibility_guard.hpp#L157).*

<a id="recoverwheelvoltage"></a>

## `recoverWheelVoltage`

```cpp
[[nodiscard]] inline units::Voltage recoverWheelVoltage(units::Voltage v, units::Voltage ceiling, FaultLatch& faults, std::string_view subsystem) noexcept
```

Invariant 3 (header): one wheel volt, made consistent with the battery ceiling. ALWAYS returns a safe value (the FiniteGuard shape): finite in-range volts pass untouched; non-finite → Implausible + 0 V; over-ceiling → Implausible + clamped.

*free function, declared at [`include/shulib/diag/plausibility_guard.hpp:183`](../../include/shulib/diag/plausibility_guard.hpp#L183).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 40 lines</summary>

```text

 Physical-plausibility invariants (diagnostics-plan D-5; WS13, chunk C5) —
 FiniteGuard's log-and-recover posture, extended beyond finiteness. A value can
 be perfectly finite and still be a lie: a pose that teleported 8 inches in one
 10 ms tick, a commanded speed beyond every configured budget, a wheel volt above
 the battery ceiling. Each such violation raises FaultCode::Implausible (a fault,
 never a crash — fault.hpp's absolute rule) and the run continues on a safe
 fallback. A3 proved this class of guard catches real defects; D-6's flight
 recorder (E1) will trigger on exactly these faults.

 The three invariants and where each runs:
   1. POSE DELTA (PoseDeltaGuard, run by the scheduler each tick): |Δposition| and
      |Δheading| within the drivetrain's physical maximum × margin × dt. Catches a
      lying estimate the moment it lies (encoder glitch, fusion bug, a mid-run
      setPose). Recovery = the fault is ADVISORY: the guard does not rewrite the
      estimate (the Localizer owns the pose; a diagnostic that mutates the data
      path is worse than the bug it hunts — principle 4). The watchdogs and the
      fault policy bound the damage; E-phase correctors may additionally gate on
      this fault. EPISODE-gated like HealthMonitor: a jump that persists is one
      episode, not a 100 Hz fault storm; a healthy tick re-arms.
   2. COMMAND WITHIN CAPABILITY (commandWithinCapability, called by the command
      pipeline after its own clamps): the final body command must respect the
      configured budgets. The pipeline just ENFORCED those clamps, which is the
      point — this is a defense-in-depth self-check, and a violation means the
      pipeline itself regressed (the class of bug no closed-loop test can see —
      C4's M21 lesson). Deliberately NOT episode-gated: it has no home for state
      in a free-function pipeline, and a persistent pipeline regression SHOULD be
      loud (the latch's saturating tally bounds the damage).
   3. WHEEL VOLTS CONSISTENT (recoverWheelVoltage, same call site): each commanded
      volt finite and within the battery ceiling. Recovery is REAL here: a
      non-finite volt becomes 0 V, an over-ceiling volt is clamped — the bad value
      never reaches a motor.

 The margins: margin (default 1.5) absorbs what the estimate may legitimately do
 beyond commanded physics — fusion nudges (never-snap-clamped, but nonzero),
 discretization, settle chatter. kCommandAuditMargin (1.01) is float headroom
 over an exact clamp. Both are logic constants. The PHYSICAL MAXIMA defaults are
 hardware claims: PROVISIONAL (A4: HA-56) — generous upper bounds no VEX
 drivetrain approaches, so a false positive requires the estimate to be wrong by
 construction; R3/R5 replace them with measured envelopes.
```

</details>
