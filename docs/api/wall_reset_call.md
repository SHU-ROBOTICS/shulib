<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/wall_reset_call.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `wall_reset_call.hpp`

chassis::resetFromWall — the wall reset's EXPLICIT call (chunk L6-A1, 2026-10-29): re-seat ONE axis of the estimate against a field wall from a side distance sensor, over sensor handles the caller passes.

This header declares **1** free function and **1** constant.

Extracted from [`include/shulib/chassis/wall_reset_call.hpp`](../../include/shulib/chassis/wall_reset_call.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kWallResetWaitBackstopSeconds`](#kwallresetwaitbackstopseconds) — *constant*
- [`resetFromWall`](#resetfromwall) — *free function*

<a id="kwallresetwaitbackstopseconds"></a>

## `kWallResetWaitBackstopSeconds`

```cpp
inline constexpr double kWallResetWaitBackstopSeconds = 1.0
```

Slack past `windowMax` given to the sampling wait's own timeout (seconds). Unreachable by construction — the predicate ends the window at `windowMax` by the clock — so it is code-level belt-and-braces, not an HA entry (Chassis's wait() backstop is the same idea).

*constant, declared at [`include/shulib/chassis/wall_reset_call.hpp:61`](../../include/shulib/chassis/wall_reset_call.hpp#L61).*

<a id="resetfromwall"></a>

## `resetFromWall`

```cpp
[[nodiscard]] inline localization::WallResetReport resetFromWall(Chassis& chassis, const localization::WallSensor& sensor, const localization::WallResetRequest& request, const localization::WallResetLimits& limits)
```

THE EXPLICIT CALL (L6-A1 — a hand-wired Tier-3 robot's form, and the host tests'): re-seat ONE axis of `chassis`'s estimate against `request.wall` from `sensor`, inside `limits` — blocking, at most `limits.windowMax` plus a tick; the setup and pre-checks first (no time, nothing read), then a window of samples inside `chassis.waitUntil` (its predicate ENDS the wait — no "timed out" line), the verdict, and, when SEATED, `Localizer::seatAxis` on that axis only. ONE log line either way (subsystem "WALL"). Commands nothing to the drive. The heading tolerance and the along-wall drift allowance come from the Localizer's counters (`turnedSinceHeadingSeed`, `travelSinceSeat`). The window is a FROZEN wait: a deadline landing in it is acted on up to `windowMax` late (the everyday form, `competition::resetFromWall(ctx, ...)`, is cut instead). Returns the report — a VALUE; `continues()` is what a `then()` returns (D3).

*free function, declared at [`include/shulib/chassis/wall_reset_call.hpp:170`](../../include/shulib/chassis/wall_reset_call.hpp#L170).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 43 lines</summary>

```text

 chassis::resetFromWall — the wall reset's EXPLICIT call (chunk L6-A1, 2026-10-29): re-seat ONE axis
 of the estimate against a field wall from a side distance sensor, over sensor handles the caller
 passes. The form for a hand-wired Tier-3 robot and for the host tests; an auton uses the everyday
 form over its context (`competition::resetFromWall(ctx, ...)`, competition/auton_wall_reset.hpp — L6-A2),
 which runs the same steps through the same sampler below.

 ── Why it sits HERE, at the facade's layer ──────────────────────────────────────────────────────
 It needs the Chassis (the estimate, the scheduler's wait, the telemetry), and a localization/
 header may not include upward (the master plan's §5 — and doing so closes an include cycle through
 motion/motion.hpp, which includes the Localizer). So the pure gates live in
 localization/wall_reset.hpp and this call sits over chassis.hpp, as chassis/routine.hpp does.

 ── What one call does ───────────────────────────────────────────────────────────────────────────
   1. Setup, reading NOTHING: the sensor exists (else SensorAbsent, with its absence sentence), its
      mount was measured (MountUnset), the estimate is live (NotLive), NO motion is armed
      (MotionActive — the sampling wait keeps an armed motion ticking, and a seat under it is the
      master plan's risk 5, a pose jump mid-motion), the request is usable (BadRequest, naming the
      field); then rules 2-4 from the estimate alone (NotSquare, KeepOut, BeamLow). A refusal here
      costs no time and reads no sensor.
   2. The window: samples inside `Chassis::waitUntil`, so the Localizer and every health check keep
      ticking. The PREDICATE ends the wait — at the first refusing tick, once the window has enough,
      or at `windowMax` by the clock — so the wait never reaches its own timeout and never writes
      the scheduler's "waitUntil timed out" line.
   3. The verdict (rules 5-9, localization::evaluateWallReset); when SEATED, ONE axis is seated
      through `Localizer::seatAxis` — never setPose, never the heading.
   4. ONE log line, subsystem "WALL": Info when SEATED, Warn when REFUSED.
 It commands NOTHING to the drive: a robot still coasting refuses MOVING. A refusal leaves the
 estimate bit-identical; what the auton does next is its `required` (D3) through `then()`.

 ── The frozen wait ──────────────────────────────────────────────────────────────────────────────
 `Chassis::waitUntil` is a wait the run guard cannot cut, so a deadline or a disable that lands
 mid-window is acted on at most `windowMax` (0.5 s by default, never more than 1 s) late. The
 everyday form (competition/auton_wall_reset.hpp) samples through the guard's deadline-aware wait instead
 and refuses CUT.

 ── A seat is a deliberate jump (the scheduler's pose guard) ────────────────────────────────────
 The scheduler's plausibility guard raises an advisory IMPLAUSIBLE when one tick's pose change
 exceeds 150 in/s x 1.5 x dt — 2.25 in at the 10-ms tick — and forgives a jump only on a dt-0 tick,
 which every verb and wait gives on its first tick. So a seat over 2.25 in followed by a Routine
 step raises nothing; a Tier-3 loop that ticks the scheduler itself (`scheduler().tick()`) sees ONE
 advisory IMPLAUSIBLE per such seat. Never "fix" that by clamping the seat (the report would then lie
 about what it applied) or by silencing the guard.
```

</details>
