<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/tank_steer.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tank_steer.hpp`

tank_steer.hpp — the pure geometry of TANK1: where a tank is relative to its target's LINE, and how far MoveToPose bends the heading setpoint to steer it back.

This header declares **2** types (8 members), **4** free functions, and **1** constant.

Extracted from [`include/shulib/motion/tank_steer.hpp`](../../include/shulib/motion/tank_steer.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct TankLineError`](#struct-tanklineerror)
  - [`along`](#tanklineerror-along)
  - [`cross`](#tanklineerror-cross)
- [`tankLineError`](#tanklineerror) — *free function*
- [`tankLeadCross`](#tankleadcross) — *free function*
- [`tankSteerAngle`](#tanksteerangle) — *free function*
- [`tankLegEngages`](#tanklegengages) — *free function*
- [`struct TankSteerStatus`](#struct-tanksteerstatus)
  - [`eligible`](#tanksteerstatus-eligible)
  - [`engaged`](#tanksteerstatus-engaged)
  - [`cross`](#tanksteerstatus-cross)
  - [`crossAhead`](#tanksteerstatus-crossahead)
  - [`bend`](#tanksteerstatus-bend)
  - [`yawCorrection`](#tanksteerstatus-yawcorrection)
- [`kTankSteerNoiseFraction`](#ktanksteernoisefraction) — *constant*

<a id="struct-tanklineerror"></a>

## `struct TankLineError`

```cpp
struct TankLineError
```

The robot's position in its target's line frame (header): inches.

*struct, declared at [`include/shulib/motion/tank_steer.hpp:64`](../../include/shulib/motion/tank_steer.hpp#L64).*

<a id="tanklineerror-along"></a>

### `TankLineError::along`

```cpp
double along = 0.0
```

along the target heading, from the target (− before it going forward)

*field, declared at [`include/shulib/motion/tank_steer.hpp:65`](../../include/shulib/motion/tank_steer.hpp#L65).*

<a id="tanklineerror-cross"></a>

### `TankLineError::cross`

```cpp
double cross = 0.0
```

off the line, + to the LEFT of the target heading

*field, declared at [`include/shulib/motion/tank_steer.hpp:66`](../../include/shulib/motion/tank_steer.hpp#L66).*

<a id="tanklineerror"></a>

## `tankLineError`

```cpp
[[nodiscard]] inline TankLineError tankLineError(const math::Pose2d& pose, const math::Pose2d& target) noexcept
```

Where `pose` sits relative to the line through `target` along `target.heading()`.

*free function, declared at [`include/shulib/motion/tank_steer.hpp:70`](../../include/shulib/motion/tank_steer.hpp#L70).*

<a id="tankleadcross"></a>

## `tankLeadCross`

```cpp
[[nodiscard]] inline double tankLeadCross(const TankLineError& e, double headingFromTarget, double forwardSpeed, const TankSteerConfig& cfg) noexcept
```

The cross-track error (in) `cfg.leadTime` seconds ahead (header: the lead): the error now plus how far the robot closes on or leaves the line in that time if it keeps its heading and `forwardSpeed` (in/s along its own heading, + forward, − backing up — MoveToPose passes the body forward speed it commanded the tick before). `headingFromTarget` is the robot's heading minus the target heading (rad; any wrap — only its sine is used). leadTime 0 returns e.cross.

*free function, declared at [`include/shulib/motion/tank_steer.hpp:85`](../../include/shulib/motion/tank_steer.hpp#L85).*

<a id="tanksteerangle"></a>

## `tankSteerAngle`

```cpp
[[nodiscard]] inline double tankSteerAngle(const TankLineError& e, const TankSteerConfig& cfg) noexcept
```

The bend (rad) TANK1 adds to the target heading for this line error (header formula) — MoveToPose passes the error with its cross replaced by tankLeadCross(). Bounded by cfg.maxSteer; zero at along == 0 and at cross == 0; odd in cross; its sign flips with the direction of approach.

*free function, declared at [`include/shulib/motion/tank_steer.hpp:95`](../../include/shulib/motion/tank_steer.hpp#L95).*

<a id="tanklegengages"></a>

## `tankLegEngages`

```cpp
[[nodiscard]] inline bool tankLegEngages(double startHeadingError, const TankSteerConfig& cfg) noexcept
```

Whether a tank leg whose FIRST LIVE heading error (target heading minus heading, shortest, rad) is `startHeadingError` is steered at all: only when TANK1 is on (maxSteer > 0) and the leg starts within maxSteer of its heading — inclusive, so a leg exactly at the window is steered. A leg outside it is the author's turn (face()), and runs exactly as before TANK1.

*free function, declared at [`include/shulib/motion/tank_steer.hpp:106`](../../include/shulib/motion/tank_steer.hpp#L106).*

<a id="struct-tanksteerstatus"></a>

## `struct TankSteerStatus`

```cpp
struct TankSteerStatus
```

What TANK1 did on a MoveToPose's most recent tick — for tests and Tier-3 tooling; the record stream carries only the flag (DebugRecord::tankSteerActive).

*struct, declared at [`include/shulib/motion/tank_steer.hpp:113`](../../include/shulib/motion/tank_steer.hpp#L113).*

<a id="tanksteerstatus-eligible"></a>

### `TankSteerStatus::eligible`

```cpp
bool eligible = false
```

tank + plain MoveToPose + TANK1 on

*field, declared at [`include/shulib/motion/tank_steer.hpp:114`](../../include/shulib/motion/tank_steer.hpp#L114).*

<a id="tanksteerstatus-engaged"></a>

### `TankSteerStatus::engaged`

```cpp
bool engaged = false
```

this run steers (decided at the first live tick)

*field, declared at [`include/shulib/motion/tank_steer.hpp:115`](../../include/shulib/motion/tank_steer.hpp#L115).*

<a id="tanksteerstatus-cross"></a>

### `TankSteerStatus::cross`

```cpp
double cross = 0.0
```

last tick's cross-track error (in), + left of the line

*field, declared at [`include/shulib/motion/tank_steer.hpp:116`](../../include/shulib/motion/tank_steer.hpp#L116).*

<a id="tanksteerstatus-crossahead"></a>

### `TankSteerStatus::crossAhead`

```cpp
double crossAhead = 0.0
```

last tick's tankLeadCross() — the error the bend steered on

*field, declared at [`include/shulib/motion/tank_steer.hpp:117`](../../include/shulib/motion/tank_steer.hpp#L117).*

<a id="tanksteerstatus-bend"></a>

### `TankSteerStatus::bend`

```cpp
double bend = 0.0
```

last tick's heading-setpoint bend (rad)

*field, declared at [`include/shulib/motion/tank_steer.hpp:118`](../../include/shulib/motion/tank_steer.hpp#L118).*

<a id="tanksteerstatus-yawcorrection"></a>

### `TankSteerStatus::yawCorrection`

```cpp
double yawCorrection = 0.0
```

the yaw rate (rad/s) TANK1 subtracted last tick

*field, declared at [`include/shulib/motion/tank_steer.hpp:119`](../../include/shulib/motion/tank_steer.hpp#L119).*

<a id="ktanksteernoisefraction"></a>

## `kTankSteerNoiseFraction`

```cpp
inline constexpr double kTankSteerNoiseFraction = 0.02
```

tankSteerActive's legibility floor, as a fraction of maxAngularSpeed: TANK1 must have changed the tick's yaw command (its bend's P-share plus its yaw correction) by more than this before the tick is flagged. 2 % of the HA-50 budget is 0.12 rad/s; the floor in rad/s moves with maxAngularSpeed. Measured on the host test plant (test/motion_tank_steer_test.cpp, P7, at 6 rad/s): under composed hostility on an EVEN drive, over ten 40 in legs, the flag lights on 20 of 2 125 ticks (0.9 %; the worst single leg 2.9 %), where with no floor IMU rate noise reaching the yaw observer lights it on 2 109 of them — nearly every tick; a 5 % weaker side lights it on 81 of a leg's 186 ticks. (Re-measured 2026-09-28 with the bend's lead: 14 of 2 125 before it.) The same kind of telemetry-legibility constant as kStrafeFallbackNoiseFraction — host-decidable, not a register entry (A4 register rule 1).

*constant, declared at [`include/shulib/motion/tank_steer.hpp:132`](../../include/shulib/motion/tank_steer.hpp#L132).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 tank_steer.hpp — the pure geometry of TANK1: where a tank is relative to its target's LINE,
 and how far MoveToPose bends the heading setpoint to steer it back (chunk TANK1, 2026-09).

 ── The line, and the two numbers ────────────────────────────────────────────────────
 The line runs through the target POINT along the target HEADING. In that line's frame:
   along = the robot's signed distance along the target heading, measured FROM the target
           (negative before the target on a forward approach, positive past it);
   cross = the robot's signed distance off the line, + to the LEFT of the target heading.
 Neither uses a bearing to the target point: nothing here ever aims the robot AT the point.
 That is the difference from a pursuit or boomerang controller, and it is why TANK1 cannot
 plan a turn (C1's D12, amended by TANK1: bounded steering toward the author's LINE is
 allowed and telemetry-visible; turning toward the POINT is not).

 ── The bend ─────────────────────────────────────────────────────────────────────────
   delta = −clamp(−along / fadeDistance, −1, 1) · clamp(crossGain · cross, ±maxSteer)
 The first factor is the fade AND the direction: +1 while the target is ahead along its
 heading (a forward approach), −1 while it is behind (a reverse approach — backing up, the
 nose must turn the OTHER way to bring the robot back), and continuous through 0 at arrival,
 so the bend vanishes exactly where the heading criterion is judged. There is no speed in any
 denominator: the classic Stanley singularity (gain ∝ 1/v as v → 0) cannot occur.

 ── The lead: steer on where the robot is GOING to be off the line ───────────────────
 The cross fed to the bend is not the error now but the error leadTime seconds ahead, if the
 robot kept its heading and the forward speed it was last commanded:
   crossAhead = cross + leadTime · v · sin(heading − target heading)
 A tank moves along its heading, so v·sin(θ − θt) is exactly how fast it closes on (or leaves)
 the line; signed v makes the same formula right backing up. Why it is there (the Part A
 review, 2026-09-28, measured on the host test plant): the heading loop turns the robot with a
 lag (kP 4 ≈ 0.25 s, ~15 in at full speed), so a bend on the present error alone is an
 UNDER-DAMPED line loop — a leg that starts pointed at its line (10–15° off its target heading,
 its point on the arc the heading loop drives) ran at the line, crossed it near the target, and
 the fade then left 0.5–0.7 in nobody could remove: legs that finished before TANK1 timed out.
 The same root left holes in the lateral reach (a faced leg 50 in long settled a 3 in and a
 7 in offset but timed out on 4–6 in). Steering on the predicted error damps it: nothing is
 computed from the target POINT, the bend keeps its cap, and leadTime 0 is the bend without it.

 ── The settle bound ─────────────────────────────────────────────────────────────────
 When a leg is within the translation tolerance (|along| ≤ 0.5 in and |cross| ≤ 0.5 in),
 |delta| ≤ (0.5/fadeDistance)·crossGain·|crossAhead|, which at the PROVISIONAL defaults is
 0.0042 rad for |crossAhead| ≤ 0.5 in — about a fifth of the 0.02 rad heading tolerance — so the
 bend can never hold a settle open. The lead adds nothing there that matters: a leg in its
 tolerance is commanded at most translation.kP × 0.5 in = 1.5 in/s, and with the heading inside
 its own 0.02 rad the lead is at most leadTime × 1.5 × 0.02 = 0.002 in. Pinned over a grid by
 test/tank_steer_test.cpp.

 What uses this: MoveToPose (move_to_pose.hpp) on a drivetrain whose strafeAuthority() is
 exactly 0, for a leg that starts within maxSteer of its target heading — the only thing that
 STEERS with it. One reader besides (TANK1 Part B): the scheduler's MotionStatsSink
 (motion_scheduler.hpp) reports tankLineError's `cross` of a finished motion's last estimate as
 the result line's ` xt` column, on the same drivetrains — a number, never a command.
 PROS-free, allocation-free, no state: every function here is pure.
```

</details>
