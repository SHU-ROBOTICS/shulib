<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/leg_progress.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `leg_progress.hpp`

leg_progress.hpp — how far a drive or a turn has got, as a PARTWAY TRIGGER reads it (chunk STEP1-B, 2026-11-11; the STEP1 brief's D7 (a)).

This header declares **1** type (9 members), **2** free functions, and **2** constants.

Extracted from [`include/shulib/motion/leg_progress.hpp`](../../include/shulib/motion/leg_progress.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct LegProgress`](#struct-legprogress)
  - [`live`](#legprogress-live)
  - [`fraction`](#legprogress-fraction)
  - [`along`](#legprogress-along)
  - [`turnedRad`](#legprogress-turnedrad)
  - [`hasDirection`](#legprogress-hasdirection)
  - [`length`](#legprogress-length)
  - [`sizeRad`](#legprogress-sizerad)
  - [`positionTolerance`](#legprogress-positiontolerance)
  - [`headingToleranceRad`](#legprogress-headingtolerancerad)
- [`kMinLegLength`](#kminleglength) — *constant*
- [`kMinTurnRad`](#kminturnrad) — *constant*
- [`legProgressAlong`](#legprogressalong) — *free function*
- [`turnProgress`](#turnprogress) — *free function*

<a id="struct-legprogress"></a>

## `struct LegProgress`

```cpp
struct LegProgress
```

A leg's progress, as a partway trigger reads it — handed to a chassis::LegObserver after every tick of the leg that did not end it (chassis/chassis.hpp's observer overloads). OPEN to new fields (an aggregate's trailing field is additive): no pin covers its field list (the STEP1 brief's D1).

*struct, declared at [`include/shulib/motion/leg_progress.hpp:37`](../../include/shulib/motion/leg_progress.hpp#L37).*

<a id="legprogress-live"></a>

### `LegProgress::live`

```cpp
bool live = false
```

True once the leg's motion has had its first LIVE tick (a live estimate; the start pose is captured there). False through the boot wait: nothing else here is meaningful then.

*field, declared at [`include/shulib/motion/leg_progress.hpp:40`](../../include/shulib/motion/leg_progress.hpp#L40).*

<a id="legprogress-fraction"></a>

### `LegProgress::fraction`

```cpp
double fraction = 0.0
```

The share of the leg done: `along / length` on a drive, `turnedRad / sizeRad` on a turn (1.0 = at the target; above 1 = past it). 0 when the leg has no direction.

*field, declared at [`include/shulib/motion/leg_progress.hpp:43`](../../include/shulib/motion/leg_progress.hpp#L43).*

<a id="legprogress-along"></a>

### `LegProgress::along`

```cpp
units::Length along{0.0}
```

A DRIVE's distance covered along the straight line from its live start to its target (D7 (a)); 0 on a turn, and on a drive with no direction.

*field, declared at [`include/shulib/motion/leg_progress.hpp:46`](../../include/shulib/motion/leg_progress.hpp#L46).*

<a id="legprogress-turnedrad"></a>

### `LegProgress::turnedRad`

```cpp
double turnedRad = 0.0
```

A TURN's angle turned toward its target since its live start, radians (the file header); 0 on a drive, and on a turn with no direction.

*field, declared at [`include/shulib/motion/leg_progress.hpp:49`](../../include/shulib/motion/leg_progress.hpp#L49).*

<a id="legprogress-hasdirection"></a>

### `LegProgress::hasDirection`

```cpp
bool hasDirection = false
```

False for a zero-size leg (the file header, B5): nothing measures its progress.

*field, declared at [`include/shulib/motion/leg_progress.hpp:53`](../../include/shulib/motion/leg_progress.hpp#L53).*

<a id="legprogress-length"></a>

### `LegProgress::length`

```cpp
units::Length length{0.0}
```

A DRIVE's straight-line length from its live start to its target; 0 on a turn.

*field, declared at [`include/shulib/motion/leg_progress.hpp:55`](../../include/shulib/motion/leg_progress.hpp#L55).*

<a id="legprogress-sizerad"></a>

### `LegProgress::sizeRad`

```cpp
double sizeRad = 0.0
```

A TURN's size: |the start's shortest signed error to the target|, radians; 0 on a drive.

*field, declared at [`include/shulib/motion/leg_progress.hpp:57`](../../include/shulib/motion/leg_progress.hpp#L57).*

<a id="legprogress-positiontolerance"></a>

### `LegProgress::positionTolerance`

```cpp
units::Length positionTolerance{0.0}
```

The leg's EFFECTIVE position arrival tolerance — the settle distance it runs with (the robot's own, or the one a MotionOptions::positionTolerance loosened: motion::loosenedSettle).

*field, declared at [`include/shulib/motion/leg_progress.hpp:60`](../../include/shulib/motion/leg_progress.hpp#L60).*

<a id="legprogress-headingtolerancerad"></a>

### `LegProgress::headingToleranceRad`

```cpp
double headingToleranceRad = 0.0
```

The leg's EFFECTIVE heading arrival tolerance, radians (as positionTolerance, on the heading).

*field, declared at [`include/shulib/motion/leg_progress.hpp:62`](../../include/shulib/motion/leg_progress.hpp#L62).*

<a id="kminleglength"></a>

## `kMinLegLength`

```cpp
inline constexpr double kMinLegLength = 1e-6
```

Below this a drive has no direction (inches): the projection would divide by a length that is numerical noise. The same order as Routine's bearingTo() refusal (1e-9 in), kept looser because a fraction over a length of 1e-7 in is meaningless even where a bearing is defined.

*constant, declared at [`include/shulib/motion/leg_progress.hpp:68`](../../include/shulib/motion/leg_progress.hpp#L68).*

<a id="kminturnrad"></a>

## `kMinTurnRad`

```cpp
inline constexpr double kMinTurnRad = 1e-9
```

Below this a turn has no direction (radians) — kMinLegLength's reason, on the heading.

*constant, declared at [`include/shulib/motion/leg_progress.hpp:70`](../../include/shulib/motion/leg_progress.hpp#L70).*

<a id="legprogressalong"></a>

## `legProgressAlong`

```cpp
[[nodiscard]] inline LegProgress legProgressAlong(const math::Pose2d& start, const math::Pose2d& target, const math::Pose2d& pose) noexcept
```

A DRIVE leg's progress at `pose`, the leg having started (live) at `start` and aiming at `target`'s POSITION (its heading is not read): the projection of (pose − start) on the unit start→target vector (the file header). `live` is set; the tolerances are left 0 for the caller.

*free function, declared at [`include/shulib/motion/leg_progress.hpp:75`](../../include/shulib/motion/leg_progress.hpp#L75).*

<a id="turnprogress"></a>

## `turnProgress`

```cpp
[[nodiscard]] inline LegProgress turnProgress(math::Angle startHeading, math::Angle target, math::Angle heading) noexcept
```

A TURN leg's progress at `heading`, the turn having started (live) at `startHeading` and aiming at `target`: the angle turned in the direction of the start's shortest signed error, over its size (the file header). `live` is set; the tolerances are left 0 for the caller.

*free function, declared at [`include/shulib/motion/leg_progress.hpp:97`](../../include/shulib/motion/leg_progress.hpp#L97).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 23 lines</summary>

```text

 leg_progress.hpp — how far a drive or a turn has got, as a PARTWAY TRIGGER reads it (chunk STEP1-B,
 2026-11-11; the STEP1 brief's D7 (a)). PURE: geometry only — no chassis, no scheduler, no sequence
 include — so every rule here is host-tested on its own (test/leg_progress_test.cpp).

 ── What "60 % of the drive" measures (D7 (a), the team lead's answer of 2026-11-03) ────────────
 A DRIVE's progress is the distance covered ALONG THE STRAIGHT LINE from where the leg started to
 its target: the projection of (pose − start) on the unit start→target vector. Not the distance
 still to go (a robot that starts off its line would gain "progress" by moving sideways), not the
 odometry's path length (a wobbly leg would reach "60 %" early). It can pass 100 % (an overshoot)
 and read below 0 (a robot pushed backwards); a trigger compares it with `>=`, once.
 A TURN's progress is the angle turned in the direction of the start's shortest signed error, over
 that error's size — the same short way TurnTo turns (F3's errorTo; an exact ±180° resolves CCW,
 as TurnTo's does). The turned angle is `startHeading.errorTo(heading)` with the turn's sign, so
 it is honest while the robot has turned less than 180° from its start in either direction; a turn
 near 180° that OVERSHOOTS past 180° from its start wraps to a negative angle (a mark at or below
 100 % has been crossed by then — the wrap only hides an overshoot, never a mark short of it).

 ── A zero-size leg has no direction (the brief's B5) ───────────────────────────────────────────
 A drive whose target is (numerically) where it started — under kMinLegLength — or a turn to the
 heading it already has — under kMinTurnRad — has no "percent" and no direction to measure along:
 `hasDirection` is false and every progress field reads 0, so nothing fires DURING such a leg.
 (What fires when it settles is the Routine's rule: chassis/routine.hpp, B3.)
```

</details>
