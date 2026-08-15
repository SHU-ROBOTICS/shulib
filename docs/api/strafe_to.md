<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/strafe_to.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `strafe_to.hpp`

StrafeTo — translate to a FIELD (x, y) while HOLDING heading.

This header declares **1** type (2 members).

Extracted from [`include/shulib/motion/strafe_to.hpp`](../../include/shulib/motion/strafe_to.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class StrafeTo`](#class-strafeto)
  - [`StrafeTo`](#strafeto-strafeto)
  - [`name`](#strafeto-name)

<a id="class-strafeto"></a>

## `class StrafeTo`

```cpp
class StrafeTo final : public MoveToPose
```

Translate to a FIELD (x, y) while actively HOLDING heading. The heading loop is not switched off — it is given a target: the heading the robot actually has at the FIRST LIVE tick, captured then rather than at start() so it can never lock onto a boot-window estimate. So "hold" here means a closed loop that fights disturbance, not an absence of rotation command, and a heading that wanders during the translation is a test failure. The target heading is therefore NOT knowable before the motion runs: read target() after the first live tick, and note that the heading component of the inherited setTarget() is discarded by the same capture. On a drivetrain with no strafe authority (tank) the body-lateral command is clamped to zero, so a laterally-offset target is physically unreachable and the motion exits TimedOut rather than hanging or claiming a success it never had.

*class, declared at [`include/shulib/motion/strafe_to.hpp:34`](../../include/shulib/motion/strafe_to.hpp#L34).*

<a id="strafeto-strafeto"></a>

### `StrafeTo::StrafeTo`

```cpp
StrafeTo(const MotionDeps& deps, units::Length x, units::Length y, const MotionConfig& config = {}, double timeout = 0.0)
```

Translate to FIELD (x, y), holding the first-live heading. `timeout` (s) bounds the whole motion including any boot wait; 0 selects config.defaultTimeout.

*function, declared at [`include/shulib/motion/strafe_to.hpp:39`](../../include/shulib/motion/strafe_to.hpp#L39).*

<a id="strafeto-name"></a>

### `StrafeTo::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

"StrafeTo". Overridden so the two places this string surfaces name THIS motion rather than the MoveToPose it is built from: the detail of the MotionTimeout fault the base raises, and the scheduler's CompletedMotion / run result line. Without it a strafe failure would be reported under the wrong primitive. It does NOT reach the DebugRecord stream — a record identifies its motion only by the scheduler-assigned activeCommandId, so filtering a blackbox for "StrafeTo" finds nothing.

*function, declared at [`include/shulib/motion/strafe_to.hpp:50`](../../include/shulib/motion/strafe_to.hpp#L50).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 StrafeTo — translate to a FIELD (x, y) while HOLDING heading (chunk C1).

 The same decoupled three-axis engine as MoveToPose (one pipeline, one set of
 fixes) with the heading target CAPTURED AT THE FIRST LIVE TICK — the heading
 the robot actually has once the estimate exists, per the wait-for-live
 contract (motion.hpp): capturing at start() during the boot window would lock
 onto calibration garbage. The heading loop then actively HOLDS that capture
 throughout the translation (a pure strafe that lets heading wander is a
 decoupling failure, pinned by test).

 Drivetrain honesty: on a drive with strafeAuthority() == 0 (tank) the lateral
 component of the body command is clamped to zero by the engine's authority
 clamp — the motion physically cannot reach a laterally-offset target and
 exits TimedOut via the watchdog rather than hanging or lying. On the X-drive
 (authority 1.0) it strafes freely; C3's H-drive lands between.
```

</details>
