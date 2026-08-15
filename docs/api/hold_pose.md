<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/hold_pose.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `hold_pose.hpp`

HoldPose — actively hold a FIELD pose against disturbance.

This header declares **1** type (3 members).

Extracted from [`include/shulib/motion/hold_pose.hpp`](../../include/shulib/motion/hold_pose.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class HoldPose`](#class-holdpose)
  - [`HoldPose`](#holdpose-holdpose)
  - [`HoldPose (overload 2)`](#holdpose-holdpose-2)
  - [`name`](#holdpose-name)

<a id="class-holdpose"></a>

## `class HoldPose`

```cpp
class HoldPose final : public MoveToPose
```

Actively hold a FIELD pose against disturbance — a shove, a defender, field contact — by running MoveToPose's three decoupled loops in HOLD mode. It NEVER exits early: the hold window always runs to the end, and only then does being inside the settle tolerances decide the verdict — Settled if the robot is on target AT THAT MOMENT, TimedOut (with MOTION_TIMEOUT raised) if it was pushed away and never recovered, because holding the clock out while 10 inches off is not success. The window is measured from the FIRST LIVE tick, not from start(). Distinct from IMotor BrakeMode::Hold, a per-motor firmware hold with no pose in it; DriveBrake is the open-loop stop.

*class, declared at [`include/shulib/motion/hold_pose.hpp:36`](../../include/shulib/motion/hold_pose.hpp#L36).*

<a id="holdpose-holdpose"></a>

### `HoldPose::HoldPose`

```cpp
HoldPose(const MotionDeps& deps, double holdFor, const MotionConfig& config = {})
```

Hold the pose the robot has at the first live tick, for `holdFor` seconds.

*function, declared at [`include/shulib/motion/hold_pose.hpp:39`](../../include/shulib/motion/hold_pose.hpp#L39).*

<a id="holdpose-holdpose-2"></a>

### `HoldPose::HoldPose (overload 2)`

```cpp
HoldPose(const MotionDeps& deps, const math::Pose2d& pose, double holdFor, const MotionConfig& config = {})
```

Hold an explicit FIELD pose for `holdFor` seconds.

*function, declared at [`include/shulib/motion/hold_pose.hpp:47`](../../include/shulib/motion/hold_pose.hpp#L47).*

<a id="holdpose-name"></a>

### `HoldPose::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The literal "HoldPose" — overriding the inherited "MoveToPose" so a result line or a MOTION_TIMEOUT fault names the hold that failed, not the engine it borrows.

*function, declared at [`include/shulib/motion/hold_pose.hpp:56`](../../include/shulib/motion/hold_pose.hpp#L56).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 21 lines</summary>

```text

 HoldPose — actively hold a FIELD pose against disturbance (chunk C1).

 The same decoupled three-axis engine as MoveToPose in HOLD mode: it never
 settle-exits early — it keeps closing all three loops for `holdFor` seconds,
 driving back any disturbance (a shove, a defender, field contact), then
 reports:
   * Settled  — the hold window ended AND the robot is currently within the
                settle tolerances (it held its ground);
   * TimedOut — the window ended OFF target (pushed away and not recovered) —
                "held the clock out while 10 inches off" must never read as
                success. MOTION_TIMEOUT is raised, same as any timeout.

 Two constructions:
   * capture-current (the common one): the pose captured at the FIRST LIVE
     tick (wait-for-live contract, motion.hpp) — "stay where you are".
   * explicit pose: "hold THIS spot" (e.g. a scoring alignment).

 This is closed-loop position holding, distinct from IMotor BrakeMode::Hold
 (a per-motor firmware hold): HoldPose recovers a POSE, using the full
 holonomic authority of the drive. DriveBrake is the open-loop stop.
```

</details>
