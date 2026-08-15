<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/command_pipeline.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `command_pipeline.hpp`

applyCommandPipeline — the ONE command path from a chassis-speeds demand to energized motors.

This header declares **1** type (2 members), **1** free function, and **1** constant.

Extracted from [`include/shulib/motion/command_pipeline.hpp`](../../include/shulib/motion/command_pipeline.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kStrafeFallbackNoiseFraction`](#kstrafefallbacknoisefraction) — *constant*
- [`struct CommandOutcome`](#struct-commandoutcome)
  - [`body`](#commandoutcome-body)
  - [`strafeFallback`](#commandoutcome-strafefallback)
- [`applyCommandPipeline`](#applycommandpipeline) — *free function*

<a id="kstrafefallbacknoisefraction"></a>

## `kStrafeFallbackNoiseFraction`

```cpp
inline constexpr double kStrafeFallbackNoiseFraction = 0.01
```

strafeFallbackActive's legibility floor, as a fraction of maxLinearSpeed: the authority clamp must be removing more than this much lateral speed before a tick is flagged as fallback. The floor exists so sub-perceptible PID chatter near settle (or on tank, where the limit is 0) cannot light the flag on every tick — a permanently-on flag is as undebuggable as a silent one. At the HA-50 default budget this is 0.6 in/s — far below any deliberate strafe, far above near-settle chatter. Telemetry-legibility constant, host-decidable — not an A4 register entry (register rule 1). (Moved here from MoveToPose at C4, unchanged, when the pipeline was extracted — the flag is computed where the clamp is applied.)

*constant, declared at [`include/shulib/motion/command_pipeline.hpp:82`](../../include/shulib/motion/command_pipeline.hpp#L82).*

<a id="struct-commandoutcome"></a>

## `struct CommandOutcome`

```cpp
struct CommandOutcome
```

What the pipeline commanded, for the caller's record.

*struct, declared at [`include/shulib/motion/command_pipeline.hpp:85`](../../include/shulib/motion/command_pipeline.hpp#L85).*

<a id="commandoutcome-body"></a>

### `CommandOutcome::body`

```cpp
math::ChassisSpeeds body{}
```

The final achievable command in the BODY frame (post every clamp) — exactly what went into toWheels(). Record it via robotToField().

*field, declared at [`include/shulib/motion/command_pipeline.hpp:88`](../../include/shulib/motion/command_pipeline.hpp#L88).*

<a id="commandoutcome-strafefallback"></a>

### `CommandOutcome::strafeFallback`

```cpp
bool strafeFallback = false
```

True iff the strafe-authority clamp bound meaningfully this call (the C3 fallback contract — telemetry-visible, never silent).

*field, declared at [`include/shulib/motion/command_pipeline.hpp:91`](../../include/shulib/motion/command_pipeline.hpp#L91).*

<a id="applycommandpipeline"></a>

## `applyCommandPipeline`

```cpp
[[nodiscard]] inline CommandOutcome applyCommandPipeline(const MotionDeps& deps, const MotionConfig& cfg, const control::Feedforward& ff, const math::ChassisSpeeds& command, math::Frame frame, math::Angle heading)
```

Run the full choreography above and command the motors. `command` is expressed in `frame`; `heading` is the robot's current estimated heading (used only for the Field→Body rotation — pass the pose the caller already read this tick, so the whole tick acts on ONE snapshot).

*free function, declared at [`include/shulib/motion/command_pipeline.hpp:98`](../../include/shulib/motion/command_pipeline.hpp#L98).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

```text

 applyCommandPipeline — the ONE command path from a chassis-speeds demand to
 energized motors (chunk C4; extracted verbatim from MoveToPose's tick, where
 chunk C1 first assembled it).

 ── Why this is one function in one place ───────────────────────────────────────────
 C1's Freeze-Register note named the saturation choreography "the de-facto
 command path the facade's drive(ChassisSpeeds, Frame) verb must reuse, not
 re-derive — one pipeline, one place." Before C4 that choreography lived
 inside MoveToPose::tick() (with a degenerate copy in TurnTo); the facade
 would have needed a THIRD copy, and three copies of a clamp order is how a
 clamp order silently diverges. So C4 extracted it here — the motions and the
 facade now share one definition, and a bug fixed in the pipeline is fixed
 for every caller at once. (The C2 bit-identity suites — scheduled ==
 hand-chained, clean and hostile — pin that this extraction changed nothing:
 the arithmetic and its order are exactly C1's.)

 ── The choreography (MotionConfig header's saturation policy, F1/F5) ───────────────
   1. |ω| clamped to maxAngularSpeed (scalar; a rate is frame-invariant).
   2. (vx, vy) NORM-capped to maxLinearSpeed — uniformly, so the commanded
      direction is preserved. The Euclidean norm is rotation-invariant, so
      capping before or after the frame rotation is the same operation; it
      happens here, once, for both input frames.
   3. Frame::Field input only: math::fieldToRobot — THE one F1 rotation.
      Frame::Body input skips it (already body-frame; rotating it would BE
      the frame bug this parameter exists to prevent).
   4. body |vy| clamped to strafeAuthority()·maxLinearSpeed — the upstream
      clamp §13 #5 assigns to the MOTION layer; kinematics never clamps (F5).
      The strafeFallback flag reports when the clamp BOUND meaningfully
      (> kStrafeFallbackNoiseFraction·maxLinearSpeed removed — the C3
      telemetry-visibility contract; rationale at the constant below).
   5. IKinematics::toWheels — pure, unclamped inverse kinematics.
   6. IKinematics::desaturate(maxWheelSpeed) — the downstream uniform scale.
   7. Feedforward → compensateForBattery → IMotor::setVoltage, per wheel.

 ── The D-5 self-audit (chunk C5; diag/plausibility_guard.hpp carries the why) ──────
 After the clamps, the pipeline AUDITS its own output: the final body command
 must sit inside the configured budgets (invariant 2) and every wheel volt must
 be finite and inside the battery ceiling (invariant 3). On a healthy pipeline
 the audit is a pure pass-through — values are untouched, bit-identically (the
 C2 equivalence suites re-pin this) — so the audit costs a few compares per
 tick and changes nothing. It exists for the pipeline REGRESSION no closed-loop
 test can see (C4's M21: a defeated clamp left every accuracy suite green):
 in the field, that regression now raises IMPLAUSIBLE and the volts are
 clamped/zeroed before they reach a motor, instead of quietly over-driving.

 The function COMMANDS THE MOTORS (step 7) — it is the pipeline, not a
 planner — and returns what it commanded so the caller can record it (the
 DebugRecord `commanded` field carries the final achievable command in the
 FIELD frame; callers convert via robotToField). It does NOT emit records,
 tick health, or evaluate exits: those stay with the caller, because the
 motion primitives and the facade's drive() legitimately differ there.

 Preconditions are the callers' (MotionConfig::validate, finite inputs):
 this function is the hot path and adds none of its own.
```

</details>
