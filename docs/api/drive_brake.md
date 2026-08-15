<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/drive_brake.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_brake.hpp`

DriveBrake — stop the drivetrain and confirm it stopped.

This header declares **1** type (8 members).

Extracted from [`include/shulib/motion/drive_brake.hpp`](../../include/shulib/motion/drive_brake.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class DriveBrake`](#class-drivebrake)
  - [`kTwistAvgTicks`](#drivebrake-ktwistavgticks)
  - [`DriveBrake`](#drivebrake-drivebrake)
  - [`start`](#drivebrake-start)
  - [`tick`](#drivebrake-tick)
  - [`cancel`](#drivebrake-cancel)
  - [`exitReason`](#drivebrake-exitreason)
  - [`state`](#drivebrake-state)
  - [`name`](#drivebrake-name)

<a id="class-drivebrake"></a>

## `class DriveBrake`

```cpp
class DriveBrake final : public IMotion
```

The OPEN-LOOP stop: 0 V under BrakeMode::Brake on every drive motor, every tick, plus a verdict that the drivetrain actually came to rest. The ONE primitive exempt from the wait-for-live-estimate gate (motion.hpp) — zero output is the safe action in every state, boot window included — so the command lands immediately and only the SETTLE verdict depends on the estimator. Settled therefore means "the estimate can certify rest", which during the boot window can read stopped for a robot being pushed (header). Nothing is commanded, so no stall/health cross-check runs here. HoldPose is the closed-loop counterpart: it recovers a pose, this one only stops.

*class, declared at [`include/shulib/motion/drive_brake.hpp:63`](../../include/shulib/motion/drive_brake.hpp#L63).*

<a id="drivebrake-ktwistavgticks"></a>

### `DriveBrake::kTwistAvgTicks`

```cpp
static constexpr int kTwistAvgTicks = 5
```

Twist-averaging window, ticks (header note). At 100 Hz: 50 ms.

*field, declared at [`include/shulib/motion/drive_brake.hpp:66`](../../include/shulib/motion/drive_brake.hpp#L66).*

<a id="drivebrake-drivebrake"></a>

### `DriveBrake::DriveBrake`

```cpp
DriveBrake(const MotionDeps& deps, const MotionConfig& config = {}, double timeout = 0.0)
```

`timeout` (s) bounds the whole stop; 0 selects config.defaultTimeout.

*function, declared at [`include/shulib/motion/drive_brake.hpp:68`](../../include/shulib/motion/drive_brake.hpp#L68).*

<a id="drivebrake-start"></a>

### `DriveBrake::start`

```cpp
void start() override
```

Arm: reset the settle window, restart the watchdog, and empty the twist-averaging ring so a re-run never averages across the previous stop. Goes straight to MotionState::Running — this is the one primitive that never reports WaitingForEstimate. Re-callable: a finished brake re-arms completely.

*function, declared at [`include/shulib/motion/drive_brake.hpp:82`](../../include/shulib/motion/drive_brake.hpp#L82).*

<a id="drivebrake-tick"></a>

### `DriveBrake::tick`

```cpp
[[nodiscard]] control::ExitReason tick() override
```

One tick. Re-commands Brake + 0 V (idempotent), then judges the estimated speed norm |v| + rotationRadius·|ω| — in/s, folding rotation into linear currency so a spinning robot is not "stopped" — against config.brakeSettle. The norm is built from the kTwistAvgTicks-tick VECTOR average of the localizer's twist, not the raw one, so the first few ticks after start() average over fewer samples. TimedOut also raises FaultCode::MotionTimeout. Precondition: start() was called. Once a verdict is reached, later calls return it and command nothing further, and emit no record — the motors were already left at 0 V.

*function, declared at [`include/shulib/motion/drive_brake.hpp:100`](../../include/shulib/motion/drive_brake.hpp#L100).*

<a id="drivebrake-cancel"></a>

### `DriveBrake::cancel`

```cpp
void cancel() override
```

The cancel contract (motion.hpp). Cancelling a brake changes nothing physical — the safe state (0 V + Brake) is exactly what every tick was already commanding — but the verdict discipline is identical to every other motion: a running brake ends Cancelled (it never CERTIFIED the stop), a settled/timed-out one keeps its verdict.

*function, declared at [`include/shulib/motion/drive_brake.hpp:160`](../../include/shulib/motion/drive_brake.hpp#L160).*

<a id="drivebrake-exitreason"></a>

### `DriveBrake::exitReason`

```cpp
[[nodiscard]] control::ExitReason exitReason() const noexcept override
```

The verdict of the last tick(): Running until the AVERAGED speed norm settles, then Settled / TimedOut / Cancelled, held unchanged from then on. Running before the first tick(). Settled is a claim about what the estimate can certify, not about ground-truth stillness (header).

*function, declared at [`include/shulib/motion/drive_brake.hpp:177`](../../include/shulib/motion/drive_brake.hpp#L177).*

<a id="drivebrake-state"></a>

### `DriveBrake::state`

```cpp
[[nodiscard]] MotionState state() const noexcept override
```

Idle before start(); Running from start() onward — WaitingForEstimate is the one MotionState this primitive never reports — then Settled / TimedOut / Cancelled. This is the value the record stream carries in DebugRecord.activeCommandState.

*function, declared at [`include/shulib/motion/drive_brake.hpp:182`](../../include/shulib/motion/drive_brake.hpp#L182).*

<a id="drivebrake-name"></a>

### `DriveBrake::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The literal "DriveBrake" — what result lines and the MOTION_TIMEOUT fault name this motion, so a stop that never certified is distinguishable from a move that failed.

*function, declared at [`include/shulib/motion/drive_brake.hpp:186`](../../include/shulib/motion/drive_brake.hpp#L186).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 39 lines</summary>

```text

 DriveBrake — stop the drivetrain and confirm it stopped (chunk C1).

 Commands ZERO volts to every drive motor each ACTIVE tick (with BrakeMode::Brake so
 real hardware resists rather than coasts — the A2 plant does not model brake
 modes, documented limitation) and exits Settled once the ESTIMATED speed
 norm |v| + rotationRadius·|ω| has stayed inside brakeSettle for its held
 time; the watchdog bounds the wait (a robot that never reads stopped — e.g.
 a dead estimator — exits TimedOut rather than hanging).

 ── The estimator's twist noise floor (measured at C1, why the averaging) ───────────
 The M2 Localizer's twist is a RAW finite difference of the fused position; at
 a physical dead stop under A3's composed hostility it reads 0.5–1.5 in/s of
 noise (heading noise ≈ HA-21 through the tracking-offset correction path,
 differentiated at 100 Hz). A settle threshold below that floor can never
 certify "stopped". So the speed norm is built from a kTwistAvgTicks-tick
 VECTOR average (≈√n noise reduction, ~50 ms verdict delay), and the
 brakeSettle default sits above the averaged floor (HA-51). TRUE stopped-ness
 is pinned against plant ground truth by test; the verdict here is what the
 ESTIMATE can honestly certify at M2.

 Boot-window verdict deferral, documented: during IMU calibration the yaw-rate
 stream is garbage (±10 rad/s class), so a brake issued mid-boot holds its
 zero-volt command IMMEDIATELY but cannot certify "stopped" until the sensor
 goes live (~calibration end). Action now, verdict when provable.

 ── EXEMPT from the wait-for-live gate, BY DESIGN (motion.hpp) ──────────────────────
 Zero output is the SAFE action in every state, including the boot window —
 gating "stop" behind an estimate would be absurd (and the guaranteed
 end-of-run park must be able to kill the drive unconditionally). The cost,
 documented honestly: during Uninitialized the settle verdict reads the
 Localizer's boot-frozen twist (reports ~0), so a robot being PUSHED during
 calibration can read "stopped" while moving. With zero volts commanded there
 is no better observable at M2; the verdict is best-available, not an
 accuracy claim.

 No stall/health wiring here: nothing is commanded, so the spin-vs-motion
 cross-check has no spin to compare (and a false ODO_STUCK from a coasting
 wheel would be noise). The record stream still runs (A1 contract).
```

</details>
