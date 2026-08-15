<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/manipulation/mechanism_op.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism_op.hpp`

The bounded mechanism operation (chunk F1, WS7/M4): IMechanismOp + the two season-free operations every scoring verb decomposes into — run a motor mechanism until something confirms (RunUntilConfirmed) and fire a discrete actuator, wait for it to physically…

This header declares **7** types (49 members).

Extracted from [`include/shulib/manipulation/mechanism_op.hpp`](../../include/shulib/manipulation/mechanism_op.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct MechanismDeps`](#struct-mechanismdeps)
  - [`clock`](#mechanismdeps-clock)
  - [`faults`](#mechanismdeps-faults)
  - [`telemetry`](#mechanismdeps-telemetry)
  - [`validate`](#mechanismdeps-validate)
  - [`validatedClock`](#mechanismdeps-validatedclock)
- [`class IMechanismOp`](#class-imechanismop)
  - [`~IMechanismOp`](#imechanismop-destructor-imechanismop)
  - [`IMechanismOp`](#imechanismop-imechanismop)
  - [`IMechanismOp (overload 2)`](#imechanismop-imechanismop-2)
  - [`IMechanismOp (overload 3)`](#imechanismop-imechanismop-3)
  - [`operator=`](#imechanismop-operator-eq)
  - [`operator= (overload 2)`](#imechanismop-operator-eq-2)
  - [`start`](#imechanismop-start)
  - [`tick`](#imechanismop-tick)
  - [`cancel`](#imechanismop-cancel)
  - [`outcome`](#imechanismop-outcome)
  - [`started`](#imechanismop-started)
  - [`name`](#imechanismop-name)
  - [`finished`](#imechanismop-finished)
- [`struct AlwaysConfirmed`](#struct-alwaysconfirmed)
  - [`operator()`](#alwaysconfirmed-operator-call)
- [`struct RunUntilConfirmedConfig`](#struct-rununtilconfirmedconfig)
  - [`voltage`](#rununtilconfirmedconfig-voltage)
  - [`timeout`](#rununtilconfirmedconfig-timeout)
  - [`stall`](#rununtilconfirmedconfig-stall)
- [`class RunUntilConfirmed`](#class-rununtilconfirmed)
  - [`RunUntilConfirmed`](#rununtilconfirmed-rununtilconfirmed)
  - [`~RunUntilConfirmed`](#rununtilconfirmed-destructor-rununtilconfirmed)
  - [`RunUntilConfirmed (overload 2)`](#rununtilconfirmed-rununtilconfirmed-2)
  - [`RunUntilConfirmed (overload 3)`](#rununtilconfirmed-rununtilconfirmed-3)
  - [`operator=`](#rununtilconfirmed-operator-eq)
  - [`operator= (overload 2)`](#rununtilconfirmed-operator-eq-2)
  - [`start`](#rununtilconfirmed-start)
  - [`tick`](#rununtilconfirmed-tick)
  - [`cancel`](#rununtilconfirmed-cancel)
  - [`outcome`](#rununtilconfirmed-outcome)
  - [`started`](#rununtilconfirmed-started)
  - [`name`](#rununtilconfirmed-name)
- [`struct ActuateAndConfirmConfig`](#struct-actuateandconfirmconfig)
  - [`target`](#actuateandconfirmconfig-target)
  - [`actuationTime`](#actuateandconfirmconfig-actuationtime)
  - [`confirmWindow`](#actuateandconfirmconfig-confirmwindow)
- [`class ActuateAndConfirm`](#class-actuateandconfirm)
  - [`ActuateAndConfirm`](#actuateandconfirm-actuateandconfirm)
  - [`~ActuateAndConfirm`](#actuateandconfirm-destructor-actuateandconfirm)
  - [`ActuateAndConfirm (overload 2)`](#actuateandconfirm-actuateandconfirm-2)
  - [`ActuateAndConfirm (overload 3)`](#actuateandconfirm-actuateandconfirm-3)
  - [`operator=`](#actuateandconfirm-operator-eq)
  - [`operator= (overload 2)`](#actuateandconfirm-operator-eq-2)
  - [`start`](#actuateandconfirm-start)
  - [`tick`](#actuateandconfirm-tick)
  - [`cancel`](#actuateandconfirm-cancel)
  - [`outcome`](#actuateandconfirm-outcome)
  - [`started`](#actuateandconfirm-started)
  - [`name`](#actuateandconfirm-name)

<a id="struct-mechanismdeps"></a>

## `struct MechanismDeps`

```cpp
struct MechanismDeps
```

The dependencies every mechanism operation shares, as NAMED pointers (designated initializers at the call site), validated non-null — the MotionDeps pattern, not a third convention. Deliberately SMALLER than MotionDeps: no localizer (no estimate is read), no kinematics (nothing to desaturate), no HealthMonitor (the loop owner ticks it). All pointees must outlive the operation.

*struct, declared at [`include/shulib/manipulation/mechanism_op.hpp:152`](../../include/shulib/manipulation/mechanism_op.hpp#L152).*

<a id="mechanismdeps-clock"></a>

### `MechanismDeps::clock`

```cpp
hal::IClock* clock = nullptr
```

watchdog / deadlines / stall window

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:153`](../../include/shulib/manipulation/mechanism_op.hpp#L153).*

<a id="mechanismdeps-faults"></a>

### `MechanismDeps::faults`

```cpp
diag::FaultLatch* faults = nullptr
```

run-scoped latch (MechanismStalled)

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:154`](../../include/shulib/manipulation/mechanism_op.hpp#L154).*

<a id="mechanismdeps-telemetry"></a>

### `MechanismDeps::telemetry`

```cpp
hal::ITelemetrySink* telemetry = nullptr
```

the Warn lines (TimedOut/Unconfirmed)

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:155`](../../include/shulib/manipulation/mechanism_op.hpp#L155).*

<a id="mechanismdeps-validate"></a>

### `MechanismDeps::validate`

```cpp
void validate() const
```

Every dependency non-null, or a loud precondition naming the one that is not. Each operation calls it at CONSTRUCTION, so a forgotten pointer is an error at the wiring site rather than a null dereference on the first tick.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:160`](../../include/shulib/manipulation/mechanism_op.hpp#L160).*

<a id="mechanismdeps-validatedclock"></a>

### `MechanismDeps::validatedClock`

```cpp
[[nodiscard]] hal::IClock& validatedClock() const
```

validate(), then hand out the clock — for a member-initializer list's FIRST dependency use (MotionDeps' own trick), so a null pointer trips the precondition instead of being dereferenced.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:169`](../../include/shulib/manipulation/mechanism_op.hpp#L169).*

<a id="class-imechanismop"></a>

## `class IMechanismOp`

```cpp
class IMechanismOp : public hal::ICancellable
```

The operation contract (file banner). Pure-virtual cancel() ON PURPOSE, exactly as IMotion rules: an operation type without a cancellation story is the forgettable-safety-step failure mode; every implementer must state one.  Since F2 the contract IS a hal::ICancellable: an operation registers itself on the mechanism's claim at start() (tryClaim(*this)), which is how the end-of-run guard — holding only span<hal::IMechanism*> — reaches a live operation and renders it inert at the deadline (mechanism.hpp's claimant- hook banner carries the measured failure this closes). cancel()'s meaning is unchanged; the base only makes it reachable from the hal tier.

*class, declared at [`include/shulib/manipulation/mechanism_op.hpp:185`](../../include/shulib/manipulation/mechanism_op.hpp#L185).*

<a id="imechanismop-destructor-imechanismop"></a>

### `IMechanismOp::~IMechanismOp`

```cpp
~IMechanismOp() override = default
```

Interface plumbing: the destructor is virtual through hal::ICancellable, so an operation may be owned and destroyed through either base, and the rest are restated because declaring a destructor suppresses the implicit moves. Defaulting them here is base-class boilerplate, NOT permission to copy an operation — a concrete operation holds a mechanism claim and a claimant registration pointing at itself, so both library operations delete all four.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:193`](../../include/shulib/manipulation/mechanism_op.hpp#L193).*

<a id="imechanismop-imechanismop"></a>

### `IMechanismOp::IMechanismOp`

```cpp
IMechanismOp() = default
```

*Covered by the comment on [`~IMechanismOp`](#imechanismop-destructor-imechanismop) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:194`](../../include/shulib/manipulation/mechanism_op.hpp#L194).*

<a id="imechanismop-imechanismop-2"></a>

### `IMechanismOp::IMechanismOp (overload 2)`

```cpp
IMechanismOp(const IMechanismOp&) = default
```

*Covered by the comment on [`~IMechanismOp`](#imechanismop-destructor-imechanismop) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:195`](../../include/shulib/manipulation/mechanism_op.hpp#L195).*

<a id="imechanismop-imechanismop-3"></a>

### `IMechanismOp::IMechanismOp (overload 3)`

```cpp
IMechanismOp(IMechanismOp&&) = default
```

*Covered by the comment on [`~IMechanismOp`](#imechanismop-destructor-imechanismop) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:196`](../../include/shulib/manipulation/mechanism_op.hpp#L196).*

<a id="imechanismop-operator-eq"></a>

### `IMechanismOp::operator=`

```cpp
IMechanismOp& operator=(const IMechanismOp&) = default
```

*Covered by the comment on [`~IMechanismOp`](#imechanismop-destructor-imechanismop) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:197`](../../include/shulib/manipulation/mechanism_op.hpp#L197).*

<a id="imechanismop-operator-eq-2"></a>

### `IMechanismOp::operator= (overload 2)`

```cpp
IMechanismOp& operator=(IMechanismOp&&) = default
```

*Covered by the comment on [`~IMechanismOp`](#imechanismop-destructor-imechanismop) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:198`](../../include/shulib/manipulation/mechanism_op.hpp#L198).*

<a id="imechanismop-start"></a>

### `IMechanismOp::start`

```cpp
virtual void start() = 0
```

Arm the operation: claim the mechanism (loud precondition if another operation holds it), reset detectors, arm the watchdog/deadlines. Re-callable — a finished operation re-arms completely.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:203`](../../include/shulib/manipulation/mechanism_op.hpp#L203).*

<a id="imechanismop-tick"></a>

### `IMechanismOp::tick`

```cpp
[[nodiscard]] virtual MechanismOutcome tick() = 0
```

One step (see the tick contract). Precondition: start() has been called.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:206`](../../include/shulib/manipulation/mechanism_op.hpp#L206).*

<a id="imechanismop-cancel"></a>

### `IMechanismOp::cancel`

```cpp
void cancel() override = 0
```

Stop from outside (see the cancel contract). Idempotent; never raises; applies the mechanism's declared safe state whenever started. Overrides hal::ICancellable — this is the member the claimant hook exposes.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:211`](../../include/shulib/manipulation/mechanism_op.hpp#L211).*

<a id="imechanismop-outcome"></a>

### `IMechanismOp::outcome`

```cpp
[[nodiscard]] virtual MechanismOutcome outcome() const noexcept = 0
```

The verdict of the most recent tick (Running before the first, and before start()).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:215`](../../include/shulib/manipulation/mechanism_op.hpp#L215).*

<a id="imechanismop-started"></a>

### `IMechanismOp::started`

```cpp
[[nodiscard]] virtual bool started() const noexcept = 0
```

True once start() has been called (stays true after an exit).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:218`](../../include/shulib/manipulation/mechanism_op.hpp#L218).*

<a id="imechanismop-name"></a>

### `IMechanismOp::name`

```cpp
[[nodiscard]] virtual const char* name() const noexcept = 0
```

Stable short name for logs / step labels (e.g. "grab").

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:221`](../../include/shulib/manipulation/mechanism_op.hpp#L221).*

<a id="imechanismop-finished"></a>

### `IMechanismOp::finished`

```cpp
[[nodiscard]] bool finished() const noexcept
```

Convenience: exited with a real verdict (defined from the virtuals so it can never disagree with them).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:225`](../../include/shulib/manipulation/mechanism_op.hpp#L225).*

<a id="struct-alwaysconfirmed"></a>

## `struct AlwaysConfirmed`

```cpp
struct AlwaysConfirmed
```

A caller-supplied confirmation that always holds: "completion IS the confirmation" — for fire-and-forget actions (deploy an actuator) where no sensor exists to ask. Using it is a visible, greppable declaration that an action is unverified, which is exactly what an invisible default would hide.

*struct, declared at [`include/shulib/manipulation/mechanism_op.hpp:234`](../../include/shulib/manipulation/mechanism_op.hpp#L234).*

<a id="alwaysconfirmed-operator-call"></a>

### `AlwaysConfirmed::operator()`

```cpp
[[nodiscard]] bool operator()() const noexcept
```

Always true, on every call. In ActuateAndConfirm that means "confirmed the instant the actuation deadline passes"; in RunUntilConfirmed it means the FIRST tick Succeeds with zero volts ever commanded — which is why this belongs on discrete actuators and is a mistake on a motor operation.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:239`](../../include/shulib/manipulation/mechanism_op.hpp#L239).*

<a id="struct-rununtilconfirmedconfig"></a>

## `struct RunUntilConfirmedConfig`

```cpp
struct RunUntilConfirmedConfig
```

RunUntilConfirmed's three numbers, all REQUIRED and all validated at construction — there is no library default for a voltage, a budget or a jam threshold, because every one of them is a fact about one mechanism on one robot. Zero-initializing this struct does not get you a working operation; it gets you a loud precondition.

*struct, declared at [`include/shulib/manipulation/mechanism_op.hpp:247`](../../include/shulib/manipulation/mechanism_op.hpp#L247).*

<a id="rununtilconfirmedconfig-voltage"></a>

### `RunUntilConfirmedConfig::voltage`

```cpp
units::Voltage voltage
```

Commanded while running (finite, non-zero — a 0 V "run" is a nonsense request, refused loudly at construction).

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:250`](../../include/shulib/manipulation/mechanism_op.hpp#L250).*

<a id="rununtilconfirmedconfig-timeout"></a>

### `RunUntilConfirmedConfig::timeout`

```cpp
units::Time timeout
```

Watchdog budget (finite, > 0). Typed time, the D2 discipline.

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:252`](../../include/shulib/manipulation/mechanism_op.hpp#L252).*

<a id="rununtilconfirmedconfig-stall"></a>

### `RunUntilConfirmedConfig::stall`

```cpp
StallConfig stall
```

Jam/stall thresholds — REQUIRED, per mechanism, no library defaults (stall_detector.hpp says why).

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:255`](../../include/shulib/manipulation/mechanism_op.hpp#L255).*

<a id="class-rununtilconfirmed"></a>

## `class RunUntilConfirmed`

```cpp
template <typename Confirm> class RunUntilConfirmed final : public IMechanismOp
```

Run a motor mechanism at a fixed voltage until a caller-supplied confirmation holds. The season-free skeleton of intakeUntilCapture and every "spin until the sensor says so" verb. Possible verdicts: Succeeded / Stalled / TimedOut / Cancelled. `Unconfirmed` is UNREACHABLE here by construction and that is honest: an open-ended run has no notion of "the action completed" separate from its confirmation, so a world that never confirms is a timeout, not a completed-but-unconfirmed act (contrast ActuateAndConfirm, where actuation completing is a fact of its own).  WHAT CONFIRMS is deliberately a caller-supplied predicate: hue for a Toggle, proximity for a cup, a current signature for a capture — that meaning is season- and robot-specific and belongs above this layer (hal/vision.hpp's opaque-classId house rule, applied to actions). The predicate is TRUSTED: a predicate that lies "confirmed" produces a false Succeeded, which is why F3's primitives must confirm on real sensors — the F1 hostile suite demonstrates the boundary rather than pretending to close it. Checked at the TOP of every tick, before any command (true on entry ⇒ Succeeded with zero volts ever commanded — already holding the ring must not spin the intake; the C2 pred-before-first-tick shape).

*class, declared at [`include/shulib/manipulation/mechanism_op.hpp:278`](../../include/shulib/manipulation/mechanism_op.hpp#L278).*

<a id="rununtilconfirmed-rununtilconfirmed"></a>

### `RunUntilConfirmed::RunUntilConfirmed`

```cpp
RunUntilConfirmed(hal::MotorMechanism& mech, const MechanismDeps& deps, const RunUntilConfirmedConfig& config, Confirm confirm, const char* opName = "runUntilConfirmed")
```

`mech` must outlive the operation; `opName` must be a stable literal.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:284`](../../include/shulib/manipulation/mechanism_op.hpp#L284).*

<a id="rununtilconfirmed-destructor-rununtilconfirmed"></a>

### `RunUntilConfirmed::~RunUntilConfirmed`

```cpp
~RunUntilConfirmed() override
```

F2 (rule-of-three, found building the end-of-run guard): an operation destroyed MID-FLIGHT — the timeout-mismatch idiom, where the caller's wait gives up before the op's own watchdog — previously left its mechanism CLAIMED FOREVER and ENERGIZED at the last commanded voltage: no code path stopped it, and the stuck claim made the end action's own operation throw at start(). Destruction now cancels a still-running operation (declared safe state, claim + claimant released). A FINISHED operation is deliberately untouched: cancel() would re-apply the safe state, and on a discrete actuator that un-does the completed action (the T4 persist rule).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:307`](../../include/shulib/manipulation/mechanism_op.hpp#L307).*

<a id="rununtilconfirmed-rununtilconfirmed-2"></a>

### `RunUntilConfirmed::RunUntilConfirmed (overload 2)`

```cpp
RunUntilConfirmed(const RunUntilConfirmed&) = delete
```

Non-copyable/non-movable (F2): the claim is a resource and the mechanism's registered claimant points at THIS object — a copy would double-release the claim and a move would leave the registration dangling. Construct where it will live; start() re-arms for retries.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:317`](../../include/shulib/manipulation/mechanism_op.hpp#L317).*

<a id="rununtilconfirmed-rununtilconfirmed-3"></a>

### `RunUntilConfirmed::RunUntilConfirmed (overload 3)`

```cpp
RunUntilConfirmed(RunUntilConfirmed&&) = delete
```

*Covered by the comment on [`RunUntilConfirmed (overload 2)`](#rununtilconfirmed-rununtilconfirmed-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:318`](../../include/shulib/manipulation/mechanism_op.hpp#L318).*

<a id="rununtilconfirmed-operator-eq"></a>

### `RunUntilConfirmed::operator=`

```cpp
RunUntilConfirmed& operator=(const RunUntilConfirmed&) = delete
```

*Covered by the comment on [`RunUntilConfirmed (overload 2)`](#rununtilconfirmed-rununtilconfirmed-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:319`](../../include/shulib/manipulation/mechanism_op.hpp#L319).*

<a id="rununtilconfirmed-operator-eq-2"></a>

### `RunUntilConfirmed::operator= (overload 2)`

```cpp
RunUntilConfirmed& operator=(RunUntilConfirmed&&) = delete
```

*Covered by the comment on [`RunUntilConfirmed (overload 2)`](#rununtilconfirmed-rununtilconfirmed-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:320`](../../include/shulib/manipulation/mechanism_op.hpp#L320).*

<a id="rununtilconfirmed-start"></a>

### `RunUntilConfirmed::start`

```cpp
void start() override
```

Claim the mechanism and register this object as its claimant (loud precondition if another operation already holds it — cancel that one first), forget any partial stall window, and arm the watchdog HERE; no code path disarms it, so a world that never confirms still exits. Re-callable: a finished operation re-arms completely for a retry, and a claim this object already holds is kept rather than re-taken. Commands nothing by itself.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:328`](../../include/shulib/manipulation/mechanism_op.hpp#L328).*

<a id="rununtilconfirmed-tick"></a>

### `RunUntilConfirmed::tick`

```cpp
[[nodiscard]] MechanismOutcome tick() override
```

One step, in an order pinned by test: confirmation FIRST (success outranks a simultaneous stall or timeout), then the stall detector, then the watchdog; the configured voltage is commanded only when none of the three fired — so a confirmation already true on entry Succeeds with zero volts ever reaching the motors. EVERY exit applies the mechanism's declared safe state and releases the claim, so a caller can never be left owing a stop. Once finished it is a no-op returning the cached verdict. Precondition: start() has been called.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:349`](../../include/shulib/manipulation/mechanism_op.hpp#L349).*

<a id="rununtilconfirmed-cancel"></a>

### `RunUntilConfirmed::cancel`

```cpp
void cancel() override
```

Make the mechanism safe NOW: the declared safe state is applied on every call made after start(), idempotently, but the VERDICT is written only if the operation was still running — an operation that Succeeded stays Succeeded, because rewriting history would lie to whoever reads outcome() later. Before start() it is a complete no-op. Raises no fault: cancellation is a commanded, normal act. This is also what the end-of-run guard reaches through the mechanism's registered claimant.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:379`](../../include/shulib/manipulation/mechanism_op.hpp#L379).*

<a id="rununtilconfirmed-outcome"></a>

### `RunUntilConfirmed::outcome`

```cpp
[[nodiscard]] MechanismOutcome outcome() const noexcept override
```

The verdict of the most recent tick, cached: Running before the first tick and before start(), then frozen at whichever of Succeeded / Stalled / TimedOut / Cancelled ended the operation, until the next start() re-arms.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:406`](../../include/shulib/manipulation/mechanism_op.hpp#L406).*

<a id="rununtilconfirmed-started"></a>

### `RunUntilConfirmed::started`

```cpp
[[nodiscard]] bool started() const noexcept override
```

True from the first start() onward — including after an exit, and after a cancel(). Nothing clears it, so it answers "has this ever run?", not "is it running?" (pair it with outcome() != Running, or just call finished()).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:410`](../../include/shulib/manipulation/mechanism_op.hpp#L410).*

<a id="rununtilconfirmed-name"></a>

### `RunUntilConfirmed::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The `opName` handed to the constructor, returned unchanged and not copied. It is what the MechanismStalled fault detail and the timeout Warn line quote, so it is the string you grep a transcript for.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:414`](../../include/shulib/manipulation/mechanism_op.hpp#L414).*

<a id="struct-actuateandconfirmconfig"></a>

## `struct ActuateAndConfirmConfig`

```cpp
struct ActuateAndConfirmConfig
```

ActuateAndConfirm's schedule: WHAT to command, how long the hardware needs to do it, and how long afterwards proof may take to arrive. Both times are validated finite and >= 0 at construction and become ABSOLUTE instants at start() — they are this operation's watchdog, so no later path can extend them. The physical actuation time is a measured property of the cylinder, not a number the library can supply.

*struct, declared at [`include/shulib/manipulation/mechanism_op.hpp:482`](../../include/shulib/manipulation/mechanism_op.hpp#L482).*

<a id="actuateandconfirmconfig-target"></a>

### `ActuateAndConfirmConfig::target`

```cpp
bool target = true
```

The commanded line state (what it means physically is the mechanism's).

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:484`](../../include/shulib/manipulation/mechanism_op.hpp#L484).*

<a id="actuateandconfirmconfig-actuationtime"></a>

### `ActuateAndConfirmConfig::actuationTime`

```cpp
units::Time actuationTime
```

How long the actuation physically takes before the world can honestly be asked about it (finite, >= 0). The confirmation is NOT consulted before this deadline: a clamp's "closed" sensor may still be reporting the PREVIOUS grab, and confirming on the pre-actuation state is the silent- success door. Invented values are R4's to measure; register them (HA).

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:490`](../../include/shulib/manipulation/mechanism_op.hpp#L490).*

<a id="actuateandconfirmconfig-confirmwindow"></a>

### `ActuateAndConfirmConfig::confirmWindow`

```cpp
units::Time confirmWindow
```

How long after actuation the confirmation may take to arrive (finite, >= 0; 0 = one check exactly at the actuation deadline). Expiring with the confirmation still false is the Unconfirmed verdict.

*field, declared at [`include/shulib/manipulation/mechanism_op.hpp:494`](../../include/shulib/manipulation/mechanism_op.hpp#L494).*

<a id="class-actuateandconfirm"></a>

## `class ActuateAndConfirm`

```cpp
template <typename Confirm> class ActuateAndConfirm final : public IMechanismOp
```

Fire a discrete actuator, wait out its physical actuation time, then require a caller-supplied confirmation within a bounded window. The season-free skeleton of clampActuate+clampConfirm and deployActuator — and the operation that makes `Unconfirmed` REAL: the command completed (time passed, the solenoid was told), the mechanism is healthy, and the world reports the thing did not happen. Possible verdicts: Succeeded / Unconfirmed / Cancelled. `Stalled` is unreachable (no current channel exists on a solenoid — digital_out.hpp) and `TimedOut` is unreachable because the deadline pair IS the watchdog (file banner): both instants are fixed at start() and a monotonic clock reaches them. With AlwaysConfirmed, completion is the confirmation (deploy — visibly unverified by declaration).  ON EXIT (the T4 split, banner): Succeeded and Unconfirmed LEAVE THE COMMANDED STATE IN PLACE — the completed actuation persists (a successful grab must not be un-grabbed by its own success); only cancel() applies the declared safe state.

*class, declared at [`include/shulib/manipulation/mechanism_op.hpp:515`](../../include/shulib/manipulation/mechanism_op.hpp#L515).*

<a id="actuateandconfirm-actuateandconfirm"></a>

### `ActuateAndConfirm::ActuateAndConfirm`

```cpp
ActuateAndConfirm(hal::PneumaticMechanism& mech, const MechanismDeps& deps, const ActuateAndConfirmConfig& config, Confirm confirm, const char* opName = "actuateAndConfirm")
```

`mech` must outlive the operation; `opName` must be a stable literal.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:521`](../../include/shulib/manipulation/mechanism_op.hpp#L521).*

<a id="actuateandconfirm-destructor-actuateandconfirm"></a>

### `ActuateAndConfirm::~ActuateAndConfirm`

```cpp
~ActuateAndConfirm() override
```

Cancel-on-destruction for a MID-FLIGHT operation; a finished one is untouched (RunUntilConfirmed's destructor note — the T4 persist rule is the reason the guard is conditional, and it matters MOST here: an unconditional cancel would force the declared safe state onto every successfully-grabbed clamp the moment its operation went out of scope).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:540`](../../include/shulib/manipulation/mechanism_op.hpp#L540).*

<a id="actuateandconfirm-actuateandconfirm-2"></a>

### `ActuateAndConfirm::ActuateAndConfirm (overload 2)`

```cpp
ActuateAndConfirm(const ActuateAndConfirm&) = delete
```

Non-copyable/non-movable (F2): same claim-resource reasoning as RunUntilConfirmed.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:548`](../../include/shulib/manipulation/mechanism_op.hpp#L548).*

<a id="actuateandconfirm-actuateandconfirm-3"></a>

### `ActuateAndConfirm::ActuateAndConfirm (overload 3)`

```cpp
ActuateAndConfirm(ActuateAndConfirm&&) = delete
```

*Covered by the comment on [`ActuateAndConfirm (overload 2)`](#actuateandconfirm-actuateandconfirm-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:549`](../../include/shulib/manipulation/mechanism_op.hpp#L549).*

<a id="actuateandconfirm-operator-eq"></a>

### `ActuateAndConfirm::operator=`

```cpp
ActuateAndConfirm& operator=(const ActuateAndConfirm&) = delete
```

*Covered by the comment on [`ActuateAndConfirm (overload 2)`](#actuateandconfirm-actuateandconfirm-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:550`](../../include/shulib/manipulation/mechanism_op.hpp#L550).*

<a id="actuateandconfirm-operator-eq-2"></a>

### `ActuateAndConfirm::operator= (overload 2)`

```cpp
ActuateAndConfirm& operator=(ActuateAndConfirm&&) = delete
```

*Covered by the comment on [`ActuateAndConfirm (overload 2)`](#actuateandconfirm-actuateandconfirm-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:551`](../../include/shulib/manipulation/mechanism_op.hpp#L551).*

<a id="actuateandconfirm-start"></a>

### `ActuateAndConfirm::start`

```cpp
void start() override
```

Claim the mechanism and register this object as its claimant (loud precondition if another operation already holds it), then compute the deadline pair ONCE from the clock — actuation, then confirm. Those two absolute instants ARE this operation's bound: no later path extends them, so an erratic dt cannot stretch the budget and a clock jump cannot skip it. Re-callable; a finished operation re-arms for a retry. Commands nothing by itself — the first tick() is what fires the actuator.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:560`](../../include/shulib/manipulation/mechanism_op.hpp#L560).*

<a id="actuateandconfirm-tick"></a>

### `ActuateAndConfirm::tick`

```cpp
[[nodiscard]] MechanismOutcome tick() override
```

One step: re-command the target line state (idempotent — the solenoid holds it), then ask the confirmation ONLY past the actuation deadline. Before that instant the operation stays Running WITHOUT consulting the world, because a clamp's "closed" sensor may still be reporting the previous grab and confirming on the pre-actuation state is the silent-success door. On exit the COMMANDED STATE STAYS PUT for both Succeeded and Unconfirmed — a successful grab must not be un-grabbed by its own success, and an unconfirmed one is left where it is for the caller's retry/undo decision. The claim is released on every exit. Once finished it commands nothing. Precondition: start().

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:586`](../../include/shulib/manipulation/mechanism_op.hpp#L586).*

<a id="actuateandconfirm-cancel"></a>

### `ActuateAndConfirm::cancel`

```cpp
void cancel() override
```

The ONLY path that forces the declared safe state onto this actuator — every other exit leaves the commanded state in place. Applied on every call made after start(), idempotently, so it can and does un-do a completed actuation: that is deliberate, and it is how a team says "the clamp stays closed at the buzzer" or "the cylinder retracts inside the expansion limit" through the mechanism's declared safe value. The verdict of an already-finished operation is preserved; before start() it is a complete no-op; it raises no fault.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:616`](../../include/shulib/manipulation/mechanism_op.hpp#L616).*

<a id="actuateandconfirm-outcome"></a>

### `ActuateAndConfirm::outcome`

```cpp
[[nodiscard]] MechanismOutcome outcome() const noexcept override
```

The verdict of the most recent tick, cached: Running before the first tick and before start(), then frozen at Succeeded, Unconfirmed or Cancelled — Stalled and TimedOut are unreachable for this operation (no current channel on a solenoid; the deadline pair is the watchdog).

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:641`](../../include/shulib/manipulation/mechanism_op.hpp#L641).*

<a id="actuateandconfirm-started"></a>

### `ActuateAndConfirm::started`

```cpp
[[nodiscard]] bool started() const noexcept override
```

True from the first start() onward — including after an exit. Nothing clears it, so it answers "has this ever run?", not "is it running?".

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:644`](../../include/shulib/manipulation/mechanism_op.hpp#L644).*

<a id="actuateandconfirm-name"></a>

### `ActuateAndConfirm::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The `opName` handed to the constructor, returned unchanged and not copied. The Unconfirmed Warn line quotes it, so it is the string that identifies this actuation in a transcript.

*function, declared at [`include/shulib/manipulation/mechanism_op.hpp:648`](../../include/shulib/manipulation/mechanism_op.hpp#L648).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 126 lines, click to expand</summary>

```text

 The bounded mechanism operation (chunk F1, WS7/M4): IMechanismOp + the two
 season-free operations every scoring verb decomposes into — run a motor
 mechanism until something confirms (RunUntilConfirmed) and fire a discrete
 actuator, wait for it to physically happen, then check (ActuateAndConfirm).
 F3's concrete primitives (intakeUntilCapture, clampActuate+clampConfirm,
 deployActuator, …) are these two shapes with real sensors and real
 thresholds; F2's combinators interleave, pre-empt and time-bound them. This
 header is the contract both of those chunks are built against.

 The shape MIRRORS motion/motion.hpp — deliberately, so nobody maintains two
 near-identical contracts that quietly diverge — and every place the mirror
 STOPS is named here, because an undocumented divergence between two contracts
 this similar is a bug factory:

 ── The tick contract (identical to IMotion's) ──────────────────────────────────────
 An operation does NOT own a loop, a task, or a clock cadence. The loop owner
 ticks it: a caller's own loop, or — the documented idiom for blocking use —
 the scheduler's existing wait primitive, with the operation ticked from the
 predicate:

     op.start();
     chassis.waitUntil([&] { return op.tick() != MechanismOutcome::Running; },
                       units::Time{2.0});
     // op.outcome() is the verdict

 That one line IS the blocking form. There is deliberately NO blocking helper
 in this layer: the predicate idiom inherits every C2 guard for free (the
 required finite timeout, the stalled-pace precondition, the no-blocking-
 verbs-inside-a-predicate re-entrancy rule), it runs concurrently with an
 active motion (the scheduler keeps ticking it — "intake while driving"), and
 a second loop owner that could deadlock against the first NEVER EXISTS. A
 helper that owned its own pace loop would advance the world underneath a
 scheduler that thinks it is mid-predicate; rather than make that safe in
 every position a caller could put it, the tick form is the only form (the
 C2 stalled-pacer precedent: make the un-ownable loud or make it impossible).

 After a non-Running verdict the operation is FINISHED: further tick() calls
 are safe no-ops that return the cached verdict and command nothing (the
 mechanism is already in its safe state). start() fully re-arms — an
 operation object is reusable (retry a grab).

 ── The cancel contract (mirrored from motion.hpp, divergences named) ───────────────
 Identical in every clause:
   * RUNNING: the mechanism is put in its safe state synchronously, the
     verdict becomes Cancelled, the operation is FINISHED and inert.
   * Already FINISHED: the safe state is STILL applied (cancel() means "make
     it safe NOW" and must be idempotent) but the verdict is PRESERVED — an
     operation that succeeded really did succeed; rewriting history would lie
     to whoever reads the outcome later (C2's exact ruling for motions).
   * Never started: complete no-op — an unstarted operation has no
     relationship to its mechanism yet; commanding it would be the surprise.
   * cancel() raises NO fault: cancellation is a commanded, normal act.
 The ONE divergence: the safe state applied is the MECHANISM'S DECLARED one
 (mechanism.hpp, T4), not the drivetrain's 0 V + Brake — a loaded lift ends in
 Hold, a jammed intake does not. One definition per mechanism instead of one
 for all is the entire point of T4.

 ── Which exits apply the safe state — split by actuation physics (T4) ──────────────
 MOTOR operations end in the declared safe state on EVERY path — Succeeded,
 Stalled, TimedOut, Cancelled: "capture, and the intake stops"; "arrive, and
 the lift holds" (safe mode Hold + 0 V IS the hold). A motor left energized is
 the forgettable-safety-step failure mode A1 exists to prevent, so no exit may
 depend on the caller remembering to stop it. An open-ended behavior — keep
 the intake spinning while driving to the next goal — is deliberately NOT an
 operation: command the mechanism directly (mech.setVoltage(...)) and own the
 stop yourself.
 DISCRETE operations are different, and the difference was caught by writing
 the expected timeline before writing the code: a clamp whose declared safe
 command is "open" would FLING ITS GOAL the instant a grab SUCCEEDED if
 success applied the safe state. A solenoid consumes no energy holding a
 state, so the motor rationale does not transfer; un-commanding a completed
 actuation would undo the very act. So ActuateAndConfirm leaves the COMMANDED
 state in place on Succeeded and Unconfirmed (the completed action persists;
 an unconfirmed one stays put for the caller's retry/undo decision, loudly
 reported), and applies the declared safe state ONLY on cancel() — the
 outside hammer, which is exactly the park-guard path, where the DECLARED
 value is how a team says "clamp stays closed at the buzzer" vs "cylinder
 retracts inside the expansion limit".

 ── The watchdog (C1's discipline, one divergence named) ────────────────────────────
 RunUntilConfirmed arms a control::Watchdog in start(); NO code path disarms
 it; a never-confirming world exits TimedOut. ActuateAndConfirm's bound is its
 DEADLINE PAIR (actuation deadline + confirm deadline), both computed ONCE in
 start() from a monotonic clock and never extended — structurally the same
 guarantee (armed at start, nothing disarms it) without a redundant second
 timer that no test could ever see fire. Under an adversarial clock (jumps,
 erratic dt) every path still exits: deadlines are absolute instants, not
 accumulated dt. A FROZEN clock is the loop owner's pathology, not the
 operation's — the scheduler's stalled-pace guard turns it into a loud
 precondition (C2), and an operation cannot hang a loop it does not own.

 ── Faults vs verdicts (T6 — the line, drawn) ───────────────────────────────────────
   * Stalled  → FaultCode::MechanismStalled IS raised. High current + a shaft
     that will not turn is the robot being unwell (a jam, a bind, a motor
     cooking toward thermal throttle) — triage material.
   * TimedOut → NO fault, one Warn line. This DIVERGES from MotionTimeout on
     purpose, and the argument matters: a motion has authority over its own
     success — the drive not arriving means the robot is unwell or the
     estimate lies, so C1 latches it. A mechanism's success routinely depends
     on the WORLD cooperating: "spin until a ring arrives, 3 s budget" timing
     out with a healthy, unjammed mechanism means no ring came — strategy,
     not pathology. Latching that would flood first-fault triage with normal
     outcomes (the same reasoning as waitUntil's no-fault timeout, which is
     the precedent followed here). The genuinely-unwell timeout — stuck but
     not stalling — is indistinguishable from the world not cooperating at
     this layer; F3's primitives, which know what "should have happened",
     may mint sharper codes when they can prove them.
   * Unconfirmed → NO fault, one Warn line: healthy mechanism, failed task.
   * Succeeded / Cancelled → silent (a normal act needs no line).

 ── Where the mirror of IMotion stops (the divergences, in one list) ────────────────
   * Verdicts are MechanismOutcome, not ExitReason (mechanism_outcome.hpp).
   * No wait-for-live-estimate contract and no MotionState: an operation
     never reads the pose estimate, so there is no boot window to wait
     through and no estimate-derived target to capture.
   * No per-tick DebugRecord emission and no HealthMonitor tick: the loop
     owner (scheduler / caller) already owns both; an operation emitting its
     own records would double-count the tick.
   * The safe state is per-mechanism (above).
   * One-operation-per-mechanism is enforced by the mechanism's claim token
     (mechanism.hpp): start() takes the claim or trips a loud precondition;
     every exit releases it. C2 enforced one-active-motion with pre-empt
     policy in the scheduler; the policy layer for mechanisms (cancel the old
     operation, then start the new one) is F2's, so F1 ships only the
     structural impossibility of a silent double-drive.
```

</details>
