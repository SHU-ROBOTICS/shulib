<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/fault.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `fault.hpp`

Fault discipline (master plan §18.4; WS13, chunk A1) — the stable numeric fault-code enum and the latched first-fault capture.

This header declares **2** types (21 members) and **1** free function.

Extracted from [`include/shulib/diag/fault.hpp`](../../include/shulib/diag/fault.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class FaultCode`](#enum-class-faultcode)
  - [`None`](#faultcode-none)
  - [`Precondition`](#faultcode-precondition)
  - [`NanPose`](#faultcode-nanpose)
  - [`LoopOverrun`](#faultcode-loopoverrun)
  - [`OdoStuck`](#faultcode-odostuck)
  - [`ImuLost`](#faultcode-imulost)
  - [`GpsGateReject`](#faultcode-gpsgatereject)
  - [`Brownout`](#faultcode-brownout)
  - [`MotionTimeout`](#faultcode-motiontimeout)
  - [`MotorOverTemp`](#faultcode-motorovertemp)
  - [`Implausible`](#faultcode-implausible)
  - [`MechanismStalled`](#faultcode-mechanismstalled)
- [`faultCodeName`](#faultcodename) — *free function*
- [`class FaultLatch`](#class-faultlatch)
  - [`FaultLatch`](#faultlatch-faultlatch)
  - [`raise`](#faultlatch-raise)
  - [`hasFault`](#faultlatch-hasfault)
  - [`raiseCount`](#faultlatch-raisecount)
  - [`firstFault`](#faultlatch-firstfault)
  - [`firstFaultTime`](#faultlatch-firstfaulttime)
  - [`lastFault`](#faultlatch-lastfault)
  - [`faultCount`](#faultlatch-faultcount)
  - [`clear`](#faultlatch-clear)

<a id="enum-class-faultcode"></a>

## `enum class FaultCode`

```cpp
enum class FaultCode : std::uint16_t
```

Stable numeric fault codes (§18.4). WIRE-STABLE: explicit values, append-only — pinned by test. `None` (0) means "no fault" and is not raisable.

*enum class, declared at [`include/shulib/diag/fault.hpp:44`](../../include/shulib/diag/fault.hpp#L44).*

<a id="faultcode-none"></a>

### `FaultCode::None`

```cpp
None = 0
```

no fault (the DebugRecord default; never latched)

*enumerator, declared at [`include/shulib/diag/fault.hpp:45`](../../include/shulib/diag/fault.hpp#L45).*

<a id="faultcode-precondition"></a>

### `FaultCode::Precondition`

```cpp
Precondition = 1
```

SHULIB_PRECONDITION violated (routed here on-robot via the check.hpp policy seam; host builds throw instead — §18.4)

*enumerator, declared at [`include/shulib/diag/fault.hpp:46`](../../include/shulib/diag/fault.hpp#L46).*

<a id="faultcode-nanpose"></a>

### `FaultCode::NanPose`

```cpp
NanPose = 2
```

a non-finite pose/quantity was caught and recovered from

*enumerator, declared at [`include/shulib/diag/fault.hpp:48`](../../include/shulib/diag/fault.hpp#L48).*

<a id="faultcode-loopoverrun"></a>

### `FaultCode::LoopOverrun`

```cpp
LoopOverrun = 3
```

a control tick blew its dt budget (corrupts PID dt → §18.4)

*enumerator, declared at [`include/shulib/diag/fault.hpp:49`](../../include/shulib/diag/fault.hpp#L49).*

<a id="faultcode-odostuck"></a>

### `FaultCode::OdoStuck`

```cpp
OdoStuck = 4
```

odometry implausible / wheel stuck (raised by the C/E layers)

*enumerator, declared at [`include/shulib/diag/fault.hpp:50`](../../include/shulib/diag/fault.hpp#L50).*

<a id="faultcode-imulost"></a>

### `FaultCode::ImuLost`

```cpp
ImuLost = 5
```

IMU not ready / lost mid-run

*enumerator, declared at [`include/shulib/diag/fault.hpp:51`](../../include/shulib/diag/fault.hpp#L51).*

<a id="faultcode-gpsgatereject"></a>

### `FaultCode::GpsGateReject`

```cpp
GpsGateReject = 6
```

a GPS fix was rejected by the fusion gate (E2)

*enumerator, declared at [`include/shulib/diag/fault.hpp:52`](../../include/shulib/diag/fault.hpp#L52).*

<a id="faultcode-brownout"></a>

### `FaultCode::Brownout`

```cpp
Brownout = 7
```

battery collapsed below the brownout threshold

*enumerator, declared at [`include/shulib/diag/fault.hpp:53`](../../include/shulib/diag/fault.hpp#L53).*

<a id="faultcode-motiontimeout"></a>

### `FaultCode::MotionTimeout`

```cpp
MotionTimeout = 8
```

a motion hit its watchdog (FAULT_ABORT / TimedOut, C1/C2)

*enumerator, declared at [`include/shulib/diag/fault.hpp:54`](../../include/shulib/diag/fault.hpp#L54).*

<a id="faultcode-motorovertemp"></a>

### `FaultCode::MotorOverTemp`

```cpp
MotorOverTemp = 9
```

a motor crossed the thermal-throttle threshold (~55 °C) — the droop corrupts kS/kV/kA, so it must be visible (§8/§18.4; APPENDED at chunk A3, per the append-only rule above)

*enumerator, declared at [`include/shulib/diag/fault.hpp:55`](../../include/shulib/diag/fault.hpp#L55).*

<a id="faultcode-implausible"></a>

### `FaultCode::Implausible`

```cpp
Implausible = 10
```

a physical-plausibility invariant fired: per-tick pose delta beyond the drivetrain's physical maximum, a commanded speed outside its budget, or a wheel volt inconsistent with the battery ceiling (diagnostics-plan D-5 — FiniteGuard's log-and-recover posture extended beyond finiteness; APPENDED at chunk C5, per the append-only rule above)

*enumerator, declared at [`include/shulib/diag/fault.hpp:58`](../../include/shulib/diag/fault.hpp#L58).*

<a id="faultcode-mechanismstalled"></a>

### `FaultCode::MechanismStalled`

```cpp
MechanismStalled = 11
```

a mechanism's stall detector tripped: stall-grade current with the shaft not turning, held past the persistence window — a jam or mechanical bind (manipulation layer, T6: the one mechanism failure that IS a pathology; an operation merely timing out raises nothing — see manipulation/mechanism_op.hpp. Lands on the CONTINUE side of the C2 abort mask by default: a jammed intake must not abort a drive. APPENDED at chunk F1, per the append-only rule above)

*enumerator, declared at [`include/shulib/diag/fault.hpp:64`](../../include/shulib/diag/fault.hpp#L64).*

<a id="faultcodename"></a>

## `faultCodeName`

```cpp
[[nodiscard]] constexpr const char* faultCodeName(FaultCode code) noexcept
```

The §18.4 spelling of each code, for TermSink lines and the run summary. Never returns null; an out-of-range cast renders as "UNKNOWN" (never a crash).

*free function, declared at [`include/shulib/diag/fault.hpp:77`](../../include/shulib/diag/fault.hpp#L77).*

<a id="class-faultlatch"></a>

## `class FaultLatch`

```cpp
class FaultLatch
```

Latched first-fault capture + cascade counting (§18.4). See the header note for the root-cause rationale and the noexcept/concurrency contracts.

*class, declared at [`include/shulib/diag/fault.hpp:97`](../../include/shulib/diag/fault.hpp#L97).*

<a id="faultlatch-faultlatch"></a>

### `FaultLatch::FaultLatch`

```cpp
FaultLatch(hal::ITelemetrySink& sink, hal::IClock& clock) noexcept
```

Both references must outlive the latch. The sink receives one Error-level line per raised fault; the clock timestamps the first fault.

*function, declared at [`include/shulib/diag/fault.hpp:101`](../../include/shulib/diag/fault.hpp#L101).*

<a id="faultlatch-raise"></a>

### `FaultLatch::raise`

```cpp
void raise(FaultCode code, std::string_view subsystem, std::string_view detail) noexcept
```

Raise a fault: latch it (first-fault immutably), count it, and log one structured Error line — `fault=<NAME> n=<count>[ FIRST] <detail>`. Raising FaultCode::None is a defensive NO-OP (it is "no fault", and the error path must never crash — a precondition throw here would turn a bad raise into a dead robot).

*function, declared at [`include/shulib/diag/fault.hpp:108`](../../include/shulib/diag/fault.hpp#L108).*

<a id="faultlatch-hasfault"></a>

### `FaultLatch::hasFault`

```cpp
[[nodiscard]] bool hasFault() const noexcept
```

True once ANY fault has been raised since construction/clear(), and true for the rest of the run thereafter — this is a LATCH, not a live "is something wrong right now" query, and nothing but clear() lowers it. Raising FaultCode::None is a no-op and never sets it. For triage read firstFault(): the root cause is the first fault, not the last or the loudest.

*function, declared at [`include/shulib/diag/fault.hpp:142`](../../include/shulib/diag/fault.hpp#L142).*

<a id="faultlatch-raisecount"></a>

### `FaultLatch::raiseCount`

```cpp
[[nodiscard]] int raiseCount(FaultCode code) const noexcept
```

How many times `code` has been raised since construction/clear(). ADDED at chunk C2 (additive, like the A3 MotorOverTemp append): the scheduler's fault policy must distinguish a fault raised DURING the current motion from one latched by an earlier motion — a since-clear bitmask cannot see a RE-raise (a dead encoder that faulted in motion 1 must still abort motion 2), so the latch keeps a per-code tally. Saturates at UINT16_MAX; codes beyond the fixed slot capacity (far past today's 11) count only in faultCount().

*function, declared at [`include/shulib/diag/fault.hpp:150`](../../include/shulib/diag/fault.hpp#L150).*

<a id="faultlatch-firstfault"></a>

### `FaultLatch::firstFault`

```cpp
[[nodiscard]] FaultCode firstFault() const noexcept
```

The ROOT CAUSE: the first fault raised since construction/clear() (None if none).

*function, declared at [`include/shulib/diag/fault.hpp:155`](../../include/shulib/diag/fault.hpp#L155).*

<a id="faultlatch-firstfaulttime"></a>

### `FaultLatch::firstFaultTime`

```cpp
[[nodiscard]] units::Time firstFaultTime() const noexcept
```

When the first fault was raised (Time{0} if none, or if the clock threw).

*function, declared at [`include/shulib/diag/fault.hpp:157`](../../include/shulib/diag/fault.hpp#L157).*

<a id="faultlatch-lastfault"></a>

### `FaultLatch::lastFault`

```cpp
[[nodiscard]] FaultCode lastFault() const noexcept
```

The most recent fault in the cascade (None if none) — display only, never triage.

*function, declared at [`include/shulib/diag/fault.hpp:159`](../../include/shulib/diag/fault.hpp#L159).*

<a id="faultlatch-faultcount"></a>

### `FaultLatch::faultCount`

```cpp
[[nodiscard]] int faultCount() const noexcept
```

Total faults raised since construction/clear() (first + cascade).

*function, declared at [`include/shulib/diag/fault.hpp:161`](../../include/shulib/diag/fault.hpp#L161).*

<a id="faultlatch-clear"></a>

### `FaultLatch::clear`

```cpp
void clear() noexcept
```

Reset between runs. The first-fault latch is immutable WITHIN a run by design; only an explicit new-run boundary may clear it.

*function, declared at [`include/shulib/diag/fault.hpp:165`](../../include/shulib/diag/fault.hpp#L165).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 Fault discipline (master plan §18.4; WS13, chunk A1) — the stable numeric fault-code
 enum and the latched first-fault capture. The rule this file enforces: FAULTS LOG AND
 RECOVER, THEY NEVER CRASH. A NaN pose, a sensor pathology, or a loop overrun raises a
 code and the run continues on a safe fallback; nothing in this file can abort an auton.

 Why the FIRST fault is latched distinctly from the cascade: one root cause (say a NaN
 pose) typically triggers a burst of follow-on faults (odom stuck, gate rejects, motion
 timeout). At 2am you need the ROOT CAUSE, and it is the first fault, not the loudest or
 the last. FaultLatch therefore records the first (code + time) immutably until clear(),
 while still counting and logging every subsequent fault in the cascade.

 Why the enum values are EXPLICIT and never reordered: these numbers go on the F9 wire
 (the SHUL/2 serialization of DebugRecord, frozen at H1) and into SdSink blackbox files
 (E1). A reorder would silently re-label historical logs. The numeric values are pinned
 by test/fault_test.cpp — reordering turns the suite red. New codes are ADDED at the
 end, never inserted.

 Concurrency contract (the legacy logger's racing flush, designed against): FaultLatch
 is owned and mutated by ONE task (the control loop). It has no background task, no
 buffering, and no flush — raise() formats into a stack buffer and hands one line to the
 sink synchronously on the caller's task. Cross-task use requires external serialization.

 raise() is noexcept BY CONTRACT — it is the error path, so it must be unconditionally
 safe to call from any failure handler. A sink or clock that throws violates its own
 interface contract ("implementations MUST NOT throw"); raise() swallows such a throw
 (after the latch state is already updated) rather than crashing the run over a broken
 diagnostic channel. The latch always latches, even if logging fails.
```

</details>
