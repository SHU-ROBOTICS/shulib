<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/run_summary.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `run_summary.hpp`

RunSummary — the end-of-run one-screen summary, as DATA.

This header declares **1** type (22 members).

Extracted from [`include/shulib/diag/run_summary.hpp`](../../include/shulib/diag/run_summary.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct RunSummary`](#struct-runsummary)
  - [`motionsStarted`](#runsummary-motionsstarted)
  - [`motionsSettled`](#runsummary-motionssettled)
  - [`motionsTimedOut`](#runsummary-motionstimedout)
  - [`motionsCancelled`](#runsummary-motionscancelled)
  - [`motionsAborted`](#runsummary-motionsaborted)
  - [`hasHeadingData`](#runsummary-hasheadingdata)
  - [`headingMax`](#runsummary-headingmax)
  - [`headingFinal`](#runsummary-headingfinal)
  - [`gatingRejects`](#runsummary-gatingrejects)
  - [`brownout`](#runsummary-brownout)
  - [`worstLoopDt`](#runsummary-worstloopdt)
  - [`firstFault`](#runsummary-firstfault)
  - [`firstFaultTime`](#runsummary-firstfaulttime)
  - [`droppedRecords`](#runsummary-droppedrecords)
  - [`droppedLines`](#runsummary-droppedlines)
  - [`blackboxDropped`](#runsummary-blackboxdropped)
  - [`batteryStart`](#runsummary-batterystart)
  - [`batteryEnd`](#runsummary-batteryend)
  - [`setBuildHash`](#runsummary-setbuildhash)
  - [`setRoutineId`](#runsummary-setroutineid)
  - [`buildHash`](#runsummary-buildhash)
  - [`routineId`](#runsummary-routineid)

<a id="struct-runsummary"></a>

## `struct RunSummary`

```cpp
struct RunSummary
```

The end-of-run summary as DATA, never as an assembled essay: structured fields that one producer fills and any number of renderers format — the boxed terminal block, an appended blackbox frame, a wire message. A VALUE TYPE that owns its provenance strings in bounded in-struct arrays and allocates nothing, so a sink may RETAIN a copy without holding a dangling view into some caller's stack. Assembled once per run and delivered through hal::ITelemetrySink::summarize().

*struct, declared at [`include/shulib/diag/run_summary.hpp:45`](../../include/shulib/diag/run_summary.hpp#L45).*

<a id="runsummary-motionsstarted"></a>

### `RunSummary::motionsStarted`

```cpp
int motionsStarted = 0
```

Motions the scheduler handed a start(). It EXCEEDS the four outcome counts below whenever a motion was still running when the summary was taken — they partition the FINISHED motions only, so started minus their sum is what was still in flight.

*field, declared at [`include/shulib/diag/run_summary.hpp:50`](../../include/shulib/diag/run_summary.hpp#L50).*

<a id="runsummary-motionssettled"></a>

### `RunSummary::motionsSettled`

```cpp
int motionsSettled = 0
```

Exited inside its tolerances — the only outcome that means success

*field, declared at [`include/shulib/diag/run_summary.hpp:51`](../../include/shulib/diag/run_summary.hpp#L51).*

<a id="runsummary-motionstimedout"></a>

### `RunSummary::motionsTimedOut`

```cpp
int motionsTimedOut = 0
```

Exited on the watchdog; each one also raised MOTION_TIMEOUT

*field, declared at [`include/shulib/diag/run_summary.hpp:52`](../../include/shulib/diag/run_summary.hpp#L52).*

<a id="runsummary-motionscancelled"></a>

### `RunSummary::motionsCancelled`

```cpp
int motionsCancelled = 0
```

user/pre-empt cancels (no causal fault)

*field, declared at [`include/shulib/diag/run_summary.hpp:53`](../../include/shulib/diag/run_summary.hpp#L53).*

<a id="runsummary-motionsaborted"></a>

### `RunSummary::motionsAborted`

```cpp
int motionsAborted = 0
```

fault-policy / task-boundary aborts

*field, declared at [`include/shulib/diag/run_summary.hpp:54`](../../include/shulib/diag/run_summary.hpp#L54).*

<a id="runsummary-hasheadingdata"></a>

### `RunSummary::hasHeadingData`

```cpp
bool hasHeadingData = false
```

False when no motion produced heading data (record stream off, or nothing ran) — renderers show "n/a", never a fabricated 0.0 (a 0.0° claim with no data behind it is exactly the lying-number failure C5's brief bans).

*field, declared at [`include/shulib/diag/run_summary.hpp:60`](../../include/shulib/diag/run_summary.hpp#L60).*

<a id="runsummary-headingmax"></a>

### `RunSummary::headingMax`

```cpp
units::AngleDim headingMax{}
```

worst per-motion final |heading error| (radians)

*field, declared at [`include/shulib/diag/run_summary.hpp:61`](../../include/shulib/diag/run_summary.hpp#L61).*

<a id="runsummary-headingfinal"></a>

### `RunSummary::headingFinal`

```cpp
units::AngleDim headingFinal{}
```

the LAST motion's final |heading error| (radians)

*field, declared at [`include/shulib/diag/run_summary.hpp:62`](../../include/shulib/diag/run_summary.hpp#L62).*

<a id="runsummary-gatingrejects"></a>

### `RunSummary::gatingRejects`

```cpp
int gatingRejects = 0
```

GPS_GATE_REJECT episodes (FaultLatch tally)

*field, declared at [`include/shulib/diag/run_summary.hpp:65`](../../include/shulib/diag/run_summary.hpp#L65).*

<a id="runsummary-brownout"></a>

### `RunSummary::brownout`

```cpp
bool brownout = false
```

HealthMonitor::brownedOut() — latched, E1 semantics

*field, declared at [`include/shulib/diag/run_summary.hpp:66`](../../include/shulib/diag/run_summary.hpp#L66).*

<a id="runsummary-worstloopdt"></a>

### `RunSummary::worstLoopDt`

```cpp
units::Time worstLoopDt{}
```

LoopMonitor::worstDt()

*field, declared at [`include/shulib/diag/run_summary.hpp:67`](../../include/shulib/diag/run_summary.hpp#L67).*

<a id="runsummary-firstfault"></a>

### `RunSummary::firstFault`

```cpp
FaultCode firstFault = FaultCode::None
```

the ROOT CAUSE (FaultLatch first-fault)

*field, declared at [`include/shulib/diag/run_summary.hpp:68`](../../include/shulib/diag/run_summary.hpp#L68).*

<a id="runsummary-firstfaulttime"></a>

### `RunSummary::firstFaultTime`

```cpp
units::Time firstFaultTime{}
```

when it latched (0 if none)

*field, declared at [`include/shulib/diag/run_summary.hpp:69`](../../include/shulib/diag/run_summary.hpp#L69).*

<a id="runsummary-droppedrecords"></a>

### `RunSummary::droppedRecords`

```cpp
std::uint32_t droppedRecords = 0
```

RateLimitedSink::droppedRecords()

*field, declared at [`include/shulib/diag/run_summary.hpp:72`](../../include/shulib/diag/run_summary.hpp#L72).*

<a id="runsummary-droppedlines"></a>

### `RunSummary::droppedLines`

```cpp
std::uint32_t droppedLines = 0
```

RateLimitedSink::droppedLines()

*field, declared at [`include/shulib/diag/run_summary.hpp:73`](../../include/shulib/diag/run_summary.hpp#L73).*

<a id="runsummary-blackboxdropped"></a>

### `RunSummary::blackboxDropped`

```cpp
std::uint32_t blackboxDropped = 0
```

Frames the E1 blackbox (diag::SdSink) dropped because its RAM byte budget was exhausted, or because a device write failed. A SEPARATE counter from the two above on purpose: those are rate-limiter drops on the terminal channel, and merging two different failures into one number is how a diagnostic starts lying. 0 also means "no blackbox was attached", which is why renderers show this one only when it is non-zero (TermSink's summarize note). — E1

*field, declared at [`include/shulib/diag/run_summary.hpp:80`](../../include/shulib/diag/run_summary.hpp#L80).*

<a id="runsummary-batterystart"></a>

### `RunSummary::batteryStart`

```cpp
units::Voltage batteryStart{}
```

Pack volts READ at session start, never caller-typed: a typed 12.6 that was really 11.9 is exactly the lying number this record exists to avoid.

*field, declared at [`include/shulib/diag/run_summary.hpp:85`](../../include/shulib/diag/run_summary.hpp#L85).*

<a id="runsummary-batteryend"></a>

### `RunSummary::batteryEnd`

```cpp
units::Voltage batteryEnd{}
```

Pack volts read when the summary was assembled; with batteryStart, the run's sag. Both are 0 V on a summary nobody filled in — there is no "unset" sentinel here.

*field, declared at [`include/shulib/diag/run_summary.hpp:88`](../../include/shulib/diag/run_summary.hpp#L88).*

<a id="runsummary-setbuildhash"></a>

### `RunSummary::setBuildHash`

```cpp
void setBuildHash(std::string_view hash) noexcept
```

Empty ⇒ MISSING (rendered loudly; header note). 47 bytes admits a full 40-char git SHA plus a "-dirty" suffix.

*function, declared at [`include/shulib/diag/run_summary.hpp:92`](../../include/shulib/diag/run_summary.hpp#L92).*

<a id="runsummary-setroutineid"></a>

### `RunSummary::setRoutineId`

```cpp
void setRoutineId(std::string_view id) noexcept
```

Copy the auton routine's name (e.g. "redLeftTall") in, TRUNCATED at 31 characters. Empty is ordinary here — only buildHash treats empty as the loud MISSING case.

*function, declared at [`include/shulib/diag/run_summary.hpp:95`](../../include/shulib/diag/run_summary.hpp#L95).*

<a id="runsummary-buildhash"></a>

### `RunSummary::buildHash`

```cpp
[[nodiscard]] std::string_view buildHash() const noexcept
```

The stored hash; EMPTY means the build system provided none, which renderers must print as MISSING rather than anything plausible-looking. LIFETIME: the view points into THIS object — it dies with the summary, the next setBuildHash() invalidates it, and a copied summary hands back views into the COPY. That is the whole reason this is a value type rather than a struct of string_views.

*function, declared at [`include/shulib/diag/run_summary.hpp:102`](../../include/shulib/diag/run_summary.hpp#L102).*

<a id="runsummary-routineid"></a>

### `RunSummary::routineId`

```cpp
[[nodiscard]] std::string_view routineId() const noexcept
```

The stored routine name; empty if never set. Same lifetime rule as buildHash(): the view is into this object, never into what the caller passed setRoutineId().

*function, declared at [`include/shulib/diag/run_summary.hpp:106`](../../include/shulib/diag/run_summary.hpp#L106).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 27 lines</summary>

```text

 RunSummary — the end-of-run one-screen summary, as DATA (master plan §18.3; WS13,
 chunk C5). The §18.3 block is one of the two ideas worth salvaging from the legacy
 "logging extreme" code, re-expressed the §18 way: STRUCTURED FIELDS the sinks
 format, never an essay assembled in a motion loop.

 Why a struct on the sink seam (hal::ITelemetrySink::summarize) rather than text
 pushed through log(): the summary is a RECORD with one producer and many possible
 renderings — TermSink draws the boxed block, E1's SdSink will append it to the
 blackbox, H1's wire can carry it to VexBuilder. Formatting it before the seam
 would freeze the terminal rendering as the only consumer (§18.1 "one record, many
 sinks" applies to every record type, not just the per-tick one).

 VALUE TYPE, deliberately: the provenance strings live in bounded in-struct arrays
 (set via setBuildHash/setRoutineId), not string_views — a sink that RETAINS a
 summary (FakeTelemetrySink, the E1 blackbox) must not be handed dangling views
 into some caller's stack. Nothing here allocates.

 Content mapping, stated honestly (§18.3's sketch shows "scored 6 pin · 1 cup"):
 game-object scoring belongs to the strategy layer (the G-phase command registry —
 the library cannot know what a motion scored). The M2-honest equivalent is the
 MOTION LEDGER (started/settled/timeout/cancelled/aborted); when G2's registry
 exists, scoring lands as an ADDITIVE field, not a reshape.

 The "MISSING" contract (§18.5): an EMPTY buildHash means the build system did not
 provide one. Renderers must say MISSING, loudly — never invent a plausible value.
 A wrong hash is worse than an absent one.
```

</details>
