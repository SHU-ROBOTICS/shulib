<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/tick_attribution.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tick_attribution.hpp`

TickAttribution — WHO consumed the loop budget.

This header declares **3** types (18 members) and **1** free function.

Extracted from [`include/shulib/diag/tick_attribution.hpp`](../../include/shulib/diag/tick_attribution.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class TickAttribution`](#class-tickattribution)
  - [`Phases`](#tickattribution-phases)
  - [`TickAttribution`](#tickattribution-tickattribution)
  - [`beginTick`](#tickattribution-begintick)
  - [`phase`](#tickattribution-phase)
  - [`phaseInPlace`](#tickattribution-phaseinplace)
  - [`endTick`](#tickattribution-endtick)
  - [`abandonTick`](#tickattribution-abandontick)
  - [`hasCompletedTick`](#tickattribution-hascompletedtick)
  - [`lastPhases`](#tickattribution-lastphases)
  - [`lastTotal`](#tickattribution-lasttotal)
  - [`lastAttributed`](#tickattribution-lastattributed)
  - [`lastOther`](#tickattribution-lastother)
  - [`lastWorstPhase`](#tickattribution-lastworstphase)
  - [`reset`](#tickattribution-reset)
  - [`class TickAttribution::PhaseScope`](#class-tickattribution-phasescope)
    - [`PhaseScope`](#tickattribution-phasescope-phasescope)
    - [`~PhaseScope`](#tickattribution-phasescope-destructor-phasescope)
    - [`PhaseScope (overload 2)`](#tickattribution-phasescope-phasescope-2)
    - [`operator=`](#tickattribution-phasescope-operator-eq)
    - [`class TickAttribution::PhaseScope::Key`](#class-tickattribution-phasescope-key)
- [`tickPhaseName`](#tickphasename) — *free function*

<a id="class-tickattribution"></a>

## `class TickAttribution`

```cpp
class TickAttribution
```

Measures where one tick's time went, phase by phase, on an INJECTED clock — the "who" that LoopMonitor's "this tick blew its budget" cannot answer on its own. Only the LAST COMPLETED tick's breakdown is kept, so a record stamped mid-tick necessarily carries the previous tick's numbers; for the overrun path that lag is exactly right, because an overrun is detected on the tick AFTER the one that caused it. Needs a clock that advances DURING a tick, which is why it takes its own: the host sim clock only moves between ticks and would report every phase as zero. Single-task by contract, like the rest of diag/.

*class, declared at [`include/shulib/diag/tick_attribution.hpp:54`](../../include/shulib/diag/tick_attribution.hpp#L54).*

<a id="tickattribution-phases"></a>

### `TickAttribution::Phases`

```cpp
using Phases = std::array<units::Time, static_cast<std::size_t>(kTickPhaseSlots)>
```

Per-phase durations for one tick, indexed by TickPhase. Sized by kTickPhaseSlots rather than by the phases that exist today — the spare slots are what make a new phase an append to the vocabulary instead of a reshape of the telemetry wire.

*alias, declared at [`include/shulib/diag/tick_attribution.hpp:59`](../../include/shulib/diag/tick_attribution.hpp#L59).*

<a id="tickattribution-tickattribution"></a>

### `TickAttribution::TickAttribution`

```cpp
explicit TickAttribution(hal::IClock& clock) noexcept
```

`clock` must outlive the instance (see header for WHICH clock).

*function, declared at [`include/shulib/diag/tick_attribution.hpp:62`](../../include/shulib/diag/tick_attribution.hpp#L62).*

<a id="tickattribution-begintick"></a>

### `TickAttribution::beginTick`

```cpp
void beginTick()
```

Open a tick: zero the working phases, mark the start instant.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:65`](../../include/shulib/diag/tick_attribution.hpp#L65).*

<a id="tickattribution-phase"></a>

### `TickAttribution::phase`

```cpp
[[nodiscard]] PhaseScope phase(TickPhase p)
```

Open a scope that charges its own lifetime to `p`. Requires a tick to be open. The result MUST be bound to a named variable — an unnamed temporary dies at the semicolon and charges nothing, which is the whole reason this is [[nodiscard]].

*function, declared at [`include/shulib/diag/tick_attribution.hpp:123`](../../include/shulib/diag/tick_attribution.hpp#L123).*

<a id="tickattribution-phaseinplace"></a>

### `TickAttribution::phaseInPlace`

```cpp
[[nodiscard]] std::optional<PhaseScope> phaseInPlace(TickPhase p)
```

The same scope, in an optional. Exists because PhaseScope is deliberately non-movable, so phase()'s by-value return cannot be stored in one — and a caller that needs the optional shape (attribution is switchable, and must cost nothing when off) previously had to construct a PhaseScope directly with `std::in_place`, walking around the tick-open check. MotionScheduler was that caller, and was the only user of the bypass; with this it goes through the same precondition as everyone else. Same requirement as phase(): bind the result to a named variable, or it charges nothing.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:135`](../../include/shulib/diag/tick_attribution.hpp#L135).*

<a id="tickattribution-endtick"></a>

### `TickAttribution::endTick`

```cpp
void endTick()
```

Close the tick: snapshot the working phases + total as the LAST COMPLETED tick (what records and overrun lines read).

*function, declared at [`include/shulib/diag/tick_attribution.hpp:142`](../../include/shulib/diag/tick_attribution.hpp#L142).*

<a id="tickattribution-abandontick"></a>

### `TickAttribution::abandonTick`

```cpp
void abandonTick() noexcept
```

Discard a half-measured tick (an exception unwound through the tick body): its numbers never completed, so they are dropped rather than reported, and the instrument re-arms. The last COMPLETED tick's story is untouched.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:153`](../../include/shulib/diag/tick_attribution.hpp#L153).*

<a id="tickattribution-hascompletedtick"></a>

### `TickAttribution::hasCompletedTick`

```cpp
[[nodiscard]] bool hasCompletedTick() const noexcept
```

False until the first endTick(), and again after reset(). Worth asking first: before any tick completes every lastX() accessor reads zero, which is indistinguishable from a tick that genuinely cost nothing.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:158`](../../include/shulib/diag/tick_attribution.hpp#L158).*

<a id="tickattribution-lastphases"></a>

### `TickAttribution::lastPhases`

```cpp
[[nodiscard]] const Phases& lastPhases() const noexcept
```

The last completed tick's per-phase durations (zeros before any tick).

*function, declared at [`include/shulib/diag/tick_attribution.hpp:160`](../../include/shulib/diag/tick_attribution.hpp#L160).*

<a id="tickattribution-lasttotal"></a>

### `TickAttribution::lastTotal`

```cpp
[[nodiscard]] units::Time lastTotal() const noexcept
```

Seconds from beginTick() to endTick() of the last completed tick, on the attribution clock. It spans the whole tick, including work no phase scope wrapped — that remainder is what lastOther() reports rather than smearing it into a named phase.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:164`](../../include/shulib/diag/tick_attribution.hpp#L164).*

<a id="tickattribution-lastattributed"></a>

### `TickAttribution::lastAttributed`

```cpp
[[nodiscard]] units::Time lastAttributed() const noexcept
```

Sum of the attributed phases of the last completed tick.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:167`](../../include/shulib/diag/tick_attribution.hpp#L167).*

<a id="tickattribution-lastother"></a>

### `TickAttribution::lastOther`

```cpp
[[nodiscard]] units::Time lastOther() const noexcept
```

total − attributed: un-instrumented work. Floored at 0 (a clock that jumped mid-phase can make phases overshoot the total; the floor keeps the report coherent rather than printing a negative time).

*function, declared at [`include/shulib/diag/tick_attribution.hpp:178`](../../include/shulib/diag/tick_attribution.hpp#L178).*

<a id="tickattribution-lastworstphase"></a>

### `TickAttribution::lastWorstPhase`

```cpp
[[nodiscard]] TickPhase lastWorstPhase() const noexcept
```

The phase that consumed the most of the last completed tick — the NAME the overrun line prints. Ties resolve to the lower index (deterministic).

*function, declared at [`include/shulib/diag/tick_attribution.hpp:185`](../../include/shulib/diag/tick_attribution.hpp#L185).*

<a id="tickattribution-reset"></a>

### `TickAttribution::reset`

```cpp
void reset() noexcept
```

Forget everything (run boundary). The next tick starts a fresh story.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:196`](../../include/shulib/diag/tick_attribution.hpp#L196).*

<a id="class-tickattribution-phasescope"></a>

## `class TickAttribution::PhaseScope`

```cpp
class PhaseScope
```

Time one phase, RAII-style: the duration is credited when the scope closes. { auto scope = att.phase(TickPhase::Localization); localizer.update(); } Phases may repeat within a tick (durations accumulate); scopes must not overlap the same phase (the second-open would double-charge the overlap).

*class, declared at [`include/shulib/diag/tick_attribution.hpp:76`](../../include/shulib/diag/tick_attribution.hpp#L76).*

<a id="tickattribution-phasescope-phasescope"></a>

### `TickAttribution::PhaseScope::PhaseScope`

```cpp
PhaseScope(Key /*unused*/, TickAttribution& att, TickPhase phase) noexcept
```

Stamps the start instant. Reachable only through TickAttribution::phase() or ::phaseInPlace(), both of which check that a tick is actually open — the `Key` parameter is what makes that structural. It was a plain public constructor, which made the tick-open precondition advisory: a direct `PhaseScope s{att, p}` compiled with no tick open and its destructor still wrote into current_, crediting the interval to whatever tick happened to be open when it closed.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:96`](../../include/shulib/diag/tick_attribution.hpp#L96).*

<a id="tickattribution-phasescope-destructor-phasescope"></a>

### `TickAttribution::PhaseScope::~PhaseScope`

```cpp
~PhaseScope()
```

Credits (now − start) to the phase on scope exit, and only then: a scope still alive when endTick() runs contributes nothing to the tick it was opened in — its interval lands on whatever tick is open when it finally closes, or is discarded outright if the next beginTick() zeroes the working phases first. Repeated scopes on the same phase within one tick ACCUMULATE rather than replace.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:104`](../../include/shulib/diag/tick_attribution.hpp#L104).*

<a id="tickattribution-phasescope-phasescope-2"></a>

### `TickAttribution::PhaseScope::PhaseScope (overload 2)`

```cpp
PhaseScope(const PhaseScope&) = delete
```

Non-copyable, and therefore non-movable: a scope charges exactly one interval, and a copy would charge it twice. phase() still returns one by value — that is guaranteed elision, not a move.

*function, declared at [`include/shulib/diag/tick_attribution.hpp:111`](../../include/shulib/diag/tick_attribution.hpp#L111).*

<a id="tickattribution-phasescope-operator-eq"></a>

### `TickAttribution::PhaseScope::operator=`

```cpp
PhaseScope& operator=(const PhaseScope&) = delete
```

*Covered by the comment on [`PhaseScope (overload 2)`](#tickattribution-phasescope-phasescope-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/diag/tick_attribution.hpp:112`](../../include/shulib/diag/tick_attribution.hpp#L112).*

<a id="class-tickattribution-phasescope-key"></a>

## `class TickAttribution::PhaseScope::Key`

```cpp
class Key
```

Passkey. The TYPE is public so TickAttribution can name it; its CONSTRUCTOR is private with TickAttribution as the only friend, so nobody else can produce one. PhaseScope's own constructor therefore stays public — which std::optional's in-place construction requires, because optional does the constructing and cannot be made a friend — while remaining unreachable without a Key. A simple private constructor plus `friend` looks tidier and does not work here for exactly that reason.

*class, declared at [`include/shulib/diag/tick_attribution.hpp:85`](../../include/shulib/diag/tick_attribution.hpp#L85).*

_No public members._

<a id="tickphasename"></a>

## `tickPhaseName`

```cpp
[[nodiscard]] constexpr const char* tickPhaseName(TickPhase phase) noexcept
```

Short display token per phase for the overrun-attribution line ("loc"/"mot"/…).

*free function, declared at [`include/shulib/diag/tick_attribution.hpp:215`](../../include/shulib/diag/tick_attribution.hpp#L215).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 32 lines</summary>

```text

 TickAttribution — WHO consumed the loop budget (diagnostics-plan D-3; WS13, C5).

 LoopMonitor (A1) detects that a tick blew its budget; it cannot say WHO —
 "localization 4 ms, motion 2 ms, sinks 1 ms" is the question an overrun actually
 raises. This class measures named phases inside one tick on an injected clock
 and keeps the LAST COMPLETED tick's breakdown, so:
   * the scheduler stamps it into every DebugRecord (the D-3 schema slots), and
   * an overrun line can NAME the worst consumer instead of shrugging.

 ── Which clock, and why it is injected separately ──────────────────────────────────
 Phase timing needs a clock that ADVANCES DURING A TICK. On the robot that is
 real time (R1 wires the microsecond clock — the same IClock the loop uses). In
 HOST SIM the sim clock advances only between ticks (the pacer steps the world),
 so phases would all read 0 — true, harmless, and useless. The attribution clock
 is therefore its own injection: tests drive a deterministic one, sim runs may
 leave attribution off entirely (a null clock in the scheduler config = feature
 off = zero clock calls — the A1 cost contract, structurally).

 ── One-tick lag, stated honestly ───────────────────────────────────────────────────
 Records are emitted DURING the Motion phase (by the motion itself), before that
 phase's duration — or the tick's total — is knowable. So what rides a record is
 the breakdown of the most recently COMPLETED tick, uniformly (documented on the
 schema field). For the overrun path this lag is exactly right: the overrun is
 DETECTED at tick N+1 (its dt covers tick N's work), and the last completed
 breakdown at that moment IS tick N — the tick that overran.

 Sum contract (pinned by test): attributed phase times never exceed the tick
 total on the same clock; total − attributed = "other" (un-instrumented work +
 pacing), reported as its own quantity rather than smeared into a named phase.

 Single-task by contract, like the rest of diag/.
```

</details>
