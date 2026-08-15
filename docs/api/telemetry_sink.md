<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/telemetry_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `telemetry_sink.hpp`

ITelemetrySink — the diagnostics output seam.

This header declares **2** types (15 members) and **1** free function.

Extracted from [`include/shulib/hal/telemetry_sink.hpp`](../../include/shulib/hal/telemetry_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class LogLevel`](#enum-class-loglevel)
  - [`Error`](#loglevel-error)
  - [`Warn`](#loglevel-warn)
  - [`Info`](#loglevel-info)
  - [`Debug`](#loglevel-debug)
  - [`Trace`](#loglevel-trace)
- [`class ITelemetrySink`](#class-itelemetrysink)
  - [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink)
  - [`ITelemetrySink`](#itelemetrysink-itelemetrysink)
  - [`ITelemetrySink (overload 2)`](#itelemetrysink-itelemetrysink-2)
  - [`ITelemetrySink (overload 3)`](#itelemetrysink-itelemetrysink-3)
  - [`operator=`](#itelemetrysink-operator-eq)
  - [`operator= (overload 2)`](#itelemetrysink-operator-eq-2)
  - [`log`](#itelemetrysink-log)
  - [`wantsRecord`](#itelemetrysink-wantsrecord)
  - [`emit`](#itelemetrysink-emit)
  - [`summarize`](#itelemetrysink-summarize)
- [`emitRecord`](#emitrecord) — *free function*

<a id="enum-class-loglevel"></a>

## `enum class LogLevel`

```cpp
enum class LogLevel
```

Severity, high → low. TRACE is compile-time strippable off the hot path (§18.3; see shulib/diag/trace.hpp for the strip mechanism).

*enum class, declared at [`include/shulib/hal/telemetry_sink.hpp:64`](../../include/shulib/hal/telemetry_sink.hpp#L64).*

<a id="loglevel-error"></a>

### `LogLevel::Error`

```cpp
Error
```

Something went wrong that a person must know about — a fault raised, a device refusing, a precondition the library could not honour. The one level nothing filters away by default.

*enumerator, declared at [`include/shulib/hal/telemetry_sink.hpp:68`](../../include/shulib/hal/telemetry_sink.hpp#L68).*

<a id="loglevel-warn"></a>

### `LogLevel::Warn`

```cpp
Warn
```

Degraded but still running: a fallback taken, a reading rejected, a limit clamped. The run continues and the outcome may still be correct.

*enumerator, declared at [`include/shulib/hal/telemetry_sink.hpp:71`](../../include/shulib/hal/telemetry_sink.hpp#L71).*

<a id="loglevel-info"></a>

### `LogLevel::Info`

```cpp
Info
```

The run's narrative — motions starting and finishing, the session header, the end-of-run summary. What you read to follow what happened.

*enumerator, declared at [`include/shulib/hal/telemetry_sink.hpp:74`](../../include/shulib/hal/telemetry_sink.hpp#L74).*

<a id="loglevel-debug"></a>

### `LogLevel::Debug`

```cpp
Debug
```

Per-subsystem detail useful while working on that subsystem, and noise otherwise. `LevelFilterSink` exists so one tag can be raised to this without drowning the rest.

*enumerator, declared at [`include/shulib/hal/telemetry_sink.hpp:78`](../../include/shulib/hal/telemetry_sink.hpp#L78).*

<a id="loglevel-trace"></a>

### `LogLevel::Trace`

```cpp
Trace
```

Per-tick firehose. Distinct from the four above in that it is COMPILE-TIME strippable: `SHULIB_TRACE` compiles to nothing unless tracing is enabled, so a stripped build pays no argument evaluation, not merely no output (see `shulib/diag/trace.hpp`).

*enumerator, declared at [`include/shulib/hal/telemetry_sink.hpp:83`](../../include/shulib/hal/telemetry_sink.hpp#L83).*

<a id="class-itelemetrysink"></a>

## `class ITelemetrySink`

```cpp
class ITelemetrySink
```

The ONE diagnostics output seam — NullSink, TermSink, SdSink, Shul2Sink and every decorator sit behind it, so the same trace reaches the terminal, an SD blackbox or the wire without the core knowing which. THREE channels ride it: log() (leveled messages, pure virtual), emit() (per-tick DebugRecord) and summarize() (once per run). Only log() is pure — emit() and summarize() default to no-ops so a sink written against an older version of this seam keeps compiling when a channel is added. Everything runs SYNCHRONOUSLY on the caller's task: there is no background thread and no queue here, implementations MUST NOT throw, and each one documents its own thread-safety (the shipped sinks are single-task by contract).

*class, declared at [`include/shulib/hal/telemetry_sink.hpp:94`](../../include/shulib/hal/telemetry_sink.hpp#L94).*

<a id="itelemetrysink-destructor-itelemetrysink"></a>

### `ITelemetrySink::~ITelemetrySink`

```cpp
virtual ~ITelemetrySink() = default
```

Abstract base, held and destroyed through ITelemetrySink*. Sinks are referenced, never owned, by everything that logs (FaultLatch, RobotContext, every decorator's `inner_`), so a sink must outlive the whole chain that points at it — and decorators must be destroyed before the sink they wrap. Copy/move are defaulted because this base carries no state, but copying through it slices away a concrete sink's buffers and file handles.

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:101`](../../include/shulib/hal/telemetry_sink.hpp#L101).*

<a id="itelemetrysink-itelemetrysink"></a>

### `ITelemetrySink::ITelemetrySink`

```cpp
ITelemetrySink() = default
```

*Covered by the comment on [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:102`](../../include/shulib/hal/telemetry_sink.hpp#L102).*

<a id="itelemetrysink-itelemetrysink-2"></a>

### `ITelemetrySink::ITelemetrySink (overload 2)`

```cpp
ITelemetrySink(const ITelemetrySink&) = default
```

*Covered by the comment on [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:103`](../../include/shulib/hal/telemetry_sink.hpp#L103).*

<a id="itelemetrysink-itelemetrysink-3"></a>

### `ITelemetrySink::ITelemetrySink (overload 3)`

```cpp
ITelemetrySink(ITelemetrySink&&) = default
```

*Covered by the comment on [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:104`](../../include/shulib/hal/telemetry_sink.hpp#L104).*

<a id="itelemetrysink-operator-eq"></a>

### `ITelemetrySink::operator=`

```cpp
ITelemetrySink& operator=(const ITelemetrySink&) = default
```

*Covered by the comment on [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:105`](../../include/shulib/hal/telemetry_sink.hpp#L105).*

<a id="itelemetrysink-operator-eq-2"></a>

### `ITelemetrySink::operator= (overload 2)`

```cpp
ITelemetrySink& operator=(ITelemetrySink&&) = default
```

*Covered by the comment on [`~ITelemetrySink`](#itelemetrysink-destructor-itelemetrysink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:106`](../../include/shulib/hal/telemetry_sink.hpp#L106).*

<a id="itelemetrysink-log"></a>

### `ITelemetrySink::log`

```cpp
virtual void log(LogLevel level, std::string_view subsystem, std::string_view message) = 0
```

Emit a leveled, subsystem-tagged message (§18.3). Implementations MUST NOT throw.

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:109`](../../include/shulib/hal/telemetry_sink.hpp#L109).*

<a id="itelemetrysink-wantsrecord"></a>

### `ITelemetrySink::wantsRecord`

```cpp
[[nodiscard]] virtual bool wantsRecord() const noexcept
```

Would emit() consume a DebugRecord? Callers use this (via emitRecord below) to skip record POPULATION entirely when nothing consumes it — the null-sink cost mechanism (header note). Default false, matching the default no-op emit(): OVERRIDE THIS AND emit() AS A PAIR.

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:115`](../../include/shulib/hal/telemetry_sink.hpp#L115).*

<a id="itelemetrysink-emit"></a>

### `ITelemetrySink::emit`

```cpp
virtual void emit(const diag::DebugRecord& /*record*/)
```

Consume one per-tick DebugRecord (§18.2). NON-pure with a default no-op body BY CONTRACT (header note) — a sink implementing only log() stays valid forever. Implementations MUST NOT throw.

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:120`](../../include/shulib/hal/telemetry_sink.hpp#L120).*

<a id="itelemetrysink-summarize"></a>

### `ITelemetrySink::summarize`

```cpp
virtual void summarize(const diag::RunSummary& /*summary*/)
```

Consume the end-of-run summary (§18.3's one-screen block, as data — see shulib/diag/run_summary.hpp). Added at chunk C5 by the SAME additive recipe as emit(): non-pure, default no-op, so every sink written before it kept compiling untouched. Called ONCE per run (cold path — no wants-style query is needed; building one struct per run is not a cost). DECORATOR RULE: a sink that wraps another MUST forward this, like log()/emit() — a decorator with the default body silently eats the summary. Implementations MUST NOT throw.

*function, declared at [`include/shulib/hal/telemetry_sink.hpp:129`](../../include/shulib/hal/telemetry_sink.hpp#L129).*

<a id="emitrecord"></a>

## `emitRecord`

```cpp
template <typename BuildFn> inline void emitRecord(ITelemetrySink& sink, BuildFn&& buildRecord)
```

THE record-emission idiom: build the record lazily, ONLY if the sink consumes it.  hal::emitRecord(sink, [&] { DebugRecord r; …populate…; return r; });  This exists so skipping population is the path of least resistance — the same lesson as the legacy escapeJSONString defect (a safety step a caller can forget is a safety step that WILL be forgotten): callers who go through emitRecord() cannot accidentally pay for a record nothing reads. `buildRecord` is any callable returning a DebugRecord (by value; it binds to emit's const&). Calling sites need the full schema header.

*free function, declared at [`include/shulib/hal/telemetry_sink.hpp:142`](../../include/shulib/hal/telemetry_sink.hpp#L142).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

```text

 ITelemetrySink — the diagnostics output seam (master plan §18.1: "one record, many
 sinks"). Every sink — NullSink, TermSink, SdSink, Shul2Sink — sits behind this one
 HAL interface, so the same trace can go to the terminal, an SD blackbox, or the
 SHUL/2 wire without the core knowing which. NullSink (zero-cost) is the
 competition-build default.

 TWO channels ride this one seam:
   * log()  — the LEVELED-MESSAGE channel (§18.3): leveled, subsystem-tagged lines.
     Here since M1; pure virtual (every sink must decide what a message means to it).
   * emit() — the per-tick DebugRecord channel (§18.2), added at M2 (chunk A1) exactly
     as this header promised: as a NON-pure virtual with a default no-op body, so every
     sink written against the M1 seam kept compiling untouched. THAT is what makes the
     addition additive and F9-versionable, never a break — a pure-virtual emit() would
     have broken NullSink, FakeTelemetrySink, and every future implementer at once.
     (Pinned by the message-only-sink additivity test in test/telemetry_sink_test.cpp.)

 ── The null-sink COST mechanism (§18.2 "NullSink ≈ free"), and why this shape ──────
 The subtle failure this seam must prevent: if the tick loop always builds a
 DebugRecord and hands it to whatever sink is installed, a competition build pays the
 FULL population cost (~30 field writes, pose copies, per-wheel reads) every tick just
 to have NullSink discard it. So the seam carries a cheap consumer query:

     wantsRecord()  — "would emit() do anything with a record?"

 and the emitRecord() helper below makes lazy population the DEFAULT idiom: the record
 is built by a callable that is INVOKED ONLY IF the sink wants it. With NullSink the
 per-tick cost is one devirtualizable bool query — population never happens at all
 (pinned by the builder-not-invoked test in test/debug_record_test.cpp).

 Alternatives considered and rejected (chunk A1 decision log):
   * Always-build-and-emit — pays full population cost to discard; rejected outright.
   * A compile-time sink policy (template / #ifdef) — truly zero-cost, but it bifurcates
     the build, breaks the ONE runtime seam F4 froze (sinks must be swappable behind
     ITelemetrySink& at runtime — dev builds switch sinks without recompiling the core),
     and buys ~nothing: the residual cost is one virtual call per 10ms tick.
   * wantsRecord() defaulting to TRUE (only NullSink opts out) — then every message-only
     sink silently pays record population for a no-op emit(). The defaults must AGREE:
     the default emit() is a no-op, so the default wantsRecord() is false. A sink that
     overrides emit() MUST override wantsRecord() too — override them as a PAIR.

 Concurrency contract (explicit, because the legacy logger's flush raced a background
 task): this seam has no background anything. Every log()/emit() call runs synchronously
 on the CALLER's task, and implementations must document their own thread-safety (the
 shipped sinks are single-task by contract). Implementations MUST NOT throw.
```

</details>
