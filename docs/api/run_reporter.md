<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/run_reporter.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `run_reporter.hpp`

RunReporter — the glue that makes a run LEGIBLE end to end (WS13, chunk C5): session header (§18.5) → per-motion result lines (§18.3/§18.4) → run summary (§18.3).

This header declares **1** type (9 members).

Extracted from [`include/shulib/motion/run_reporter.hpp`](../../include/shulib/motion/run_reporter.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class RunReporter`](#class-runreporter)
  - [`RunReporter`](#runreporter-runreporter)
  - [`~RunReporter`](#runreporter-destructor-runreporter)
  - [`RunReporter (overload 2)`](#runreporter-runreporter-2)
  - [`RunReporter (overload 3)`](#runreporter-runreporter-3)
  - [`operator=`](#runreporter-operator-eq)
  - [`operator= (overload 2)`](#runreporter-operator-eq-2)
  - [`sessionStart`](#runreporter-sessionstart)
  - [`onMotionComplete`](#runreporter-onmotioncomplete)
  - [`finishRun`](#runreporter-finishrun)

<a id="class-runreporter"></a>

## `class RunReporter`

```cpp
class RunReporter final : public IMotionObserver
```

The glue that makes one run legible end to end: a session header first, a result line at every motion boundary, a summary at the end. It formats nothing itself — diag/ owns the vocabulary and the formatters — and it remembers almost nothing: apart from the provenance strings and the starting battery voltage, everything the summary reports is read LIVE off the scheduler and its deps at finishRun().  Result lines are STRUCTURAL rather than remembered: construction attaches the reporter as the scheduler's boundary observer and destruction detaches it, so settle, timeout, cancel, fault abort and pre-empt each emit their line with no per-verb call a routine could forget.  ONE reporter and ONE scheduler per run — the ordinary auton shape. The scheduler's counters are lifetime-cumulative and the fault latch clears only at explicit run boundaries, so driving a second run through the same pair reports the first run's totals over again. Single-task by contract, and it never throws into the scheduler: an observer that threw would abort the very motion it exists to describe.

*class, declared at [`include/shulib/motion/run_reporter.hpp:82`](../../include/shulib/motion/run_reporter.hpp#L82).*

<a id="runreporter-runreporter"></a>

### `RunReporter::RunReporter`

```cpp
RunReporter(hal::ITelemetrySink& out, MotionScheduler& sched, const diag::RateLimitedSink* limiter = nullptr, const diag::SdSink* blackbox = nullptr) noexcept
```

`out` is where the report goes (see header: the UNTHROTTLED head); `sched` is the run's scheduler — the reporter self-attaches as its boundary observer. `limiter`, when given, contributes the D-2 drop totals to the summary (nullptr = no limiter in the chain = zeros); `blackbox`, when given, contributes the E1 blackbox's own drop count so a file with gaps in it says so on the terminal too (nullptr = no blackbox = the summary stays silent about one, rather than claiming a healthy zero for something that never ran). All must outlive the reporter.

*function, declared at [`include/shulib/motion/run_reporter.hpp:92`](../../include/shulib/motion/run_reporter.hpp#L92).*

<a id="runreporter-destructor-runreporter"></a>

### `RunReporter::~RunReporter`

```cpp
~RunReporter() override
```

Detaches from the scheduler, but only while the scheduler still points at THIS reporter: if something else took the observer slot in the meantime, that one is left attached rather than silently unhooked. The scheduler must outlive the reporter: this destructor reads it, so tearing the scheduler down first is a use-after-free rather than a quiet no-op.

*function, declared at [`include/shulib/motion/run_reporter.hpp:103`](../../include/shulib/motion/run_reporter.hpp#L103).*

<a id="runreporter-runreporter-2"></a>

### `RunReporter::RunReporter (overload 2)`

```cpp
RunReporter(const RunReporter&) = delete
```

Neither copyable nor movable: the scheduler holds a raw back-pointer to this exact object, installed by the constructor and by nothing else. A copy would therefore never register — the one observer slot would still hold the ORIGINAL, and the copy would be a silent second reporter that emits a header and a summary but never a single result line (its destructor's identity check correctly declines to unhook the original on the way out). A move is worse: the members are raw pointers, so the scheduler would be left aimed at the husk that was moved out of. Construct it where it will live.

*function, declared at [`include/shulib/motion/run_reporter.hpp:116`](../../include/shulib/motion/run_reporter.hpp#L116).*

<a id="runreporter-runreporter-3"></a>

### `RunReporter::RunReporter (overload 3)`

```cpp
RunReporter(RunReporter&&) = delete
```

*Covered by the comment on [`RunReporter (overload 2)`](#runreporter-runreporter-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/run_reporter.hpp:117`](../../include/shulib/motion/run_reporter.hpp#L117).*

<a id="runreporter-operator-eq"></a>

### `RunReporter::operator=`

```cpp
RunReporter& operator=(const RunReporter&) = delete
```

*Covered by the comment on [`RunReporter (overload 2)`](#runreporter-runreporter-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/run_reporter.hpp:118`](../../include/shulib/motion/run_reporter.hpp#L118).*

<a id="runreporter-operator-eq-2"></a>

### `RunReporter::operator= (overload 2)`

```cpp
RunReporter& operator=(RunReporter&&) = delete
```

*Covered by the comment on [`RunReporter (overload 2)`](#runreporter-runreporter-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/run_reporter.hpp:119`](../../include/shulib/motion/run_reporter.hpp#L119).*

<a id="runreporter-sessionstart"></a>

### `RunReporter::sessionStart`

```cpp
void sessionStart(const diag::SessionInfo& info)
```

Emit the §18.5 session header — call FIRST, before any motion, so provenance is the first thing in every log (§18.5: "first record of every run"). Battery start is READ here (a live value, not caller homework) and remembered for the summary's start→end pair; the hash and routine id are re-copied into bounded storage for the summary (the caller's string_views are not retained).

*function, declared at [`include/shulib/motion/run_reporter.hpp:127`](../../include/shulib/motion/run_reporter.hpp#L127).*

<a id="runreporter-onmotioncomplete"></a>

### `RunReporter::onMotionComplete`

```cpp
void onMotionComplete(const CompletedMotion& completed) override
```

The scheduler's boundary callback: one §18.3 result line per finished motion, translated to §18.4's boundary vocabulary (header note).

*function, declared at [`include/shulib/motion/run_reporter.hpp:136`](../../include/shulib/motion/run_reporter.hpp#L136).*

<a id="runreporter-finishrun"></a>

### `RunReporter::finishRun`

```cpp
void finishRun()
```

Assemble the §18.3 run summary from live state and hand it to the sink's summarize() channel (TermSink renders the block). Call once, at run end.

*function, declared at [`include/shulib/motion/run_reporter.hpp:152`](../../include/shulib/motion/run_reporter.hpp#L152).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 RunReporter — the glue that makes a run LEGIBLE end to end (WS13, chunk C5):
 session header (§18.5) → per-motion result lines (§18.3/§18.4) → run summary
 (§18.3). This is the class that closes M2's "the run is legible in real time
 on the terminal" clause.

     TermSink term{clock, out};
     Chassis chassis{deps, pacer};
     RunReporter report{term, chassis.scheduler()};
     report.sessionStart({.buildHash = diag::compiledBuildHash(),
                          .routineId = "redLeftTall", .alliance = "red",
                          .side = "left", .portMap = "L1,2,3 R4,5,6 IMU10"});
     …the auton…
     report.finishRun();

 ── Why it lives in motion/, not diag/ ──────────────────────────────────────────────
 A1 predicted C5's result/summary code would land in diag/. The dependency
 reality discovered here: the result line's DATA SOURCE is CompletedMotion and
 the scheduler's counters — motion-layer types diag/ must never name (diag/ is
 a dependency LEAF; debug_record.hpp's rule). So the split is: diag/ owns the
 VOCABULARY and FORMATTERS (MotionResult, emitResultLine, SessionInfo,
 emitSessionHeader, RunSummary — all diag-level, all golden-testable with hand
 data), and this class is the thin GLUE that feeds them from the scheduler.
 Formatting stays with the schema it formats; glue sits with the data it reads.

 ── Boundary results are STRUCTURAL, not remembered ─────────────────────────────────
 Construction ATTACHES the reporter as the scheduler's boundary observer, so
 every motion boundary — settle, timeout, cancel, fault abort, pre-empt — emits
 its result line with no per-verb calls to forget (the A1 emitRecord lesson;
 destruction detaches, if still attached). §18.4's boundary vocabulary is
 derived here: Cancelled + causal fault ⇒ FAULT_ABORT; Cancelled + preempted ⇒
 SUPERSEDED; bare Cancelled stays CANCELLED.

 ── The sink argument: give it the UNTHROTTLED head ─────────────────────────────────
 Header, result lines, and summary are LOW-RATE run landmarks — a handful per
 run. Wire the reporter directly to the formatter (TermSink), and put D-2's
 RateLimitedSink on the HIGH-RATE path (the deps' record/tick stream) instead:
 a result line eaten by a token bucket that per-tick chatter drained would be
 a landmark lost to noise control. (Info-level result lines through a shared
 throttled head DO get counted if dropped — nothing is ever silent — but the
 recommended wiring never puts them there.)

 ── What the summary reads, and one-run scope ───────────────────────────────────────
 Counters/latch/health/battery are read LIVE at finishRun() from the scheduler
 and its deps (battery END is a reading, not a memory). Scheduler counters are
 lifetime-cumulative and FaultLatch clears only at explicit run boundaries, so:
 ONE reporter + ONE scheduler per run — the normal auton shape. gatingRejects
 counts GPS_GATE_REJECT raises (HealthMonitor raises once per EPISODE, so this
 is episodes, not raw rejected fixes — honest label, E2 refines it).

 Single-task by contract, like everything it composes. Never throws into the
 scheduler (IMotionObserver contract): it only formats and logs.
```

</details>
