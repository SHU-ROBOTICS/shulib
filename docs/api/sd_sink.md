<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/sd_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `sd_sink.hpp`

SdSink — the BLACKBOX: a binary, versioned, session-stamped record of a run, written to the brain's SD card.

This header declares **4** types (32 members) and **2** constants.

Extracted from [`include/shulib/diag/sd_sink.hpp`](../../include/shulib/diag/sd_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDefaultFlightRingTicks`](#kdefaultflightringticks) — *constant*
- [`kRecommendedBufferBytes`](#krecommendedbufferbytes) — *constant*
- [`struct SdSinkConfig`](#struct-sdsinkconfig)
  - [`enabled`](#sdsinkconfig-enabled)
  - [`streamTicks`](#sdsinkconfig-streamticks)
  - [`dumpOnFault`](#sdsinkconfig-dumponfault)
  - [`flushOnFault`](#sdsinkconfig-flushonfault)
- [`struct SdSinkStorage`](#struct-sdsinkstorage)
  - [`ring`](#sdsinkstorage-ring)
  - [`buffer`](#sdsinkstorage-buffer)
- [`struct SdSinkBuffers`](#struct-sdsinkbuffers)
  - [`ring`](#sdsinkbuffers-ring)
  - [`buffer`](#sdsinkbuffers-buffer)
  - [`view`](#sdsinkbuffers-view)
- [`class SdSink`](#class-sdsink)
  - [`SdSink`](#sdsink-sdsink)
  - [`open`](#sdsink-open)
  - [`log`](#sdsink-log)
  - [`wantsRecord`](#sdsink-wantsrecord)
  - [`emit`](#sdsink-emit)
  - [`summarize`](#sdsink-summarize)
  - [`flush`](#sdsink-flush)
  - [`close`](#sdsink-close)
  - [`markBrownout`](#sdsink-markbrownout)
  - [`triggerDump`](#sdsink-triggerdump)
  - [`droppedFrames`](#sdsink-droppedframes)
  - [`tickFrames`](#sdsink-tickframes)
  - [`recordsSeen`](#sdsink-recordsseen)
  - [`messagesSeen`](#sdsink-messagesseen)
  - [`bytesWritten`](#sdsink-byteswritten)
  - [`bytesBuffered`](#sdsink-bytesbuffered)
  - [`dumped`](#sdsink-dumped)
  - [`brownout`](#sdsink-brownout)
  - [`deviceFailed`](#sdsink-devicefailed)
  - [`closed`](#sdsink-closed)
  - [`ringSize`](#sdsink-ringsize)
  - [`triage`](#sdsink-triage)
  - [`triageTick`](#sdsink-triagetick)

<a id="kdefaultflightringticks"></a>

## `kDefaultFlightRingTicks`

```cpp
inline constexpr std::size_t kDefaultFlightRingTicks = 200
```

D-6's own number: the flight recorder holds the last 200 ticks (~2 s at a 100 Hz loop). PROVISIONAL (A4: HA-58) — an INVENTED depth, not a measured one; R4 settles how far back a real failure's cause actually sits.

*constant, declared at [`include/shulib/diag/sd_sink.hpp:95`](../../include/shulib/diag/sd_sink.hpp#L95).*

<a id="krecommendedbufferbytes"></a>

## `kRecommendedBufferBytes`

```cpp
inline constexpr std::size_t kRecommendedBufferBytes = 65536
```

The recommended RAM byte budget for the staging buffer: 64 KiB. Stated honestly, because the arithmetic matters — a full default dump is a triage frame plus 200 tick frames, about 87 KB, so 64 KiB does NOT hold one: a dump of that size writes in two device calls rather than one (supported and tested). Sizing the buffer to hold a whole dump costs 88 KB of RAM permanently to save one write() call at the one moment the run is already compromised, which is the wrong trade. PROVISIONAL (A4: HA-59) — INVENTED; R4 measures what the brain can spare.

*constant, declared at [`include/shulib/diag/sd_sink.hpp:104`](../../include/shulib/diag/sd_sink.hpp#L104).*

<a id="struct-sdsinkconfig"></a>

## `struct SdSinkConfig`

```cpp
struct SdSinkConfig
```

Configuration for SdSink. Every default is the COMPETITION posture: the flight recorder on, streaming off, dump on the first fault, and write it immediately.

*struct, declared at [`include/shulib/diag/sd_sink.hpp:108`](../../include/shulib/diag/sd_sink.hpp#L108).*

<a id="sdsinkconfig-enabled"></a>

### `SdSinkConfig::enabled`

```cpp
bool enabled = true
```

false ⇒ the sink is inert: wantsRecord() is false, so the record is never even built (A1's cost contract), and no byte is ever written.

*field, declared at [`include/shulib/diag/sd_sink.hpp:111`](../../include/shulib/diag/sd_sink.hpp#L111).*

<a id="sdsinkconfig-streamticks"></a>

### `SdSinkConfig::streamTicks`

```cpp
bool streamTicks = false
```

true ⇒ every record is staged as a Tick frame as it arrives (a bench/dev posture). false ⇒ D-6: records go to the RAM ring only, and reach the file only through a fault dump.

*field, declared at [`include/shulib/diag/sd_sink.hpp:115`](../../include/shulib/diag/sd_sink.hpp#L115).*

<a id="sdsinkconfig-dumponfault"></a>

### `SdSinkConfig::dumpOnFault`

```cpp
bool dumpOnFault = true
```

Dump the flight recorder when a record carries a fault. FIRST fault only — the FaultLatch precedent: a cascade must not dump twenty times.

*field, declared at [`include/shulib/diag/sd_sink.hpp:118`](../../include/shulib/diag/sd_sink.hpp#L118).*

<a id="sdsinkconfig-flushonfault"></a>

### `SdSinkConfig::flushOnFault`

```cpp
bool flushOnFault = true
```

Let the fault dump write to the device immediately (header note). false defers the bytes to the caller's next flush(), at the risk of losing them to a power loss — bounded, counted, and the caller's choice.

*field, declared at [`include/shulib/diag/sd_sink.hpp:122`](../../include/shulib/diag/sd_sink.hpp#L122).*

<a id="struct-sdsinkstorage"></a>

## `struct SdSinkStorage`

```cpp
struct SdSinkStorage
```

Caller-owned storage for one SdSink. NEVER put this on a task stack (header note).

*struct, declared at [`include/shulib/diag/sd_sink.hpp:126`](../../include/shulib/diag/sd_sink.hpp#L126).*

<a id="sdsinkstorage-ring"></a>

### `SdSinkStorage::ring`

```cpp
std::span<DebugRecord> ring{}
```

The D-6 flight-recorder ring. May be empty (no flight recorder).

*field, declared at [`include/shulib/diag/sd_sink.hpp:128`](../../include/shulib/diag/sd_sink.hpp#L128).*

<a id="sdsinkstorage-buffer"></a>

### `SdSinkStorage::buffer`

```cpp
std::span<std::byte> buffer{}
```

The staging buffer — this IS the byte budget. Must hold the header plus one triage frame (checked by precondition).

*field, declared at [`include/shulib/diag/sd_sink.hpp:131`](../../include/shulib/diag/sd_sink.hpp#L131).*

<a id="struct-sdsinkbuffers"></a>

## `struct SdSinkBuffers`

```cpp
template <std::size_t RingTicks, std::size_t BufferBytes> struct SdSinkBuffers
```

The one-liner for the common case: declare it at file scope (or as a static) and hand view() to the sink.  static shulib::diag::SdSinkBuffers<200, 65536> blackboxRam; shulib::diag::SdSink blackbox{card, clock, blackboxRam.view()};

*struct, declared at [`include/shulib/diag/sd_sink.hpp:140`](../../include/shulib/diag/sd_sink.hpp#L140).*

<a id="sdsinkbuffers-ring"></a>

### `SdSinkBuffers::ring`

```cpp
std::array<DebugRecord, RingTicks> ring{}
```

The flight-recorder ring storage.

*field, declared at [`include/shulib/diag/sd_sink.hpp:142`](../../include/shulib/diag/sd_sink.hpp#L142).*

<a id="sdsinkbuffers-buffer"></a>

### `SdSinkBuffers::buffer`

```cpp
std::array<std::byte, BufferBytes> buffer{}
```

The staging-buffer storage.

*field, declared at [`include/shulib/diag/sd_sink.hpp:144`](../../include/shulib/diag/sd_sink.hpp#L144).*

<a id="sdsinkbuffers-view"></a>

### `SdSinkBuffers::view`

```cpp
[[nodiscard]] SdSinkStorage view() noexcept
```

A storage view over both arrays, for the SdSink constructor.

*function, declared at [`include/shulib/diag/sd_sink.hpp:146`](../../include/shulib/diag/sd_sink.hpp#L146).*

<a id="class-sdsink"></a>

## `class SdSink`

```cpp
class SdSink final : public hal::ITelemetrySink
```

The blackbox: a binary, versioned, session-stamped record of a run on the brain's SD card, behind the same ITelemetrySink seam TermSink sits on — one record, two renderings. Its DEFAULT posture writes nothing at all: every record lands in the caller's RAM ring, and bytes reach the device only on the first faulted record, on an explicit flush(), or at close(). Lifecycle: open() once before the run, flush() wherever a few milliseconds of IO is affordable, close() at the end; a clean run that never had anything to say costs zero bytes. It never allocates, never throws, and — outside the fault dump — never writes behind your back: a frame that does not fit the buffer is dropped WHOLE and counted, so the file always explains its own gaps. Single-task, like every sink here.

*class, declared at [`include/shulib/diag/sd_sink.hpp:161`](../../include/shulib/diag/sd_sink.hpp#L161).*

<a id="sdsink-sdsink"></a>

### `SdSink::SdSink`

```cpp
SdSink(hal::IBlockSink& out, hal::IClock& clock, SdSinkStorage storage, const SdSinkConfig& config = {})
```

`out` is the block device (R1's /usd/ adapter on the robot, FakeBlockSink in tests), `clock` stamps the run epoch and the end frame, `storage` is caller-owned (header note). All references must outlive the sink.

*function, declared at [`include/shulib/diag/sd_sink.hpp:166`](../../include/shulib/diag/sd_sink.hpp#L166).*

<a id="sdsink-open"></a>

### `SdSink::open`

```cpp
void open(const SessionInfo& info) noexcept
```

Stamp the run's provenance (§18.5) and take the epoch reading. Call once, before the run. The header is STAGED, not written — a run that never has anything to say still writes nothing at all. An EMPTY build hash stays empty all the way to disk: MISSING must stay loud, and a wrong hash is worse than an absent one.

*function, declared at [`include/shulib/diag/sd_sink.hpp:180`](../../include/shulib/diag/sd_sink.hpp#L180).*

<a id="sdsink-log"></a>

### `SdSink::log`

```cpp
void log(hal::LogLevel /*level*/, std::string_view /*subsystem*/, std::string_view /*message*/) override
```

v1 does not carry the message channel (header note). The line is counted so the omission is visible in the file's end frame rather than silent.

*function, declared at [`include/shulib/diag/sd_sink.hpp:192`](../../include/shulib/diag/sd_sink.hpp#L192).*

<a id="sdsink-wantsrecord"></a>

### `SdSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

True while the sink is enabled — the ring needs every record even when nothing is being streamed. Overridden as a pair with emit(), per the seam contract.

*function, declared at [`include/shulib/diag/sd_sink.hpp:199`](../../include/shulib/diag/sd_sink.hpp#L199).*

<a id="sdsink-emit"></a>

### `SdSink::emit`

```cpp
void emit(const DebugRecord& record) override
```

One tick: stream it if configured, dump on the FIRST faulted record, then push it into the flight ring. The dump runs BEFORE the push on purpose, so the dumped ticks are strictly the ones PRECEDING the fault and the fault tick itself appears exactly once (inside the triage frame).

*function, declared at [`include/shulib/diag/sd_sink.hpp:205`](../../include/shulib/diag/sd_sink.hpp#L205).*

<a id="sdsink-summarize"></a>

### `SdSink::summarize`

```cpp
void summarize(const RunSummary& summary) override
```

The end-of-run summary (§18.3) as a frame. The sink's OWN drop count rides along, so the file always explains its own gaps.

*function, declared at [`include/shulib/diag/sd_sink.hpp:224`](../../include/shulib/diag/sd_sink.hpp#L224).*

<a id="sdsink-flush"></a>

### `SdSink::flush`

```cpp
bool flush() noexcept
```

Push everything staged to the device. THIS is the caller-paced write (T1): call it at a motion boundary, at auton end, or wherever a few milliseconds of IO is affordable. Returns false if the device refused any byte; the staged bytes are dropped (and counted) either way, so a failing device can never grow the buffer.  The cost this whole arrangement rests on: a flush of tens of kilobytes is assumed to take single-digit milliseconds — affordable HERE, and not affordable inside a 10 ms control tick. That assumption is INVENTED and the reason writes are caller-paced at all; PROVISIONAL (A4: HA-60), and R4 measures it. If the real figure is far worse, the flush POINTS move (fewer of them, or auton-end only) — the format and the sink do not.

*function, declared at [`include/shulib/diag/sd_sink.hpp:249`](../../include/shulib/diag/sd_sink.hpp#L249).*

<a id="sdsink-close"></a>

### `SdSink::close`

```cpp
void close() noexcept
```

Graceful end: write the end frame, flush, and flush the device. The end frame's PRESENCE is what tells a reader the run closed cleanly — its absence is how a truncated file identifies itself. Writes nothing at all if the run never had anything to say (D-6's promise: a clean run costs zero bytes).

*function, declared at [`include/shulib/diag/sd_sink.hpp:269`](../../include/shulib/diag/sd_sink.hpp#L269).*

<a id="sdsink-markbrownout"></a>

### `SdSink::markBrownout`

```cpp
void markBrownout() noexcept
```

Latch the brownout marker from outside the record stream (HealthMonitor's brownedOut(), say). Latched for the run: a battery that recovers does not erase the fact that it collapsed.

*function, declared at [`include/shulib/diag/sd_sink.hpp:291`](../../include/shulib/diag/sd_sink.hpp#L291).*

<a id="sdsink-triggerdump"></a>

### `SdSink::triggerDump`

```cpp
bool triggerDump(FaultCode fault, const DebugRecord& faultTick) noexcept
```

Dump the flight recorder explicitly, for a fault that never rode a record. Honours the first-fault rule; returns false if a dump already happened or the sink is disabled.

*function, declared at [`include/shulib/diag/sd_sink.hpp:296`](../../include/shulib/diag/sd_sink.hpp#L296).*

<a id="sdsink-droppedframes"></a>

### `SdSink::droppedFrames`

```cpp
[[nodiscard]] std::uint32_t droppedFrames() const noexcept
```

Frames dropped for want of buffer, plus any staged frames a failed device write discarded. THE number for "what is missing from this file".

*function, declared at [`include/shulib/diag/sd_sink.hpp:308`](../../include/shulib/diag/sd_sink.hpp#L308).*

<a id="sdsink-tickframes"></a>

### `SdSink::tickFrames`

```cpp
[[nodiscard]] std::uint32_t tickFrames() const noexcept
```

Tick frames staged over the run (streamed plus dumped).

*function, declared at [`include/shulib/diag/sd_sink.hpp:310`](../../include/shulib/diag/sd_sink.hpp#L310).*

<a id="sdsink-recordsseen"></a>

### `SdSink::recordsSeen`

```cpp
[[nodiscard]] std::uint32_t recordsSeen() const noexcept
```

Records handed to emit() over the run.

*function, declared at [`include/shulib/diag/sd_sink.hpp:312`](../../include/shulib/diag/sd_sink.hpp#L312).*

<a id="sdsink-messagesseen"></a>

### `SdSink::messagesSeen`

```cpp
[[nodiscard]] std::uint32_t messagesSeen() const noexcept
```

log() lines handed to the sink and not carried by v1 (header note).

*function, declared at [`include/shulib/diag/sd_sink.hpp:314`](../../include/shulib/diag/sd_sink.hpp#L314).*

<a id="sdsink-byteswritten"></a>

### `SdSink::bytesWritten`

```cpp
[[nodiscard]] std::uint32_t bytesWritten() const noexcept
```

Bytes the device confirmed. After a device failure this is a LOWER BOUND: a partial write's prefix is unknowable through the seam.

*function, declared at [`include/shulib/diag/sd_sink.hpp:317`](../../include/shulib/diag/sd_sink.hpp#L317).*

<a id="sdsink-bytesbuffered"></a>

### `SdSink::bytesBuffered`

```cpp
[[nodiscard]] std::size_t bytesBuffered() const noexcept
```

Bytes staged and not yet written.

*function, declared at [`include/shulib/diag/sd_sink.hpp:319`](../../include/shulib/diag/sd_sink.hpp#L319).*

<a id="sdsink-dumped"></a>

### `SdSink::dumped`

```cpp
[[nodiscard]] bool dumped() const noexcept
```

True once the fault dump has fired (first fault only).

*function, declared at [`include/shulib/diag/sd_sink.hpp:321`](../../include/shulib/diag/sd_sink.hpp#L321).*

<a id="sdsink-brownout"></a>

### `SdSink::brownout`

```cpp
[[nodiscard]] bool brownout() const noexcept
```

The latched brownout marker.

*function, declared at [`include/shulib/diag/sd_sink.hpp:323`](../../include/shulib/diag/sd_sink.hpp#L323).*

<a id="sdsink-devicefailed"></a>

### `SdSink::deviceFailed`

```cpp
[[nodiscard]] bool deviceFailed() const noexcept
```

True once any write() or flush() reported failure.

*function, declared at [`include/shulib/diag/sd_sink.hpp:325`](../../include/shulib/diag/sd_sink.hpp#L325).*

<a id="sdsink-closed"></a>

### `SdSink::closed`

```cpp
[[nodiscard]] bool closed() const noexcept
```

True once close() has run.

*function, declared at [`include/shulib/diag/sd_sink.hpp:327`](../../include/shulib/diag/sd_sink.hpp#L327).*

<a id="sdsink-ringsize"></a>

### `SdSink::ringSize`

```cpp
[[nodiscard]] std::size_t ringSize() const noexcept
```

How many records the flight ring currently holds.

*function, declared at [`include/shulib/diag/sd_sink.hpp:329`](../../include/shulib/diag/sd_sink.hpp#L329).*

<a id="sdsink-triage"></a>

### `SdSink::triage`

```cpp
[[nodiscard]] const blackbox::TriageInfo& triage() const noexcept
```

The D-7 triage block for the dump that fired (all zeros until dumped()). The SAME struct that went into the file, so the terminal report (diag/triage.hpp, called by RunReporter at run end) and the blackbox cannot disagree.

*function, declared at [`include/shulib/diag/sd_sink.hpp:333`](../../include/shulib/diag/sd_sink.hpp#L333).*

<a id="sdsink-triagetick"></a>

### `SdSink::triageTick`

```cpp
[[nodiscard]] const DebugRecord& triageTick() const noexcept
```

The record of the tick the fault fired on (all defaults until dumped()).

*function, declared at [`include/shulib/diag/sd_sink.hpp:335`](../../include/shulib/diag/sd_sink.hpp#L335).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 71 lines, click to expand</summary>

```text

 SdSink — the BLACKBOX: a binary, versioned, session-stamped record of a run, written
 to the brain's SD card (master plan §18; diagnostics-plan D-6/D-7; WS13, chunk E1).

 ── Why it exists ───────────────────────────────────────────────────────────────────
 The terminal (A1/C5) is the primary debug surface, and at a competition there is no
 terminal. Without this file a field run is undiagnosable: the robot did something
 wrong, and the only record is what somebody remembers seeing. SdSink is the
 counterpart to TermSink on the same seam — ONE record, MANY sinks — so a field trace
 and a bench trace are the same data in two renderings.

 ── D-6, the flight recorder: always on, written only when something breaks ─────────
 A competition build cannot afford always-on SD writing, but the 200 ticks BEFORE a
 fault are exactly what you need and exactly what you never have. So by default this
 sink STREAMS NOTHING. Every record goes into a fixed RAM ring, overwriting the oldest;
 the device sees no bytes at all until a fault fires. Then, and only then, the sink
 writes: the triage block first, and the preceding ticks after it.

 ── The dump ORDER is a decision, not an accident ───────────────────────────────────
 The fault that triggers a dump may be a brownout — the condition least compatible
 with a long synchronous write — so the order is chosen assuming the write may be cut
 off partway:
   1. the file header (provenance: which binary, which routine),
   2. the TRIAGE frame, which carries the fault code, its time, the tick index, AND
      THE COMPLETE RECORD OF THE FAULT TICK — the single most valuable record in the
      file, written before anything else,
   3. the preceding ticks, OLDEST FIRST.
 A file cut anywhere after step 2 still answers "what broke, when, and in what state".
 REJECTED: newest-first ticks (it protects the most recent ticks against a cut, but it
 puts every reader in reverse and makes a partial file's ordering depend on where the
 cut fell); embedding the fault tick only in the ring (then a cut in step 3 can lose
 the one record that names the failure).

 ── T1: there is no background task, and that is a decision ─────────────────────────
 The build order once specified "double-buffered off-task writes". C2/C4 decided this
 tree has NO background task, and that decision stands here: bytes are encoded into a
 caller-owned RAM buffer synchronously on the caller's task, and reach the device only
 when the caller says so — flush() at a motion boundary, close() at auton end, or the
 fault dump. A writer task would be this tree's first two-task design, is unbuildable
 PROS-free at M2, and would end host determinism: every closed-loop test in this
 project is reproducible from a seed BECAUSE there is exactly one task. The cost of
 caller-paced writing is that a caller who flushes too rarely overruns the byte budget
 — which is why the budget drops and COUNTS (principle 5: silent degradation is a bug).

 ── Bounded, and never auto-flushing ────────────────────────────────────────────────
 The buffer is caller-owned and fixed. When a frame does not fit, it is DROPPED WHOLE
 and counted — never half-written, never grown, and (outside the fault dump) never
 resolved by writing to the device behind the caller's back. An automatic flush would
 put an unpredictable multi-millisecond SD write inside an arbitrary control tick,
 which is the exact cost D-6 exists to avoid. The one exception is the fault dump,
 which MAY write immediately (cfg.flushOnFault, default true): the fault has already
 happened, the run is already compromised, and the evidence is worth one late tick.

 ── Storage is caller-owned, and must not live on a task stack ──────────────────────
 The ring and the buffer are spans the caller provides (the Localizer's corrector-span
 precedent). That is not ceremony: 200 records plus a 64 KB buffer is far more than a
 PROS task stack holds, so the storage must be static / file-scope / heap. The
 SdSinkBuffers helper below is the one-liner for the common case.

 ── Cost when disabled ──────────────────────────────────────────────────────────────
 With cfg.enabled == false, wantsRecord() is false, so hal::emitRecord never even
 BUILDS a record (A1's cost contract), no ring is touched and no byte is written.
 Pinned by test, not asserted in prose.

 ── What v1 does NOT carry ──────────────────────────────────────────────────────────
 The log() message channel (blackbox_format.hpp explains why). Lines handed to this
 sink are counted and the count is written into the end frame, so the omission is
 visible in the file instead of silent.

 Single-task by contract, like every sink in this tree. Nothing here allocates, and
 nothing here throws.
```

</details>
