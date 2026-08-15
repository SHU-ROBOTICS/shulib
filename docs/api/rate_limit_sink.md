<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/rate_limit_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `rate_limit_sink.hpp`

RateLimitedSink — per-channel rate limiting with COUNTED, REPORTED drops.

This header declares **2** types (9 members).

Extracted from [`include/shulib/diag/rate_limit_sink.hpp`](../../include/shulib/diag/rate_limit_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct RateLimitConfig`](#struct-ratelimitconfig)
  - [`recordsPerSecond`](#ratelimitconfig-recordspersecond)
  - [`linesPerSecondPerChannel`](#ratelimitconfig-linespersecondperchannel)
- [`class RateLimitedSink`](#class-ratelimitedsink)
  - [`RateLimitedSink`](#ratelimitedsink-ratelimitedsink)
  - [`log`](#ratelimitedsink-log)
  - [`wantsRecord`](#ratelimitedsink-wantsrecord)
  - [`emit`](#ratelimitedsink-emit)
  - [`summarize`](#ratelimitedsink-summarize)
  - [`droppedRecords`](#ratelimitedsink-droppedrecords)
  - [`droppedLines`](#ratelimitedsink-droppedlines)

<a id="struct-ratelimitconfig"></a>

## `struct RateLimitConfig`

```cpp
struct RateLimitConfig
```

The two per-second budgets. Both are TERMINAL-BANDWIDTH choices rather than hardware limits — a 115200-baud console renders roughly 120 of these lines a second — so retuning them per session is expected, not exceptional. Each bucket holds one second's worth and STARTS FULL, so a burst at t=0 passes before throttling bites.

*struct, declared at [`include/shulib/diag/rate_limit_sink.hpp:62`](../../include/shulib/diag/rate_limit_sink.hpp#L62).*

<a id="ratelimitconfig-recordspersecond"></a>

### `RateLimitConfig::recordsPerSecond`

```cpp
double recordsPerSecond = 50.0
```

emit()-channel budget, records/second. Must be > 0 and finite.

*field, declared at [`include/shulib/diag/rate_limit_sink.hpp:64`](../../include/shulib/diag/rate_limit_sink.hpp#L64).*

<a id="ratelimitconfig-linespersecondperchannel"></a>

### `RateLimitConfig::linesPerSecondPerChannel`

```cpp
double linesPerSecondPerChannel = 20.0
```

log()-channel budget PER SUBSYSTEM TAG, lines/second (Info/Debug/Trace only — Error/Warn are exempt; header note). Must be > 0 and finite.

*field, declared at [`include/shulib/diag/rate_limit_sink.hpp:67`](../../include/shulib/diag/rate_limit_sink.hpp#L67).*

<a id="class-ratelimitedsink"></a>

## `class RateLimitedSink`

```cpp
class RateLimitedSink final : public hal::ITelemetrySink
```

A pass-through ITelemetrySink decorator that caps what each channel may forward per second — and COUNTS, STAMPS and ANNOUNCES everything it drops, because a silent drop reads as "nothing happened", which is how an afternoon is lost to a problem that was never there. Error and Warn lines and summarize() are never throttled. Holds `inner` and `clock` by NON-OWNING reference; both must outlive the sink. Single-task by contract, like every sink in this tree, and it allocates nothing.

*class, declared at [`include/shulib/diag/rate_limit_sink.hpp:76`](../../include/shulib/diag/rate_limit_sink.hpp#L76).*

<a id="ratelimitedsink-ratelimitedsink"></a>

### `RateLimitedSink::RateLimitedSink`

```cpp
RateLimitedSink(hal::ITelemetrySink& inner, hal::IClock& clock, const RateLimitConfig& config = {})
```

`inner` and `clock` must outlive the sink.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:79`](../../include/shulib/diag/rate_limit_sink.hpp#L79).*

<a id="ratelimitedsink-log"></a>

### `RateLimitedSink::log`

```cpp
void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override
```

Forward one line unless this subsystem's bucket is empty. Error and Warn ALWAYS pass — a throttled fault is a lost root cause. Budgets are PER TAG, in a bounded table of 16; a 17th tag, a tag over 16 bytes, and the empty tag all share ONE overflow bucket (bounded memory beats fairness for a hypothetical tag, and the sharing is documented rather than silent). When a throttled tag resumes, ONE Warn "throttled TAG: dropped N lines" goes out under the "DIA" tag BEFORE the resuming line, so the gap is explained exactly where it sits.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:97`](../../include/shulib/diag/rate_limit_sink.hpp#L97).*

<a id="ratelimitedsink-wantsrecord"></a>

### `RateLimitedSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

Forwards the INNER answer even when the bucket is empty (header cost note: drops must be seen to be counted) — the pair rule, one level up.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:130`](../../include/shulib/diag/rate_limit_sink.hpp#L130).*

<a id="ratelimitedsink-emit"></a>

### `RateLimitedSink::emit`

```cpp
void emit(const DebugRecord& record) override
```

Forward one record unless the record bucket is empty, STAMPING the running drop totals onto the copy that survives — so a gap in the stream carries its own explanation in the records around it, with no second channel to correlate. The caller has ALREADY paid to populate `record` (see wantsRecord()): throttling here buys bandwidth, not the cost of building it.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:137`](../../include/shulib/diag/rate_limit_sink.hpp#L137).*

<a id="ratelimitedsink-summarize"></a>

### `RateLimitedSink::summarize`

```cpp
void summarize(const RunSummary& summary) override
```

NEVER throttled (header contract): the one-per-run summary must always land.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:153`](../../include/shulib/diag/rate_limit_sink.hpp#L153).*

<a id="ratelimitedsink-droppedrecords"></a>

### `RateLimitedSink::droppedRecords`

```cpp
[[nodiscard]] std::uint32_t droppedRecords() const noexcept
```

Cumulative counts since construction — the summary's "dropped N rec M ln".

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:156`](../../include/shulib/diag/rate_limit_sink.hpp#L156).*

<a id="ratelimitedsink-droppedlines"></a>

### `RateLimitedSink::droppedLines`

```cpp
[[nodiscard]] std::uint32_t droppedLines() const noexcept
```

Info/Debug/Trace lines dropped since construction, summed over ALL tags including the shared overflow bucket. Error and Warn are never throttled, so they can never appear in this number — a non-zero count is always lost detail, never a lost fault.

*function, declared at [`include/shulib/diag/rate_limit_sink.hpp:160`](../../include/shulib/diag/rate_limit_sink.hpp#L160).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 42 lines</summary>

```text

 RateLimitedSink — per-channel rate limiting with COUNTED, REPORTED drops
 (diagnostics-plan D-2; §18.2 "rate-budgeted" / §18.3 "throttled"; WS13, chunk C5).

 Why: a high-rate channel (a 100 Hz record stream, a chatty Debug tag) must not
 drown the terminal or push the loop over budget — but a SILENT drop reads as
 "nothing happened", which is how an afternoon is spent debugging a problem that
 was never there. So every drop is:
   1. COUNTED   — cumulative droppedRecords()/droppedLines() accessors,
   2. ON-WIRE   — the counts are stamped into the D-2 schema fields of every
                  record this sink FORWARDS (a gap in the stream carries its own
                  explanation in the surviving records themselves),
   3. ANNOUNCED — when a line-channel drop episode ends, ONE Warn notice names
                  the channel and the count ("throttled MOT: dropped 47 lines"),
   4. SUMMARIZED — RunReporter reads the totals into the §18.3 summary block.

 What is NEVER throttled (each a deliberate contract, pinned by test):
   * Error and Warn lines — the error path is sacred; FaultLatch lines ride
     log(Error) and a throttled fault is a lost root cause.
   * summarize() — one struct per run is not a rate problem, and eating the
     summary would be the decorator-swallows-it bug the seam warns about.

 Mechanism: token buckets (capacity = one second's budget, starting full,
 continuous refill on the injected clock — deterministic under FakeClock).
 Line channels are keyed by subsystem tag in a bounded table; tags beyond the
 table share one overflow bucket (bounded memory beats per-tag fairness for
 hypothetical tag #17, and the sharing is documented rather than silent).

 Cost note (the A1 contract, stated honestly): wantsRecord() forwards the INNER
 sink's answer even when the record bucket is empty — a throttled tick still
 pays record population so the drop can be SEEN and counted at emit(). Counting
 inside wantsRecord() instead would make a query with no call-count contract
 mutate state (unreliable), and skipping population would make drops invisible —
 the exact failure D-2 exists to prevent. With NullSink inner, wantsRecord() is
 false and nothing is built or counted: the competition build stays free.

 The default budgets are TERMINAL-BANDWIDTH choices (a 115200-baud serial console
 renders ~120 of these lines/s; half a stream of 100 Hz records is plenty for a
 live eye), not hardware claims — logic constants, no register entry, and any
 dev can retune them per session via the config.

 Single-task by contract, like every sink in this tree.
```

</details>
