<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/level_filter_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `level_filter_sink.hpp`

LevelFilterSink — per-subsystem log levels.

This header declares **1** type (8 members).

Extracted from [`include/shulib/diag/level_filter_sink.hpp`](../../include/shulib/diag/level_filter_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class LevelFilterSink`](#class-levelfiltersink)
  - [`LevelFilterSink`](#levelfiltersink-levelfiltersink)
  - [`setGlobalLevel`](#levelfiltersink-setgloballevel)
  - [`setLevel`](#levelfiltersink-setlevel)
  - [`clearLevels`](#levelfiltersink-clearlevels)
  - [`log`](#levelfiltersink-log)
  - [`wantsRecord`](#levelfiltersink-wantsrecord)
  - [`emit`](#levelfiltersink-emit)
  - [`summarize`](#levelfiltersink-summarize)

<a id="class-levelfiltersink"></a>

## `class LevelFilterSink`

```cpp
class LevelFilterSink final : public hal::ITelemetrySink
```

A RUNTIME per-subsystem log-level dial, as a decorator wrapped around any other sink: turn [MOT] up to Debug while holding [LOC] at Warn, with no rebuild. A line passes iff its level is at least as severe as its channel's threshold, which is the TAG's override if one is set and otherwise the global level — default Trace, so a freshly constructed filter is transparent. Only the leveled-message channel is filtered; records, summaries and wantsRecord() forward untouched. A blocked line is NOT a drop and is never tallied as one: the operator asked not to see it, which is configuration, not the involuntary degradation RateLimitedSink counts. Fixed capacity, no heap; single-task by contract, like the rest of diag/.

*class, declared at [`include/shulib/diag/level_filter_sink.hpp:53`](../../include/shulib/diag/level_filter_sink.hpp#L53).*

<a id="levelfiltersink-levelfiltersink"></a>

### `LevelFilterSink::LevelFilterSink`

```cpp
explicit LevelFilterSink(hal::ITelemetrySink& inner) noexcept
```

`inner` must outlive the filter. Transparent until configured (global default Trace: everything passes).

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:57`](../../include/shulib/diag/level_filter_sink.hpp#L57).*

<a id="levelfiltersink-setgloballevel"></a>

### `LevelFilterSink::setGlobalLevel`

```cpp
void setGlobalLevel(hal::LogLevel level) noexcept
```

Threshold for every tag WITHOUT an override. Trace = pass everything.

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:60`](../../include/shulib/diag/level_filter_sink.hpp#L60).*

<a id="levelfiltersink-setlevel"></a>

### `LevelFilterSink::setLevel`

```cpp
void setLevel(std::string_view subsystem, hal::LogLevel level)
```

Per-subsystem override. Re-setting an existing tag updates it in place. Precondition: tag non-empty, ≤ kMaxTagBytes; table not full (LOUD, never a silently ignored dial — kMaxOverrides is far past any real tag census).

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:65`](../../include/shulib/diag/level_filter_sink.hpp#L65).*

<a id="levelfiltersink-clearlevels"></a>

### `LevelFilterSink::clearLevels`

```cpp
void clearLevels() noexcept
```

Drop every override (the global level stays).

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:85`](../../include/shulib/diag/level_filter_sink.hpp#L85).*

<a id="levelfiltersink-log"></a>

### `LevelFilterSink::log`

```cpp
void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override
```

Pass the line to the inner sink iff `level` is at least as severe as `subsystem`'s threshold — an unrecognised tag simply has no override and gets the global level, so there is nothing to register in advance. A blocked line is discarded here and counted nowhere (see the class note). Throws nothing the inner sink does not: the seam forbids it.

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:91`](../../include/shulib/diag/level_filter_sink.hpp#L91).*

<a id="levelfiltersink-wantsrecord"></a>

### `LevelFilterSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

Whatever the inner sink answers — this decorator never suppresses record POPULATION. Answered as a PAIR with emit(), which is the seam's rule: overriding one without the other is how a sink ends up paying to build records it then throws away.

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:101`](../../include/shulib/diag/level_filter_sink.hpp#L101).*

<a id="levelfiltersink-emit"></a>

### `LevelFilterSink::emit`

```cpp
void emit(const DebugRecord& record) override
```

Forwarded untouched. Per-tick records are DATA, not chatter, so no level threshold applies to them; the record stream's own dial is RateLimitedSink.

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:104`](../../include/shulib/diag/level_filter_sink.hpp#L104).*

<a id="levelfiltersink-summarize"></a>

### `LevelFilterSink::summarize`

```cpp
void summarize(const RunSummary& summary) override
```

Forwarded untouched, and forwarded deliberately: the base's summarize() is a no-op body, so a decorator that failed to override it would silently EAT the end-of-run summary.

*function, declared at [`include/shulib/diag/level_filter_sink.hpp:107`](../../include/shulib/diag/level_filter_sink.hpp#L107).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 33 lines</summary>

```text

 LevelFilterSink — per-subsystem log levels (diagnostics-plan D-1; WS13, chunk C5).

 The single most-used debugging move in practice: turn [MOT] up to DEBUG while
 holding [LOC] at WARN. Before this decorator, levels were global — chasing one
 subsystem meant either drowning in every other subsystem's chatter or a REBUILD
 with different compile-time levels. This is a RUNTIME dial on the sink chain:

     TermSink term{clock, out};
     LevelFilterSink filtered{term};
     filtered.setLevel("LOC", LogLevel::Warn);   // quiet the localizer…
     filtered.setLevel("MOT", LogLevel::Debug);  // …while watching the motion layer

 Semantics (each pinned by test/level_filter_test.cpp):
   * A line passes iff its level is AT OR ABOVE the channel's threshold in
     severity (Error > Warn > Info > Debug > Trace). setLevel(tag, Warn) means
     "from tag, Warn and Error only".
   * The threshold is the TAG's override if one is set, else the global level
     (default Trace = everything passes — the decorator is transparent until told
     otherwise).
   * Filtering affects ONLY the leveled-message channel: records, summaries, and
     wantsRecord() forward untouched (they are data, not chatter — and the record
     stream has its own dial, RateLimitedSink).

 FILTERING IS NOT DROPPING (the D-2 distinction, stated deliberately): a filtered
 line is one the operator ASKED not to see — explicit configuration, not silent
 degradation — so it is not counted in any dropped tally. Throttling (D-2) is
 involuntary and therefore counted. Conflating the two would bury real drops in
 requested quiet.

 Fixed capacity, no heap: overrides live in a bounded table; exceeding it is a
 LOUD precondition, never a silently ignored setLevel. Single-task by contract,
 like the rest of diag/.
```

</details>
