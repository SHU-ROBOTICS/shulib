<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/null_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `null_sink.hpp`

NullSink — the zero-cost default ITelemetrySink (§18.1).

This header declares **1** type (1 member).

Extracted from [`include/shulib/hal/null_sink.hpp`](../../include/shulib/hal/null_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class NullSink`](#class-nullsink)
  - [`log`](#nullsink-log)

<a id="class-nullsink"></a>

## `class NullSink`

```cpp
class NullSink final : public ITelemetrySink
```

The competition-build default sink: every channel is dropped. What matters is what it does NOT override — wantsRecord() stays false and emit()/summarize() stay the seam's no-ops, which is what makes it ≈free rather than merely fast. A tick loop going through hal::emitRecord() never even POPULATES a DebugRecord, because the builder callable is not invoked at all; the whole per-tick cost is one bool query the compiler can devirtualize. The ONE load-bearing omission is wantsRecord(): overriding it is what would reintroduce the per-tick population cost this class exists to remove. Overriding emit() alone would not — emitRecord() gates solely on wantsRecord(), so the builder would still never run, which is precisely why the seam says override the two as a PAIR — and summarize() is a once-per-run cold path that costs nothing per tick either way.

*class, declared at [`include/shulib/hal/null_sink.hpp:29`](../../include/shulib/hal/null_sink.hpp#L29).*

<a id="nullsink-log"></a>

### `NullSink::log`

```cpp
void log(LogLevel /*level*/, std::string_view /*subsystem*/, std::string_view /*message*/) override
```

Discards the message. The parameters are unnamed on purpose — nothing is read, so nothing is formatted or copied, and the body inlines to nothing.

*function, declared at [`include/shulib/hal/null_sink.hpp:33`](../../include/shulib/hal/null_sink.hpp#L33).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 10 lines</summary>

```text

 NullSink — the zero-cost default ITelemetrySink (§18.1). Drops everything; the
 competition build routes telemetry here so observability costs ≈nothing when off.

 DELIBERATELY does not override wantsRecord()/emit(): it inherits the seam's defaults
 (wantsRecord() == false, no-op emit()), which is what makes it genuinely ≈free — a
 tick loop using hal::emitRecord() never even POPULATES a DebugRecord for it (the
 §18.2 cost mechanism; see telemetry_sink.hpp's header note). The absence of overrides
 here is also the living proof that the M2 emit() addition was additive: this M1 sink
 compiles untouched.
```

</details>
