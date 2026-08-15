<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/trace.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `trace.hpp`

SHULIB_TRACE — the compile-time TRACE strip.

Extracted from [`include/shulib/diag/trace.hpp`](../../include/shulib/diag/trace.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

This header declares no public types, functions or constants of its own — it exists for the design commentary below, for the includes it gathers, or for preprocessor macros (which this tool does not extract; see the [reference overview](README.md)).

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 SHULIB_TRACE — the compile-time TRACE strip (master plan §18.3; WS13, chunk A1).

 TRACE is the only level allowed on the hot path (per-tick math internals), so it is
 the only level that must be PROVABLY free in a competition build — not merely skipped
 at runtime behind a branch, but compiled out so its ARGUMENT EXPRESSIONS are never
 evaluated (a runtime level check would still pay for building the message).

     SHULIB_TRACE(sink, "MOT", buildExpensiveMessage());   // free unless enabled

 Enablement: dev builds pass -DSHULIB_ENABLE_TRACE; the DEFAULT (no define) is the
 STRIPPED build — the safe default is the free one, and you opt IN to trace cost.

 ── Strip mechanism, and why this one (chunk A1 decision log) ───────────────────────
 The stripped form expands to

     (true ? (void)0 : (void)((sink).log(Trace, subsystem, message)))

 rather than the naive `((void)0)`, for three load-bearing reasons:
   1. GUARANTEED non-evaluation — the language mandates only one branch of ?: is ever
      evaluated, so the arguments cost nothing at ANY optimization level (pinned by the
      side-effect-counter test in test/trace_strip_test.cpp), and the dead branch folds
      away trivially under the competition -Os.
   2. Arguments stay TYPE-CHECKED in every build — a trace call cannot bit-rot into a
      compile error that only surfaces when someone enables tracing months later.
   3. Variables used only in trace calls stay odr-used — no -Wunused-variable churn
      under -Werror when the strip removes their only reader.
 Rejected: `((void)0)` (loses 2 and 3); a runtime `if (level <= threshold)` (still
 evaluates/builds the message — not zero-cost, which is the §18.3 requirement).

 Per-TU note: the flag is tested at preprocessing time in each translation unit, which
 is what lets the test suite prove BOTH behaviors in one binary (one TU default, one TU
 defining SHULIB_ENABLE_TRACE before including this header). A robot build sets the
 flag globally in the build system, never per-file.
```

</details>
