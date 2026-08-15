<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/triage.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `triage.hpp`

The D-7 TRIAGE BLOCK — "why did it break", rendered for a human.

This header declares **1** free function.

Extracted from [`include/shulib/diag/triage.hpp`](../../include/shulib/diag/triage.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`emitTriageBlock`](#emittriageblock) — *free function*

<a id="emittriageblock"></a>

## `emitTriageBlock`

```cpp
inline void emitTriageBlock(hal::ITelemetrySink& sink, const blackbox::TriageInfo& info, const DebugRecord& faultTick)
```

Render the D-7 triage block as two [ERROR][TRI] lines (byte shapes pinned by test). Through TermSink they read:  [t=   4.20] [ERROR][TRI] fault ODO_STUCK @  4.20s tick 421 preceding 200 brownout no [t=   4.20] [ERROR][TRI] state pos(  24.0,  36.0) hdg  90.0° q=0.91 DR cmd#7▸1 batt 11.9V  The lines carry no newline of their own: the sink frames them (one write() per line — the A1 framing contract), and a stray newline in a message would be sanitized to '?' anyway.

*free function, declared at [`include/shulib/diag/triage.hpp:42`](../../include/shulib/diag/triage.hpp#L42).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 21 lines</summary>

```text

 The D-7 TRIAGE BLOCK — "why did it break", rendered for a human (diagnostics-plan
 D-7; WS13, chunk E1).

 The end-of-run summary (C5) answers "how did the run go". This answers the other
 question, and it is the one asked at 2am: which fault fired, at what time, on which
 tick, what the robot's state was at that instant, and how many ticks of history the
 flight recorder captured before it.

 Same architecture as every other §18.3 renderer in this directory: diag/ owns the
 VOCABULARY (blackbox::TriageInfo — the SAME struct that goes into the blackbox file)
 and the FORMATTER; the glue that decides WHEN to print it lives with the data
 (motion/run_reporter.hpp calls this at run end, but only when the blackbox actually
 dumped). One record, many renderings: these exact fields are in the file too, so the
 terminal block and the blackbox can never disagree about what happened.

 Why it rides log() at ERROR rather than drawing a box on the character device like
 the run summary: this is a fault report, and §18.4's discipline is that faults are
 structured, leveled, greppable key=value lines — not decoration. It also means a
 message-only sink (no record channel) still receives the triage, and that D-2's rate
 limiter is forbidden from throttling it (Error is exempt by contract).
```

</details>
