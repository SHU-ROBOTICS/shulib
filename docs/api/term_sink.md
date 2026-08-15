<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/term_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `term_sink.hpp`

TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface.

This header declares **1** type (5 members).

Extracted from [`include/shulib/diag/term_sink.hpp`](../../include/shulib/diag/term_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class TermSink`](#class-termsink)
  - [`TermSink`](#termsink-termsink)
  - [`log`](#termsink-log)
  - [`wantsRecord`](#termsink-wantsrecord)
  - [`emit`](#termsink-emit)
  - [`summarize`](#termsink-summarize)

<a id="class-termsink"></a>

## `class TermSink`

```cpp
class TermSink final : public hal::ITelemetrySink
```

The human-readable terminal stream and the primary dev surface: leveled tagged lines from log(), one column-aligned line per DebugRecord from emit(), and the one-screen block from summarize(). It holds NO mutable state — no buffer, no queue, no background flush — so every LINE is formatted into a stack-local buffer and handed to the ICharSink as exactly ONE write(): one for log(), one for emit(), six for summarize()'s block. A line can therefore never be torn or interleaved mid-way, whatever a caller's string contains; whole LINES can be, so two tasks sharing a sink can split summarize()'s six. Nothing here allocates. Every line is pinned by a golden test, which is why the character device is injected rather than hard-coded to stdout. This is a DISPLAY edge: degrees are rendered here and only here, and only the headline fields appear — the full record belongs to the blackbox and SHUL/2 sinks.

*class, declared at [`include/shulib/diag/term_sink.hpp:77`](../../include/shulib/diag/term_sink.hpp#L77).*

<a id="termsink-termsink"></a>

### `TermSink::TermSink`

```cpp
TermSink(hal::IClock& clock, hal::ICharSink& out) noexcept
```

Both references must outlive the sink. `clock` stamps log() lines; emit() lines are stamped from the record itself (see header).

*function, declared at [`include/shulib/diag/term_sink.hpp:81`](../../include/shulib/diag/term_sink.hpp#L81).*

<a id="termsink-log"></a>

### `TermSink::log`

```cpp
void log(hal::LogLevel level, std::string_view subsystem, std::string_view message) override
```

One leveled, subsystem-tagged line, stamped from the injected CLOCK (unlike emit(), which stamps from the record). Info carries no level tag — the common case stays quiet — while the others render [LEVEL] butted against [TAG]. Both `subsystem` and `message` are caller-controlled text and are SANITIZED on the way out: control bytes become '?', bytes >= 0x80 pass through, and over-long text truncates with '…' at a UTF-8 boundary (16 and 200 bytes). There is no unsanitized path to the device, so no caller string can break the one-line-per-call framing or escape into the terminal.

*function, declared at [`include/shulib/diag/term_sink.hpp:90`](../../include/shulib/diag/term_sink.hpp#L90).*

<a id="termsink-wantsrecord"></a>

### `TermSink::wantsRecord`

```cpp
[[nodiscard]] bool wantsRecord() const noexcept override
```

TermSink consumes records — overridden as a pair with emit(), per the seam contract.

*function, declared at [`include/shulib/diag/term_sink.hpp:103`](../../include/shulib/diag/term_sink.hpp#L103).*

<a id="termsink-emit"></a>

### `TermSink::emit`

```cpp
void emit(const DebugRecord& r) override
```

One line per tick, stamped from the RECORD's own `t` and not the clock — which is what makes a replayed record render byte-identically to a live one. The tick is tagged [MOT] when it carries a command id OR a non-idle motion state (a C1 motion running standalone has id 0, so discriminating on the id alone once rendered an ACTIVE motion as "[LOC] idle"), and "[LOC] idle" only when both are zero. Trailing flags are appended ONLY when set: " DR", " SFB", " CLMP", " flt=NAME". Angles print in degrees, and a non-finite value renders as a deterministic "NaN"/"+Inf"/"-Inf" token rather than libc's locale-varying spelling.

*function, declared at [`include/shulib/diag/term_sink.hpp:112`](../../include/shulib/diag/term_sink.hpp#L112).*

<a id="termsink-summarize"></a>

### `TermSink::summarize`

```cpp
void summarize(const RunSummary& s) override
```

Render the §18.3 one-screen run-summary block. UNSTAMPED by design — the block is a run artifact, not a timed event (the sketch shows no [t=]); each of its six lines is one write() (the framing contract). Field renderings (chosen at C5, each pinned by golden test): * heading values render "n/a" when hasHeadingData is false — the block never fabricates a 0.0° it has no data for * an EMPTY buildHash renders the literal token MISSING (never a plausible-looking placeholder — §18.5's loudness rule) * the drop counters are ALWAYS shown, zeros included: "dropped 0 rec 0 ln" is a positive health claim, not noise (D-2: silence is the bug) * first fault carries its latch time ("ODO_STUCK@  4.2s") — the 2am root-cause line

*function, declared at [`include/shulib/diag/term_sink.hpp:179`](../../include/shulib/diag/term_sink.hpp#L179).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 TermSink — the human-readable terminal stream, the PRIMARY dev/debug surface (master
 plan §18.3; WS13, chunk A1). A pretty-printer for humans: leveled, subsystem-tagged,
 column-aligned lines — never prose, never the JSON wire (that is Shul2Sink at H1).

 Injected dependencies (the pattern control/ set): the clock stamps leveled messages;
 the character sink is where bytes go — which is what makes every line here a golden-
 testable claim (test/term_sink_test.cpp pins exact output) instead of an eyeballed one.

 ── Output shape (§18.3 target; result lines + summary added at chunk C5) ───────────
   leveled message:  [t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)
                     — Info lines carry NO level tag (the common case stays quiet);
                       Error/Warn/Debug/Trace carry [LEVEL] butted against [TAG].
   per-tick record:  [t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) err(  0.40",  0.20",  0.3°) v(  18.0,   4.0, 0.10) q=0.91
                     — timestamped from the RECORD's t (not the clock), so replayed
                       records render identically to live ones; idle ticks (no active
                       command) are tagged [LOC]. Trailing flags appear only when set:
                       " DR" (dead-reckoning), " SFB" (strafe fallback), " CLMP"
                       (nudge clamped), " flt=NAME" (fault this tick). The line shows
                       the HEADLINE fields; the full record belongs to SdSink/SHUL/2.
   run summary:      summarize(RunSummary) renders the §18.3 one-screen block (chunk
                     C5) — six lines, unstamped, byte-pinned by golden test. The
                     per-motion RESULT LINE does not enter here: it rides log() as
                     structured Info text (diag/motion_result.hpp formats it), which
                     is what gives it the exact "[t=…] [MOT] …" §18.3 shape.

 Formatting decisions that keep "column-aligned" true (each pinned by a golden test):
   * Fixed-width numeric columns ([t=%7.2f], %6.1f coords, …). Deviates from the §18.3
     sketch's unpadded [t=12.34] deliberately: alignment across ticks is the actual
     requirement; the sketch is a shape, not a byte spec.
   * Non-finite values render as deterministic right-aligned tokens ("NaN", "+Inf",
     "-Inf") — never libc's locale/sign-varying "nan"/"-nan(0x…)".
   * A value too wide for its column (|v| pathological) re-renders compactly as %.3g
     ("1e+300") — the column widens slightly rather than exploding to 300+ digits.
   * Degrees appear here and only here: this is a DISPLAY edge, the one place F3
     permits radians→degrees.

 Sanitization is UNAVOIDABLE BY CONSTRUCTION — the legacy escapeJSONString lesson (a
 helper the caller must remember is a helper that gets forgotten): every byte of
 caller-controlled text (subsystem, message) enters the line through ONE bounded,
 sanitizing append. Control bytes (< 0x20, 0x7F) become '?' so a stray '\n' or ESC
 sequence can never break the one-line-per-call framing or the terminal; bytes ≥ 0x80
 pass through (UTF-8 is welcome). Over-long messages truncate with '…' at a UTF-8
 boundary. There is no unsanitized path to the device.

 Concurrency contract (the legacy racing-flush, designed OUT rather than fixed): the
 sink holds NO mutable state — no buffer, no queue, no background flush task. Each LINE
 is formatted into a stack-local buffer and handed to the device as exactly one write():
 one for log(), one for emit(), SIX for summarize()'s block. So a line can never be torn
 or interleaved mid-way — but whole lines can be, summarize()'s six included. If multiple
 tasks share one TermSink, ordering across writes is whatever the ICharSink's per-call
 atomicity provides. Nothing here allocates.
```

</details>
