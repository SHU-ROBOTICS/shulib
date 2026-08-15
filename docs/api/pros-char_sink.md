<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/char_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `char_sink.hpp`

ProsCharSink — ICharSink over the V5's USB serial (chunk R1a): where TermSink's diagnostic bytes physically go on the robot (`pros terminal` displays them).

This header declares **1** type (2 members).

Extracted from [`include/shulib/hal/pros/char_sink.hpp`](../../include/shulib/hal/pros/char_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsCharSink`](#class-proscharsink)
  - [`ProsCharSink`](#proscharsink-proscharsink)
  - [`write`](#proscharsink-write)

<a id="class-proscharsink"></a>

## `class ProsCharSink`

```cpp
class ProsCharSink final : public ICharSink
```

The on-robot ICharSink: TermSink's diagnostic bytes onto the V5's USB serial, where `pros terminal` displays them. Writes through newlib stdout with fwrite, so it pulls in <cstdio> and no PROS header at all. FLUSHES ON EVERY write — boot-banner visibility is worth more here than buffered throughput, and the diagnostics layer above already rate-limits. Cannot throw, as the ICharSink contract requires.

*class, declared at [`include/shulib/hal/pros/char_sink.hpp:42`](../../include/shulib/hal/pros/char_sink.hpp#L42).*

<a id="proscharsink-proscharsink"></a>

### `ProsCharSink::ProsCharSink`

```cpp
explicit ProsCharSink(std::FILE* out = stdout)
```

`out` must outlive the sink; defaults to the V5 USB serial (stdout).

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:45`](../../include/shulib/hal/pros/char_sink.hpp#L45).*

<a id="proscharsink-write"></a>

### `ProsCharSink::write`

```cpp
void write(std::string_view text) override
```

Verbatim bytes + flush. MUST NOT throw (contract) — and cannot.

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:48`](../../include/shulib/hal/pros/char_sink.hpp#L48).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 26 lines</summary>

```text

 ProsCharSink — ICharSink over the V5's USB serial (chunk R1a): where
 TermSink's diagnostic bytes physically go on the robot (`pros terminal`
 displays them).

 BINDS: newlib stdout via std::fwrite + std::fflush. The PROS kernel wires
 stdout to the USB serial, so this adapter deliberately includes NO <pros/*>
 header at all — it lives under hal/pros/ because it is the ON-ROBOT sink
 (and so the PROS-free guard's one exemption covers it if it ever needs the
 direct serial API), but its only dependency is <cstdio>. This is the same
 sink main.cpp carried privately as StdoutCharSink since C7, promoted to the
 adapter tree so tests and future consumers share one implementation.

 CONTRACT (char_sink.hpp:31-32): bytes verbatim, synchronous on the caller's
 task, MUST NOT throw — fwrite/fflush cannot throw. One write() call carries
 one complete line (the FORMATTER's framing), and fwrite is atomic per call
 at this layer, so lines never interleave.

 FLUSH PER WRITE, deliberately: boot-banner visibility is worth more than
 buffered throughput here, and the diagnostics layer already rate-limits
 (the C7 ruling, carried forward unchanged).

 The FILE* is injectable (default stdout) so a host test can hand it a
 tmpfile() and assert exact bytes — the same injected-device pattern that
 made TermSink's output a testable claim in the first place (char_sink.hpp
 header).
```

</details>
