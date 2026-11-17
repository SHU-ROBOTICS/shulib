<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/char_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `char_sink.hpp`

ProsCharSink, serialLine() and serialWrapped() — the V5's USB serial, where TermSink's diagnostic bytes and every program's own lines physically go (`pros terminal` displays them).

This header declares **2** types (4 members), **4** free functions, and **4** constants.

Extracted from [`include/shulib/hal/pros/char_sink.hpp`](../../include/shulib/hal/pros/char_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kSerialWriteMaxBytes`](#kserialwritemaxbytes) — *constant*
- [`kSerialLineBytes`](#kseriallinebytes) — *constant*
- [`kSerialTagBytes`](#kserialtagbytes) — *constant*
- [`kSerialCutMarker`](#kserialcutmarker) — *constant*
- [`struct PosixWrite`](#struct-posixwrite)
  - [`operator()`](#posixwrite-operator-call)
- [`writeWhole`](#writewhole) — *free function*
- [`class ProsCharSink`](#class-proscharsink)
  - [`ProsCharSink`](#proscharsink-proscharsink)
  - [`ProsCharSink (overload 2)`](#proscharsink-proscharsink-2)
  - [`write`](#proscharsink-write)
- [`serialLine`](#serialline) — *free function*
- [`serialWrapped`](#serialwrapped) — *free function*
- [`serialWrapped (overload 2)`](#serialwrapped-2) — *free function*

<a id="kserialwritemaxbytes"></a>

## `kSerialWriteMaxBytes`

```cpp
inline constexpr std::size_t kSerialWriteMaxBytes = 512
```

The most bytes ONE write() call this header makes sends — writeWhole() offers a longer text in pieces of at most this. PROS's serial stream holds 2047 bytes and a frame waits until it fits whole, holding PROS's write mutex (HA-191): a frame of 512 bytes always fits, so no call made here can wait for ever on its own size — and since PROS copies each frame onto the writer's stack, it also bounds the stack a write() costs (the header's banner). Above every line shulib prints (Line::kCapacity, 384), so each of those is still ONE call; a longer text through the sink is not one line, and arrives as pieces another task's line can land between.

*constant, declared at [`include/shulib/hal/pros/char_sink.hpp:125`](../../include/shulib/hal/pros/char_sink.hpp#L125).*

<a id="kseriallinebytes"></a>

## `kSerialLineBytes`

```cpp
inline constexpr std::size_t kSerialLineBytes = 256
```

The most bytes ONE serialLine() call, or one serialWrapped() piece, sends — its `\n` included. Sized from SER1's census of every line src/ prints (the longest that does not wrap is under 220 bytes); a longer text goes through serialWrapped(), one piece per write. Far under PROS's 2047-byte stream (HA-191), and small enough that the buffer it sizes stays cheap on any task's stack.

*constant, declared at [`include/shulib/hal/pros/char_sink.hpp:131`](../../include/shulib/hal/pros/char_sink.hpp#L131).*

<a id="kserialtagbytes"></a>

## `kSerialTagBytes`

```cpp
inline constexpr std::size_t kSerialTagBytes = 16
```

The most characters of a tag serialWrapped() prints inside its `[` `]` — TermSink's own limit (diag/term_sink.hpp), so the prefix can never crowd the text out of a piece.

*constant, declared at [`include/shulib/hal/pros/char_sink.hpp:135`](../../include/shulib/hal/pros/char_sink.hpp#L135).*

<a id="kserialcutmarker"></a>

## `kSerialCutMarker`

```cpp
inline constexpr std::string_view kSerialCutMarker = "...[cut]\n"
```

What ends a serialLine() whose formatted text did not fit kSerialLineBytes: the text is CUT there, visibly, and the line still ends in one `\n`. serialLine() cannot wrap — vsnprintf never yields the bytes past its buffer — so a line that can be longer belongs to serialWrapped(), which never cuts.

*constant, declared at [`include/shulib/hal/pros/char_sink.hpp:140`](../../include/shulib/hal/pros/char_sink.hpp#L140).*

<a id="struct-posixwrite"></a>

## `struct PosixWrite`

```cpp
struct PosixWrite
```

The one place this header calls `write()`: POSIX's write on `fd`. The default for writeWhole(), and the only one outside a test.

*struct, declared at [`include/shulib/hal/pros/char_sink.hpp:156`](../../include/shulib/hal/pros/char_sink.hpp#L156).*

<a id="posixwrite-operator-call"></a>

### `PosixWrite::operator()`

```cpp
auto operator()(int fd, const char* data, std::size_t size) const noexcept
```

Forwards to `::write(fd, data, size)` and returns its result — the bytes written, or 0 / −1.

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:158`](../../include/shulib/hal/pros/char_sink.hpp#L158).*

<a id="writewhole"></a>

## `writeWhole`

```cpp
template <class WriteFn> std::size_t writeWhole(int fd, std::string_view bytes, WriteFn&& writeFn) noexcept
```

Writes ALL of `bytes` to `fd` through `writeFn` (a callable `(int, const char*, std::size_t)` returning the bytes written, like POSIX write) and returns how many were written. Each call is offered at most kSerialWriteMaxBytes, so a text that long or shorter — every line shulib prints — goes in ONE call when the device takes it, which PROS's serial always does or refuses (HA-189), and a longer one in pieces of that size (no single frame can outgrow PROS's stream, HA-191). On a SHORT write it writes the REMAINDER (never the start again); when a call returns 0 or less — an error, or a device that takes nothing — it STOPS, so it can never spin. A count above what a call was offered counts as what it was offered (never a read past `bytes`). The injectable `writeFn` exists for the host tests' short-write and failure fakes; everything else passes PosixWrite.

*free function, declared at [`include/shulib/hal/pros/char_sink.hpp:173`](../../include/shulib/hal/pros/char_sink.hpp#L173).*

<a id="class-proscharsink"></a>

## `class ProsCharSink`

```cpp
class ProsCharSink final : public ICharSink
```

The on-robot ICharSink: TermSink's diagnostic bytes onto the V5's USB serial, where `pros terminal` displays them. Each write() is ONE direct `write()` on the serial's descriptor — never through stdout's shared, unguarded FILE buffer — so a line stays whole when two tasks log at once (HA-189; the header's banner). A text longer than kSerialWriteMaxBytes goes out in pieces of that size (HA-191). PROS-free: no PROS header at all. Cannot throw, as the ICharSink contract requires.

*class, declared at [`include/shulib/hal/pros/char_sink.hpp:193`](../../include/shulib/hal/pros/char_sink.hpp#L193).*

<a id="proscharsink-proscharsink"></a>

### `ProsCharSink::ProsCharSink`

```cpp
ProsCharSink() noexcept
```

The robot's sink: the V5's USB serial, STDOUT_FILENO. Names no FILE — no `stdout`, no `fileno`, no `fflush` — so a program's objects reference none of stdout's machinery (the src build gate's step 9 checks that). The provider's `usb_{}` is this constructor.

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:198`](../../include/shulib/hal/pros/char_sink.hpp#L198).*

<a id="proscharsink-proscharsink-2"></a>

### `ProsCharSink::ProsCharSink (overload 2)`

```cpp
explicit ProsCharSink(std::FILE* out)
```

A sink over `out`'s DESCRIPTOR — a host test hands it a `tmpfile()`. `out` must be NON-NULL and have a descriptor, and must outlive the sink. FLUSHES `out` ONCE, here, so bytes already in its buffer are not left behind the first direct write; construct it before anything else writes `out`, and never write `out` through stdio afterwards — a direct write passes anything still buffered there. The null check is not decoration: `tmpfile()` returns null on failure, and `fileno(nullptr)` is undefined behaviour rather than a loud contract breach.

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:206`](../../include/shulib/hal/pros/char_sink.hpp#L206).*

<a id="proscharsink-write"></a>

### `ProsCharSink::write`

```cpp
void write(std::string_view text) noexcept override
```

The text, verbatim, in ONE `write()` — in pieces of at most kSerialWriteMaxBytes if it is longer; the remainder after a short write; nothing more after a failed one (writeWhole()). No buffer, no flush, nothing shared between tasks. MUST NOT throw (contract) — and cannot.

*function, declared at [`include/shulib/hal/pros/char_sink.hpp:216`](../../include/shulib/hal/pros/char_sink.hpp#L216).*

<a id="serialline"></a>

## `serialLine`

```cpp
[[gnu::format(printf, 1, 2)]] inline void serialLine(const char* format, ...) noexcept
```

One line to the V5's USB serial, printf-style: formatted into a buffer ON THE CALLER'S STACK (kSerialLineBytes), given exactly ONE trailing `\n` (added when the format does not end in one; never doubled), and sent with ONE `write()` on STDOUT_FILENO — so it stays whole when two tasks print at once (HA-189). A format whose result does not fit is CUT, visibly (kSerialCutMarker): print a text that can be long with serialWrapped(). A null format sends an empty line, and so does a format vsnprintf REFUSES (it returns an error — glibc and newlib both do for a `%ls` character the C locale cannot encode): never the buffer's stale bytes. `%f`, `%e` and `%g` allocate (newlib); the stack it costs is in the header's banner. Cannot throw.

*free function, declared at [`include/shulib/hal/pros/char_sink.hpp:232`](../../include/shulib/hal/pros/char_sink.hpp#L232).*

<a id="serialwrapped"></a>

## `serialWrapped`

```cpp
inline void serialWrapped(const char* tag, std::initializer_list<std::string_view> parts) noexcept
```

A text of ANY length to the V5's USB serial as `[tag] ` lines of at most kSerialLineBytes, ONE `write()` each — wrapped, never cut. `parts` are joined as one text (so a sentence built from a literal, a reason and a literal needs no buffer of its own). A line too long for one piece breaks at a space — each piece as many whole words as fit, the space at the break dropped — and only a word longer than a piece breaks hard, at the piece's end; every later piece of that line begins `[tag] ... `. Only the ONE space at a break is dropped: any other space is the text's own and is kept (a double space at a break leaves its other space at the end of one piece or the start of the next), so nothing else is lost; and a piece that would hold nothing after its `[tag] ... ` is never sent. A `\n` in the text ends a line there (a blank line stays blank). An empty text sends one `[tag] ` line. `tag` may be null (no `[tag] ` prefix); at most kSerialTagBytes of it are printed. A null part's text is the caller's to avoid: a `std::string_view` cannot hold one. The buffer is on the caller's stack (the stack it costs is in the header's banner). Cannot throw.

*free function, declared at [`include/shulib/hal/pros/char_sink.hpp:265`](../../include/shulib/hal/pros/char_sink.hpp#L265).*

<a id="serialwrapped-2"></a>

## `serialWrapped (overload 2)`

```cpp
inline void serialWrapped(const char* tag, std::string_view text) noexcept
```

serialWrapped() for one text: `[tag] text`, wrapped into pieces of at most kSerialLineBytes, one `write()` each (the list form's rules).

*free function, declared at [`include/shulib/hal/pros/char_sink.hpp:332`](../../include/shulib/hal/pros/char_sink.hpp#L332).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 101 lines, click to expand</summary>

```text

 ProsCharSink, serialLine() and serialWrapped() — the V5's USB serial, where TermSink's diagnostic
 bytes and every program's own lines physically go (`pros terminal` displays them). ProsCharSink is
 chunk R1a's; ONE-CALL WRITES are chunk SER1's (2026-10-08, the team lead's ruling).

 ═══ THE RULE: ONE LINE, ONE write() CALL ═══════════════════════════════════════════════
 Every line this header sends reaches the USB in ONE direct `write()` on the serial's descriptor
 (fd 1, STDOUT_FILENO) — never through stdout's FILE buffer. That is what keeps a line WHOLE when
 two tasks print at once. Why it has to be this way (A4 register HA-189, reasoned from the
 vendored libc.a/libpros.a, disassembled 2026-10-08 — NOT measured on a brain):
   * newlib's per-FILE locks are no-op stubs in this toolchain, and stdout is LINE-BUFFERED through
     ONE buffer every task shares — so two tasks inside printf/fwrite/puts on stdout at once can mix
     or lose each other's bytes. This header's old banner said "fwrite is atomic per call at this
     layer, so lines never interleave": that was FALSE, and is gone.
   * One `write()` call is safe: newlib's write → _write_r → PROS's ser_write_r, which builds the
     frame on the CALLER'S stack and sends it under PROS's serial write mutex — one call, one whole
     frame, no shared buffer, no stdio state touched.
   * `printf("x\n")` is not one call even in one task: the compiler rewrites it to `puts`, which
     newlib sends as TWO writes (the text, then the newline).
 So: ProsCharSink::write() sends its text with one write(); serialLine() formats a line into a
 buffer on the CALLER'S STACK and sends it with one write(); serialWrapped() sends a long text as
 pieces of at most kSerialLineBytes, one write() each. A short write (fewer bytes than asked) gets
 its REMAINDER written, never a re-send; a write that returns 0 or −1 STOPS the loop (writeWhole()).
 PROS's ser_write_r returns the whole length or 0, and PROS's _write returns −1 for a bad fd: a
 partial count is believed never to happen, and the loop is correct if it does. And no write()
 call this header makes is longer than kSerialWriteMaxBytes (512): a longer text goes out in pieces
 of at most that (HA-191, below) — longer than any line shulib prints, so every shulib line is still
 ONE call.

 ═══ THE LIMITS (what one call does NOT cure) ═══════════════════════════════════════════
   * HA-191 (reasoned from libpros.a, not seen): PROS's serial stream holds 2047 bytes, and a frame
     waits until it fits WHOLE — so one write() of more than ~2,035 bytes of text (4-byte stream id
     + data + framing + delimiter) would wait FOREVER holding PROS's write mutex, and every later
     serial line would block. No call in this header can send that much: writeWhole() offers one
     write() at most kSerialWriteMaxBytes, so a longer text through the sink goes out in pieces —
     and another task's line can land between two pieces, so a line that must stay whole stays
     under 512 bytes (shulib's are at most 384, diag/line_format.hpp's Line::kCapacity);
     serialLine()'s and serialWrapped()'s lines are at most kSerialLineBytes. A team's OWN `write()`
     on fd 1 has no such cap.
   * HA-190 (reasoned from libpros.a, not seen): a task PROS DELETES while it is inside ser_write_r
     leaves PROS's write mutex held, and every later serial write — stdout and stderr alike —
     blocks forever. A competition program's opcontrol() that logs right after its loop sees the
     disable logs at exactly that moment. SER1 neither causes nor cures it (a fix would be PROS's,
     or one printing task).
   * What else writes the serial through stdio — all PROS's, none of it shulib's after SER1: the
     serial daemon's banners (on every "pRb" from `pros terminal`, through stdout's buffer — in
     which, after SER1, no shulib text ever waits; both banners end in "\n\n\n", so the buffer is
     flushed whole); the expander ADI's `puts("Error: validate_type")` (reachable through shulib's
     expander ports, hal/pros/digital_in.hpp and digital_out.hpp); crash dumps; task_fn_wrapper's
     `fprintf(stderr)`. Each ends its lines, so it meets shulib's at a line boundary; none of it is
     guarded against two tasks (HA-189).

 ═══ THE RULE FOR A TEAM'S OWN CODE ═════════════════════════════════════════════════════
 Print a line with hal::pros::serialLine("[TAG] ...", ...) (or through a sink); a long text with
 hal::pros::serialWrapped("TAG", text). Never `printf`, `puts`, `putchar` or a `std::cout << ... <<`
 chain from more than one task — each is several calls into one shared, unguarded buffer. A `write()`
 of your own on fd 1 stays under 2 KB (HA-191; this header's calls do that for you). And never mix
 the two on one line: a direct write() after printf text still waiting in stdout's buffer goes out
 FIRST, out of order. shulib's src/ build gate fails a src/ object that names stdin/stdout/stderr
 (or their FILEs) or calls a printer that writes them — every one the vendored libc.a defines, and
 the C++ standard streams (tools/src_build_gate.py, step 9; its header says what it cannot see).

 ═══ THE STACK (read from the compiled code; upper bounds — SER1, 2026-10-08, re-walked 10-15) ══
 The buffer is ON THE CALLER'S STACK, always — never a member of the sink, never a static: two
 tasks would share it. kSerialLineBytes = 256, sized from SER1's census of every src/ line (a line
 that can be longer goes through serialWrapped()). shulib's frames measured with the build's own
 ARM flags (common.mk: `-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -mthumb`,
 `-fstack-usage`); the libraries' from the deepest call chain through the vendored
 libc.a/libm.a/libpros.a objects' prologues (a tail branch counted as a call, so these are upper
 bounds, not a run on a brain). The most a call takes below its caller:
   * serialLine() with `%s`/`%d`: ≈ 0.87 KB — its own 304 bytes (the 257-byte buffer, the saved
     varargs), then the larger of the format (vsnprintf 120 + _svfprintf_r 384, ≈ 0.85 KB) and the
     write of a full 256-byte line (≈ 0.87 KB: ser_write_r COPIES the framed line onto its own stack,
     ≈ 264 bytes for 256, so a write costs about its own length again).
   * serialLine() with `%f`, `%e` or `%g`: ≈ 1.24 KB — and those three ALLOCATE (newlib's _dtoa_r
     → malloc, which PROS locks by suspending the scheduler): avoid them in a hot loop.
   * serialWrapped(): ≈ 0.92 KB (its 344-byte frame and the write of a full piece).
   * ProsCharSink::write() of a 384-byte Line: ≈ 0.69 KB (no buffer of its own; the copy is PROS's);
     of ANY text, at most ≈ 0.82 KB — each write() call carries at most kSerialWriteMaxBytes.
 Plus, at any instant, the ≈ 332 bytes a context switch saves on a task's stack (every task pays
 that everywhere). Fine on a PROS task's default 32 KB stack (TASK_STACK_DEPTH_DEFAULT, 0x2000
 words); a team task created with a small stack must leave room — TASK_STACK_DEPTH_MIN (0x200
 words, 2 KB) leaves too little for a `%f` line on top of the task's own frames. (The first
 ruling, unbuffered stdout, would have put newlib's ≈ 1.2 KB __sbprintf frame under EVERY printf
 in every task — including PROS's serial daemon, whose 2 KB stack the brief's checker found it
 would overflow when `pros terminal` connects: the reason for ONE-CALL WRITES.)

 ═══ THE TWO CONSTRUCTORS ═══════════════════════════════════════════════════════════════
 ProsCharSink() — the robot's: writes STDOUT_FILENO and names no FILE (no stdout, no fileno, no
 fflush), so no stdio reference reaches a program's objects (the gate's step 9).
 ProsCharSink(std::FILE*) — a test's: writes that FILE's DESCRIPTOR (fileno), and fflushes the FILE
 ONCE at construction so nothing printed into it earlier is left behind the first direct write.
 Construct it before anything else writes that FILE; after that, never write the FILE through stdio.

 CONTRACT (hal/char_sink.hpp): bytes verbatim, synchronous on the caller's task, MUST NOT throw —
 write() cannot. One ICharSink::write() carries one complete line (the FORMATTER's framing) and is
 ONE write() call below (pieces of at most kSerialWriteMaxBytes for a longer text), so two tasks'
 lines do not interleave at THIS layer. That PROS's mutex keeps one call whole on the brain is
 reasoned from its library (HA-189), not yet seen.
 PROS-FREE: <cstdio>, <unistd.h> and the standard library only — no <pros/*> header (the CI
 PROS-free guard SKIPS hal/pros/, so test/serial_line_test.cpp pins it).
```

</details>
