<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/block_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_sink.hpp`

ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk R1b): where the blackbox's binary blocks physically go.

This header declares **3** types (17 members) and **1** free function.

Extracted from [`include/shulib/hal/pros/block_sink.hpp`](../../include/shulib/hal/pros/block_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ProsFileMode`](#enum-class-prosfilemode)
  - [`Truncate`](#prosfilemode-truncate)
  - [`Append`](#prosfilemode-append)
- [`enum class AppendOpen`](#enum-class-appendopen)
  - [`Appended`](#appendopen-appended)
  - [`Created`](#appendopen-created)
  - [`Refused`](#appendopen-refused)
- [`openForAppend`](#openforappend) — *free function*
- [`class ProsBlockSink`](#class-prosblocksink)
  - [`ProsBlockSink`](#prosblocksink-prosblocksink)
  - [`ProsBlockSink (overload 2)`](#prosblocksink-prosblocksink-2)
  - [`~ProsBlockSink`](#prosblocksink-destructor-prosblocksink)
  - [`ProsBlockSink (overload 3)`](#prosblocksink-prosblocksink-3)
  - [`operator=`](#prosblocksink-operator-eq)
  - [`ProsBlockSink (overload 4)`](#prosblocksink-prosblocksink-4)
  - [`operator= (overload 2)`](#prosblocksink-operator-eq-2)
  - [`write`](#prosblocksink-write)
  - [`flush`](#prosblocksink-flush)
  - [`isOpen`](#prosblocksink-isopen)
  - [`path`](#prosblocksink-path)
  - [`appendOpen`](#prosblocksink-appendopen)

<a id="enum-class-prosfilemode"></a>

## `enum class ProsFileMode`

```cpp
enum class ProsFileMode : std::uint8_t
```

How a ProsBlockSink opens its file (header: THE APPEND MODE). The default, and every caller before COMP-t, is Truncate.

*enum class, declared at [`include/shulib/hal/pros/block_sink.hpp:80`](../../include/shulib/hal/pros/block_sink.hpp#L80).*

<a id="prosfilemode-truncate"></a>

### `ProsFileMode::Truncate`

```cpp
Truncate
```

"wb": one file per boot (the blackbox's; every existing caller)

*enumerator, declared at [`include/shulib/hal/pros/block_sink.hpp:81`](../../include/shulib/hal/pros/block_sink.hpp#L81).*

<a id="prosfilemode-append"></a>

### `ProsFileMode::Append`

```cpp
Append
```

keep every earlier byte and write after them (openForAppend below)

*enumerator, declared at [`include/shulib/hal/pros/block_sink.hpp:82`](../../include/shulib/hal/pros/block_sink.hpp#L82).*

<a id="enum-class-appendopen"></a>

## `enum class AppendOpen`

```cpp
enum class AppendOpen : std::uint8_t
```

How an Append open went — for the caller's one boot line.

*enum class, declared at [`include/shulib/hal/pros/block_sink.hpp:86`](../../include/shulib/hal/pros/block_sink.hpp#L86).*

<a id="appendopen-appended"></a>

### `AppendOpen::Appended`

```cpp
Appended
```

the file existed; new bytes go after its last byte

*enumerator, declared at [`include/shulib/hal/pros/block_sink.hpp:87`](../../include/shulib/hal/pros/block_sink.hpp#L87).*

<a id="appendopen-created"></a>

### `AppendOpen::Created`

```cpp
Created
```

the file did not exist; it was created

*enumerator, declared at [`include/shulib/hal/pros/block_sink.hpp:88`](../../include/shulib/hal/pros/block_sink.hpp#L88).*

<a id="appendopen-refused"></a>

### `AppendOpen::Refused`

```cpp
Refused
```

no card, or the file exists but would not open for appending (never truncated)

*enumerator, declared at [`include/shulib/hal/pros/block_sink.hpp:89`](../../include/shulib/hal/pros/block_sink.hpp#L89).*

<a id="openforappend"></a>

## `openForAppend`

```cpp
template <class Open, class Close> [[nodiscard]] std::FILE* openForAppend(const char* path, Open&& open, Close&& close, AppendOpen& how)
```

THE APPEND OPEN, decided here and nowhere else (header: THE APPEND MODE): ask with "rb" whether `path` exists (closing that probe with `close`), open "ab", and if that fails create with "wb" ONLY when the file was absent — an existing log is never truncated. `open(path, mode)` returns a FILE* or nullptr; the adapter passes std::fopen, a host test a script. `how` says which branch was taken. No mode here has a '+'. Returns the open FILE*, or nullptr (then `how` is Refused).

*free function, declared at [`include/shulib/hal/pros/block_sink.hpp:99`](../../include/shulib/hal/pros/block_sink.hpp#L99).*

<a id="class-prosblocksink"></a>

## `class ProsBlockSink`

```cpp
class ProsBlockSink final : public IBlockSink
```

IBlockSink over a PROS FILE* on the V5's SD card — where the blackbox's binary blocks physically land. It OWNS the file: opened at construction — truncating by default (one file per boot), or appending (ProsFileMode::Append, since COMP-t) — and closed at destruction, so it must outlive every SdSink or char sink writing through it. A MISSING CARD IS NOT AN ERROR — construction still succeeds and the sink simply refuses (isOpen() false, every write()/flush() false), because a robot must still drive without a card, and E1's drop-and-count design already handles a sink that says no. It also owns the /usd/ prefix: pass a BARE file name.

*class, declared at [`include/shulib/hal/pros/block_sink.hpp:128`](../../include/shulib/hal/pros/block_sink.hpp#L128).*

<a id="prosblocksink-prosblocksink"></a>

### `ProsBlockSink::ProsBlockSink`

```cpp
explicit ProsBlockSink(const char* fileName, const char* mountRoot = "/usd/")
```

Open `<mountRoot><fileName>` for binary writing (truncating — one blackbox file per boot). `fileName` must be BARE: no leading '/' (the adapter owns the /usd/ prefix — header note). `mountRoot` must end in '/'; it defaults to the robot truth and exists so host tests exercise the same join+open path. No card / failed open → a refusing sink, NOT a throw (T5): isOpen() false, every write()/flush() false.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:136`](../../include/shulib/hal/pros/block_sink.hpp#L136).*

<a id="prosblocksink-prosblocksink-2"></a>

### `ProsBlockSink::ProsBlockSink (overload 2)`

```cpp
ProsBlockSink(const char* fileName, ProsFileMode mode, const char* mountRoot = "/usd/")
```

The same, in `mode`: Truncate is exactly the constructor above; Append keeps the file's earlier bytes and writes after them (openForAppend — `appendOpen()` says how it went). The prefix rule, the no-card refusal and the preconditions are the same.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:142`](../../include/shulib/hal/pros/block_sink.hpp#L142).*

<a id="prosblocksink-destructor-prosblocksink"></a>

### `ProsBlockSink::~ProsBlockSink`

```cpp
~ProsBlockSink() override
```

fclose the file, which also pushes newlib's remaining buffer out. Nothing else holds this FILE*, so anything still writing through this sink — an SdSink, most likely — must be destroyed first. No-op on a refusing sink.  THE ONE UNREPORTABLE FAILURE, stated rather than left implicit. fclose FLUSHES before it closes and returns EOF if that write fails — the card-full, card-yanked, dying-card case — and a destructor has no channel to say so. Every other path in this class is bool-valued for exactly that reason (write() is even [[nodiscard]], and block_sink.hpp justifies it: "a caller that ignores the result cannot notice a truncated file"). The last buffered block is both the most likely to be lost and the only one whose loss nothing here can report. A caller that needs the final bytes CONFIRMED must call flush(), which is bool for this reason, before letting the sink die.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:176`](../../include/shulib/hal/pros/block_sink.hpp#L176).*

<a id="prosblocksink-prosblocksink-3"></a>

### `ProsBlockSink::ProsBlockSink (overload 3)`

```cpp
ProsBlockSink(const ProsBlockSink&) = delete
```

Non-copyable and non-movable: this adapter OWNS the FILE*, and a second handle to it would fclose the same file twice. (ProsCharSink merely borrows stdout, which is why it carries no such restriction — the difference is ownership, not policy.)

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:185`](../../include/shulib/hal/pros/block_sink.hpp#L185).*

<a id="prosblocksink-operator-eq"></a>

### `ProsBlockSink::operator=`

```cpp
ProsBlockSink& operator=(const ProsBlockSink&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 3)`](#prosblocksink-prosblocksink-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:186`](../../include/shulib/hal/pros/block_sink.hpp#L186).*

<a id="prosblocksink-prosblocksink-4"></a>

### `ProsBlockSink::ProsBlockSink (overload 4)`

```cpp
ProsBlockSink(ProsBlockSink&&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 3)`](#prosblocksink-prosblocksink-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:187`](../../include/shulib/hal/pros/block_sink.hpp#L187).*

<a id="prosblocksink-operator-eq-2"></a>

### `ProsBlockSink::operator= (overload 2)`

```cpp
ProsBlockSink& operator=(ProsBlockSink&&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 3)`](#prosblocksink-prosblocksink-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:188`](../../include/shulib/hal/pros/block_sink.hpp#L188).*

<a id="prosblocksink-write"></a>

### `ProsBlockSink::write`

```cpp
[[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override
```

Verbatim bytes; false unless EVERY byte was accepted (a short write leaves a prefix — the format decodes up to the cut). False always while refusing (no card / failed open).

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:193`](../../include/shulib/hal/pros/block_sink.hpp#L193).*

<a id="prosblocksink-flush"></a>

### `ProsBlockSink::flush`

```cpp
bool flush() noexcept override
```

fflush to the FatFS driver (the platform's strongest "on the medium" — header note). False on device failure or while refusing.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:205`](../../include/shulib/hal/pros/block_sink.hpp#L205).*

<a id="prosblocksink-isopen"></a>

### `ProsBlockSink::isOpen`

```cpp
[[nodiscard]] bool isOpen() const noexcept
```

False = the sink is refusing (no card at boot, or the open failed). The composition root checks this ONCE and reports through the diagnostics layer — the T5 visibility rule.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:215`](../../include/shulib/hal/pros/block_sink.hpp#L215).*

<a id="prosblocksink-path"></a>

### `ProsBlockSink::path`

```cpp
[[nodiscard]] const char* path() const noexcept
```

The full path this sink writes (for the one-time diagnostics line).

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:218`](../../include/shulib/hal/pros/block_sink.hpp#L218).*

<a id="prosblocksink-appendopen"></a>

### `ProsBlockSink::appendOpen`

```cpp
[[nodiscard]] AppendOpen appendOpen() const noexcept
```

How an Append open went: Appended, Created, or Refused (no card, or an existing file that would not open for appending). Always Refused in Truncate mode, which never asks.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:222`](../../include/shulib/hal/pros/block_sink.hpp#L222).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 57 lines, click to expand</summary>

```text

 ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk
 R1b): where the blackbox's binary blocks physically go. The device seam E1
 shipped the interface for (block_sink.hpp names R1 as this file's owner);
 diag/'s SdSink writes through it.

 BINDS:
  * ::pros::usd::is_installed() — the no-card probe, at construction
    (vendored misc.hpp:555-568; HA-122)
  * std::fopen/std::fwrite/std::fflush on "/usd/<name>" — the PROS kernel
    mounts the SD card at /usd/ for newlib file IO (HA-122)

 ── THE /usd/ PATH QUIRK (HA-122 — register + FAQ) ─────────────────────────────────
 Two conventions in ONE API, the same shape as R1a's 0-vs-1-indexed registry
 finding: usd_list_files() documents "DO NOT PREPEND YOUR PATHS WITH /usd/"
 (vendored misc.h:824-825) while fopen REQUIRES the /usd/ prefix. This
 adapter owns the prefix in exactly one place: the constructor takes a BARE
 file name (a leading '/' is a loud precondition, so the double-prefix
 mistake cannot compile out of sight), and joins it to `mountRoot` —
 "/usd/" on the robot, injectable so a host test can point it at a real
 temp directory and exercise THIS code path, not a copy of it.

 ── NO CARD AT BOOT (T5's ruling) ──────────────────────────────────────────────────
 Construction SUCCEEDS, write() returns false from the first call, and the
 fact is visible through isOpen() — checked once by the composition root /
 diagnostics layer, which owns saying it out loud (hal/ is below diag/). A
 missing SD card must not stop a robot from driving, and E1's drop-and-
 count design already handles a sink that refuses; a throw here would turn
 a missing card into a dead robot. Rejected: silently succeeding — the
 exact "nothing looks wrong" failure the blackbox exists to avoid.

 CONTRACT (block_sink.hpp): bytes VERBATIM, synchronous, MUST NOT throw —
 fwrite/fflush cannot throw, and both paths return bool. A short fwrite
 (card full, yanked, dying) returns false and leaves a prefix on the
 device, which is exactly what the format is designed to decode up to.
 flush() is fflush() — pushing newlib's buffer to the FatFS driver; there
 is no fsync in PROS's exposed surface, so "on the medium" is as strong as
 the platform allows (HA-122's weak half — bench: yank the card after a
 flushed write and count what survived). PROS's own usd_write_r does call
 vexFileSync after every write it passes on, ignoring what it returns; what
 that sync commits is VEXos's (HA-166 (5)).

 OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
 non-copyable/non-movable — unlike ProsCharSink, which borrows stdout.

 ── THE APPEND MODE (chunk COMP-t, 2026-09-28 — additive) ─────────────────────────
 ProsFileMode::Append keeps every earlier byte of the file and writes after them: the
 Auton Tester's run log, ONE file on the card across reboots. The decision is made in
 ONE function, openForAppend() below, host-tested with a scripted opener: ask with "rb"
 whether the file exists, open "ab", and create with "wb" ONLY when the file was absent,
 so an existing log is never truncated whichever way VEXos answers an append on a
 missing file (PROS hands an O_APPEND open to vexFileOpenWrite; whether that creates is
 VEXos's — A4 register HA-177). Every mode string lives in this file, none with a '+'
 (the brain refuses read-write opens), and the two-argument constructor still means
 Truncate, byte for byte, for every existing caller.

 HA register: HA-122, HA-177 (docs/hardware-assumptions.md).
```

</details>
