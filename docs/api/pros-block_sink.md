<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/block_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_sink.hpp`

ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk R1b): where the blackbox's binary blocks physically go.

This header declares **1** type (10 members).

Extracted from [`include/shulib/hal/pros/block_sink.hpp`](../../include/shulib/hal/pros/block_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsBlockSink`](#class-prosblocksink)
  - [`ProsBlockSink`](#prosblocksink-prosblocksink)
  - [`~ProsBlockSink`](#prosblocksink-destructor-prosblocksink)
  - [`ProsBlockSink (overload 2)`](#prosblocksink-prosblocksink-2)
  - [`operator=`](#prosblocksink-operator-eq)
  - [`ProsBlockSink (overload 3)`](#prosblocksink-prosblocksink-3)
  - [`operator= (overload 2)`](#prosblocksink-operator-eq-2)
  - [`write`](#prosblocksink-write)
  - [`flush`](#prosblocksink-flush)
  - [`isOpen`](#prosblocksink-isopen)
  - [`path`](#prosblocksink-path)

<a id="class-prosblocksink"></a>

## `class ProsBlockSink`

```cpp
class ProsBlockSink final : public IBlockSink
```

IBlockSink over a PROS FILE* on the V5's SD card — where the blackbox's binary blocks physically land. It OWNS the file: opened truncating at construction and closed at destruction, so one instance is one file per boot and it must outlive every SdSink writing through it. A MISSING CARD IS NOT AN ERROR — construction still succeeds and the sink simply refuses (isOpen() false, every write()/flush() false), because a robot must still drive without a card, and E1's drop-and-count design already handles a sink that says no. It also owns the /usd/ prefix: pass a BARE file name.

*class, declared at [`include/shulib/hal/pros/block_sink.hpp:71`](../../include/shulib/hal/pros/block_sink.hpp#L71).*

<a id="prosblocksink-prosblocksink"></a>

### `ProsBlockSink::ProsBlockSink`

```cpp
explicit ProsBlockSink(const char* fileName, const char* mountRoot = "/usd/")
```

Open `<mountRoot><fileName>` for binary writing (truncating — one blackbox file per boot). `fileName` must be BARE: no leading '/' (the adapter owns the /usd/ prefix — header note). `mountRoot` must end in '/'; it defaults to the robot truth and exists so host tests exercise the same join+open path. No card / failed open → a refusing sink, NOT a throw (T5): isOpen() false, every write()/flush() false.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:79`](../../include/shulib/hal/pros/block_sink.hpp#L79).*

<a id="prosblocksink-destructor-prosblocksink"></a>

### `ProsBlockSink::~ProsBlockSink`

```cpp
~ProsBlockSink() override
```

fclose the file, which also pushes newlib's remaining buffer out. Nothing else holds this FILE*, so anything still writing through this sink — an SdSink, most likely — must be destroyed first. No-op on a refusing sink.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:98`](../../include/shulib/hal/pros/block_sink.hpp#L98).*

<a id="prosblocksink-prosblocksink-2"></a>

### `ProsBlockSink::ProsBlockSink (overload 2)`

```cpp
ProsBlockSink(const ProsBlockSink&) = delete
```

Non-copyable and non-movable: this adapter OWNS the FILE*, and a second handle to it would fclose the same file twice. (ProsCharSink merely borrows stdout, which is why it carries no such restriction — the difference is ownership, not policy.)

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:107`](../../include/shulib/hal/pros/block_sink.hpp#L107).*

<a id="prosblocksink-operator-eq"></a>

### `ProsBlockSink::operator=`

```cpp
ProsBlockSink& operator=(const ProsBlockSink&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 2)`](#prosblocksink-prosblocksink-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:108`](../../include/shulib/hal/pros/block_sink.hpp#L108).*

<a id="prosblocksink-prosblocksink-3"></a>

### `ProsBlockSink::ProsBlockSink (overload 3)`

```cpp
ProsBlockSink(ProsBlockSink&&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 2)`](#prosblocksink-prosblocksink-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:109`](../../include/shulib/hal/pros/block_sink.hpp#L109).*

<a id="prosblocksink-operator-eq-2"></a>

### `ProsBlockSink::operator= (overload 2)`

```cpp
ProsBlockSink& operator=(ProsBlockSink&&) = delete
```

*Covered by the comment on [`ProsBlockSink (overload 2)`](#prosblocksink-prosblocksink-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:110`](../../include/shulib/hal/pros/block_sink.hpp#L110).*

<a id="prosblocksink-write"></a>

### `ProsBlockSink::write`

```cpp
[[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override
```

Verbatim bytes; false unless EVERY byte was accepted (a short write leaves a prefix — the format decodes up to the cut). False always while refusing (no card / failed open).

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:115`](../../include/shulib/hal/pros/block_sink.hpp#L115).*

<a id="prosblocksink-flush"></a>

### `ProsBlockSink::flush`

```cpp
bool flush() noexcept override
```

fflush to the FatFS driver (the platform's strongest "on the medium" — header note). False on device failure or while refusing.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:127`](../../include/shulib/hal/pros/block_sink.hpp#L127).*

<a id="prosblocksink-isopen"></a>

### `ProsBlockSink::isOpen`

```cpp
[[nodiscard]] bool isOpen() const noexcept
```

False = the sink is refusing (no card at boot, or the open failed). The composition root checks this ONCE and reports through the diagnostics layer — the T5 visibility rule.

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:137`](../../include/shulib/hal/pros/block_sink.hpp#L137).*

<a id="prosblocksink-path"></a>

### `ProsBlockSink::path`

```cpp
[[nodiscard]] const char* path() const noexcept
```

The full path this sink writes (for the one-time diagnostics line).

*function, declared at [`include/shulib/hal/pros/block_sink.hpp:140`](../../include/shulib/hal/pros/block_sink.hpp#L140).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 44 lines</summary>

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
 flushed write and count what survived).

 OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
 non-copyable/non-movable — unlike ProsCharSink, which borrows stdout.

 HA register: HA-122 (docs/hardware-assumptions.md).
```

</details>
