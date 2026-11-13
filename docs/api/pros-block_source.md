<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/block_source.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_source.hpp`

ProsBlockSource — IBlockSource over a PROS FILE* on the V5's SD card (chunk R3d Part 2): where the calibration record is read back from.

This header declares **1** type (9 members).

Extracted from [`include/shulib/hal/pros/block_source.hpp`](../../include/shulib/hal/pros/block_source.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsBlockSource`](#class-prosblocksource)
  - [`ProsBlockSource`](#prosblocksource-prosblocksource)
  - [`~ProsBlockSource`](#prosblocksource-destructor-prosblocksource)
  - [`ProsBlockSource (overload 2)`](#prosblocksource-prosblocksource-2)
  - [`operator=`](#prosblocksource-operator-eq)
  - [`ProsBlockSource (overload 3)`](#prosblocksource-prosblocksource-3)
  - [`operator= (overload 2)`](#prosblocksource-operator-eq-2)
  - [`readAll`](#prosblocksource-readall)
  - [`isOpen`](#prosblocksource-isopen)
  - [`path`](#prosblocksource-path)

<a id="class-prosblocksource"></a>

## `class ProsBlockSource`

```cpp
class ProsBlockSource final : public IBlockSource
```

IBlockSource over a PROS FILE* on the V5's SD card — where the calibration record is read back at boot. It OWNS the file: opened for binary reading at construction and closed at destruction. A MISSING CARD OR A MISSING FILE IS NOT AN ERROR — construction still succeeds and the source simply refuses (isOpen() false, readAll() 0), because a robot with no calibration must still be able to boot and say so. It also owns the /usd/ prefix: pass a BARE file name.

*class, declared at [`include/shulib/hal/pros/block_source.hpp:64`](../../include/shulib/hal/pros/block_source.hpp#L64).*

<a id="prosblocksource-prosblocksource"></a>

### `ProsBlockSource::ProsBlockSource`

```cpp
explicit ProsBlockSource(const char* fileName, const char* mountRoot = "/usd/")
```

Open `<mountRoot><fileName>` for binary READING. `fileName` must be BARE: no leading '/' (the adapter owns the /usd/ prefix — header note). `mountRoot` must end in '/'; it defaults to the robot truth and exists so host tests exercise the same join+open path. No card, or a file that does not exist → a refusing source, NOT a throw (T5): isOpen() false, readAll() 0.

*function, declared at [`include/shulib/hal/pros/block_source.hpp:71`](../../include/shulib/hal/pros/block_source.hpp#L71).*

<a id="prosblocksource-destructor-prosblocksource"></a>

### `ProsBlockSource::~ProsBlockSource`

```cpp
~ProsBlockSource() override
```

fclose the file. Nothing else holds this FILE*. No-op on a refusing source. Unlike the SINK's destructor there is no unreportable failure here: closing a file opened for reading flushes nothing and loses nothing.

*function, declared at [`include/shulib/hal/pros/block_source.hpp:90`](../../include/shulib/hal/pros/block_source.hpp#L90).*

<a id="prosblocksource-prosblocksource-2"></a>

### `ProsBlockSource::ProsBlockSource (overload 2)`

```cpp
ProsBlockSource(const ProsBlockSource&) = delete
```

Non-copyable and non-movable: this adapter OWNS the FILE*, and a second handle to it would fclose the same file twice (ProsBlockSink's reasoning, unchanged).

*function, declared at [`include/shulib/hal/pros/block_source.hpp:98`](../../include/shulib/hal/pros/block_source.hpp#L98).*

<a id="prosblocksource-operator-eq"></a>

### `ProsBlockSource::operator=`

```cpp
ProsBlockSource& operator=(const ProsBlockSource&) = delete
```

*Covered by the comment on [`ProsBlockSource (overload 2)`](#prosblocksource-prosblocksource-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_source.hpp:99`](../../include/shulib/hal/pros/block_source.hpp#L99).*

<a id="prosblocksource-prosblocksource-3"></a>

### `ProsBlockSource::ProsBlockSource (overload 3)`

```cpp
ProsBlockSource(ProsBlockSource&&) = delete
```

*Covered by the comment on [`ProsBlockSource (overload 2)`](#prosblocksource-prosblocksource-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_source.hpp:100`](../../include/shulib/hal/pros/block_source.hpp#L100).*

<a id="prosblocksource-operator-eq-2"></a>

### `ProsBlockSource::operator= (overload 2)`

```cpp
ProsBlockSource& operator=(ProsBlockSource&&) = delete
```

*Covered by the comment on [`ProsBlockSource (overload 2)`](#prosblocksource-prosblocksource-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/pros/block_source.hpp:101`](../../include/shulib/hal/pros/block_source.hpp#L101).*

<a id="prosblocksource-readall"></a>

### `ProsBlockSource::readAll`

```cpp
[[nodiscard]] std::size_t readAll(std::span<std::byte> into) noexcept override
```

Read up to `into.size()` bytes from the START of the file and return the count. Seeks to 0 first, so the seam's "reads from the start" contract holds even if somebody calls it twice. 0 while refusing, 0 on an empty file, and a short count on a truncated one — which the format refuses by name, not this adapter.

*function, declared at [`include/shulib/hal/pros/block_source.hpp:107`](../../include/shulib/hal/pros/block_source.hpp#L107).*

<a id="prosblocksource-isopen"></a>

### `ProsBlockSource::isOpen`

```cpp
[[nodiscard]] bool isOpen() const noexcept override
```

False = the source is refusing (no card at boot, or the file does not exist / failed to open). The composition root checks this ONCE and reports — the T5 visibility rule.

*function, declared at [`include/shulib/hal/pros/block_source.hpp:119`](../../include/shulib/hal/pros/block_source.hpp#L119).*

<a id="prosblocksource-path"></a>

### `ProsBlockSource::path`

```cpp
[[nodiscard]] const char* path() const noexcept
```

The full path this source reads (for the one-time diagnostics line).

*function, declared at [`include/shulib/hal/pros/block_source.hpp:122`](../../include/shulib/hal/pros/block_source.hpp#L122).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 38 lines</summary>

```text

 ProsBlockSource — IBlockSource over a PROS FILE* on the V5's SD card (chunk R3d
 Part 2): where the calibration record is read back from. The mirror image of
 ProsBlockSink, deliberately line for line, because the two must agree about the
 path or "shulib Drive" reads a file "shulib Calibrate" never wrote.

 BINDS:
  * ::pros::usd::is_installed() — the no-card probe, at construction
    (vendored misc.hpp:555-568; HA-122)
  * std::fopen(…, "rb") / std::fread on "/usd/<name>" — the PROS kernel mounts the
    SD card at /usd/ for newlib file IO (HA-122). "rb" and not "r": the record is
    binary and contains 0x1A, 0x0A and 0x00; a text-mode read is a corrupt record on
    any platform that translates, and the host test runs the SAME call.

 ── THE /usd/ PATH QUIRK (HA-122 — register + FAQ), owned here too ──────────────────
 usd_list_files() documents "DO NOT PREPEND YOUR PATHS WITH /usd/" while fopen REQUIRES
 the prefix. This adapter owns the prefix in exactly one place: the constructor takes a
 BARE file name (a leading '/' is a loud precondition) and joins it to `mountRoot` —
 "/usd/" on the robot, injectable so a host test can point it at a real temp directory
 and exercise THIS code path rather than a copy of it.

 ── NO CARD, OR NO FILE (T5's ruling, and its read-side half) ──────────────────────
 Construction SUCCEEDS in both cases; readAll() returns 0 and isOpen() is false. A missing
 card must not stop a robot from driving, and a MISSING RECORD is the normal state of a
 robot that has never been calibrated — neither is an error, and a throw here would turn
 either into a dead robot at boot. The composition root asks isOpen() once and says which
 it was: "no SD card" and "no calibration on this card" send an operator to two different
 places, and the seam's contract (block_source.hpp) exists to keep them apart.

 A SHORT read is NOT an error here: the count comes back and the FORMAT refuses by name
 (calibration_record.hpp's length check). An adapter that turned a truncated file into a
 bool would have thrown away the one word the operator needs.

 OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
 non-copyable/non-movable — the sink's reasoning, unchanged: a second handle would fclose
 the same file twice.

 HA register: HA-122 (docs/hardware-assumptions.md), plus HA-139 for the "rb" belief.
```

</details>
