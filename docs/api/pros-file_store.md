<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/file_store.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `file_store.hpp`

ProsFileStore — hal::IFileStore on the V5's SD card (chunk SAFE-SAVE Part A, 2026-09-29): the two operations the safe save builds on — read a file whole, replace it by truncating it — composed from the two adapters that already own the card, one adapter per…

This header declares **1** type (5 members).

Extracted from [`include/shulib/hal/pros/file_store.hpp`](../../include/shulib/hal/pros/file_store.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsFileStore`](#class-prosfilestore)
  - [`ProsFileStore`](#prosfilestore-prosfilestore)
  - [`ProsFileStore (overload 2)`](#prosfilestore-prosfilestore-2)
  - [`cardPresent`](#prosfilestore-cardpresent)
  - [`read`](#prosfilestore-read)
  - [`replace`](#prosfilestore-replace)

<a id="class-prosfilestore"></a>

## `class ProsFileStore`

```cpp
class ProsFileStore final : public IFileStore
```

hal::IFileStore on the V5's SD card: read() builds a ProsBlockSource and replace() a ProsBlockSink, each in its own scope, so every file is closed before the call returns. No fopen, no mode string and no "/usd/" of its own — the two adapters own all three (HA-122). replace() opens TRUNCATING (HA-166 (1)) and cannot report the close; read the file back.

*class, declared at [`include/shulib/hal/pros/file_store.hpp:59`](../../include/shulib/hal/pros/file_store.hpp#L59).*

<a id="prosfilestore-prosfilestore"></a>

### `ProsFileStore::ProsFileStore`

```cpp
ProsFileStore() = default
```

The robot's store: each adapter is built with its OWN default root ("/usd/" — owned there).

*function, declared at [`include/shulib/hal/pros/file_store.hpp:62`](../../include/shulib/hal/pros/file_store.hpp#L62).*

<a id="prosfilestore-prosfilestore-2"></a>

### `ProsFileStore::ProsFileStore (overload 2)`

```cpp
explicit ProsFileStore(const char* mountRoot)
```

A store rooted at `mountRoot` (must end in '/'; the adapters check it) — for host tests that exercise this exact code against a real temp directory. The pointer is borrowed and must outlive the store.

*function, declared at [`include/shulib/hal/pros/file_store.hpp:67`](../../include/shulib/hal/pros/file_store.hpp#L67).*

<a id="prosfilestore-cardpresent"></a>

### `ProsFileStore::cardPresent`

```cpp
[[nodiscard]] bool cardPresent() const override
```

`usd_is_installed() != 0` — the adapters' own no-card probe (HA-122 (1)).

*function, declared at [`include/shulib/hal/pros/file_store.hpp:73`](../../include/shulib/hal/pros/file_store.hpp#L73).*

<a id="prosfilestore-read"></a>

### `ProsFileStore::read`

```cpp
[[nodiscard]] StoreRead read(const char* bareName, std::span<std::byte> into) override
```

Build a ProsBlockSource for `bareName`; not open → `opened` false; open → its readAll() from the start. The path is the source's own path(). A name that is not bare throws.

*function, declared at [`include/shulib/hal/pros/file_store.hpp:77`](../../include/shulib/hal/pros/file_store.hpp#L77).*

<a id="prosfilestore-replace"></a>

### `ProsFileStore::replace`

```cpp
[[nodiscard]] StoreReplace replace(const char* bareName, std::span<const std::byte> bytes) override
```

Build a ProsBlockSink for `bareName` (its "wb" open TRUNCATES — HA-166 (1)), write, flush, and let its scope close it. Not open → NoCard if cardPresent() is false, else Open; then Write or Flush as they fail. The close cannot report, so `None` proves nothing (header).

*function, declared at [`include/shulib/hal/pros/file_store.hpp:93`](../../include/shulib/hal/pros/file_store.hpp#L93).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 ProsFileStore — hal::IFileStore on the V5's SD card (chunk SAFE-SAVE Part A, 2026-09-29): the
 two operations the safe save builds on — read a file whole, replace it by truncating it —
 composed from the two adapters that already own the card, one adapter per call.

 BINDS:
  * ::pros::usd::is_installed() — cardPresent(), the adapters' own no-card probe (vendored
    misc.hpp:555-568; HA-122 (1))
  * hal::pros::ProsBlockSource — read(): its "rb" open, its readAll() from the start, its path()
  * hal::pros::ProsBlockSink — replace(): its "wb" open (TRUNCATING — HA-166 (1)), its write(),
    its flush(), and its destructor's close, which cannot report (block_sink.hpp)

 ── No fopen, no mode string and no "/usd/" of its own ──────────────────────────────
 Every call builds a ProsBlockSource or a ProsBlockSink in its OWN scope, so the file is closed
 before the call returns — a replace's writer is gone before the caller's read-back opens the
 file again. The adapters stay the only places in the library that call fopen and the only places
 that spell a mode, and the /usd/ prefix stays owned by them (HA-122; calibration_record.hpp's
 kRecordFileName says so). A default-constructed store builds each adapter with ONE argument, so
 the adapter's own default root applies; `ProsFileStore(root)` passes a root through, for host
 tests against a real temp directory. The platform scan (test/record_store_platform_scan_test.cpp)
 pins all three.

 ── What replace() can and cannot say ───────────────────────────────────────────────
 Not open → NoCard when cardPresent() is false (the sink never called fopen, so nothing was
 created), else Open. Then Write or Flush as they fail. The close is the sink's destructor, at
 the end of the call's scope: it cannot report (block_sink.hpp), and PROS's close returns 0
 whatever VEXos answers (HA-166 (5)). So replace() makes NO claim about the close, and `None`
 proves nothing — the caller's read-back is the proof (calibration/record_store.hpp's put()).

 ── Exceptions and allocation ───────────────────────────────────────────────────────
 Not noexcept (hal/file_store.hpp): a name that is not bare throws PreconditionError from the
 adapter's constructor, before any card is touched, and that must propagate. Allocation happens
 only inside the adapters (each one's std::string path, as today's calibration save already
 does); this class holds one pointer.

 HA register: HA-166 (what the card allows), HA-122 (the prefix, the probe).
```

</details>
