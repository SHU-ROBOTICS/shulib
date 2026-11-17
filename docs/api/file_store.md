<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/file_store.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `file_store.hpp`

IFileStore — the two things the brain's SD card offers that a SAFE save can be built on (chunk SAFE-SAVE Part A, 2026-09-29): read a whole named file, or replace one by truncating it.

This header declares **4** types (19 members), **1** free function, and **1** constant.

Extracted from [`include/shulib/hal/file_store.hpp`](../../include/shulib/hal/file_store.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kStorePathBytes`](#kstorepathbytes) — *constant*
- [`struct StoreRead`](#struct-storeread)
  - [`opened`](#storeread-opened)
  - [`count`](#storeread-count)
  - [`path`](#storeread-path)
- [`enum class ReplaceStep`](#enum-class-replacestep)
  - [`None`](#replacestep-none)
  - [`NoCard`](#replacestep-nocard)
  - [`Open`](#replacestep-open)
  - [`Write`](#replacestep-write)
  - [`Flush`](#replacestep-flush)
- [`struct StoreReplace`](#struct-storereplace)
  - [`failed`](#storereplace-failed)
  - [`path`](#storereplace-path)
- [`class IFileStore`](#class-ifilestore)
  - [`~IFileStore`](#ifilestore-destructor-ifilestore)
  - [`IFileStore`](#ifilestore-ifilestore)
  - [`IFileStore (overload 2)`](#ifilestore-ifilestore-2)
  - [`IFileStore (overload 3)`](#ifilestore-ifilestore-3)
  - [`operator=`](#ifilestore-operator-eq)
  - [`operator= (overload 2)`](#ifilestore-operator-eq-2)
  - [`cardPresent`](#ifilestore-cardpresent)
  - [`read`](#ifilestore-read)
  - [`replace`](#ifilestore-replace)
- [`sameFileOnCard`](#samefileoncard) — *free function*

<a id="kstorepathbytes"></a>

## `kStorePathBytes`

```cpp
inline constexpr std::size_t kStorePathBytes = 96
```

Capacity of the `path` a store result carries, terminator included. A longer path is CUT in the result and nothing else: the path is for messages only, and the operation used the full path the adapter holds.

*constant, declared at [`include/shulib/hal/file_store.hpp:71`](../../include/shulib/hal/file_store.hpp#L71).*

<a id="struct-storeread"></a>

## `struct StoreRead`

```cpp
struct StoreRead
```

What one read() saw: whether the file opened, how many bytes came back, and the path it used.

*struct, declared at [`include/shulib/hal/file_store.hpp:74`](../../include/shulib/hal/file_store.hpp#L74).*

<a id="storeread-opened"></a>

### `StoreRead::opened`

```cpp
bool opened = false
```

False: no card, no such file, or the open was refused. On this brain a file EXISTS exactly when a read opens it (there is no stat — HA-166 (2)).

*field, declared at [`include/shulib/hal/file_store.hpp:77`](../../include/shulib/hal/file_store.hpp#L77).*

<a id="storeread-count"></a>

### `StoreRead::count`

```cpp
std::size_t count = 0
```

Bytes copied into the caller's buffer, from the START of the file. 0 on an open but EMPTY file; a count below the buffer's size means the file ran out (a SHORT file, which the caller's format refuses by name). 0 when `opened` is false — both shipped stores do so, and calibration/record_store.hpp does not rely on it: it counts a read that did not open as 0 bytes, and clamps any count to the buffer, whatever a store reports.

*field, declared at [`include/shulib/hal/file_store.hpp:83`](../../include/shulib/hal/file_store.hpp#L83).*

<a id="storeread-path"></a>

### `StoreRead::path`

```cpp
char path[kStorePathBytes] = ""
```

The path the read used — on the brain the adapter's `/usd/<name>` — for messages only.

*field, declared at [`include/shulib/hal/file_store.hpp:85`](../../include/shulib/hal/file_store.hpp#L85).*

<a id="enum-class-replacestep"></a>

## `enum class ReplaceStep`

```cpp
enum class ReplaceStep : std::uint8_t
```

Where a replace() stopped. `None` means it reported no failure, which does NOT mean the bytes are on the card (the close cannot report — header note); only a read-back proves that.

*enum class, declared at [`include/shulib/hal/file_store.hpp:90`](../../include/shulib/hal/file_store.hpp#L90).*

<a id="replacestep-none"></a>

### `ReplaceStep::None`

```cpp
None
```

opened, wrote every byte, flushed, closed — as far as the card REPORTED

*enumerator, declared at [`include/shulib/hal/file_store.hpp:91`](../../include/shulib/hal/file_store.hpp#L91).*

<a id="replacestep-nocard"></a>

### `ReplaceStep::NoCard`

```cpp
NoCard
```

no card: nothing was opened, nothing was created, the file is untouched

*enumerator, declared at [`include/shulib/hal/file_store.hpp:92`](../../include/shulib/hal/file_store.hpp#L92).*

<a id="replacestep-open"></a>

### `ReplaceStep::Open`

```cpp
Open
```

a card, but the truncating open was refused; the file is untouched

*enumerator, declared at [`include/shulib/hal/file_store.hpp:93`](../../include/shulib/hal/file_store.hpp#L93).*

<a id="replacestep-write"></a>

### `ReplaceStep::Write`

```cpp
Write
```

the open succeeded (the old contents are GONE) and the write fell short

*enumerator, declared at [`include/shulib/hal/file_store.hpp:94`](../../include/shulib/hal/file_store.hpp#L94).*

<a id="replacestep-flush"></a>

### `ReplaceStep::Flush`

```cpp
Flush
```

every byte written, and the flush reported failure (the bytes may still be there)

*enumerator, declared at [`include/shulib/hal/file_store.hpp:95`](../../include/shulib/hal/file_store.hpp#L95).*

<a id="struct-storereplace"></a>

## `struct StoreReplace`

```cpp
struct StoreReplace
```

What one replace() did: the step it stopped at, and the path it used.

*struct, declared at [`include/shulib/hal/file_store.hpp:99`](../../include/shulib/hal/file_store.hpp#L99).*

<a id="storereplace-failed"></a>

### `StoreReplace::failed`

```cpp
ReplaceStep failed = ReplaceStep::None
```

The step that failed, or `None` (which proves nothing about the medium — header note).

*field, declared at [`include/shulib/hal/file_store.hpp:101`](../../include/shulib/hal/file_store.hpp#L101).*

<a id="storereplace-path"></a>

### `StoreReplace::path`

```cpp
char path[kStorePathBytes] = ""
```

The path the replace used — on the brain the adapter's `/usd/<name>` — for messages only.

*field, declared at [`include/shulib/hal/file_store.hpp:103`](../../include/shulib/hal/file_store.hpp#L103).*

<a id="class-ifilestore"></a>

## `class IFileStore`

```cpp
class IFileStore
```

The brain's SD card as a program can actually use it: READ a whole named file from its start, or REPLACE a named file by truncating it. Deliberately NO rename, NO remove, NO exists-by-stat and NO read-write open — the brain has none of them (HA-166, header). Names are BARE (no leading '/'): the implementation owns the mount prefix. Not noexcept: a name that is not bare is a thrown PreconditionError, while every refusal by the card is a returned value.

*class, declared at [`include/shulib/hal/file_store.hpp:111`](../../include/shulib/hal/file_store.hpp#L111).*

<a id="ifilestore-destructor-ifilestore"></a>

### `IFileStore::~IFileStore`

```cpp
virtual ~IFileStore() = default
```

The polymorphic-base special members: a virtual destructor so a store may be owned and destroyed through this interface, and defaulted copy/move because the interface itself holds no state. Stores are held BY REFERENCE by the save and the load.

*function, declared at [`include/shulib/hal/file_store.hpp:116`](../../include/shulib/hal/file_store.hpp#L116).*

<a id="ifilestore-ifilestore"></a>

### `IFileStore::IFileStore`

```cpp
IFileStore() = default
```

*Covered by the comment on [`~IFileStore`](#ifilestore-destructor-ifilestore) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/file_store.hpp:117`](../../include/shulib/hal/file_store.hpp#L117).*

<a id="ifilestore-ifilestore-2"></a>

### `IFileStore::IFileStore (overload 2)`

```cpp
IFileStore(const IFileStore&) = default
```

*Covered by the comment on [`~IFileStore`](#ifilestore-destructor-ifilestore) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/file_store.hpp:118`](../../include/shulib/hal/file_store.hpp#L118).*

<a id="ifilestore-ifilestore-3"></a>

### `IFileStore::IFileStore (overload 3)`

```cpp
IFileStore(IFileStore&&) = default
```

*Covered by the comment on [`~IFileStore`](#ifilestore-destructor-ifilestore) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/file_store.hpp:119`](../../include/shulib/hal/file_store.hpp#L119).*

<a id="ifilestore-operator-eq"></a>

### `IFileStore::operator=`

```cpp
IFileStore& operator=(const IFileStore&) = default
```

*Covered by the comment on [`~IFileStore`](#ifilestore-destructor-ifilestore) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/file_store.hpp:120`](../../include/shulib/hal/file_store.hpp#L120).*

<a id="ifilestore-operator-eq-2"></a>

### `IFileStore::operator= (overload 2)`

```cpp
IFileStore& operator=(IFileStore&&) = default
```

*Covered by the comment on [`~IFileStore`](#ifilestore-destructor-ifilestore) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/file_store.hpp:121`](../../include/shulib/hal/file_store.hpp#L121).*

<a id="ifilestore-cardpresent"></a>

### `IFileStore::cardPresent`

```cpp
[[nodiscard]] virtual bool cardPresent() const = 0
```

True when a card is in the brain, as the platform reports it RIGHT NOW (on the brain, `usd_is_installed()`; HA-122 (1) — confirmed at boot, never for a card pulled mid-run).

*function, declared at [`include/shulib/hal/file_store.hpp:125`](../../include/shulib/hal/file_store.hpp#L125).*

<a id="ifilestore-read"></a>

### `IFileStore::read`

```cpp
[[nodiscard]] virtual StoreRead read(const char* bareName, std::span<std::byte> into) = 0
```

Read the file `bareName` from its START into `into`: up to `into.size()` bytes. The result says whether it opened, how many bytes came back, and the path used. Changes no file.

*function, declared at [`include/shulib/hal/file_store.hpp:129`](../../include/shulib/hal/file_store.hpp#L129).*

<a id="ifilestore-replace"></a>

### `IFileStore::replace`

```cpp
[[nodiscard]] virtual StoreReplace replace(const char* bareName, std::span<const std::byte> bytes) = 0
```

Replace the file `bareName` with `bytes`: open it TRUNCATING (treat the old contents as gone from here — HA-166 (1)), write, flush, close. Creates the file if it does not exist. The result names the step that failed; `None` proves nothing about the medium (header note) — read the file back to know.

*function, declared at [`include/shulib/hal/file_store.hpp:135`](../../include/shulib/hal/file_store.hpp#L135).*

<a id="samefileoncard"></a>

## `sameFileOnCard`

```cpp
[[nodiscard]] constexpr bool sameFileOnCard(const char* a, const char* b) noexcept
```

True when FAT's name rule makes `a` and `b` the SAME file (header: "Two names, one file"): equal once ASCII letter case is ignored and trailing dots and spaces are dropped. The brain's card is taken to follow that rule (HA-166 (6); its VEXos half is not measured). So "gains.bin" matches "GAINS.BIN" and "gains.bin.", and not "gains.bak". constexpr, so two name constants can be checked at compile time. Null matches only null. (A FAT short-name alias such as "SHULIB~1.BIN" is not checked: no caller names a file that way.)

*free function, declared at [`include/shulib/hal/file_store.hpp:145`](../../include/shulib/hal/file_store.hpp#L145).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 59 lines, click to expand</summary>

```text

 IFileStore — the two things the brain's SD card offers that a SAFE save can be built on (chunk
 SAFE-SAVE Part A, 2026-09-29): read a whole named file, or replace one by truncating it. The
 brain has no rename, no delete, no stat and no read-write open (below). PROS does pass an "ab"
 (append) open to a different VEXos call; what VEXos does with it is not measured, and an append
 is not a replace, so this seam leaves it out.

 ── Why a store, and why it is a SIBLING of IBlockSink / IBlockSource ───────────────
 IBlockSink and IBlockSource are single-file HANDLES: an adapter opens its one file at
 construction and closes it at destruction, which is exactly right for a blackbox (one file per
 boot) and for the boot's one read of the calibration record. The calibration record's SAFE save
 needs TWO named files — the record and its backup copy — each opened, written, closed and read
 back several times in one save, in an order that is the whole point
 (calibration/record_store.hpp).
 Widening either handle into "open any file, any number of times" would change what every
 existing user of it holds. So this is a new, additive seam over the same two adapters: the PROS
 implementation (hal/pros/file_store.hpp) builds a ProsBlockSource or a ProsBlockSink per call,
 each in its own scope, and the /usd/ prefix stays owned where it already is (HA-122).

 ── What is DELIBERATELY missing: rename, remove, stat, a read-write open (HA-166) ──
 What the vendored libpros.a and libc.a show (HA-166, read 2026-09-26), and what they leave to
 VEXos:
   (1) fopen "wb" reaches VEXos's create-a-file call, which is TAKEN to empty an existing file at
       the OPEN, before a byte is written (reasoned from those libraries, not measured). An "ab"
       open reaches a different VEXos call, vexFileOpenWrite, instead — what that call does to
       an existing file is VEXos's, and not measured;
   (2) newlib's rename() is _link_r + _unlink_r, and remove()/unlink() are _unlink_r; PROS 4.2.2
       defines _link, _unlink, _stat and mkdir as stubs that fail with ENOSYS. So there is no
       rename, no delete, no directory, and no stat() to ask whether a file exists;
   (3) PROS's usd_open_r refuses a read-write open (EINVAL): no file can be updated in place.
 A seam offering any of those would pass every HOST test (Linux has them) and fail on the robot.
 So the seam has none of them, and a file EXISTS when read() opens it.

 ── Two names, one file (HA-166 (6)) ────────────────────────────────────────────────
 FAT matches a long name without regard to letter case and drops trailing dots and spaces, so
 under FAT "gains.bin", "GAINS.BIN" and "gains.bin." are ONE file, while Linux, and so every
 host test, holds three. The brain's card is TAKEN to follow FAT's rule (HA-166 (6): FAT's rule
 is documented; that VEXos applies it is not measured). Were it so, a save whose backup aliased
 its primary would skip the backup (it reads back identical) and then truncate the only copy.
 sameFileOnCard() below is FAT's rule, and the safe save's preconditions use it; if the card
 tells such names apart after all, the check only refuses a few name pairs it need not have.

 ── What replace() promises, and what it cannot ────────────────────────────────────
   * The old contents are to be treated as GONE from the open onward (HA-166 (1)). A replace that
     fails at its write or its flush has already destroyed what was there.
   * `ReplaceStep::None` does NOT mean the bytes are on the card. The writer's close cannot report
     a failure (hal/pros/block_sink.hpp's destructor, and PROS's close returns 0 whatever VEXos
     answers — HA-166 (5)). Only a READ-BACK proves anything, and the caller does it: see
     calibration/record_store.hpp's put(), which never decides from what replace() reported.

 ── Exceptions and allocation ──────────────────────────────────────────────────────
 The methods are NOT noexcept. An implementation may throw PreconditionError for a name that is
 not bare, plus whatever its adapters' construction throws (core/check.hpp: both shipped policies
 throw; the PROS adapters allocate their path string at construction). A refusal by the CARD —
 no card, an open refused, a short write, a failed flush, a short read — is always a RETURNED
 value, never a throw. A noexcept wrapper would turn an adapter's precondition into
 std::terminate with no message. This header itself allocates nothing.

 Additive; not one of the frozen F4 seams. PROS-free. HA register: HA-166, HA-122.
```

</details>
