<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/block_source.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_source.hpp`

IBlockSource — where BINARY BLOCKS come FROM (an SD-card file on the brain, a canned buffer in a test).

This header declares **1** type (8 members).

Extracted from [`include/shulib/hal/block_source.hpp`](../../include/shulib/hal/block_source.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IBlockSource`](#class-iblocksource)
  - [`~IBlockSource`](#iblocksource-destructor-iblocksource)
  - [`IBlockSource`](#iblocksource-iblocksource)
  - [`IBlockSource (overload 2)`](#iblocksource-iblocksource-2)
  - [`IBlockSource (overload 3)`](#iblocksource-iblocksource-3)
  - [`operator=`](#iblocksource-operator-eq)
  - [`operator= (overload 2)`](#iblocksource-operator-eq-2)
  - [`readAll`](#iblocksource-readall)
  - [`isOpen`](#iblocksource-isopen)

<a id="class-iblocksource"></a>

## `class IBlockSource`

```cpp
class IBlockSource
```

The seam BINARY BLOCKS arrive through — an SD-card file on the brain, a canned buffer in a test — so "the file on the card said exactly these bytes" is a claim a host test can assert. The read sibling of IBlockSink, and deliberately whole-file rather than streaming (header). Additive, not one of the frozen F4 ten.

*class, declared at [`include/shulib/hal/block_source.hpp:49`](../../include/shulib/hal/block_source.hpp#L49).*

<a id="iblocksource-destructor-iblocksource"></a>

### `IBlockSource::~IBlockSource`

```cpp
virtual ~IBlockSource() = default
```

The polymorphic-base special members: a virtual destructor so a source may be owned and destroyed through this interface, and defaulted copy/move because the interface itself holds no state. The shipped sources are held BY REFERENCE and must outlive their user — nothing here owns, opens or closes the underlying device.

*function, declared at [`include/shulib/hal/block_source.hpp:55`](../../include/shulib/hal/block_source.hpp#L55).*

<a id="iblocksource-iblocksource"></a>

### `IBlockSource::IBlockSource`

```cpp
IBlockSource() = default
```

*Covered by the comment on [`~IBlockSource`](#iblocksource-destructor-iblocksource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_source.hpp:56`](../../include/shulib/hal/block_source.hpp#L56).*

<a id="iblocksource-iblocksource-2"></a>

### `IBlockSource::IBlockSource (overload 2)`

```cpp
IBlockSource(const IBlockSource&) = default
```

*Covered by the comment on [`~IBlockSource`](#iblocksource-destructor-iblocksource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_source.hpp:57`](../../include/shulib/hal/block_source.hpp#L57).*

<a id="iblocksource-iblocksource-3"></a>

### `IBlockSource::IBlockSource (overload 3)`

```cpp
IBlockSource(IBlockSource&&) = default
```

*Covered by the comment on [`~IBlockSource`](#iblocksource-destructor-iblocksource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_source.hpp:58`](../../include/shulib/hal/block_source.hpp#L58).*

<a id="iblocksource-operator-eq"></a>

### `IBlockSource::operator=`

```cpp
IBlockSource& operator=(const IBlockSource&) = default
```

*Covered by the comment on [`~IBlockSource`](#iblocksource-destructor-iblocksource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_source.hpp:59`](../../include/shulib/hal/block_source.hpp#L59).*

<a id="iblocksource-operator-eq-2"></a>

### `IBlockSource::operator= (overload 2)`

```cpp
IBlockSource& operator=(IBlockSource&&) = default
```

*Covered by the comment on [`~IBlockSource`](#iblocksource-destructor-iblocksource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_source.hpp:60`](../../include/shulib/hal/block_source.hpp#L60).*

<a id="iblocksource-readall"></a>

### `IBlockSource::readAll`

```cpp
[[nodiscard]] virtual std::size_t readAll(std::span<std::byte> into) noexcept = 0
```

Copy up to `into.size()` bytes from the start of the source into `into` and return how many were copied. 0 = nothing (no device, no file, or an empty file — isOpen() tells them apart); a count below `into.size()` means the source ran out, which is a TRUNCATED source and the format's business, not an error here. MUST NOT throw.

*function, declared at [`include/shulib/hal/block_source.hpp:66`](../../include/shulib/hal/block_source.hpp#L66).*

<a id="iblocksource-isopen"></a>

### `IBlockSource::isOpen`

```cpp
[[nodiscard]] virtual bool isOpen() const noexcept = 0
```

False = this source is refusing: there is no device, or the open failed. Checked ONCE by the composition root, which owns saying it out loud — the same T5 visibility rule IBlockSink's adapter follows.

*function, declared at [`include/shulib/hal/block_source.hpp:71`](../../include/shulib/hal/block_source.hpp#L71).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 37 lines</summary>

```text

 IBlockSource — where BINARY BLOCKS come FROM (an SD-card file on the brain, a
 canned buffer in a test). The READ sibling of IBlockSink, added at chunk R3d Part 2
 for the calibration record: a description measured on one boot has to be read back
 on the next one, and the read half is where "refuse, don't misread" begins.

 ── Why a whole-file read, and not a stream ─────────────────────────────────────────
 Everything this seam carries is a SMALL, FIXED-WIDTH, self-checking record read ONCE at
 boot (calibration/calibration_record.hpp: 180 bytes under a CRC). A stream interface —
 open/read/seek/tell/eof — would be five more ways for an adapter to be subtly wrong about
 a file whose whole content fits in a stack buffer, and every one of them would need its
 own host test against a real filesystem. readAll() into the caller's span is the entire
 operation, so the adapter has exactly one behaviour to get right and the test has exactly
 one thing to pin. The day something wants to stream a big file back off the card (replay
 a blackbox on the brain?), THAT is the chunk that adds a streaming sibling — with its own
 reasoning — rather than this one growing a surface nobody needs yet.

 Contract:
   * readAll() copies up to `into.size()` bytes from the START of the source and returns
     how many it copied. It is NOT resumable: two calls on the same source may read the
     same bytes again, or fewer — an adapter is free to keep its file positioned wherever
     it likes. One call per boot is the intended use, and the record's own length check
     catches anything else.
   * It MUST NOT throw (noexcept) and MUST NOT allocate.
   * A return of 0 means "nothing came back", which is THREE different situations — no
     device, no such file, or an empty file. isOpen() tells them apart, and the caller
     that reports to a human needs to: "no SD card" and "no calibration on this card"
     send an operator to two different places.
   * A SHORT read (0 < n < into.size()) is a TRUNCATED source, not an error: the seam
     hands back what there was and the FORMAT decides. That is why the count is returned
     rather than a bool — a truncated calibration record must be refused BY NAME
     ("truncated"), and a bool would have thrown that away.
   * The result is [[nodiscard]]: a caller that ignores the count cannot tell a full
     record from an empty file, and would decode whatever its buffer happened to hold.

 hal/pros/block_source.hpp owns the on-robot /usd/ adapter; hal/fake/fake_block_source.hpp
 is the host fake. PROS-free, allocation-free.
```

</details>
