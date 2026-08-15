<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/block_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_sink.hpp`

IBlockSink — where BINARY BLOCKS physically go (an SD-card file on the brain, a captured buffer in a test).

This header declares **1** type (8 members).

Extracted from [`include/shulib/hal/block_sink.hpp`](../../include/shulib/hal/block_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IBlockSink`](#class-iblocksink)
  - [`~IBlockSink`](#iblocksink-destructor-iblocksink)
  - [`IBlockSink`](#iblocksink-iblocksink)
  - [`IBlockSink (overload 2)`](#iblocksink-iblocksink-2)
  - [`IBlockSink (overload 3)`](#iblocksink-iblocksink-3)
  - [`operator=`](#iblocksink-operator-eq)
  - [`operator= (overload 2)`](#iblocksink-operator-eq-2)
  - [`write`](#iblocksink-write)
  - [`flush`](#iblocksink-flush)

<a id="class-iblocksink"></a>

## `class IBlockSink`

```cpp
class IBlockSink
```

The seam BINARY BLOCKS leave through — an SD-card file on the brain, a captured buffer in a test — so "the file has these exact bytes" is a claim a host test can assert instead of one somebody eyeballs. Deliberately a SIBLING of ICharSink rather than a widening of it: ICharSink promises TEXT, one complete LINE per call, and that promise is what makes TermSink's framing provable; a blackbox is fixed-width binary carrying every byte value, 0x00 and 0x0A included. Framing, escaping and structure belong to the FORMAT, never to this interface. Additive, not one of the frozen F4 ten.

*class, declared at [`include/shulib/hal/block_sink.hpp:49`](../../include/shulib/hal/block_sink.hpp#L49).*

<a id="iblocksink-destructor-iblocksink"></a>

### `IBlockSink::~IBlockSink`

```cpp
virtual ~IBlockSink() = default
```

The polymorphic-base special members: a virtual destructor so a sink may be owned and destroyed through this interface, and defaulted copy/move because the interface itself holds no state. The shipped sinks are held BY REFERENCE and must outlive their user — nothing here owns, opens or closes the underlying device.

*function, declared at [`include/shulib/hal/block_sink.hpp:55`](../../include/shulib/hal/block_sink.hpp#L55).*

<a id="iblocksink-iblocksink"></a>

### `IBlockSink::IBlockSink`

```cpp
IBlockSink() = default
```

*Covered by the comment on [`~IBlockSink`](#iblocksink-destructor-iblocksink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_sink.hpp:56`](../../include/shulib/hal/block_sink.hpp#L56).*

<a id="iblocksink-iblocksink-2"></a>

### `IBlockSink::IBlockSink (overload 2)`

```cpp
IBlockSink(const IBlockSink&) = default
```

*Covered by the comment on [`~IBlockSink`](#iblocksink-destructor-iblocksink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_sink.hpp:57`](../../include/shulib/hal/block_sink.hpp#L57).*

<a id="iblocksink-iblocksink-3"></a>

### `IBlockSink::IBlockSink (overload 3)`

```cpp
IBlockSink(IBlockSink&&) = default
```

*Covered by the comment on [`~IBlockSink`](#iblocksink-destructor-iblocksink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_sink.hpp:58`](../../include/shulib/hal/block_sink.hpp#L58).*

<a id="iblocksink-operator-eq"></a>

### `IBlockSink::operator=`

```cpp
IBlockSink& operator=(const IBlockSink&) = default
```

*Covered by the comment on [`~IBlockSink`](#iblocksink-destructor-iblocksink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_sink.hpp:59`](../../include/shulib/hal/block_sink.hpp#L59).*

<a id="iblocksink-operator-eq-2"></a>

### `IBlockSink::operator= (overload 2)`

```cpp
IBlockSink& operator=(IBlockSink&&) = default
```

*Covered by the comment on [`~IBlockSink`](#iblocksink-destructor-iblocksink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/block_sink.hpp:60`](../../include/shulib/hal/block_sink.hpp#L60).*

<a id="iblocksink-write"></a>

### `IBlockSink::write`

```cpp
[[nodiscard]] virtual bool write(std::span<const std::byte> bytes) noexcept = 0
```

Write `bytes` verbatim to the device. Returns false if any byte was not accepted (a partial write may leave a prefix behind — header note). MUST NOT throw.

*function, declared at [`include/shulib/hal/block_sink.hpp:65`](../../include/shulib/hal/block_sink.hpp#L65).*

<a id="iblocksink-flush"></a>

### `IBlockSink::flush`

```cpp
virtual bool flush() noexcept
```

Push any device-side buffering out to the medium (fflush/fsync on the robot). Returns false if the device reported a failure. NON-pure with a default success body, so an implementation with no buffering of its own — the common case, including the test fake — stays a two-line class. MUST NOT throw.

*function, declared at [`include/shulib/hal/block_sink.hpp:71`](../../include/shulib/hal/block_sink.hpp#L71).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 IBlockSink — where BINARY BLOCKS physically go (an SD-card file on the brain, a
 captured buffer in a test). The sibling of ICharSink, added at chunk E1 for the
 blackbox (SdSink): it is the same idea one layer over — the DEVICE is injected, so
 "the file has these exact bytes" is a testable claim instead of an eyeballed one.

 ── Why a NEW seam instead of reusing ICharSink (chunk E1, tension T2) ──────────────
 ICharSink's contract is text and LINE-oriented: "one write() call carries one
 complete line", which is what makes TermSink's framing proof possible (a sink that
 is atomic per call can never interleave lines). A blackbox is neither text nor
 line-oriented — it is fixed-width binary containing every byte value including 0x00
 and 0x0A. Redefining ICharSink to mean "bytes, maybe lines, maybe not" would keep
 one seam at the cost of that seam meaning nothing, and would silently invalidate
 the framing argument every TermSink golden rests on. ICharSink is deliberately NOT
 part of the frozen F4 ten (it is an additive diagnostics-output seam), so a SIBLING
 is cheap and honest. The alternative considered and rejected: base64/hex text over
 ICharSink — a text blackbox, explicitly rejected by the E1 brief, paying ~33% more
 bytes for a format that still needs a parser.

 Contract:
   * write() takes the bytes VERBATIM — no framing, no escaping, no sanitization
     (the FORMAT owns its own structure; see diag/blackbox_format.hpp).
   * It is called SYNCHRONOUSLY on the caller's task and MUST NOT throw (noexcept).
   * It returns FALSE if the device did not accept every byte. That return value is
     the whole point of the seam being bool-valued: an SD card that fills up, is
     yanked, or dies mid-write is the NORMAL failure of a blackbox, and a void
     write() would make it invisible. A caller that ignores the result cannot notice
     a truncated file, so the result is [[nodiscard]].
   * A partial write may leave a PREFIX of the bytes on the device. The format is
     designed for exactly that (a truncated blackbox decodes up to the cut and says
     so) — see diag/blackbox_reader.hpp.

 R1 owns the on-robot /usd/ adapter (PROS FILE* behind this interface); E1 ships the
 interface and hal::fake::FakeBlockSink.
```

</details>
