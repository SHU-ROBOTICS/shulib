<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/block_char_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `block_char_sink.hpp`

BlockCharSink — an ICharSink over an IBlockSink: text lines onto a block device such as the SD card.

This header declares **1** type (3 members).

Extracted from [`include/shulib/hal/block_char_sink.hpp`](../../include/shulib/hal/block_char_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class BlockCharSink`](#class-blockcharsink)
  - [`BlockCharSink`](#blockcharsink-blockcharsink)
  - [`write`](#blockcharsink-write)
  - [`droppedWrites`](#blockcharsink-droppedwrites)

<a id="class-blockcharsink"></a>

## `class BlockCharSink`

```cpp
class BlockCharSink final : public ICharSink
```

An ICharSink over a BORROWED IBlockSink (file banner): each write() is one block write, then flush(); a refusal of either is dropped and counted, never thrown. The block sink must outlive this, and nothing else may write through it between this sink's lines.

*class, declared at [`include/shulib/hal/block_char_sink.hpp:35`](../../include/shulib/hal/block_char_sink.hpp#L35).*

<a id="blockcharsink-blockcharsink"></a>

### `BlockCharSink::BlockCharSink`

```cpp
explicit BlockCharSink(IBlockSink& sink) noexcept
```

Borrow `sink` for this object's whole life.

*function, declared at [`include/shulib/hal/block_char_sink.hpp:38`](../../include/shulib/hal/block_char_sink.hpp#L38).*

<a id="blockcharsink-write"></a>

### `BlockCharSink::write`

```cpp
void write(std::string_view text) noexcept override
```

`text` verbatim as ONE block write, then a flush — both attempted every time (a failed write still flushes what the device took). A refusal of either counts one dropped line.

*function, declared at [`include/shulib/hal/block_char_sink.hpp:42`](../../include/shulib/hal/block_char_sink.hpp#L42).*

<a id="blockcharsink-droppedwrites"></a>

### `BlockCharSink::droppedWrites`

```cpp
[[nodiscard]] std::uint32_t droppedWrites() const noexcept
```

Lines the block sink refused (its write or its flush) since construction.

*function, declared at [`include/shulib/hal/block_char_sink.hpp:53`](../../include/shulib/hal/block_char_sink.hpp#L53).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 19 lines</summary>

```text

 BlockCharSink — an ICharSink over an IBlockSink: text lines onto a block device such as the SD
 card (chunk COMP-t, 2026-09-28). The driver-program chunk (T4) specified it for its SD copy of
 serial; it is built first here, to that specification, for the Auton Tester's SD run log.

 WHY: diag::TermSink writes its lines to an ICharSink, and the SD card's adapter
 (hal/pros/block_sink.hpp) is an IBlockSink. This joins them — each write() is ONE block write
 then ONE flush — so a second TermSink can put the same readable lines on the card that serial
 carries.

 A REFUSAL IS DROPPED AND COUNTED, NEVER THROWN. No card, a full card or a yanked card makes the
 block sink say no; that must never stop a robot (the same rule the blackbox follows —
 block_sink.hpp's contract). droppedWrites() counts every line a write OR its flush refused, so
 a caller can say "the card lost N lines" instead of pretending it lost none.

 ICharSink's contract (char_sink.hpp): bytes verbatim, synchronous, one complete line per call,
 never throws. This sink allocates nothing and holds nothing but the borrowed block sink and a
 count. PROS-free, header-only; host-tested with hal::fake::FakeBlockSink
 (test/block_char_sink_test.cpp). NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
