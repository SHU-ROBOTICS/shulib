<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/blackbox_reader.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `blackbox_reader.hpp`

BlackboxReader — THE DECODER. It ships in the same chunk as the encoder, because a format nothing can read is not a record: the first time a blackbox genuinely matters is a competition afternoon, and a file that cannot be opened that afternoon is worth exac…

This header declares **3** types (19 members) and **1** free function.

Extracted from [`include/shulib/diag/blackbox_reader.hpp`](../../include/shulib/diag/blackbox_reader.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ReadStatus`](#enum-class-readstatus)
  - [`Ok`](#readstatus-ok)
  - [`Empty`](#readstatus-empty)
  - [`HeaderTruncated`](#readstatus-headertruncated)
  - [`BadMagic`](#readstatus-badmagic)
  - [`UnsupportedVersion`](#readstatus-unsupportedversion)
  - [`LayoutMismatch`](#readstatus-layoutmismatch)
- [`readStatusName`](#readstatusname) — *free function*
- [`class BlackboxReader`](#class-blackboxreader)
  - [`BlackboxReader`](#blackboxreader-blackboxreader)
  - [`status`](#blackboxreader-status)
  - [`usable`](#blackboxreader-usable)
  - [`header`](#blackboxreader-header)
  - [`next`](#blackboxreader-next)
  - [`truncated`](#blackboxreader-truncated)
  - [`truncatedFrameType`](#blackboxreader-truncatedframetype)
  - [`sawEnd`](#blackboxreader-sawend)
  - [`framesRead`](#blackboxreader-framesread)
  - [`skippedFrames`](#blackboxreader-skippedframes)
  - [`bytesConsumed`](#blackboxreader-bytesconsumed)
  - [`struct BlackboxReader::Frame`](#struct-blackboxreader-frame)
    - [`type`](#blackboxreader-frame-type)
    - [`payload`](#blackboxreader-frame-payload)

<a id="enum-class-readstatus"></a>

## `enum class ReadStatus`

```cpp
enum class ReadStatus : std::uint8_t
```

Why a file is or is not readable by THIS build. Anything other than Ok means no frames are delivered at all — the refuse-don't-misread rule.

*enum class, declared at [`include/shulib/diag/blackbox_reader.hpp:68`](../../include/shulib/diag/blackbox_reader.hpp#L68).*

<a id="readstatus-ok"></a>

### `ReadStatus::Ok`

```cpp
Ok = 0
```

header parsed and this build can read this version

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:69`](../../include/shulib/diag/blackbox_reader.hpp#L69).*

<a id="readstatus-empty"></a>

### `ReadStatus::Empty`

```cpp
Empty = 1
```

nothing at all was written (a run with nothing to say)

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:70`](../../include/shulib/diag/blackbox_reader.hpp#L70).*

<a id="readstatus-headertruncated"></a>

### `ReadStatus::HeaderTruncated`

```cpp
HeaderTruncated = 2
```

fewer bytes than a complete header

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:71`](../../include/shulib/diag/blackbox_reader.hpp#L71).*

<a id="readstatus-badmagic"></a>

### `ReadStatus::BadMagic`

```cpp
BadMagic = 3
```

not a shulib blackbox

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:72`](../../include/shulib/diag/blackbox_reader.hpp#L72).*

<a id="readstatus-unsupportedversion"></a>

### `ReadStatus::UnsupportedVersion`

```cpp
UnsupportedVersion = 4
```

a version this build was not written for — REFUSED

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:73`](../../include/shulib/diag/blackbox_reader.hpp#L73).*

<a id="readstatus-layoutmismatch"></a>

### `ReadStatus::LayoutMismatch`

```cpp
LayoutMismatch = 5
```

right version, wrong record width — REFUSED (header note)

*enumerator, declared at [`include/shulib/diag/blackbox_reader.hpp:74`](../../include/shulib/diag/blackbox_reader.hpp#L74).*

<a id="readstatusname"></a>

## `readStatusName`

```cpp
[[nodiscard]] constexpr const char* readStatusName(ReadStatus s) noexcept
```

The §18.5 spelling of a ReadStatus, for messages. Never returns null.

*free function, declared at [`include/shulib/diag/blackbox_reader.hpp:78`](../../include/shulib/diag/blackbox_reader.hpp#L78).*

<a id="class-blackboxreader"></a>

## `class BlackboxReader`

```cpp
class BlackboxReader
```

The decoder for a blackbox file: point it at bytes the caller already holds — it BORROWS the span, copies nothing and allocates nothing — and pull frames with next() until that returns false. Three rules it will not bend. A file this build cannot interpret is REFUSED WHOLE: status() says why and not one frame is delivered, because a number read wrongly but confidently sends an investigation somewhere false. It never throws and never reads past the end, which matters most for the damaged files that matter most. And truncation is a RESULT, not an error: every frame before the cut is delivered, truncated() is true, sawEnd() is false — the exact signature a brownout leaves. Frame kinds this build does not know are skipped by their declared length and counted in skippedFrames(), which is what lets a v1 reader survive a later writer. There is deliberately NO per-frame checksum, so a bit flip landing on a merely-wrong value inside a known frame decodes silently as that value (measured, not assumed — the header states the boundary in full). A consumer needing end-to-end integrity must add its own check rather than inherit one from here that does not exist.

*class, declared at [`include/shulib/diag/blackbox_reader.hpp:104`](../../include/shulib/diag/blackbox_reader.hpp#L104).*

<a id="blackboxreader-blackboxreader"></a>

### `BlackboxReader::BlackboxReader`

```cpp
explicit BlackboxReader(std::span<const std::byte> file) noexcept
```

Parse the header of `file` and position at the first frame. Never throws; the verdict is in status(). The span must outlive the reader.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:115`](../../include/shulib/diag/blackbox_reader.hpp#L115).*

<a id="blackboxreader-status"></a>

### `BlackboxReader::status`

```cpp
[[nodiscard]] ReadStatus status() const noexcept
```

Whether this build can read this file at all (header note, rule 1).

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:142`](../../include/shulib/diag/blackbox_reader.hpp#L142).*

<a id="blackboxreader-usable"></a>

### `BlackboxReader::usable`

```cpp
[[nodiscard]] bool usable() const noexcept
```

Shorthand for status() == Ok.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:144`](../../include/shulib/diag/blackbox_reader.hpp#L144).*

<a id="blackboxreader-header"></a>

### `BlackboxReader::header`

```cpp
[[nodiscard]] const BlackboxHeader& header() const noexcept
```

The decoded header. Meaningful for Ok and UnsupportedVersion (a refused file still tells you which version it claims to be — that is how you find the build that wrote it).

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:148`](../../include/shulib/diag/blackbox_reader.hpp#L148).*

<a id="blackboxreader-next"></a>

### `BlackboxReader::next`

```cpp
[[nodiscard]] bool next(Frame& out) noexcept
```

Deliver the next KNOWN frame, skipping (and counting) unknown ones. Returns false at the end of the file, on a cut, or when the file is not usable.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:152`](../../include/shulib/diag/blackbox_reader.hpp#L152).*

<a id="blackboxreader-truncated"></a>

### `BlackboxReader::truncated`

```cpp
[[nodiscard]] bool truncated() const noexcept
```

True when the file ended mid-frame — the run was cut short (brownout, pulled card, a program that never closed). The frames delivered before the cut are all valid; this says the story stops there.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:193`](../../include/shulib/diag/blackbox_reader.hpp#L193).*

<a id="blackboxreader-truncatedframetype"></a>

### `BlackboxReader::truncatedFrameType`

```cpp
[[nodiscard]] std::uint8_t truncatedFrameType() const noexcept
```

The raw frame-type byte of the frame that was cut, when truncated() (0 if the cut fell in a frame prefix).

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:196`](../../include/shulib/diag/blackbox_reader.hpp#L196).*

<a id="blackboxreader-sawend"></a>

### `BlackboxReader::sawEnd`

```cpp
[[nodiscard]] bool sawEnd() const noexcept
```

True once the graceful-end frame has been delivered. Its ABSENCE at the end of iteration is the honest signal that the run did not close cleanly.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:199`](../../include/shulib/diag/blackbox_reader.hpp#L199).*

<a id="blackboxreader-framesread"></a>

### `BlackboxReader::framesRead`

```cpp
[[nodiscard]] std::uint32_t framesRead() const noexcept
```

Known frames delivered so far.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:201`](../../include/shulib/diag/blackbox_reader.hpp#L201).*

<a id="blackboxreader-skippedframes"></a>

### `BlackboxReader::skippedFrames`

```cpp
[[nodiscard]] std::uint32_t skippedFrames() const noexcept
```

Frames skipped because this build does not know their type, or because a known type carried a payload of the wrong size (a corruption signal that must not stop the rest of the file from being read).

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:205`](../../include/shulib/diag/blackbox_reader.hpp#L205).*

<a id="blackboxreader-bytesconsumed"></a>

### `BlackboxReader::bytesConsumed`

```cpp
[[nodiscard]] std::size_t bytesConsumed() const noexcept
```

Bytes consumed so far, including the header.

*function, declared at [`include/shulib/diag/blackbox_reader.hpp:207`](../../include/shulib/diag/blackbox_reader.hpp#L207).*

<a id="struct-blackboxreader-frame"></a>

## `struct BlackboxReader::Frame`

```cpp
struct Frame
```

One frame as delivered by next(): its type and a view of its payload, valid for as long as the caller's file buffer is.

*struct, declared at [`include/shulib/diag/blackbox_reader.hpp:108`](../../include/shulib/diag/blackbox_reader.hpp#L108).*

<a id="blackboxreader-frame-type"></a>

### `BlackboxReader::Frame::type`

```cpp
FrameType type = FrameType::Tick
```

the frame's kind (known types only)

*field, declared at [`include/shulib/diag/blackbox_reader.hpp:109`](../../include/shulib/diag/blackbox_reader.hpp#L109).*

<a id="blackboxreader-frame-payload"></a>

### `BlackboxReader::Frame::payload`

```cpp
std::span<const std::byte> payload{}
```

exactly the declared payload bytes

*field, declared at [`include/shulib/diag/blackbox_reader.hpp:110`](../../include/shulib/diag/blackbox_reader.hpp#L110).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

```text

 BlackboxReader — THE DECODER. It ships in the same chunk as the encoder, because a
 format nothing can read is not a record: the first time a blackbox genuinely matters
 is a competition afternoon, and a file that cannot be opened that afternoon is worth
 exactly nothing. (WS13, chunk E1; format in diag/blackbox_format.hpp.)

 It is also the thing that makes the encoder TESTABLE. The round trip — encode a known
 stream, decode it, compare field by field — is the chunk's central test, and it needs
 both halves to exist in the same suite.

 ── Three rules it will not bend ────────────────────────────────────────────────────
 1. REFUSE, NEVER MISREAD. A file whose version this build does not know is rejected
    whole; so is a file whose self-declared tick width disagrees with this build's.
    A wrong number read confidently is worse than no number: it sends an investigation
    somewhere false with full confidence. (The width cross-check exists for one very
    human failure: changing the layout and forgetting to bump the version.)
 2. NEVER THROW, NEVER READ PAST THE END. The files that matter most are the damaged
    ones — a run that died mid-write, a card pulled at the wrong moment. Every read
    goes through a bounds-latching cursor, and a non-finite heading (which the Angle
    type refuses by precondition) decodes to zero with `corrupt()` raised rather than
    aborting the program that is trying to read the evidence.
 3. TRUNCATION IS A RESULT, NOT AN ERROR. A cut file decodes up to the cut and SAYS
    SO: truncated() is true, the frames before the cut are all delivered, and sawEnd()
    is false because the graceful-end frame never arrived. That combination — good
    frames, no end stamp — is exactly what a brownout leaves behind.

 Forward compatibility: a frame type this build does not know is SKIPPED BY ITS
 DECLARED LENGTH and counted in skippedFrames(). That is what lets a v1 reader survive
 a later writer that appended a frame kind, without ever guessing at its content.

 ── The boundary of rule 1, stated so nobody over-trusts it ─────────────────────────
 "Refuse, never misread" is about INTERPRETATION, not INTEGRITY, and the difference
 matters to anyone building on this format (H1's SHUL/2 in particular).

 What is detected: a file that is not a blackbox (BadMagic), a version this build does
 not know (UnsupportedVersion), a layout whose declared widths disagree (LayoutMismatch),
 a cut (truncated()), a frame kind this build lacks (skippedFrames()), and a decoded
 value that is implausible on its face — a non-finite heading raises corrupt() rather
 than tripping Angle's precondition.

 What is NOT detected: there is deliberately NO per-frame checksum, so a bit flip inside
 a frame this build CAN interpret, landing on a value that is merely wrong rather than
 impossible, decodes silently as that wrong value. That was measured, not assumed:
 flipping a payload byte mid-file leaves status Ok and every frame delivered.

 This is a considered trade, not an oversight — a CRC costs bytes and cycles on every
 tick of a fixed-width budget, and the SD card already carries hardware ECC beneath us.
 Recorded because the honest scope of a guarantee is part of the guarantee: if a future
 consumer needs end-to-end integrity (a wire protocol over a lossy link is the obvious
 case, and F9 is exactly that), it must add its own frame check rather than inherit one
 from here that does not exist. (Boundary verified during E1's independent review.)

 Allocation-free and PROS-free, like everything in this tree: it reads a span the
 caller already holds (a whole file loaded into a buffer, or the bytes a test just
 captured), and iterates.
```

</details>
