<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/shul2_reader.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `shul2_reader.hpp`

Shul2Reader — the SHUL/2 reader over a WHOLE capture.

This header declares **5** types (50 members) and **1** free function.

Extracted from [`include/shulib/diag/shul2_reader.hpp`](../../include/shulib/diag/shul2_reader.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class CaptureStatus`](#enum-class-capturestatus)
  - [`Empty`](#capturestatus-empty)
  - [`NoFrame`](#capturestatus-noframe)
  - [`NoSession`](#capturestatus-nosession)
  - [`Refused`](#capturestatus-refused)
  - [`Ok`](#capturestatus-ok)
- [`captureStatusName`](#capturestatusname) — *free function*
- [`struct ReaderCounters`](#struct-readercounters)
  - [`bytesSkipped`](#readercounters-bytesskipped)
  - [`framesVerified`](#readercounters-framesverified)
  - [`crcFailures`](#readercounters-crcfailures)
  - [`lengthOverruns`](#readercounters-lengthoverruns)
  - [`sessions`](#readercounters-sessions)
  - [`refusedSessions`](#readercounters-refusedsessions)
  - [`lateJoins`](#readercounters-latejoins)
  - [`framesDelivered`](#readercounters-framesdelivered)
  - [`framesOutsideSession`](#readercounters-framesoutsidesession)
  - [`framesInRefusedSessions`](#readercounters-framesinrefusedsessions)
  - [`unknownTypeSkipped`](#readercounters-unknowntypeskipped)
  - [`wrongWidthSkipped`](#readercounters-wrongwidthskipped)
  - [`extensionBytesIgnored`](#readercounters-extensionbytesignored)
  - [`staleFrames`](#readercounters-staleframes)
  - [`helloRepeats`](#readercounters-hellorepeats)
  - [`helloMismatches`](#readercounters-hellomismatches)
  - [`lostFrames`](#readercounters-lostframes)
  - [`linkLostFrames`](#readercounters-linklostframes)
  - [`sessionsLinkLossUnknown`](#readercounters-sessionslinklossunknown)
  - [`endDisagreements`](#readercounters-enddisagreements)
  - [`sessionsEnded`](#readercounters-sessionsended)
  - [`truncated`](#readercounters-truncated)
  - [`truncatedAt`](#readercounters-truncatedat)
- [`struct SessionReport`](#struct-sessionreport)
  - [`hello`](#sessionreport-hello)
  - [`verdict`](#sessionreport-verdict)
  - [`helloSeq`](#sessionreport-helloseq)
  - [`lateJoin`](#sessionreport-latejoin)
  - [`lostFrames`](#sessionreport-lostframes)
  - [`sawEnd`](#sessionreport-sawend)
  - [`end`](#sessionreport-end)
  - [`linkLossKnown`](#sessionreport-linklossknown)
  - [`linkLostFrames`](#sessionreport-linklostframes)
- [`class Shul2Reader`](#class-shul2reader)
  - [`Shul2Reader`](#shul2reader-shul2reader)
  - [`next`](#shul2reader-next)
  - [`drain`](#shul2reader-drain)
  - [`finished`](#shul2reader-finished)
  - [`status`](#shul2reader-status)
  - [`counters`](#shul2reader-counters)
  - [`hasSession`](#shul2reader-hassession)
  - [`latestSession`](#shul2reader-latestsession)
  - [`bytesConsumed`](#shul2reader-bytesconsumed)
  - [`struct Shul2Reader::Frame`](#struct-shul2reader-frame)
    - [`type`](#shul2reader-frame-type)
    - [`flags`](#shul2reader-frame-flags)
    - [`seq`](#shul2reader-frame-seq)
    - [`payload`](#shul2reader-frame-payload)

<a id="enum-class-capturestatus"></a>

## `enum class CaptureStatus`

```cpp
enum class CaptureStatus : std::uint8_t
```

One named verdict for a whole capture. Final once Shul2Reader::finished() is true.

*enum class, declared at [`include/shulib/diag/shul2_reader.hpp:99`](../../include/shulib/diag/shul2_reader.hpp#L99).*

<a id="capturestatus-empty"></a>

### `CaptureStatus::Empty`

```cpp
Empty = 0
```

no bytes at all

*enumerator, declared at [`include/shulib/diag/shul2_reader.hpp:100`](../../include/shulib/diag/shul2_reader.hpp#L100).*

<a id="capturestatus-noframe"></a>

### `CaptureStatus::NoFrame`

```cpp
NoFrame = 1
```

bytes, but no frame verified: a bit-7-stripping or NUL-truncating transport, or not SHUL/2 at all

*enumerator, declared at [`include/shulib/diag/shul2_reader.hpp:101`](../../include/shulib/diag/shul2_reader.hpp#L101).*

<a id="capturestatus-nosession"></a>

### `CaptureStatus::NoSession`

```cpp
NoSession = 2
```

frames verified, but no Hello: a newline-translating transport (the signature's CR LF kills every Hello), or a capture that ended before the first re-Hello

*enumerator, declared at [`include/shulib/diag/shul2_reader.hpp:103`](../../include/shulib/diag/shul2_reader.hpp#L103).*

<a id="capturestatus-refused"></a>

### `CaptureStatus::Refused`

```cpp
Refused = 3
```

every session was refused; the latest session's verdict says why

*enumerator, declared at [`include/shulib/diag/shul2_reader.hpp:106`](../../include/shulib/diag/shul2_reader.hpp#L106).*

<a id="capturestatus-ok"></a>

### `CaptureStatus::Ok`

```cpp
Ok = 4
```

at least one session was accepted

*enumerator, declared at [`include/shulib/diag/shul2_reader.hpp:107`](../../include/shulib/diag/shul2_reader.hpp#L107).*

<a id="capturestatusname"></a>

## `captureStatusName`

```cpp
[[nodiscard]] constexpr const char* captureStatusName(CaptureStatus s) noexcept
```

The §18.5 spelling of a CaptureStatus, for messages. Never returns null.

*free function, declared at [`include/shulib/diag/shul2_reader.hpp:111`](../../include/shulib/diag/shul2_reader.hpp#L111).*

<a id="struct-readercounters"></a>

## `struct ReaderCounters`

```cpp
struct ReaderCounters
```

Capture-wide counts, as of the bytes consumed so far (final once finished()). Byte counts are std::size_t; loss figures saturate rather than wrap.

*struct, declared at [`include/shulib/diag/shul2_reader.hpp:124`](../../include/shulib/diag/shul2_reader.hpp#L124).*

<a id="readercounters-bytesskipped"></a>

### `ReaderCounters::bytesSkipped`

```cpp
std::size_t bytesSkipped = 0
```

every byte not inside a verified frame

*field, declared at [`include/shulib/diag/shul2_reader.hpp:125`](../../include/shulib/diag/shul2_reader.hpp#L125).*

<a id="readercounters-framesverified"></a>

### `ReaderCounters::framesVerified`

```cpp
std::uint32_t framesVerified = 0
```

frames whose sync matched and CRC verified

*field, declared at [`include/shulib/diag/shul2_reader.hpp:126`](../../include/shulib/diag/shul2_reader.hpp#L126).*

<a id="readercounters-crcfailures"></a>

### `ReaderCounters::crcFailures`

```cpp
std::uint32_t crcFailures = 0
```

whole candidates whose CRC did not verify

*field, declared at [`include/shulib/diag/shul2_reader.hpp:127`](../../include/shulib/diag/shul2_reader.hpp#L127).*

<a id="readercounters-lengthoverruns"></a>

### `ReaderCounters::lengthOverruns`

```cpp
std::uint32_t lengthOverruns = 0
```

tail candidates (a NeedMore) a later frame displaced

*field, declared at [`include/shulib/diag/shul2_reader.hpp:128`](../../include/shulib/diag/shul2_reader.hpp#L128).*

<a id="readercounters-sessions"></a>

### `ReaderCounters::sessions`

```cpp
std::uint32_t sessions = 0
```

sessions started (accepted and refused)

*field, declared at [`include/shulib/diag/shul2_reader.hpp:129`](../../include/shulib/diag/shul2_reader.hpp#L129).*

<a id="readercounters-refusedsessions"></a>

### `ReaderCounters::refusedSessions`

```cpp
std::uint32_t refusedSessions = 0
```

sessions negotiate() refused

*field, declared at [`include/shulib/diag/shul2_reader.hpp:130`](../../include/shulib/diag/shul2_reader.hpp#L130).*

<a id="readercounters-latejoins"></a>

### `ReaderCounters::lateJoins`

```cpp
std::uint32_t lateJoins = 0
```

sessions started at a Hello with seq > 0, none open

*field, declared at [`include/shulib/diag/shul2_reader.hpp:131`](../../include/shulib/diag/shul2_reader.hpp#L131).*

<a id="readercounters-framesdelivered"></a>

### `ReaderCounters::framesDelivered`

```cpp
std::uint32_t framesDelivered = 0
```

frames next() handed out

*field, declared at [`include/shulib/diag/shul2_reader.hpp:132`](../../include/shulib/diag/shul2_reader.hpp#L132).*

<a id="readercounters-framesoutsidesession"></a>

### `ReaderCounters::framesOutsideSession`

```cpp
std::uint32_t framesOutsideSession = 0
```

verified frames with no session open

*field, declared at [`include/shulib/diag/shul2_reader.hpp:133`](../../include/shulib/diag/shul2_reader.hpp#L133).*

<a id="readercounters-framesinrefusedsessions"></a>

### `ReaderCounters::framesInRefusedSessions`

```cpp
std::uint32_t framesInRefusedSessions = 0
```

verified frames of refused sessions

*field, declared at [`include/shulib/diag/shul2_reader.hpp:134`](../../include/shulib/diag/shul2_reader.hpp#L134).*

<a id="readercounters-unknowntypeskipped"></a>

### `ReaderCounters::unknownTypeSkipped`

```cpp
std::uint32_t unknownTypeSkipped = 0
```

in-session frames of a type this build does not assign

*field, declared at [`include/shulib/diag/shul2_reader.hpp:135`](../../include/shulib/diag/shul2_reader.hpp#L135).*

<a id="readercounters-wrongwidthskipped"></a>

### `ReaderCounters::wrongWidthSkipped`

```cpp
std::uint32_t wrongWidthSkipped = 0
```

known-type frames whose width disagrees with the Hello

*field, declared at [`include/shulib/diag/shul2_reader.hpp:136`](../../include/shulib/diag/shul2_reader.hpp#L136).*

<a id="readercounters-extensionbytesignored"></a>

### `ReaderCounters::extensionBytesIgnored`

```cpp
std::size_t extensionBytesIgnored = 0
```

bytes beyond this build's known widths, not read

*field, declared at [`include/shulib/diag/shul2_reader.hpp:137`](../../include/shulib/diag/shul2_reader.hpp#L137).*

<a id="readercounters-staleframes"></a>

### `ReaderCounters::staleFrames`

```cpp
std::uint32_t staleFrames = 0
```

stale or duplicate frames (d ≥ 2^31), skipped

*field, declared at [`include/shulib/diag/shul2_reader.hpp:138`](../../include/shulib/diag/shul2_reader.hpp#L138).*

<a id="readercounters-hellorepeats"></a>

### `ReaderCounters::helloRepeats`

```cpp
std::uint32_t helloRepeats = 0
```

re-Hellos identical to their session's Hello

*field, declared at [`include/shulib/diag/shul2_reader.hpp:139`](../../include/shulib/diag/shul2_reader.hpp#L139).*

<a id="readercounters-hellomismatches"></a>

### `ReaderCounters::helloMismatches`

```cpp
std::uint32_t helloMismatches = 0
```

re-Hellos at seq > 0 that differed (new session)

*field, declared at [`include/shulib/diag/shul2_reader.hpp:140`](../../include/shulib/diag/shul2_reader.hpp#L140).*

<a id="readercounters-lostframes"></a>

### `ReaderCounters::lostFrames`

```cpp
std::uint32_t lostFrames = 0
```

sequence gaps in accepted sessions, summed

*field, declared at [`include/shulib/diag/shul2_reader.hpp:141`](../../include/shulib/diag/shul2_reader.hpp#L141).*

<a id="readercounters-linklostframes"></a>

### `ReaderCounters::linkLostFrames`

```cpp
std::uint32_t linkLostFrames = 0
```

link loss, summed over sessions where it is known

*field, declared at [`include/shulib/diag/shul2_reader.hpp:142`](../../include/shulib/diag/shul2_reader.hpp#L142).*

<a id="readercounters-sessionslinklossunknown"></a>

### `ReaderCounters::sessionsLinkLossUnknown`

```cpp
std::uint32_t sessionsLinkLossUnknown = 0
```

accepted sessions whose link loss is unknowable

*field, declared at [`include/shulib/diag/shul2_reader.hpp:143`](../../include/shulib/diag/shul2_reader.hpp#L143).*

<a id="readercounters-enddisagreements"></a>

### `ReaderCounters::endDisagreements`

```cpp
std::uint32_t endDisagreements = 0
```

Ends that disagree with what the reader saw

*field, declared at [`include/shulib/diag/shul2_reader.hpp:144`](../../include/shulib/diag/shul2_reader.hpp#L144).*

<a id="readercounters-sessionsended"></a>

### `ReaderCounters::sessionsEnded`

```cpp
std::uint32_t sessionsEnded = 0
```

accepted sessions whose End was delivered

*field, declared at [`include/shulib/diag/shul2_reader.hpp:145`](../../include/shulib/diag/shul2_reader.hpp#L145).*

<a id="readercounters-truncated"></a>

### `ReaderCounters::truncated`

```cpp
bool truncated = false
```

the capture ends inside an undisplaced candidate

*field, declared at [`include/shulib/diag/shul2_reader.hpp:146`](../../include/shulib/diag/shul2_reader.hpp#L146).*

<a id="readercounters-truncatedat"></a>

### `ReaderCounters::truncatedAt`

```cpp
std::size_t truncatedAt = 0
```

that candidate's offset, when `truncated`

*field, declared at [`include/shulib/diag/shul2_reader.hpp:147`](../../include/shulib/diag/shul2_reader.hpp#L147).*

<a id="struct-sessionreport"></a>

## `struct SessionReport`

```cpp
struct SessionReport
```

What the reader knows about ONE session — Shul2Reader::latestSession() is the latest.

*struct, declared at [`include/shulib/diag/shul2_reader.hpp:151`](../../include/shulib/diag/shul2_reader.hpp#L151).*

<a id="sessionreport-hello"></a>

### `SessionReport::hello`

```cpp
Hello hello{}
```

Its Hello, decoded (views into the capture). For a session of ANOTHER major only the signature and version are decoded (`hello.complete` false): the rest is that major's.

*field, declared at [`include/shulib/diag/shul2_reader.hpp:154`](../../include/shulib/diag/shul2_reader.hpp#L154).*

<a id="sessionreport-verdict"></a>

### `SessionReport::verdict`

```cpp
Verdict verdict = Verdict::BadSignature
```

negotiate()'s verdict on it

*field, declared at [`include/shulib/diag/shul2_reader.hpp:155`](../../include/shulib/diag/shul2_reader.hpp#L155).*

<a id="sessionreport-helloseq"></a>

### `SessionReport::helloSeq`

```cpp
std::uint32_t helloSeq = 0
```

the sequence number of the Hello that started it

*field, declared at [`include/shulib/diag/shul2_reader.hpp:156`](../../include/shulib/diag/shul2_reader.hpp#L156).*

<a id="sessionreport-latejoin"></a>

### `SessionReport::lateJoin`

```cpp
bool lateJoin = false
```

True when the session did not start at its seq-0 Hello — a late join or a mismatched re-Hello — so its link loss cannot be known.

*field, declared at [`include/shulib/diag/shul2_reader.hpp:159`](../../include/shulib/diag/shul2_reader.hpp#L159).*

<a id="sessionreport-lostframes"></a>

### `SessionReport::lostFrames`

```cpp
std::uint32_t lostFrames = 0
```

its sequence gaps (accepted sessions only)

*field, declared at [`include/shulib/diag/shul2_reader.hpp:160`](../../include/shulib/diag/shul2_reader.hpp#L160).*

<a id="sessionreport-sawend"></a>

### `SessionReport::sawEnd`

```cpp
bool sawEnd = false
```

its End was delivered

*field, declared at [`include/shulib/diag/shul2_reader.hpp:161`](../../include/shulib/diag/shul2_reader.hpp#L161).*

<a id="sessionreport-end"></a>

### `SessionReport::end`

```cpp
EndInfo end{}
```

that End, when sawEnd

*field, declared at [`include/shulib/diag/shul2_reader.hpp:162`](../../include/shulib/diag/shul2_reader.hpp#L162).*

<a id="sessionreport-linklossknown"></a>

### `SessionReport::linkLossKnown`

```cpp
bool linkLossKnown = false
```

linkLostFrames is meaningful (seq-0 start, End seen)

*field, declared at [`include/shulib/diag/shul2_reader.hpp:163`](../../include/shulib/diag/shul2_reader.hpp#L163).*

<a id="sessionreport-linklostframes"></a>

### `SessionReport::linkLostFrames`

```cpp
std::uint32_t linkLostFrames = 0
```

max(0, lostFrames − end.framesFailed), when known

*field, declared at [`include/shulib/diag/shul2_reader.hpp:164`](../../include/shulib/diag/shul2_reader.hpp#L164).*

<a id="class-shul2reader"></a>

## `class Shul2Reader`

```cpp
class Shul2Reader
```

The reader of a whole SHUL/2 capture: point it at bytes the caller holds — it BORROWS the span and allocates nothing — and pull frames with next() until it returns false. Frames are delivered only from accepted sessions and only at the widths their Hello declared; every byte and frame that is not delivered is counted by name in counters(). A corrupted frame costs exactly that frame: scanning resumes one byte on, never at a claimed length. At the tail, a candidate that runs past the end is truncation only if no later frame verifies (header). The verdict for the whole capture is status(), final once finished().

*class, declared at [`include/shulib/diag/shul2_reader.hpp:174`](../../include/shulib/diag/shul2_reader.hpp#L174).*

<a id="shul2reader-shul2reader"></a>

### `Shul2Reader::Shul2Reader`

```cpp
explicit Shul2Reader(std::span<const std::byte> capture, const ReaderProfile& profile = kThisReader) noexcept
```

Read `capture` with `profile` (this build's, unless a test stands up another). Both are borrowed: the capture and the profile's rows must outlive the reader — never hand it a temporary buffer (H1's own test suite did once, and read freed memory). Never throws.

*function, declared at [`include/shulib/diag/shul2_reader.hpp:188`](../../include/shulib/diag/shul2_reader.hpp#L188).*

<a id="shul2reader-next"></a>

### `Shul2Reader::next`

```cpp
[[nodiscard]] bool next(Frame& out) noexcept
```

Deliver the next frame, skipping and counting everything that is not delivered. Returns false at the end of the capture, which finishes it (finished() is then true).

*function, declared at [`include/shulib/diag/shul2_reader.hpp:194`](../../include/shulib/diag/shul2_reader.hpp#L194).*

<a id="shul2reader-drain"></a>

### `Shul2Reader::drain`

```cpp
void drain() noexcept
```

Read to the end, discarding the frames — for a caller that wants only the verdict and the counts.

*function, declared at [`include/shulib/diag/shul2_reader.hpp:233`](../../include/shulib/diag/shul2_reader.hpp#L233).*

<a id="shul2reader-finished"></a>

### `Shul2Reader::finished`

```cpp
[[nodiscard]] bool finished() const noexcept
```

True once next() has returned false: status() and counters() are then final.

*function, declared at [`include/shulib/diag/shul2_reader.hpp:240`](../../include/shulib/diag/shul2_reader.hpp#L240).*

<a id="shul2reader-status"></a>

### `Shul2Reader::status`

```cpp
[[nodiscard]] CaptureStatus status() const noexcept
```

The capture's verdict (final once finished()).

*function, declared at [`include/shulib/diag/shul2_reader.hpp:243`](../../include/shulib/diag/shul2_reader.hpp#L243).*

<a id="shul2reader-counters"></a>

### `Shul2Reader::counters`

```cpp
[[nodiscard]] const ReaderCounters& counters() const noexcept
```

The capture-wide counts (final once finished()).

*function, declared at [`include/shulib/diag/shul2_reader.hpp:260`](../../include/shulib/diag/shul2_reader.hpp#L260).*

<a id="shul2reader-hassession"></a>

### `Shul2Reader::hasSession`

```cpp
[[nodiscard]] bool hasSession() const noexcept
```

True once any Hello has started a session.

*function, declared at [`include/shulib/diag/shul2_reader.hpp:262`](../../include/shulib/diag/shul2_reader.hpp#L262).*

<a id="shul2reader-latestsession"></a>

### `Shul2Reader::latestSession`

```cpp
[[nodiscard]] const SessionReport& latestSession() const noexcept
```

The latest session (a default report while hasSession() is false).

*function, declared at [`include/shulib/diag/shul2_reader.hpp:264`](../../include/shulib/diag/shul2_reader.hpp#L264).*

<a id="shul2reader-bytesconsumed"></a>

### `Shul2Reader::bytesConsumed`

```cpp
[[nodiscard]] std::size_t bytesConsumed() const noexcept
```

Bytes of the capture consumed so far.

*function, declared at [`include/shulib/diag/shul2_reader.hpp:266`](../../include/shulib/diag/shul2_reader.hpp#L266).*

<a id="struct-shul2reader-frame"></a>

## `struct Shul2Reader::Frame`

```cpp
struct Frame
```

One delivered frame: its type, envelope flags, sequence number and whole payload (a view valid while the capture is).

*struct, declared at [`include/shulib/diag/shul2_reader.hpp:178`](../../include/shulib/diag/shul2_reader.hpp#L178).*

<a id="shul2reader-frame-type"></a>

### `Shul2Reader::Frame::type`

```cpp
FrameType type = FrameType::Tick
```

one of the five known types

*field, declared at [`include/shulib/diag/shul2_reader.hpp:179`](../../include/shulib/diag/shul2_reader.hpp#L179).*

<a id="shul2reader-frame-flags"></a>

### `Shul2Reader::Frame::flags`

```cpp
std::uint8_t flags = 0
```

envelope flags as they arrived (never judged)

*field, declared at [`include/shulib/diag/shul2_reader.hpp:180`](../../include/shulib/diag/shul2_reader.hpp#L180).*

<a id="shul2reader-frame-seq"></a>

### `Shul2Reader::Frame::seq`

```cpp
std::uint32_t seq = 0
```

its sequence number

*field, declared at [`include/shulib/diag/shul2_reader.hpp:181`](../../include/shulib/diag/shul2_reader.hpp#L181).*

<a id="shul2reader-frame-payload"></a>

### `Shul2Reader::Frame::payload`

```cpp
std::span<const std::byte> payload{}
```

the WHOLE payload, appended bytes included

*field, declared at [`include/shulib/diag/shul2_reader.hpp:182`](../../include/shulib/diag/shul2_reader.hpp#L182).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 86 lines, click to expand</summary>

```text

 Shul2Reader — the SHUL/2 reader over a WHOLE capture (chunk H1 Part A1, 2026-09-29; the
 format is diag/shul2_format.hpp). It ships in the same part as the format for the reason
 BlackboxReader shipped with the blackbox: a format nothing can read is not a record, and the
 reader is what makes the encoders testable at all.

 It borrows the span (a capture the caller already holds — a file, or the bytes a test just
 built), allocates nothing, never throws, and never reads past the end. It is built on
 scanFrame() and negotiate() and on nothing else, so a LIVE reader (H2's socket loop, which
 waits on NeedMore instead) composes the same two functions and meets the same rules. There
 is no socket code here.

 ── Where it differs from BlackboxReader, and why ───────────────────────────────────
 At the TAIL. BlackboxReader stops at a frame whose declared length runs past the end and
 calls the file truncated — right for a file with no check, where the length is all there is
 to believe. Here every frame carries a CRC, so a declared length is only a CLAIM until the
 CRC verifies, and a corrupted length near the end of a capture would otherwise throw away
 every good frame after it. So a whole capture, which never receives more bytes, treats a
 NeedMore at position p as a failed candidate and resumes scanning at p + 1:
   * if a frame verifies later, p was a corrupted length — counted in lengthOverruns;
   * `truncated` is set if and only if the capture ends inside a candidate no later frame
     displaced: the EARLIEST NeedMore after the last verified frame (truncatedAt);
   * a capture that ends exactly on a frame boundary is not truncated.

 ── Delivery ────────────────────────────────────────────────────────────────────────
 next() delivers verified frames {type, flags, seq, payload} of the five known types, ONLY
 from ACCEPTED sessions (Exact, OlderMinor, NewerMinor), and only when the frame's width
 agrees with its session's Hello: a Tick, Summary or End of exactly the declared width; a Log
 whose tag and message fit and whose non-text bytes equal the declared logFixedBytes. A frame
 that disagrees counts in wrongWidthSkipped. Bytes beyond this build's known width are counted
 in extensionBytesIgnored (Hello: payload − the known Hello width; Log: non-text − the known
 14; the others: payload − the known width) — that is how a NEWER minor's appended fields are
 ignored without being lost from the account. The payload delivered is the WHOLE payload; the
 decoders in shul2_format.hpp read the known prefix of it.

 ── Sessions ────────────────────────────────────────────────────────────────────────
   * A Hello with seq 0 starts a new session (a writer that re-opened, or the first).
   * A Hello with seq > 0 while no session is open is a LATE JOIN; the session starts there.
   * A Hello with seq > 0 inside a session must match that session's Hello byte for byte — a
     repeat (helloRepeats), delivered like any frame. Otherwise it is a helloMismatch and
     starts a new session.
   * An End closes its session. A frame outside any open session (before the first Hello, or
     after an End and before the next Hello) counts in framesOutsideSession.
   * A REFUSED session's frames — its End and its repeats included — count in
     framesInRefusedSessions until the next Hello that differs from its own, and are never
     delivered: nothing in a stream this build cannot interpret is interpreted.
   * Any role is accepted and reported; delivery is by type, never by role.

 ── Sequence ────────────────────────────────────────────────────────────────────────
 Every frame of an accepted session (unknown types and repeats included — the writer offered
 them) is checked against expected = the last accepted seq + 1, modulo 2^32:
   d = 0 in order; 0 < d < 2^31 means d frames were lost (lostFrames); d ≥ 2^31 is a stale or
   duplicate frame, skipped (staleFrames). A sequence that wraps 0xFFFFFFFF → 0 is in order
   and starts nothing: only a HELLO at seq 0 starts a session.

 ── Link loss, per session ──────────────────────────────────────────────────────────
 linkLostFrames = max(0, lostFrames − End.framesFailed): the frames the LINK lost, as opposed
 to the ones the writer's device refused (which the writer counted, and which also appear as
 gaps). Known ONLY for a session read from its seq-0 Hello whose End was delivered; every
 other ACCEPTED session counts in sessionsLinkLossUnknown (a late join, a mismatch-started
 session, a session with no End). An End whose framesOffered differs from its own sequence
 number counts in endDisagreements; so does one of a seq-0 session whose framesFailed exceeds
 the frames the session lost — and the figure then saturates at 0 instead of wrapping. (For a
 late-joined session that comparison means nothing: the frames lost before the join were
 never seen.)

 ── What a live reader must hold, and what it must do when its stream ends ──────────
 At least kMaxFrameBytes (14 + 4096) bytes. A corrupted length stalls a live reader until that
 many bytes have arrived — about nine tick frames, ~90 ms at 100 Hz — and then the frames
 inside those bytes are recovered, because scanning resumes one byte on. A whole capture has
 no stall: it resumes at once. When a live stream ENDS — the socket closes, the file ends —
 what the live reader still holds IS a whole capture, and it must apply the tail rule above to
 it: a corrupted length in the last 4 KB would otherwise hold back every frame after it, the
 End included, for good, and that session's link loss would become unknowable. (Applying the
 rule after an idle timeout as well is a live reader's own policy — H2's: it releases a held
 End sooner, at the cost of dropping a frame whose bytes were merely late.)

 ── What reading costs ──────────────────────────────────────────────────────────────
 Linear in the capture for ordinary damage and random bytes (a random sync pair is about one
 position in 65,536). The worst case is bytes CRAFTED to hold as many candidates claiming an
 accepted length as a byte pattern can — about three per ten bytes — each costing up to 4104
 bytes of bitwise CRC before it fails: about 1,200 bytes of CRC per input byte, measured on a
 desktop host at roughly 14-23 s per MB of such input (shul2_format.hpp, the resync rule). There
 is no time budget here — this reader runs on a host; a reader on a UI thread should bound its own.

 Allocation-free and PROS-free, like everything in this tree.
```

</details>
