<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/shul2_format.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `shul2_format.hpp`

SHUL/2 — the telemetry WIRE, format v1.0: DebugRecord streamed, sequenced, checked and versioned.

This header declares **11** types (66 members), **16** free functions, and **30** constants.

Extracted from [`include/shulib/diag/shul2_format.hpp`](../../include/shulib/diag/shul2_format.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kSync0`](#ksync0) — *constant*
- [`kSync1`](#ksync1) — *constant*
- [`kTypeOffset`](#ktypeoffset) — *constant*
- [`kFlagsOffset`](#kflagsoffset) — *constant*
- [`kSequenceOffset`](#ksequenceoffset) — *constant*
- [`kPayloadBytesOffset`](#kpayloadbytesoffset) — *constant*
- [`kPayloadOffset`](#kpayloadoffset) — *constant*
- [`kCrcBytes`](#kcrcbytes) — *constant*
- [`kFrameOverheadBytes`](#kframeoverheadbytes) — *constant*
- [`kMaxPayloadBytes`](#kmaxpayloadbytes) — *constant*
- [`kMaxFrameBytes`](#kmaxframebytes) — *constant*
- [`kVersionMajor`](#kversionmajor) — *constant*
- [`kVersionMinor`](#kversionminor) — *constant*
- [`enum class FrameType`](#enum-class-frametype)
  - [`Hello`](#frametype-hello)
  - [`Tick`](#frametype-tick)
  - [`Log`](#frametype-log)
  - [`Summary`](#frametype-summary)
  - [`End`](#frametype-end)
- [`kFirstRuntimeDataType`](#kfirstruntimedatatype) — *constant*
- [`kLastRuntimeDataType`](#klastruntimedatatype) — *constant*
- [`kFirstPeerDataType`](#kfirstpeerdatatype) — *constant*
- [`kLastPeerDataType`](#klastpeerdatatype) — *constant*
- [`kRoleRuntime`](#kroleruntime) — *constant*
- [`kRolePeer`](#krolepeer) — *constant*
- [`kSignature`](#ksignature) — *constant*
- [`kHelloInvariantBytes`](#khelloinvariantbytes) — *constant*
- [`kHelloPayloadBytes`](#khellopayloadbytes) — *constant*
- [`kLogFixedBytes`](#klogfixedbytes) — *constant*
- [`kEndPayloadBytes`](#kendpayloadbytes) — *constant*
- [`kLogTagTruncated`](#klogtagtruncated) — *constant*
- [`kLogMessageTruncated`](#klogmessagetruncated) — *constant*
- [`struct WidthRow`](#struct-widthrow)
  - [`hello`](#widthrow-hello)
  - [`tick`](#widthrow-tick)
  - [`summary`](#widthrow-summary)
  - [`end`](#widthrow-end)
  - [`logFixed`](#widthrow-logfixed)
- [`kWidthTable`](#kwidthtable) — *constant*
- [`kApiMajorByte`](#kapimajorbyte) — *constant*
- [`kApiMinorByte`](#kapiminorbyte) — *constant*
- [`struct ReaderProfile`](#struct-readerprofile)
  - [`versionMajor`](#readerprofile-versionmajor)
  - [`rows`](#readerprofile-rows)
- [`kThisReader`](#kthisreader) — *constant*
- [`enum class Verdict`](#enum-class-verdict)
  - [`Exact`](#verdict-exact)
  - [`OlderMinor`](#verdict-olderminor)
  - [`NewerMinor`](#verdict-newerminor)
  - [`BadSignature`](#verdict-badsignature)
  - [`RefusedMajor`](#verdict-refusedmajor)
  - [`LayoutMismatch`](#verdict-layoutmismatch)
- [`accepted`](#accepted) — *free function*
- [`verdictName`](#verdictname) — *free function*
- [`struct Hello`](#struct-hello)
  - [`payloadBytes`](#hello-payloadbytes)
  - [`signature`](#hello-signature)
  - [`versionMajor`](#hello-versionmajor)
  - [`versionMinor`](#hello-versionminor)
  - [`complete`](#hello-complete)
  - [`role`](#hello-role)
  - [`flags`](#hello-flags)
  - [`declared`](#hello-declared)
  - [`epochSeconds`](#hello-epochseconds)
  - [`apiMajor`](#hello-apimajor)
  - [`apiMinor`](#hello-apiminor)
  - [`buildHash`](#hello-buildhash)
  - [`routineId`](#hello-routineid)
  - [`alliance`](#hello-alliance)
  - [`side`](#hello-side)
  - [`portMap`](#hello-portmap)
- [`negotiate`](#negotiate) — *free function*
- [`enum class ScanKind`](#enum-class-scankind)
  - [`NeedMore`](#scankind-needmore)
  - [`Frame`](#scankind-frame)
  - [`Skip`](#scankind-skip)
- [`enum class SkipReason`](#enum-class-skipreason)
  - [`None`](#skipreason-none)
  - [`NoSync`](#skipreason-nosync)
  - [`LengthTooLarge`](#skipreason-lengthtoolarge)
  - [`CrcMismatch`](#skipreason-crcmismatch)
- [`struct ScanResult`](#struct-scanresult)
  - [`kind`](#scanresult-kind)
  - [`consumed`](#scanresult-consumed)
  - [`reason`](#scanresult-reason)
  - [`type`](#scanresult-type)
  - [`flags`](#scanresult-flags)
  - [`seq`](#scanresult-seq)
  - [`payload`](#scanresult-payload)
- [`scanFrame`](#scanframe) — *free function*
- [`encodeFrame`](#encodeframe) — *free function*
- [`encodeHello`](#encodehello) — *free function*
- [`struct LogLine`](#struct-logline)
  - [`level`](#logline-level)
  - [`flags`](#logline-flags)
  - [`t`](#logline-t)
  - [`tag`](#logline-tag)
  - [`message`](#logline-message)
  - [`nonTextBytes`](#logline-nontextbytes)
- [`encodeLog`](#encodelog) — *free function*
- [`struct EndInfo`](#struct-endinfo)
  - [`framesOffered`](#endinfo-framesoffered)
  - [`framesFailed`](#endinfo-framesfailed)
  - [`recordsSeen`](#endinfo-recordsseen)
  - [`linesSeen`](#endinfo-linesseen)
  - [`linesTruncated`](#endinfo-linestruncated)
  - [`notOpenDropped`](#endinfo-notopendropped)
  - [`endTime`](#endinfo-endtime)
- [`encodeEnd`](#encodeend) — *free function*
- [`decodeHello`](#decodehello) — *free function*
- [`decodeLog`](#decodelog) — *free function*
- [`decodeEnd`](#decodeend) — *free function*
- [`decodeTick`](#decodetick) — *free function*
- [`decodeSummary`](#decodesummary) — *free function*
- [`enum class WireLevel`](#enum-class-wirelevel)
  - [`Error`](#wirelevel-error)
  - [`Warn`](#wirelevel-warn)
  - [`Info`](#wirelevel-info)
  - [`Debug`](#wirelevel-debug)
  - [`Trace`](#wirelevel-trace)
- [`wireLevel`](#wirelevel) — *free function*
- [`logLevelFromWire`](#loglevelfromwire) — *free function*
- [`utf8Prefix`](#utf8prefix) — *free function*

<a id="ksync0"></a>

## `kSync0`

```cpp
inline constexpr std::uint8_t kSync0 = 0xFE
```

First sync byte of every frame. 0xFE never occurs in valid UTF-8, so text a writer carries cannot open a false frame inside a Log's own bytes as easily as ASCII would.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:194`](../../include/shulib/diag/shul2_format.hpp#L194).*

<a id="ksync1"></a>

## `kSync1`

```cpp
inline constexpr std::uint8_t kSync1 = 0x53
```

Second sync byte of every frame ('S').

*constant, declared at [`include/shulib/diag/shul2_format.hpp:196`](../../include/shulib/diag/shul2_format.hpp#L196).*

<a id="ktypeoffset"></a>

## `kTypeOffset`

```cpp
inline constexpr std::size_t kTypeOffset = 2
```

Offset of the type byte in a frame.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:198`](../../include/shulib/diag/shul2_format.hpp#L198).*

<a id="kflagsoffset"></a>

## `kFlagsOffset`

```cpp
inline constexpr std::size_t kFlagsOffset = 3
```

Offset of the flags byte in a frame.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:200`](../../include/shulib/diag/shul2_format.hpp#L200).*

<a id="ksequenceoffset"></a>

## `kSequenceOffset`

```cpp
inline constexpr std::size_t kSequenceOffset = 4
```

Offset of the u32 sequence number in a frame.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:202`](../../include/shulib/diag/shul2_format.hpp#L202).*

<a id="kpayloadbytesoffset"></a>

## `kPayloadBytesOffset`

```cpp
inline constexpr std::size_t kPayloadBytesOffset = 8
```

Offset of the u16 payloadBytes field in a frame.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:204`](../../include/shulib/diag/shul2_format.hpp#L204).*

<a id="kpayloadoffset"></a>

## `kPayloadOffset`

```cpp
inline constexpr std::size_t kPayloadOffset = 10
```

Offset of the payload in a frame — equally, the envelope's bytes BEFORE the payload.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:206`](../../include/shulib/diag/shul2_format.hpp#L206).*

<a id="kcrcbytes"></a>

## `kCrcBytes`

```cpp
inline constexpr std::size_t kCrcBytes = 4
```

Width of the trailing CRC-32.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:208`](../../include/shulib/diag/shul2_format.hpp#L208).*

<a id="kframeoverheadbytes"></a>

## `kFrameOverheadBytes`

```cpp
inline constexpr std::size_t kFrameOverheadBytes = kPayloadOffset + kCrcBytes
```

Envelope overhead per frame: the 10 bytes before the payload plus the 4-byte CRC.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:210`](../../include/shulib/diag/shul2_format.hpp#L210).*

<a id="kmaxpayloadbytes"></a>

## `kMaxPayloadBytes`

```cpp
inline constexpr std::size_t kMaxPayloadBytes = 4096
```

The largest payload any frame may declare. A declared length above it is not a frame (scanFrame resyncs one byte on), which also bounds a live reader's buffer.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:213`](../../include/shulib/diag/shul2_format.hpp#L213).*

<a id="kmaxframebytes"></a>

## `kMaxFrameBytes`

```cpp
inline constexpr std::size_t kMaxFrameBytes = kFrameOverheadBytes + kMaxPayloadBytes
```

The largest whole frame, envelope included: what a live reader must be able to buffer.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:215`](../../include/shulib/diag/shul2_format.hpp#L215).*

<a id="kversionmajor"></a>

## `kVersionMajor`

```cpp
inline constexpr std::uint16_t kVersionMajor = 1
```

The wire's MAJOR version. A reader refuses any other major whole (header, "Versions"). Independent of shulib::kApiMajor: the wire carries its own version in every Hello.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:221`](../../include/shulib/diag/shul2_format.hpp#L221).*

<a id="kversionminor"></a>

## `kVersionMinor`

```cpp
inline constexpr std::uint16_t kVersionMinor = 0
```

The wire's MINOR version: the newest row of kWidthTable. A minor grows frames only by appending, so every reader of this major can still read it.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:224`](../../include/shulib/diag/shul2_format.hpp#L224).*

<a id="enum-class-frametype"></a>

## `enum class FrameType`

```cpp
enum class FrameType : std::uint8_t
```

What a frame carries. WIRE values: explicit, never renumbered. Hello and End are session frames (either direction); the rest are data frames in the runtime → peer range.

*enum class, declared at [`include/shulib/diag/shul2_format.hpp:228`](../../include/shulib/diag/shul2_format.hpp#L228).*

<a id="frametype-hello"></a>

### `FrameType::Hello`

```cpp
Hello = 1
```

opens a session: signature, version, widths, provenance (256 bytes in 1.0)

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:229`](../../include/shulib/diag/shul2_format.hpp#L229).*

<a id="frametype-tick"></a>

### `FrameType::Tick`

```cpp
Tick = 2
```

one DebugRecord: blackbox::encodeTick's 428 bytes

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:230`](../../include/shulib/diag/shul2_format.hpp#L230).*

<a id="frametype-log"></a>

### `FrameType::Log`

```cpp
Log = 3
```

one log() line: 14 fixed bytes, then the tag and the message

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:231`](../../include/shulib/diag/shul2_format.hpp#L231).*

<a id="frametype-summary"></a>

### `FrameType::Summary`

```cpp
Summary = 4
```

one RunSummary: blackbox::encodeSummary's 168 bytes

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:232`](../../include/shulib/diag/shul2_format.hpp#L232).*

<a id="frametype-end"></a>

### `FrameType::End`

```cpp
End = 5
```

closes a session with the writer's own account (32 bytes in 1.0)

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:233`](../../include/shulib/diag/shul2_format.hpp#L233).*

<a id="kfirstruntimedatatype"></a>

## `kFirstRuntimeDataType`

```cpp
inline constexpr std::uint8_t kFirstRuntimeDataType = 2
```

First DATA frame type a runtime sends to a peer (types 2-63).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:237`](../../include/shulib/diag/shul2_format.hpp#L237).*

<a id="klastruntimedatatype"></a>

## `kLastRuntimeDataType`

```cpp
inline constexpr std::uint8_t kLastRuntimeDataType = 63
```

Last DATA frame type a runtime sends to a peer.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:239`](../../include/shulib/diag/shul2_format.hpp#L239).*

<a id="kfirstpeerdatatype"></a>

## `kFirstPeerDataType`

```cpp
inline constexpr std::uint8_t kFirstPeerDataType = 64
```

First DATA frame type a peer sends to a runtime (64-127: H2's simulated sensor frames).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:241`](../../include/shulib/diag/shul2_format.hpp#L241).*

<a id="klastpeerdatatype"></a>

## `kLastPeerDataType`

```cpp
inline constexpr std::uint8_t kLastPeerDataType = 127
```

Last DATA frame type a peer sends to a runtime; 128-255 are reserved.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:243`](../../include/shulib/diag/shul2_format.hpp#L243).*

<a id="kroleruntime"></a>

## `kRoleRuntime`

```cpp
inline constexpr std::uint8_t kRoleRuntime = 0
```

The Hello's `role` for the runtime that emits records (the robot, or a program on the host).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:246`](../../include/shulib/diag/shul2_format.hpp#L246).*

<a id="krolepeer"></a>

## `kRolePeer`

```cpp
inline constexpr std::uint8_t kRolePeer = 1
```

The Hello's `role` for a peer (H2's VexBuilder side). Other values are reserved; a reader accepts and reports any role, and delivers frames by type, never by role.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:249`](../../include/shulib/diag/shul2_format.hpp#L249).*

<a id="ksignature"></a>

## `kSignature`

```cpp
inline constexpr std::uint8_t kSignature[8] = {0x89, 0x53, 0x48, 0x55, 0x4C, 0x00, 0x0D, 0x0A}
```

The Hello's first 8 payload bytes: 0x89 "SHUL" NUL CR LF. The high-bit byte, the NUL and the CR LF make a transport that strips bit 7, truncates at a NUL or translates newlines corrupt every Hello (header, "What the signature does").

*constant, declared at [`include/shulib/diag/shul2_format.hpp:254`](../../include/shulib/diag/shul2_format.hpp#L254).*

<a id="khelloinvariantbytes"></a>

## `kHelloInvariantBytes`

```cpp
inline constexpr std::size_t kHelloInvariantBytes = 12
```

The Hello's invariant prefix: the signature and the two version fields, the same in every major. decodeHello() reads these from any Hello of at least this many bytes.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:258`](../../include/shulib/diag/shul2_format.hpp#L258).*

<a id="khellopayloadbytes"></a>

## `kHelloPayloadBytes`

```cpp
inline constexpr std::size_t kHelloPayloadBytes = 256
```

Payload width of a v1.0 Hello.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:260`](../../include/shulib/diag/shul2_format.hpp#L260).*

<a id="klogfixedbytes"></a>

## `kLogFixedBytes`

```cpp
inline constexpr std::size_t kLogFixedBytes = 14
```

Bytes of a v1.0 Log that are not text (the fixed part before the tag).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:262`](../../include/shulib/diag/shul2_format.hpp#L262).*

<a id="kendpayloadbytes"></a>

## `kEndPayloadBytes`

```cpp
inline constexpr std::size_t kEndPayloadBytes = 32
```

Payload width of a v1.0 End.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:264`](../../include/shulib/diag/shul2_format.hpp#L264).*

<a id="klogtagtruncated"></a>

## `kLogTagTruncated`

```cpp
inline constexpr std::uint8_t kLogTagTruncated = 0x01
```

Log flags bit 0: the writer cut the tag to its cap.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:267`](../../include/shulib/diag/shul2_format.hpp#L267).*

<a id="klogmessagetruncated"></a>

## `kLogMessageTruncated`

```cpp
inline constexpr std::uint8_t kLogMessageTruncated = 0x02
```

Log flags bit 1: the writer cut the message to its cap.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:269`](../../include/shulib/diag/shul2_format.hpp#L269).*

<a id="struct-widthrow"></a>

## `struct WidthRow`

```cpp
struct WidthRow
```

One minor version's payload widths, in bytes: a row of kWidthTable, and what a Hello declares about the frames that follow it.

*struct, declared at [`include/shulib/diag/shul2_format.hpp:273`](../../include/shulib/diag/shul2_format.hpp#L273).*

<a id="widthrow-hello"></a>

### `WidthRow::hello`

```cpp
std::uint16_t hello = 0
```

the Hello's own payload width

*field, declared at [`include/shulib/diag/shul2_format.hpp:274`](../../include/shulib/diag/shul2_format.hpp#L274).*

<a id="widthrow-tick"></a>

### `WidthRow::tick`

```cpp
std::uint16_t tick = 0
```

every Tick payload

*field, declared at [`include/shulib/diag/shul2_format.hpp:275`](../../include/shulib/diag/shul2_format.hpp#L275).*

<a id="widthrow-summary"></a>

### `WidthRow::summary`

```cpp
std::uint16_t summary = 0
```

every Summary payload

*field, declared at [`include/shulib/diag/shul2_format.hpp:276`](../../include/shulib/diag/shul2_format.hpp#L276).*

<a id="widthrow-end"></a>

### `WidthRow::end`

```cpp
std::uint16_t end = 0
```

the End payload

*field, declared at [`include/shulib/diag/shul2_format.hpp:277`](../../include/shulib/diag/shul2_format.hpp#L277).*

<a id="widthrow-logfixed"></a>

### `WidthRow::logFixed`

```cpp
std::uint16_t logFixed = 0
```

every Log byte that is not text

*field, declared at [`include/shulib/diag/shul2_format.hpp:278`](../../include/shulib/diag/shul2_format.hpp#L278).*

<a id="kwidthtable"></a>

## `kWidthTable`

```cpp
inline constexpr WidthRow kWidthTable[] = {{256, 428, 168, 32, 14}}
```

The width table: row i describes minor i, append-only. Row 1.0 is {256, 428, 168, 32, 14}.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:282`](../../include/shulib/diag/shul2_format.hpp#L282).*

<a id="kapimajorbyte"></a>

## `kApiMajorByte`

```cpp
inline constexpr std::uint8_t kApiMajorByte = static_cast<std::uint8_t>(shulib::kApiMajor)
```

shulib::kApiMajor as the Hello's one byte carries it (range-checked at compile time).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:301`](../../include/shulib/diag/shul2_format.hpp#L301).*

<a id="kapiminorbyte"></a>

## `kApiMinorByte`

```cpp
inline constexpr std::uint8_t kApiMinorByte = static_cast<std::uint8_t>(shulib::kApiMinor)
```

shulib::kApiMinor as the Hello's one byte carries it (range-checked at compile time).

*constant, declared at [`include/shulib/diag/shul2_format.hpp:303`](../../include/shulib/diag/shul2_format.hpp#L303).*

<a id="struct-readerprofile"></a>

## `struct ReaderProfile`

```cpp
struct ReaderProfile
```

What a READER knows: the major it reads and one width row per minor it knows. A parameter rather than a constant so a test can stand up a hypothetical newer reader with its own row; kThisReader is this build's. The rows are borrowed and must outlive every use. Its major is kVersionMajor in practice: decodeHello() reads a Hello's bytes 12 onward only for this build's major (they belong to their major), so a profile of another major can name a stream but never accept one.

*struct, declared at [`include/shulib/diag/shul2_format.hpp:313`](../../include/shulib/diag/shul2_format.hpp#L313).*

<a id="readerprofile-versionmajor"></a>

### `ReaderProfile::versionMajor`

```cpp
std::uint16_t versionMajor = kVersionMajor
```

the only major this reader accepts

*field, declared at [`include/shulib/diag/shul2_format.hpp:314`](../../include/shulib/diag/shul2_format.hpp#L314).*

<a id="readerprofile-rows"></a>

### `ReaderProfile::rows`

```cpp
std::span<const WidthRow> rows{}
```

row i = minor i; the last row is the newest

*field, declared at [`include/shulib/diag/shul2_format.hpp:315`](../../include/shulib/diag/shul2_format.hpp#L315).*

<a id="kthisreader"></a>

## `kThisReader`

```cpp
inline constexpr ReaderProfile kThisReader{kVersionMajor, std::span<const WidthRow>{kWidthTable}}
```

This build's reader: major kVersionMajor, the rows of kWidthTable.

*constant, declared at [`include/shulib/diag/shul2_format.hpp:319`](../../include/shulib/diag/shul2_format.hpp#L319).*

<a id="enum-class-verdict"></a>

## `enum class Verdict`

```cpp
enum class Verdict : std::uint8_t
```

negotiate()'s verdict on one Hello: exactly one, checked in the order listed below.

*enum class, declared at [`include/shulib/diag/shul2_format.hpp:322`](../../include/shulib/diag/shul2_format.hpp#L322).*

<a id="verdict-exact"></a>

### `Verdict::Exact`

```cpp
Exact = 0
```

same major and minor, every width equal to the reader's row

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:323`](../../include/shulib/diag/shul2_format.hpp#L323).*

<a id="verdict-olderminor"></a>

### `Verdict::OlderMinor`

```cpp
OlderMinor = 1
```

an older minor whose widths equal THAT minor's row

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:324`](../../include/shulib/diag/shul2_format.hpp#L324).*

<a id="verdict-newerminor"></a>

### `Verdict::NewerMinor`

```cpp
NewerMinor = 2
```

a newer minor; every width at least the reader's newest row

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:325`](../../include/shulib/diag/shul2_format.hpp#L325).*

<a id="verdict-badsignature"></a>

### `Verdict::BadSignature`

```cpp
BadSignature = 3
```

fewer than 12 Hello bytes, or a signature that differs

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:326`](../../include/shulib/diag/shul2_format.hpp#L326).*

<a id="verdict-refusedmajor"></a>

### `Verdict::RefusedMajor`

```cpp
RefusedMajor = 4
```

a major other than the reader's — refused whole

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:327`](../../include/shulib/diag/shul2_format.hpp#L327).*

<a id="verdict-layoutmismatch"></a>

### `Verdict::LayoutMismatch`

```cpp
LayoutMismatch = 5
```

a short Hello, or a width that breaks the rules above

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:328`](../../include/shulib/diag/shul2_format.hpp#L328).*

<a id="accepted"></a>

## `accepted`

```cpp
[[nodiscard]] constexpr bool accepted(Verdict v) noexcept
```

True for the three verdicts whose sessions a reader delivers (Exact, OlderMinor, NewerMinor); false for the three that refuse the session.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:333`](../../include/shulib/diag/shul2_format.hpp#L333).*

<a id="verdictname"></a>

## `verdictName`

```cpp
[[nodiscard]] constexpr const char* verdictName(Verdict v) noexcept
```

The §18.5 spelling of a Verdict, for messages. Never returns null.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:338`](../../include/shulib/diag/shul2_format.hpp#L338).*

<a id="struct-hello"></a>

## `struct Hello`

```cpp
struct Hello
```

A decoded Hello. Its text fields are VIEWS into the caller's payload bytes, valid while those bytes are; each ends at its field's first NUL (or fills the field). Everything past byte 11 is meaningful only when `complete` is true — and for another major it never is: those bytes belong to that major, so they are left at their defaults, never read with this major's layout.

*struct, declared at [`include/shulib/diag/shul2_format.hpp:355`](../../include/shulib/diag/shul2_format.hpp#L355).*

<a id="hello-payloadbytes"></a>

### `Hello::payloadBytes`

```cpp
std::size_t payloadBytes = 0
```

the Hello's payload length as it arrived

*field, declared at [`include/shulib/diag/shul2_format.hpp:356`](../../include/shulib/diag/shul2_format.hpp#L356).*

<a id="hello-signature"></a>

### `Hello::signature`

```cpp
std::array<std::uint8_t, 8> signature{}
```

payload bytes 0-7 as they arrived

*field, declared at [`include/shulib/diag/shul2_format.hpp:357`](../../include/shulib/diag/shul2_format.hpp#L357).*

<a id="hello-versionmajor"></a>

### `Hello::versionMajor`

```cpp
std::uint16_t versionMajor = 0
```

payload bytes 8-9

*field, declared at [`include/shulib/diag/shul2_format.hpp:358`](../../include/shulib/diag/shul2_format.hpp#L358).*

<a id="hello-versionminor"></a>

### `Hello::versionMinor`

```cpp
std::uint16_t versionMinor = 0
```

payload bytes 10-11

*field, declared at [`include/shulib/diag/shul2_format.hpp:359`](../../include/shulib/diag/shul2_format.hpp#L359).*

<a id="hello-complete"></a>

### `Hello::complete`

```cpp
bool complete = false
```

The payload held at least the 1.0 Hello's 256 bytes AND its major is kVersionMajor, so the fields below were decoded.

*field, declared at [`include/shulib/diag/shul2_format.hpp:362`](../../include/shulib/diag/shul2_format.hpp#L362).*

<a id="hello-role"></a>

### `Hello::role`

```cpp
std::uint8_t role = 0
```

kRoleRuntime, kRolePeer or a reserved value

*field, declared at [`include/shulib/diag/shul2_format.hpp:363`](../../include/shulib/diag/shul2_format.hpp#L363).*

<a id="hello-flags"></a>

### `Hello::flags`

```cpp
std::uint16_t flags = 0
```

reserved; 0 in 1.0 and never judged

*field, declared at [`include/shulib/diag/shul2_format.hpp:364`](../../include/shulib/diag/shul2_format.hpp#L364).*

<a id="hello-declared"></a>

### `Hello::declared`

```cpp
WidthRow declared{}
```

the widths it declares (`hello` = payloadBytes, capped)

*field, declared at [`include/shulib/diag/shul2_format.hpp:365`](../../include/shulib/diag/shul2_format.hpp#L365).*

<a id="hello-epochseconds"></a>

### `Hello::epochSeconds`

```cpp
double epochSeconds = 0.0
```

the writer's clock at open(), raw seconds

*field, declared at [`include/shulib/diag/shul2_format.hpp:366`](../../include/shulib/diag/shul2_format.hpp#L366).*

<a id="hello-apimajor"></a>

### `Hello::apiMajor`

```cpp
std::uint8_t apiMajor = 0
```

the writer's shulib::kApiMajor

*field, declared at [`include/shulib/diag/shul2_format.hpp:367`](../../include/shulib/diag/shul2_format.hpp#L367).*

<a id="hello-apiminor"></a>

### `Hello::apiMinor`

```cpp
std::uint8_t apiMinor = 0
```

the writer's shulib::kApiMinor

*field, declared at [`include/shulib/diag/shul2_format.hpp:368`](../../include/shulib/diag/shul2_format.hpp#L368).*

<a id="hello-buildhash"></a>

### `Hello::buildHash`

```cpp
std::string_view buildHash{}
```

EMPTY means MISSING — render it loudly

*field, declared at [`include/shulib/diag/shul2_format.hpp:369`](../../include/shulib/diag/shul2_format.hpp#L369).*

<a id="hello-routineid"></a>

### `Hello::routineId`

```cpp
std::string_view routineId{}
```

may be empty

*field, declared at [`include/shulib/diag/shul2_format.hpp:370`](../../include/shulib/diag/shul2_format.hpp#L370).*

<a id="hello-alliance"></a>

### `Hello::alliance`

```cpp
std::string_view alliance{}
```

free text; may be empty

*field, declared at [`include/shulib/diag/shul2_format.hpp:371`](../../include/shulib/diag/shul2_format.hpp#L371).*

<a id="hello-side"></a>

### `Hello::side`

```cpp
std::string_view side{}
```

free text; may be empty

*field, declared at [`include/shulib/diag/shul2_format.hpp:372`](../../include/shulib/diag/shul2_format.hpp#L372).*

<a id="hello-portmap"></a>

### `Hello::portMap`

```cpp
std::string_view portMap{}
```

caller-authored; may be empty

*field, declared at [`include/shulib/diag/shul2_format.hpp:373`](../../include/shulib/diag/shul2_format.hpp#L373).*

<a id="negotiate"></a>

## `negotiate`

```cpp
[[nodiscard]] inline Verdict negotiate(const Hello& stream, const ReaderProfile& reader) noexcept
```

Judge a stream's Hello against a reader's profile. Pure. Exactly one verdict, checked in this order: BadSignature (fewer than 12 bytes, or a signature that differs); RefusedMajor; LayoutMismatch (a Hello shorter than 256 bytes; at the SAME minor any declared width that differs from the reader's row; at an OLDER minor any that differs from THAT minor's row; at a NEWER minor any shorter than the reader's newest row; or a profile with no rows); then Exact, OlderMinor or NewerMinor. Reserved bytes, flags and role are never judged.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:382`](../../include/shulib/diag/shul2_format.hpp#L382).*

<a id="enum-class-scankind"></a>

## `enum class ScanKind`

```cpp
enum class ScanKind : std::uint8_t
```

What scanFrame() found at the start of the bytes it was given.

*enum class, declared at [`include/shulib/diag/shul2_format.hpp:420`](../../include/shulib/diag/shul2_format.hpp#L420).*

<a id="scankind-needmore"></a>

### `ScanKind::NeedMore`

```cpp
NeedMore = 0
```

a proper prefix of a frame that could still be valid (consumed 0)

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:421`](../../include/shulib/diag/shul2_format.hpp#L421).*

<a id="scankind-frame"></a>

### `ScanKind::Frame`

```cpp
Frame = 1
```

a whole frame whose sync matched and whose CRC verified

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:422`](../../include/shulib/diag/shul2_format.hpp#L422).*

<a id="scankind-skip"></a>

### `ScanKind::Skip`

```cpp
Skip = 2
```

not a frame here: drop `consumed` bytes and scan again

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:423`](../../include/shulib/diag/shul2_format.hpp#L423).*

<a id="enum-class-skipreason"></a>

## `enum class SkipReason`

```cpp
enum class SkipReason : std::uint8_t
```

Why scanFrame() skipped.

*enum class, declared at [`include/shulib/diag/shul2_format.hpp:427`](../../include/shulib/diag/shul2_format.hpp#L427).*

<a id="skipreason-none"></a>

### `SkipReason::None`

```cpp
None = 0
```

not a skip

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:428`](../../include/shulib/diag/shul2_format.hpp#L428).*

<a id="skipreason-nosync"></a>

### `SkipReason::NoSync`

```cpp
NoSync = 1
```

no sync pair at offset 0: skip up to the next 0xFE (or everything)

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:429`](../../include/shulib/diag/shul2_format.hpp#L429).*

<a id="skipreason-lengthtoolarge"></a>

### `SkipReason::LengthTooLarge`

```cpp
LengthTooLarge = 2
```

a declared length above kMaxPayloadBytes: skip 1 byte

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:430`](../../include/shulib/diag/shul2_format.hpp#L430).*

<a id="skipreason-crcmismatch"></a>

### `SkipReason::CrcMismatch`

```cpp
CrcMismatch = 3
```

a whole candidate whose CRC did not verify: skip 1 byte

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:431`](../../include/shulib/diag/shul2_format.hpp#L431).*

<a id="struct-scanresult"></a>

## `struct ScanResult`

```cpp
struct ScanResult
```

scanFrame()'s result. For a Frame, `payload` views the caller's bytes.

*struct, declared at [`include/shulib/diag/shul2_format.hpp:435`](../../include/shulib/diag/shul2_format.hpp#L435).*

<a id="scanresult-kind"></a>

### `ScanResult::kind`

```cpp
ScanKind kind = ScanKind::NeedMore
```

what was found

*field, declared at [`include/shulib/diag/shul2_format.hpp:436`](../../include/shulib/diag/shul2_format.hpp#L436).*

<a id="scanresult-consumed"></a>

### `ScanResult::consumed`

```cpp
std::size_t consumed = 0
```

bytes to drop before scanning again

*field, declared at [`include/shulib/diag/shul2_format.hpp:437`](../../include/shulib/diag/shul2_format.hpp#L437).*

<a id="scanresult-reason"></a>

### `ScanResult::reason`

```cpp
SkipReason reason = SkipReason::None
```

why, for a Skip

*field, declared at [`include/shulib/diag/shul2_format.hpp:438`](../../include/shulib/diag/shul2_format.hpp#L438).*

<a id="scanresult-type"></a>

### `ScanResult::type`

```cpp
std::uint8_t type = 0
```

a Frame's raw type byte

*field, declared at [`include/shulib/diag/shul2_format.hpp:439`](../../include/shulib/diag/shul2_format.hpp#L439).*

<a id="scanresult-flags"></a>

### `ScanResult::flags`

```cpp
std::uint8_t flags = 0
```

a Frame's envelope flags (never judged)

*field, declared at [`include/shulib/diag/shul2_format.hpp:440`](../../include/shulib/diag/shul2_format.hpp#L440).*

<a id="scanresult-seq"></a>

### `ScanResult::seq`

```cpp
std::uint32_t seq = 0
```

a Frame's sequence number

*field, declared at [`include/shulib/diag/shul2_format.hpp:441`](../../include/shulib/diag/shul2_format.hpp#L441).*

<a id="scanresult-payload"></a>

### `ScanResult::payload`

```cpp
std::span<const std::byte> payload{}
```

a Frame's payload

*field, declared at [`include/shulib/diag/shul2_format.hpp:442`](../../include/shulib/diag/shul2_format.hpp#L442).*

<a id="scanframe"></a>

## `scanFrame`

```cpp
[[nodiscard]] inline ScanResult scanFrame(std::span<const std::byte> buffered) noexcept
```

Look for ONE frame at the start of `buffered`. Pure; never reads past the span. NeedMore (consumed 0) when the bytes are a proper prefix of a frame that could still be valid — the empty span, a lone 0xFE, or a valid header whose frame is longer than the bytes held. Frame when the sync matched, the length is at most 4096 and the CRC verified (consumed 14 + n). Skip otherwise. After a failed candidate, scanning ALWAYS resumes at the next byte — never at the claimed length (header, the resync rule). A live reader appends bytes and calls again on NeedMore; a reader of a whole capture applies the tail rule instead (shul2_reader.hpp) — and so does a live reader once its stream ENDS (the socket closes, the file ends): what it still holds is then a whole capture, and waiting on its NeedMore would hold back every frame after a corrupted length, the End included, for good.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:455`](../../include/shulib/diag/shul2_format.hpp#L455).*

<a id="encodeframe"></a>

## `encodeFrame`

```cpp
[[nodiscard]] inline std::size_t encodeFrame(std::span<std::byte> out, std::uint8_t type, std::uint32_t seq, std::span<const std::byte> payload, std::uint8_t flags = 0U) noexcept
```

Wrap `payload` in the envelope: sync, type, flags, sequence, length, payload, CRC. Returns the frame's bytes (14 + payload), or 0 — writing nothing — if the payload is longer than kMaxPayloadBytes or `out` is too small. `type` is the raw byte, so a caller passes `static_cast<std::uint8_t>(FrameType::Tick)` (explicit values: the cast is the value). The payload may already sit at `out[10..]` (a writer that encodes in place); it is then not copied. Anywhere else it is moved in first, overlap allowed.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:514`](../../include/shulib/diag/shul2_format.hpp#L514).*

<a id="encodehello"></a>

## `encodeHello`

```cpp
[[nodiscard]] inline std::size_t encodeHello(std::span<std::byte> out, const SessionInfo& info, double epochSeconds, std::uint8_t role = kRoleRuntime) noexcept
```

Encode THIS build's Hello (v1.0, 256 bytes): the signature, kVersionMajor/kVersionMinor, `role`, the 1.0 widths, `epochSeconds`, the API version bytes and `info`'s provenance, each text truncated to its field and NUL-padded. `info`'s views are read during this call only. Returns 256, or 0 if `out` is too small. An EMPTY build hash stays empty.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:545`](../../include/shulib/diag/shul2_format.hpp#L545).*

<a id="struct-logline"></a>

## `struct LogLine`

```cpp
struct LogLine
```

One Log line: the fields of a Log payload. For decodeLog() the text fields are VIEWS into the caller's payload bytes and are never NUL-terminated.

*struct, declared at [`include/shulib/diag/shul2_format.hpp:580`](../../include/shulib/diag/shul2_format.hpp#L580).*

<a id="logline-level"></a>

### `LogLine::level`

```cpp
std::uint8_t level = 2
```

the WIRE level code (Error 0 … Trace 4); raw, may be unknown

*field, declared at [`include/shulib/diag/shul2_format.hpp:581`](../../include/shulib/diag/shul2_format.hpp#L581).*

<a id="logline-flags"></a>

### `LogLine::flags`

```cpp
std::uint8_t flags = 0
```

kLogTagTruncated | kLogMessageTruncated; bits 2-7 ignored

*field, declared at [`include/shulib/diag/shul2_format.hpp:582`](../../include/shulib/diag/shul2_format.hpp#L582).*

<a id="logline-t"></a>

### `LogLine::t`

```cpp
double t = 0.0
```

the clock's raw reading when log() was called, seconds

*field, declared at [`include/shulib/diag/shul2_format.hpp:583`](../../include/shulib/diag/shul2_format.hpp#L583).*

<a id="logline-tag"></a>

### `LogLine::tag`

```cpp
std::string_view tag{}
```

the subsystem tag, verbatim

*field, declared at [`include/shulib/diag/shul2_format.hpp:584`](../../include/shulib/diag/shul2_format.hpp#L584).*

<a id="logline-message"></a>

### `LogLine::message`

```cpp
std::string_view message{}
```

the message, verbatim — never sanitized

*field, declared at [`include/shulib/diag/shul2_format.hpp:585`](../../include/shulib/diag/shul2_format.hpp#L585).*

<a id="logline-nontextbytes"></a>

### `LogLine::nonTextBytes`

```cpp
std::size_t nonTextBytes = kLogFixedBytes
```

Decoded only: every payload byte that is not text (payloadBytes − tag − message), which a reader compares with the Hello's logFixedBytes. The encoder always writes 1.0's 14.

*field, declared at [`include/shulib/diag/shul2_format.hpp:588`](../../include/shulib/diag/shul2_format.hpp#L588).*

<a id="encodelog"></a>

## `encodeLog`

```cpp
[[nodiscard]] inline std::size_t encodeLog(std::span<std::byte> out, const LogLine& line) noexcept
```

Encode one Log payload: the 14 fixed bytes, then the tag, then the message, verbatim. The caller has already applied any caps (a writer POLICY; utf8Prefix() cuts on a boundary). Returns 14 + tag + message, or 0 — writing nothing — if the tag is longer than 255 bytes, the payload would exceed kMaxPayloadBytes, or `out` is too small.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:595`](../../include/shulib/diag/shul2_format.hpp#L595).*

<a id="struct-endinfo"></a>

## `struct EndInfo`

```cpp
struct EndInfo
```

The End: the writer's own account of a session, sent at close().

*struct, declared at [`include/shulib/diag/shul2_format.hpp:617`](../../include/shulib/diag/shul2_format.hpp#L617).*

<a id="endinfo-framesoffered"></a>

### `EndInfo::framesOffered`

```cpp
std::uint32_t framesOffered = 0
```

frames offered to the device — the End's own sequence number

*field, declared at [`include/shulib/diag/shul2_format.hpp:618`](../../include/shulib/diag/shul2_format.hpp#L618).*

<a id="endinfo-framesfailed"></a>

### `EndInfo::framesFailed`

```cpp
std::uint32_t framesFailed = 0
```

frames the device refused (they appear as gaps)

*field, declared at [`include/shulib/diag/shul2_format.hpp:619`](../../include/shulib/diag/shul2_format.hpp#L619).*

<a id="endinfo-recordsseen"></a>

### `EndInfo::recordsSeen`

```cpp
std::uint32_t recordsSeen = 0
```

DebugRecords handed to the sink this session

*field, declared at [`include/shulib/diag/shul2_format.hpp:620`](../../include/shulib/diag/shul2_format.hpp#L620).*

<a id="endinfo-linesseen"></a>

### `EndInfo::linesSeen`

```cpp
std::uint32_t linesSeen = 0
```

log() lines handed to the sink this session

*field, declared at [`include/shulib/diag/shul2_format.hpp:621`](../../include/shulib/diag/shul2_format.hpp#L621).*

<a id="endinfo-linestruncated"></a>

### `EndInfo::linesTruncated`

```cpp
std::uint32_t linesTruncated = 0
```

lines whose tag or message was cut to its cap

*field, declared at [`include/shulib/diag/shul2_format.hpp:622`](../../include/shulib/diag/shul2_format.hpp#L622).*

<a id="endinfo-notopendropped"></a>

### `EndInfo::notOpenDropped`

```cpp
std::uint32_t notOpenDropped = 0
```

items handed over while no session was open

*field, declared at [`include/shulib/diag/shul2_format.hpp:623`](../../include/shulib/diag/shul2_format.hpp#L623).*

<a id="endinfo-endtime"></a>

### `EndInfo::endTime`

```cpp
double endTime = 0.0
```

the clock's raw reading at close(), seconds

*field, declared at [`include/shulib/diag/shul2_format.hpp:624`](../../include/shulib/diag/shul2_format.hpp#L624).*

<a id="encodeend"></a>

## `encodeEnd`

```cpp
[[nodiscard]] inline std::size_t encodeEnd(std::span<std::byte> out, const EndInfo& e) noexcept
```

Encode the End payload (v1.0, 32 bytes). Returns 32, or 0 if `out` is too small.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:628`](../../include/shulib/diag/shul2_format.hpp#L628).*

<a id="decodehello"></a>

## `decodeHello`

```cpp
[[nodiscard]] inline bool decodeHello(std::span<const std::byte> payload, Hello& out) noexcept
```

Decode a Hello. Bytes 0-11 (signature and version) are decoded from any payload of at least 12 bytes — so a short or foreign Hello can still be judged — and the rest only from a payload of 256 bytes or more whose major is kVersionMajor, which sets `complete`. Another major's bytes 12 onward belong to that major (header, "Versions"), so they are never read with this major's layout: its role, widths, epoch and text stay at their defaults. Returns false only when fewer than 12 bytes arrived. Never judges: negotiate() does. Text fields view `payload` and end at their first NUL, or at their field's width.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:667`](../../include/shulib/diag/shul2_format.hpp#L667).*

<a id="decodelog"></a>

## `decodeLog`

```cpp
[[nodiscard]] inline bool decodeLog(std::span<const std::byte> payload, LogLine& out) noexcept
```

Decode a Log payload. Returns false if it is shorter than 14 bytes or its declared tag and message run past its end. Fields a later minor appends after the text are not read, and show in `nonTextBytes`. The level stays a raw code: map it with logLevelFromWire().

*free function, declared at [`include/shulib/diag/shul2_format.hpp:705`](../../include/shulib/diag/shul2_format.hpp#L705).*

<a id="decodeend"></a>

## `decodeEnd`

```cpp
[[nodiscard]] inline bool decodeEnd(std::span<const std::byte> payload, EndInfo& e) noexcept
```

Decode an End payload of at least 32 bytes (a later minor's appended bytes are not read). Returns false if it is shorter.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:731`](../../include/shulib/diag/shul2_format.hpp#L731).*

<a id="decodetick"></a>

## `decodeTick`

```cpp
[[nodiscard]] inline bool decodeTick(std::span<const std::byte> payload, DebugRecord& r, bool& corrupt) noexcept
```

Decode a Tick payload of at least 428 bytes through blackbox::decodeTick on its first 428 (a newer minor's appended bytes are not read). Returns false if it is shorter. `corrupt` is raised as the blackbox decoder raises it: a non-finite heading (decoded as 0) or a wheelCount above DebugRecord::kMaxWheels (decoded as kMaxWheels).

*free function, declared at [`include/shulib/diag/shul2_format.hpp:750`](../../include/shulib/diag/shul2_format.hpp#L750).*

<a id="decodesummary"></a>

## `decodeSummary`

```cpp
[[nodiscard]] inline bool decodeSummary(std::span<const std::byte> payload, RunSummary& s, std::uint32_t& blackboxDropped) noexcept
```

Decode a Summary payload of at least 168 bytes through blackbox::decodeSummary on its first 168. `blackboxDropped` receives the summary's own RunSummary::blackboxDropped figure as the writer carried it. Returns false if the payload is shorter.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:761`](../../include/shulib/diag/shul2_format.hpp#L761).*

<a id="enum-class-wirelevel"></a>

## `enum class WireLevel`

```cpp
enum class WireLevel : std::uint8_t
```

The wire's level codes. EXPLICIT values, because hal::LogLevel declares none: the two are mapped by a switch in both directions (wireLevel, logLevelFromWire), never by a cast.

*enum class, declared at [`include/shulib/diag/shul2_format.hpp:773`](../../include/shulib/diag/shul2_format.hpp#L773).*

<a id="wirelevel-error"></a>

### `WireLevel::Error`

```cpp
Error = 0
```

hal::LogLevel::Error

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:774`](../../include/shulib/diag/shul2_format.hpp#L774).*

<a id="wirelevel-warn"></a>

### `WireLevel::Warn`

```cpp
Warn = 1
```

hal::LogLevel::Warn

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:775`](../../include/shulib/diag/shul2_format.hpp#L775).*

<a id="wirelevel-info"></a>

### `WireLevel::Info`

```cpp
Info = 2
```

hal::LogLevel::Info

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:776`](../../include/shulib/diag/shul2_format.hpp#L776).*

<a id="wirelevel-debug"></a>

### `WireLevel::Debug`

```cpp
Debug = 3
```

hal::LogLevel::Debug

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:777`](../../include/shulib/diag/shul2_format.hpp#L777).*

<a id="wirelevel-trace"></a>

### `WireLevel::Trace`

```cpp
Trace = 4
```

hal::LogLevel::Trace

*enumerator, declared at [`include/shulib/diag/shul2_format.hpp:778`](../../include/shulib/diag/shul2_format.hpp#L778).*

<a id="wirelevel"></a>

## `wireLevel`

```cpp
[[nodiscard]] constexpr WireLevel wireLevel(hal::LogLevel level) noexcept
```

The wire code for a hal::LogLevel (a switch, never a cast).

*free function, declared at [`include/shulib/diag/shul2_format.hpp:782`](../../include/shulib/diag/shul2_format.hpp#L782).*

<a id="loglevelfromwire"></a>

## `logLevelFromWire`

```cpp
[[nodiscard]] constexpr bool logLevelFromWire(std::uint8_t code, hal::LogLevel& out) noexcept
```

Map a raw wire level code back to hal::LogLevel (a switch, never a cast). Returns false — "unknown", `out` untouched — for a code this build does not name; the code itself stays available as the raw number in LogLine::level.

*free function, declared at [`include/shulib/diag/shul2_format.hpp:796`](../../include/shulib/diag/shul2_format.hpp#L796).*

<a id="utf8prefix"></a>

## `utf8Prefix`

```cpp
[[nodiscard]] constexpr std::string_view utf8Prefix(std::string_view text, std::size_t cap) noexcept
```

The longest prefix of `text` of at most `cap` bytes that does not split a UTF-8 sequence: `text` itself when it fits, otherwise cut at `cap` and backed off over continuation bytes — the same back-off as lineformat::Line::appendSanitized, WITHOUT the sanitizing (wire text is carried verbatim). For a writer's caps (a policy, H1 Part A2).

*free function, declared at [`include/shulib/diag/shul2_format.hpp:811`](../../include/shulib/diag/shul2_format.hpp#L811).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 170 lines, click to expand</summary>

```text

 SHUL/2 — the telemetry WIRE, format v1.0: DebugRecord streamed, sequenced, checked and
 versioned (master plan §18.1; diagnostics-plan; chunk H1 Part A1, 2026-09-29). This header
 is the format: the envelope, the five frame types, their layouts, the version rules, the
 encoders and decoders, and scanFrame() — the one function every reader is built on. Its
 reader over a whole capture is diag/shul2_reader.hpp. Its sink (Shul2Sink) is H1 Part A2's
 and does not exist yet; nothing in src/ writes this wire.

 NO HARDWARE CLAIM. These bytes have never crossed a real link — no USB port, no socket, no
 radio. Every byte here is a host byte, proven by the host suite. What a V5 brain's stdout
 does to them is a Hardware Assumption the wire's sink (H1 Part A2) will register — not
 something this header knows.

 NOT FROZEN YET. Register row F9 freezes this wire at H1 Part B; until then every layout
 below may still move. After it, the rules in "What every major keeps" and the 1.0 layouts
 are the contract, and growth is by appending (the version rules below).

 ── Binary, not JSON (decision D1) ──────────────────────────────────────────────────
 Earlier documents called SHUL/2 "the JSON wire". It is binary: little-endian, fixed-width
 fields, every floating-point value IEEE-754 binary64 with no narrowing — the blackbox's rules
 (blackbox_format.hpp). JSON cannot carry NaN or Inf at all; a double survives decimal text
 only with 17 significant digits and a correct parser on both ends; and it would bring heap
 and formatting cost onto the brain, every tick. Every field is written byte by byte through
 blackbox::ByteWriter / ByteReader — never a struct memcpy (padding and ABI are not a format).

 ── The Tick and Summary payloads ARE the blackbox's bytes (D2) ─────────────────────
 A Tick payload is blackbox::encodeTick's 428 bytes and a Summary payload is
 blackbox::encodeSummary's 168, and this header has NO encoder of its own for either. One
 field order by construction: a Tick read off the SD card can be re-wrapped as a SHUL/2 frame
 without decoding it — the premise of a replay in VexBuilder. Two encoders that must agree
 are the blindness E1's mutation 1b measured; one encoder cannot disagree with itself. The
 cost, stated: once F9 freezes, those two layouts are frozen with it (blackbox_format.hpp
 says so), and a minor that GROWS the tick must give the tick decoder an older-minor path.

 ── The envelope: 14 bytes per frame, the same in EVERY major version ──────────────
    off  size  field
      0     1  sync0 = 0xFE        (0xFE never occurs in valid UTF-8 text)
      1     1  sync1 = 0x53        ('S')
      2     1  type                (FrameType; 0 is never assigned)
      3     1  flags               (0 in major 1; readers IGNORE unknown bits)
      4     4  sequence (u32)      per writer, per session; see "Sequence"
      8     2  payloadBytes (u16)  at most kMaxPayloadBytes = 4096
     10     n  payload
   10+n     4  CRC-32/IEEE (u32)   diag::crc32Ieee over bytes [2, 10+n): type, flags,
                                   sequence, length and payload — everything but the sync
 The check is verified BEFORE a single byte inside the frame is believed (D3). Resync rule:
 after a failed candidate — no sync, a length above 4096, a CRC that does not verify —
 scanning resumes at the NEXT byte, never at the claimed end: a corrupted length would
 otherwise throw away every good frame it claimed. The cost is bounded: at most 4104 bytes of
 CRC per candidate position. Stated for a reader's budget, because it is per POSITION: bytes
 crafted so that sync pairs claiming a length the rule accepts sit as densely as possible cost
 about 1,200 bytes of bitwise CRC per input byte. (A candidate at p puts 0x53 at p + 1 and needs
 a length byte of at most 0x10 at p + 9, so neither p + 1, p + 8 nor p + 9 can start another;
 no byte pattern then holds more than three candidates per ten bytes on average.) Measured on a
 desktop host (g++ -O2, 2026-09-30): about 14 s per MB of that densest pattern with the bitwise
 CRC running at ~98 MB/s, and about 23 s per MB while other builds loaded the host (the CRC at
 ~55 MB/s). Ordinary damage and random bytes are cheap: a random sync pair occurs about once in
 65,536 positions, and a megabyte of random bytes reads in under a millisecond. A host-side
 reader that must bound its time may use a table-driven CRC-32 (the same values); the bitwise one
 is the brain's choice, for flash (diag/crc32.hpp).

 ── Frame types, and one writer per capture (D4, D5) ────────────────────────────────
   Hello = 1, Tick = 2, Log = 3, Summary = 4, End = 5.
 Hello and End are SESSION frames and travel in either direction (a peer's Hello carries
 role 1). The ranges govern DATA frames: 2-63 runtime → peer, 64-127 peer → runtime (H2's
 simulated sensor frames, and any later peer → runtime data), 128-255 reserved.
 A verified frame of a type this build does not assign — 0 included — is skipped by its
 length and counted. ONE capture (or one direction of a live link) carries ONE writer's bytes:
 a recorder that keeps both directions keeps two captures, because two writers interleaved
 into one byte stream break the sequence rules below.
 v1 does NOT carry: per-motion results or markers as data (both ride log() as text today, so
 they reach the wire inside Log frames, verbatim — an additive data type later), inbound
 frames (H2's, in the peer range), or a Triage frame (a stream carries every tick, including
 the one that faulted).

 ── The time base ───────────────────────────────────────────────────────────────────
 Every time on the wire — the Tick's `t`, the Log's `t`, the End's `endTime` — is the injected
 clock's RAW reading in seconds: the base the motion scheduler stamps DebugRecord::t in and
 TermSink stamps its lines in. The Hello's `epochSeconds` is the same clock's reading when the
 session opened; subtract it for session-relative time. (The blackbox does the same.)

 ── The Hello (v1.0: 256 bytes) ─────────────────────────────────────────────────────
    off  size  field                         off  size  field
      0     8  signature 89 53 48 55 4C 00 0D 0A    32     1  apiMajor (shulib::kApiMajor)
      8     2  versionMajor = 1               33     1  apiMinor (shulib::kApiMinor)
     10     2  versionMinor = 0               34     2  reserved (0)
     12     1  role (0 runtime, 1 peer)       36    48  buildHash  (text, NUL-padded)
     13     1  reserved (0)                   84    32  routineId
     14     2  flags (0)                     116    16  alliance
     16     2  tickPayloadBytes = 428        132    16  side
     18     2  summaryPayloadBytes = 168     148    96  portMap
     20     2  endPayloadBytes = 32          244    12  reserved (0)
     22     2  logFixedBytes = 14            256  = end
     24     8  epochSeconds (f64)
 Bytes 0-11 — the signature and the version — are the same in every major, and so is the
 Hello's type (1). Bytes 12 onward belong to their major. Text fields end at their first NUL
 (or fill their width); an EMPTY build hash stays empty, because MISSING must stay loud.
 Reserved bytes and flag bits are written 0 and NEVER judged by a reader, so a later minor may
 give them only a meaning that is safe to ignore.

 What the signature does, and what it cannot do. Its high-bit byte, its NUL and its CR LF
 guarantee that a transport which strips bit 7, truncates at a NUL, or translates newlines
 (either way) corrupts EVERY Hello — which matters most for newline translation, the one
 transform that can leave a Tick intact (a Tick holding no 0x0A or 0x0D byte), so no session
 ever starts on such a link. But the CRC catches that corruption BEFORE the signature is
 compared, so a reader reports such a capture as NoFrame or NoSession (shul2_reader.hpp),
 never as BadSignature. BadSignature means a Hello whose CRC VERIFIED and whose signature is
 wrong: a writer bug, or a stream that is not SHUL/2 inside a SHUL-shaped envelope.

 ── The Tick (428 bytes) and the Summary (168 bytes) ────────────────────────────────
 Laid out in blackbox_format.hpp ("The tick record", "The summary frame"). Canonical units:
 inches, radians (headings wrapped), seconds, volts, amperes, in/s, rad/s. The small
 vocabularies travel as their numbers — fault (u16 FaultCode), gateReason (u8 GateReason),
 activeCommandState (u8 motion::MotionState), qualityClass (u8, the Localizer::Quality mirror
 0-3) — and the tick flags byte is bit 0 clampedThisTick, bit 1 strafeFallbackActive, bit 2
 tankSteerActive (TANK1), bits 3-7 written 0 and ignored. A value this build does not name
 decodes as its number and is never refused. The Summary's `blackboxDropped` field carries the
 summary's OWN figure (RunSummary::blackboxDropped): the wire never overwrites another sink's
 count, and its own losses ride the End.

 ── The Log (14 bytes, then the text) ───────────────────────────────────────────────
    off  size  field
      0     1  level (wire code: Error 0, Warn 1, Info 2, Debug 3, Trace 4)
      1     1  flags (bit 0 tag truncated, bit 1 message truncated; bits 2-7 ignored)
      2     1  tagBytes
      3     1  reserved (0)
      4     2  messageBytes (u16)
      6     8  t (f64, the clock's raw reading when log() was called)
     14        tag, then message — VERBATIM, never sanitized, never NUL-terminated
 The text always starts at offset 14; fields a later minor appends go AFTER the text.
 `logFixedBytes` (the Hello) means every Log byte that is not text: payloadBytes - tagBytes -
 messageBytes. 14 in 1.0; a minor appending k bytes after the text declares 14 + k. The level
 codes map to hal::LogLevel by a switch in both directions, never by a cast (LogLevel declares
 no explicit values); an unknown code decodes as its raw number and maps to "unknown".

 ── The End (32 bytes) ──────────────────────────────────────────────────────────────
    off  size  field                     off  size  field
      0     4  framesOffered (= the End's   16     4  linesTruncated
               own sequence number)         20     4  notOpenDropped
      4     4  framesFailed (device refused) 24    8  endTime (f64, clock's raw reading)
      8     4  recordsSeen                  32  = end
     12     4  linesSeen

 ── Versions (D4, D8) ───────────────────────────────────────────────────────────────
 What every major keeps: the envelope (sync bytes, offsets and widths, the 4096 bound, the
 CRC and what it covers), the Hello's type (1) and the Hello's payload bytes 0-11. Without
 that, a 1.x reader could not even NAME a 2.0 stream as refused. Everything else may change
 at a major. negotiate() then judges a stream's Hello against the reader's profile:
   * another MAJOR is refused whole;
   * a NEWER minor is ACCEPTED: the reader decodes the prefix of each payload it knows, and
     ignores and counts the appended bytes — so every frame type grows only by APPENDING;
   * an OLDER minor is accepted when its declared widths equal THAT minor's row in the
     reader's width table (kWidthTable, one row per minor, append-only), and the fields it
     lacks take their documented defaults;
   * at the SAME minor, any width that differs is refused — the blackbox's width cross-check,
     kept where it belongs: it catches a changed layout whose version was never bumped.
 The blackbox's decoders take EXACTLY their width, so the day a minor grows the tick, a reader
 built on them needs an older-minor decode path; H1 Part B pins that with a permanent test.

 ── What changed shape here, and why (rejected alternatives) ────────────────────────
   * No check on each frame (the blackbox's choice) — rejected: a wire over a lossy link must
     add its own (blackbox_reader.hpp says so), and after a freeze a check could only arrive
     with a major bump.
   * COBS byte-stuffing — rejected: PROS already COBS-frames stdout, a local socket does not
     need it, and sync + length + CRC resynchronises within a bounded number of bytes.
   * The blackbox's refuse-any-other-width rule — rejected for the wire: VexBuilder's contract
     promises that unknown newer fields are ignored, never fatal.

 PROS-free, sim-free, allocation-free, never throws. A leaf of diag/: it includes only diag/,
 the hal/ telemetry seam, and shulib/version.hpp.
```

</details>
