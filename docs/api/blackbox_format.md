<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/blackbox_format.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `blackbox_format.hpp`

The SHULIB BLACKBOX on-disk format, v1 — the binary record SdSink writes and BlackboxReader reads.

This header declares **6** types (55 members), **12** free functions, and **8** constants.

Extracted from [`include/shulib/diag/blackbox_format.hpp`](../../include/shulib/diag/blackbox_format.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMagic`](#kmagic) — *constant*
- [`kFormatVersion`](#kformatversion) — *constant*
- [`kHeaderBytes`](#kheaderbytes) — *constant*
- [`kFrameHeaderBytes`](#kframeheaderbytes) — *constant*
- [`kTickPayloadBytes`](#ktickpayloadbytes) — *constant*
- [`kSummaryPayloadBytes`](#ksummarypayloadbytes) — *constant*
- [`kTriagePayloadBytes`](#ktriagepayloadbytes) — *constant*
- [`kEndPayloadBytes`](#kendpayloadbytes) — *constant*
- [`enum class FrameType`](#enum-class-frametype)
  - [`Tick`](#frametype-tick)
  - [`Summary`](#frametype-summary)
  - [`Triage`](#frametype-triage)
  - [`End`](#frametype-end)
- [`struct TriageInfo`](#struct-triageinfo)
  - [`fault`](#triageinfo-fault)
  - [`brownout`](#triageinfo-brownout)
  - [`tickIndex`](#triageinfo-tickindex)
  - [`faultTime`](#triageinfo-faulttime)
  - [`precedingTicks`](#triageinfo-precedingticks)
- [`struct EndInfo`](#struct-endinfo)
  - [`tickFrames`](#endinfo-tickframes)
  - [`droppedFrames`](#endinfo-droppedframes)
  - [`bytesBefore`](#endinfo-bytesbefore)
  - [`messagesSeen`](#endinfo-messagesseen)
  - [`brownout`](#endinfo-brownout)
  - [`deviceFailed`](#endinfo-devicefailed)
  - [`endTime`](#endinfo-endtime)
- [`struct BlackboxHeader`](#struct-blackboxheader)
  - [`formatVersion`](#blackboxheader-formatversion)
  - [`headerBytes`](#blackboxheader-headerbytes)
  - [`tickRecordBytes`](#blackboxheader-tickrecordbytes)
  - [`flags`](#blackboxheader-flags)
  - [`epochSeconds`](#blackboxheader-epochseconds)
  - [`ringCapacity`](#blackboxheader-ringcapacity)
  - [`byteBudget`](#blackboxheader-bytebudget)
  - [`buildHash`](#blackboxheader-buildhash)
  - [`routineId`](#blackboxheader-routineid)
  - [`alliance`](#blackboxheader-alliance)
  - [`side`](#blackboxheader-side)
  - [`portMap`](#blackboxheader-portmap)
  - [`buildHash_`](#blackboxheader-buildhash_)
  - [`routineId_`](#blackboxheader-routineid_)
  - [`alliance_`](#blackboxheader-alliance_)
  - [`side_`](#blackboxheader-side_)
  - [`portMap_`](#blackboxheader-portmap_)
- [`class ByteWriter`](#class-bytewriter)
  - [`ByteWriter`](#bytewriter-bytewriter)
  - [`u8`](#bytewriter-u8)
  - [`boolean`](#bytewriter-boolean)
  - [`u16`](#bytewriter-u16)
  - [`u32`](#bytewriter-u32)
  - [`i32`](#bytewriter-i32)
  - [`f64`](#bytewriter-f64)
  - [`text`](#bytewriter-text)
  - [`zeros`](#bytewriter-zeros)
  - [`offset`](#bytewriter-offset)
  - [`ok`](#bytewriter-ok)
- [`class ByteReader`](#class-bytereader)
  - [`ByteReader`](#bytereader-bytereader)
  - [`u8`](#bytereader-u8)
  - [`boolean`](#bytereader-boolean)
  - [`u16`](#bytereader-u16)
  - [`u32`](#bytereader-u32)
  - [`i32`](#bytereader-i32)
  - [`f64`](#bytereader-f64)
  - [`text`](#bytereader-text)
  - [`skip`](#bytereader-skip)
  - [`offset`](#bytereader-offset)
  - [`ok`](#bytereader-ok)
- [`encodeHeader`](#encodeheader) — *free function*
- [`decodeHeader`](#decodeheader) — *free function*
- [`encodeTick`](#encodetick) — *free function*
- [`safeAngle`](#safeangle) — *free function*
- [`decodeTick`](#decodetick) — *free function*
- [`encodeSummary`](#encodesummary) — *free function*
- [`decodeSummary`](#decodesummary) — *free function*
- [`encodeTriage`](#encodetriage) — *free function*
- [`decodeTriage`](#decodetriage) — *free function*
- [`encodeEnd`](#encodeend) — *free function*
- [`decodeEnd`](#decodeend) — *free function*
- [`encodeFrameHeader`](#encodeframeheader) — *free function*

<a id="kmagic"></a>

## `kMagic`

```cpp
inline constexpr char kMagic[4] = {'S', 'H', 'B', 'B'}
```

The four magic bytes every blackbox file starts with ("SHulib BlackBox").

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:70`](../../include/shulib/diag/blackbox_format.hpp#L70).*

<a id="kformatversion"></a>

## `kFormatVersion`

```cpp
inline constexpr std::uint16_t kFormatVersion = 1
```

On-disk format version. BUMP THIS whenever any layout below changes — a reader refuses a version it was not built for rather than misreading it (header note).

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:74`](../../include/shulib/diag/blackbox_format.hpp#L74).*

<a id="kheaderbytes"></a>

## `kHeaderBytes`

```cpp
inline constexpr std::size_t kHeaderBytes = 256
```

Size of the fixed file header, in bytes (v1). Fixed width so a reader can seek past it without parsing, and generous enough to hold full provenance.

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:78`](../../include/shulib/diag/blackbox_format.hpp#L78).*

<a id="kframeheaderbytes"></a>

## `kFrameHeaderBytes`

```cpp
inline constexpr std::size_t kFrameHeaderBytes = 4
```

Size of the per-frame prefix: {u8 type, u8 reserved, u16 payloadBytes}.

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:81`](../../include/shulib/diag/blackbox_format.hpp#L81).*

<a id="ktickpayloadbytes"></a>

## `kTickPayloadBytes`

```cpp
inline constexpr std::size_t kTickPayloadBytes = 428
```

Payload size of one Tick frame (v1). Pinned by the golden test; the encoder asserts it wrote exactly this many bytes.

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:85`](../../include/shulib/diag/blackbox_format.hpp#L85).*

<a id="ksummarypayloadbytes"></a>

## `kSummaryPayloadBytes`

```cpp
inline constexpr std::size_t kSummaryPayloadBytes = 168
```

Payload size of one Summary frame (v1).

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:88`](../../include/shulib/diag/blackbox_format.hpp#L88).*

<a id="ktriagepayloadbytes"></a>

## `kTriagePayloadBytes`

```cpp
inline constexpr std::size_t kTriagePayloadBytes = 24 + kTickPayloadBytes
```

Payload size of one Triage frame (v1): the D-7 triage fields PLUS the complete record of the tick the fault fired on (header note on dump ordering in sd_sink.hpp).

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:92`](../../include/shulib/diag/blackbox_format.hpp#L92).*

<a id="kendpayloadbytes"></a>

## `kEndPayloadBytes`

```cpp
inline constexpr std::size_t kEndPayloadBytes = 28
```

Payload size of one End frame (v1) — the graceful-end stamp.

*constant, declared at [`include/shulib/diag/blackbox_format.hpp:95`](../../include/shulib/diag/blackbox_format.hpp#L95).*

<a id="enum-class-frametype"></a>

## `enum class FrameType`

```cpp
enum class FrameType : std::uint8_t
```

What a frame carries. WIRE-STABLE: explicit values, append-only — an unknown type is skipped by length, never guessed at.

*enum class, declared at [`include/shulib/diag/blackbox_format.hpp:99`](../../include/shulib/diag/blackbox_format.hpp#L99).*

<a id="frametype-tick"></a>

### `FrameType::Tick`

```cpp
Tick = 1
```

one DebugRecord (kTickPayloadBytes)

*enumerator, declared at [`include/shulib/diag/blackbox_format.hpp:100`](../../include/shulib/diag/blackbox_format.hpp#L100).*

<a id="frametype-summary"></a>

### `FrameType::Summary`

```cpp
Summary = 2
```

one RunSummary (kSummaryPayloadBytes)

*enumerator, declared at [`include/shulib/diag/blackbox_format.hpp:101`](../../include/shulib/diag/blackbox_format.hpp#L101).*

<a id="frametype-triage"></a>

### `FrameType::Triage`

```cpp
Triage = 3
```

the D-7 fault triage block + the fault tick's own record

*enumerator, declared at [`include/shulib/diag/blackbox_format.hpp:102`](../../include/shulib/diag/blackbox_format.hpp#L102).*

<a id="frametype-end"></a>

### `FrameType::End`

```cpp
End = 4
```

the graceful-end stamp: counts, brownout latch, end time

*enumerator, declared at [`include/shulib/diag/blackbox_format.hpp:103`](../../include/shulib/diag/blackbox_format.hpp#L103).*

<a id="struct-triageinfo"></a>

## `struct TriageInfo`

```cpp
struct TriageInfo
```

The D-7 triage block, as data: which fault, when, on which tick, and how many preceding ticks follow it in the file. The record of the fault tick itself travels in the same frame (see sd_sink.hpp's dump-ordering rule).

*struct, declared at [`include/shulib/diag/blackbox_format.hpp:109`](../../include/shulib/diag/blackbox_format.hpp#L109).*

<a id="triageinfo-fault"></a>

### `TriageInfo::fault`

```cpp
FaultCode fault = FaultCode::None
```

the fault that triggered the dump

*field, declared at [`include/shulib/diag/blackbox_format.hpp:110`](../../include/shulib/diag/blackbox_format.hpp#L110).*

<a id="triageinfo-brownout"></a>

### `TriageInfo::brownout`

```cpp
bool brownout = false
```

the latched brownout marker at dump time

*field, declared at [`include/shulib/diag/blackbox_format.hpp:111`](../../include/shulib/diag/blackbox_format.hpp#L111).*

<a id="triageinfo-tickindex"></a>

### `TriageInfo::tickIndex`

```cpp
std::uint32_t tickIndex = 0
```

how many records the sink had seen when it fired

*field, declared at [`include/shulib/diag/blackbox_format.hpp:112`](../../include/shulib/diag/blackbox_format.hpp#L112).*

<a id="triageinfo-faulttime"></a>

### `TriageInfo::faultTime`

```cpp
double faultTime = 0.0
```

the fault tick's `t`, seconds since the run epoch

*field, declared at [`include/shulib/diag/blackbox_format.hpp:113`](../../include/shulib/diag/blackbox_format.hpp#L113).*

<a id="triageinfo-precedingticks"></a>

### `TriageInfo::precedingTicks`

```cpp
std::uint32_t precedingTicks = 0
```

Tick frames that follow, oldest first (0 when streaming)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:114`](../../include/shulib/diag/blackbox_format.hpp#L114).*

<a id="struct-endinfo"></a>

## `struct EndInfo`

```cpp
struct EndInfo
```

The end frame: what the sink knows about its own run when it closes cleanly. A file WITHOUT this frame ended abruptly — that absence is the truncation signal a reader can act on.

*struct, declared at [`include/shulib/diag/blackbox_format.hpp:120`](../../include/shulib/diag/blackbox_format.hpp#L120).*

<a id="endinfo-tickframes"></a>

### `EndInfo::tickFrames`

```cpp
std::uint32_t tickFrames = 0
```

Tick frames staged over the run

*field, declared at [`include/shulib/diag/blackbox_format.hpp:121`](../../include/shulib/diag/blackbox_format.hpp#L121).*

<a id="endinfo-droppedframes"></a>

### `EndInfo::droppedFrames`

```cpp
std::uint32_t droppedFrames = 0
```

frames dropped for want of buffer (byte budget)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:122`](../../include/shulib/diag/blackbox_format.hpp#L122).*

<a id="endinfo-bytesbefore"></a>

### `EndInfo::bytesBefore`

```cpp
std::uint32_t bytesBefore = 0
```

Bytes of this file that PRECEDE this frame — i.e. the frame's own offset. A reader can verify it against where it actually found the frame, which is how a file that was appended to, interleaved, or spliced gives itself away. (It is NOT "bytes the device confirmed": at close() the bulk of a caller-paced run is still staged and goes out in the same write as this frame, so that figure would read 0 for the most common run of all.)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:129`](../../include/shulib/diag/blackbox_format.hpp#L129).*

<a id="endinfo-messagesseen"></a>

### `EndInfo::messagesSeen`

```cpp
std::uint32_t messagesSeen = 0
```

log() lines handed to the sink and NOT carried (header note)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:130`](../../include/shulib/diag/blackbox_format.hpp#L130).*

<a id="endinfo-brownout"></a>

### `EndInfo::brownout`

```cpp
bool brownout = false
```

the latched brownout marker

*field, declared at [`include/shulib/diag/blackbox_format.hpp:131`](../../include/shulib/diag/blackbox_format.hpp#L131).*

<a id="endinfo-devicefailed"></a>

### `EndInfo::deviceFailed`

```cpp
bool deviceFailed = false
```

a write() or flush() reported failure during the run

*field, declared at [`include/shulib/diag/blackbox_format.hpp:132`](../../include/shulib/diag/blackbox_format.hpp#L132).*

<a id="endinfo-endtime"></a>

### `EndInfo::endTime`

```cpp
double endTime = 0.0
```

clock time at close, seconds since the run epoch

*field, declared at [`include/shulib/diag/blackbox_format.hpp:133`](../../include/shulib/diag/blackbox_format.hpp#L133).*

<a id="struct-blackboxheader"></a>

## `struct BlackboxHeader`

```cpp
struct BlackboxHeader
```

A decoded file header. Value type with bounded storage, like RunSummary: a decoded header must never hold views into a buffer the caller may free.

*struct, declared at [`include/shulib/diag/blackbox_format.hpp:138`](../../include/shulib/diag/blackbox_format.hpp#L138).*

<a id="blackboxheader-formatversion"></a>

### `BlackboxHeader::formatVersion`

```cpp
std::uint16_t formatVersion = 0
```

as read from the file

*field, declared at [`include/shulib/diag/blackbox_format.hpp:139`](../../include/shulib/diag/blackbox_format.hpp#L139).*

<a id="blackboxheader-headerbytes"></a>

### `BlackboxHeader::headerBytes`

```cpp
std::uint16_t headerBytes = 0
```

self-declared header size (lets a reader seek)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:140`](../../include/shulib/diag/blackbox_format.hpp#L140).*

<a id="blackboxheader-tickrecordbytes"></a>

### `BlackboxHeader::tickRecordBytes`

```cpp
std::uint16_t tickRecordBytes = 0
```

self-declared Tick payload size (cross-checked)

*field, declared at [`include/shulib/diag/blackbox_format.hpp:141`](../../include/shulib/diag/blackbox_format.hpp#L141).*

<a id="blackboxheader-flags"></a>

### `BlackboxHeader::flags`

```cpp
std::uint16_t flags = 0
```

reserved, 0 in v1

*field, declared at [`include/shulib/diag/blackbox_format.hpp:142`](../../include/shulib/diag/blackbox_format.hpp#L142).*

<a id="blackboxheader-epochseconds"></a>

### `BlackboxHeader::epochSeconds`

```cpp
double epochSeconds = 0.0
```

the injected clock's reading when the file opened

*field, declared at [`include/shulib/diag/blackbox_format.hpp:143`](../../include/shulib/diag/blackbox_format.hpp#L143).*

<a id="blackboxheader-ringcapacity"></a>

### `BlackboxHeader::ringCapacity`

```cpp
std::uint32_t ringCapacity = 0
```

flight-recorder ring size the writer was configured with

*field, declared at [`include/shulib/diag/blackbox_format.hpp:144`](../../include/shulib/diag/blackbox_format.hpp#L144).*

<a id="blackboxheader-bytebudget"></a>

### `BlackboxHeader::byteBudget`

```cpp
std::uint32_t byteBudget = 0
```

RAM byte budget the writer was configured with

*field, declared at [`include/shulib/diag/blackbox_format.hpp:145`](../../include/shulib/diag/blackbox_format.hpp#L145).*

<a id="blackboxheader-buildhash"></a>

### `BlackboxHeader::buildHash`

```cpp
[[nodiscard]] std::string_view buildHash() const noexcept
```

The git build hash the run was built from. EMPTY means MISSING — render it loudly and never invent a plausible value (§18.5, build_info.hpp).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:149`](../../include/shulib/diag/blackbox_format.hpp#L149).*

<a id="blackboxheader-routineid"></a>

### `BlackboxHeader::routineId`

```cpp
[[nodiscard]] std::string_view routineId() const noexcept
```

The routine id the run was started with (may be empty).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:151`](../../include/shulib/diag/blackbox_format.hpp#L151).*

<a id="blackboxheader-alliance"></a>

### `BlackboxHeader::alliance`

```cpp
[[nodiscard]] std::string_view alliance() const noexcept
```

Alliance as free text ("red"/"blue"/"skills"); may be empty.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:153`](../../include/shulib/diag/blackbox_format.hpp#L153).*

<a id="blackboxheader-side"></a>

### `BlackboxHeader::side`

```cpp
[[nodiscard]] std::string_view side() const noexcept
```

Side as free text ("left"/"right"); may be empty.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:155`](../../include/shulib/diag/blackbox_format.hpp#L155).*

<a id="blackboxheader-portmap"></a>

### `BlackboxHeader::portMap`

```cpp
[[nodiscard]] std::string_view portMap() const noexcept
```

The caller-authored port map; may be empty.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:157`](../../include/shulib/diag/blackbox_format.hpp#L157).*

<a id="blackboxheader-buildhash_"></a>

### `BlackboxHeader::buildHash_`

```cpp
char buildHash_[48] = ""
```

Storage for buildHash() — written by the decoder, NUL-terminated.

*field, declared at [`include/shulib/diag/blackbox_format.hpp:160`](../../include/shulib/diag/blackbox_format.hpp#L160).*

<a id="blackboxheader-routineid_"></a>

### `BlackboxHeader::routineId_`

```cpp
char routineId_[32] = ""
```

Storage for routineId().

*field, declared at [`include/shulib/diag/blackbox_format.hpp:162`](../../include/shulib/diag/blackbox_format.hpp#L162).*

<a id="blackboxheader-alliance_"></a>

### `BlackboxHeader::alliance_`

```cpp
char alliance_[16] = ""
```

Storage for alliance().

*field, declared at [`include/shulib/diag/blackbox_format.hpp:164`](../../include/shulib/diag/blackbox_format.hpp#L164).*

<a id="blackboxheader-side_"></a>

### `BlackboxHeader::side_`

```cpp
char side_[16] = ""
```

Storage for side().

*field, declared at [`include/shulib/diag/blackbox_format.hpp:166`](../../include/shulib/diag/blackbox_format.hpp#L166).*

<a id="blackboxheader-portmap_"></a>

### `BlackboxHeader::portMap_`

```cpp
char portMap_[96] = ""
```

Storage for portMap().

*field, declared at [`include/shulib/diag/blackbox_format.hpp:168`](../../include/shulib/diag/blackbox_format.hpp#L168).*

<a id="class-bytewriter"></a>

## `class ByteWriter`

```cpp
class ByteWriter
```

Little-endian byte writer with a hard end: a write that would not fit writes NOTHING and latches overflow, so an undersized buffer can never corrupt neighbouring memory and can never half-write a field. Callers check ok().

*class, declared at [`include/shulib/diag/blackbox_format.hpp:174`](../../include/shulib/diag/blackbox_format.hpp#L174).*

<a id="bytewriter-bytewriter"></a>

### `ByteWriter::ByteWriter`

```cpp
explicit ByteWriter(std::span<std::byte> out) noexcept
```

Write into `out`, starting at offset 0.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:177`](../../include/shulib/diag/blackbox_format.hpp#L177).*

<a id="bytewriter-u8"></a>

### `ByteWriter::u8`

```cpp
void u8(std::uint8_t v) noexcept
```

Append one unsigned byte.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:180`](../../include/shulib/diag/blackbox_format.hpp#L180).*

<a id="bytewriter-boolean"></a>

### `ByteWriter::boolean`

```cpp
void boolean(bool v) noexcept
```

Append a bool as 0x00 / 0x01.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:187`](../../include/shulib/diag/blackbox_format.hpp#L187).*

<a id="bytewriter-u16"></a>

### `ByteWriter::u16`

```cpp
void u16(std::uint16_t v) noexcept
```

Append a 16-bit unsigned value, little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:189`](../../include/shulib/diag/blackbox_format.hpp#L189).*

<a id="bytewriter-u32"></a>

### `ByteWriter::u32`

```cpp
void u32(std::uint32_t v) noexcept
```

Append a 32-bit unsigned value, little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:197`](../../include/shulib/diag/blackbox_format.hpp#L197).*

<a id="bytewriter-i32"></a>

### `ByteWriter::i32`

```cpp
void i32(std::int32_t v) noexcept
```

Append a 32-bit signed value as two's complement, little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:206`](../../include/shulib/diag/blackbox_format.hpp#L206).*

<a id="bytewriter-f64"></a>

### `ByteWriter::f64`

```cpp
void f64(double v) noexcept
```

Append an IEEE-754 binary64 value, little-endian (bit pattern preserved, so a NaN or an infinity survives the trip exactly as it was recorded).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:209`](../../include/shulib/diag/blackbox_format.hpp#L209).*

<a id="bytewriter-text"></a>

### `ByteWriter::text`

```cpp
void text(std::string_view s, std::size_t fieldBytes) noexcept
```

Append `fieldBytes` of text: `s` truncated to fit, NUL-padded to the full width. Fixed width by design — a variable-length string would make every later offset depend on run-time content.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:222`](../../include/shulib/diag/blackbox_format.hpp#L222).*

<a id="bytewriter-zeros"></a>

### `ByteWriter::zeros`

```cpp
void zeros(std::size_t n) noexcept
```

Append `n` zero bytes (reserved space).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:232`](../../include/shulib/diag/blackbox_format.hpp#L232).*

<a id="bytewriter-offset"></a>

### `ByteWriter::offset`

```cpp
[[nodiscard]] std::size_t offset() const noexcept
```

How many bytes have been appended.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:241`](../../include/shulib/diag/blackbox_format.hpp#L241).*

<a id="bytewriter-ok"></a>

### `ByteWriter::ok`

```cpp
[[nodiscard]] bool ok() const noexcept
```

False once any append did not fit (nothing was written for that append).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:243`](../../include/shulib/diag/blackbox_format.hpp#L243).*

<a id="class-bytereader"></a>

## `class ByteReader`

```cpp
class ByteReader
```

Little-endian byte reader with a hard end: a read past the end yields zero and latches exhaustion, so a truncated or corrupt file can never read out of bounds and can never half-read a field. Callers check ok().

*class, declared at [`include/shulib/diag/blackbox_format.hpp:262`](../../include/shulib/diag/blackbox_format.hpp#L262).*

<a id="bytereader-bytereader"></a>

### `ByteReader::ByteReader`

```cpp
explicit ByteReader(std::span<const std::byte> in) noexcept
```

Read from `in`, starting at offset 0.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:265`](../../include/shulib/diag/blackbox_format.hpp#L265).*

<a id="bytereader-u8"></a>

### `ByteReader::u8`

```cpp
[[nodiscard]] std::uint8_t u8() noexcept
```

Read one unsigned byte (0 past the end).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:268`](../../include/shulib/diag/blackbox_format.hpp#L268).*

<a id="bytereader-boolean"></a>

### `ByteReader::boolean`

```cpp
[[nodiscard]] bool boolean() noexcept
```

Read a bool: any nonzero byte is true.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:275`](../../include/shulib/diag/blackbox_format.hpp#L275).*

<a id="bytereader-u16"></a>

### `ByteReader::u16`

```cpp
[[nodiscard]] std::uint16_t u16() noexcept
```

Read a 16-bit unsigned value, little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:277`](../../include/shulib/diag/blackbox_format.hpp#L277).*

<a id="bytereader-u32"></a>

### `ByteReader::u32`

```cpp
[[nodiscard]] std::uint32_t u32() noexcept
```

Read a 32-bit unsigned value, little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:286`](../../include/shulib/diag/blackbox_format.hpp#L286).*

<a id="bytereader-i32"></a>

### `ByteReader::i32`

```cpp
[[nodiscard]] std::int32_t i32() noexcept
```

Read a 32-bit signed value (two's complement), little-endian.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:297`](../../include/shulib/diag/blackbox_format.hpp#L297).*

<a id="bytereader-f64"></a>

### `ByteReader::f64`

```cpp
[[nodiscard]] double f64() noexcept
```

Read an IEEE-754 binary64 value, little-endian (bit pattern preserved).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:299`](../../include/shulib/diag/blackbox_format.hpp#L299).*

<a id="bytereader-text"></a>

### `ByteReader::text`

```cpp
void text(char* dst, std::size_t dstBytes, std::size_t fieldBytes) noexcept
```

Read `fieldBytes` of NUL-padded text into `dst` (capacity `dstBytes`, always NUL-terminated). Bytes beyond the destination are consumed and discarded, so the cursor stays aligned no matter how the caller sized its storage.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:314`](../../include/shulib/diag/blackbox_format.hpp#L314).*

<a id="bytereader-skip"></a>

### `ByteReader::skip`

```cpp
void skip(std::size_t n) noexcept
```

Skip `n` bytes (reserved space).

*function, declared at [`include/shulib/diag/blackbox_format.hpp:327`](../../include/shulib/diag/blackbox_format.hpp#L327).*

<a id="bytereader-offset"></a>

### `ByteReader::offset`

```cpp
[[nodiscard]] std::size_t offset() const noexcept
```

How many bytes have been consumed.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:333`](../../include/shulib/diag/blackbox_format.hpp#L333).*

<a id="bytereader-ok"></a>

### `ByteReader::ok`

```cpp
[[nodiscard]] bool ok() const noexcept
```

False once any read ran past the end.

*function, declared at [`include/shulib/diag/blackbox_format.hpp:335`](../../include/shulib/diag/blackbox_format.hpp#L335).*

<a id="encodeheader"></a>

## `encodeHeader`

```cpp
[[nodiscard]] inline std::size_t encodeHeader(std::span<std::byte> out, const SessionInfo& info, double epochSeconds, std::uint32_t ringCapacity, std::uint32_t byteBudget) noexcept
```

Encode the 256-byte file header into `out`. Returns the bytes written (0 if `out` is too small). Provenance strings are copied in, truncated to their field widths — an EMPTY build hash stays empty, because MISSING must stay loud all the way to disk.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:365`](../../include/shulib/diag/blackbox_format.hpp#L365).*

<a id="decodeheader"></a>

## `decodeHeader`

```cpp
[[nodiscard]] inline bool decodeHeader(std::span<const std::byte> in, BlackboxHeader& out) noexcept
```

Decode a file header. Returns false if `in` is shorter than the header or the magic does not match; the VERSION is decoded but NOT judged here — BlackboxReader owns the refusal policy, and a caller inspecting a rejected file still wants to see what version it claims to be.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:395`](../../include/shulib/diag/blackbox_format.hpp#L395).*

<a id="encodetick"></a>

## `encodeTick`

```cpp
[[nodiscard]] inline std::size_t encodeTick(std::span<std::byte> out, const DebugRecord& r) noexcept
```

Encode one DebugRecord. Returns the bytes written, or 0 if `out` was too small or the layout did not come out to exactly kTickPayloadBytes (a loud, testable failure rather than a silently short record).

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:440`](../../include/shulib/diag/blackbox_format.hpp#L440).*

<a id="safeangle"></a>

## `safeAngle`

```cpp
[[nodiscard]] inline math::Angle safeAngle(double radians, bool& corrupt) noexcept
```

Rebuild an Angle from a decoded radian value WITHOUT trusting the file: a corrupt or truncated blackbox can contain any bit pattern, and math::Angle's factory rejects non-finite input by precondition. A decoder that throws on a corrupt file is a decoder you cannot use on the file you most need to read, so a non-finite heading decodes to zero and `corrupt` is raised for the caller to see.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:500`](../../include/shulib/diag/blackbox_format.hpp#L500).*

<a id="decodetick"></a>

## `decodeTick`

```cpp
[[nodiscard]] inline bool decodeTick(std::span<const std::byte> in, DebugRecord& r, bool& corrupt) noexcept
```

Decode one DebugRecord. Returns false if the payload is not exactly kTickPayloadBytes. `corrupt` is set (never cleared) when a field could not be represented — today: a non-finite heading, which decodes to zero (safeAngle).

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:511`](../../include/shulib/diag/blackbox_format.hpp#L511).*

<a id="encodesummary"></a>

## `encodeSummary`

```cpp
[[nodiscard]] inline std::size_t encodeSummary(std::span<std::byte> out, const RunSummary& s, std::uint32_t blackboxDropped) noexcept
```

Encode one RunSummary. `blackboxDropped` is the SINK's own drop count, passed in rather than read from the summary so the file always carries the writer's live figure even when the caller assembled the summary before the last drop. Returns the bytes written, or 0 on a layout/space failure.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:592`](../../include/shulib/diag/blackbox_format.hpp#L592).*

<a id="decodesummary"></a>

## `decodeSummary`

```cpp
[[nodiscard]] inline bool decodeSummary(std::span<const std::byte> in, RunSummary& s, std::uint32_t& blackboxDropped) noexcept
```

Decode one RunSummary; `blackboxDropped` receives the sink's own drop count. Returns false if the payload is not exactly kSummaryPayloadBytes.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:623`](../../include/shulib/diag/blackbox_format.hpp#L623).*

<a id="encodetriage"></a>

## `encodeTriage`

```cpp
[[nodiscard]] inline std::size_t encodeTriage(std::span<std::byte> out, const TriageInfo& info, const DebugRecord& faultTick) noexcept
```

Encode the D-7 triage block plus the complete record of the tick the fault fired on. Returns the bytes written, or 0 on a layout/space failure.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:663`](../../include/shulib/diag/blackbox_format.hpp#L663).*

<a id="decodetriage"></a>

## `decodeTriage`

```cpp
[[nodiscard]] inline bool decodeTriage(std::span<const std::byte> in, TriageInfo& info, DebugRecord& faultTick, bool& corrupt) noexcept
```

Decode a triage frame and the fault tick's record. Returns false if the payload is not exactly kTriagePayloadBytes.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:688`](../../include/shulib/diag/blackbox_format.hpp#L688).*

<a id="encodeend"></a>

## `encodeEnd`

```cpp
[[nodiscard]] inline std::size_t encodeEnd(std::span<std::byte> out, const EndInfo& e) noexcept
```

Encode the graceful-end stamp. Its PRESENCE is the signal that the run closed cleanly; its absence is how a reader knows a file was cut short. Returns the bytes written, or 0 on a layout/space failure.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:711`](../../include/shulib/diag/blackbox_format.hpp#L711).*

<a id="decodeend"></a>

## `decodeEnd`

```cpp
[[nodiscard]] inline bool decodeEnd(std::span<const std::byte> in, EndInfo& e) noexcept
```

Decode the graceful-end stamp. Returns false if the payload is not exactly kEndPayloadBytes.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:729`](../../include/shulib/diag/blackbox_format.hpp#L729).*

<a id="encodeframeheader"></a>

## `encodeFrameHeader`

```cpp
[[nodiscard]] inline std::size_t encodeFrameHeader(std::span<std::byte> out, FrameType type, std::uint16_t payloadBytes) noexcept
```

Write a frame prefix {type, reserved, payloadBytes} into `out`. Returns the bytes written (kFrameHeaderBytes) or 0 if it did not fit.

*free function, declared at [`include/shulib/diag/blackbox_format.hpp:747`](../../include/shulib/diag/blackbox_format.hpp#L747).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 The SHULIB BLACKBOX on-disk format, v1 — the binary record SdSink writes and
 BlackboxReader reads (master plan §18; diagnostics-plan D-6/D-7; WS13, chunk E1).

 ── Why a format document lives in code ─────────────────────────────────────────────
 A file outlives the program that wrote it. Six months from now the only thing that
 can open a run from a competition day is a decoder that agrees with this layout, so
 the layout is written down ONCE, here, with its version, and the encoder and the
 decoder are both held to it BY A BYTE-EXACT GOLDEN TEST — not by their agreement
 with each other. Two sides that share one bug agree perfectly; that is precisely the
 failure a round trip cannot see, which is why test/blackbox_format_test.cpp pins the
 literal bytes of the header and of one full tick record.

 ── The shape ───────────────────────────────────────────────────────────────────────
   [ 256-byte file header ][ frame ][ frame ][ frame ] …
 Every frame is  { u8 type, u8 reserved, u16 payloadBytes }  followed by payloadBytes
 of payload. A reader that meets a frame type it does not know SKIPS it by its length
 and counts it — that is what makes a v1 reader survive a v1.1 writer that appended a
 new frame kind, without ever GUESSING at content it does not understand.

 ── Versioned from byte zero, and refusal over misreading ───────────────────────────
 The header carries kFormatVersion. A decoder that mis-reads an old file is worse
 than one that refuses it: a wrong number read confidently sends the 2am investigation
 somewhere false, while a refusal sends it to the git history. So the reader refuses
 any version it was not built for, and ALSO cross-checks the header's self-declared
 record width against its own — that catches the specific human error of changing the
 layout and forgetting to bump the version.

 ── Fixed width, and no narrowing ANYWHERE ──────────────────────────────────────────
 Every field is fixed width and little-endian, written byte by byte (never a struct
 memcpy: padding and ABI are not a file format). Every floating-point field is IEEE-754
 binary64, including the per-wheel arrays and the tick-phase slots, where binary32
 would have saved ~35% of the file. That trade was made deliberately: the chunk's
 central promise is that a decoded record equals the encoded one FIELD BY FIELD, and
 with narrowing that sentence would have quietly become "equals after a documented
 rounding" — a weaker claim that also makes every future comparison of two runs
 depend on rounding behaviour. Bytes are cheap on an SD card; a fuzzy record is not.

 ── What v1 does NOT carry, said plainly ────────────────────────────────────────────
 The log() message channel. Variable-length text in a fixed-width record format is a
 different problem with different trade-offs (bounded-vs-truncated strings, per-line
 framing), and the terminal (A1) already owns text. The blackbox carries the per-tick
 RECORD, the run SUMMARY, and the fault TRIAGE block. SdSink counts the messages it
 was handed and writes that count into the end frame, so the omission is visible in
 the file rather than silent — a reader can always see that N lines existed elsewhere.

 ── What H1 (F9, the SHUL/2 wire) inherits ──────────────────────────────────────────
 This is a PERSISTENCE contract the moment a file exists, but it is deliberately NOT
 the wire: SHUL/2 is streamed, sequenced and versioned on its own terms. What H1
 should reuse is the FIELD ORDER of encodeTick() (it follows debug_record.hpp's own
 declaration order, so a schema append lands at the end on both) and the refuse-don't-
 misread rule. Nothing here freezes anything: F9 is H1's, and E1 freezes nothing.
```

</details>
