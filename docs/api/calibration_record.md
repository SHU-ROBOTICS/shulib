<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibration_record.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibration_record.hpp`

The CALIBRATION RECORD — a hal::DrivetrainDescription as bytes on the SD card.

This header declares **5** free functions and **10** constants.

Extracted from [`include/shulib/calibration/calibration_record.hpp`](../../include/shulib/calibration/calibration_record.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kRecordMagic`](#krecordmagic) — *constant*
- [`kRecordVersion`](#krecordversion) — *constant*
- [`kRecordFileName`](#krecordfilename) — *constant*
- [`kRecordBackupFileName`](#krecordbackupfilename) — *constant*
- [`kProvenanceBytes`](#kprovenancebytes) — *constant*
- [`kRecordNameKey`](#krecordnamekey) — *constant*
- [`kRecordNameMax`](#krecordnamemax) — *constant*
- [`isRecordName`](#isrecordname) — *free function*
- [`recordRobotName`](#recordrobotname) — *free function*
- [`kRecordPayloadBytes`](#krecordpayloadbytes) — *constant*
- [`kRecordPrefixBytes`](#krecordprefixbytes) — *constant*
- [`kRecordBytes`](#krecordbytes) — *constant*
- [`crc32Ieee`](#crc32ieee) — *free function*
- [`encodeRecord`](#encoderecord) — *free function*
- [`decodeRecord`](#decoderecord) — *free function*

<a id="krecordmagic"></a>

## `kRecordMagic`

```cpp
inline constexpr char kRecordMagic[4] = {'S', 'H', 'D', 'C'}
```

The four magic bytes every calibration record starts with ("SHulib Drivetrain Calibration"). Distinct from the blackbox's "SHBB" so a file pointed at the wrong decoder is refused at byte 0 instead of being parsed into nonsense.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:81`](../../include/shulib/calibration/calibration_record.hpp#L81).*

<a id="krecordversion"></a>

## `kRecordVersion`

```cpp
inline constexpr std::uint16_t kRecordVersion = 1
```

On-card format version. BUMP THIS whenever the layout below changes — a decoder refuses a version it was not built for rather than misreading it. Append-only reasoning applies to the enums the payload carries as bytes (hal::DrivetrainKind, hal::MotorCartridge, hal::DescriptionSource all say so at their declarations).

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:87`](../../include/shulib/calibration/calibration_record.hpp#L87).*

<a id="krecordfilename"></a>

## `kRecordFileName`

```cpp
inline constexpr const char* kRecordFileName = "shulib_drivetrain.bin"
```

The ONE file name every program uses, so "shulib Calibrate" writes what "shulib Drive" (and every program that boots from the record) reads. BARE — the /usd/ prefix belongs to hal/pros/block_sink.hpp and block_source.hpp, which own it in one place each (HA-122).

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:92`](../../include/shulib/calibration/calibration_record.hpp#L92).*

<a id="krecordbackupfilename"></a>

## `kRecordBackupFileName`

```cpp
inline constexpr const char* kRecordBackupFileName = "shulib_drivetrain.bak"
```

The record's BACKUP copy (chunk SAFE-SAVE Part A): the same 180 bytes, in a second file, so that a save which fails part-way does not leave the card without a good record — given the two beliefs about the card it rests on (HA-166 (4) and (5)), and short of the two-fault limit calibration/record_store.hpp states. The brain cannot rename or delete a file, and "wb" is taken to empty one at the open (HA-166), so the safe save copies the old record HERE and reads it back before it opens the record file for writing, and a load that finds the record file damaged falls back to it. Named beside kRecordFileName so both programs share ONE spelling of each. BARE, like it.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:102`](../../include/shulib/calibration/calibration_record.hpp#L102).*

<a id="kprovenancebytes"></a>

## `kProvenanceBytes`

```cpp
inline constexpr std::size_t kProvenanceBytes = sizeof(hal::DrivetrainDescription::provenance)
```

Width of the description's provenance field, in bytes — taken from the struct rather than retyped, so a widening there is a compile error here instead of a silently short record.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:128`](../../include/shulib/calibration/calibration_record.hpp#L128).*

<a id="krecordnamekey"></a>

## `kRecordNameKey`

```cpp
inline constexpr std::string_view kRecordNameKey = "robot="
```

The key before a robot's name in a record's provenance: `robot=<name>`, at the start of a word.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:138`](../../include/shulib/calibration/calibration_record.hpp#L138).*

<a id="krecordnamemax"></a>

## `kRecordNameMax`

```cpp
inline constexpr std::size_t kRecordNameMax = 24
```

The longest robot name a record carries (the key, the name and a space fit the 40-character cut).

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:141`](../../include/shulib/calibration/calibration_record.hpp#L141).*

<a id="isrecordname"></a>

## `isRecordName`

```cpp
[[nodiscard]] constexpr bool isRecordName(std::string_view name) noexcept
```

True when `name` can be a record's robot name: 1..kRecordNameMax lower-case letters, digits or '-'.

*free function, declared at [`include/shulib/calibration/calibration_record.hpp:144`](../../include/shulib/calibration/calibration_record.hpp#L144).*

<a id="recordrobotname"></a>

## `recordRobotName`

```cpp
[[nodiscard]] constexpr std::string_view recordRobotName(const char (&provenance)[kProvenanceBytes]) noexcept
```

The robot a record's provenance names: the word after the first `robot=` that starts a word, when it is a record name (isRecordName) — else EMPTY (an older record, or a name that was cut). `provenance` is the stored field (it need not end in a NUL when full); the view points into it.

*free function, declared at [`include/shulib/calibration/calibration_record.hpp:160`](../../include/shulib/calibration/calibration_record.hpp#L160).*

<a id="krecordpayloadbytes"></a>

## `kRecordPayloadBytes`

```cpp
inline constexpr std::size_t kRecordPayloadBytes = 1 + 1 + 1 + (hal::kMaxSmartPorts + 1 + 8) * 2 + 8 + 1 + kProvenanceBytes
```

Bytes of the payload (everything between the length field and the CRC), v1: kind 1 + cartridge 1 + imuPort 1 + left (21 + 1 + 8) + right (21 + 1 + 8) + trackWidth 8 + source 1 + provenance 96 = 168. The encoder asserts it wrote exactly this many.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:184`](../../include/shulib/calibration/calibration_record.hpp#L184).*

<a id="krecordprefixbytes"></a>

## `kRecordPrefixBytes`

```cpp
inline constexpr std::size_t kRecordPrefixBytes = 4 + 2 + 2
```

Bytes of the fixed prefix: magic 4 + version 2 + payloadBytes 2.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:188`](../../include/shulib/calibration/calibration_record.hpp#L188).*

<a id="krecordbytes"></a>

## `kRecordBytes`

```cpp
inline constexpr std::size_t kRecordBytes = kRecordPrefixBytes + kRecordPayloadBytes + 4
```

The WHOLE record, in bytes: prefix + payload + the trailing u32 CRC. A file of any other length is refused by name before a single field is believed.

*constant, declared at [`include/shulib/calibration/calibration_record.hpp:192`](../../include/shulib/calibration/calibration_record.hpp#L192).*

<a id="crc32ieee"></a>

## `crc32Ieee`

```cpp
[[nodiscard]] inline std::uint32_t crc32Ieee(std::span<const std::byte> bytes) noexcept
```

CRC-32/IEEE 802.3 over `bytes` — FORWARDS to diag::crc32Ieee (diag/crc32.hpp), where the algorithm has lived since chunk H1 Part A1 so the SHUL/2 wire can share it; this spelling is kept so the record's encoder and decoder and every existing caller compile unchanged. The same number `zlib.crc32` and `crc32` produce (reflected polynomial 0xEDB88320, all-ones initial value, final inversion), computed bitwise with no lookup table.

*free function, declared at [`include/shulib/calibration/calibration_record.hpp:202`](../../include/shulib/calibration/calibration_record.hpp#L202).*

<a id="encoderecord"></a>

## `encodeRecord`

```cpp
[[nodiscard]] inline std::size_t encodeRecord(const hal::DrivetrainDescription& desc, std::span<std::byte> out) noexcept
```

Write `desc` into `out` as a v1 record. Returns the bytes written (kRecordBytes) or 0 if `out` is too small or the layout did not come out to exactly kRecordBytes — whole or nothing, so a short buffer can never leave a half-record that decodes.  It does NOT check desc.valid(): a caller may legitimately want to persist whatever the calibrator produced, and the DECODER is where the refusal lives (one gate, at the point where the bytes become a belief). The calibrate program checks valid() itself before it ever gets here, because a record nobody can boot from is not worth writing.

*free function, declared at [`include/shulib/calibration/calibration_record.hpp:240`](../../include/shulib/calibration/calibration_record.hpp#L240).*

<a id="decoderecord"></a>

## `decodeRecord`

```cpp
[[nodiscard]] inline bool decodeRecord(std::span<const std::byte> in, hal::DrivetrainDescription& desc, char* why = nullptr, std::size_t cap = 0) noexcept
```

Read a v1 record out of `in` into `desc`. Returns false — leaving `desc` untouched — and writes the CAUSE into `why` (optional; pass nullptr and 0 to skip) for any of the five refusals in the header, in that order: magic, version, length, CRC, then the decoded description's own valid(). Never throws, never allocates, never reads past `in`.  The ORDER is deliberate. Length before CRC, because a CRC over the wrong number of bytes says "corrupt" about a file that is really "from another layout", and the operator needs the second answer. valid() LAST, because a description that fails valid() with an intact CRC is a producer bug, not a card problem, and saying "CRC ok but the description contradicts itself" is what separates the two at 8 a.m. on a bench.

*free function, declared at [`include/shulib/calibration/calibration_record.hpp:289`](../../include/shulib/calibration/calibration_record.hpp#L289).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 61 lines, click to expand</summary>

```text

 The CALIBRATION RECORD — a hal::DrivetrainDescription as bytes on the SD card
 (chunk R3d Part 2, 2026-09-18).

 ── Why a record exists at all ──────────────────────────────────────────────────────
 The calibrator (Part 1) measures a drivetrain in about two minutes of pushing and
 spinning. Without somewhere to put the answer, every boot would have to repeat that,
 which means nobody would ever run it and the typed table would stay the truth. So the
 answer goes on the card, and the programs that drive robot two read it at boot ("shulib
 Drive", the competition program, the Auton Tester). One file, one format,
 one name (kRecordFileName): a second spelling is how a program reads a file another
 program never wrote.

 ── Refuse, never misread ───────────────────────────────────────────────────────────
 This record drives a robot. A flipped bit in a SIGN reverses a motor against its
 side-mates; a flipped bit in a SCALE makes every odometry inch a lie; a flipped bit in
 the exponent of the track width makes every commanded turn wrong. None of those is
 visible from the outside — the robot simply drives wrong, confidently. So the format is
 checked before it is believed, and every check names itself:
   1. MAGIC   — this is not our file at all
   2. VERSION — this is our file from a different layout
   3. LENGTH  — truncated, padded, or a different layout that kept the version
   4. CRC-32  — the bytes changed since they were written
   5. valid() — the bytes are intact and the description they spell is self-contradictory
 A decoder that guesses at any of those is worse than one that refuses: a refusal sends
 the operator to "run shulib Calibrate", a misread sends a robot into a wall.

 ── Why a CRC here, where the blackbox deliberately has none ────────────────────────
 diag/blackbox_format.hpp carries no checksum, on purpose: a blackbox is READ BY A HUMAN
 after the fact, one frame at a time, and a truncated tail is expected and decodable. The
 cost of a corrupt field there is a misleading number in an investigation. Here the file
 is read BY THE ROBOT, once, at boot, and every field is immediately load-bearing — so
 the trade flips, and 4 bytes buy the difference between "refuse and say so" and "drive
 wrong". CRC-32/IEEE (reflected polynomial 0xEDB88320), computed BITWISE with no lookup
 table: a 1 KiB table in a header that a composition root includes is 1 KiB of flash for
 180 bytes of input, and the bitwise loop runs in microseconds on a Cortex-A9. The
 implementation is the textbook one so that any other tool (`python3 -c "import zlib"`,
 `crc32` on the card's file) computes the same number — which is how the golden test
 derives its expected value WITHOUT running this code. Since chunk H1 Part A1 (2026-09-29)
 the algorithm lives in diag/crc32.hpp, because the SHUL/2 wire checks every frame with it
 and diag/ may not include calibration/; crc32Ieee below forwards there, so this spelling
 and every caller of it are unchanged.

 ── Fixed width, little-endian, DECLARATION ORDER, byte by byte ─────────────────────
 Never a struct memcpy: padding and ABI are not a file format, and a host test writing a
 record an ARM brain must read is exactly where that bites. The field order is
 hal::DrivetrainDescription's own declaration order, so a field APPENDED to the struct
 lands at the end here too and the diff is one line in each place. The layout table below
 is the document; test/calibration_record_test.cpp pins it with a BYTE-EXACT golden whose
 literal bytes were derived from that table by hand (and whose CRC came from an
 independent implementation), never captured from the encoder — because an encoder and a
 decoder that move together agree perfectly, and a round trip cannot see it (the E1
 lesson, mutation 1b).

 ON HARDWARE, ONCE. On 2026-09-25 a record "shulib Calibrate" wrote to robot two's card was
 read back and DECODED — magic, version, length, CRC and valid() — at all eight starts of
 "shulib Teleop" that evening, which booted from it. That is one card, one record, one night;
 it settles nothing about a torn write, a pulled card or a power cut (HA-139, HA-166). The
 golden bytes are still proven against the host suite and a real temp directory only.

 PROS-free, sim-free, allocation-free.
```

</details>
