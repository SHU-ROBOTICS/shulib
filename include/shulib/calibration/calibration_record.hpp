#pragma once
//
// The CALIBRATION RECORD — a hal::DrivetrainDescription as bytes on the SD card
// (chunk R3d Part 2, 2026-09-18).
//
// ── Why a record exists at all ──────────────────────────────────────────────────────
// The calibrator (Part 1) measures a drivetrain in about two minutes of pushing and
// spinning. Without somewhere to put the answer, every boot would have to repeat that,
// which means nobody would ever run it and the typed table would stay the truth. So the
// answer goes on the card, and "shulib Teleop" reads it at boot. One file, one format,
// one name (kRecordFileName): a second spelling is how a program reads a file another
// program never wrote.
//
// ── Refuse, never misread ───────────────────────────────────────────────────────────
// This record drives a robot. A flipped bit in a SIGN reverses a motor against its
// side-mates; a flipped bit in a SCALE makes every odometry inch a lie; a flipped bit in
// the exponent of the track width makes every commanded turn wrong. None of those is
// visible from the outside — the robot simply drives wrong, confidently. So the format is
// checked before it is believed, and every check names itself:
//   1. MAGIC   — this is not our file at all
//   2. VERSION — this is our file from a different layout
//   3. LENGTH  — truncated, padded, or a different layout that kept the version
//   4. CRC-32  — the bytes changed since they were written
//   5. valid() — the bytes are intact and the description they spell is self-contradictory
// A decoder that guesses at any of those is worse than one that refuses: a refusal sends
// the operator to "run shulib Calibrate", a misread sends a robot into a wall.
//
// ── Why a CRC here, where the blackbox deliberately has none ────────────────────────
// diag/blackbox_format.hpp carries no checksum, on purpose: a blackbox is READ BY A HUMAN
// after the fact, one frame at a time, and a truncated tail is expected and decodable. The
// cost of a corrupt field there is a misleading number in an investigation. Here the file
// is read BY THE ROBOT, once, at boot, and every field is immediately load-bearing — so
// the trade flips, and 4 bytes buy the difference between "refuse and say so" and "drive
// wrong". CRC-32/IEEE (reflected polynomial 0xEDB88320), computed BITWISE with no lookup
// table: a 1 KiB table in a header that a composition root includes is 1 KiB of flash for
// 180 bytes of input, and the bitwise loop runs in microseconds on a Cortex-A9. The
// implementation is the textbook one so that any other tool (`python3 -c "import zlib"`,
// `crc32` on the card's file) computes the same number — which is how the golden test
// derives its expected value WITHOUT running this code. Since chunk H1 Part A1 (2026-09-29)
// the algorithm lives in diag/crc32.hpp, because the SHUL/2 wire checks every frame with it
// and diag/ may not include calibration/; crc32Ieee below forwards there, so this spelling
// and every caller of it are unchanged.
//
// ── Fixed width, little-endian, DECLARATION ORDER, byte by byte ─────────────────────
// Never a struct memcpy: padding and ABI are not a file format, and a host test writing a
// record an ARM brain must read is exactly where that bites. The field order is
// hal::DrivetrainDescription's own declaration order, so a field APPENDED to the struct
// lands at the end here too and the diff is one line in each place. The layout table below
// is the document; test/calibration_record_test.cpp pins it with a BYTE-EXACT golden whose
// literal bytes were derived from that table by hand (and whose CRC came from an
// independent implementation), never captured from the encoder — because an encoder and a
// decoder that move together agree perfectly, and a round trip cannot see it (the E1
// lesson, mutation 1b).
//
// ON HARDWARE, ONCE. On 2026-09-25 a record "shulib Calibrate" wrote to robot two's card was
// read back and DECODED — magic, version, length, CRC and valid() — at all eight starts of
// "shulib Teleop" that evening, which booted from it. That is one card, one record, one night;
// it settles nothing about a torn write, a pulled card or a power cut (HA-139, HA-166). The
// golden bytes are still proven against the host suite and a real temp directory only.
//
// PROS-free, sim-free, allocation-free.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/crc32.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/file_store.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

/// The four magic bytes every calibration record starts with ("SHulib Drivetrain
/// Calibration"). Distinct from the blackbox's "SHBB" so a file pointed at the wrong
/// decoder is refused at byte 0 instead of being parsed into nonsense.
inline constexpr char kRecordMagic[4] = {'S', 'H', 'D', 'C'};

/// On-card format version. BUMP THIS whenever the layout below changes — a decoder refuses a
/// version it was not built for rather than misreading it. Append-only reasoning applies to
/// the enums the payload carries as bytes (hal::DrivetrainKind, hal::MotorCartridge,
/// hal::DescriptionSource all say so at their declarations).
inline constexpr std::uint16_t kRecordVersion = 1;

/// The ONE file name both programs use, so "shulib Calibrate" writes what "shulib Teleop"
/// reads. BARE — the /usd/ prefix belongs to hal/pros/block_sink.hpp and block_source.hpp,
/// which own it in one place each (HA-122).
inline constexpr const char* kRecordFileName = "shulib_drivetrain.bin";

/// The record's BACKUP copy (chunk SAFE-SAVE Part A): the same 180 bytes, in a second file, so
/// that a save which fails part-way does not leave the card without a good record — given the
/// two beliefs about the card it rests on (HA-166 (4) and (5)), and short of the two-fault limit
/// calibration/record_store.hpp states. The brain cannot rename or delete a file, and "wb" is
/// taken to empty one at the open (HA-166), so the safe save copies the old record HERE and reads
/// it back before it opens the record file for writing, and a load that finds the record file
/// damaged falls back to it. Named beside kRecordFileName so both programs share ONE spelling of
/// each. BARE, like it.
inline constexpr const char* kRecordBackupFileName = "shulib_drivetrain.bak";

namespace detail {
/// True when `name` is non-empty and holds no '/' — a BARE name the adapters accept.
[[nodiscard]] constexpr bool isBareFileName(const char* name) noexcept {
    if (name == nullptr || name[0] == '\0') {
        return false;
    }
    for (const char* c = name; *c != '\0'; ++c) {
        if (*c == '/') {
            return false;
        }
    }
    return true;
}
}  // namespace detail

static_assert(detail::isBareFileName(kRecordFileName) &&
                  detail::isBareFileName(kRecordBackupFileName),
              "both record file names must be BARE -- the adapters own the /usd/ prefix (HA-122)");
static_assert(!hal::sameFileOnCard(kRecordFileName, kRecordBackupFileName),
              "the record and its backup must be two DIFFERENT files on the card (FAT ignores "
              "letter case and trailing dots and spaces) -- or the backup IS the record");

/// Width of the description's provenance field, in bytes — taken from the struct rather than
/// retyped, so a widening there is a compile error here instead of a silently short record.
inline constexpr std::size_t kProvenanceBytes = sizeof(hal::DrivetrainDescription::provenance);

/// Bytes of the payload (everything between the length field and the CRC), v1:
/// kind 1 + cartridge 1 + imuPort 1 + left (21 + 1 + 8) + right (21 + 1 + 8) + trackWidth 8
/// + source 1 + provenance 96 = 168. The encoder asserts it wrote exactly this many.
inline constexpr std::size_t kRecordPayloadBytes =
    1 + 1 + 1 + (hal::kMaxSmartPorts + 1 + 8) * 2 + 8 + 1 + kProvenanceBytes;

/// Bytes of the fixed prefix: magic 4 + version 2 + payloadBytes 2.
inline constexpr std::size_t kRecordPrefixBytes = 4 + 2 + 2;

/// The WHOLE record, in bytes: prefix + payload + the trailing u32 CRC. A file of any other
/// length is refused by name before a single field is believed.
inline constexpr std::size_t kRecordBytes = kRecordPrefixBytes + kRecordPayloadBytes + 4;

static_assert(kRecordPayloadBytes == 168, "the v1 payload is 168 bytes — see the layout table");
static_assert(kRecordBytes == 180, "the v1 record is 180 bytes — see the layout table");

/// CRC-32/IEEE 802.3 over `bytes` — FORWARDS to diag::crc32Ieee (diag/crc32.hpp), where the
/// algorithm has lived since chunk H1 Part A1 so the SHUL/2 wire can share it; this spelling
/// is kept so the record's encoder and decoder and every existing caller compile unchanged.
/// The same number `zlib.crc32` and `crc32` produce (reflected polynomial 0xEDB88320,
/// all-ones initial value, final inversion), computed bitwise with no lookup table.
[[nodiscard]] inline std::uint32_t crc32Ieee(std::span<const std::byte> bytes) noexcept {
    return diag::crc32Ieee(bytes);
}

// ── THE LAYOUT, v1 — this table IS the document ─────────────────────────────────────
//   off  size  field
//     0     4  magic "SHDC"
//     4     2  version (u16 LE) = 1
//     6     2  payloadBytes (u16 LE) = 168
//   ── payload, hal::DrivetrainDescription in DECLARATION order ──
//     8     1  kind (u8)                     hal::DrivetrainKind
//     9     1  cartridge (u8)                hal::MotorCartridge (0 = UNSET, refused by valid())
//    10     1  imuPort (u8)                  1..21; 0 = UNSET, refused by valid()
//    11    21  left.signedPorts[21] (i8 each, two's complement — the SIGN LIVES IN THE PORT)
//    32     1  left.count (u8)
//    33     8  left.inchesPerRadian (f64 LE, IEEE-754 binary64)
//    41    21  right.signedPorts[21] (i8 each)
//    62     1  right.count (u8)
//    63     8  right.inchesPerRadian (f64 LE)
//    71     8  trackWidth (f64 LE)
//    79     1  source (u8)                   hal::DescriptionSource
//    80    96  provenance (NUL-padded text)
//   ── end of payload (176) ──
//   176     4  CRC-32 (u32 LE) over bytes 0..175
//   180 = kRecordBytes
//
// The f64 fields keep their BIT PATTERN (diag::blackbox::ByteWriter::f64), so a NaN or an
// infinity written by a broken producer survives the trip exactly and is refused by valid()
// rather than rounded into something plausible.

/// Write `desc` into `out` as a v1 record. Returns the bytes written (kRecordBytes) or 0 if
/// `out` is too small or the layout did not come out to exactly kRecordBytes — whole or
/// nothing, so a short buffer can never leave a half-record that decodes.
///
/// It does NOT check desc.valid(): a caller may legitimately want to persist whatever the
/// calibrator produced, and the DECODER is where the refusal lives (one gate, at the point
/// where the bytes become a belief). The calibrate program checks valid() itself before it
/// ever gets here, because a record nobody can boot from is not worth writing.
[[nodiscard]] inline std::size_t encodeRecord(const hal::DrivetrainDescription& desc,
                                              std::span<std::byte> out) noexcept {
    if (out.size() < kRecordBytes) {
        return 0U;
    }
    diag::blackbox::ByteWriter w{out};
    for (const char c : kRecordMagic) {
        w.u8(static_cast<std::uint8_t>(c));
    }
    w.u16(kRecordVersion);
    w.u16(static_cast<std::uint16_t>(kRecordPayloadBytes));
    w.u8(static_cast<std::uint8_t>(desc.kind));
    w.u8(static_cast<std::uint8_t>(desc.cartridge));
    w.u8(desc.imuPort);
    const hal::DriveSideDescription* sides[2] = {&desc.left, &desc.right};
    for (const hal::DriveSideDescription* side : sides) {
        for (std::size_t i = 0; i < hal::kMaxSmartPorts; ++i) {
            w.u8(static_cast<std::uint8_t>(side->signedPorts[i]));
        }
        w.u8(side->count);
        w.f64(side->inchesPerRadian.value());
    }
    w.f64(desc.trackWidth.value());
    w.u8(static_cast<std::uint8_t>(desc.source));
    // BOUNDED length, not std::strlen: a description built by hand (or decoded from a record
    // whose provenance filled the field) need not be NUL-terminated, and strlen would walk off
    // the struct. The field is fixed width; so is the read of it.
    std::size_t provLen = 0;
    while (provLen < kProvenanceBytes && desc.provenance[provLen] != '\0') {
        ++provLen;
    }
    w.text(std::string_view{desc.provenance, provLen}, kProvenanceBytes);
    if (!w.ok() || w.offset() != kRecordPrefixBytes + kRecordPayloadBytes) {
        return 0U;
    }
    w.u32(crc32Ieee(out.subspan(0, kRecordPrefixBytes + kRecordPayloadBytes)));
    return w.ok() && w.offset() == kRecordBytes ? w.offset() : 0U;
}

/// Read a v1 record out of `in` into `desc`. Returns false — leaving `desc` untouched — and
/// writes the CAUSE into `why` (optional; pass nullptr and 0 to skip) for any of the five
/// refusals in the header, in that order: magic, version, length, CRC, then the decoded
/// description's own valid(). Never throws, never allocates, never reads past `in`.
///
/// The ORDER is deliberate. Length before CRC, because a CRC over the wrong number of bytes
/// says "corrupt" about a file that is really "from another layout", and the operator needs
/// the second answer. valid() LAST, because a description that fails valid() with an intact
/// CRC is a producer bug, not a card problem, and saying "CRC ok but the description
/// contradicts itself" is what separates the two at 8 a.m. on a bench.
[[nodiscard]] inline bool decodeRecord(std::span<const std::byte> in,
                                       hal::DrivetrainDescription& desc, char* why = nullptr,
                                       std::size_t cap = 0) noexcept {
    if (why != nullptr && cap > 0) {
        why[0] = '\0';
    }
    if (in.size() < kRecordPrefixBytes) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "record is %d bytes -- too short to carry even a header (%d)",
                          static_cast<int>(in.size()), static_cast<int>(kRecordPrefixBytes));
        }
        return false;
    }
    diag::blackbox::ByteReader r{in};
    for (const char c : kRecordMagic) {
        if (r.u8() != static_cast<std::uint8_t>(c)) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "magic is not \"SHDC\" -- this is not a calibration record");
            }
            return false;
        }
    }
    const std::uint16_t version = r.u16();
    if (version != kRecordVersion) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap,
                          "record version %d -- this build reads version %d only; recalibrate",
                          static_cast<int>(version), static_cast<int>(kRecordVersion));
        }
        return false;
    }
    const std::uint16_t payloadBytes = r.u16();
    if (payloadBytes != static_cast<std::uint16_t>(kRecordPayloadBytes)) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "record declares a %d-byte payload; version %d is %d bytes",
                          static_cast<int>(payloadBytes), static_cast<int>(kRecordVersion),
                          static_cast<int>(kRecordPayloadBytes));
        }
        return false;
    }
    if (in.size() != kRecordBytes) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap,
                          "record is %d bytes; version %d is exactly %d -- truncated or padded",
                          static_cast<int>(in.size()), static_cast<int>(kRecordVersion),
                          static_cast<int>(kRecordBytes));
        }
        return false;
    }
    const std::uint32_t want =
        crc32Ieee(in.subspan(0, kRecordPrefixBytes + kRecordPayloadBytes));
    diag::blackbox::ByteReader crcReader{in.subspan(kRecordPrefixBytes + kRecordPayloadBytes, 4)};
    const std::uint32_t stored = crcReader.u32();
    if (stored != want) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap,
                          "CRC-32 mismatch: stored 0x%08lX, computed 0x%08lX -- the bytes changed "
                          "since they were written; recalibrate",
                          static_cast<unsigned long>(stored), static_cast<unsigned long>(want));
        }
        return false;
    }
    hal::DrivetrainDescription decoded{};
    decoded.kind = static_cast<hal::DrivetrainKind>(r.u8());
    decoded.cartridge = static_cast<hal::MotorCartridge>(r.u8());
    decoded.imuPort = r.u8();
    hal::DriveSideDescription* sides[2] = {&decoded.left, &decoded.right};
    for (hal::DriveSideDescription* side : sides) {
        for (std::size_t i = 0; i < hal::kMaxSmartPorts; ++i) {
            side->signedPorts[i] = static_cast<std::int8_t>(r.u8());
        }
        side->count = r.u8();
        side->inchesPerRadian = units::Length{r.f64()};
    }
    decoded.trackWidth = units::Length{r.f64()};
    decoded.source = static_cast<hal::DescriptionSource>(r.u8());
    r.text(decoded.provenance, kProvenanceBytes, kProvenanceBytes);
    if (!r.ok() || r.offset() != kRecordPrefixBytes + kRecordPayloadBytes) {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "record payload ended early -- truncated");
        }
        return false;
    }
    char inner[160];
    if (!decoded.valid(inner, sizeof inner)) {
        if (why != nullptr && cap > 0) {
            // %.100s, not %s: the inner reason is up to 159 bytes and the 79-byte prefix leaves
            // less than that in a caller's buffer, which -Wformat-truncation reports as a
            // DISALLOWED warning in the src build gate (it caught this on the first six-build
            // run). A bounded precision makes the truncation a decision rather than a warning —
            // and valid()'s reasons put the field name first, so 100 bytes keeps the part that
            // names the problem.
            std::snprintf(why, cap, "record is intact (CRC ok) but the description it spells is "
                                    "self-contradictory: %.100s",
                          inner);
        }
        return false;
    }
    desc = decoded;
    return true;
}

}  // namespace shulib::calibration
