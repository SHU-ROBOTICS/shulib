#pragma once
//
// SHUL/2 — the telemetry WIRE, format v1.0: DebugRecord streamed, sequenced, checked and
// versioned (master plan §18.1; diagnostics-plan; chunk H1 Part A1, 2026-09-29). This header
// is the format: the envelope, the five frame types, their layouts, the version rules, the
// encoders and decoders, and scanFrame() — the one function every reader is built on. Its
// reader over a whole capture is diag/shul2_reader.hpp. Its sink (Shul2Sink) is H1 Part A2's
// and does not exist yet; nothing in src/ writes this wire.
//
// NO HARDWARE CLAIM. These bytes have never crossed a real link — no USB port, no socket, no
// radio. Every byte here is a host byte, proven by the host suite. What a V5 brain's stdout
// does to them is a Hardware Assumption the wire's sink (H1 Part A2) will register — not
// something this header knows.
//
// NOT FROZEN YET. Register row F9 freezes this wire at H1 Part B; until then every layout
// below may still move. After it, the rules in "What every major keeps" and the 1.0 layouts
// are the contract, and growth is by appending (the version rules below).
//
// ── Binary, not JSON (decision D1) ──────────────────────────────────────────────────
// Earlier documents called SHUL/2 "the JSON wire". It is binary: little-endian, fixed-width
// fields, every floating-point value IEEE-754 binary64 with no narrowing — the blackbox's rules
// (blackbox_format.hpp). JSON cannot carry NaN or Inf at all; a double survives decimal text
// only with 17 significant digits and a correct parser on both ends; and it would bring heap
// and formatting cost onto the brain, every tick. Every field is written byte by byte through
// blackbox::ByteWriter / ByteReader — never a struct memcpy (padding and ABI are not a format).
//
// ── The Tick and Summary payloads ARE the blackbox's bytes (D2) ─────────────────────
// A Tick payload is blackbox::encodeTick's 428 bytes and a Summary payload is
// blackbox::encodeSummary's 168, and this header has NO encoder of its own for either. One
// field order by construction: a Tick read off the SD card can be re-wrapped as a SHUL/2 frame
// without decoding it — the premise of a replay in VexBuilder. Two encoders that must agree
// are the blindness E1's mutation 1b measured; one encoder cannot disagree with itself. The
// cost, stated: once F9 freezes, those two layouts are frozen with it (blackbox_format.hpp
// says so), and a minor that GROWS the tick must give the tick decoder an older-minor path.
//
// ── The envelope: 14 bytes per frame, the same in EVERY major version ──────────────
//    off  size  field
//      0     1  sync0 = 0xFE        (0xFE never occurs in valid UTF-8 text)
//      1     1  sync1 = 0x53        ('S')
//      2     1  type                (FrameType; 0 is never assigned)
//      3     1  flags               (0 in major 1; readers IGNORE unknown bits)
//      4     4  sequence (u32)      per writer, per session; see "Sequence"
//      8     2  payloadBytes (u16)  at most kMaxPayloadBytes = 4096
//     10     n  payload
//   10+n     4  CRC-32/IEEE (u32)   diag::crc32Ieee over bytes [2, 10+n): type, flags,
//                                   sequence, length and payload — everything but the sync
// The check is verified BEFORE a single byte inside the frame is believed (D3). Resync rule:
// after a failed candidate — no sync, a length above 4096, a CRC that does not verify —
// scanning resumes at the NEXT byte, never at the claimed end: a corrupted length would
// otherwise throw away every good frame it claimed. The cost is bounded: at most 4104 bytes of
// CRC per candidate position. Stated for a reader's budget, because it is per POSITION: bytes
// crafted so that sync pairs claiming a length the rule accepts sit as densely as possible cost
// about 1,200 bytes of bitwise CRC per input byte. (A candidate at p puts 0x53 at p + 1 and needs
// a length byte of at most 0x10 at p + 9, so neither p + 1, p + 8 nor p + 9 can start another;
// no byte pattern then holds more than three candidates per ten bytes on average.) Measured on a
// desktop host (g++ -O2, 2026-09-30): about 14 s per MB of that densest pattern with the bitwise
// CRC running at ~98 MB/s, and about 23 s per MB while other builds loaded the host (the CRC at
// ~55 MB/s). Ordinary damage and random bytes are cheap: a random sync pair occurs about once in
// 65,536 positions, and a megabyte of random bytes reads in under a millisecond. A host-side
// reader that must bound its time may use a table-driven CRC-32 (the same values); the bitwise one
// is the brain's choice, for flash (diag/crc32.hpp).
//
// ── Frame types, and one writer per capture (D4, D5) ────────────────────────────────
//   Hello = 1, Tick = 2, Log = 3, Summary = 4, End = 5.
// Hello and End are SESSION frames and travel in either direction (a peer's Hello carries
// role 1). The ranges govern DATA frames: 2-63 runtime → peer, 64-127 peer → runtime (H2's
// simulated sensor frames, and any later peer → runtime data), 128-255 reserved.
// A verified frame of a type this build does not assign — 0 included — is skipped by its
// length and counted. ONE capture (or one direction of a live link) carries ONE writer's bytes:
// a recorder that keeps both directions keeps two captures, because two writers interleaved
// into one byte stream break the sequence rules below.
// v1 does NOT carry: per-motion results or markers as data (both ride log() as text today, so
// they reach the wire inside Log frames, verbatim — an additive data type later), inbound
// frames (H2's, in the peer range), or a Triage frame (a stream carries every tick, including
// the one that faulted).
//
// ── The time base ───────────────────────────────────────────────────────────────────
// Every time on the wire — the Tick's `t`, the Log's `t`, the End's `endTime` — is the injected
// clock's RAW reading in seconds: the base the motion scheduler stamps DebugRecord::t in and
// TermSink stamps its lines in. The Hello's `epochSeconds` is the same clock's reading when the
// session opened; subtract it for session-relative time. (The blackbox does the same.)
//
// ── The Hello (v1.0: 256 bytes) ─────────────────────────────────────────────────────
//    off  size  field                         off  size  field
//      0     8  signature 89 53 48 55 4C 00 0D 0A    32     1  apiMajor (shulib::kApiMajor)
//      8     2  versionMajor = 1               33     1  apiMinor (shulib::kApiMinor)
//     10     2  versionMinor = 0               34     2  reserved (0)
//     12     1  role (0 runtime, 1 peer)       36    48  buildHash  (text, NUL-padded)
//     13     1  reserved (0)                   84    32  routineId
//     14     2  flags (0)                     116    16  alliance
//     16     2  tickPayloadBytes = 428        132    16  side
//     18     2  summaryPayloadBytes = 168     148    96  portMap
//     20     2  endPayloadBytes = 32          244    12  reserved (0)
//     22     2  logFixedBytes = 14            256  = end
//     24     8  epochSeconds (f64)
// Bytes 0-11 — the signature and the version — are the same in every major, and so is the
// Hello's type (1). Bytes 12 onward belong to their major. Text fields end at their first NUL
// (or fill their width); an EMPTY build hash stays empty, because MISSING must stay loud.
// Reserved bytes and flag bits are written 0 and NEVER judged by a reader, so a later minor may
// give them only a meaning that is safe to ignore.
//
// What the signature does, and what it cannot do. Its high-bit byte, its NUL and its CR LF
// guarantee that a transport which strips bit 7, truncates at a NUL, or translates newlines
// (either way) corrupts EVERY Hello — which matters most for newline translation, the one
// transform that can leave a Tick intact (a Tick holding no 0x0A or 0x0D byte), so no session
// ever starts on such a link. But the CRC catches that corruption BEFORE the signature is
// compared, so a reader reports such a capture as NoFrame or NoSession (shul2_reader.hpp),
// never as BadSignature. BadSignature means a Hello whose CRC VERIFIED and whose signature is
// wrong: a writer bug, or a stream that is not SHUL/2 inside a SHUL-shaped envelope.
//
// ── The Tick (428 bytes) and the Summary (168 bytes) ────────────────────────────────
// Laid out in blackbox_format.hpp ("The tick record", "The summary frame"). Canonical units:
// inches, radians (headings wrapped), seconds, volts, amperes, in/s, rad/s. The small
// vocabularies travel as their numbers — fault (u16 FaultCode), gateReason (u8 GateReason),
// activeCommandState (u8 motion::MotionState), qualityClass (u8, the Localizer::Quality mirror
// 0-3) — and the tick flags byte is bit 0 clampedThisTick, bit 1 strafeFallbackActive, bit 2
// tankSteerActive (TANK1), bits 3-7 written 0 and ignored. A value this build does not name
// decodes as its number and is never refused. The Summary's `blackboxDropped` field carries the
// summary's OWN figure (RunSummary::blackboxDropped): the wire never overwrites another sink's
// count, and its own losses ride the End.
//
// ── The Log (14 bytes, then the text) ───────────────────────────────────────────────
//    off  size  field
//      0     1  level (wire code: Error 0, Warn 1, Info 2, Debug 3, Trace 4)
//      1     1  flags (bit 0 tag truncated, bit 1 message truncated; bits 2-7 ignored)
//      2     1  tagBytes
//      3     1  reserved (0)
//      4     2  messageBytes (u16)
//      6     8  t (f64, the clock's raw reading when log() was called)
//     14        tag, then message — VERBATIM, never sanitized, never NUL-terminated
// The text always starts at offset 14; fields a later minor appends go AFTER the text.
// `logFixedBytes` (the Hello) means every Log byte that is not text: payloadBytes - tagBytes -
// messageBytes. 14 in 1.0; a minor appending k bytes after the text declares 14 + k. The level
// codes map to hal::LogLevel by a switch in both directions, never by a cast (LogLevel declares
// no explicit values); an unknown code decodes as its raw number and maps to "unknown".
//
// ── The End (32 bytes) ──────────────────────────────────────────────────────────────
//    off  size  field                     off  size  field
//      0     4  framesOffered (= the End's   16     4  linesTruncated
//               own sequence number)         20     4  notOpenDropped
//      4     4  framesFailed (device refused) 24    8  endTime (f64, clock's raw reading)
//      8     4  recordsSeen                  32  = end
//     12     4  linesSeen
//
// ── Versions (D4, D8) ───────────────────────────────────────────────────────────────
// What every major keeps: the envelope (sync bytes, offsets and widths, the 4096 bound, the
// CRC and what it covers), the Hello's type (1) and the Hello's payload bytes 0-11. Without
// that, a 1.x reader could not even NAME a 2.0 stream as refused. Everything else may change
// at a major. negotiate() then judges a stream's Hello against the reader's profile:
//   * another MAJOR is refused whole;
//   * a NEWER minor is ACCEPTED: the reader decodes the prefix of each payload it knows, and
//     ignores and counts the appended bytes — so every frame type grows only by APPENDING;
//   * an OLDER minor is accepted when its declared widths equal THAT minor's row in the
//     reader's width table (kWidthTable, one row per minor, append-only), and the fields it
//     lacks take their documented defaults;
//   * at the SAME minor, any width that differs is refused — the blackbox's width cross-check,
//     kept where it belongs: it catches a changed layout whose version was never bumped.
// The blackbox's decoders take EXACTLY their width, so the day a minor grows the tick, a reader
// built on them needs an older-minor decode path; H1 Part B pins that with a permanent test.
//
// ── What changed shape here, and why (rejected alternatives) ────────────────────────
//   * No check on each frame (the blackbox's choice) — rejected: a wire over a lossy link must
//     add its own (blackbox_reader.hpp says so), and after a freeze a check could only arrive
//     with a major bump.
//   * COBS byte-stuffing — rejected: PROS already COBS-frames stdout, a local socket does not
//     need it, and sync + length + CRC resynchronises within a bounded number of bytes.
//   * The blackbox's refuse-any-other-width rule — rejected for the wire: VexBuilder's contract
//     promises that unknown newer fields are ignored, never fatal.
//
// PROS-free, sim-free, allocation-free, never throws. A leaf of diag/: it includes only diag/,
// the hal/ telemetry seam, and shulib/version.hpp.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/crc32.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/version.hpp"

namespace shulib::diag::shul2 {

// ── The envelope ────────────────────────────────────────────────────────────────────

/// First sync byte of every frame. 0xFE never occurs in valid UTF-8, so text a writer
/// carries cannot open a false frame inside a Log's own bytes as easily as ASCII would.
inline constexpr std::uint8_t kSync0 = 0xFE;
/// Second sync byte of every frame ('S').
inline constexpr std::uint8_t kSync1 = 0x53;
/// Offset of the type byte in a frame.
inline constexpr std::size_t kTypeOffset = 2;
/// Offset of the flags byte in a frame.
inline constexpr std::size_t kFlagsOffset = 3;
/// Offset of the u32 sequence number in a frame.
inline constexpr std::size_t kSequenceOffset = 4;
/// Offset of the u16 payloadBytes field in a frame.
inline constexpr std::size_t kPayloadBytesOffset = 8;
/// Offset of the payload in a frame — equally, the envelope's bytes BEFORE the payload.
inline constexpr std::size_t kPayloadOffset = 10;
/// Width of the trailing CRC-32.
inline constexpr std::size_t kCrcBytes = 4;
/// Envelope overhead per frame: the 10 bytes before the payload plus the 4-byte CRC.
inline constexpr std::size_t kFrameOverheadBytes = kPayloadOffset + kCrcBytes;
/// The largest payload any frame may declare. A declared length above it is not a frame
/// (scanFrame resyncs one byte on), which also bounds a live reader's buffer.
inline constexpr std::size_t kMaxPayloadBytes = 4096;
/// The largest whole frame, envelope included: what a live reader must be able to buffer.
inline constexpr std::size_t kMaxFrameBytes = kFrameOverheadBytes + kMaxPayloadBytes;

// ── Versions and frame types ────────────────────────────────────────────────────────

/// The wire's MAJOR version. A reader refuses any other major whole (header, "Versions").
/// Independent of shulib::kApiMajor: the wire carries its own version in every Hello.
inline constexpr std::uint16_t kVersionMajor = 1;
/// The wire's MINOR version: the newest row of kWidthTable. A minor grows frames only by
/// appending, so every reader of this major can still read it.
inline constexpr std::uint16_t kVersionMinor = 0;

/// What a frame carries. WIRE values: explicit, never renumbered. Hello and End are session
/// frames (either direction); the rest are data frames in the runtime → peer range.
enum class FrameType : std::uint8_t {
    Hello = 1,    ///< opens a session: signature, version, widths, provenance (256 bytes in 1.0)
    Tick = 2,     ///< one DebugRecord: blackbox::encodeTick's 428 bytes
    Log = 3,      ///< one log() line: 14 fixed bytes, then the tag and the message
    Summary = 4,  ///< one RunSummary: blackbox::encodeSummary's 168 bytes
    End = 5,      ///< closes a session with the writer's own account (32 bytes in 1.0)
};

/// First DATA frame type a runtime sends to a peer (types 2-63).
inline constexpr std::uint8_t kFirstRuntimeDataType = 2;
/// Last DATA frame type a runtime sends to a peer.
inline constexpr std::uint8_t kLastRuntimeDataType = 63;
/// First DATA frame type a peer sends to a runtime (64-127: H2's simulated sensor frames).
inline constexpr std::uint8_t kFirstPeerDataType = 64;
/// Last DATA frame type a peer sends to a runtime; 128-255 are reserved.
inline constexpr std::uint8_t kLastPeerDataType = 127;

/// The Hello's `role` for the runtime that emits records (the robot, or a program on the host).
inline constexpr std::uint8_t kRoleRuntime = 0;
/// The Hello's `role` for a peer (H2's VexBuilder side). Other values are reserved; a reader
/// accepts and reports any role, and delivers frames by type, never by role.
inline constexpr std::uint8_t kRolePeer = 1;

/// The Hello's first 8 payload bytes: 0x89 "SHUL" NUL CR LF. The high-bit byte, the NUL and
/// the CR LF make a transport that strips bit 7, truncates at a NUL or translates newlines
/// corrupt every Hello (header, "What the signature does").
inline constexpr std::uint8_t kSignature[8] = {0x89, 0x53, 0x48, 0x55, 0x4C, 0x00, 0x0D, 0x0A};

/// The Hello's invariant prefix: the signature and the two version fields, the same in every
/// major. decodeHello() reads these from any Hello of at least this many bytes.
inline constexpr std::size_t kHelloInvariantBytes = 12;
/// Payload width of a v1.0 Hello.
inline constexpr std::size_t kHelloPayloadBytes = 256;
/// Bytes of a v1.0 Log that are not text (the fixed part before the tag).
inline constexpr std::size_t kLogFixedBytes = 14;
/// Payload width of a v1.0 End.
inline constexpr std::size_t kEndPayloadBytes = 32;

/// Log flags bit 0: the writer cut the tag to its cap.
inline constexpr std::uint8_t kLogTagTruncated = 0x01;
/// Log flags bit 1: the writer cut the message to its cap.
inline constexpr std::uint8_t kLogMessageTruncated = 0x02;

/// One minor version's payload widths, in bytes: a row of kWidthTable, and what a Hello
/// declares about the frames that follow it.
struct WidthRow {
    std::uint16_t hello = 0;     ///< the Hello's own payload width
    std::uint16_t tick = 0;      ///< every Tick payload
    std::uint16_t summary = 0;   ///< every Summary payload
    std::uint16_t end = 0;       ///< the End payload
    std::uint16_t logFixed = 0;  ///< every Log byte that is not text
};

/// The width table: row i describes minor i, append-only. Row 1.0 is {256, 428, 168, 32, 14}.
inline constexpr WidthRow kWidthTable[] = {{256, 428, 168, 32, 14}};

static_assert(sizeof(kWidthTable) / sizeof(kWidthTable[0]) == std::size_t{kVersionMinor} + 1U,
              "SHUL/2: kWidthTable needs exactly one row per minor, 0..kVersionMinor");
static_assert(blackbox::kTickPayloadBytes == kWidthTable[kVersionMinor].tick,
              "SHUL/2: blackbox::encodeTick changed width — the wire's Tick IS its bytes, so "
              "bump kVersionMinor and append a width row (and give the reader an older-minor path)");
static_assert(blackbox::kSummaryPayloadBytes == kWidthTable[kVersionMinor].summary,
              "SHUL/2: blackbox::encodeSummary changed width — bump kVersionMinor and append a row");
static_assert(kHelloPayloadBytes == kWidthTable[kVersionMinor].hello
                  && kEndPayloadBytes == kWidthTable[kVersionMinor].end
                  && kLogFixedBytes == kWidthTable[kVersionMinor].logFixed,
              "SHUL/2: this header's layout constants and its width row disagree");

static_assert(shulib::kApiMajor >= 0 && shulib::kApiMajor <= 0xFF,
              "SHUL/2: the Hello carries shulib::kApiMajor in ONE byte");
static_assert(shulib::kApiMinor >= 0 && shulib::kApiMinor <= 0xFF,
              "SHUL/2: the Hello carries shulib::kApiMinor in ONE byte");
/// shulib::kApiMajor as the Hello's one byte carries it (range-checked at compile time).
inline constexpr std::uint8_t kApiMajorByte = static_cast<std::uint8_t>(shulib::kApiMajor);
/// shulib::kApiMinor as the Hello's one byte carries it (range-checked at compile time).
inline constexpr std::uint8_t kApiMinorByte = static_cast<std::uint8_t>(shulib::kApiMinor);

// ── The reader's profile and the negotiation ────────────────────────────────────────

/// What a READER knows: the major it reads and one width row per minor it knows. A
/// parameter rather than a constant so a test can stand up a hypothetical newer reader with
/// its own row; kThisReader is this build's. The rows are borrowed and must outlive every use.
/// Its major is kVersionMajor in practice: decodeHello() reads a Hello's bytes 12 onward only
/// for this build's major (they belong to their major), so a profile of another major can name
/// a stream but never accept one.
struct ReaderProfile {
    std::uint16_t versionMajor = kVersionMajor;  ///< the only major this reader accepts
    std::span<const WidthRow> rows{};            ///< row i = minor i; the last row is the newest
};

/// This build's reader: major kVersionMajor, the rows of kWidthTable.
inline constexpr ReaderProfile kThisReader{kVersionMajor, std::span<const WidthRow>{kWidthTable}};

/// negotiate()'s verdict on one Hello: exactly one, checked in the order listed below.
enum class Verdict : std::uint8_t {
    Exact = 0,           ///< same major and minor, every width equal to the reader's row
    OlderMinor = 1,      ///< an older minor whose widths equal THAT minor's row
    NewerMinor = 2,      ///< a newer minor; every width at least the reader's newest row
    BadSignature = 3,    ///< fewer than 12 Hello bytes, or a signature that differs
    RefusedMajor = 4,    ///< a major other than the reader's — refused whole
    LayoutMismatch = 5,  ///< a short Hello, or a width that breaks the rules above
};

/// True for the three verdicts whose sessions a reader delivers (Exact, OlderMinor,
/// NewerMinor); false for the three that refuse the session.
[[nodiscard]] constexpr bool accepted(Verdict v) noexcept {
    return v == Verdict::Exact || v == Verdict::OlderMinor || v == Verdict::NewerMinor;
}

/// The §18.5 spelling of a Verdict, for messages. Never returns null.
[[nodiscard]] constexpr const char* verdictName(Verdict v) noexcept {
    switch (v) {
        case Verdict::Exact: return "EXACT";
        case Verdict::OlderMinor: return "OLDER_MINOR";
        case Verdict::NewerMinor: return "NEWER_MINOR";
        case Verdict::BadSignature: return "BAD_SIGNATURE";
        case Verdict::RefusedMajor: return "REFUSED_MAJOR";
        case Verdict::LayoutMismatch: return "LAYOUT_MISMATCH";
    }
    return "UNKNOWN";
}

/// A decoded Hello. Its text fields are VIEWS into the caller's payload bytes, valid while
/// those bytes are; each ends at its field's first NUL (or fills the field). Everything past
/// byte 11 is meaningful only when `complete` is true — and for another major it never is:
/// those bytes belong to that major, so they are left at their defaults, never read with this
/// major's layout.
struct Hello {
    std::size_t payloadBytes = 0;          ///< the Hello's payload length as it arrived
    std::array<std::uint8_t, 8> signature{};  ///< payload bytes 0-7 as they arrived
    std::uint16_t versionMajor = 0;        ///< payload bytes 8-9
    std::uint16_t versionMinor = 0;        ///< payload bytes 10-11
    /// The payload held at least the 1.0 Hello's 256 bytes AND its major is kVersionMajor, so
    /// the fields below were decoded.
    bool complete = false;
    std::uint8_t role = 0;                 ///< kRoleRuntime, kRolePeer or a reserved value
    std::uint16_t flags = 0;               ///< reserved; 0 in 1.0 and never judged
    WidthRow declared{};                   ///< the widths it declares (`hello` = payloadBytes, capped)
    double epochSeconds = 0.0;             ///< the writer's clock at open(), raw seconds
    std::uint8_t apiMajor = 0;             ///< the writer's shulib::kApiMajor
    std::uint8_t apiMinor = 0;             ///< the writer's shulib::kApiMinor
    std::string_view buildHash{};          ///< EMPTY means MISSING — render it loudly
    std::string_view routineId{};          ///< may be empty
    std::string_view alliance{};           ///< free text; may be empty
    std::string_view side{};               ///< free text; may be empty
    std::string_view portMap{};            ///< caller-authored; may be empty
};

/// Judge a stream's Hello against a reader's profile. Pure. Exactly one verdict, checked in
/// this order: BadSignature (fewer than 12 bytes, or a signature that differs); RefusedMajor;
/// LayoutMismatch (a Hello shorter than 256 bytes; at the SAME minor any declared width that
/// differs from the reader's row; at an OLDER minor any that differs from THAT minor's row; at
/// a NEWER minor any shorter than the reader's newest row; or a profile with no rows); then
/// Exact, OlderMinor or NewerMinor. Reserved bytes, flags and role are never judged.
[[nodiscard]] inline Verdict negotiate(const Hello& stream, const ReaderProfile& reader) noexcept {
    if (stream.payloadBytes < kHelloInvariantBytes) {
        return Verdict::BadSignature;
    }
    for (std::size_t i = 0; i < stream.signature.size(); ++i) {
        if (stream.signature[i] != kSignature[i]) {
            return Verdict::BadSignature;
        }
    }
    if (stream.versionMajor != reader.versionMajor) {
        return Verdict::RefusedMajor;
    }
    if (!stream.complete || reader.rows.empty()) {
        return Verdict::LayoutMismatch;
    }
    const std::size_t newest = reader.rows.size() - 1U;
    const std::size_t minor = stream.versionMinor;
    const WidthRow& d = stream.declared;
    if (minor <= newest) {
        const WidthRow& row = reader.rows[minor];
        const bool same = stream.payloadBytes == row.hello && d.tick == row.tick
                          && d.summary == row.summary && d.end == row.end
                          && d.logFixed == row.logFixed;
        if (!same) {
            return Verdict::LayoutMismatch;
        }
        return minor == newest ? Verdict::Exact : Verdict::OlderMinor;
    }
    const WidthRow& row = reader.rows[newest];
    const bool atLeast = stream.payloadBytes >= row.hello && d.tick >= row.tick
                         && d.summary >= row.summary && d.end >= row.end
                         && d.logFixed >= row.logFixed;
    return atLeast ? Verdict::NewerMinor : Verdict::LayoutMismatch;
}

// ── Scanning a byte stream for frames ───────────────────────────────────────────────

/// What scanFrame() found at the start of the bytes it was given.
enum class ScanKind : std::uint8_t {
    NeedMore = 0,  ///< a proper prefix of a frame that could still be valid (consumed 0)
    Frame = 1,     ///< a whole frame whose sync matched and whose CRC verified
    Skip = 2,      ///< not a frame here: drop `consumed` bytes and scan again
};

/// Why scanFrame() skipped.
enum class SkipReason : std::uint8_t {
    None = 0,            ///< not a skip
    NoSync = 1,          ///< no sync pair at offset 0: skip up to the next 0xFE (or everything)
    LengthTooLarge = 2,  ///< a declared length above kMaxPayloadBytes: skip 1 byte
    CrcMismatch = 3,     ///< a whole candidate whose CRC did not verify: skip 1 byte
};

/// scanFrame()'s result. For a Frame, `payload` views the caller's bytes.
struct ScanResult {
    ScanKind kind = ScanKind::NeedMore;     ///< what was found
    std::size_t consumed = 0;               ///< bytes to drop before scanning again
    SkipReason reason = SkipReason::None;   ///< why, for a Skip
    std::uint8_t type = 0;                  ///< a Frame's raw type byte
    std::uint8_t flags = 0;                 ///< a Frame's envelope flags (never judged)
    std::uint32_t seq = 0;                  ///< a Frame's sequence number
    std::span<const std::byte> payload{};   ///< a Frame's payload
};

/// Look for ONE frame at the start of `buffered`. Pure; never reads past the span. NeedMore
/// (consumed 0) when the bytes are a proper prefix of a frame that could still be valid — the
/// empty span, a lone 0xFE, or a valid header whose frame is longer than the bytes held. Frame
/// when the sync matched, the length is at most 4096 and the CRC verified (consumed 14 + n).
/// Skip otherwise. After a failed candidate, scanning ALWAYS resumes at the next byte — never at
/// the claimed length (header, the resync rule). A live reader appends bytes and calls again on
/// NeedMore; a reader of a whole capture applies the tail rule instead (shul2_reader.hpp) — and
/// so does a live reader once its stream ENDS (the socket closes, the file ends): what it still
/// holds is then a whole capture, and waiting on its NeedMore would hold back every frame after a
/// corrupted length, the End included, for good.
[[nodiscard]] inline ScanResult scanFrame(std::span<const std::byte> buffered) noexcept {
    ScanResult r{};
    if (buffered.empty()) {
        return r;  // NeedMore
    }
    const auto byteAt = [&buffered](std::size_t i) { return static_cast<std::uint8_t>(buffered[i]); };
    if (byteAt(0) != kSync0 || (buffered.size() >= 2 && byteAt(1) != kSync1)) {
        std::size_t next = 1;
        while (next < buffered.size() && byteAt(next) != kSync0) {
            ++next;
        }
        r.kind = ScanKind::Skip;
        r.reason = SkipReason::NoSync;
        r.consumed = next;
        return r;
    }
    if (buffered.size() < kPayloadOffset) {
        return r;  // NeedMore: the envelope's header is not all here yet
    }
    blackbox::ByteReader head{buffered.subspan(kTypeOffset, kPayloadOffset - kTypeOffset)};
    const std::uint8_t type = head.u8();
    const std::uint8_t flags = head.u8();
    const std::uint32_t seq = head.u32();
    const std::size_t n = head.u16();
    if (n > kMaxPayloadBytes) {
        r.kind = ScanKind::Skip;
        r.reason = SkipReason::LengthTooLarge;
        r.consumed = 1;
        return r;
    }
    if (buffered.size() < kFrameOverheadBytes + n) {
        return r;  // NeedMore: a valid header whose frame is longer than the bytes held
    }
    blackbox::ByteReader tail{buffered.subspan(kPayloadOffset + n, kCrcBytes)};
    const std::uint32_t stored = tail.u32();
    const std::uint32_t computed = crc32Ieee(buffered.subspan(kTypeOffset, kPayloadOffset - kTypeOffset + n));
    if (stored != computed) {
        r.kind = ScanKind::Skip;
        r.reason = SkipReason::CrcMismatch;
        r.consumed = 1;
        return r;
    }
    r.kind = ScanKind::Frame;
    r.consumed = kFrameOverheadBytes + n;
    r.type = type;
    r.flags = flags;
    r.seq = seq;
    r.payload = buffered.subspan(kPayloadOffset, n);
    return r;
}

// ── Encoders (whole or nothing, like the blackbox's) ────────────────────────────────

/// Wrap `payload` in the envelope: sync, type, flags, sequence, length, payload, CRC. Returns
/// the frame's bytes (14 + payload), or 0 — writing nothing — if the payload is longer than
/// kMaxPayloadBytes or `out` is too small. `type` is the raw byte, so a caller passes
/// `static_cast<std::uint8_t>(FrameType::Tick)` (explicit values: the cast is the value). The
/// payload may already sit at `out[10..]` (a writer that encodes in place); it is then not
/// copied. Anywhere else it is moved in first, overlap allowed.
[[nodiscard]] inline std::size_t encodeFrame(std::span<std::byte> out, std::uint8_t type,
                                             std::uint32_t seq, std::span<const std::byte> payload,
                                             std::uint8_t flags = 0U) noexcept {
    if (payload.size() > kMaxPayloadBytes) {
        return 0U;
    }
    const std::size_t total = kFrameOverheadBytes + payload.size();
    if (out.size() < total) {
        return 0U;  // whole or nothing: never leave half a frame behind
    }
    if (!payload.empty() && payload.data() != out.data() + kPayloadOffset) {
        std::memmove(out.data() + kPayloadOffset, payload.data(), payload.size());
    }
    blackbox::ByteWriter w{out.first(kPayloadOffset)};
    w.u8(kSync0);
    w.u8(kSync1);
    w.u8(type);
    w.u8(flags);
    w.u32(seq);
    w.u16(static_cast<std::uint16_t>(payload.size()));
    const std::span<const std::byte> covered =
        std::span<const std::byte>{out}.subspan(kTypeOffset, kPayloadOffset - kTypeOffset + payload.size());
    blackbox::ByteWriter c{out.subspan(kPayloadOffset + payload.size(), kCrcBytes)};
    c.u32(crc32Ieee(covered));
    return w.ok() && c.ok() ? total : 0U;
}

/// Encode THIS build's Hello (v1.0, 256 bytes): the signature, kVersionMajor/kVersionMinor,
/// `role`, the 1.0 widths, `epochSeconds`, the API version bytes and `info`'s provenance, each
/// text truncated to its field and NUL-padded. `info`'s views are read during this call only.
/// Returns 256, or 0 if `out` is too small. An EMPTY build hash stays empty.
[[nodiscard]] inline std::size_t encodeHello(std::span<std::byte> out, const SessionInfo& info,
                                             double epochSeconds,
                                             std::uint8_t role = kRoleRuntime) noexcept {
    if (out.size() < kHelloPayloadBytes) {
        return 0U;  // whole or nothing
    }
    const WidthRow& row = kWidthTable[kVersionMinor];
    blackbox::ByteWriter w{out};
    for (const std::uint8_t s : kSignature) {
        w.u8(s);
    }
    w.u16(kVersionMajor);
    w.u16(kVersionMinor);
    w.u8(role);
    w.u8(0U);   // reserved
    w.u16(0U);  // flags
    w.u16(row.tick);
    w.u16(row.summary);
    w.u16(row.end);
    w.u16(row.logFixed);
    w.f64(epochSeconds);
    w.u8(kApiMajorByte);
    w.u8(kApiMinorByte);
    w.u16(0U);  // reserved
    w.text(info.buildHash, 48);
    w.text(info.routineId, 32);
    w.text(info.alliance, 16);
    w.text(info.side, 16);
    w.text(info.portMap, 96);
    w.zeros(12);
    return w.ok() && w.offset() == kHelloPayloadBytes ? w.offset() : 0U;
}

/// One Log line: the fields of a Log payload. For decodeLog() the text fields are VIEWS into
/// the caller's payload bytes and are never NUL-terminated.
struct LogLine {
    std::uint8_t level = 2;   ///< the WIRE level code (Error 0 … Trace 4); raw, may be unknown
    std::uint8_t flags = 0;   ///< kLogTagTruncated | kLogMessageTruncated; bits 2-7 ignored
    double t = 0.0;           ///< the clock's raw reading when log() was called, seconds
    std::string_view tag{};      ///< the subsystem tag, verbatim
    std::string_view message{};  ///< the message, verbatim — never sanitized
    /// Decoded only: every payload byte that is not text (payloadBytes − tag − message), which
    /// a reader compares with the Hello's logFixedBytes. The encoder always writes 1.0's 14.
    std::size_t nonTextBytes = kLogFixedBytes;
};

/// Encode one Log payload: the 14 fixed bytes, then the tag, then the message, verbatim. The
/// caller has already applied any caps (a writer POLICY; utf8Prefix() cuts on a boundary).
/// Returns 14 + tag + message, or 0 — writing nothing — if the tag is longer than 255 bytes,
/// the payload would exceed kMaxPayloadBytes, or `out` is too small.
[[nodiscard]] inline std::size_t encodeLog(std::span<std::byte> out, const LogLine& line) noexcept {
    const std::size_t total = kLogFixedBytes + line.tag.size() + line.message.size();
    if (line.tag.size() > 0xFFU || total > kMaxPayloadBytes || out.size() < total) {
        return 0U;
    }
    blackbox::ByteWriter w{out};
    w.u8(line.level);
    w.u8(line.flags);
    w.u8(static_cast<std::uint8_t>(line.tag.size()));
    w.u8(0U);  // reserved
    w.u16(static_cast<std::uint16_t>(line.message.size()));
    w.f64(line.t);
    for (const char c : line.tag) {
        w.u8(static_cast<std::uint8_t>(c));
    }
    for (const char c : line.message) {
        w.u8(static_cast<std::uint8_t>(c));
    }
    return w.ok() && w.offset() == total ? total : 0U;
}

/// The End: the writer's own account of a session, sent at close().
struct EndInfo {
    std::uint32_t framesOffered = 0;   ///< frames offered to the device — the End's own sequence number
    std::uint32_t framesFailed = 0;    ///< frames the device refused (they appear as gaps)
    std::uint32_t recordsSeen = 0;     ///< DebugRecords handed to the sink this session
    std::uint32_t linesSeen = 0;       ///< log() lines handed to the sink this session
    std::uint32_t linesTruncated = 0;  ///< lines whose tag or message was cut to its cap
    std::uint32_t notOpenDropped = 0;  ///< items handed over while no session was open
    double endTime = 0.0;              ///< the clock's raw reading at close(), seconds
};

/// Encode the End payload (v1.0, 32 bytes). Returns 32, or 0 if `out` is too small.
[[nodiscard]] inline std::size_t encodeEnd(std::span<std::byte> out, const EndInfo& e) noexcept {
    if (out.size() < kEndPayloadBytes) {
        return 0U;  // whole or nothing
    }
    blackbox::ByteWriter w{out};
    w.u32(e.framesOffered);
    w.u32(e.framesFailed);
    w.u32(e.recordsSeen);
    w.u32(e.linesSeen);
    w.u32(e.linesTruncated);
    w.u32(e.notOpenDropped);
    w.f64(e.endTime);
    return w.ok() && w.offset() == kEndPayloadBytes ? w.offset() : 0U;
}

// ── Decoders (a payload of AT LEAST the known width; its known prefix is decoded) ────

namespace detail {
/// A NUL-padded text field of `width` bytes at `at`, as a view ending at its first NUL — or at
/// the field's end when it fills its width with no NUL (a foreign or later writer may), never
/// past it: the next field and the frame's own CRC are not text.
[[nodiscard]] inline std::string_view textField(std::span<const std::byte> payload, std::size_t at,
                                                std::size_t width) noexcept {
    const char* p = reinterpret_cast<const char*>(payload.data() + at);
    std::size_t len = 0;
    while (len < width && p[len] != '\0') {
        ++len;
    }
    return std::string_view{p, len};
}
}  // namespace detail

/// Decode a Hello. Bytes 0-11 (signature and version) are decoded from any payload of at least
/// 12 bytes — so a short or foreign Hello can still be judged — and the rest only from a payload
/// of 256 bytes or more whose major is kVersionMajor, which sets `complete`. Another major's
/// bytes 12 onward belong to that major (header, "Versions"), so they are never read with this
/// major's layout: its role, widths, epoch and text stay at their defaults. Returns false only
/// when fewer than 12 bytes arrived. Never judges: negotiate() does. Text fields view `payload`
/// and end at their first NUL, or at their field's width.
[[nodiscard]] inline bool decodeHello(std::span<const std::byte> payload, Hello& out) noexcept {
    out = Hello{};
    out.payloadBytes = payload.size();
    if (payload.size() < kHelloInvariantBytes) {
        return false;
    }
    blackbox::ByteReader r{payload};
    for (auto& s : out.signature) {
        s = r.u8();
    }
    out.versionMajor = r.u16();
    out.versionMinor = r.u16();
    if (payload.size() < kHelloPayloadBytes || out.versionMajor != kVersionMajor) {
        return true;  // the invariant part only; `complete` stays false
    }
    out.role = r.u8();
    r.skip(1);  // reserved: never judged
    out.flags = r.u16();
    out.declared.hello = static_cast<std::uint16_t>(payload.size() > 0xFFFFU ? 0xFFFFU : payload.size());
    out.declared.tick = r.u16();
    out.declared.summary = r.u16();
    out.declared.end = r.u16();
    out.declared.logFixed = r.u16();
    out.epochSeconds = r.f64();
    out.apiMajor = r.u8();
    out.apiMinor = r.u8();
    out.buildHash = detail::textField(payload, 36, 48);
    out.routineId = detail::textField(payload, 84, 32);
    out.alliance = detail::textField(payload, 116, 16);
    out.side = detail::textField(payload, 132, 16);
    out.portMap = detail::textField(payload, 148, 96);
    out.complete = r.ok();
    return true;
}

/// Decode a Log payload. Returns false if it is shorter than 14 bytes or its declared tag and
/// message run past its end. Fields a later minor appends after the text are not read, and
/// show in `nonTextBytes`. The level stays a raw code: map it with logLevelFromWire().
[[nodiscard]] inline bool decodeLog(std::span<const std::byte> payload, LogLine& out) noexcept {
    if (payload.size() < kLogFixedBytes) {
        return false;
    }
    blackbox::ByteReader r{payload};
    const std::uint8_t level = r.u8();
    const std::uint8_t flags = r.u8();
    const std::size_t tagBytes = r.u8();
    r.skip(1);  // reserved: never judged
    const std::size_t messageBytes = r.u16();
    const double t = r.f64();
    if (!r.ok() || kLogFixedBytes + tagBytes + messageBytes > payload.size()) {
        return false;
    }
    const char* text = reinterpret_cast<const char*>(payload.data() + kLogFixedBytes);
    out.level = level;
    out.flags = flags;
    out.t = t;
    out.tag = std::string_view{text, tagBytes};
    out.message = std::string_view{text + tagBytes, messageBytes};
    out.nonTextBytes = payload.size() - tagBytes - messageBytes;
    return true;
}

/// Decode an End payload of at least 32 bytes (a later minor's appended bytes are not read).
/// Returns false if it is shorter.
[[nodiscard]] inline bool decodeEnd(std::span<const std::byte> payload, EndInfo& e) noexcept {
    if (payload.size() < kEndPayloadBytes) {
        return false;
    }
    blackbox::ByteReader r{payload.first(kEndPayloadBytes)};
    e.framesOffered = r.u32();
    e.framesFailed = r.u32();
    e.recordsSeen = r.u32();
    e.linesSeen = r.u32();
    e.linesTruncated = r.u32();
    e.notOpenDropped = r.u32();
    e.endTime = r.f64();
    return r.ok();
}

/// Decode a Tick payload of at least 428 bytes through blackbox::decodeTick on its first 428
/// (a newer minor's appended bytes are not read). Returns false if it is shorter. `corrupt` is
/// raised as the blackbox decoder raises it: a non-finite heading (decoded as 0) or a
/// wheelCount above DebugRecord::kMaxWheels (decoded as kMaxWheels).
[[nodiscard]] inline bool decodeTick(std::span<const std::byte> payload, DebugRecord& r,
                                     bool& corrupt) noexcept {
    if (payload.size() < blackbox::kTickPayloadBytes) {
        return false;
    }
    return blackbox::decodeTick(payload.first(blackbox::kTickPayloadBytes), r, corrupt);
}

/// Decode a Summary payload of at least 168 bytes through blackbox::decodeSummary on its first
/// 168. `blackboxDropped` receives the summary's own RunSummary::blackboxDropped figure as the
/// writer carried it. Returns false if the payload is shorter.
[[nodiscard]] inline bool decodeSummary(std::span<const std::byte> payload, RunSummary& s,
                                        std::uint32_t& blackboxDropped) noexcept {
    if (payload.size() < blackbox::kSummaryPayloadBytes) {
        return false;
    }
    return blackbox::decodeSummary(payload.first(blackbox::kSummaryPayloadBytes), s, blackboxDropped);
}

// ── Log levels on the wire, and the UTF-8-safe cut ──────────────────────────────────

/// The wire's level codes. EXPLICIT values, because hal::LogLevel declares none: the two are
/// mapped by a switch in both directions (wireLevel, logLevelFromWire), never by a cast.
enum class WireLevel : std::uint8_t {
    Error = 0,  ///< hal::LogLevel::Error
    Warn = 1,   ///< hal::LogLevel::Warn
    Info = 2,   ///< hal::LogLevel::Info
    Debug = 3,  ///< hal::LogLevel::Debug
    Trace = 4,  ///< hal::LogLevel::Trace
};

/// The wire code for a hal::LogLevel (a switch, never a cast).
[[nodiscard]] constexpr WireLevel wireLevel(hal::LogLevel level) noexcept {
    switch (level) {
        case hal::LogLevel::Error: return WireLevel::Error;
        case hal::LogLevel::Warn: return WireLevel::Warn;
        case hal::LogLevel::Info: return WireLevel::Info;
        case hal::LogLevel::Debug: return WireLevel::Debug;
        case hal::LogLevel::Trace: return WireLevel::Trace;
    }
    return WireLevel::Error;  // unreachable for a valid LogLevel; the loudest level if not
}

/// Map a raw wire level code back to hal::LogLevel (a switch, never a cast). Returns false —
/// "unknown", `out` untouched — for a code this build does not name; the code itself stays
/// available as the raw number in LogLine::level.
[[nodiscard]] constexpr bool logLevelFromWire(std::uint8_t code, hal::LogLevel& out) noexcept {
    switch (code) {
        case 0: out = hal::LogLevel::Error; return true;
        case 1: out = hal::LogLevel::Warn; return true;
        case 2: out = hal::LogLevel::Info; return true;
        case 3: out = hal::LogLevel::Debug; return true;
        case 4: out = hal::LogLevel::Trace; return true;
        default: return false;
    }
}

/// The longest prefix of `text` of at most `cap` bytes that does not split a UTF-8 sequence:
/// `text` itself when it fits, otherwise cut at `cap` and backed off over continuation bytes —
/// the same back-off as lineformat::Line::appendSanitized, WITHOUT the sanitizing (wire text is
/// carried verbatim). For a writer's caps (a policy, H1 Part A2).
[[nodiscard]] constexpr std::string_view utf8Prefix(std::string_view text, std::size_t cap) noexcept {
    if (text.size() <= cap) {
        return text;
    }
    std::size_t take = cap;
    while (take > 0 && (static_cast<unsigned char>(text[take]) & 0xC0U) == 0x80U) {
        --take;
    }
    return text.substr(0, take);
}

}  // namespace shulib::diag::shul2
