#pragma once
//
// The SHULIB BLACKBOX on-disk format, v1 — the binary record SdSink writes and
// BlackboxReader reads (master plan §18; diagnostics-plan D-6/D-7; WS13, chunk E1).
//
// ── Why a format document lives in code ─────────────────────────────────────────────
// A file outlives the program that wrote it. Six months from now the only thing that
// can open a run from a competition day is a decoder that agrees with this layout, so
// the layout is written down ONCE, here, with its version, and the encoder and the
// decoder are both held to it BY A BYTE-EXACT GOLDEN TEST — not by their agreement
// with each other. Two sides that share one bug agree perfectly; that is precisely the
// failure a round trip cannot see, which is why test/blackbox_format_test.cpp pins the
// literal bytes of the header and of one full tick record.
//
// ── The shape ───────────────────────────────────────────────────────────────────────
//   [ 256-byte file header ][ frame ][ frame ][ frame ] …
// Every frame is  { u8 type, u8 reserved, u16 payloadBytes }  followed by payloadBytes
// of payload. A reader that meets a frame type it does not know SKIPS it by its length
// and counts it — that is what makes a v1 reader survive a v1.1 writer that appended a
// new frame kind, without ever GUESSING at content it does not understand.
//
// ── Versioned from byte zero, and refusal over misreading ───────────────────────────
// The header carries kFormatVersion. A decoder that mis-reads an old file is worse
// than one that refuses it: a wrong number read confidently sends the 2am investigation
// somewhere false, while a refusal sends it to the git history. So the reader refuses
// any version it was not built for, and ALSO cross-checks the header's self-declared
// record width against its own — that catches the specific human error of changing the
// layout and forgetting to bump the version.
//
// ── Fixed width, and no narrowing ANYWHERE ──────────────────────────────────────────
// Every field is fixed width and little-endian, written byte by byte (never a struct
// memcpy: padding and ABI are not a file format). Every floating-point field is IEEE-754
// binary64, including the per-wheel arrays and the tick-phase slots, where binary32
// would have saved ~35% of the file. That trade was made deliberately: the chunk's
// central promise is that a decoded record equals the encoded one FIELD BY FIELD, and
// with narrowing that sentence would have quietly become "equals after a documented
// rounding" — a weaker claim that also makes every future comparison of two runs
// depend on rounding behaviour. Bytes are cheap on an SD card; a fuzzy record is not.
//
// ── What v1 does NOT carry, said plainly ────────────────────────────────────────────
// The log() message channel. Variable-length text in a fixed-width record format is a
// different problem with different trade-offs (bounded-vs-truncated strings, per-line
// framing), and the terminal (A1) already owns text. The blackbox carries the per-tick
// RECORD, the run SUMMARY, and the fault TRIAGE block. SdSink counts the messages it
// was handed and writes that count into the end frame, so the omission is visible in
// the file rather than silent — a reader can always see that N lines existed elsewhere.
//
// ── What H1 (F9, the SHUL/2 wire) inherits ──────────────────────────────────────────
// This is a PERSISTENCE contract the moment a file exists, but it is deliberately NOT
// the wire: SHUL/2 is streamed, sequenced and versioned on its own terms. What H1
// should reuse is the FIELD ORDER of encodeTick() (it follows debug_record.hpp's own
// declaration order, so a schema append lands at the end on both) and the refuse-don't-
// misread rule. Nothing here freezes anything: F9 is H1's, and E1 freezes nothing.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"

namespace shulib::diag::blackbox {

/// The four magic bytes every blackbox file starts with ("SHulib BlackBox").
inline constexpr char kMagic[4] = {'S', 'H', 'B', 'B'};

/// On-disk format version. BUMP THIS whenever any layout below changes — a reader
/// refuses a version it was not built for rather than misreading it (header note).
inline constexpr std::uint16_t kFormatVersion = 1;

/// Size of the fixed file header, in bytes (v1). Fixed width so a reader can seek
/// past it without parsing, and generous enough to hold full provenance.
inline constexpr std::size_t kHeaderBytes = 256;

/// Size of the per-frame prefix: {u8 type, u8 reserved, u16 payloadBytes}.
inline constexpr std::size_t kFrameHeaderBytes = 4;

/// Payload size of one Tick frame (v1). Pinned by the golden test; the encoder
/// asserts it wrote exactly this many bytes.
inline constexpr std::size_t kTickPayloadBytes = 428;

/// Payload size of one Summary frame (v1).
inline constexpr std::size_t kSummaryPayloadBytes = 168;

/// Payload size of one Triage frame (v1): the D-7 triage fields PLUS the complete
/// record of the tick the fault fired on (header note on dump ordering in sd_sink.hpp).
inline constexpr std::size_t kTriagePayloadBytes = 24 + kTickPayloadBytes;

/// Payload size of one End frame (v1) — the graceful-end stamp.
inline constexpr std::size_t kEndPayloadBytes = 28;

/// What a frame carries. WIRE-STABLE: explicit values, append-only — an unknown type
/// is skipped by length, never guessed at.
enum class FrameType : std::uint8_t {
    Tick = 1,     ///< one DebugRecord (kTickPayloadBytes)
    Summary = 2,  ///< one RunSummary (kSummaryPayloadBytes)
    Triage = 3,   ///< the D-7 fault triage block + the fault tick's own record
    End = 4,      ///< the graceful-end stamp: counts, brownout latch, end time
};

/// The D-7 triage block, as data: which fault, when, on which tick, and how many
/// preceding ticks follow it in the file. The record of the fault tick itself travels
/// in the same frame (see sd_sink.hpp's dump-ordering rule).
struct TriageInfo {
    FaultCode fault = FaultCode::None;   ///< the fault that triggered the dump
    bool brownout = false;               ///< the latched brownout marker at dump time
    std::uint32_t tickIndex = 0;         ///< how many records the sink had seen when it fired
    double faultTime = 0.0;              ///< the fault tick's `t`, seconds since the run epoch
    std::uint32_t precedingTicks = 0;    ///< Tick frames that follow, oldest first (0 when streaming)
};

/// The end frame: what the sink knows about its own run when it closes cleanly. A file
/// WITHOUT this frame ended abruptly — that absence is the truncation signal a reader
/// can act on.
struct EndInfo {
    std::uint32_t tickFrames = 0;      ///< Tick frames staged over the run
    std::uint32_t droppedFrames = 0;   ///< frames dropped for want of buffer (byte budget)
    /// Bytes of this file that PRECEDE this frame — i.e. the frame's own offset. A
    /// reader can verify it against where it actually found the frame, which is how a
    /// file that was appended to, interleaved, or spliced gives itself away. (It is
    /// NOT "bytes the device confirmed": at close() the bulk of a caller-paced run is
    /// still staged and goes out in the same write as this frame, so that figure would
    /// read 0 for the most common run of all.)
    std::uint32_t bytesBefore = 0;
    std::uint32_t messagesSeen = 0;    ///< log() lines handed to the sink and NOT carried (header note)
    bool brownout = false;             ///< the latched brownout marker
    bool deviceFailed = false;         ///< a write() or flush() reported failure during the run
    double endTime = 0.0;              ///< clock time at close, seconds since the run epoch
};

/// A decoded file header. Value type with bounded storage, like RunSummary: a decoded
/// header must never hold views into a buffer the caller may free.
struct BlackboxHeader {
    std::uint16_t formatVersion = 0;    ///< as read from the file
    std::uint16_t headerBytes = 0;      ///< self-declared header size (lets a reader seek)
    std::uint16_t tickRecordBytes = 0;  ///< self-declared Tick payload size (cross-checked)
    std::uint16_t flags = 0;            ///< reserved, 0 in v1
    double epochSeconds = 0.0;          ///< the injected clock's reading when the file opened
    std::uint32_t ringCapacity = 0;     ///< flight-recorder ring size the writer was configured with
    std::uint32_t byteBudget = 0;       ///< RAM byte budget the writer was configured with

    /// The git build hash the run was built from. EMPTY means MISSING — render it
    /// loudly and never invent a plausible value (§18.5, build_info.hpp).
    [[nodiscard]] std::string_view buildHash() const noexcept { return buildHash_; }
    /// The routine id the run was started with (may be empty).
    [[nodiscard]] std::string_view routineId() const noexcept { return routineId_; }
    /// Alliance as free text ("red"/"blue"/"skills"); may be empty.
    [[nodiscard]] std::string_view alliance() const noexcept { return alliance_; }
    /// Side as free text ("left"/"right"); may be empty.
    [[nodiscard]] std::string_view side() const noexcept { return side_; }
    /// The caller-authored port map; may be empty.
    [[nodiscard]] std::string_view portMap() const noexcept { return portMap_; }

    /// Storage for buildHash() — written by the decoder, NUL-terminated.
    char buildHash_[48] = "";
    /// Storage for routineId().
    char routineId_[32] = "";
    /// Storage for alliance().
    char alliance_[16] = "";
    /// Storage for side().
    char side_[16] = "";
    /// Storage for portMap().
    char portMap_[96] = "";
};

/// Little-endian byte writer with a hard end: a write that would not fit writes
/// NOTHING and latches overflow, so an undersized buffer can never corrupt neighbouring
/// memory and can never half-write a field. Callers check ok().
class ByteWriter {
public:
    /// Write into `out`, starting at offset 0.
    explicit ByteWriter(std::span<std::byte> out) noexcept : out_{out} {}

    /// Append one unsigned byte.
    void u8(std::uint8_t v) noexcept {
        if (!room(1)) {
            return;
        }
        out_[at_++] = static_cast<std::byte>(v);
    }
    /// Append a bool as 0x00 / 0x01.
    void boolean(bool v) noexcept { u8(v ? 1U : 0U); }
    /// Append a 16-bit unsigned value, little-endian.
    void u16(std::uint16_t v) noexcept {
        if (!room(2)) {
            return;
        }
        u8(static_cast<std::uint8_t>(v & 0xFFU));
        u8(static_cast<std::uint8_t>((v >> 8) & 0xFFU));
    }
    /// Append a 32-bit unsigned value, little-endian.
    void u32(std::uint32_t v) noexcept {
        if (!room(4)) {
            return;
        }
        for (int i = 0; i < 4; ++i) {
            u8(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
        }
    }
    /// Append a 32-bit signed value as two's complement, little-endian.
    void i32(std::int32_t v) noexcept { u32(static_cast<std::uint32_t>(v)); }
    /// Append an IEEE-754 binary64 value, little-endian (bit pattern preserved, so a
    /// NaN or an infinity survives the trip exactly as it was recorded).
    void f64(double v) noexcept {
        if (!room(8)) {
            return;
        }
        std::uint64_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        for (int i = 0; i < 8; ++i) {
            u8(static_cast<std::uint8_t>((bits >> (8 * i)) & 0xFFU));
        }
    }
    /// Append `fieldBytes` of text: `s` truncated to fit, NUL-padded to the full width.
    /// Fixed width by design — a variable-length string would make every later offset
    /// depend on run-time content.
    void text(std::string_view s, std::size_t fieldBytes) noexcept {
        if (!room(fieldBytes)) {
            return;
        }
        const std::size_t take = s.size() < fieldBytes ? s.size() : fieldBytes - 1;
        for (std::size_t i = 0; i < fieldBytes; ++i) {
            u8(i < take ? static_cast<std::uint8_t>(s[i]) : 0U);
        }
    }
    /// Append `n` zero bytes (reserved space).
    void zeros(std::size_t n) noexcept {
        if (!room(n)) {
            return;
        }
        for (std::size_t i = 0; i < n; ++i) {
            u8(0U);
        }
    }
    /// How many bytes have been appended.
    [[nodiscard]] std::size_t offset() const noexcept { return at_; }
    /// False once any append did not fit (nothing was written for that append).
    [[nodiscard]] bool ok() const noexcept { return !overflow_; }

private:
    [[nodiscard]] bool room(std::size_t n) noexcept {
        if (overflow_ || at_ + n > out_.size()) {
            overflow_ = true;
            return false;
        }
        return true;
    }

    std::span<std::byte> out_;
    std::size_t at_ = 0;
    bool overflow_ = false;
};

/// Little-endian byte reader with a hard end: a read past the end yields zero and
/// latches exhaustion, so a truncated or corrupt file can never read out of bounds and
/// can never half-read a field. Callers check ok().
class ByteReader {
public:
    /// Read from `in`, starting at offset 0.
    explicit ByteReader(std::span<const std::byte> in) noexcept : in_{in} {}

    /// Read one unsigned byte (0 past the end).
    [[nodiscard]] std::uint8_t u8() noexcept {
        if (!room(1)) {
            return 0U;
        }
        return static_cast<std::uint8_t>(in_[at_++]);
    }
    /// Read a bool: any nonzero byte is true.
    [[nodiscard]] bool boolean() noexcept { return u8() != 0U; }
    /// Read a 16-bit unsigned value, little-endian.
    [[nodiscard]] std::uint16_t u16() noexcept {
        if (!room(2)) {
            return 0U;
        }
        const auto lo = static_cast<std::uint16_t>(u8());
        const auto hi = static_cast<std::uint16_t>(u8());
        return static_cast<std::uint16_t>(lo | static_cast<std::uint16_t>(hi << 8));
    }
    /// Read a 32-bit unsigned value, little-endian.
    [[nodiscard]] std::uint32_t u32() noexcept {
        if (!room(4)) {
            return 0U;
        }
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            v |= static_cast<std::uint32_t>(u8()) << (8 * i);
        }
        return v;
    }
    /// Read a 32-bit signed value (two's complement), little-endian.
    [[nodiscard]] std::int32_t i32() noexcept { return static_cast<std::int32_t>(u32()); }
    /// Read an IEEE-754 binary64 value, little-endian (bit pattern preserved).
    [[nodiscard]] double f64() noexcept {
        if (!room(8)) {
            return 0.0;
        }
        std::uint64_t bits = 0;
        for (int i = 0; i < 8; ++i) {
            bits |= static_cast<std::uint64_t>(u8()) << (8 * i);
        }
        double v = 0.0;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    /// Read `fieldBytes` of NUL-padded text into `dst` (capacity `dstBytes`, always
    /// NUL-terminated). Bytes beyond the destination are consumed and discarded, so the
    /// cursor stays aligned no matter how the caller sized its storage.
    void text(char* dst, std::size_t dstBytes, std::size_t fieldBytes) noexcept {
        std::size_t written = 0;
        for (std::size_t i = 0; i < fieldBytes; ++i) {
            const auto c = static_cast<char>(u8());
            if (written + 1 < dstBytes && c != '\0') {
                dst[written++] = c;
            }
        }
        if (dstBytes > 0) {
            dst[written] = '\0';
        }
    }
    /// Skip `n` bytes (reserved space).
    void skip(std::size_t n) noexcept {
        for (std::size_t i = 0; i < n; ++i) {
            (void)u8();
        }
    }
    /// How many bytes have been consumed.
    [[nodiscard]] std::size_t offset() const noexcept { return at_; }
    /// False once any read ran past the end.
    [[nodiscard]] bool ok() const noexcept { return !exhausted_; }

private:
    [[nodiscard]] bool room(std::size_t n) noexcept {
        if (exhausted_ || at_ + n > in_.size()) {
            exhausted_ = true;
            return false;
        }
        return true;
    }

    std::span<const std::byte> in_;
    std::size_t at_ = 0;
    bool exhausted_ = false;
};

// ── The file header ─────────────────────────────────────────────────────────────────
//   0  u8[4]   magic "SHBB"            12  f64     epochSeconds
//   4  u16     formatVersion           20  u32     ringCapacity
//   6  u16     headerBytes (256)       24  u32     byteBudget
//   8  u16     tickRecordBytes (428)   28  u8[48]  buildHash
//  10  u16     flags (0)               76  u8[32]  routineId
//                                     108  u8[16]  alliance
//                                     124  u8[16]  side
//                                     140  u8[96]  portMap
//                                     236  u8[20]  reserved (zero)

/// Encode the 256-byte file header into `out`. Returns the bytes written (0 if `out`
/// is too small). Provenance strings are copied in, truncated to their field widths —
/// an EMPTY build hash stays empty, because MISSING must stay loud all the way to disk.
[[nodiscard]] inline std::size_t encodeHeader(std::span<std::byte> out, const SessionInfo& info,
                                              double epochSeconds, std::uint32_t ringCapacity,
                                              std::uint32_t byteBudget) noexcept {
    if (out.size() < kHeaderBytes) {
        return 0U;  // whole or nothing: never leave a half-written header behind
    }
    ByteWriter w{out};
    for (const char c : kMagic) {
        w.u8(static_cast<std::uint8_t>(c));
    }
    w.u16(kFormatVersion);
    w.u16(static_cast<std::uint16_t>(kHeaderBytes));
    w.u16(static_cast<std::uint16_t>(kTickPayloadBytes));
    w.u16(0U);  // flags
    w.f64(epochSeconds);
    w.u32(ringCapacity);
    w.u32(byteBudget);
    w.text(info.buildHash, 48);
    w.text(info.routineId, 32);
    w.text(info.alliance, 16);
    w.text(info.side, 16);
    w.text(info.portMap, 96);
    w.zeros(20);
    return w.ok() && w.offset() == kHeaderBytes ? w.offset() : 0U;
}

/// Decode a file header. Returns false if `in` is shorter than the header or the magic
/// does not match; the VERSION is decoded but NOT judged here — BlackboxReader owns the
/// refusal policy, and a caller inspecting a rejected file still wants to see what
/// version it claims to be.
[[nodiscard]] inline bool decodeHeader(std::span<const std::byte> in, BlackboxHeader& out) noexcept {
    if (in.size() < kHeaderBytes) {
        return false;
    }
    ByteReader r{in};
    for (const char c : kMagic) {
        if (r.u8() != static_cast<std::uint8_t>(c)) {
            return false;
        }
    }
    out.formatVersion = r.u16();
    out.headerBytes = r.u16();
    out.tickRecordBytes = r.u16();
    out.flags = r.u16();
    out.epochSeconds = r.f64();
    out.ringCapacity = r.u32();
    out.byteBudget = r.u32();
    r.text(out.buildHash_, sizeof out.buildHash_, 48);
    r.text(out.routineId_, sizeof out.routineId_, 32);
    r.text(out.alliance_, sizeof out.alliance_, 16);
    r.text(out.side_, sizeof out.side_, 16);
    r.text(out.portMap_, sizeof out.portMap_, 96);
    r.skip(20);
    return r.ok();
}

// ── The tick record ─────────────────────────────────────────────────────────────────
// Field order follows debug_record.hpp's own declaration order, with the small scalars
// gathered into one block so the layout has no implicit padding anywhere. Offsets:
//    0 t                 88 commanded.vx      120 fault(u16)      292 gateResidualX
//    8 dt                96 commanded.vy      122 gateReason(u8)  300 gateResidualY
//   16 targetPose.x     104 commanded.omega   123 flags(u8)       308 gateResidualHeading
//   24 targetPose.y     112 wheelCount(u8)    124 droppedRecords  316 gateMahalanobis
//   32 targetPose.h     113 cmdState(u8)      128 droppedLines    324 correctionDx
//   40 measuredPose.x   114 deadReckon(u8)    132 wheelVoltage[8] 332 correctionDy
//   48 measuredPose.y   115 qualityClass(u8)  196 wheelCurrent[8] 340 correctionDTheta
//   56 measuredPose.h   116 activeCommandId   260 imuYaw          348 batteryVoltage
//   64 errorX                                 268 imuYawRate      356 batteryCurrent
//   72 errorY                                 276 quality         364 tickPhase[8]
//   80 errorHeading                           284 covarianceTrace 428 = end
// flags bit 0 = clampedThisTick, bit 1 = strafeFallbackActive.

/// Encode one DebugRecord. Returns the bytes written, or 0 if `out` was too small or
/// the layout did not come out to exactly kTickPayloadBytes (a loud, testable failure
/// rather than a silently short record).
[[nodiscard]] inline std::size_t encodeTick(std::span<std::byte> out, const DebugRecord& r) noexcept {
    if (out.size() < kTickPayloadBytes) {
        return 0U;  // whole or nothing (see encodeHeader)
    }
    ByteWriter w{out};
    w.f64(r.t.value());
    w.f64(r.dt.value());
    w.f64(r.targetPose.x().value());
    w.f64(r.targetPose.y().value());
    w.f64(r.targetPose.heading().radians());
    w.f64(r.measuredPose.x().value());
    w.f64(r.measuredPose.y().value());
    w.f64(r.measuredPose.heading().radians());
    w.f64(r.errorX.value());
    w.f64(r.errorY.value());
    w.f64(r.errorHeading.value());
    w.f64(r.commanded.vx().value());
    w.f64(r.commanded.vy().value());
    w.f64(r.commanded.omega().value());
    w.u8(static_cast<std::uint8_t>(r.wheelCount < 0 ? 0 : r.wheelCount));
    w.u8(r.activeCommandState);
    w.boolean(r.deadReckoning);
    w.u8(r.qualityClass);
    w.u32(r.activeCommandId);
    w.u16(static_cast<std::uint16_t>(r.fault));
    w.u8(static_cast<std::uint8_t>(r.gateReason));
    w.u8(static_cast<std::uint8_t>((r.clampedThisTick ? 1U : 0U)
                                   | (r.strafeFallbackActive ? 2U : 0U)));
    w.u32(r.droppedRecords);
    w.u32(r.droppedLines);
    for (const auto v : r.wheelVoltage) {
        w.f64(v.value());
    }
    for (const auto v : r.wheelCurrent) {
        w.f64(v.value());
    }
    w.f64(r.imuYaw.radians());
    w.f64(r.imuYawRate.value());
    w.f64(r.quality);
    w.f64(r.covarianceTrace);
    w.f64(r.gateResidualX.value());
    w.f64(r.gateResidualY.value());
    w.f64(r.gateResidualHeading.value());
    w.f64(r.gateMahalanobis);
    w.f64(r.correctionDx.value());
    w.f64(r.correctionDy.value());
    w.f64(r.correctionDTheta.value());
    w.f64(r.batteryVoltage.value());
    w.f64(r.batteryCurrent.value());
    for (const auto v : r.tickPhase) {
        w.f64(v.value());
    }
    return w.ok() && w.offset() == kTickPayloadBytes ? w.offset() : 0U;
}

/// Rebuild an Angle from a decoded radian value WITHOUT trusting the file: a corrupt
/// or truncated blackbox can contain any bit pattern, and math::Angle's factory rejects
/// non-finite input by precondition. A decoder that throws on a corrupt file is a
/// decoder you cannot use on the file you most need to read, so a non-finite heading
/// decodes to zero and `corrupt` is raised for the caller to see.
[[nodiscard]] inline math::Angle safeAngle(double radians, bool& corrupt) noexcept {
    if (!std::isfinite(radians)) {
        corrupt = true;
        return math::Angle{};
    }
    return math::Angle::radians(radians);
}

/// Decode one DebugRecord. Returns false if the payload is not exactly
/// kTickPayloadBytes. `corrupt` is set (never cleared) when a field could not be
/// represented — today: a non-finite heading, which decodes to zero (safeAngle).
[[nodiscard]] inline bool decodeTick(std::span<const std::byte> in, DebugRecord& r,
                                     bool& corrupt) noexcept {
    if (in.size() != kTickPayloadBytes) {
        return false;
    }
    ByteReader b{in};
    r.t = units::Time{b.f64()};
    r.dt = units::Time{b.f64()};
    {
        const double x = b.f64();
        const double y = b.f64();
        const double h = b.f64();
        r.targetPose = math::Pose2d{units::Length{x}, units::Length{y}, safeAngle(h, corrupt)};
    }
    {
        const double x = b.f64();
        const double y = b.f64();
        const double h = b.f64();
        r.measuredPose = math::Pose2d{units::Length{x}, units::Length{y}, safeAngle(h, corrupt)};
    }
    r.errorX = units::Length{b.f64()};
    r.errorY = units::Length{b.f64()};
    r.errorHeading = units::AngleDim{b.f64()};
    {
        const double vx = b.f64();
        const double vy = b.f64();
        const double w = b.f64();
        r.commanded = math::ChassisSpeeds{units::Velocity{vx}, units::Velocity{vy},
                                          units::AngularVelocity{w}};
    }
    r.wheelCount = static_cast<int>(b.u8());
    r.activeCommandState = b.u8();
    r.deadReckoning = b.boolean();
    r.qualityClass = b.u8();
    r.activeCommandId = b.u32();
    r.fault = static_cast<FaultCode>(b.u16());
    r.gateReason = static_cast<GateReason>(b.u8());
    {
        const std::uint8_t flags = b.u8();
        r.clampedThisTick = (flags & 1U) != 0U;
        r.strafeFallbackActive = (flags & 2U) != 0U;
    }
    r.droppedRecords = b.u32();
    r.droppedLines = b.u32();
    for (auto& v : r.wheelVoltage) {
        v = units::Voltage{b.f64()};
    }
    for (auto& v : r.wheelCurrent) {
        v = units::Current{b.f64()};
    }
    r.imuYaw = safeAngle(b.f64(), corrupt);
    r.imuYawRate = units::AngularVelocity{b.f64()};
    r.quality = b.f64();
    r.covarianceTrace = b.f64();
    r.gateResidualX = units::Length{b.f64()};
    r.gateResidualY = units::Length{b.f64()};
    r.gateResidualHeading = units::AngleDim{b.f64()};
    r.gateMahalanobis = b.f64();
    r.correctionDx = units::Length{b.f64()};
    r.correctionDy = units::Length{b.f64()};
    r.correctionDTheta = units::AngleDim{b.f64()};
    r.batteryVoltage = units::Voltage{b.f64()};
    r.batteryCurrent = units::Current{b.f64()};
    for (auto& v : r.tickPhase) {
        v = units::Time{b.f64()};
    }
    return b.ok() && b.offset() == kTickPayloadBytes;
}

// ── The summary frame ───────────────────────────────────────────────────────────────
//   0 motionsStarted    20 gatingRejects    44 worstLoopDt      72 batteryStart
//   4 motionsSettled    24 hasHeadingData   52 firstFaultTime   80 batteryEnd
//   8 motionsTimedOut   25 brownout         60 droppedRecords   88 buildHash[48]
//  12 motionsCancelled  26 firstFault(u16)  64 droppedLines    136 routineId[32]
//  16 motionsAborted    28 headingMax       68 blackboxDropped 168 = end
//                       36 headingFinal

/// Encode one RunSummary. `blackboxDropped` is the SINK's own drop count, passed in
/// rather than read from the summary so the file always carries the writer's live
/// figure even when the caller assembled the summary before the last drop.
/// Returns the bytes written, or 0 on a layout/space failure.
[[nodiscard]] inline std::size_t encodeSummary(std::span<std::byte> out, const RunSummary& s,
                                               std::uint32_t blackboxDropped) noexcept {
    if (out.size() < kSummaryPayloadBytes) {
        return 0U;  // whole or nothing (see encodeHeader)
    }
    ByteWriter w{out};
    w.i32(s.motionsStarted);
    w.i32(s.motionsSettled);
    w.i32(s.motionsTimedOut);
    w.i32(s.motionsCancelled);
    w.i32(s.motionsAborted);
    w.i32(s.gatingRejects);
    w.boolean(s.hasHeadingData);
    w.boolean(s.brownout);
    w.u16(static_cast<std::uint16_t>(s.firstFault));
    w.f64(s.headingMax.value());
    w.f64(s.headingFinal.value());
    w.f64(s.worstLoopDt.value());
    w.f64(s.firstFaultTime.value());
    w.u32(s.droppedRecords);
    w.u32(s.droppedLines);
    w.u32(blackboxDropped);
    w.f64(s.batteryStart.value());
    w.f64(s.batteryEnd.value());
    w.text(s.buildHash(), 48);
    w.text(s.routineId(), 32);
    return w.ok() && w.offset() == kSummaryPayloadBytes ? w.offset() : 0U;
}

/// Decode one RunSummary; `blackboxDropped` receives the sink's own drop count.
/// Returns false if the payload is not exactly kSummaryPayloadBytes.
[[nodiscard]] inline bool decodeSummary(std::span<const std::byte> in, RunSummary& s,
                                        std::uint32_t& blackboxDropped) noexcept {
    if (in.size() != kSummaryPayloadBytes) {
        return false;
    }
    ByteReader b{in};
    s.motionsStarted = b.i32();
    s.motionsSettled = b.i32();
    s.motionsTimedOut = b.i32();
    s.motionsCancelled = b.i32();
    s.motionsAborted = b.i32();
    s.gatingRejects = b.i32();
    s.hasHeadingData = b.boolean();
    s.brownout = b.boolean();
    s.firstFault = static_cast<FaultCode>(b.u16());
    s.headingMax = units::AngleDim{b.f64()};
    s.headingFinal = units::AngleDim{b.f64()};
    s.worstLoopDt = units::Time{b.f64()};
    s.firstFaultTime = units::Time{b.f64()};
    s.droppedRecords = b.u32();
    s.droppedLines = b.u32();
    blackboxDropped = b.u32();
    s.batteryStart = units::Voltage{b.f64()};
    s.batteryEnd = units::Voltage{b.f64()};
    char hash[48] = "";
    char routine[32] = "";
    b.text(hash, sizeof hash, 48);
    b.text(routine, sizeof routine, 32);
    s.setBuildHash(hash);
    s.setRoutineId(routine);
    return b.ok() && b.offset() == kSummaryPayloadBytes;
}

// ── The triage frame (D-7) ──────────────────────────────────────────────────────────
//   0 fault(u16)   4 tickIndex(u32)   16 precedingTicks(u32)   24 the fault tick's
//   2 flags(u8)    8 faultTime(f64)   20 reserved(u32)            full record
//   3 reserved

/// Encode the D-7 triage block plus the complete record of the tick the fault fired
/// on. Returns the bytes written, or 0 on a layout/space failure.
[[nodiscard]] inline std::size_t encodeTriage(std::span<std::byte> out, const TriageInfo& info,
                                              const DebugRecord& faultTick) noexcept {
    if (out.size() < kTriagePayloadBytes) {
        return 0U;  // whole or nothing (see encodeHeader)
    }
    ByteWriter w{out};
    w.u16(static_cast<std::uint16_t>(info.fault));
    w.u8(info.brownout ? 1U : 0U);
    w.u8(0U);
    w.u32(info.tickIndex);
    w.f64(info.faultTime);
    w.u32(info.precedingTicks);
    w.u32(0U);
    if (!w.ok() || w.offset() != 24) {
        return 0U;
    }
    if (out.size() < kTriagePayloadBytes) {
        return 0U;
    }
    const std::size_t n = encodeTick(out.subspan(24, kTickPayloadBytes), faultTick);
    return n == kTickPayloadBytes ? kTriagePayloadBytes : 0U;
}

/// Decode a triage frame and the fault tick's record. Returns false if the payload is
/// not exactly kTriagePayloadBytes.
[[nodiscard]] inline bool decodeTriage(std::span<const std::byte> in, TriageInfo& info,
                                       DebugRecord& faultTick, bool& corrupt) noexcept {
    if (in.size() != kTriagePayloadBytes) {
        return false;
    }
    ByteReader b{in.subspan(0, 24)};
    info.fault = static_cast<FaultCode>(b.u16());
    info.brownout = b.u8() != 0U;
    b.skip(1);
    info.tickIndex = b.u32();
    info.faultTime = b.f64();
    info.precedingTicks = b.u32();
    b.skip(4);
    return b.ok() && decodeTick(in.subspan(24, kTickPayloadBytes), faultTick, corrupt);
}

// ── The end frame ───────────────────────────────────────────────────────────────────
//   0 tickFrames   8 bytesBefore    16 brownout(u8)   18 reserved(u16)
//   4 dropped     12 messagesSeen   17 flags(u8)      20 endTime(f64)   28 = end

/// Encode the graceful-end stamp. Its PRESENCE is the signal that the run closed
/// cleanly; its absence is how a reader knows a file was cut short.
/// Returns the bytes written, or 0 on a layout/space failure.
[[nodiscard]] inline std::size_t encodeEnd(std::span<std::byte> out, const EndInfo& e) noexcept {
    if (out.size() < kEndPayloadBytes) {
        return 0U;  // whole or nothing (see encodeHeader)
    }
    ByteWriter w{out};
    w.u32(e.tickFrames);
    w.u32(e.droppedFrames);
    w.u32(e.bytesBefore);
    w.u32(e.messagesSeen);
    w.boolean(e.brownout);
    w.u8(e.deviceFailed ? 1U : 0U);
    w.u16(0U);
    w.f64(e.endTime);
    return w.ok() && w.offset() == kEndPayloadBytes ? w.offset() : 0U;
}

/// Decode the graceful-end stamp. Returns false if the payload is not exactly
/// kEndPayloadBytes.
[[nodiscard]] inline bool decodeEnd(std::span<const std::byte> in, EndInfo& e) noexcept {
    if (in.size() != kEndPayloadBytes) {
        return false;
    }
    ByteReader b{in};
    e.tickFrames = b.u32();
    e.droppedFrames = b.u32();
    e.bytesBefore = b.u32();
    e.messagesSeen = b.u32();
    e.brownout = b.boolean();
    e.deviceFailed = b.u8() != 0U;
    b.skip(2);
    e.endTime = b.f64();
    return b.ok() && b.offset() == kEndPayloadBytes;
}

/// Write a frame prefix {type, reserved, payloadBytes} into `out`. Returns the bytes
/// written (kFrameHeaderBytes) or 0 if it did not fit.
[[nodiscard]] inline std::size_t encodeFrameHeader(std::span<std::byte> out, FrameType type,
                                                   std::uint16_t payloadBytes) noexcept {
    if (out.size() < kFrameHeaderBytes) {
        return 0U;  // whole or nothing (see encodeHeader)
    }
    ByteWriter w{out};
    w.u8(static_cast<std::uint8_t>(type));
    w.u8(0U);
    w.u16(payloadBytes);
    return w.ok() ? w.offset() : 0U;
}

}  // namespace shulib::diag::blackbox
