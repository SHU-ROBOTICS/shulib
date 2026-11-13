#pragma once
//
// shul2_test_support.hpp — the shared capture builder for the SHUL/2 tests (chunk H1 Part A1):
// test/shul2_format_test.cpp and test/shul2_reader_test.cpp build their captures with the
// FORMAT'S OWN ENCODERS through these helpers, and read them back with independent
// little-endian readers where a golden is the point. Nothing here decides a verdict: every
// helper builds bytes; the reader under test judges them.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

#include "doctest.h"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/shul2_format.hpp"
#include "shulib/diag/shul2_reader.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shul2test {

namespace s2 = shulib::diag::shul2;
namespace bb = shulib::diag::blackbox;
namespace units = shulib::units;
using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::GateReason;
using shulib::diag::RunSummary;
using shulib::diag::SessionInfo;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using Bytes = std::vector<std::byte>;

/// The raw byte of a frame type.
inline std::uint8_t typeByte(s2::FrameType t) { return static_cast<std::uint8_t>(t); }

/// Independent little-endian readers (never ByteReader: a golden read with the code it
/// checks proves only self-consistency — the blackbox_format_test.cpp precedent).
inline std::uint16_t le16(std::span<const std::byte> b, std::size_t at) {
    return static_cast<std::uint16_t>(static_cast<unsigned>(b[at])
                                      | (static_cast<unsigned>(b[at + 1]) << 8U));
}
inline std::uint32_t le32(std::span<const std::byte> b, std::size_t at) {
    std::uint32_t v = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        v |= static_cast<std::uint32_t>(b[at + i]) << (8U * i);
    }
    return v;
}
inline std::uint64_t le64(std::span<const std::byte> b, std::size_t at) {
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        v |= static_cast<std::uint64_t>(b[at + i]) << (8U * i);
    }
    return v;
}
/// An independent little-endian writer, for patching a payload by hand.
inline void putLe(Bytes& b, std::size_t at, std::uint64_t v, std::size_t width) {
    for (std::size_t i = 0; i < width; ++i) {
        b[at + i] = static_cast<std::byte>((v >> (8U * i)) & 0xFFU);
    }
}
/// The bit pattern of a double (for bit-exact comparison: == fails on NaN and cannot see -0).
inline std::uint64_t bitsOf(double v) {
    std::uint64_t u = 0;
    std::memcpy(&u, &v, sizeof u);
    return u;
}
/// A double from its bit pattern.
inline double fromBits(std::uint64_t u) {
    double v = 0.0;
    std::memcpy(&v, &u, sizeof v);
    return v;
}
/// Bytes of a string, for text payloads.
inline Bytes bytesOf(std::string_view s) {
    Bytes b(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        b[i] = static_cast<std::byte>(static_cast<unsigned char>(s[i]));
    }
    return b;
}

/// The provenance the standard Hello carries.
inline SessionInfo standardInfo() {
    SessionInfo info;
    info.buildHash = "0b4948a-dirty";
    info.routineId = "redLeftTall";
    info.alliance = "red";
    info.side = "left";
    info.portMap = "L1,2,3 R4,5,6 IMU10";
    return info;
}

/// This build's Hello payload.
inline Bytes helloPayload(const SessionInfo& info = standardInfo(), double epoch = 100.25,
                          std::uint8_t role = s2::kRoleRuntime) {
    Bytes p(s2::kHelloPayloadBytes);
    REQUIRE(s2::encodeHello(p, info, epoch, role) == s2::kHelloPayloadBytes);
    return p;
}

/// A record with a DIFFERENT value in every field (so a swap cannot pass), shifted by `k`.
inline DebugRecord record(double k) {
    DebugRecord r;
    r.t = units::Time{100.5 + k};
    r.dt = units::Time{0.01 + k};
    r.targetPose = Pose2d{units::Length{24.0 + k}, units::Length{-36.5 + k}, Angle::radians(1.25)};
    r.measuredPose = Pose2d{units::Length{23.25 + k}, units::Length{-36.0 + k}, Angle::radians(-0.75)};
    r.errorX = units::Length{0.75 + k};
    r.errorY = units::Length{-0.5 + k};
    r.errorHeading = units::AngleDim{0.125 + k};
    r.commanded = ChassisSpeeds{units::Velocity{18.5 + k}, units::Velocity{-4.25 + k},
                                units::AngularVelocity{0.875 + k}};
    r.wheelCount = 6;
    for (std::size_t i = 0; i < r.wheelVoltage.size(); ++i) {
        r.wheelVoltage[i] = units::Voltage{static_cast<double>(i) + 0.5 + k};
        r.wheelCurrent[i] = units::Current{static_cast<double>(i) * 0.25 - 1.0 + k};
    }
    r.imuYaw = Angle::radians(2.5);
    r.imuYawRate = units::AngularVelocity{-1.5 + k};
    r.activeCommandId = 7;
    r.activeCommandState = 2;
    r.deadReckoning = true;
    r.qualityClass = 2;
    r.quality = 0.875;
    r.covarianceTrace = 3.25 + k;
    r.gateResidualX = units::Length{1.5 + k};
    r.gateResidualY = units::Length{-2.5 + k};
    r.gateResidualHeading = units::AngleDim{0.0625 + k};
    r.gateMahalanobis = 4.75 + k;
    r.gateReason = GateReason::RejectedMahalanobis;
    r.correctionDx = units::Length{0.25 + k};
    r.correctionDy = units::Length{-0.125 + k};
    r.correctionDTheta = units::AngleDim{0.03125 + k};
    r.clampedThisTick = true;
    r.strafeFallbackActive = false;
    r.tankSteerActive = true;
    r.fault = FaultCode::OdoStuck;
    r.batteryVoltage = units::Voltage{12.4 + k};
    r.batteryCurrent = units::Current{5.5 + k};
    r.droppedRecords = 41;
    r.droppedLines = 42;
    for (std::size_t i = 0; i < r.tickPhase.size(); ++i) {
        r.tickPhase[i] = units::Time{0.001 * static_cast<double>(i + 1) + k};
    }
    return r;
}

/// A Tick payload (blackbox::encodeTick's bytes — D2: the wire has no encoder of its own).
inline Bytes tickPayload(const DebugRecord& r) {
    Bytes p(bb::kTickPayloadBytes);
    REQUIRE(bb::encodeTick(p, r) == bb::kTickPayloadBytes);
    return p;
}

/// A Log payload.
inline Bytes logPayload(const s2::LogLine& line) {
    Bytes p(s2::kLogFixedBytes + line.tag.size() + line.message.size());
    REQUIRE(s2::encodeLog(p, line) == p.size());
    return p;
}

/// A Log payload from level, tag, message and t.
inline Bytes logPayload(std::uint8_t level, std::string_view tag, std::string_view message, double t) {
    s2::LogLine line;
    line.level = level;
    line.tag = tag;
    line.message = message;
    line.t = t;
    return logPayload(line);
}

/// An End payload.
inline Bytes endPayload(const s2::EndInfo& e) {
    Bytes p(s2::kEndPayloadBytes);
    REQUIRE(s2::encodeEnd(p, e) == s2::kEndPayloadBytes);
    return p;
}

/// An End payload whose framesOffered is `seq` (the End's own sequence number, as a writer
/// sends it) and whose framesFailed is `failed`.
inline Bytes endPayloadAt(std::uint32_t seq, std::uint32_t failed = 0, double endTime = 130.5) {
    s2::EndInfo e;
    e.framesOffered = seq;
    e.framesFailed = failed;
    e.recordsSeen = 11;
    e.linesSeen = 12;
    e.linesTruncated = 1;
    e.notOpenDropped = 3;
    e.endTime = endTime;
    return endPayload(e);
}

/// A Summary payload (blackbox::encodeSummary's bytes), carrying the summary's own
/// blackboxDropped figure.
inline Bytes summaryPayload(const RunSummary& s) {
    Bytes p(bb::kSummaryPayloadBytes);
    REQUIRE(bb::encodeSummary(p, s, s.blackboxDropped) == bb::kSummaryPayloadBytes);
    return p;
}

/// One whole frame.
inline Bytes frameOf(std::uint8_t type, std::uint32_t seq, const Bytes& payload, std::uint8_t flags = 0) {
    Bytes out(s2::kFrameOverheadBytes + payload.size());
    REQUIRE(s2::encodeFrame(out, type, seq, payload, flags) == out.size());
    return out;
}

/// A capture being built: its bytes, and where each frame starts (with its seq and type).
struct Capture {
    Bytes bytes;                        ///< the capture
    std::vector<std::size_t> starts;    ///< byte offset of each frame added
    std::vector<std::uint32_t> seqs;    ///< each frame's sequence number
    std::vector<std::uint8_t> types;    ///< each frame's raw type
    std::uint32_t nextSeq = 0;          ///< the sequence number add() uses next

    /// Append a frame at an explicit sequence number (nextSeq follows it).
    void addAt(std::uint8_t type, std::uint32_t seq, const Bytes& payload, std::uint8_t flags = 0) {
        starts.push_back(bytes.size());
        seqs.push_back(seq);
        types.push_back(type);
        const Bytes f = frameOf(type, seq, payload, flags);
        bytes.insert(bytes.end(), f.begin(), f.end());
        nextSeq = seq + 1U;
    }
    /// Append a frame at the next sequence number.
    void add(s2::FrameType type, const Bytes& payload, std::uint8_t flags = 0) {
        addAt(typeByte(type), nextSeq, payload, flags);
    }
    /// Append raw bytes (garbage, a cut frame) that are not a frame.
    void raw(const Bytes& b) { bytes.insert(bytes.end(), b.begin(), b.end()); }
    /// Byte length of frame i.
    [[nodiscard]] std::size_t frameBytes(std::size_t i) const {
        return (i + 1 < starts.size() ? starts[i + 1] : bytes.size()) - starts[i];
    }
};

/// A standard session: Hello(0), `ticks` Ticks, a Log after every tenth tick, then an End
/// whose framesOffered is its own seq (when `end`).
inline Capture standardSession(int ticks, bool end = true) {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < ticks; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
        if (i % 10 == 9) {
            c.add(s2::FrameType::Log, logPayload(2, "MOT", "moveTo settled", 100.5 + i));
        }
    }
    if (end) {
        c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    }
    return c;
}

/// A delivered frame, copied out of the reader.
struct Delivered {
    s2::FrameType type = s2::FrameType::Tick;  ///< its type
    std::uint8_t flags = 0;                    ///< its envelope flags
    std::uint32_t seq = 0;                     ///< its sequence number
    Bytes payload;                             ///< its whole payload
};

/// Drain a reader, copying every delivered frame.
inline std::vector<Delivered> readAll(s2::Shul2Reader& reader) {
    std::vector<Delivered> out;
    s2::Shul2Reader::Frame f;
    while (reader.next(f)) {
        out.push_back(Delivered{f.type, f.flags, f.seq, Bytes(f.payload.begin(), f.payload.end())});
    }
    REQUIRE(reader.finished());
    return out;
}

/// Field-by-field equality of two records, f64s by BIT PATTERN.
inline void checkSameRecordBits(const DebugRecord& a, const DebugRecord& b) {
    CHECK(bitsOf(a.t.value()) == bitsOf(b.t.value()));
    CHECK(bitsOf(a.dt.value()) == bitsOf(b.dt.value()));
    CHECK(bitsOf(a.targetPose.x().value()) == bitsOf(b.targetPose.x().value()));
    CHECK(bitsOf(a.targetPose.y().value()) == bitsOf(b.targetPose.y().value()));
    CHECK(bitsOf(a.targetPose.heading().radians()) == bitsOf(b.targetPose.heading().radians()));
    CHECK(bitsOf(a.measuredPose.x().value()) == bitsOf(b.measuredPose.x().value()));
    CHECK(bitsOf(a.measuredPose.y().value()) == bitsOf(b.measuredPose.y().value()));
    CHECK(bitsOf(a.measuredPose.heading().radians()) == bitsOf(b.measuredPose.heading().radians()));
    CHECK(bitsOf(a.errorX.value()) == bitsOf(b.errorX.value()));
    CHECK(bitsOf(a.errorY.value()) == bitsOf(b.errorY.value()));
    CHECK(bitsOf(a.errorHeading.value()) == bitsOf(b.errorHeading.value()));
    CHECK(bitsOf(a.commanded.vx().value()) == bitsOf(b.commanded.vx().value()));
    CHECK(bitsOf(a.commanded.vy().value()) == bitsOf(b.commanded.vy().value()));
    CHECK(bitsOf(a.commanded.omega().value()) == bitsOf(b.commanded.omega().value()));
    CHECK(a.wheelCount == b.wheelCount);
    for (std::size_t i = 0; i < a.wheelVoltage.size(); ++i) {
        CHECK(bitsOf(a.wheelVoltage[i].value()) == bitsOf(b.wheelVoltage[i].value()));
        CHECK(bitsOf(a.wheelCurrent[i].value()) == bitsOf(b.wheelCurrent[i].value()));
    }
    CHECK(bitsOf(a.imuYaw.radians()) == bitsOf(b.imuYaw.radians()));
    CHECK(bitsOf(a.imuYawRate.value()) == bitsOf(b.imuYawRate.value()));
    CHECK(a.activeCommandId == b.activeCommandId);
    CHECK(a.activeCommandState == b.activeCommandState);
    CHECK(a.deadReckoning == b.deadReckoning);
    CHECK(a.qualityClass == b.qualityClass);
    CHECK(bitsOf(a.quality) == bitsOf(b.quality));
    CHECK(bitsOf(a.covarianceTrace) == bitsOf(b.covarianceTrace));
    CHECK(bitsOf(a.gateResidualX.value()) == bitsOf(b.gateResidualX.value()));
    CHECK(bitsOf(a.gateResidualY.value()) == bitsOf(b.gateResidualY.value()));
    CHECK(bitsOf(a.gateResidualHeading.value()) == bitsOf(b.gateResidualHeading.value()));
    CHECK(bitsOf(a.gateMahalanobis) == bitsOf(b.gateMahalanobis));
    CHECK(a.gateReason == b.gateReason);
    CHECK(bitsOf(a.correctionDx.value()) == bitsOf(b.correctionDx.value()));
    CHECK(bitsOf(a.correctionDy.value()) == bitsOf(b.correctionDy.value()));
    CHECK(bitsOf(a.correctionDTheta.value()) == bitsOf(b.correctionDTheta.value()));
    CHECK(a.clampedThisTick == b.clampedThisTick);
    CHECK(a.strafeFallbackActive == b.strafeFallbackActive);
    CHECK(a.tankSteerActive == b.tankSteerActive);
    CHECK(a.fault == b.fault);
    CHECK(bitsOf(a.batteryVoltage.value()) == bitsOf(b.batteryVoltage.value()));
    CHECK(bitsOf(a.batteryCurrent.value()) == bitsOf(b.batteryCurrent.value()));
    CHECK(a.droppedRecords == b.droppedRecords);
    CHECK(a.droppedLines == b.droppedLines);
    for (std::size_t i = 0; i < a.tickPhase.size(); ++i) {
        CHECK(bitsOf(a.tickPhase[i].value()) == bitsOf(b.tickPhase[i].value()));
    }
}

/// A test-side LIVE reassembler (what H2's socket loop will be): it buffers bytes, calls
/// scanFrame, waits on NeedMore, and — at the end of input only — applies the whole-capture
/// tail rule (a NeedMore is a failed candidate: drop one byte). It records every VERIFIED frame
/// and how many bytes had been fed when it was emitted.
struct Reassembler {
    /// One verified frame, copied out.
    struct Out {
        std::uint8_t type = 0;       ///< raw type
        std::uint32_t seq = 0;       ///< sequence number
        Bytes payload;               ///< whole payload
        std::size_t fedWhenEmitted = 0;  ///< bytes fed so far when it was emitted
    };
    Bytes buf;                       ///< bytes held
    std::size_t head = 0;            ///< start of the unscanned bytes in buf
    std::size_t fed = 0;             ///< bytes fed so far
    std::size_t skipped = 0;         ///< bytes dropped without a verified frame
    std::uint32_t crcFailures = 0;   ///< CRC failures seen
    std::vector<Out> frames;         ///< verified frames in order

    void compact() {
        if (head > 8192) {
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(head));
            head = 0;
        }
    }
    /// Scan what is held; returns on NeedMore (a live reader waits for more bytes).
    void scan(bool endOfInput) {
        while (head < buf.size()) {
            const std::span<const std::byte> held{buf.data() + head, buf.size() - head};
            const s2::ScanResult r = s2::scanFrame(held);
            if (r.kind == s2::ScanKind::NeedMore) {
                if (!endOfInput) {
                    return;
                }
                ++skipped;  // the tail rule: the input is over, so this candidate failed
                ++head;
                continue;
            }
            if (r.kind == s2::ScanKind::Skip) {
                if (r.reason == s2::SkipReason::CrcMismatch) {
                    ++crcFailures;
                }
                skipped += r.consumed;
                head += r.consumed;
                continue;
            }
            frames.push_back(Out{r.type, r.seq, Bytes(r.payload.begin(), r.payload.end()), fed});
            head += r.consumed;
        }
        compact();
    }
    /// Feed a chunk of a live stream.
    void feed(std::span<const std::byte> chunk) {
        buf.insert(buf.end(), chunk.begin(), chunk.end());
        fed += chunk.size();
        scan(false);
    }
    /// The input is over: apply the whole-capture tail rule to what is left.
    void finish() { scan(true); }
};

}  // namespace shul2test
