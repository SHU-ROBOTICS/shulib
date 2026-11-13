// Tests for the SHUL/2 wire FORMAT — diag/crc32.hpp and diag/shul2_format.hpp (chunk H1
// Part A1). Its reader's tests are test/shul2_reader_test.cpp. What each targets:
//
//  * THE BYTE-EXACT GOLDENS: a round trip is structurally BLIND to a field moved in both the
//    encoder and the decoder (E1's mutation 1b; this chunk's mutation 26 re-measures it). So
//    the envelope, the Hello, the Log, the End and a whole Tick frame are pinned against bytes
//    derived BY HAND from the layout tables and read back with INDEPENDENT little-endian
//    readers, and every CRC literal was computed OUTSIDE this code with Python's zlib (the
//    commands are in the H1 PROGRESS log). Never captured from the encoder.
//  * THE ROUND TRIP: every frame type field for field; every f64 bit-exact (NaN, ±Inf, −0.0,
//    denormals) except the three math::Angle fields, which refuse non-finite values and decode
//    one as 0 with `corrupt` raised; every vocabulary value, known or not, as its number.
//  * THE NEGOTIATION: the whole version table under this build's 1.0 profile AND a
//    hypothetical 1.1 profile, and the signature's alterations.
//  * The CRC's published check values, the wire level codes, the UTF-8-safe cut, the
//    wheelCount clamp (E1's decoder, fixed at its source), the encoders' whole-or-nothing, the
//    decoders' refusal of a short payload, a Hello whose text fills its fields to the width, and
//    another major's Hello (decoded in its invariant bytes only).
//
// The H1 Part A1 review (2026-09-29) added the cases and lines that name a review mutation
// (R… / N…): each one's mutant survived the first version of this suite.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "shul2_test_support.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/diag/crc32.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/version.hpp"

using namespace shul2test;
using shulib::hal::LogLevel;

namespace {

std::uint32_t crcOf(std::string_view s) {
    return shulib::diag::crc32Ieee(std::as_bytes(std::span<const char>{s.data(), s.size()}));
}

/// The text of a NUL-padded field, read independently of the decoder.
std::string textAt(std::span<const std::byte> b, std::size_t at, std::size_t width) {
    std::string s;
    for (std::size_t i = 0; i < width; ++i) {
        const auto c = static_cast<char>(b[at + i]);
        if (c == '\0') {
            break;
        }
        s += c;
    }
    return s;
}

/// Read a capture holding ONE session of this build and return its delivered frames.
std::vector<Delivered> readCapture(const Bytes& capture) {
    s2::Shul2Reader reader{capture};
    auto out = readAll(reader);
    CHECK(reader.status() == s2::CaptureStatus::Ok);
    return out;
}

/// Wrap one Tick payload in a Hello … End session and return the payload the reader delivers.
Bytes throughTheWire(const Bytes& tick) {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, tick);
    c.add(s2::FrameType::End, endPayloadAt(2));
    const auto frames = readCapture(c.bytes);
    REQUIRE(frames.size() == 3);
    REQUIRE(frames[1].type == s2::FrameType::Tick);
    return frames[1].payload;
}

}  // namespace

// ── The CRC ─────────────────────────────────────────────────────────────────────────

// Would catch: a wrong polynomial, initial value or final inversion — named here as the
// arithmetic, not as a byte offset inside a frame. The values are zlib's (logged command).
TEST_CASE("SHUL/2 crc32: diag::crc32Ieee gives the published CRC-32/IEEE check values") {
    CHECK(crcOf("") == 0x00000000U);
    CHECK(crcOf("a") == 0xE8B7BE43U);
    CHECK(crcOf("123456789") == 0xCBF43926U);  // the canonical check value
}

// Would catch: the calibration spelling drifting from the one algorithm (a second copy, or a
// forwarding function that forwards somewhere else). CalibrationRecord's own golden and its
// 1b vectors still run unchanged in calibration_record_test.cpp through the forwarder.
TEST_CASE("SHUL/2 crc32: calibration::crc32Ieee forwards to diag::crc32Ieee") {
    Bytes b(300);
    for (std::size_t i = 0; i < b.size(); ++i) {
        b[i] = static_cast<std::byte>((i * 37U + 11U) & 0xFFU);
    }
    CHECK(shulib::calibration::crc32Ieee(b) == shulib::diag::crc32Ieee(b));
    CHECK(shulib::calibration::crc32Ieee(Bytes{}) == 0x00000000U);
}

// ── The goldens ─────────────────────────────────────────────────────────────────────

// Would catch: any envelope field moved, resized or re-ordered, the wrong endianness, and a
// CRC over the wrong range (payload only, or including the sync) — the CRC literal is zlib's
// over bytes [2, 13): python3 -c "import zlib; f=bytes([0xFE,0x53,0x03,0x5A,0x01,0x02,0x03,
// 0x04,0x03,0x00])+b'abc'; print(hex(zlib.crc32(f[2:])))" → 0x2eb168d9.
TEST_CASE("SHUL/2 golden: the envelope, byte for byte") {
    const Bytes f = frameOf(3, 0x04030201U, bytesOf("abc"), 0x5A);
    REQUIRE(f.size() == 17);
    const std::uint8_t head[13] = {0xFE, 0x53, 0x03, 0x5A, 0x01, 0x02, 0x03, 0x04,
                                   0x03, 0x00, 'a', 'b', 'c'};
    for (std::size_t i = 0; i < 13; ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::uint8_t>(f[i]) == head[i]);
    }
    CHECK(le32(f, 13) == 0x2EB168D9U);
    // The constants that name those offsets agree with the table.
    CHECK(s2::kTypeOffset == 2);
    CHECK(s2::kFlagsOffset == 3);
    CHECK(s2::kSequenceOffset == 4);
    CHECK(s2::kPayloadBytesOffset == 8);
    CHECK(s2::kPayloadOffset == 10);
    CHECK(s2::kFrameOverheadBytes == 14);
    CHECK(s2::kMaxPayloadBytes == 4096);
}

// Would catch: a Hello field moved or resized, a width row that stopped matching the layout,
// the API bytes taken from anywhere but version.hpp, text not NUL-padded, reserved bytes not
// zero, and the signature changed (mutation 17 turns its NUL into 0x20). Hand-derived offsets.
TEST_CASE("SHUL/2 golden: the Hello, every field at its offset") {
    const Bytes h = helloPayload(standardInfo(), 12.5, s2::kRoleRuntime);
    REQUIRE(h.size() == 256);
    const std::uint8_t sig[8] = {0x89, 0x53, 0x48, 0x55, 0x4C, 0x00, 0x0D, 0x0A};
    for (std::size_t i = 0; i < 8; ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::uint8_t>(h[i]) == sig[i]);
    }
    CHECK(le16(h, 8) == 1);    // versionMajor
    CHECK(le16(h, 10) == 0);   // versionMinor
    CHECK(static_cast<std::uint8_t>(h[12]) == 0);  // role: runtime
    CHECK(static_cast<std::uint8_t>(h[13]) == 0);  // reserved
    CHECK(le16(h, 14) == 0);   // flags
    CHECK(le16(h, 16) == 428); // tickPayloadBytes
    CHECK(le16(h, 18) == 168); // summaryPayloadBytes
    CHECK(le16(h, 20) == 32);  // endPayloadBytes
    CHECK(le16(h, 22) == 14);  // logFixedBytes
    CHECK(le64(h, 24) == 0x4029000000000000ULL);  // 12.5, hand-derived
    CHECK(static_cast<int>(h[32]) == shulib::kApiMajor);
    CHECK(static_cast<int>(h[33]) == shulib::kApiMinor);
    CHECK(le16(h, 34) == 0);   // reserved
    CHECK(textAt(h, 36, 48) == "0b4948a-dirty");
    CHECK(textAt(h, 84, 32) == "redLeftTall");
    CHECK(textAt(h, 116, 16) == "red");
    CHECK(textAt(h, 132, 16) == "left");
    CHECK(textAt(h, 148, 96) == "L1,2,3 R4,5,6 IMU10");
    for (std::size_t i = 36 + 13; i < 84; ++i) {  // the hash's padding
        CHECK(static_cast<std::uint8_t>(h[i]) == 0U);
    }
    for (std::size_t i = 148 + 19; i < 256; ++i) {  // the port map's padding + the reserved tail
        CHECK(static_cast<std::uint8_t>(h[i]) == 0U);
    }

    // A peer's Hello differs in the role byte only.
    const Bytes peer = helloPayload(standardInfo(), 12.5, s2::kRolePeer);
    CHECK(static_cast<std::uint8_t>(peer[12]) == 1);
    for (std::size_t i = 0; i < 256; ++i) {
        if (i != 12) {
            CAPTURE(i);
            CHECK(peer[i] == h[i]);
        }
    }

    // MISSING stays loud: an empty hash is 48 zero bytes, never a placeholder.
    SessionInfo noHash = standardInfo();
    noHash.buildHash = {};
    const Bytes m = helloPayload(noHash, 12.5);
    for (std::size_t i = 36; i < 84; ++i) {
        CHECK(static_cast<std::uint8_t>(m[i]) == 0U);
    }
}

// Would catch: a Log field moved, the text not at offset 14, the tag and message swapped, and
// the frame's CRC range. The frame CRC is zlib's (logged):
// python3 -c "import zlib,struct; p=bytes([1,2,3,0,2,0])+struct.pack('<d',12.5)+b'MOThi';
// f=bytes([0xFE,0x53,0x03,0x00])+struct.pack('<IH',7,len(p))+p; print(hex(zlib.crc32(f[2:])))"
// → 0xbaf80e96.
TEST_CASE("SHUL/2 golden: the Log") {
    s2::LogLine line;
    line.level = static_cast<std::uint8_t>(s2::wireLevel(LogLevel::Warn));
    line.flags = s2::kLogMessageTruncated;
    line.tag = "MOT";
    line.message = "hi";
    line.t = 12.5;
    const Bytes p = logPayload(line);
    const std::uint8_t expect[19] = {0x01, 0x02, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x29, 0x40, 'M', 'O', 'T', 'h', 'i'};
    REQUIRE(p.size() == 19);
    for (std::size_t i = 0; i < 19; ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::uint8_t>(p[i]) == expect[i]);
    }
    const Bytes f = frameOf(typeByte(s2::FrameType::Log), 7, p);
    CHECK(le32(f, 10 + 19) == 0xBAF80E96U);
}

// Would catch: an End field moved or resized. Hand-derived.
TEST_CASE("SHUL/2 golden: the End") {
    s2::EndInfo e;
    e.framesOffered = 0x04030201U;
    e.framesFailed = 0x08070605U;
    e.recordsSeen = 0x0C0B0A09U;
    e.linesSeen = 0x100F0E0DU;
    e.linesTruncated = 0x14131211U;
    e.notOpenDropped = 0x18171615U;
    e.endTime = 2.0;
    const Bytes p = endPayload(e);
    REQUIRE(p.size() == 32);
    for (std::size_t i = 0; i < 24; ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::size_t>(p[i]) == i + 1);  // the counters' bytes are 1..24 in order
    }
    CHECK(le64(p, 24) == 0x4000000000000000ULL);  // 2.0
}

// Would catch: ANY tick field moved in BOTH encodeTick and decodeTick (mutation 26: the round
// trip stays green, this goes red — the reason the goldens exist), the tick flag bits (bit 2 is
// TANK1's tankSteerActive), the enum bytes (FaultCode 12, GateReason 12, MotionState 5,
// qualityClass 3), and the frame's CRC. Every field is non-zero, and no two fields of one width
// hold the same value — the 51 f64s all differ by bit pattern, and so do the six u8s and the
// three u32s — so a both-sides swap of any two FIELDS changes these bytes. What this golden
// cannot see: a swap of two of the three flag BITS, which share one byte, 0x07 here.
// blackbox_format_test's per-field golden sets each flag alone and does see it. (The first
// version of this golden repeated 1.0, 0.5, 2..8 and -1.0 across fields, so a both-sides swap
// of two equal values was invisible to it; the H1 Part A1 review found that, 2026-09-29.)
// The CRC was computed OUTSIDE this code, in Python, from the layout table — never from the
// encoder. The script, reproduced so anyone can re-derive 0x75a478de:
//   import struct, zlib
//   p = bytearray(428); d = lambda o, v: struct.pack_into('<d', p, o, v)
//   d(0,1.0); d(8,0.375); d(16,2.0); d(24,3.0); d(32,0.5); d(40,4.0); d(48,5.0); d(56,-0.5)
//   d(64,6.0); d(72,7.0); d(80,0.25); d(88,8.0); d(96,9.0); d(104,0.125)
//   p[112]=2; p[113]=5; p[114]=1; p[115]=3; struct.pack_into('<I', p, 116, 0x01020304)
//   struct.pack_into('<H', p, 120, 12); p[122]=12; p[123]=0x07
//   struct.pack_into('<I', p, 124, 41); struct.pack_into('<I', p, 128, 42)
//   for i in range(8): d(132+8*i, 101.0+i); d(196+8*i, -(101.0+i)); d(364+8*i, 17.0+i)
//   d(260,1.5); d(268,-2.5); d(276,0.75); d(284,10.0); d(292,11.0); d(300,12.0)
//   d(308,0.0625); d(316,13.0); d(324,14.0); d(332,15.0); d(340,0.03125); d(348,12.5); d(356,16.0)
//   f = bytes([0xFE,0x53,0x02,0x00]) + struct.pack('<IH', 258, 428) + bytes(p)
//   print(hex(zlib.crc32(f[2:])))   # -> 0x75a478de
TEST_CASE("SHUL/2 golden: a whole Tick frame, CRC included") {
    DebugRecord r;
    r.t = units::Time{1.0};
    r.dt = units::Time{0.375};
    r.targetPose = Pose2d{units::Length{2.0}, units::Length{3.0}, Angle::radians(0.5)};
    r.measuredPose = Pose2d{units::Length{4.0}, units::Length{5.0}, Angle::radians(-0.5)};
    r.errorX = units::Length{6.0};
    r.errorY = units::Length{7.0};
    r.errorHeading = units::AngleDim{0.25};
    r.commanded = ChassisSpeeds{units::Velocity{8.0}, units::Velocity{9.0}, units::AngularVelocity{0.125}};
    r.wheelCount = 2;
    r.activeCommandState = 5;  // motion::MotionState::Cancelled
    r.deadReckoning = true;
    r.qualityClass = 3;        // Localizer::Quality::Degraded
    r.activeCommandId = 0x01020304U;
    r.fault = FaultCode::MotorGroupDisagree;     // 12
    r.gateReason = GateReason::CovarianceReinit;  // 12
    r.clampedThisTick = true;
    r.strafeFallbackActive = true;
    r.tankSteerActive = true;
    r.droppedRecords = 41;
    r.droppedLines = 42;
    for (std::size_t i = 0; i < 8; ++i) {
        r.wheelVoltage[i] = units::Voltage{static_cast<double>(101 + i)};
        r.wheelCurrent[i] = units::Current{-static_cast<double>(101 + i)};
        r.tickPhase[i] = units::Time{static_cast<double>(17 + i)};
    }
    r.imuYaw = Angle::radians(1.5);
    r.imuYawRate = units::AngularVelocity{-2.5};
    r.quality = 0.75;
    r.covarianceTrace = 10.0;
    r.gateResidualX = units::Length{11.0};
    r.gateResidualY = units::Length{12.0};
    r.gateResidualHeading = units::AngleDim{0.0625};
    r.gateMahalanobis = 13.0;
    r.correctionDx = units::Length{14.0};
    r.correctionDy = units::Length{15.0};
    r.correctionDTheta = units::AngleDim{0.03125};
    r.batteryVoltage = units::Voltage{12.5};
    r.batteryCurrent = units::Current{16.0};

    const Bytes f = frameOf(typeByte(s2::FrameType::Tick), 258, tickPayload(r));
    REQUIRE(f.size() == 442);

    // The expected frame, written from the layout table by an independent writer.
    Bytes e(442);
    const std::uint8_t env[10] = {0xFE, 0x53, 0x02, 0x00, 0x02, 0x01, 0x00, 0x00, 0xAC, 0x01};
    for (std::size_t i = 0; i < 10; ++i) {
        e[i] = static_cast<std::byte>(env[i]);
    }
    const auto f64At = [&e](std::size_t off, std::uint64_t bits) { putLe(e, 10 + off, bits, 8); };
    f64At(0, 0x3FF0000000000000ULL);    // t 1.0
    f64At(8, 0x3FD8000000000000ULL);    // dt 0.375 (1.5 × 2^-2)
    f64At(16, 0x4000000000000000ULL);   // targetPose.x 2.0
    f64At(24, 0x4008000000000000ULL);   // targetPose.y 3.0
    f64At(32, 0x3FE0000000000000ULL);   // targetPose.heading 0.5
    f64At(40, 0x4010000000000000ULL);   // measuredPose.x 4.0
    f64At(48, 0x4014000000000000ULL);   // measuredPose.y 5.0
    f64At(56, 0xBFE0000000000000ULL);   // measuredPose.heading -0.5
    f64At(64, 0x4018000000000000ULL);   // errorX 6.0
    f64At(72, 0x401C000000000000ULL);   // errorY 7.0
    f64At(80, 0x3FD0000000000000ULL);   // errorHeading 0.25
    f64At(88, 0x4020000000000000ULL);   // commanded.vx 8.0
    f64At(96, 0x4022000000000000ULL);   // commanded.vy 9.0
    f64At(104, 0x3FC0000000000000ULL);  // commanded.omega 0.125
    putLe(e, 10 + 112, 2, 1);           // wheelCount
    putLe(e, 10 + 113, 5, 1);           // activeCommandState
    putLe(e, 10 + 114, 1, 1);           // deadReckoning
    putLe(e, 10 + 115, 3, 1);           // qualityClass
    putLe(e, 10 + 116, 0x01020304U, 4); // activeCommandId
    putLe(e, 10 + 120, 12, 2);          // fault
    putLe(e, 10 + 122, 12, 1);          // gateReason
    putLe(e, 10 + 123, 0x07, 1);        // flags: bits 0, 1, 2
    putLe(e, 10 + 124, 41, 4);          // droppedRecords
    putLe(e, 10 + 128, 42, 4);          // droppedLines
    // 101..108 = (1 + k/64) × 2^6 for k = 37..44: exponent 0x405, one step of k is 2^46.
    const std::uint64_t wheels[8] = {0x4059400000000000ULL, 0x4059800000000000ULL,
                                     0x4059C00000000000ULL, 0x405A000000000000ULL,
                                     0x405A400000000000ULL, 0x405A800000000000ULL,
                                     0x405AC00000000000ULL, 0x405B000000000000ULL};  // 101..108
    for (std::size_t i = 0; i < 8; ++i) {
        f64At(132 + 8 * i, wheels[i]);                          // wheelVoltage 101..108
        f64At(196 + 8 * i, wheels[i] | 0x8000000000000000ULL);  // wheelCurrent -101..-108
        f64At(364 + 8 * i, 0x4031000000000000ULL + (static_cast<std::uint64_t>(i) << 48U));  // 17..24
    }
    f64At(260, 0x3FF8000000000000ULL);  // imuYaw 1.5
    f64At(268, 0xC004000000000000ULL);  // imuYawRate -2.5 (-1.25 × 2^1)
    f64At(276, 0x3FE8000000000000ULL);  // quality 0.75
    f64At(284, 0x4024000000000000ULL);  // covarianceTrace 10
    f64At(292, 0x4026000000000000ULL);  // gateResidualX 11
    f64At(300, 0x4028000000000000ULL);  // gateResidualY 12
    f64At(308, 0x3FB0000000000000ULL);  // gateResidualHeading 0.0625
    f64At(316, 0x402A000000000000ULL);  // gateMahalanobis 13
    f64At(324, 0x402C000000000000ULL);  // correctionDx 14
    f64At(332, 0x402E000000000000ULL);  // correctionDy 15
    f64At(340, 0x3FA0000000000000ULL);  // correctionDTheta 0.03125
    f64At(348, 0x4029000000000000ULL);  // batteryVoltage 12.5
    f64At(356, 0x4030000000000000ULL);  // batteryCurrent 16
    putLe(e, 438, 0x75A478DEU, 4);      // the CRC, from zlib (never from crc32Ieee)

    for (std::size_t i = 0; i < 442; ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::uint8_t>(f[i]) == static_cast<std::uint8_t>(e[i]));
    }
}

// Would catch: two wire level codes swapped (mutation 13) and a mapping written as a cast.
TEST_CASE("SHUL/2 golden: the wire level codes, both directions") {
    CHECK(s2::wireLevel(LogLevel::Error) == s2::WireLevel::Error);
    CHECK(s2::wireLevel(LogLevel::Warn) == s2::WireLevel::Warn);
    CHECK(s2::wireLevel(LogLevel::Info) == s2::WireLevel::Info);
    CHECK(s2::wireLevel(LogLevel::Debug) == s2::WireLevel::Debug);
    CHECK(s2::wireLevel(LogLevel::Trace) == s2::WireLevel::Trace);
    CHECK(static_cast<int>(s2::WireLevel::Error) == 0);
    CHECK(static_cast<int>(s2::WireLevel::Warn) == 1);
    CHECK(static_cast<int>(s2::WireLevel::Info) == 2);
    CHECK(static_cast<int>(s2::WireLevel::Debug) == 3);
    CHECK(static_cast<int>(s2::WireLevel::Trace) == 4);
    const LogLevel levels[5] = {LogLevel::Error, LogLevel::Warn, LogLevel::Info, LogLevel::Debug,
                                LogLevel::Trace};
    for (std::uint8_t code = 0; code < 5; ++code) {
        LogLevel out = LogLevel::Info;
        CHECK(s2::logLevelFromWire(code, out));
        CHECK(out == levels[code]);
    }
}

// Would catch: an unknown level code turned into a LogLevel by a cast (mutation 14) — which
// would invent a level the enum does not have.
TEST_CASE("SHUL/2: an unknown wire level code maps to \"unknown\", never by cast") {
    for (const std::uint8_t code : {std::uint8_t{5}, std::uint8_t{7}, std::uint8_t{255}}) {
        CAPTURE(static_cast<int>(code));
        LogLevel out = LogLevel::Debug;
        CHECK_FALSE(s2::logLevelFromWire(code, out));
        CHECK(out == LogLevel::Debug);  // untouched
    }
}

// ── The UTF-8-safe cut ──────────────────────────────────────────────────────────────

// Would catch: a cut that splits a multi-byte sequence (mutation 15) — an invalid UTF-8 tail
// on the wire — and a cut that trims text which already fits.
TEST_CASE("SHUL/2: utf8Prefix never splits a sequence and never trims what fits") {
    const std::string_view euro = "ab\xE2\x82\xAC";  // "ab€": € is 3 bytes
    CHECK(s2::utf8Prefix(euro, 5) == euro);
    CHECK(s2::utf8Prefix(euro, 9) == euro);
    CHECK(s2::utf8Prefix(euro, 4) == "ab");
    CHECK(s2::utf8Prefix(euro, 3) == "ab");
    CHECK(s2::utf8Prefix(euro, 2) == "ab");
    CHECK(s2::utf8Prefix(euro, 1) == "a");
    CHECK(s2::utf8Prefix(euro, 0).empty());
    const std::string_view emoji = "x\xF0\x9F\x98\x80y";  // x😀y: the emoji is 4 bytes
    CHECK(s2::utf8Prefix(emoji, 4) == "x");
    CHECK(s2::utf8Prefix(emoji, 5) == "x\xF0\x9F\x98\x80");
    // The same view, not a copy, when it fits.
    CHECK(s2::utf8Prefix(euro, 5).data() == euro.data());

    // A view that EXACTLY fits is returned whole, without a look at the byte after it (review
    // mutation N14: `<` for `<=`). The literals above end in a NUL, which hides that look; here
    // the view is cut out of a longer buffer whose next byte is a continuation byte, so a cut at
    // `cap` would back off over it and lose a byte that fits.
    const std::string_view buffer = "ab\x80";
    const std::string_view view = buffer.substr(0, 2);
    CHECK(s2::utf8Prefix(view, 2) == "ab");
    CHECK(s2::utf8Prefix(view, 2).data() == view.data());
}

// ── Round trips ─────────────────────────────────────────────────────────────────────

// Would catch: a Hello field dropped or mis-decoded, text views that run past their NUL, and
// the API bytes decoded from the wrong offset.
TEST_CASE("SHUL/2 round trip: the Hello, field for field") {
    const Bytes p = helloPayload(standardInfo(), 1234.5, s2::kRolePeer);
    s2::Hello h;
    REQUIRE(s2::decodeHello(p, h));
    CHECK(h.complete);
    CHECK(h.payloadBytes == 256);
    for (std::size_t i = 0; i < 8; ++i) {
        CHECK(h.signature[i] == s2::kSignature[i]);
    }
    CHECK(h.versionMajor == 1);
    CHECK(h.versionMinor == 0);
    CHECK(h.role == 1);
    CHECK(h.flags == 0);
    CHECK(h.declared.hello == 256);
    CHECK(h.declared.tick == 428);
    CHECK(h.declared.summary == 168);
    CHECK(h.declared.end == 32);
    CHECK(h.declared.logFixed == 14);
    CHECK(h.epochSeconds == 1234.5);
    CHECK(static_cast<int>(h.apiMajor) == shulib::kApiMajor);
    CHECK(static_cast<int>(h.apiMinor) == shulib::kApiMinor);
    CHECK(h.buildHash == "0b4948a-dirty");
    CHECK(h.routineId == "redLeftTall");
    CHECK(h.alliance == "red");
    CHECK(h.side == "left");
    CHECK(h.portMap == "L1,2,3 R4,5,6 IMU10");
    CHECK(s2::negotiate(h, s2::kThisReader) == s2::Verdict::Exact);

    // A text longer than its field is cut by the ENCODER: ByteWriter::text keeps at most width − 1
    // bytes and NUL-pads, so every field this build writes ends at a NUL. (The decoder's own width
    // bound — a field that FILLS its width with no NUL — is the next case's: this build's Hellos
    // cannot exercise it.)
    const std::string longRoutine(40, 'r');
    SessionInfo info = standardInfo();
    info.routineId = longRoutine;
    const Bytes q = helloPayload(info, 1.0);
    s2::Hello hq;
    REQUIRE(s2::decodeHello(q, hq));
    CHECK(hq.routineId == std::string(31, 'r'));  // width − 1, then the NUL
    CHECK(static_cast<std::uint8_t>(q[84 + 31]) == 0U);
    CHECK(hq.alliance == "red");
}

// Would catch: a text field's width bound read off by one (the view takes the next field's first
// byte — review mutation N4) or dropped (the view runs on through the reserved tail, the CRC and
// into the next frame — N4b: foreign bytes shown as provenance). This build's encoder never fills
// a field (previous case), so the Hello here is HAND-BUILT: every text field filled to its width
// with no NUL (each with its own letter, so a view that crosses into the next field shows), the
// reserved tail 0xFF (never judged), then framed — the CRC recomputed, because these are a
// writer's bytes, not damage. It is decoded directly AND through the reader, whose report views
// the capture itself, so an over-read runs into real neighbouring bytes, never off a copy's end.
TEST_CASE("SHUL/2 round trip: a Hello whose text fields fill their widths is read to the width, no further") {
    Bytes h = helloPayload();
    struct Field {
        std::size_t at;
        std::size_t width;
        char letter;
    };
    const Field fields[5] = {{36, 48, 'H'}, {84, 32, 'R'}, {116, 16, 'A'}, {132, 16, 'S'}, {148, 96, 'P'}};
    for (const Field& f : fields) {
        for (std::size_t i = 0; i < f.width; ++i) {
            h[f.at + i] = static_cast<std::byte>(f.letter);
        }
    }
    for (std::size_t i = 244; i < 256; ++i) {
        h[i] = std::byte{0xFF};
    }
    Capture c;
    c.add(s2::FrameType::Hello, h);
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    s2::Shul2Reader reader{c.bytes};
    REQUIRE(readAll(reader).size() == 2);
    CHECK(reader.latestSession().verdict == s2::Verdict::Exact);
    s2::Hello direct;
    REQUIRE(s2::decodeHello(std::span<const std::byte>{c.bytes}.subspan(c.starts[0] + 10, 256), direct));
    const s2::Hello* decoded[2] = {&reader.latestSession().hello, &direct};
    for (const s2::Hello* d : decoded) {
        REQUIRE(d->complete);
        CHECK(d->buildHash == std::string(48, 'H'));
        CHECK(d->routineId == std::string(32, 'R'));
        CHECK(d->alliance == std::string(16, 'A'));
        CHECK(d->side == std::string(16, 'S'));
        CHECK(d->portMap == std::string(96, 'P'));
    }
}

// Would catch: bytes 12 onward of ANOTHER major's Hello read with 1.0's layout (review finding,
// correctness lens). They belong to their major (D4), and the published contract says another
// major is refused whole, "read nothing from it" — a refused session's report must never show a
// future layout's bytes as a role, widths, an epoch or provenance. Only the invariant part is
// decoded, and the verdict is unchanged: RefusedMajor, which negotiate() checks first.
TEST_CASE("SHUL/2: another major's Hello is decoded in its invariant bytes 0-11 only") {
    for (const std::uint16_t major : {std::uint16_t{0}, std::uint16_t{2}, std::uint16_t{65535}}) {
        CAPTURE(major);
        Bytes p = helloPayload(standardInfo(), 1234.5, s2::kRolePeer);  // 1.0's whole layout…
        putLe(p, 8, major, 2);                                          // …under another major
        s2::Hello h;
        REQUIRE(s2::decodeHello(p, h));  // true: the invariant part arrived
        CHECK(h.payloadBytes == 256);
        CHECK(h.signature[0] == 0x89);
        CHECK(h.versionMajor == major);
        CHECK(h.versionMinor == 0);
        CHECK_FALSE(h.complete);
        CHECK(h.role == 0);
        CHECK(h.flags == 0);
        CHECK(h.declared.hello == 0);
        CHECK(h.declared.tick == 0);
        CHECK(h.declared.logFixed == 0);
        CHECK(bitsOf(h.epochSeconds) == 0ULL);
        CHECK(h.apiMajor == 0);
        CHECK(h.apiMinor == 0);
        CHECK(h.buildHash.empty());
        CHECK(h.routineId.empty());
        CHECK(h.alliance.empty());
        CHECK(h.side.empty());
        CHECK(h.portMap.empty());
        CHECK(s2::negotiate(h, s2::kThisReader) == s2::Verdict::RefusedMajor);
    }
    // The same bytes at THIS major are decoded whole: the rule keys on the major, nothing else.
    const Bytes same = helloPayload(standardInfo(), 1234.5, s2::kRolePeer);
    s2::Hello h;
    REQUIRE(s2::decodeHello(same, h));
    CHECK(h.complete);
    CHECK(h.role == 1);
    CHECK(h.routineId == "redLeftTall");
}

// Would catch: a record field dropped, retyped or re-ordered through the WHOLE path (encode,
// frame, scan, deliver, decode), with every field distinct and non-default.
TEST_CASE("SHUL/2 round trip: Ticks through the reader, field for field") {
    const Capture c = standardSession(12);
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    int k = 0;
    for (const auto& f : frames) {
        if (f.type != s2::FrameType::Tick) {
            continue;
        }
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(s2::decodeTick(f.payload, out, corrupt));
        CHECK_FALSE(corrupt);
        checkSameRecordBits(record(static_cast<double>(k)), out);
        ++k;
    }
    CHECK(k == 12);
}

// Would catch: the Summary's fields, and its blackboxDropped taken from anywhere but the
// summary's OWN field.
TEST_CASE("SHUL/2 round trip: a Summary carries the summary's own blackboxDropped") {
    RunSummary s;
    s.motionsStarted = 6;
    s.motionsSettled = 4;
    s.motionsTimedOut = 1;
    s.motionsCancelled = 2;
    s.motionsAborted = 3;
    s.gatingRejects = 9;
    s.hasHeadingData = true;
    s.brownout = true;
    s.firstFault = FaultCode::MotorOverTemp;
    s.headingMax = units::AngleDim{0.25};
    s.headingFinal = units::AngleDim{-0.125};
    s.worstLoopDt = units::Time{0.0125};
    s.firstFaultTime = units::Time{4.5};
    s.droppedRecords = 11;
    s.droppedLines = 12;
    s.blackboxDropped = 77;
    s.batteryStart = units::Voltage{12.5};
    s.batteryEnd = units::Voltage{11.75};
    s.setBuildHash("abcdef1-dirty");
    s.setRoutineId("blueRightShort");

    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Summary, summaryPayload(s));
    const auto frames = readCapture(c.bytes);
    REQUIRE(frames.size() == 2);
    REQUIRE(frames[1].type == s2::FrameType::Summary);
    RunSummary out;
    std::uint32_t dropped = 0;
    REQUIRE(s2::decodeSummary(frames[1].payload, out, dropped));
    CHECK(dropped == 77);
    CHECK(out.motionsStarted == 6);
    CHECK(out.motionsSettled == 4);
    CHECK(out.motionsTimedOut == 1);
    CHECK(out.motionsCancelled == 2);
    CHECK(out.motionsAborted == 3);
    CHECK(out.gatingRejects == 9);
    CHECK(out.hasHeadingData);
    CHECK(out.brownout);
    CHECK(out.firstFault == FaultCode::MotorOverTemp);
    CHECK(out.headingMax.value() == 0.25);
    CHECK(out.headingFinal.value() == -0.125);
    CHECK(out.worstLoopDt.value() == 0.0125);
    CHECK(out.firstFaultTime.value() == 4.5);
    CHECK(out.droppedRecords == 11);
    CHECK(out.droppedLines == 12);
    CHECK(out.batteryStart.value() == 12.5);
    CHECK(out.batteryEnd.value() == 11.75);
    CHECK(out.buildHash() == "abcdef1-dirty");
    CHECK(out.routineId() == "blueRightShort");
}

// Would catch: an End field dropped or mis-decoded through the reader.
TEST_CASE("SHUL/2 round trip: the End, field for field") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    s2::EndInfo e;
    e.framesOffered = 1;
    e.framesFailed = 0;
    e.recordsSeen = 101;
    e.linesSeen = 102;
    e.linesTruncated = 103;
    e.notOpenDropped = 104;
    e.endTime = 187.625;
    c.add(s2::FrameType::End, endPayload(e));
    const auto frames = readCapture(c.bytes);
    REQUIRE(frames.size() == 2);
    s2::EndInfo out;
    REQUIRE(s2::decodeEnd(frames[1].payload, out));
    CHECK(out.framesOffered == 1);
    CHECK(out.framesFailed == 0);
    CHECK(out.recordsSeen == 101);
    CHECK(out.linesSeen == 102);
    CHECK(out.linesTruncated == 103);
    CHECK(out.notOpenDropped == 104);
    CHECK(out.endTime == 187.625);
}

// Would catch: text that is sanitized, NUL-terminated or re-framed on the way (every byte value
// 0x00-0xFF, and a false sync pair 0xFE 0x53 inside the text, must arrive verbatim and leave the
// framing unbroken), an empty tag or message mishandled, and an unknown level refused rather
// than carried as its number.
TEST_CASE("SHUL/2 round trip: Log text is verbatim — every byte value, empty fields, level 7") {
    std::string all(256, '\0');
    for (std::size_t i = 0; i < 256; ++i) {
        all[i] = static_cast<char>(i);
    }
    const std::string tag = std::string("T\xFE\x53") + '\0' + "g";
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Log, logPayload(2, "", "", 101.0));
    c.add(s2::FrameType::Log, logPayload(1, tag, all, 102.0));
    c.add(s2::FrameType::Log, logPayload(7, "SEQ", "level seven", 103.0));
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    const auto frames = readCapture(c.bytes);
    REQUIRE(frames.size() == 5);

    s2::LogLine a;
    REQUIRE(s2::decodeLog(frames[1].payload, a));
    CHECK(a.tag.empty());
    CHECK(a.message.empty());
    CHECK(a.t == 101.0);
    CHECK(a.nonTextBytes == 14);

    s2::LogLine b;
    REQUIRE(s2::decodeLog(frames[2].payload, b));
    CHECK(b.level == 1);
    CHECK(std::string(b.tag) == tag);
    CHECK(std::string(b.message) == all);
    CHECK(b.t == 102.0);

    s2::LogLine seven;
    REQUIRE(s2::decodeLog(frames[3].payload, seven));
    CHECK(seven.level == 7);
    CHECK(seven.message == "level seven");
    LogLevel mapped = LogLevel::Info;
    CHECK_FALSE(s2::logLevelFromWire(seven.level, mapped));

    CHECK(frames[4].type == s2::FrameType::Tick);  // the framing after them is intact
}

// Would catch: an encoder that formats floats as text or narrows them, and a decoder that
// canonicalises NaN or loses −0.0's sign — in EVERY f64 field of the Tick except the three
// Angle fields (next case), and in the Log's t, the End's endTime, the Hello's epoch and the
// Summary's six. Compared by BIT PATTERN: == fails on NaN and cannot see −0.0.
TEST_CASE("SHUL/2 round trip: NaN, ±Inf, −0.0 and denormals survive bit-exactly") {
    const std::uint64_t specials[] = {
        0x7FF8000000000000ULL,  // quiet NaN
        0xFFF8000000000000ULL,  // negative quiet NaN
        0x7FF8000000000ABCULL,  // quiet NaN with a payload
        0x7FF0000000000000ULL,  // +Inf
        0xFFF0000000000000ULL,  // -Inf
        0x8000000000000000ULL,  // -0.0
        0x0000000000000001ULL,  // smallest denormal
        0x000FFFFFFFFFFFFFULL,  // largest denormal
        0x8000000000000001ULL,  // negative denormal
    };
    using Set = std::function<void(DebugRecord&, double)>;
    using Get = std::function<double(const DebugRecord&)>;
    struct Field {
        std::string name;
        Set set;
        Get get;
    };
    std::vector<Field> fields = {
        {"t", [](DebugRecord& r, double v) { r.t = units::Time{v}; }, [](const DebugRecord& r) { return r.t.value(); }},
        {"dt", [](DebugRecord& r, double v) { r.dt = units::Time{v}; }, [](const DebugRecord& r) { return r.dt.value(); }},
        {"targetPose.x",
         [](DebugRecord& r, double v) { r.targetPose = Pose2d{units::Length{v}, r.targetPose.y(), r.targetPose.heading()}; },
         [](const DebugRecord& r) { return r.targetPose.x().value(); }},
        {"targetPose.y",
         [](DebugRecord& r, double v) { r.targetPose = Pose2d{r.targetPose.x(), units::Length{v}, r.targetPose.heading()}; },
         [](const DebugRecord& r) { return r.targetPose.y().value(); }},
        {"measuredPose.x",
         [](DebugRecord& r, double v) {
             r.measuredPose = Pose2d{units::Length{v}, r.measuredPose.y(), r.measuredPose.heading()};
         },
         [](const DebugRecord& r) { return r.measuredPose.x().value(); }},
        {"measuredPose.y",
         [](DebugRecord& r, double v) {
             r.measuredPose = Pose2d{r.measuredPose.x(), units::Length{v}, r.measuredPose.heading()};
         },
         [](const DebugRecord& r) { return r.measuredPose.y().value(); }},
        {"errorX", [](DebugRecord& r, double v) { r.errorX = units::Length{v}; }, [](const DebugRecord& r) { return r.errorX.value(); }},
        {"errorY", [](DebugRecord& r, double v) { r.errorY = units::Length{v}; }, [](const DebugRecord& r) { return r.errorY.value(); }},
        {"errorHeading", [](DebugRecord& r, double v) { r.errorHeading = units::AngleDim{v}; },
         [](const DebugRecord& r) { return r.errorHeading.value(); }},
        {"commanded.vx",
         [](DebugRecord& r, double v) { r.commanded = ChassisSpeeds{units::Velocity{v}, r.commanded.vy(), r.commanded.omega()}; },
         [](const DebugRecord& r) { return r.commanded.vx().value(); }},
        {"commanded.vy",
         [](DebugRecord& r, double v) { r.commanded = ChassisSpeeds{r.commanded.vx(), units::Velocity{v}, r.commanded.omega()}; },
         [](const DebugRecord& r) { return r.commanded.vy().value(); }},
        {"commanded.omega",
         [](DebugRecord& r, double v) {
             r.commanded = ChassisSpeeds{r.commanded.vx(), r.commanded.vy(), units::AngularVelocity{v}};
         },
         [](const DebugRecord& r) { return r.commanded.omega().value(); }},
        {"imuYawRate", [](DebugRecord& r, double v) { r.imuYawRate = units::AngularVelocity{v}; },
         [](const DebugRecord& r) { return r.imuYawRate.value(); }},
        {"quality", [](DebugRecord& r, double v) { r.quality = v; }, [](const DebugRecord& r) { return r.quality; }},
        {"covarianceTrace", [](DebugRecord& r, double v) { r.covarianceTrace = v; },
         [](const DebugRecord& r) { return r.covarianceTrace; }},
        {"gateResidualX", [](DebugRecord& r, double v) { r.gateResidualX = units::Length{v}; },
         [](const DebugRecord& r) { return r.gateResidualX.value(); }},
        {"gateResidualY", [](DebugRecord& r, double v) { r.gateResidualY = units::Length{v}; },
         [](const DebugRecord& r) { return r.gateResidualY.value(); }},
        {"gateResidualHeading", [](DebugRecord& r, double v) { r.gateResidualHeading = units::AngleDim{v}; },
         [](const DebugRecord& r) { return r.gateResidualHeading.value(); }},
        {"gateMahalanobis", [](DebugRecord& r, double v) { r.gateMahalanobis = v; },
         [](const DebugRecord& r) { return r.gateMahalanobis; }},
        {"correctionDx", [](DebugRecord& r, double v) { r.correctionDx = units::Length{v}; },
         [](const DebugRecord& r) { return r.correctionDx.value(); }},
        {"correctionDy", [](DebugRecord& r, double v) { r.correctionDy = units::Length{v}; },
         [](const DebugRecord& r) { return r.correctionDy.value(); }},
        {"correctionDTheta", [](DebugRecord& r, double v) { r.correctionDTheta = units::AngleDim{v}; },
         [](const DebugRecord& r) { return r.correctionDTheta.value(); }},
        {"batteryVoltage", [](DebugRecord& r, double v) { r.batteryVoltage = units::Voltage{v}; },
         [](const DebugRecord& r) { return r.batteryVoltage.value(); }},
        {"batteryCurrent", [](DebugRecord& r, double v) { r.batteryCurrent = units::Current{v}; },
         [](const DebugRecord& r) { return r.batteryCurrent.value(); }},
    };
    for (std::size_t i = 0; i < 8; ++i) {
        fields.push_back({"wheelVoltage[" + std::to_string(i) + "]",
                          [i](DebugRecord& r, double v) { r.wheelVoltage[i] = units::Voltage{v}; },
                          [i](const DebugRecord& r) { return r.wheelVoltage[i].value(); }});
        fields.push_back({"wheelCurrent[" + std::to_string(i) + "]",
                          [i](DebugRecord& r, double v) { r.wheelCurrent[i] = units::Current{v}; },
                          [i](const DebugRecord& r) { return r.wheelCurrent[i].value(); }});
        fields.push_back({"tickPhase[" + std::to_string(i) + "]",
                          [i](DebugRecord& r, double v) { r.tickPhase[i] = units::Time{v}; },
                          [i](const DebugRecord& r) { return r.tickPhase[i].value(); }});
    }
    CHECK(fields.size() == 48);  // every f64 of the record but the three Angles (51 − 3)

    for (const Field& f : fields) {
        for (const std::uint64_t bits : specials) {
            CAPTURE(f.name);
            CAPTURE(bits);
            DebugRecord in = record(0.0);
            f.set(in, fromBits(bits));
            DebugRecord out;
            bool corrupt = false;
            REQUIRE(s2::decodeTick(throughTheWire(tickPayload(in)), out, corrupt));
            CHECK_FALSE(corrupt);
            CHECK(bitsOf(f.get(out)) == bits);
        }
    }

    // The other frames' f64s: the Log's t, the End's endTime, the Hello's epoch, the Summary's six.
    for (const std::uint64_t bits : specials) {
        CAPTURE(bits);
        const double v = fromBits(bits);
        RunSummary s;
        s.headingMax = units::AngleDim{v};
        s.headingFinal = units::AngleDim{v};
        s.worstLoopDt = units::Time{v};
        s.firstFaultTime = units::Time{v};
        s.batteryStart = units::Voltage{v};
        s.batteryEnd = units::Voltage{v};
        Capture c;
        c.add(s2::FrameType::Hello, helloPayload(standardInfo(), v));
        c.add(s2::FrameType::Log, logPayload(2, "T", "m", v));
        c.add(s2::FrameType::Summary, summaryPayload(s));
        c.add(s2::FrameType::End, endPayloadAt(3, 0, v));
        const auto frames = readCapture(c.bytes);
        REQUIRE(frames.size() == 4);
        s2::Hello h;
        REQUIRE(s2::decodeHello(frames[0].payload, h));
        CHECK(bitsOf(h.epochSeconds) == bits);
        s2::LogLine l;
        REQUIRE(s2::decodeLog(frames[1].payload, l));
        CHECK(bitsOf(l.t) == bits);
        RunSummary so;
        std::uint32_t dropped = 0;
        REQUIRE(s2::decodeSummary(frames[2].payload, so, dropped));
        CHECK(bitsOf(so.headingMax.value()) == bits);
        CHECK(bitsOf(so.headingFinal.value()) == bits);
        CHECK(bitsOf(so.worstLoopDt.value()) == bits);
        CHECK(bitsOf(so.firstFaultTime.value()) == bits);
        CHECK(bitsOf(so.batteryStart.value()) == bits);
        CHECK(bitsOf(so.batteryEnd.value()) == bits);
        s2::EndInfo e;
        REQUIRE(s2::decodeEnd(frames[3].payload, e));
        CHECK(bitsOf(e.endTime) == bits);
    }
}

// Would catch: a decoder that trips math::Angle's precondition on a non-finite heading (a
// decoder that throws on a corrupt stream cannot read the stream you most need to read), or
// that decodes it as anything but 0 WITHOUT raising `corrupt`. The bytes are patched in by hand
// and re-framed, so the CRC verifies: the corruption happened before the wire.
TEST_CASE("SHUL/2 round trip: a non-finite Angle field decodes to 0 with corrupt raised") {
    const std::size_t angleOffsets[3] = {32, 56, 260};  // targetPose.h, measuredPose.h, imuYaw
    const std::uint64_t nonFinite[3] = {0x7FF8000000000000ULL, 0x7FF0000000000000ULL,
                                        0xFFF0000000000000ULL};
    for (const std::size_t off : angleOffsets) {
        for (const std::uint64_t bits : nonFinite) {
            CAPTURE(off);
            CAPTURE(bits);
            Bytes p = tickPayload(record(0.0));
            putLe(p, off, bits, 8);
            DebugRecord out;
            bool corrupt = false;
            REQUIRE(s2::decodeTick(throughTheWire(p), out, corrupt));
            CHECK(corrupt);
            const double got = off == 32   ? out.targetPose.heading().radians()
                               : off == 56 ? out.measuredPose.heading().radians()
                                           : out.imuYaw.radians();
            CHECK(bitsOf(got) == 0ULL);
            CHECK(out.batteryVoltage.value() == record(0.0).batteryVoltage.value());  // the rest intact
        }
    }
}

// Would catch: a vocabulary value narrowed, widened, re-mapped, or REFUSED because this build
// does not name it — every FaultCode plus 0xFFFF, every GateReason plus 200, every MotionState
// value plus 200 and qualityClass 0-3 plus 200 travel as their numbers.
TEST_CASE("SHUL/2 round trip: every vocabulary value, known or not, travels as its number") {
    const auto roundTrip = [](const DebugRecord& in) {
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(s2::decodeTick(throughTheWire(tickPayload(in)), out, corrupt));
        CHECK_FALSE(corrupt);
        return out;
    };
    std::vector<std::uint16_t> faults;
    for (std::uint16_t v = 0; v <= 12; ++v) {
        faults.push_back(v);
    }
    faults.push_back(0xFFFF);
    for (const std::uint16_t v : faults) {
        DebugRecord in = record(0.0);
        in.fault = static_cast<FaultCode>(v);
        CHECK(static_cast<std::uint16_t>(roundTrip(in).fault) == v);
    }
    std::vector<std::uint8_t> small;
    for (std::uint8_t v = 0; v <= 12; ++v) {
        small.push_back(v);
    }
    small.push_back(200);
    for (const std::uint8_t v : small) {
        CAPTURE(static_cast<int>(v));
        DebugRecord in = record(0.0);
        in.gateReason = static_cast<GateReason>(v);
        CHECK(static_cast<std::uint8_t>(roundTrip(in).gateReason) == v);
        if (v <= 5 || v == 200) {  // motion::MotionState 0-5, and an unknown
            in = record(0.0);
            in.activeCommandState = v;
            CHECK(roundTrip(in).activeCommandState == v);
        }
        if (v <= 3 || v == 200) {  // the Localizer::Quality mirror 0-3, and an unknown
            in = record(0.0);
            in.qualityClass = v;
            CHECK(roundTrip(in).qualityClass == v);
        }
    }
}

// ── The wheelCount clamp (E1's decoder, fixed at its source) ────────────────────────

// Would catch: a decoded wheelCount above the 8-wide arrays reaching a consumer that loops to
// it (mutation 25), and an encode that wraps a large count instead of clamping it.
TEST_CASE("blackbox wheelCount: decoded above 8 is 8 with corrupt raised; encoded clamped") {
    for (const std::uint8_t raw : {std::uint8_t{9}, std::uint8_t{255}}) {
        CAPTURE(static_cast<int>(raw));
        Bytes p = tickPayload(record(0.0));
        p[112] = static_cast<std::byte>(raw);
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(p, out, corrupt));
        CHECK(out.wheelCount == DebugRecord::kMaxWheels);
        CHECK(corrupt);
        // …and through the wire's decoder, the same.
        DebugRecord out2;
        bool corrupt2 = false;
        REQUIRE(s2::decodeTick(p, out2, corrupt2));
        CHECK(out2.wheelCount == 8);
        CHECK(corrupt2);
    }
    {
        Bytes p = tickPayload(record(0.0));
        p[112] = std::byte{8};
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(p, out, corrupt));
        CHECK(out.wheelCount == 8);
        CHECK_FALSE(corrupt);  // 8 is the capacity, not corruption
    }
    for (const int count : {300, 9, 256}) {
        CAPTURE(count);
        DebugRecord r = record(0.0);
        r.wheelCount = count;
        CHECK(static_cast<std::uint8_t>(tickPayload(r)[112]) == 8U);
    }
    DebugRecord neg = record(0.0);
    neg.wheelCount = -3;
    CHECK(static_cast<std::uint8_t>(tickPayload(neg)[112]) == 0U);
}

// ── Negotiation ─────────────────────────────────────────────────────────────────────

namespace {

/// A Hello payload at `major.minor`, declaring `row`, `row.hello` bytes long — built by hand
/// from this build's Hello (the version and width fields patched, the length adjusted).
Bytes helloAt(std::uint16_t major, std::uint16_t minor, const s2::WidthRow& row) {
    Bytes p = helloPayload();
    putLe(p, 8, major, 2);
    putLe(p, 10, minor, 2);
    putLe(p, 16, row.tick, 2);
    putLe(p, 18, row.summary, 2);
    putLe(p, 20, row.end, 2);
    putLe(p, 22, row.logFixed, 2);
    p.resize(row.hello, std::byte{0});
    return p;
}

s2::Verdict judge(const Bytes& payload, const s2::ReaderProfile& profile) {
    s2::Hello h;
    (void)s2::decodeHello(payload, h);
    return s2::negotiate(h, profile);
}

constexpr s2::WidthRow kRow10{256, 428, 168, 32, 14};
constexpr s2::WidthRow kRow11{264, 436, 176, 40, 22};  // a hypothetical 1.1: every width +8
constexpr s2::WidthRow kRows11[] = {kRow10, kRow11};
constexpr s2::ReaderProfile kReader11{1, std::span<const s2::WidthRow>{kRows11}};

}  // namespace

// Would catch: another major accepted (mutation 6), a newer minor refused (7), a width
// mismatch at the same minor accepted (8), an older minor checked against the wrong row (9 —
// the 1.1 profile), a SHORTER newer minor accepted, a width left unchecked (each of the five is
// varied alone), and the checks run in the wrong order. The expectation table is the brief's
// rule written as a 3×3 table per profile, not a copy of negotiate()'s code.
TEST_CASE("SHUL/2 negotiate: the whole version table, under the 1.0 AND a 1.1 profile") {
    enum class Rel { Older, Same, Newer };
    enum class Var { Equal, Shorter, Longer };
    struct Profile {
        const char* name;
        s2::ReaderProfile reader;
        std::uint16_t newest;
    };
    const Profile profiles[] = {{"1.0", s2::kThisReader, 0}, {"1.1", kReader11, 1}};
    const std::uint16_t majors[] = {0, 1, 2, 65535};
    const std::uint16_t minors[] = {0, 1, 7, 65535};
    const auto expected = [](Rel rel, Var var) {
        switch (rel) {
            case Rel::Older: return var == Var::Equal ? s2::Verdict::OlderMinor : s2::Verdict::LayoutMismatch;
            case Rel::Same: return var == Var::Equal ? s2::Verdict::Exact : s2::Verdict::LayoutMismatch;
            case Rel::Newer: return var == Var::Shorter ? s2::Verdict::LayoutMismatch : s2::Verdict::NewerMinor;
        }
        return s2::Verdict::BadSignature;
    };
    int cases = 0;
    for (const Profile& prof : profiles) {
        for (const std::uint16_t major : majors) {
            for (const std::uint16_t minor : minors) {
                const Rel rel = minor < prof.newest ? Rel::Older : minor == prof.newest ? Rel::Same : Rel::Newer;
                const s2::WidthRow base = prof.newest == 0 || minor == 0 ? kRow10 : kRow11;
                for (int field = -1; field < 5; ++field) {
                    for (const Var var : {Var::Shorter, Var::Longer}) {
                        if (field == -1 && var == Var::Longer) {
                            continue;  // field -1 = the Equal case, once
                        }
                        s2::WidthRow row = base;
                        const Var v = field == -1 ? Var::Equal : var;
                        const int delta = v == Var::Shorter ? -1 : 1;
                        std::uint16_t* w[5] = {&row.hello, &row.tick, &row.summary, &row.end, &row.logFixed};
                        if (field >= 0) {
                            *w[field] = static_cast<std::uint16_t>(*w[field] + delta);
                        }
                        CAPTURE(prof.name);
                        CAPTURE(major);
                        CAPTURE(minor);
                        CAPTURE(field);
                        CAPTURE(static_cast<int>(v));
                        const s2::Verdict want = major != 1 ? s2::Verdict::RefusedMajor : expected(rel, v);
                        CHECK(judge(helloAt(major, minor, row), prof.reader) == want);
                        ++cases;
                    }
                }
            }
        }
    }
    CHECK(cases == 2 * 4 * 4 * 11);

    // Hello payloads of 11, 12 and 255 bytes.
    Bytes p = helloPayload();
    p.resize(11);
    CHECK(judge(p, s2::kThisReader) == s2::Verdict::BadSignature);
    p = helloPayload();
    p.resize(12);
    CHECK(judge(p, s2::kThisReader) == s2::Verdict::LayoutMismatch);
    putLe(p, 8, 2, 2);  // a 12-byte 2.0 Hello is still NAMED: refused by major, not by layout
    CHECK(judge(p, s2::kThisReader) == s2::Verdict::RefusedMajor);
    p = helloPayload();
    p.resize(255);
    CHECK(judge(p, s2::kThisReader) == s2::Verdict::LayoutMismatch);

    // An empty profile knows no layout.
    CHECK(judge(helloPayload(), s2::ReaderProfile{1, {}}) == s2::Verdict::LayoutMismatch);

    // Under the 1.1 profile, a 1.0 stream at the 1.0 row is OlderMinor; at the 1.1 row, refused.
    CHECK(judge(helloAt(1, 0, kRow10), kReader11) == s2::Verdict::OlderMinor);
    CHECK(judge(helloAt(1, 0, kRow11), kReader11) == s2::Verdict::LayoutMismatch);
}

// Would catch: a signature check that tolerates a transport's damage — each alteration is what
// a real transform does to the 8 bytes, and each must be BadSignature, never acceptance. Judged
// at negotiate() AND through the reader with the CRC recomputed (the frame is re-framed): a
// Hello whose CRC verifies and whose signature is wrong is exactly what BadSignature names.
TEST_CASE("SHUL/2 negotiate: every signature alteration is BadSignature") {
    const Bytes good = helloPayload();
    std::vector<std::pair<const char*, Bytes>> altered;
    {
        Bytes b = good;
        b[0] = std::byte{0x09};  // 0x89 with bit 7 cleared
        altered.emplace_back("bit 7 stripped", b);
    }
    {
        Bytes b = good;
        b.erase(b.begin() + 5);  // the NUL removed
        altered.emplace_back("NUL removed", b);
    }
    {
        Bytes b = good;
        b.erase(b.begin() + 6);  // CR LF -> LF
        altered.emplace_back("CR LF -> LF", b);
    }
    {
        Bytes b = good;
        b.insert(b.begin() + 7, std::byte{0x0D});  // LF -> CR LF
        altered.emplace_back("LF -> CR LF", b);
    }
    CHECK(judge(good, s2::kThisReader) == s2::Verdict::Exact);
    for (const auto& [name, b] : altered) {
        CAPTURE(name);
        CHECK(judge(b, s2::kThisReader) == s2::Verdict::BadSignature);
        Capture c;
        c.add(s2::FrameType::Hello, b);
        c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
        s2::Shul2Reader reader{c.bytes};
        CHECK(readAll(reader).empty());
        CHECK(reader.status() == s2::CaptureStatus::Refused);
        CHECK(reader.latestSession().verdict == s2::Verdict::BadSignature);
        CHECK(reader.counters().framesInRefusedSessions == 1);
    }
}

// ── scanFrame and the encoders' edges ───────────────────────────────────────────────

// Would catch: a NeedMore that consumes, a missing prefix case, the 4096 bound removed
// (mutation 4: a length of 60000 would then wait for 60 KB instead of resyncing one byte on),
// and a skip that jumps past the next sync candidate.
TEST_CASE("SHUL/2 scanFrame: NeedMore, the 4096 bound, CRC and sync skips") {
    const Bytes f = frameOf(typeByte(s2::FrameType::Tick), 5, tickPayload(record(0.0)));
    for (std::size_t k = 0; k < f.size(); ++k) {  // every proper prefix, the empty one included
        CAPTURE(k);
        const s2::ScanResult r = s2::scanFrame(std::span<const std::byte>{f.data(), k});
        CHECK(r.kind == s2::ScanKind::NeedMore);
        CHECK(r.consumed == 0);
    }
    const s2::ScanResult whole = s2::scanFrame(f);
    REQUIRE(whole.kind == s2::ScanKind::Frame);
    CHECK(whole.consumed == 442);
    CHECK(whole.type == 2);
    CHECK(whole.seq == 5);
    CHECK(whole.payload.size() == 428);

    Bytes big = f;
    putLe(big, 8, 60000, 2);
    const s2::ScanResult tooLong = s2::scanFrame(big);
    CHECK(tooLong.kind == s2::ScanKind::Skip);
    CHECK(tooLong.reason == s2::SkipReason::LengthTooLarge);
    CHECK(tooLong.consumed == 1);
    putLe(big, 8, 4097, 2);  // one past the bound
    CHECK(s2::scanFrame(big).reason == s2::SkipReason::LengthTooLarge);

    // The bound is INCLUSIVE (review mutation N3/R2: `>=` would skip the largest frame a writer may
    // legally send). A payload of exactly 4096 bytes is a frame.
    const Bytes maxFrame = frameOf(typeByte(s2::FrameType::Log), 6, Bytes(4096, std::byte{0x5A}));
    const s2::ScanResult atBound = s2::scanFrame(maxFrame);
    REQUIRE(atBound.kind == s2::ScanKind::Frame);
    CHECK(atBound.consumed == 4110);
    CHECK(atBound.payload.size() == 4096);
    CHECK(atBound.seq == 6);

    Bytes bad = f;
    bad[200] ^= std::byte{0x01};
    const s2::ScanResult crc = s2::scanFrame(bad);
    CHECK(crc.kind == s2::ScanKind::Skip);
    CHECK(crc.reason == s2::SkipReason::CrcMismatch);
    CHECK(crc.consumed == 1);

    const Bytes junk = {std::byte{0x00}, std::byte{0x53}, std::byte{0x10}, std::byte{0xFE}, std::byte{0x00}};
    const s2::ScanResult noSync = s2::scanFrame(junk);
    CHECK(noSync.kind == s2::ScanKind::Skip);
    CHECK(noSync.reason == s2::SkipReason::NoSync);
    CHECK(noSync.consumed == 3);  // up to the next 0xFE
    const Bytes feJunk = {std::byte{0xFE}, std::byte{0x00}, std::byte{0x01}};
    CHECK(s2::scanFrame(feJunk).consumed == 3);  // 0xFE then no 0x53, and no later 0xFE: all of it
    const Bytes loneFe = {std::byte{0xFE}};
    CHECK(s2::scanFrame(loneFe).kind == s2::ScanKind::NeedMore);
}

// Would catch: an encoder that half-writes (a frame or payload cut short on the wire is worse
// than none), a length above the bound encoded anyway, and the in-place path copying badly.
TEST_CASE("SHUL/2 encoders: whole or nothing, and the in-place frame") {
    Bytes out(100, std::byte{0xAA});
    const Bytes payload(90, std::byte{0x11});
    CHECK(s2::encodeFrame(out, 2, 1, payload) == 0);  // 104 bytes needed
    for (const std::byte b : out) {
        CHECK(b == std::byte{0xAA});  // untouched
    }
    Bytes huge(4097);
    Bytes room(5000);
    CHECK(s2::encodeFrame(room, 2, 1, huge) == 0);
    Bytes max(4096, std::byte{0x22});
    CHECK(s2::encodeFrame(room, 2, 1, max) == 4110);

    // In place: the payload already sits at out[10..].
    Bytes inPlace(14 + 428);
    REQUIRE(bb::encodeTick(std::span<std::byte>{inPlace}.subspan(10, 428), record(3.0)) == 428);
    REQUIRE(s2::encodeFrame(inPlace, 2, 9, std::span<const std::byte>{inPlace}.subspan(10, 428)) == 442);
    CHECK(inPlace == frameOf(2, 9, tickPayload(record(3.0))));

    Bytes small(10);
    CHECK(s2::encodeHello(small, standardInfo(), 1.0) == 0);
    CHECK(s2::encodeEnd(small, s2::EndInfo{}) == 0);
    s2::LogLine line;
    const std::string longTag(256, 't');
    line.tag = longTag;
    Bytes logOut(4096);
    CHECK(s2::encodeLog(logOut, line) == 0);  // a tag longer than a u8 can declare
    // …and the longest a u8 CAN declare, 255, is allowed (review mutation N11: `>=` refused it).
    line.tag = std::string_view{longTag}.substr(0, 255);
    line.message = "m";
    REQUIRE(s2::encodeLog(logOut, line) == 14 + 255 + 1);
    s2::LogLine back;
    REQUIRE(s2::decodeLog(std::span<const std::byte>{logOut}.first(14 + 255 + 1), back));
    CHECK(back.tag == std::string(255, 't'));
    CHECK(back.message == "m");
    const std::string longMsg(4096 - 14 + 1, 'm');
    line.tag = "";
    line.message = longMsg;
    CHECK(s2::encodeLog(logOut, line) == 0);  // over the payload bound
    line.message = std::string_view{longMsg}.substr(0, 4096 - 14);
    CHECK(s2::encodeLog(logOut, line) == 4096);
    Bytes shortHello(40);
    s2::Hello h;
    CHECK_FALSE(s2::decodeLog(std::span<const std::byte>{shortHello}.first(13), line));
    CHECK_FALSE(s2::decodeHello(std::span<const std::byte>{shortHello}.first(11), h));
    CHECK(h.payloadBytes == 11);
}

// Would catch: a public decoder's short-payload guard removed (review mutations N13, N18): the
// Tick and Summary decoders would hand blackbox's decoders a span CLAIMING 428 / 168 bytes over a
// shorter one and read past its end — a heap overflow under a sanitizer. These are public: a peer
// may call them on payloads that never came through Shul2Reader. Every short payload here is a
// view into a LARGER buffer, so a missing guard reads bytes that exist and returns a
// deterministic wrong `true`, rather than touching memory that does not. At exactly the known
// width each decodes, so the guards are not `<=` either.
TEST_CASE("SHUL/2 decoders: a payload shorter than its known width is refused, never read") {
    const Bytes big(1024, std::byte{0x11});
    const std::span<const std::byte> all{big};
    DebugRecord rec;
    bool corrupt = false;
    RunSummary sum;
    std::uint32_t dropped = 0;
    s2::EndInfo end;
    s2::LogLine line;
    s2::Hello hello;
    for (const std::size_t n : {std::size_t{0}, std::size_t{1}, bb::kTickPayloadBytes - 1}) {
        CAPTURE(n);
        CHECK_FALSE(s2::decodeTick(all.first(n), rec, corrupt));
    }
    for (const std::size_t n : {std::size_t{0}, std::size_t{1}, bb::kSummaryPayloadBytes - 1}) {
        CAPTURE(n);
        CHECK_FALSE(s2::decodeSummary(all.first(n), sum, dropped));
    }
    for (const std::size_t n : {std::size_t{0}, std::size_t{1}, s2::kEndPayloadBytes - 1}) {
        CAPTURE(n);
        CHECK_FALSE(s2::decodeEnd(all.first(n), end));
    }
    for (const std::size_t n : {std::size_t{0}, std::size_t{1}, s2::kLogFixedBytes - 1}) {
        CAPTURE(n);
        CHECK_FALSE(s2::decodeLog(all.first(n), line));
    }
    for (const std::size_t n : {std::size_t{0}, std::size_t{1}, s2::kHelloInvariantBytes - 1}) {
        CAPTURE(n);
        CHECK_FALSE(s2::decodeHello(all.first(n), hello));
    }
    CHECK(s2::decodeTick(all.first(bb::kTickPayloadBytes), rec, corrupt));
    CHECK(s2::decodeSummary(all.first(bb::kSummaryPayloadBytes), sum, dropped));
    CHECK(s2::decodeEnd(all.first(s2::kEndPayloadBytes), end));
    CHECK(end.framesOffered == 0x11111111U);
}
