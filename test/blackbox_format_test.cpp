// Tests for the E1 blackbox FORMAT — diag/blackbox_format.hpp (encoder) and
// diag/blackbox_reader.hpp (decoder). What each targets:
//
//  * THE ROUND TRIP ON GROUND TRUTH: encode a known record stream, decode it, compare
//    FIELD BY FIELD. Catches a field dropped, added, retyped, or written in the wrong
//    order by either half — the "a format nothing can read is not a record" rule.
//  * THE BYTE-EXACT GOLDEN: the round trip is structurally BLIND to a symmetric
//    mistake. Move a field's offset in both the encoder and the decoder and they still
//    agree perfectly — with every historical file now unreadable. So the header and
//    each tick field are pinned against bytes derived BY HAND from the documented
//    layout (IEEE-754 bit patterns written out literally), never by asking the encoder
//    what it produces. THIS is the test that catches an offset change.
//  * REFUSAL OVER MISREADING: a file from an unknown version, or one whose declared
//    record width disagrees with this build, must be REFUSED whole. A decoder that
//    misreads an old file is worse than one that refuses it.
//  * DAMAGE: truncation mid-frame, an unknown frame type, and a non-finite heading
//    (which math::Angle rejects by precondition) must all be handled without throwing
//    and without reading out of bounds — the files that matter most are damaged ones.

#include "doctest.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"

using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::GateReason;
using shulib::diag::RunSummary;
using shulib::diag::SessionInfo;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
namespace bb = shulib::diag::blackbox;
namespace units = shulib::units;

namespace {

/// A record with a DIFFERENT value in every field, so a round trip that swaps two
/// fields cannot accidentally pass. `k` shifts everything so a stream of records is
/// distinguishable tick by tick.
DebugRecord groundTruth(double k) {
    DebugRecord r;
    r.t = units::Time{12.5 + k};
    r.dt = units::Time{0.01 + k};
    r.targetPose = Pose2d{units::Length{24.0 + k}, units::Length{-36.5 + k}, Angle::degrees(90.0)};
    r.measuredPose = Pose2d{units::Length{23.25 + k}, units::Length{-36.0 + k}, Angle::degrees(-45.0)};
    r.errorX = units::Length{0.75 + k};
    r.errorY = units::Length{-0.5 + k};
    r.errorHeading = units::AngleDim{0.125 + k};
    r.commanded = ChassisSpeeds{units::Velocity{18.5 + k}, units::Velocity{-4.25 + k},
                                units::AngularVelocity{0.75 + k}};
    r.wheelCount = 4;
    for (int i = 0; i < DebugRecord::kMaxWheels; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        r.wheelVoltage[idx] = units::Voltage{static_cast<double>(i) + 0.5 + k};
        r.wheelCurrent[idx] = units::Current{static_cast<double>(i) * 0.25 - 1.0 + k};
    }
    r.imuYaw = Angle::degrees(135.0);
    r.imuYawRate = units::AngularVelocity{-1.5 + k};
    r.activeCommandId = 7;
    r.activeCommandState = 3;
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
    r.strafeFallbackActive = true;
    r.fault = FaultCode::OdoStuck;
    r.batteryVoltage = units::Voltage{12.4 + k};
    r.batteryCurrent = units::Current{5.5 + k};
    r.droppedRecords = 41;
    r.droppedLines = 42;
    for (int i = 0; i < shulib::diag::kTickPhaseSlots; ++i) {
        r.tickPhase[static_cast<std::size_t>(i)] = units::Time{0.001 * (i + 1) + k};
    }
    return r;
}

/// Field-by-field equality. Written out longhand ON PURPOSE: a memcmp of two structs
/// would compare padding and would say nothing about WHICH field broke.
void checkSameRecord(const DebugRecord& a, const DebugRecord& b) {
    CHECK(a.t.value() == b.t.value());
    CHECK(a.dt.value() == b.dt.value());
    CHECK(a.targetPose.x().value() == b.targetPose.x().value());
    CHECK(a.targetPose.y().value() == b.targetPose.y().value());
    CHECK(a.targetPose.heading().radians() == b.targetPose.heading().radians());
    CHECK(a.measuredPose.x().value() == b.measuredPose.x().value());
    CHECK(a.measuredPose.y().value() == b.measuredPose.y().value());
    CHECK(a.measuredPose.heading().radians() == b.measuredPose.heading().radians());
    CHECK(a.errorX.value() == b.errorX.value());
    CHECK(a.errorY.value() == b.errorY.value());
    CHECK(a.errorHeading.value() == b.errorHeading.value());
    CHECK(a.commanded.vx().value() == b.commanded.vx().value());
    CHECK(a.commanded.vy().value() == b.commanded.vy().value());
    CHECK(a.commanded.omega().value() == b.commanded.omega().value());
    CHECK(a.wheelCount == b.wheelCount);
    for (std::size_t i = 0; i < a.wheelVoltage.size(); ++i) {
        CHECK(a.wheelVoltage[i].value() == b.wheelVoltage[i].value());
        CHECK(a.wheelCurrent[i].value() == b.wheelCurrent[i].value());
    }
    CHECK(a.imuYaw.radians() == b.imuYaw.radians());
    CHECK(a.imuYawRate.value() == b.imuYawRate.value());
    CHECK(a.activeCommandId == b.activeCommandId);
    CHECK(a.activeCommandState == b.activeCommandState);
    CHECK(a.deadReckoning == b.deadReckoning);
    CHECK(a.qualityClass == b.qualityClass);
    CHECK(a.quality == b.quality);
    CHECK(a.covarianceTrace == b.covarianceTrace);
    CHECK(a.gateResidualX.value() == b.gateResidualX.value());
    CHECK(a.gateResidualY.value() == b.gateResidualY.value());
    CHECK(a.gateResidualHeading.value() == b.gateResidualHeading.value());
    CHECK(a.gateMahalanobis == b.gateMahalanobis);
    CHECK(a.gateReason == b.gateReason);
    CHECK(a.correctionDx.value() == b.correctionDx.value());
    CHECK(a.correctionDy.value() == b.correctionDy.value());
    CHECK(a.correctionDTheta.value() == b.correctionDTheta.value());
    CHECK(a.clampedThisTick == b.clampedThisTick);
    CHECK(a.strafeFallbackActive == b.strafeFallbackActive);
    CHECK(a.fault == b.fault);
    CHECK(a.batteryVoltage.value() == b.batteryVoltage.value());
    CHECK(a.batteryCurrent.value() == b.batteryCurrent.value());
    CHECK(a.droppedRecords == b.droppedRecords);
    CHECK(a.droppedLines == b.droppedLines);
    for (std::size_t i = 0; i < a.tickPhase.size(); ++i) {
        CHECK(a.tickPhase[i].value() == b.tickPhase[i].value());
    }
}

/// Independent little-endian readers for the golden tests. These deliberately do NOT
/// use ByteReader: a golden that reads with the code it is checking proves only
/// self-consistency.
std::uint16_t le16(const std::vector<std::byte>& b, std::size_t at) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[at])
                                      | static_cast<std::uint16_t>(
                                          static_cast<std::uint16_t>(b[at + 1]) << 8));
}
std::uint32_t le32(const std::vector<std::byte>& b, std::size_t at) {
    std::uint32_t v = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        v |= static_cast<std::uint32_t>(b[at + i]) << (8 * i);
    }
    return v;
}
std::uint64_t le64(const std::vector<std::byte>& b, std::size_t at) {
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        v |= static_cast<std::uint64_t>(b[at + i]) << (8 * i);
    }
    return v;
}

std::vector<std::byte> encodeOneHeader(const SessionInfo& info, double epoch) {
    std::vector<std::byte> buf(bb::kHeaderBytes);
    const std::size_t n = bb::encodeHeader(buf, info, epoch, 200, 65536);
    CHECK(n == bb::kHeaderBytes);
    return buf;
}

std::vector<std::byte> encodeOneTick(const DebugRecord& r) {
    std::vector<std::byte> buf(bb::kTickPayloadBytes);
    const std::size_t n = bb::encodeTick(buf, r);
    CHECK(n == bb::kTickPayloadBytes);
    return buf;
}

/// A complete little file: header + the given frames, so reader tests have something
/// realistic to chew on without dragging SdSink in.
struct FileBuilder {
    std::vector<std::byte> bytes;

    explicit FileBuilder(const SessionInfo& info = {}, double epoch = 1.0) {
        const auto header = encodeOneHeader(info, epoch);
        bytes.insert(bytes.end(), header.begin(), header.end());
    }

    void addFrame(bb::FrameType type, const std::vector<std::byte>& payload) {
        std::array<std::byte, bb::kFrameHeaderBytes> head{};
        CHECK(bb::encodeFrameHeader(head, type, static_cast<std::uint16_t>(payload.size()))
              == bb::kFrameHeaderBytes);
        bytes.insert(bytes.end(), head.begin(), head.end());
        bytes.insert(bytes.end(), payload.begin(), payload.end());
    }

    void addTick(const DebugRecord& r) { addFrame(bb::FrameType::Tick, encodeOneTick(r)); }
};

}  // namespace

// ── The round trip ──────────────────────────────────────────────────────────────────

// Would catch: any field dropped, added, retyped, or encoded/decoded in a different
// order — i.e. the whole encoder/decoder disagreeing about what a record is.
TEST_CASE("blackbox: one tick record round-trips field for field") {
    const DebugRecord in = groundTruth(0.0);
    const auto bytes = encodeOneTick(in);

    DebugRecord out;
    bool corrupt = false;
    REQUIRE(bb::decodeTick(bytes, out, corrupt));
    CHECK_FALSE(corrupt);
    checkSameRecord(in, out);
}

// Would catch: a stream that decodes only its first record (a cursor that fails to
// advance), or frames whose lengths are computed rather than read.
TEST_CASE("blackbox: a whole file of ticks round-trips through the reader, in order") {
    SessionInfo info;
    info.buildHash = "0b4948a-dirty";
    info.routineId = "redLeftTall";
    info.alliance = "red";
    info.side = "left";
    info.portMap = "L1,2,3 R4,5,6 IMU10";

    FileBuilder file{info, 12.5};
    std::vector<DebugRecord> expected;
    for (int i = 0; i < 5; ++i) {
        expected.push_back(groundTruth(static_cast<double>(i)));
        file.addTick(expected.back());
    }

    bb::BlackboxReader reader{file.bytes};
    REQUIRE(reader.status() == bb::ReadStatus::Ok);
    CHECK(reader.header().buildHash() == "0b4948a-dirty");
    CHECK(reader.header().routineId() == "redLeftTall");
    CHECK(reader.header().alliance() == "red");
    CHECK(reader.header().side() == "left");
    CHECK(reader.header().portMap() == "L1,2,3 R4,5,6 IMU10");
    CHECK(reader.header().epochSeconds == 12.5);
    CHECK(reader.header().ringCapacity == 200);

    std::size_t seen = 0;
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        REQUIRE(frame.type == bb::FrameType::Tick);
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(frame.payload, out, corrupt));
        CHECK_FALSE(corrupt);
        REQUIRE(seen < expected.size());
        checkSameRecord(expected[seen], out);
        ++seen;
    }
    CHECK(seen == expected.size());
    CHECK_FALSE(reader.truncated());
    CHECK_FALSE(reader.sawEnd());  // this file was assembled by hand: no end frame
}

// Would catch: a GateReason or FaultCode written in the wrong width (a u8 field that
// silently truncates a future 256th code), or an enum round-tripped through a signed
// type. Every reason in the vocabulary, in one stream.
TEST_CASE("blackbox: every GateReason and a spread of faults survive the trip") {
    const GateReason reasons[] = {GateReason::None,
                                  GateReason::Accepted,
                                  GateReason::RejectedInnovation,
                                  GateReason::RejectedMahalanobis,
                                  GateReason::RejectedNoFix,
                                  GateReason::RejectedHighYawRate};
    const FaultCode faults[] = {FaultCode::None,      FaultCode::NanPose,   FaultCode::OdoStuck,
                                FaultCode::Brownout,  FaultCode::Implausible};

    for (const GateReason reason : reasons) {
        for (const FaultCode fault : faults) {
            DebugRecord in = groundTruth(1.0);
            in.gateReason = reason;
            in.fault = fault;
            DebugRecord out;
            bool corrupt = false;
            REQUIRE(bb::decodeTick(encodeOneTick(in), out, corrupt));
            CHECK(out.gateReason == reason);
            CHECK(out.fault == fault);
        }
    }
}

// Would catch: an encoder that formats floats as text (losing exactness), or one that
// narrows to binary32 (which would quietly turn "equal" into "close"). Boundary values
// only a bit-preserving encoder survives.
TEST_CASE("blackbox: boundary values survive bit-exactly — NaN, infinities, denormals") {
    DebugRecord in;
    in.t = units::Time{std::numeric_limits<double>::denorm_min()};
    in.dt = units::Time{-0.0};
    in.errorX = units::Length{std::numeric_limits<double>::max()};
    in.errorY = units::Length{std::numeric_limits<double>::lowest()};
    in.errorHeading = units::AngleDim{std::numeric_limits<double>::quiet_NaN()};
    in.quality = std::numeric_limits<double>::infinity();
    in.covarianceTrace = -std::numeric_limits<double>::infinity();
    in.gateMahalanobis = 1e-300;
    in.batteryVoltage = units::Voltage{1e300};
    in.wheelCount = 8;
    in.activeCommandId = 4294967295U;   // u32 max
    in.activeCommandState = 255;
    in.droppedRecords = 4294967295U;
    in.droppedLines = 123456789U;
    in.qualityClass = 255;

    DebugRecord out;
    bool corrupt = false;
    REQUIRE(bb::decodeTick(encodeOneTick(in), out, corrupt));
    CHECK_FALSE(corrupt);  // headings were finite; only the *values* were extreme
    CHECK(out.t.value() == in.t.value());
    CHECK(std::signbit(out.dt.value()));  // -0.0 kept its sign: a bit copy, not a parse
    CHECK(out.errorX.value() == in.errorX.value());
    CHECK(out.errorY.value() == in.errorY.value());
    CHECK(std::isnan(out.errorHeading.value()));
    CHECK(out.quality == std::numeric_limits<double>::infinity());
    CHECK(out.covarianceTrace == -std::numeric_limits<double>::infinity());
    CHECK(out.gateMahalanobis == 1e-300);
    CHECK(out.batteryVoltage.value() == 1e300);
    CHECK(out.wheelCount == 8);
    CHECK(out.activeCommandId == 4294967295U);
    CHECK(out.activeCommandState == 255);
    CHECK(out.droppedRecords == 4294967295U);
    CHECK(out.droppedLines == 123456789U);
    CHECK(out.qualityClass == 255);
}

// Would catch: the summary and end frames drifting from their decoders — the two
// frames a field run is read through when nothing went wrong.
TEST_CASE("blackbox: summary and end frames round-trip") {
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
    s.batteryStart = units::Voltage{12.5};
    s.batteryEnd = units::Voltage{11.75};
    s.setBuildHash("abcdef1-dirty");
    s.setRoutineId("blueRightShort");

    std::vector<std::byte> buf(bb::kSummaryPayloadBytes);
    REQUIRE(bb::encodeSummary(buf, s, 77) == bb::kSummaryPayloadBytes);
    RunSummary out;
    std::uint32_t dropped = 0;
    REQUIRE(bb::decodeSummary(buf, out, dropped));
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

    bb::EndInfo e;
    e.tickFrames = 201;
    e.droppedFrames = 5;
    e.bytesBefore = 90000;
    e.messagesSeen = 33;
    e.brownout = true;
    e.deviceFailed = true;
    e.endTime = 15.25;
    std::vector<std::byte> endBuf(bb::kEndPayloadBytes);
    REQUIRE(bb::encodeEnd(endBuf, e) == bb::kEndPayloadBytes);
    bb::EndInfo outEnd;
    REQUIRE(bb::decodeEnd(endBuf, outEnd));
    CHECK(outEnd.tickFrames == 201);
    CHECK(outEnd.droppedFrames == 5);
    CHECK(outEnd.bytesBefore == 90000);
    CHECK(outEnd.messagesSeen == 33);
    CHECK(outEnd.brownout);
    CHECK(outEnd.deviceFailed);
    CHECK(outEnd.endTime == 15.25);
}

// Would catch: the triage frame losing the fault tick's own record — the single most
// valuable record in a dumped file, and the one a truncated dump must still contain.
TEST_CASE("blackbox: the triage frame carries the fault tick's whole record") {
    bb::TriageInfo info;
    info.fault = FaultCode::OdoStuck;
    info.brownout = true;
    info.tickIndex = 421;
    info.faultTime = 4.25;
    info.precedingTicks = 200;
    const DebugRecord tick = groundTruth(2.0);

    std::vector<std::byte> buf(bb::kTriagePayloadBytes);
    REQUIRE(bb::encodeTriage(buf, info, tick) == bb::kTriagePayloadBytes);

    bb::TriageInfo outInfo;
    DebugRecord outTick;
    bool corrupt = false;
    REQUIRE(bb::decodeTriage(buf, outInfo, outTick, corrupt));
    CHECK(outInfo.fault == FaultCode::OdoStuck);
    CHECK(outInfo.brownout);
    CHECK(outInfo.tickIndex == 421);
    CHECK(outInfo.faultTime == 4.25);
    CHECK(outInfo.precedingTicks == 200);
    checkSameRecord(tick, outTick);
}

// ── The byte-exact goldens (the round trip cannot see what these see) ────────────────

// Would catch: ANY change to the file header's layout — a reordered field, a resized
// text field, a version bump that forgot the code, the wrong endianness. Every
// expectation below is derived by hand from the documented layout, not from the
// encoder.
TEST_CASE("blackbox golden: the 256-byte file header, byte for byte") {
    SessionInfo info;
    info.buildHash = "0b4948a";
    info.routineId = "redLeftTall";
    info.alliance = "red";
    info.side = "left";
    info.portMap = "L1,2,3";
    const auto b = encodeOneHeader(info, 12.5);

    REQUIRE(b.size() == 256);
    CHECK(static_cast<char>(b[0]) == 'S');
    CHECK(static_cast<char>(b[1]) == 'H');
    CHECK(static_cast<char>(b[2]) == 'B');
    CHECK(static_cast<char>(b[3]) == 'B');
    CHECK(le16(b, 4) == 1);     // format version
    CHECK(le16(b, 6) == 256);   // header bytes
    CHECK(le16(b, 8) == 428);   // tick record bytes
    CHECK(le16(b, 10) == 0);    // flags
    // 12.5 is exactly 0x4029000000000000 in IEEE-754 binary64 (hand-derived).
    CHECK(le64(b, 12) == 0x4029000000000000ULL);
    CHECK(le32(b, 20) == 200);    // ring capacity
    CHECK(le32(b, 24) == 65536);  // byte budget

    const auto textAt = [&b](std::size_t at, std::size_t width) {
        std::string s;
        for (std::size_t i = 0; i < width; ++i) {
            const auto c = static_cast<char>(b[at + i]);
            if (c == '\0') {
                break;
            }
            s += c;
        }
        return s;
    };
    CHECK(textAt(28, 48) == "0b4948a");
    CHECK(textAt(76, 32) == "redLeftTall");
    CHECK(textAt(108, 16) == "red");
    CHECK(textAt(124, 16) == "left");
    CHECK(textAt(140, 96) == "L1,2,3");
    // Padding after each string, and the trailing reserved block, are ZERO — a
    // future field lands on zeros, never on somebody's leftovers.
    for (std::size_t i = 35; i < 76; ++i) {
        CHECK(static_cast<std::uint8_t>(b[i]) == 0U);
    }
    for (std::size_t i = 236; i < 256; ++i) {
        CHECK(static_cast<std::uint8_t>(b[i]) == 0U);
    }
}

// Would catch: A FIELD MOVED IN THE ENCODER. Each field is written alone into an
// otherwise-default record; the bytes must appear at the documented offset AND every
// other byte must stay zero. That second half is what catches two fields overlapping —
// a bug a round trip can hide completely when both sides make the same move.
TEST_CASE("blackbox golden: every tick field sits at its documented offset") {
    struct FieldCase {
        const char* name;
        std::size_t offset;
        std::uint64_t bits;  // expected little-endian content at `offset`
        std::size_t width;
        void (*set)(DebugRecord&);
    };

    // All bit patterns hand-derived: 1.0 = 0x3FF0…, 2.0 = 0x4000…, 3.0 = 0x4008…,
    // 4.0 = 0x4010…, 0.5 = 0x3FE0…, -1.0 = 0xBFF0…, 12.5 = 0x4029…
    static const FieldCase cases[] = {
        {"t", 0, 0x3FF0000000000000ULL, 8, [](DebugRecord& r) { r.t = units::Time{1.0}; }},
        {"dt", 8, 0x4000000000000000ULL, 8, [](DebugRecord& r) { r.dt = units::Time{2.0}; }},
        {"targetPose.x", 16, 0x4008000000000000ULL, 8,
         [](DebugRecord& r) { r.targetPose = Pose2d{units::Length{3.0}, units::Length{0.0}, Angle{}}; }},
        {"targetPose.y", 24, 0x4010000000000000ULL, 8,
         [](DebugRecord& r) { r.targetPose = Pose2d{units::Length{0.0}, units::Length{4.0}, Angle{}}; }},
        {"targetPose.heading", 32, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) {
             r.targetPose = Pose2d{units::Length{0.0}, units::Length{0.0}, Angle::radians(0.5)};
         }},
        {"measuredPose.x", 40, 0x3FF0000000000000ULL, 8,
         [](DebugRecord& r) { r.measuredPose = Pose2d{units::Length{1.0}, units::Length{0.0}, Angle{}}; }},
        {"measuredPose.y", 48, 0x4000000000000000ULL, 8,
         [](DebugRecord& r) { r.measuredPose = Pose2d{units::Length{0.0}, units::Length{2.0}, Angle{}}; }},
        {"measuredPose.heading", 56, 0xBFE0000000000000ULL, 8,
         [](DebugRecord& r) {
             r.measuredPose = Pose2d{units::Length{0.0}, units::Length{0.0}, Angle::radians(-0.5)};
         }},
        {"errorX", 64, 0x3FF0000000000000ULL, 8, [](DebugRecord& r) { r.errorX = units::Length{1.0}; }},
        {"errorY", 72, 0xBFF0000000000000ULL, 8, [](DebugRecord& r) { r.errorY = units::Length{-1.0}; }},
        {"errorHeading", 80, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) { r.errorHeading = units::AngleDim{0.5}; }},
        {"commanded.vx", 88, 0x4008000000000000ULL, 8,
         [](DebugRecord& r) {
             r.commanded = ChassisSpeeds{units::Velocity{3.0}, units::Velocity{0.0},
                                         units::AngularVelocity{0.0}};
         }},
        {"commanded.vy", 96, 0x4010000000000000ULL, 8,
         [](DebugRecord& r) {
             r.commanded = ChassisSpeeds{units::Velocity{0.0}, units::Velocity{4.0},
                                         units::AngularVelocity{0.0}};
         }},
        {"commanded.omega", 104, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) {
             r.commanded = ChassisSpeeds{units::Velocity{0.0}, units::Velocity{0.0},
                                         units::AngularVelocity{0.5}};
         }},
        {"wheelCount", 112, 4ULL, 1, [](DebugRecord& r) { r.wheelCount = 4; }},
        {"activeCommandState", 113, 3ULL, 1, [](DebugRecord& r) { r.activeCommandState = 3; }},
        {"deadReckoning", 114, 1ULL, 1, [](DebugRecord& r) { r.deadReckoning = true; }},
        {"qualityClass", 115, 2ULL, 1, [](DebugRecord& r) { r.qualityClass = 2; }},
        {"activeCommandId", 116, 7ULL, 4, [](DebugRecord& r) { r.activeCommandId = 7; }},
        {"fault", 120, 4ULL, 2, [](DebugRecord& r) { r.fault = FaultCode::OdoStuck; }},
        {"gateReason", 122, 3ULL, 1,
         [](DebugRecord& r) { r.gateReason = GateReason::RejectedMahalanobis; }},
        {"flags: clamped", 123, 1ULL, 1, [](DebugRecord& r) { r.clampedThisTick = true; }},
        {"flags: strafe fallback", 123, 2ULL, 1,
         [](DebugRecord& r) { r.strafeFallbackActive = true; }},
        {"droppedRecords", 124, 41ULL, 4, [](DebugRecord& r) { r.droppedRecords = 41; }},
        {"droppedLines", 128, 42ULL, 4, [](DebugRecord& r) { r.droppedLines = 42; }},
        {"wheelVoltage[0]", 132, 0x3FF0000000000000ULL, 8,
         [](DebugRecord& r) { r.wheelVoltage[0] = units::Voltage{1.0}; }},
        {"wheelVoltage[7]", 188, 0x4000000000000000ULL, 8,
         [](DebugRecord& r) { r.wheelVoltage[7] = units::Voltage{2.0}; }},
        {"wheelCurrent[0]", 196, 0x4008000000000000ULL, 8,
         [](DebugRecord& r) { r.wheelCurrent[0] = units::Current{3.0}; }},
        {"wheelCurrent[7]", 252, 0x4010000000000000ULL, 8,
         [](DebugRecord& r) { r.wheelCurrent[7] = units::Current{4.0}; }},
        {"imuYaw", 260, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) { r.imuYaw = Angle::radians(0.5); }},
        {"imuYawRate", 268, 0xBFF0000000000000ULL, 8,
         [](DebugRecord& r) { r.imuYawRate = units::AngularVelocity{-1.0}; }},
        {"quality", 276, 0x3FE0000000000000ULL, 8, [](DebugRecord& r) { r.quality = 0.5; }},
        {"covarianceTrace", 284, 0x4000000000000000ULL, 8,
         [](DebugRecord& r) { r.covarianceTrace = 2.0; }},
        {"gateResidualX", 292, 0x3FF0000000000000ULL, 8,
         [](DebugRecord& r) { r.gateResidualX = units::Length{1.0}; }},
        {"gateResidualY", 300, 0xC000000000000000ULL, 8,
         [](DebugRecord& r) { r.gateResidualY = units::Length{-2.0}; }},
        {"gateResidualHeading", 308, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) { r.gateResidualHeading = units::AngleDim{0.5}; }},
        {"gateMahalanobis", 316, 0x4010000000000000ULL, 8,
         [](DebugRecord& r) { r.gateMahalanobis = 4.0; }},
        {"correctionDx", 324, 0x3FF0000000000000ULL, 8,
         [](DebugRecord& r) { r.correctionDx = units::Length{1.0}; }},
        {"correctionDy", 332, 0x4000000000000000ULL, 8,
         [](DebugRecord& r) { r.correctionDy = units::Length{2.0}; }},
        {"correctionDTheta", 340, 0x3FE0000000000000ULL, 8,
         [](DebugRecord& r) { r.correctionDTheta = units::AngleDim{0.5}; }},
        {"batteryVoltage", 348, 0x4029000000000000ULL, 8,
         [](DebugRecord& r) { r.batteryVoltage = units::Voltage{12.5}; }},
        {"batteryCurrent", 356, 0x4008000000000000ULL, 8,
         [](DebugRecord& r) { r.batteryCurrent = units::Current{3.0}; }},
        {"tickPhase[0]", 364, 0x3FF0000000000000ULL, 8,
         [](DebugRecord& r) { r.tickPhase[0] = units::Time{1.0}; }},
        {"tickPhase[7]", 420, 0x4000000000000000ULL, 8,
         [](DebugRecord& r) { r.tickPhase[7] = units::Time{2.0}; }},
    };

    for (const FieldCase& c : cases) {
        CAPTURE(c.name);
        DebugRecord r;
        c.set(r);
        const auto b = encodeOneTick(r);
        REQUIRE(b.size() == 428);
        std::uint64_t got = 0;
        for (std::size_t i = 0; i < c.width; ++i) {
            got |= static_cast<std::uint64_t>(b[c.offset + i]) << (8 * i);
        }
        CHECK(got == c.bits);
        // Everything else must still be zero — this is the half that catches overlap.
        for (std::size_t i = 0; i < b.size(); ++i) {
            if (i >= c.offset && i < c.offset + c.width) {
                continue;
            }
            CAPTURE(i);
            CHECK(static_cast<std::uint8_t>(b[i]) == 0U);
        }
    }
}

// Would catch: a default-constructed record encoding to anything but zeros — which
// would mean the "quiet tick" is not quiet, and every all-other-bytes-zero check above
// would be resting on luck.
TEST_CASE("blackbox golden: a default record encodes to 428 zero bytes") {
    const auto b = encodeOneTick(DebugRecord{});
    REQUIRE(b.size() == 428);
    for (std::size_t i = 0; i < b.size(); ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::uint8_t>(b[i]) == 0U);
    }
}

// Would catch: a frame prefix that stops being {type, reserved, u16 length} — the one
// structure that lets a reader skip what it does not understand.
TEST_CASE("blackbox golden: the frame prefix") {
    std::array<std::byte, bb::kFrameHeaderBytes> head{};
    REQUIRE(bb::encodeFrameHeader(head, bb::FrameType::Triage, 452) == 4);
    CHECK(static_cast<std::uint8_t>(head[0]) == 3U);    // Triage
    CHECK(static_cast<std::uint8_t>(head[1]) == 0U);    // reserved
    CHECK(static_cast<std::uint8_t>(head[2]) == 0xC4);  // 452 = 0x01C4, little-endian
    CHECK(static_cast<std::uint8_t>(head[3]) == 0x01);
}

// ── Refusal, damage, and forward compatibility ──────────────────────────────────────

// Would catch: a decoder that reads a future file with today's layout and reports
// confident nonsense. The version byte is patched by hand — exactly what a file
// written by a later build looks like to this one.
TEST_CASE("blackbox: an unknown version is REFUSED, not misread") {
    FileBuilder file;
    file.addTick(groundTruth(0.0));
    file.bytes[4] = static_cast<std::byte>(2);  // formatVersion = 2

    bb::BlackboxReader reader{file.bytes};
    CHECK(reader.status() == bb::ReadStatus::UnsupportedVersion);
    CHECK_FALSE(reader.usable());
    CHECK(reader.header().formatVersion == 2);  // still tells you WHICH version it is
    bb::BlackboxReader::Frame frame;
    CHECK_FALSE(reader.next(frame));  // and hands out NOTHING
    CHECK(reader.framesRead() == 0);
}

// Would catch: a layout change made without a version bump. The file says its records
// are a different width than this build writes, so it is refused even though the
// version matches — the human error the width field exists for.
TEST_CASE("blackbox: right version, wrong record width is REFUSED") {
    FileBuilder file;
    file.addTick(groundTruth(0.0));
    file.bytes[8] = static_cast<std::byte>(0xAD);  // tickRecordBytes = 429
    file.bytes[9] = static_cast<std::byte>(0x01);

    bb::BlackboxReader reader{file.bytes};
    CHECK(reader.status() == bb::ReadStatus::LayoutMismatch);
    bb::BlackboxReader::Frame frame;
    CHECK_FALSE(reader.next(frame));
}

// Would catch: a reader that treats any buffer as a blackbox (and then walks off the
// end of it looking for frames).
TEST_CASE("blackbox: a file that is not a blackbox, an empty one, and a stub header") {
    {
        bb::BlackboxReader reader{std::span<const std::byte>{}};
        CHECK(reader.status() == bb::ReadStatus::Empty);
    }
    {
        std::vector<std::byte> tiny(bb::kHeaderBytes - 1, std::byte{0});
        bb::BlackboxReader reader{tiny};
        CHECK(reader.status() == bb::ReadStatus::HeaderTruncated);
        CHECK(reader.truncated());
    }
    {
        FileBuilder file;
        file.bytes[1] = static_cast<std::byte>('X');  // "SXBB"
        bb::BlackboxReader reader{file.bytes};
        CHECK(reader.status() == bb::ReadStatus::BadMagic);
    }
    CHECK(std::string_view{bb::readStatusName(bb::ReadStatus::UnsupportedVersion)}
          == "UNSUPPORTED_VERSION");
}

// Would catch: a truncated file being read as a complete one — the case that WILL
// occur in the field (a run that dies mid-write). Everything before the cut must
// decode, and the reader must say the story stops there.
TEST_CASE("blackbox: a truncated file decodes up to the cut and says so") {
    FileBuilder file;
    for (int i = 0; i < 4; ++i) {
        file.addTick(groundTruth(static_cast<double>(i)));
    }
    // Cut in the MIDDLE of the fourth tick frame.
    const std::size_t cut = bb::kHeaderBytes + 3 * (bb::kFrameHeaderBytes + bb::kTickPayloadBytes)
                            + 100;
    file.bytes.resize(cut);

    bb::BlackboxReader reader{file.bytes};
    REQUIRE(reader.status() == bb::ReadStatus::Ok);
    int seen = 0;
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(frame.payload, out, corrupt));
        CHECK(out.t.value() == 12.5 + seen);
        ++seen;
    }
    CHECK(seen == 3);  // the three whole frames, and not one byte of the fourth
    CHECK(reader.truncated());
    CHECK(reader.truncatedFrameType() == static_cast<std::uint8_t>(bb::FrameType::Tick));
    CHECK_FALSE(reader.sawEnd());  // no graceful end: THIS is how a cut run identifies itself
}

// Would catch: a cut that lands inside a frame PREFIX (fewer than 4 bytes left) being
// read as a zero-length frame instead of a truncation.
TEST_CASE("blackbox: a cut inside a frame prefix is a truncation, not a frame") {
    FileBuilder file;
    file.addTick(groundTruth(0.0));
    file.bytes.resize(bb::kHeaderBytes + bb::kFrameHeaderBytes + bb::kTickPayloadBytes + 2);

    bb::BlackboxReader reader{file.bytes};
    bb::BlackboxReader::Frame frame;
    CHECK(reader.next(frame));   // the whole first tick
    CHECK_FALSE(reader.next(frame));
    CHECK(reader.truncated());
    CHECK(reader.truncatedFrameType() == 0);
}

// Would catch: a reader that stops at (or guesses at) a frame type it does not know —
// the property that lets today's decoder read a file from a later writer.
TEST_CASE("blackbox: an unknown frame type is skipped by its length, and counted") {
    FileBuilder file;
    file.addTick(groundTruth(0.0));
    file.addFrame(static_cast<bb::FrameType>(99), std::vector<std::byte>(17, std::byte{0xAB}));
    file.addTick(groundTruth(1.0));

    bb::BlackboxReader reader{file.bytes};
    int ticks = 0;
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        CHECK(frame.type == bb::FrameType::Tick);
        ++ticks;
    }
    CHECK(ticks == 2);
    CHECK(reader.skippedFrames() == 1);
    CHECK_FALSE(reader.truncated());
}

// Would catch: a known frame type carrying the wrong payload size being decoded
// anyway — corruption read as data.
TEST_CASE("blackbox: a known frame with the wrong payload size is skipped, not decoded") {
    FileBuilder file;
    file.addFrame(bb::FrameType::Tick, std::vector<std::byte>(64, std::byte{0x11}));
    file.addTick(groundTruth(0.0));

    bb::BlackboxReader reader{file.bytes};
    bb::BlackboxReader::Frame frame;
    REQUIRE(reader.next(frame));
    CHECK(frame.payload.size() == bb::kTickPayloadBytes);  // the GOOD one
    CHECK(reader.skippedFrames() == 1);
}

// Would catch: a decoder that hands a corrupt file's bytes straight to math::Angle,
// whose factory rejects non-finite input BY PRECONDITION — i.e. a decoder that throws
// on exactly the file you most need to read.
TEST_CASE("blackbox: a corrupt (non-finite) heading decodes to zero and is reported") {
    auto bytes = encodeOneTick(groundTruth(0.0));
    // Overwrite measuredPose.heading (offset 56) with a quiet NaN.
    const std::uint64_t nanBits = 0x7FF8000000000000ULL;
    for (std::size_t i = 0; i < 8; ++i) {
        bytes[56 + i] = static_cast<std::byte>((nanBits >> (8 * i)) & 0xFFULL);
    }

    DebugRecord out;
    bool corrupt = false;
    REQUIRE(bb::decodeTick(bytes, out, corrupt));  // no throw, no abort
    CHECK(corrupt);
    CHECK(out.measuredPose.heading().radians() == 0.0);
    CHECK(out.measuredPose.x().value() == 23.25);  // the rest of the record still reads
}

// Would catch: an encoder that writes into a buffer too small for it (the one place a
// blackbox could corrupt memory instead of just losing data).
TEST_CASE("blackbox: encoding into an undersized buffer writes nothing and reports it") {
    std::vector<std::byte> tooSmall(bb::kTickPayloadBytes - 1, std::byte{0});
    CHECK(bb::encodeTick(tooSmall, groundTruth(0.0)) == 0U);
    for (const auto byteValue : tooSmall) {
        CHECK(static_cast<std::uint8_t>(byteValue) == 0U);
    }
    std::vector<std::byte> tinyHeader(bb::kHeaderBytes - 1, std::byte{0});
    CHECK(bb::encodeHeader(tinyHeader, SessionInfo{}, 1.0, 1, 1) == 0U);
}

// Would catch: a decoder that accepts a short payload and reads garbage past its end.
TEST_CASE("blackbox: decoding a short payload is refused") {
    const auto bytes = encodeOneTick(groundTruth(0.0));
    DebugRecord out;
    bool corrupt = false;
    CHECK_FALSE(bb::decodeTick(std::span<const std::byte>{bytes}.first(400), out, corrupt));
}
