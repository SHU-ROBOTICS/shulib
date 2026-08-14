// Tests for diag/sd_sink.hpp — the E1 blackbox sink. What each targets:
//
//  * D-6, THE FLIGHT RECORDER: always recording into RAM, writing NOTHING until a fault
//    fires, and then writing the ticks that PRECEDED the fault. A ring that overwrote
//    the wrong end, or a dump that captured the ticks after the fault, would both look
//    plausible in a header comment and be worthless in the field.
//  * D-7, THE TRIAGE BLOCK: the fault, its time, its tick, and the fault tick's own
//    record — written FIRST, so a dump cut off by the very brownout that triggered it
//    still answers "what broke".
//  * THE BYTE BUDGET: when the buffer is exhausted, frames are dropped WHOLE and
//    COUNTED, the loop is never blocked, memory never grows, and the file that does get
//    written still decodes. A silent drop reads as "nothing happened".
//  * THE GRACEFUL END: the end frame's presence means a clean close; its absence is how
//    a run that died mid-write identifies itself to a reader.
//  * COST WHEN DISABLED: with the sink off, hal::emitRecord must not even BUILD a
//    record (A1's cost contract) and no byte may reach the device.

#include "doctest.h"

#include <cstddef>
#include <string>
#include <vector>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"

using shulib::PreconditionError;
using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::RunSummary;
using shulib::diag::SdSink;
using shulib::diag::SdSinkConfig;
using shulib::diag::SdSinkStorage;
using shulib::diag::SessionInfo;
using shulib::hal::fake::FakeBlockSink;
using shulib::hal::fake::FakeClock;
namespace bb = shulib::diag::blackbox;
namespace units = shulib::units;

namespace {

/// A record whose `t` identifies it: every assertion below reads tick identity off
/// this one field, so "which ticks were dumped" is answerable exactly.
DebugRecord tickAt(double t, FaultCode fault = FaultCode::None) {
    DebugRecord r;
    r.t = units::Time{t};
    r.measuredPose = shulib::math::Pose2d{units::Length{t}, units::Length{-t},
                                          shulib::math::Angle{}};
    r.fault = fault;
    r.quality = 0.5;
    return r;
}

/// Storage sized per test. Deliberately NOT the SdSinkBuffers helper in most cases —
/// the tests want tiny rings and tiny budgets so wrap and exhaustion are cheap to
/// reach and easy to read.
struct Storage {
    std::vector<DebugRecord> ring;
    std::vector<std::byte> buffer;

    Storage(std::size_t ringTicks, std::size_t bufferBytes)
        : ring(ringTicks), buffer(bufferBytes) {}

    [[nodiscard]] SdSinkStorage view() { return SdSinkStorage{ring, buffer}; }
};

/// Everything a decoded file has to say, gathered in one pass.
struct Decoded {
    bb::ReadStatus status = bb::ReadStatus::Empty;
    bool truncated = false;
    bool sawEnd = false;
    bool corrupt = false;
    std::uint32_t skipped = 0;
    std::vector<DebugRecord> ticks;
    std::vector<bb::TriageInfo> triage;
    std::vector<DebugRecord> triageTicks;
    std::vector<RunSummary> summaries;
    std::vector<std::uint32_t> summaryDrops;
    std::vector<bb::EndInfo> ends;
    bb::BlackboxHeader header;
    /// Frame types in the order they appeared — the ORDER is a contract here.
    std::vector<bb::FrameType> order;
};

Decoded decode(const FakeBlockSink& device) {
    Decoded d;
    bb::BlackboxReader reader{device.view()};
    d.status = reader.status();
    d.header = reader.header();
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        d.order.push_back(frame.type);
        switch (frame.type) {
            case bb::FrameType::Tick: {
                DebugRecord r;
                REQUIRE(bb::decodeTick(frame.payload, r, d.corrupt));
                d.ticks.push_back(r);
                break;
            }
            case bb::FrameType::Triage: {
                bb::TriageInfo info;
                DebugRecord r;
                REQUIRE(bb::decodeTriage(frame.payload, info, r, d.corrupt));
                d.triage.push_back(info);
                d.triageTicks.push_back(r);
                break;
            }
            case bb::FrameType::Summary: {
                RunSummary s;
                std::uint32_t dropped = 0;
                REQUIRE(bb::decodeSummary(frame.payload, s, dropped));
                d.summaries.push_back(s);
                d.summaryDrops.push_back(dropped);
                break;
            }
            case bb::FrameType::End: {
                bb::EndInfo e;
                REQUIRE(bb::decodeEnd(frame.payload, e));
                d.ends.push_back(e);
                break;
            }
        }
    }
    d.truncated = reader.truncated();
    d.sawEnd = reader.sawEnd();
    d.skipped = reader.skippedFrames();
    return d;
}

SessionInfo demoSession() {
    SessionInfo info;
    info.buildHash = "0b4948a-dirty";
    info.routineId = "redLeftTall";
    info.alliance = "red";
    info.side = "left";
    info.portMap = "L1,2,3 R4,5,6 IMU10";
    return info;
}

}  // namespace

// Would catch: an "always-on" blackbox — the exact thing D-6 exists to avoid. A
// competition build that quietly writes every tick to the card blows the loop budget
// and the card, and nothing in a header comment would reveal it.
TEST_CASE("SdSink: with no fault, the flight recorder writes NOTHING") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{8, 4096};
    SdSink sink{device, clock, storage.view()};
    sink.open(demoSession());

    for (int i = 0; i < 500; ++i) {
        sink.emit(tickAt(i * 0.01));
    }
    CHECK(device.empty());
    CHECK(device.writeCalls() == 0);
    CHECK(sink.recordsSeen() == 500);
    CHECK(sink.ringSize() == 8);      // the ring is full and still recording
    CHECK(sink.droppedFrames() == 0); // nothing was dropped: nothing was staged
    CHECK_FALSE(sink.dumped());

    // …and a run that never had anything to say leaves no file at all.
    sink.close();
    CHECK(device.empty());
}

// Would catch: THE RING OVERWRITING THE WRONG END. A ring that discards the newest
// ticks instead of the oldest still "holds N records" and still dumps N frames — it
// just hands you the beginning of the run instead of the moments before the failure.
TEST_CASE("SdSink: the dump carries the ticks PRECEDING the fault, oldest first") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 8192};
    SdSink sink{device, clock, storage.view()};
    sink.open(demoSession());

    for (int i = 1; i <= 9; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    sink.emit(tickAt(10.0, FaultCode::OdoStuck));  // the fault tick

    CHECK(sink.dumped());
    CHECK_FALSE(device.empty());

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    // Order is the contract: triage FIRST (it survives a cut), then history.
    REQUIRE(d.order.size() == 5);
    CHECK(d.order[0] == bb::FrameType::Triage);
    for (std::size_t i = 1; i < d.order.size(); ++i) {
        CHECK(d.order[i] == bb::FrameType::Tick);
    }
    REQUIRE(d.triage.size() == 1);
    CHECK(d.triage[0].fault == FaultCode::OdoStuck);
    CHECK(d.triage[0].tickIndex == 10);
    CHECK(d.triage[0].faultTime == 10.0);
    CHECK(d.triage[0].precedingTicks == 4);
    CHECK(d.triageTicks[0].t.value() == 10.0);  // the fault tick's own record, in full

    // The four ticks BEFORE the fault, in chronological order — 6,7,8,9, not 1,2,3,4.
    REQUIRE(d.ticks.size() == 4);
    CHECK(d.ticks[0].t.value() == 6.0);
    CHECK(d.ticks[1].t.value() == 7.0);
    CHECK(d.ticks[2].t.value() == 8.0);
    CHECK(d.ticks[3].t.value() == 9.0);
    // The fault tick appears exactly ONCE (in the triage), never duplicated as history.
    for (const DebugRecord& r : d.ticks) {
        CHECK(r.t.value() != 10.0);
    }
}

// Would catch: a cascade dumping over and over. One root cause typically triggers a
// burst of follow-on faults; twenty dumps would bury the first one and blow the budget
// at the worst possible moment. The FaultLatch keeps the FIRST fault; so does this.
TEST_CASE("SdSink: only the FIRST fault dumps") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 16384};
    SdSink sink{device, clock, storage.view()};

    sink.emit(tickAt(1.0));
    sink.emit(tickAt(2.0, FaultCode::NanPose));
    const std::size_t afterFirst = device.size();
    for (int i = 3; i < 8; ++i) {
        sink.emit(tickAt(static_cast<double>(i), FaultCode::OdoStuck));
    }
    CHECK(device.size() == afterFirst);
    CHECK_FALSE(sink.triggerDump(FaultCode::Brownout, tickAt(9.0)));  // and not by hand either

    const Decoded d = decode(device);
    CHECK(d.triage.size() == 1);
    CHECK(d.triage[0].fault == FaultCode::NanPose);  // the ROOT CAUSE, not the loudest
}

// Would catch: a dropped frame that is not counted (the drop-counter increment is the
// mutation target), a buffer that grows to fit, a device written to behind the
// caller's back, or a half-written frame corrupting the frames around it.
TEST_CASE("SdSink: budget exhaustion drops WHOLE frames, counts them, and the file still decodes") {
    FakeBlockSink device;
    FakeClock clock;
    // Room for the header plus exactly three tick frames.
    const std::size_t budget = bb::kHeaderBytes + 3 * (bb::kFrameHeaderBytes + bb::kTickPayloadBytes);
    Storage storage{4, budget};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.streamTicks = true}};
    sink.open(demoSession());

    for (int i = 1; i <= 10; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    // Nothing was written behind the caller's back, and memory did not grow.
    CHECK(device.empty());
    CHECK(sink.bytesBuffered() == budget);
    CHECK(sink.droppedFrames() == 7);
    CHECK(sink.tickFrames() == 3);

    CHECK(sink.flush());
    CHECK(device.size() == budget);

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    CHECK_FALSE(d.truncated);       // whole frames only — a drop is not a truncation
    REQUIRE(d.ticks.size() == 3);
    CHECK(d.ticks[0].t.value() == 1.0);
    CHECK(d.ticks[1].t.value() == 2.0);
    CHECK(d.ticks[2].t.value() == 3.0);
}

// Would catch: the drop count never reaching the run report — a file with holes and a
// summary that says everything is fine.
TEST_CASE("SdSink: the drop count rides into the summary frame and the accessor") {
    FakeBlockSink device;
    FakeClock clock;
    const std::size_t budget = bb::kHeaderBytes + bb::kFrameHeaderBytes + bb::kTickPayloadBytes
                               + bb::kFrameHeaderBytes + bb::kSummaryPayloadBytes;
    Storage storage{2, budget};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.streamTicks = true}};

    for (int i = 0; i < 5; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    CHECK(sink.droppedFrames() == 4);

    RunSummary s;
    s.motionsStarted = 3;
    s.blackboxDropped = 0;  // a caller who assembled the summary early is not trusted:
    sink.summarize(s);      // the sink writes ITS OWN live count into the file
    CHECK(sink.flush());

    const Decoded d = decode(device);
    REQUIRE(d.summaries.size() == 1);
    CHECK(d.summaries[0].motionsStarted == 3);
    REQUIRE(d.summaryDrops.size() == 1);
    CHECK(d.summaryDrops[0] == 4);
}

// Would catch: a device failure that looks like success — the card is full, the run
// looks recorded, and the file is half a file. Also the case the graceful-end contract
// is written for: the bytes that DID land must still decode.
TEST_CASE("SdSink: a device that dies mid-write leaves a truncated, readable file") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{8, 16384};
    // The card accepts the header, two whole tick frames, and then 100 bytes of the third.
    device.setCapacity(bb::kHeaderBytes + 2 * (bb::kFrameHeaderBytes + bb::kTickPayloadBytes) + 100);
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.streamTicks = true}};
    sink.open(demoSession());

    for (int i = 1; i <= 5; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    CHECK_FALSE(sink.flush());
    CHECK(sink.deviceFailed());
    CHECK(sink.droppedFrames() == 5);  // frames that never reached the device ARE drops

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);       // the header landed: the file opens
    CHECK(d.truncated);                            // and it says where it stops
    CHECK_FALSE(d.sawEnd);                         // no graceful end — the run was cut
    REQUIRE(d.ticks.size() == 2);
    CHECK(d.ticks[0].t.value() == 1.0);
    CHECK(d.ticks[1].t.value() == 2.0);
}

// Would catch: a disabled sink that still costs the hot path. With nothing consuming
// records, emitRecord must not even RUN the builder — the A1 cost contract, pinned the
// same way debug_record_test pins it for NullSink.
TEST_CASE("SdSink: disabled costs nothing — the record is never even built") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{8, 4096};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.enabled = false}};
    sink.open(demoSession());

    int builds = 0;
    shulib::hal::ITelemetrySink& seam = sink;
    CHECK_FALSE(seam.wantsRecord());
    for (int i = 0; i < 100; ++i) {
        shulib::hal::emitRecord(seam, [&] {
            ++builds;
            return tickAt(1.0, FaultCode::NanPose);
        });
    }
    CHECK(builds == 0);
    CHECK(device.empty());
    CHECK(sink.recordsSeen() == 0);
    CHECK(sink.ringSize() == 0);

    // Even a fault handed to it directly cannot make a disabled sink write.
    CHECK_FALSE(sink.triggerDump(FaultCode::Brownout, tickAt(2.0)));
    sink.summarize(RunSummary{});
    sink.close();
    CHECK(device.empty());
}

// Would catch: a clean run leaving no evidence it happened, or a close() that forgets
// the end stamp (which is the ONLY thing distinguishing a clean end from a cut one).
TEST_CASE("SdSink: close() stamps the graceful end") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 8192};
    SdSink sink{device, clock, storage.view()};
    sink.open(demoSession());

    for (int i = 0; i < 3; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    sink.log(shulib::hal::LogLevel::Warn, "MOT", "a line v1 does not carry");
    sink.log(shulib::hal::LogLevel::Info, "SES", "another");
    RunSummary s;
    s.motionsSettled = 2;
    sink.summarize(s);
    clock.advance(units::Time{15.5});
    sink.close();

    CHECK(sink.closed());
    CHECK(device.flushCalls() == 1);  // the device was told to commit, exactly once

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    CHECK(d.sawEnd);
    CHECK_FALSE(d.truncated);
    CHECK(d.ticks.empty());  // ring-only posture: no fault, no history in the file
    REQUIRE(d.summaries.size() == 1);
    CHECK(d.summaries[0].motionsSettled == 2);
    REQUIRE(d.ends.size() == 1);
    CHECK(d.ends[0].tickFrames == 0);
    CHECK(d.ends[0].droppedFrames == 0);
    CHECK(d.ends[0].messagesSeen == 2);  // the omission is VISIBLE, not silent
    CHECK(d.ends[0].endTime == 15.5);
    CHECK_FALSE(d.ends[0].deviceFailed);
    CHECK(d.header.buildHash() == "0b4948a-dirty");  // provenance survived to the file
}

// FOUND BY MUTATION (E1 green hole #1). Would catch: the end frame reporting zeros for
// what the run actually contained. Every other test read the FRAMES; none read the
// sink's own account of them, so `tickFrames`/`bytesWritten` could be hard-coded to 0
// with the whole suite green. A file that under-reports its own contents sends a reader
// hunting for data that was never missing — and one that over-reports hides a real gap.
TEST_CASE("SdSink: the end frame's own counts describe the run truthfully") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 32768};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.streamTicks = true}};
    sink.open(demoSession());

    for (int i = 1; i <= 7; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    sink.log(shulib::hal::LogLevel::Info, "MOT", "one line");
    sink.close();

    const Decoded d = decode(device);
    REQUIRE(d.ends.size() == 1);
    CHECK(d.ends[0].tickFrames == 7);                 // exactly what was staged
    CHECK(d.ticks.size() == 7);                       // …and exactly what is in the file
    CHECK(d.ends[0].messagesSeen == 1);
    CHECK(d.ends[0].droppedFrames == 0);
    CHECK_FALSE(d.ends[0].deviceFailed);
    // bytesBefore is this frame's own offset, so it must equal the file size minus
    // the end frame itself — the self-consistency check a reader can run.
    CHECK(d.ends[0].bytesBefore
          == device.size() - (bb::kFrameHeaderBytes + bb::kEndPayloadBytes));
    CHECK(d.ends[0].bytesBefore > 0);
}

// FOUND BY MUTATION (E1 green hole #2). Would catch: the SINK writing zeros into the
// header it stamps. The format tests pinned these fields by calling encodeHeader
// directly, which proves the FORMAT can carry them and says nothing about whether the
// sink supplies them. An epoch of 0 mis-times the whole file, and a ring capacity of 0
// removes a reader's only way to know how deep the captured history goes.
TEST_CASE("SdSink: the header the SINK stamps describes the sink") {
    FakeBlockSink device;
    FakeClock clock;
    clock.set(units::Time{7.25});
    Storage storage{12, 9000};
    SdSink sink{device, clock, storage.view()};
    sink.open(demoSession());
    clock.advance(units::Time{3.0});  // the epoch is taken at open(), not at first write

    sink.emit(tickAt(1.0, FaultCode::ImuLost));  // force the header out to the device

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    CHECK(d.header.epochSeconds == 7.25);
    CHECK(d.header.ringCapacity == 12);
    CHECK(d.header.byteBudget == 9000);
    CHECK(d.header.formatVersion == bb::kFormatVersion);
    CHECK(d.header.routineId() == "redLeftTall");
}

// Would catch: provenance longer than its field corrupting the fixed-width header (or
// being silently dropped). A 60-character hash is not hypothetical — a full SHA plus a
// suffix is 47, and nothing stops a build system from passing more.
TEST_CASE("SdSink: over-long provenance is truncated, not overflowed") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{2, 8192};
    SdSink sink{device, clock, storage.view()};
    SessionInfo info;
    const std::string longHash(120, 'h');
    const std::string longRoutine(120, 'r');
    const std::string longPorts(300, 'p');
    info.buildHash = longHash;
    info.routineId = longRoutine;
    info.portMap = longPorts;
    sink.open(info);
    sink.emit(tickAt(1.0, FaultCode::NanPose));

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);   // the header is still a valid header
    CHECK(d.header.buildHash().size() == 47);  // 48-byte field, NUL-terminated
    CHECK(d.header.routineId().size() == 31);
    CHECK(d.header.portMap().size() == 95);
    CHECK(d.header.buildHash()[0] == 'h');
    CHECK(d.header.routineId()[0] == 'r');
}

// Would catch: a brownout marker that unlatches when the battery recovers. The run that
// browned out must stay identifiable afterwards — that is the whole point of a latch.
TEST_CASE("SdSink: the brownout marker latches, and reaches both the triage and the end") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 16384};
    SdSink sink{device, clock, storage.view()};

    sink.emit(tickAt(1.0));
    sink.emit(tickAt(2.0, FaultCode::Brownout));
    CHECK(sink.brownout());
    for (int i = 3; i < 6; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));  // battery recovers; the latch does not
    }
    CHECK(sink.brownout());
    sink.close();

    const Decoded d = decode(device);
    REQUIRE(d.triage.size() == 1);
    CHECK(d.triage[0].fault == FaultCode::Brownout);
    CHECK(d.triage[0].brownout);
    REQUIRE(d.ends.size() == 1);
    CHECK(d.ends[0].brownout);
}

// Would catch: markBrownout() being decorative. A brownout detected by HealthMonitor
// (not by a record's fault field) must still mark the file.
TEST_CASE("SdSink: markBrownout() latches from outside the record stream") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 8192};
    SdSink sink{device, clock, storage.view()};
    sink.emit(tickAt(1.0));
    sink.markBrownout();
    CHECK(sink.brownout());
    sink.summarize(RunSummary{});
    sink.close();
    const Decoded d = decode(device);
    REQUIRE(d.ends.size() == 1);
    CHECK(d.ends[0].brownout);
}

// Would catch: a fault raised outside the record stream (between motions, at a
// boundary) leaving no blackbox evidence.
TEST_CASE("SdSink: triggerDump() dumps a fault that never rode a record") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 16384};
    SdSink sink{device, clock, storage.view()};
    for (int i = 1; i <= 6; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    CHECK(sink.triggerDump(FaultCode::MotionTimeout, tickAt(7.0)));

    const Decoded d = decode(device);
    REQUIRE(d.triage.size() == 1);
    CHECK(d.triage[0].fault == FaultCode::MotionTimeout);
    CHECK(d.triage[0].tickIndex == 6);
    REQUIRE(d.ticks.size() == 4);
    CHECK(d.ticks[0].t.value() == 3.0);  // the last four, oldest first
    CHECK(d.ticks[3].t.value() == 6.0);
}

// Would catch: a streaming run duplicating its whole history at the fault (the ticks
// are already in the file), which would double the worst-case write at the worst
// possible moment.
TEST_CASE("SdSink: streaming mode adds only the triage frame on a fault") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 32768};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.streamTicks = true}};

    for (int i = 1; i <= 5; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    sink.emit(tickAt(6.0, FaultCode::ImuLost));
    sink.close();

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    CHECK(d.ticks.size() == 6);  // the streamed history, once
    REQUIRE(d.triage.size() == 1);
    CHECK(d.triage[0].precedingTicks == 0);  // "look upstream in the file"
    CHECK(d.triage[0].fault == FaultCode::ImuLost);
    // The streamed fault tick, then the triage — the fault tick is in the stream AND in
    // the triage frame, which is the deliberate duplication that survives a cut.
    CHECK(d.order[5] == bb::FrameType::Tick);
    CHECK(d.order[6] == bb::FrameType::Triage);
}

// Would catch: a dump bigger than the buffer silently losing its tail, or the header
// being re-stamped mid-file when a dump needs more than one device write (which would
// leave a second "SHBB" in the middle of the stream and confuse every reader).
TEST_CASE("SdSink: a dump larger than the buffer flushes as it goes, in one valid stream") {
    FakeBlockSink device;
    FakeClock clock;
    // Buffer holds the header plus a triage frame plus ~2 ticks; the ring holds 10.
    const std::size_t budget = bb::kHeaderBytes + bb::kFrameHeaderBytes + bb::kTriagePayloadBytes
                               + 2 * (bb::kFrameHeaderBytes + bb::kTickPayloadBytes);
    Storage storage{10, budget};
    SdSink sink{device, clock, storage.view()};
    sink.open(demoSession());

    for (int i = 1; i <= 10; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    sink.emit(tickAt(11.0, FaultCode::LoopOverrun));

    CHECK(device.writeCalls() > 1);       // it had to write more than once…
    CHECK(sink.droppedFrames() == 0);     // …and nothing was lost doing it

    const Decoded d = decode(device);
    REQUIRE(d.status == bb::ReadStatus::Ok);
    CHECK_FALSE(d.truncated);
    REQUIRE(d.triage.size() == 1);
    REQUIRE(d.ticks.size() == 10);
    CHECK(d.ticks[0].t.value() == 1.0);
    CHECK(d.ticks[9].t.value() == 10.0);
}

// Would catch: a caller who defers the dump losing it silently. With flushOnFault off
// the bytes wait for the caller's own flush — bounded, counted, and the caller's choice.
TEST_CASE("SdSink: flushOnFault=false defers the write to the caller") {
    FakeBlockSink device;
    FakeClock clock;
    Storage storage{4, 16384};
    SdSink sink{device, clock, storage.view(), SdSinkConfig{.flushOnFault = false}};

    sink.emit(tickAt(1.0));
    sink.emit(tickAt(2.0, FaultCode::NanPose));
    CHECK(sink.dumped());
    CHECK(device.empty());              // not one byte during the tick
    CHECK(sink.bytesBuffered() > 0);
    CHECK(sink.flush());                // the caller pays for IO where it chose to
    CHECK_FALSE(device.empty());
    const Decoded d = decode(device);
    CHECK(d.triage.size() == 1);
}

// Would catch: storage too small for the one frame that matters. A buffer that cannot
// hold a triage frame would drop the dump at the exact moment it is needed, so it is
// refused at construction rather than discovered at 2am.
TEST_CASE("SdSink: a buffer too small for a triage frame is refused at construction") {
    FakeBlockSink device;
    FakeClock clock;
    Storage tooSmall{4, bb::kHeaderBytes + bb::kTriagePayloadBytes};  // one byte short of legal
    CHECK_THROWS_AS((SdSink{device, clock, tooSmall.view()}), PreconditionError);

    Storage justEnough{4, bb::kHeaderBytes + bb::kFrameHeaderBytes + bb::kTriagePayloadBytes};
    CHECK_NOTHROW((SdSink{device, clock, justEnough.view()}));
}

// Would catch: a flight recorder with no ring silently pretending to record. A
// streaming-only configuration is legal; it must simply have nothing to dump.
TEST_CASE("SdSink: with no ring, a fault still writes its triage block") {
    FakeBlockSink device;
    FakeClock clock;
    std::vector<std::byte> buffer(8192);
    SdSinkStorage storage{{}, buffer};
    SdSink sink{device, clock, storage};

    sink.emit(tickAt(1.0));
    sink.emit(tickAt(2.0, FaultCode::Precondition));
    const Decoded d = decode(device);
    REQUIRE(d.triage.size() == 1);
    CHECK(d.triage[0].precedingTicks == 0);
    CHECK(d.ticks.empty());
    CHECK(d.triageTicks[0].t.value() == 2.0);  // the fault tick itself is never lost
}

// Would catch: the SdSinkBuffers helper (the recommended one-liner) not actually
// wiring its own arrays — an easy copy-paste error that would leave the sink pointing
// at empty spans.
TEST_CASE("SdSink: the SdSinkBuffers helper wires real storage") {
    static shulib::diag::SdSinkBuffers<16, 8192> ram;
    FakeBlockSink device;
    FakeClock clock;
    SdSink sink{device, clock, ram.view()};
    CHECK(ram.view().ring.size() == 16);
    CHECK(ram.view().buffer.size() == 8192);
    for (int i = 0; i < 20; ++i) {
        sink.emit(tickAt(static_cast<double>(i)));
    }
    CHECK(sink.ringSize() == 16);
}
