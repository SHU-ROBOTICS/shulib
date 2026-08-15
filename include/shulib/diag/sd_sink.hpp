#pragma once
//
// SdSink — the BLACKBOX: a binary, versioned, session-stamped record of a run, written
// to the brain's SD card (master plan §18; diagnostics-plan D-6/D-7; WS13, chunk E1).
//
// ── Why it exists ───────────────────────────────────────────────────────────────────
// The terminal (A1/C5) is the primary debug surface, and at a competition there is no
// terminal. Without this file a field run is undiagnosable: the robot did something
// wrong, and the only record is what somebody remembers seeing. SdSink is the
// counterpart to TermSink on the same seam — ONE record, MANY sinks — so a field trace
// and a bench trace are the same data in two renderings.
//
// ── D-6, the flight recorder: always on, written only when something breaks ─────────
// A competition build cannot afford always-on SD writing, but the 200 ticks BEFORE a
// fault are exactly what you need and exactly what you never have. So by default this
// sink STREAMS NOTHING. Every record goes into a fixed RAM ring, overwriting the oldest;
// the device sees no bytes at all until a fault fires. Then, and only then, the sink
// writes: the triage block first, and the preceding ticks after it.
//
// ── The dump ORDER is a decision, not an accident ───────────────────────────────────
// The fault that triggers a dump may be a brownout — the condition least compatible
// with a long synchronous write — so the order is chosen assuming the write may be cut
// off partway:
//   1. the file header (provenance: which binary, which routine),
//   2. the TRIAGE frame, which carries the fault code, its time, the tick index, AND
//      THE COMPLETE RECORD OF THE FAULT TICK — the single most valuable record in the
//      file, written before anything else,
//   3. the preceding ticks, OLDEST FIRST.
// A file cut anywhere after step 2 still answers "what broke, when, and in what state".
// REJECTED: newest-first ticks (it protects the most recent ticks against a cut, but it
// puts every reader in reverse and makes a partial file's ordering depend on where the
// cut fell); embedding the fault tick only in the ring (then a cut in step 3 can lose
// the one record that names the failure).
//
// ── T1: there is no background task, and that is a decision ─────────────────────────
// The build order once specified "double-buffered off-task writes". C2/C4 decided this
// tree has NO background task, and that decision stands here: bytes are encoded into a
// caller-owned RAM buffer synchronously on the caller's task, and reach the device only
// when the caller says so — flush() at a motion boundary, close() at auton end, or the
// fault dump. A writer task would be this tree's first two-task design, is unbuildable
// PROS-free at M2, and would end host determinism: every closed-loop test in this
// project is reproducible from a seed BECAUSE there is exactly one task. The cost of
// caller-paced writing is that a caller who flushes too rarely overruns the byte budget
// — which is why the budget drops and COUNTS (principle 5: silent degradation is a bug).
//
// ── Bounded, and never auto-flushing ────────────────────────────────────────────────
// The buffer is caller-owned and fixed. When a frame does not fit, it is DROPPED WHOLE
// and counted — never half-written, never grown, and (outside the fault dump) never
// resolved by writing to the device behind the caller's back. An automatic flush would
// put an unpredictable multi-millisecond SD write inside an arbitrary control tick,
// which is the exact cost D-6 exists to avoid. The one exception is the fault dump,
// which MAY write immediately (cfg.flushOnFault, default true): the fault has already
// happened, the run is already compromised, and the evidence is worth one late tick.
//
// ── Storage is caller-owned, and must not live on a task stack ──────────────────────
// The ring and the buffer are spans the caller provides (the Localizer's corrector-span
// precedent). That is not ceremony: 200 records plus a 64 KB buffer is far more than a
// PROS task stack holds, so the storage must be static / file-scope / heap. The
// SdSinkBuffers helper below is the one-liner for the common case.
//
// ── Cost when disabled ──────────────────────────────────────────────────────────────
// With cfg.enabled == false, wantsRecord() is false, so hal::emitRecord never even
// BUILDS a record (A1's cost contract), no ring is touched and no byte is written.
// Pinned by test, not asserted in prose.
//
// ── What v1 does NOT carry ──────────────────────────────────────────────────────────
// The log() message channel (blackbox_format.hpp explains why). Lines handed to this
// sink are counted and the count is written into the end frame, so the omission is
// visible in the file instead of silent.
//
// Single-task by contract, like every sink in this tree. Nothing here allocates, and
// nothing here throws.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/block_sink.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::diag {

/// D-6's own number: the flight recorder holds the last 200 ticks (~2 s at a 100 Hz
/// loop). PROVISIONAL (A4: HA-58) — an INVENTED depth, not a measured one; R4 settles
/// how far back a real failure's cause actually sits.
inline constexpr std::size_t kDefaultFlightRingTicks = 200;

/// The recommended RAM byte budget for the staging buffer: 64 KiB. Stated honestly,
/// because the arithmetic matters — a full default dump is a triage frame plus 200 tick
/// frames, about 87 KB, so 64 KiB does NOT hold one: a dump of that size writes in two
/// device calls rather than one (supported and tested). Sizing the buffer to hold a
/// whole dump costs 88 KB of RAM permanently to save one write() call at the one moment
/// the run is already compromised, which is the wrong trade. PROVISIONAL (A4: HA-59) —
/// INVENTED; R4 measures what the brain can spare.
inline constexpr std::size_t kRecommendedBufferBytes = 65536;

/// Configuration for SdSink. Every default is the COMPETITION posture: the flight
/// recorder on, streaming off, dump on the first fault, and write it immediately.
struct SdSinkConfig {
    /// false ⇒ the sink is inert: wantsRecord() is false, so the record is never even
    /// built (A1's cost contract), and no byte is ever written.
    bool enabled = true;
    /// true ⇒ every record is staged as a Tick frame as it arrives (a bench/dev
    /// posture). false ⇒ D-6: records go to the RAM ring only, and reach the file only
    /// through a fault dump.
    bool streamTicks = false;
    /// Dump the flight recorder when a record carries a fault. FIRST fault only — the
    /// FaultLatch precedent: a cascade must not dump twenty times.
    bool dumpOnFault = true;
    /// Let the fault dump write to the device immediately (header note). false defers
    /// the bytes to the caller's next flush(), at the risk of losing them to a power
    /// loss — bounded, counted, and the caller's choice.
    bool flushOnFault = true;
};

/// Caller-owned storage for one SdSink. NEVER put this on a task stack (header note).
struct SdSinkStorage {
    /// The D-6 flight-recorder ring. May be empty (no flight recorder).
    std::span<DebugRecord> ring{};
    /// The staging buffer — this IS the byte budget. Must hold the header plus one
    /// triage frame (checked by precondition).
    std::span<std::byte> buffer{};
};

/// The one-liner for the common case: declare it at file scope (or as a static) and
/// hand view() to the sink.
///
///     static shulib::diag::SdSinkBuffers<200, 65536> blackboxRam;
///     shulib::diag::SdSink blackbox{card, clock, blackboxRam.view()};
template <std::size_t RingTicks, std::size_t BufferBytes>
struct SdSinkBuffers {
    /// The flight-recorder ring storage.
    std::array<DebugRecord, RingTicks> ring{};
    /// The staging-buffer storage.
    std::array<std::byte, BufferBytes> buffer{};
    /// A storage view over both arrays, for the SdSink constructor.
    [[nodiscard]] SdSinkStorage view() noexcept {
        return SdSinkStorage{std::span<DebugRecord>{ring}, std::span<std::byte>{buffer}};
    }
};

/// The blackbox: a binary, versioned, session-stamped record of a run on the brain's SD
/// card, behind the same ITelemetrySink seam TermSink sits on — one record, two
/// renderings. Its DEFAULT posture writes nothing at all: every record lands in the
/// caller's RAM ring, and bytes reach the device only on the first faulted record, on an
/// explicit flush(), or at close(). Lifecycle: open() once before the run, flush()
/// wherever a few milliseconds of IO is affordable, close() at the end; a clean run that
/// never had anything to say costs zero bytes. It never allocates, never throws, and —
/// outside the fault dump — never writes behind your back: a frame that does not fit the
/// buffer is dropped WHOLE and counted, so the file always explains its own gaps.
/// Single-task, like every sink here.
class SdSink final : public hal::ITelemetrySink {
public:
    /// `out` is the block device (R1's /usd/ adapter on the robot, FakeBlockSink in
    /// tests), `clock` stamps the run epoch and the end frame, `storage` is caller-owned
    /// (header note). All references must outlive the sink.
    SdSink(hal::IBlockSink& out, hal::IClock& clock, SdSinkStorage storage,
           const SdSinkConfig& config = {})
        : out_{&out}, clock_{&clock}, storage_{storage}, cfg_{config} {
        SHULIB_PRECONDITION(storage.buffer.size()
                                >= blackbox::kHeaderBytes + blackbox::kFrameHeaderBytes
                                       + blackbox::kTriagePayloadBytes,
                            "SdSink: the buffer must hold the header plus one triage frame — "
                            "the fault dump's most valuable bytes must always fit");
    }

    /// Stamp the run's provenance (§18.5) and take the epoch reading. Call once, before
    /// the run. The header is STAGED, not written — a run that never has anything to say
    /// still writes nothing at all. An EMPTY build hash stays empty all the way to disk:
    /// MISSING must stay loud, and a wrong hash is worse than an absent one.
    void open(const SessionInfo& info) noexcept {
        copyBounded(hash_, sizeof hash_, info.buildHash);
        copyBounded(routine_, sizeof routine_, info.routineId);
        copyBounded(alliance_, sizeof alliance_, info.alliance);
        copyBounded(side_, sizeof side_, info.side);
        copyBounded(portMap_, sizeof portMap_, info.portMap);
        epoch_ = now();
        opened_ = true;
    }

    /// v1 does not carry the message channel (header note). The line is counted so the
    /// omission is visible in the file's end frame rather than silent.
    void log(hal::LogLevel /*level*/, std::string_view /*subsystem*/,
             std::string_view /*message*/) override {
        ++messages_;
    }

    /// True while the sink is enabled — the ring needs every record even when nothing
    /// is being streamed. Overridden as a pair with emit(), per the seam contract.
    [[nodiscard]] bool wantsRecord() const noexcept override { return cfg_.enabled; }

    /// One tick: stream it if configured, dump on the FIRST faulted record, then push it
    /// into the flight ring. The dump runs BEFORE the push on purpose, so the dumped
    /// ticks are strictly the ones PRECEDING the fault and the fault tick itself appears
    /// exactly once (inside the triage frame).
    void emit(const DebugRecord& record) override {
        if (!cfg_.enabled) {
            return;
        }
        ++records_;
        if (record.fault == FaultCode::Brownout) {
            brownout_ = true;  // latched for the rest of the run
        }
        if (cfg_.streamTicks) {
            (void)stageTick(record, false);
        }
        if (cfg_.dumpOnFault && !dumped_ && record.fault != FaultCode::None) {
            dump(record.fault, record);
        }
        pushRing(record);
    }

    /// The end-of-run summary (§18.3) as a frame. The sink's OWN drop count rides along,
    /// so the file always explains its own gaps.
    void summarize(const RunSummary& summary) override {
        if (!cfg_.enabled) {
            return;
        }
        if (summary.brownout) {
            brownout_ = true;
        }
        const std::uint32_t dropped = dropped_;
        (void)stage(blackbox::FrameType::Summary, blackbox::kSummaryPayloadBytes, false,
                    [&](std::span<std::byte> out) {
                        return blackbox::encodeSummary(out, summary, dropped);
                    });
    }

    /// Push everything staged to the device. THIS is the caller-paced write (T1): call
    /// it at a motion boundary, at auton end, or wherever a few milliseconds of IO is
    /// affordable. Returns false if the device refused any byte; the staged bytes are
    /// dropped (and counted) either way, so a failing device can never grow the buffer.
    ///
    /// The cost this whole arrangement rests on: a flush of tens of kilobytes is assumed
    /// to take single-digit milliseconds — affordable HERE, and not affordable inside a
    /// 10 ms control tick. That assumption is INVENTED and the reason writes are
    /// caller-paced at all; PROVISIONAL (A4: HA-60), and R4 measures it. If the real
    /// figure is far worse, the flush POINTS move (fewer of them, or auton-end only) —
    /// the format and the sink do not.
    bool flush() noexcept {
        if (used_ == 0) {
            return !deviceFailed_;
        }
        const bool ok = out_->write(storage_.buffer.subspan(0, used_));
        if (ok) {
            bytesWritten_ += static_cast<std::uint32_t>(used_);
        } else {
            deviceFailed_ = true;
            dropped_ += pendingFrames_;  // frames that never reached the device ARE drops
        }
        used_ = 0;
        pendingFrames_ = 0;
        return ok;
    }

    /// Graceful end: write the end frame, flush, and flush the device. The end frame's
    /// PRESENCE is what tells a reader the run closed cleanly — its absence is how a
    /// truncated file identifies itself. Writes nothing at all if the run never had
    /// anything to say (D-6's promise: a clean run costs zero bytes).
    void close() noexcept {
        if (!cfg_.enabled || (framesStaged_ == 0 && bytesWritten_ == 0)) {
            return;
        }
        blackbox::EndInfo e;
        e.tickFrames = tickFrames_;
        e.droppedFrames = dropped_;
        e.bytesBefore = bytesWritten_ + static_cast<std::uint32_t>(used_);
        e.messagesSeen = messages_;
        e.brownout = brownout_;
        e.deviceFailed = deviceFailed_;
        e.endTime = now();
        (void)stage(blackbox::FrameType::End, blackbox::kEndPayloadBytes, true,
                    [&](std::span<std::byte> out) { return blackbox::encodeEnd(out, e); });
        (void)flush();
        (void)out_->flush();
        closed_ = true;
    }

    /// Latch the brownout marker from outside the record stream (HealthMonitor's
    /// brownedOut(), say). Latched for the run: a battery that recovers does not erase
    /// the fact that it collapsed.
    void markBrownout() noexcept { brownout_ = true; }

    /// Dump the flight recorder explicitly, for a fault that never rode a record.
    /// Honours the first-fault rule; returns false if a dump already happened or the
    /// sink is disabled.
    bool triggerDump(FaultCode fault, const DebugRecord& faultTick) noexcept {
        if (!cfg_.enabled || dumped_ || fault == FaultCode::None) {
            return false;
        }
        dump(fault, faultTick);
        return true;
    }

    // ── observation (all cheap, all const) ──────────────────────────────────────────

    /// Frames dropped for want of buffer, plus any staged frames a failed device write
    /// discarded. THE number for "what is missing from this file".
    [[nodiscard]] std::uint32_t droppedFrames() const noexcept { return dropped_; }
    /// Tick frames staged over the run (streamed plus dumped).
    [[nodiscard]] std::uint32_t tickFrames() const noexcept { return tickFrames_; }
    /// Records handed to emit() over the run.
    [[nodiscard]] std::uint32_t recordsSeen() const noexcept { return records_; }
    /// log() lines handed to the sink and not carried by v1 (header note).
    [[nodiscard]] std::uint32_t messagesSeen() const noexcept { return messages_; }
    /// Bytes the device confirmed. After a device failure this is a LOWER BOUND: a
    /// partial write's prefix is unknowable through the seam.
    [[nodiscard]] std::uint32_t bytesWritten() const noexcept { return bytesWritten_; }
    /// Bytes staged and not yet written.
    [[nodiscard]] std::size_t bytesBuffered() const noexcept { return used_; }
    /// True once the fault dump has fired (first fault only).
    [[nodiscard]] bool dumped() const noexcept { return dumped_; }
    /// The latched brownout marker.
    [[nodiscard]] bool brownout() const noexcept { return brownout_; }
    /// True once any write() or flush() reported failure.
    [[nodiscard]] bool deviceFailed() const noexcept { return deviceFailed_; }
    /// True once close() has run.
    [[nodiscard]] bool closed() const noexcept { return closed_; }
    /// How many records the flight ring currently holds.
    [[nodiscard]] std::size_t ringSize() const noexcept { return ringCount_; }
    /// The D-7 triage block for the dump that fired (all zeros until dumped()). The
    /// SAME struct that went into the file, so the terminal report (diag/triage.hpp,
    /// called by RunReporter at run end) and the blackbox cannot disagree.
    [[nodiscard]] const blackbox::TriageInfo& triage() const noexcept { return triage_; }
    /// The record of the tick the fault fired on (all defaults until dumped()).
    [[nodiscard]] const DebugRecord& triageTick() const noexcept { return triageTick_; }

private:
    [[nodiscard]] double now() const noexcept {
        try {
            return clock_->now().value();
        } catch (...) {
            return 0.0;  // a throwing clock violates its contract; it must not kill the record
        }
    }

    static void copyBounded(char* dst, std::size_t cap, std::string_view src) noexcept {
        const std::size_t take = src.size() < cap - 1 ? src.size() : cap - 1;
        for (std::size_t i = 0; i < take; ++i) {
            dst[i] = src[i];
        }
        dst[take] = '\0';
    }

    /// Stage the 256-byte header, once, before any frame. Lazy on purpose: a run that
    /// never stages a frame never writes a byte.
    void ensureHeader() noexcept {
        if (headerStaged_) {
            return;
        }
        headerStaged_ = true;  // set FIRST: a failure here must not retry forever
        SessionInfo info;
        info.buildHash = hash_;
        info.routineId = routine_;
        info.alliance = alliance_;
        info.side = side_;
        info.portMap = portMap_;
        if (!opened_) {
            epoch_ = now();
        }
        const std::size_t n = blackbox::encodeHeader(
            storage_.buffer.subspan(0, blackbox::kHeaderBytes), info, epoch_,
            static_cast<std::uint32_t>(storage_.ring.size()),
            static_cast<std::uint32_t>(storage_.buffer.size()));
        used_ = n;
    }

    /// Stage one frame, whole or not at all. `allowFlush` is true only on the fault-dump
    /// path (header note): everywhere else a full buffer means DROP AND COUNT, never a
    /// surprise write inside a control tick.
    template <typename EncodeFn>
    bool stage(blackbox::FrameType type, std::size_t payloadBytes, bool allowFlush,
               EncodeFn&& encode) noexcept {
        ensureHeader();
        const std::size_t need = blackbox::kFrameHeaderBytes + payloadBytes;
        if (storage_.buffer.size() - used_ < need) {
            if (allowFlush) {
                // The file is ONE stream across many write() calls: the header was
                // already staged (and written) at the front, so a mid-dump flush just
                // empties the buffer and the next frames append after it. Re-stamping
                // provenance here would corrupt the stream, not protect it.
                (void)flush();
            }
            if (storage_.buffer.size() - used_ < need) {
                ++dropped_;
                return false;
            }
        }
        if (blackbox::encodeFrameHeader(storage_.buffer.subspan(used_, blackbox::kFrameHeaderBytes),
                                        type, static_cast<std::uint16_t>(payloadBytes))
            != blackbox::kFrameHeaderBytes) {
            ++dropped_;
            return false;
        }
        const std::size_t n = encode(storage_.buffer.subspan(used_ + blackbox::kFrameHeaderBytes,
                                                             payloadBytes));
        if (n != payloadBytes) {
            ++dropped_;  // nothing was committed: used_ is untouched, so no half-frame exists
            return false;
        }
        used_ += need;
        ++framesStaged_;
        ++pendingFrames_;
        return true;
    }

    bool stageTick(const DebugRecord& r, bool allowFlush) noexcept {
        const bool ok = stage(blackbox::FrameType::Tick, blackbox::kTickPayloadBytes, allowFlush,
                              [&](std::span<std::byte> out) { return blackbox::encodeTick(out, r); });
        if (ok) {
            ++tickFrames_;
        }
        return ok;
    }

    /// D-6/D-7: triage first (with the fault tick inside it), then the preceding ticks
    /// oldest-first. See the dump-order note at the top of this file.
    void dump(FaultCode fault, const DebugRecord& faultTick) noexcept {
        dumped_ = true;
        const bool allowFlush = cfg_.flushOnFault;
        blackbox::TriageInfo& info = triage_;
        info.fault = fault;
        info.brownout = brownout_;
        info.tickIndex = records_;
        info.faultTime = faultTick.t.value();
        triageTick_ = faultTick;
        // When streaming, the preceding ticks are already in the file — re-dumping them
        // would duplicate the record stream, so the count says 0 and the reader looks
        // upstream instead.
        info.precedingTicks = cfg_.streamTicks ? 0U : static_cast<std::uint32_t>(ringCount_);
        (void)stage(blackbox::FrameType::Triage, blackbox::kTriagePayloadBytes, allowFlush,
                    [&](std::span<std::byte> out) {
                        return blackbox::encodeTriage(out, info, faultTick);
                    });
        if (!cfg_.streamTicks) {
            for (std::size_t i = 0; i < ringCount_; ++i) {
                const std::size_t idx = (ringHead_ + storage_.ring.size() - ringCount_ + i)
                                        % storage_.ring.size();
                (void)stageTick(storage_.ring[idx], allowFlush);
            }
        }
        if (allowFlush) {
            (void)flush();
            if (!out_->flush()) {
                deviceFailed_ = true;
            }
        }
    }

    /// Overwrite the OLDEST entry once the ring is full — the whole point of a flight
    /// recorder is that the newest ticks survive.
    void pushRing(const DebugRecord& r) noexcept {
        if (storage_.ring.empty()) {
            return;
        }
        storage_.ring[ringHead_] = r;
        ringHead_ = (ringHead_ + 1) % storage_.ring.size();
        if (ringCount_ < storage_.ring.size()) {
            ++ringCount_;
        }
    }

    hal::IBlockSink* out_;
    hal::IClock* clock_;
    SdSinkStorage storage_;
    SdSinkConfig cfg_;

    char hash_[48] = "";
    char routine_[32] = "";
    char alliance_[16] = "";
    char side_[16] = "";
    char portMap_[96] = "";

    blackbox::TriageInfo triage_{};  // the dump's own account (D-7), for the terminal too
    DebugRecord triageTick_{};       // the record of the tick the fault fired on

    double epoch_ = 0.0;
    std::size_t used_ = 0;        // bytes staged in the buffer
    std::size_t ringHead_ = 0;    // next ring slot to write
    std::size_t ringCount_ = 0;   // ring entries in use
    std::uint32_t records_ = 0;
    std::uint32_t messages_ = 0;
    std::uint32_t tickFrames_ = 0;
    std::uint32_t framesStaged_ = 0;
    std::uint32_t pendingFrames_ = 0;  // frames staged since the last successful write
    std::uint32_t dropped_ = 0;
    std::uint32_t bytesWritten_ = 0;
    bool opened_ = false;
    bool headerStaged_ = false;
    bool dumped_ = false;
    bool brownout_ = false;
    bool deviceFailed_ = false;
    bool closed_ = false;
};

}  // namespace shulib::diag
