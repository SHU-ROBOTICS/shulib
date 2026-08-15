#pragma once
//
// RateLimitedSink — per-channel rate limiting with COUNTED, REPORTED drops
// (diagnostics-plan D-2; §18.2 "rate-budgeted" / §18.3 "throttled"; WS13, chunk C5).
//
// Why: a high-rate channel (a 100 Hz record stream, a chatty Debug tag) must not
// drown the terminal or push the loop over budget — but a SILENT drop reads as
// "nothing happened", which is how an afternoon is spent debugging a problem that
// was never there. So every drop is:
//   1. COUNTED   — cumulative droppedRecords()/droppedLines() accessors,
//   2. ON-WIRE   — the counts are stamped into the D-2 schema fields of every
//                  record this sink FORWARDS (a gap in the stream carries its own
//                  explanation in the surviving records themselves),
//   3. ANNOUNCED — when a line-channel drop episode ends, ONE Warn notice names
//                  the channel and the count ("throttled MOT: dropped 47 lines"),
//   4. SUMMARIZED — RunReporter reads the totals into the §18.3 summary block.
//
// What is NEVER throttled (each a deliberate contract, pinned by test):
//   * Error and Warn lines — the error path is sacred; FaultLatch lines ride
//     log(Error) and a throttled fault is a lost root cause.
//   * summarize() — one struct per run is not a rate problem, and eating the
//     summary would be the decorator-swallows-it bug the seam warns about.
//
// Mechanism: token buckets (capacity = one second's budget, starting full,
// continuous refill on the injected clock — deterministic under FakeClock).
// Line channels are keyed by subsystem tag in a bounded table; tags beyond the
// table share one overflow bucket (bounded memory beats per-tag fairness for
// hypothetical tag #17, and the sharing is documented rather than silent).
//
// Cost note (the A1 contract, stated honestly): wantsRecord() forwards the INNER
// sink's answer even when the record bucket is empty — a throttled tick still
// pays record population so the drop can be SEEN and counted at emit(). Counting
// inside wantsRecord() instead would make a query with no call-count contract
// mutate state (unreliable), and skipping population would make drops invisible —
// the exact failure D-2 exists to prevent. With NullSink inner, wantsRecord() is
// false and nothing is built or counted: the competition build stays free.
//
// The default budgets are TERMINAL-BANDWIDTH choices (a 115200-baud serial console
// renders ~120 of these lines/s; half a stream of 100 Hz records is plenty for a
// live eye), not hardware claims — logic constants, no register entry, and any
// dev can retune them per session via the config.
//
// Single-task by contract, like every sink in this tree.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::diag {

/// The two per-second budgets. Both are TERMINAL-BANDWIDTH choices rather than hardware
/// limits — a 115200-baud console renders roughly 120 of these lines a second — so
/// retuning them per session is expected, not exceptional. Each bucket holds one second's
/// worth and STARTS FULL, so a burst at t=0 passes before throttling bites.
struct RateLimitConfig {
    /// emit()-channel budget, records/second. Must be > 0 and finite.
    double recordsPerSecond = 50.0;
    /// log()-channel budget PER SUBSYSTEM TAG, lines/second (Info/Debug/Trace
    /// only — Error/Warn are exempt; header note). Must be > 0 and finite.
    double linesPerSecondPerChannel = 20.0;
};

/// A pass-through ITelemetrySink decorator that caps what each channel may forward per
/// second — and COUNTS, STAMPS and ANNOUNCES everything it drops, because a silent drop
/// reads as "nothing happened", which is how an afternoon is lost to a problem that was
/// never there. Error and Warn lines and summarize() are never throttled. Holds `inner`
/// and `clock` by NON-OWNING reference; both must outlive the sink. Single-task by
/// contract, like every sink in this tree, and it allocates nothing.
class RateLimitedSink final : public hal::ITelemetrySink {
public:
    /// `inner` and `clock` must outlive the sink.
    RateLimitedSink(hal::ITelemetrySink& inner, hal::IClock& clock,
                    const RateLimitConfig& config = {})
        : inner_{&inner}, clock_{clock}, cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.recordsPerSecond) && cfg_.recordsPerSecond > 0.0,
                            "RateLimitedSink: recordsPerSecond must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(cfg_.linesPerSecondPerChannel)
                                && cfg_.linesPerSecondPerChannel > 0.0,
                            "RateLimitedSink: linesPerSecondPerChannel must be finite and > 0");
        recordBucket_.tokens = cfg_.recordsPerSecond;  // buckets start FULL (burst-friendly)
    }

    /// Forward one line unless this subsystem's bucket is empty. Error and Warn ALWAYS
    /// pass — a throttled fault is a lost root cause. Budgets are PER TAG, in a bounded
    /// table of 16; a 17th tag, a tag over 16 bytes, and the empty tag all share ONE
    /// overflow bucket (bounded memory beats fairness for a hypothetical tag, and the
    /// sharing is documented rather than silent). When a throttled tag resumes, ONE Warn
    /// "throttled TAG: dropped N lines" goes out under the "DIA" tag BEFORE the resuming
    /// line, so the gap is explained exactly where it sits.
    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        // The error path is never throttled (header contract).
        if (level == hal::LogLevel::Error || level == hal::LogLevel::Warn) {
            inner_->log(level, subsystem, message);
            return;
        }
        Channel& ch = channelFor(subsystem);
        refill(ch.bucket, cfg_.linesPerSecondPerChannel);
        if (ch.bucket.tokens < 1.0) {
            ++droppedLines_;
            ++ch.pendingDropped;
            return;
        }
        ch.bucket.tokens -= 1.0;
        if (ch.pendingDropped > 0) {
            // The episode ends where the channel resumes: one structured notice,
            // BEFORE the resuming line, so the gap is explained where it sits.
            char buf[96];
            // %lu + cast: uint32_t is unsigned int on the host but unsigned long
            // on the ARM target — the cast is what keeps -Werror=format clean on
            // BOTH (caught by the A4 ARM gate at C5).
            std::snprintf(buf, sizeof buf, "throttled %.*s: dropped %lu lines",
                          static_cast<int>(subsystem.size()), subsystem.data(),
                          static_cast<unsigned long>(ch.pendingDropped));
            inner_->log(hal::LogLevel::Warn, "DIA", buf);
            ch.pendingDropped = 0;
        }
        inner_->log(level, subsystem, message);
    }

    /// Forwards the INNER answer even when the bucket is empty (header cost note:
    /// drops must be seen to be counted) — the pair rule, one level up.
    [[nodiscard]] bool wantsRecord() const noexcept override { return inner_->wantsRecord(); }

    /// Forward one record unless the record bucket is empty, STAMPING the running drop
    /// totals onto the copy that survives — so a gap in the stream carries its own
    /// explanation in the records around it, with no second channel to correlate. The
    /// caller has ALREADY paid to populate `record` (see wantsRecord()): throttling here
    /// buys bandwidth, not the cost of building it.
    void emit(const DebugRecord& record) override {
        refill(recordBucket_, cfg_.recordsPerSecond);
        if (recordBucket_.tokens < 1.0) {
            ++droppedRecords_;
            return;
        }
        recordBucket_.tokens -= 1.0;
        // Stamp the cumulative D-2 counters onto every record that survives, so
        // the wire itself reports what is missing from it.
        DebugRecord stamped = record;
        stamped.droppedRecords = droppedRecords_;
        stamped.droppedLines = droppedLines_;
        inner_->emit(stamped);
    }

    /// NEVER throttled (header contract): the one-per-run summary must always land.
    void summarize(const RunSummary& summary) override { inner_->summarize(summary); }

    /// Cumulative counts since construction — the summary's "dropped N rec M ln".
    [[nodiscard]] std::uint32_t droppedRecords() const noexcept { return droppedRecords_; }
    /// Info/Debug/Trace lines dropped since construction, summed over ALL tags including
    /// the shared overflow bucket. Error and Warn are never throttled, so they can never
    /// appear in this number — a non-zero count is always lost detail, never a lost fault.
    [[nodiscard]] std::uint32_t droppedLines() const noexcept { return droppedLines_; }

private:
    struct Bucket {
        double tokens = 0.0;
        double lastRefill = 0.0;
        bool hasRefillTime = false;
    };

    struct Channel {
        char tagBuf[17] = "";
        Bucket bucket{};
        std::uint32_t pendingDropped = 0;
        [[nodiscard]] std::string_view tag() const noexcept { return tagBuf; }
    };

    static constexpr int kMaxChannels = 16;  ///< + 1 shared overflow (header note)

    void refill(Bucket& b, double ratePerSecond) {
        const double now = clock_.now().value();
        if (!b.hasRefillTime) {
            b.hasRefillTime = true;
            b.lastRefill = now;
            b.tokens = ratePerSecond;  // a fresh channel starts with a full budget
            return;
        }
        const double dt = now - b.lastRefill;
        b.lastRefill = now;
        if (dt > 0.0) {  // a contract-breaking backwards clock refills nothing
            b.tokens += dt * ratePerSecond;
            if (b.tokens > ratePerSecond) {
                b.tokens = ratePerSecond;  // capacity = one second's budget
            }
        }
    }

    [[nodiscard]] Channel& channelFor(std::string_view subsystem) noexcept {
        for (int i = 0; i < channelCount_; ++i) {
            if (subsystem == channels_[static_cast<std::size_t>(i)].tag()) {
                return channels_[static_cast<std::size_t>(i)];
            }
        }
        if (channelCount_ < kMaxChannels && subsystem.size() <= 16 && !subsystem.empty()) {
            Channel& slot = channels_[static_cast<std::size_t>(channelCount_)];
            std::memcpy(slot.tagBuf, subsystem.data(), subsystem.size());
            slot.tagBuf[subsystem.size()] = '\0';
            ++channelCount_;
            return slot;
        }
        return overflow_;  // tag #17+ (or an anomalous tag) shares one bucket
    }

    hal::ITelemetrySink* inner_;
    hal::IClock& clock_;
    RateLimitConfig cfg_;
    Bucket recordBucket_{};
    Channel channels_[static_cast<std::size_t>(kMaxChannels)]{};
    Channel overflow_{};
    int channelCount_ = 0;
    std::uint32_t droppedRecords_ = 0;
    std::uint32_t droppedLines_ = 0;
};

}  // namespace shulib::diag
