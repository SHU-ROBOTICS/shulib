#pragma once
//
// LevelFilterSink — per-subsystem log levels (diagnostics-plan D-1; WS13, chunk C5).
//
// The single most-used debugging move in practice: turn [MOT] up to DEBUG while
// holding [LOC] at WARN. Before this decorator, levels were global — chasing one
// subsystem meant either drowning in every other subsystem's chatter or a REBUILD
// with different compile-time levels. This is a RUNTIME dial on the sink chain:
//
//     TermSink term{clock, out};
//     LevelFilterSink filtered{term};
//     filtered.setLevel("LOC", LogLevel::Warn);   // quiet the localizer…
//     filtered.setLevel("MOT", LogLevel::Debug);  // …while watching the motion layer
//
// Semantics (each pinned by test/level_filter_test.cpp):
//   * A line passes iff its level is AT OR ABOVE the channel's threshold in
//     severity (Error > Warn > Info > Debug > Trace). setLevel(tag, Warn) means
//     "from tag, Warn and Error only".
//   * The threshold is the TAG's override if one is set, else the global level
//     (default Trace = everything passes — the decorator is transparent until told
//     otherwise).
//   * Filtering affects ONLY the leveled-message channel: records, summaries, and
//     wantsRecord() forward untouched (they are data, not chatter — and the record
//     stream has its own dial, RateLimitedSink).
//
// FILTERING IS NOT DROPPING (the D-2 distinction, stated deliberately): a filtered
// line is one the operator ASKED not to see — explicit configuration, not silent
// degradation — so it is not counted in any dropped tally. Throttling (D-2) is
// involuntary and therefore counted. Conflating the two would bury real drops in
// requested quiet.
//
// Fixed capacity, no heap: overrides live in a bounded table; exceeding it is a
// LOUD precondition, never a silently ignored setLevel. Single-task by contract,
// like the rest of diag/.

#include <cstring>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::diag {

class LevelFilterSink final : public hal::ITelemetrySink {
public:
    /// `inner` must outlive the filter. Transparent until configured (global
    /// default Trace: everything passes).
    explicit LevelFilterSink(hal::ITelemetrySink& inner) noexcept : inner_{&inner} {}

    /// Threshold for every tag WITHOUT an override. Trace = pass everything.
    void setGlobalLevel(hal::LogLevel level) noexcept { global_ = level; }

    /// Per-subsystem override. Re-setting an existing tag updates it in place.
    /// Precondition: tag non-empty, ≤ kMaxTagBytes; table not full (LOUD, never a
    /// silently ignored dial — kMaxOverrides is far past any real tag census).
    void setLevel(std::string_view subsystem, hal::LogLevel level) {
        SHULIB_PRECONDITION(!subsystem.empty() && subsystem.size() <= kMaxTagBytes,
                            "LevelFilterSink::setLevel: tag must be 1..16 bytes");
        for (int i = 0; i < count_; ++i) {
            if (subsystem == overrides_[static_cast<std::size_t>(i)].tag()) {
                overrides_[static_cast<std::size_t>(i)].level = level;
                return;
            }
        }
        SHULIB_PRECONDITION(count_ < kMaxOverrides,
                            "LevelFilterSink::setLevel: override table full");
        Override& slot = overrides_[static_cast<std::size_t>(count_)];
        std::memcpy(slot.tagBuf, subsystem.data(), subsystem.size());
        slot.tagBuf[subsystem.size()] = '\0';
        slot.level = level;
        ++count_;
    }

    /// Drop every override (the global level stays).
    void clearLevels() noexcept { count_ = 0; }

    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        if (passes(level, subsystem)) {
            inner_->log(level, subsystem, message);
        }
    }

    // Records/summaries are data, not chatter (header note): forward untouched,
    // wantsRecord as a PAIR with emit per the seam contract.
    [[nodiscard]] bool wantsRecord() const noexcept override { return inner_->wantsRecord(); }
    void emit(const DebugRecord& record) override { inner_->emit(record); }
    void summarize(const RunSummary& summary) override { inner_->summarize(summary); }

private:
    static constexpr std::size_t kMaxTagBytes = 16;  ///< TermSink's own tag bound
    static constexpr int kMaxOverrides = 16;

    struct Override {
        char tagBuf[kMaxTagBytes + 1] = "";
        hal::LogLevel level = hal::LogLevel::Trace;
        [[nodiscard]] std::string_view tag() const noexcept { return tagBuf; }
    };

    [[nodiscard]] bool passes(hal::LogLevel level, std::string_view subsystem) const noexcept {
        hal::LogLevel threshold = global_;
        for (int i = 0; i < count_; ++i) {
            if (subsystem == overrides_[static_cast<std::size_t>(i)].tag()) {
                threshold = overrides_[static_cast<std::size_t>(i)].level;
                break;
            }
        }
        // LogLevel is ordered high→low severity (Error=0 … Trace=4): pass iff at
        // least as severe as the threshold.
        return static_cast<int>(level) <= static_cast<int>(threshold);
    }

    hal::ITelemetrySink* inner_;
    hal::LogLevel global_ = hal::LogLevel::Trace;
    Override overrides_[static_cast<std::size_t>(kMaxOverrides)]{};
    int count_ = 0;
};

}  // namespace shulib::diag
