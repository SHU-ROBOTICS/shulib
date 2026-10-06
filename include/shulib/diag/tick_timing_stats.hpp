#pragma once
//
// TickTimingStats — the statistics of a loop's tick-to-tick dt, as DATA (chunk R5a Part A, 2026-09-29).
//
// ── Why this exists beside LoopMonitor ──────────────────────────────────────────────
// LoopMonitor (loop_monitor.hpp) answers ONE question per tick — did this tick reach its budget? —
// and raises LOOP_OVERRUN through the FaultLatch when it did. What it does not keep is the SHAPE of
// the loop's timing: the mean dt, its spread, the fastest and slowest tick, how many ticks were
// late and how many missed a whole period. The hostile suites certify every controller against an
// INVENTED jitter schedule (A4 register HA-32 loop rate, HA-34 jitter — sim/hostile/composed.hpp's
// JitterSchedule: ±20 %, 2 % spikes at 5×), and nothing on the robot has ever measured what the real
// loop does. shulib Calibrate's GAINS and REST steps (calibration/drive_characterizer.hpp,
// calibration/rest_capture.hpp) record these statistics for every segment and capture, so a robot
// session produces the numbers those two register entries wait for — PARTIALLY: Calibrate's loop
// does less work per tick than a motion loop, so its timing is a floor for a motion loop's, not a
// measurement of it (HA-32/34's settle lines say so).
//
// A SIBLING, not an edit: LoopMonitor is untouched, raises nothing new, and its boundary is the one
// used here — a tick whose dt is AT OR OVER the budget counts (INCLUSIVE; loop_monitor.hpp's header
// gives the deadline-scheduling reason). The first tick after construction or rebaseline() only
// baselines, exactly as LoopMonitor's does: there is no previous tick to difference against, so it
// is not a dt of 0 and never enters the statistics.
//
// ── Numerics ────────────────────────────────────────────────────────────────────────
// Mean and spread by Welford's online update (mean and the sum of squared deviations about the
// running mean) — one pass, no stored samples, and no Σx² − (Σx)²/n cancellation: at a 10 ms dt the
// naive form subtracts two numbers ~10⁴ times larger than the variance it wants. The test pins the
// mean and σ against a two-pass reference to 1e-12 over a jittered run, and min, max and every count
// EXACTLY.
//
// PURE and allocation-free: times in, numbers out; it reads no clock (the caller passes `now`, which
// is how the characterizer shares one clock read per tick between everything it measures). Raises
// nothing. Single-task by contract, like the rest of diag/.

#include <cmath>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// What TickTimingStats measured, as a fixed-width value (the characterization record persists it
/// field by field). Every time is in SECONDS. All zero until two ticks have been seen.
struct TickTimingSummary {
    /// How many dt values were measured — the ticks AFTER the baseline tick.
    std::uint32_t count = 0;
    /// How many of them were AT OR OVER the budget (INCLUSIVE — LoopMonitor's boundary).
    std::uint32_t overBudget = 0;
    /// How many were STRICTLY over twice the nominal period — a tick that missed at least one whole
    /// period. Exactly 2× is not counted: that is one missed period to the last bit, and "over"
    /// keeps this count from depending on the last bit of a clock read.
    std::uint32_t overDouble = 0;
    /// The mean dt.
    double meanS = 0.0;
    /// The SAMPLE standard deviation of dt (divisor count − 1); 0 with fewer than two dt values.
    double sigmaS = 0.0;
    /// The shortest dt.
    double minS = 0.0;
    /// The longest dt.
    double maxS = 0.0;

    /// True when the summary holds together: every time finite, the spread non-negative, neither
    /// count past `count`, min <= max when anything was measured, and all zero when nothing was.
    /// The characterization record's decoder refuses a summary that fails this.
    [[nodiscard]] bool valid() const noexcept {
        if (!std::isfinite(meanS) || !std::isfinite(sigmaS) || !std::isfinite(minS)
            || !std::isfinite(maxS) || sigmaS < 0.0 || overBudget > count || overDouble > count) {
            return false;
        }
        if (count == 0) {
            return meanS == 0.0 && sigmaS == 0.0 && minS == 0.0 && maxS == 0.0;
        }
        return minS <= maxS;
    }
};

/// The two periods TickTimingStats judges a dt against. Copied at construction.
struct TickTimingConfig {
    /// The loop's intended period. 10 ms — the tick every shulib loop is paced at (A4 register
    /// HA-32). Must be > 0.
    units::Time nominal{0.01};
    /// The dt at which a tick counts as late, INCLUSIVE. 15 ms — LoopMonitorConfig's default, so the
    /// two tools call the same tick late. Must be strictly greater than `nominal`, for LoopMonitor's
    /// reason: a budget equal to the period counts every healthy tick.
    units::Time budget{0.015};
};

/// The online statistics of a loop's dt (header). Construct once per measured stretch — a segment,
/// a capture — or rebaseline() across a deliberate pause.
class TickTimingStats {
public:
    /// Copies `config`. Precondition: nominal finite and > 0, budget finite and > nominal.
    explicit TickTimingStats(const TickTimingConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.nominal.value()) && cfg_.nominal.value() > 0.0,
                            "TickTimingStats: nominal must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(cfg_.budget.value())
                                && cfg_.budget.value() > cfg_.nominal.value(),
                            "TickTimingStats: budget must be finite and > nominal");
    }

    /// Record one tick at time `now` and return its dt — or 0 on a baseline tick (the first after
    /// construction or rebaseline()), which enters no statistic. Precondition: `now` finite.
    units::Time tick(units::Time now) {
        SHULIB_PRECONDITION(std::isfinite(now.value()), "TickTimingStats::tick: now must be finite");
        if (!hasLast_) {
            hasLast_ = true;
            last_ = now.value();
            return units::Time{0.0};
        }
        const double dt = now.value() - last_;
        last_ = now.value();
        ++count_;
        // Welford: the mean moves by the deviation over n; M2 gains the product of the deviations
        // from the old and the new mean — exact in exact arithmetic, and free of cancellation.
        const double delta = dt - mean_;
        mean_ += delta / static_cast<double>(count_);
        m2_ += delta * (dt - mean_);
        if (count_ == 1 || dt < min_) {
            min_ = dt;
        }
        if (count_ == 1 || dt > max_) {
            max_ = dt;
        }
        if (dt >= cfg_.budget.value()) {
            ++overBudget_;  // INCLUSIVE — LoopMonitor's boundary (header)
        }
        if (dt > 2.0 * cfg_.nominal.value()) {
            ++overDouble_;
        }
        return units::Time{dt};
    }

    /// The next tick() only baselines, so a DELIBERATE gap (a pause between two captures) is not
    /// measured as a tick. Keeps every statistic — LoopMonitor::reset()'s semantics.
    void rebaseline() noexcept { hasLast_ = false; }

    /// Forget everything: the statistics and the baseline.
    void reset() noexcept { *this = TickTimingStats{cfg_}; }

    /// The statistics so far (TickTimingSummary's fields).
    [[nodiscard]] TickTimingSummary summary() const noexcept {
        TickTimingSummary s;
        s.count = count_;
        s.overBudget = overBudget_;
        s.overDouble = overDouble_;
        if (count_ > 0) {
            s.meanS = mean_;
            s.minS = min_;
            s.maxS = max_;
        }
        s.sigmaS = count_ >= 2 ? std::sqrt(m2_ / static_cast<double>(count_ - 1)) : 0.0;
        return s;
    }

    /// The config in force.
    [[nodiscard]] const TickTimingConfig& config() const noexcept { return cfg_; }

private:
    TickTimingConfig cfg_;
    bool hasLast_ = false;
    double last_ = 0.0;
    std::uint32_t count_ = 0;
    std::uint32_t overBudget_ = 0;
    std::uint32_t overDouble_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;
    double min_ = 0.0;
    double max_ = 0.0;
};

}  // namespace shulib::diag
