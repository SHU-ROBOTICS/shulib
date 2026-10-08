#pragma once
//
// LoopMonitor — loop-overrun / tick-timing detection (master plan §18.4; WS13, chunk A1).
//
// Why this exists: a blown control tick silently corrupts every dt-dependent computation
// downstream — PID derivative/integral, profile sampling, the odometry twist — which is
// exactly how the promised < 1° heading quietly degrades. The monitor measures the real
// dt between consecutive tick() calls on the injected clock and raises LOOP_OVERRUN
// through the FaultLatch when a tick reaches its budget, so a timing pathology becomes a
// visible fault instead of a mystery drift.
//
// BOUNDARY SEMANTICS (pinned by test, mutation-checked): a tick whose dt satisfies
//
//     dt >= budget          — INCLUSIVE at the budget —
//
// is an overrun. The budget is a hard deadline in the deadline-scheduling sense: a loop
// that consumes its entire budget has zero margin left and the next tick already starts
// late, so "exactly at budget" is a miss, not a pass. Consequence for configuration: the
// budget must be strictly GREATER than the nominal tick period (a healthy 10ms loop has
// dt == 10ms every tick; a 10ms budget would fault permanently — use e.g. 15ms).
//
// The first tick() after construction or reset() only baselines the clock (there is no
// previous tick to difference against) — it can never fault, returns dt = 0, and does
// not count toward worstDt(). reset() exists for deliberate pauses (e.g. between runs)
// so a legitimate gap is not reported as an overrun.
//
// ONE LINE PER EPISODE (chunk T4 Part 3, 2026-10-08; the T4-3 review's finding): EVERY tick at or
// past the budget still raises LOOP_OVERRUN — overrunCount(), the latch's faultCount() and
// raiseCount(), and each tick's fault stamp count every one, as they always did — but only the
// FIRST tick of an EPISODE logs its line (FaultLatch::raiseRepeat for the rest). An episode is a
// run of overrunning ticks: it ends at the first tick under the budget, and at reset(), so the
// next overrun opens a new one and is said. WHY: a loop that stays slow used to log one Error
// line per tick — 50 a second at a 20 ms tick — and the programs' SD copy of the log
// (diag/log_copy_sink.hpp) turned each into one card write and one flush INSIDE the slow tick,
// the cost that keeps a loop slow. The per-tick detail is not lost: every DebugRecord carries
// its tick's dt (the blackbox and the serial records keep it), and worstDt() the worst. NOT
// bounded further, stated: an INTERMITTENT overrun — one slow tick between healthy ones — is a
// new episode each time, so a loop slow every few ticks still logs a line every few ticks (as
// every overrun did before this rule). Merging near episodes (a hysteresis on the healthy ticks)
// was not done: nothing measured says it is needed.
//
// Single-task by contract, like the rest of diag/ (see fault.hpp's concurrency note).

#include <algorithm>
#include <cstdio>

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// LoopMonitor's one tuning knob, taken BY VALUE at construction — editing the struct afterwards
/// has no effect on a live monitor. The 15 ms default leaves 5 ms of margin on the nominal 10 ms
/// control loop; see `budget` for why it must not simply equal the tick period.
struct LoopMonitorConfig {
    /// The dt at which a tick counts as an overrun (INCLUSIVE — see header). Must be > 0
    /// and strictly greater than the nominal tick period.
    units::Time budget{0.015};
};

/// Loop-overrun detection: it measures the real dt between consecutive tick() calls on the
/// INJECTED clock and raises FaultCode::LoopOverrun through the latch when a tick reaches its
/// budget — every such tick counted, its line logged once per episode (header). It exists
/// because a blown control tick silently corrupts every dt-dependent computation downstream —
/// PID derivative and integral, profile sampling, the odometry twist — which is how a promised
/// sub-degree heading quietly decays into drift nobody can explain.
/// Single-task by contract, like the rest of the diagnostics layer.
class LoopMonitor {
public:
    /// `clock` and `faults` are held BY REFERENCE and must outlive the monitor; `config` is
    /// copied. `config.budget` must be > 0 (precondition) and, to be usable at all, strictly
    /// greater than the nominal tick period — a tick exactly AT the budget is an overrun, so a
    /// 10 ms budget on a 10 ms loop faults on every tick.
    LoopMonitor(hal::IClock& clock, FaultLatch& faults, const LoopMonitorConfig& config = {})
        : clock_{clock}, faults_{faults}, config_{config} {
        SHULIB_PRECONDITION(config.budget.value() > 0.0, "LoopMonitor: budget must be > 0");
    }

    /// Call exactly once per loop iteration. Returns this tick's measured dt (0 on the
    /// baseline tick). Raises LOOP_OVERRUN via the latch when dt >= budget — on every such
    /// tick, with its line on the first of an episode only (header: ONE LINE PER EPISODE).
    units::Time tick() {
        const units::Time now = clock_.now();
        if (!hasLast_) {
            hasLast_ = true;
            lastNow_ = now;
            return units::Time{0.0};
        }
        const units::Time dt = now - lastNow_;
        lastNow_ = now;
        worstDt_ = std::max(worstDt_, dt);
        if (dt.value() >= config_.budget.value()) {
            ++overrunCount_;
            char buf[64];
            std::snprintf(buf, sizeof buf, "dt=%.4f budget=%.4f", dt.value(),
                          config_.budget.value());
            if (inEpisode_) {
                faults_.raiseRepeat(FaultCode::LoopOverrun, "DIAG", buf);
            } else {
                inEpisode_ = true;
                ++episodeCount_;
                faults_.raise(FaultCode::LoopOverrun, "DIAG", buf);
            }
        } else {
            inEpisode_ = false;  // a tick under the budget ends the episode
        }
        return dt;
    }

    /// Largest dt observed since construction (the §18.3 "worst loop dt" summary
    /// quantity, consumed at C5). Time{0} until two ticks have happened.
    [[nodiscard]] units::Time worstDt() const noexcept { return worstDt_; }
    /// How many ticks have reached the budget since construction. It counts TICKS, not
    /// episodes — a loop that stays slow increments (and raises) once per tick, though it logs
    /// once per episode — and reset() does not clear it, so this is a whole-run total. Baseline
    /// ticks never count.
    [[nodiscard]] int overrunCount() const noexcept { return overrunCount_; }

    /// How many overrun EPISODES have begun since construction — each one logged ONE line (the
    /// header). A whole-run total, like overrunCount(); always <= it.
    [[nodiscard]] int overrunEpisodes() const noexcept { return episodeCount_; }

    /// Re-baseline after a DELIBERATE gap (run boundary, pause): the next tick() only
    /// baselines, so the gap is not misreported as an overrun, and an episode in progress ends
    /// (the next overrun opens a new one, with its line). Keeps worstDt/counts.
    void reset() noexcept {
        hasLast_ = false;
        inEpisode_ = false;
    }

private:
    hal::IClock& clock_;
    FaultLatch& faults_;
    LoopMonitorConfig config_;
    units::Time lastNow_{0.0};
    units::Time worstDt_{0.0};
    int overrunCount_ = 0;
    int episodeCount_ = 0;
    bool hasLast_ = false;
    bool inEpisode_ = false;
};

}  // namespace shulib::diag
