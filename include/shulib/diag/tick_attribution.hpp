#pragma once
//
// TickAttribution — WHO consumed the loop budget (diagnostics-plan D-3; WS13, C5).
//
// LoopMonitor (A1) detects that a tick blew its budget; it cannot say WHO —
// "localization 4 ms, motion 2 ms, sinks 1 ms" is the question an overrun actually
// raises. This class measures named phases inside one tick on an injected clock
// and keeps the LAST COMPLETED tick's breakdown, so:
//   * the scheduler stamps it into every DebugRecord (the D-3 schema slots), and
//   * an overrun line can NAME the worst consumer instead of shrugging.
//
// ── Which clock, and why it is injected separately ──────────────────────────────────
// Phase timing needs a clock that ADVANCES DURING A TICK. On the robot that is
// real time (R1 wires the microsecond clock — the same IClock the loop uses). In
// HOST SIM the sim clock advances only between ticks (the pacer steps the world),
// so phases would all read 0 — true, harmless, and useless. The attribution clock
// is therefore its own injection: tests drive a deterministic one, sim runs may
// leave attribution off entirely (a null clock in the scheduler config = feature
// off = zero clock calls — the A1 cost contract, structurally).
//
// ── One-tick lag, stated honestly ───────────────────────────────────────────────────
// Records are emitted DURING the Motion phase (by the motion itself), before that
// phase's duration — or the tick's total — is knowable. So what rides a record is
// the breakdown of the most recently COMPLETED tick, uniformly (documented on the
// schema field). For the overrun path this lag is exactly right: the overrun is
// DETECTED at tick N+1 (its dt covers tick N's work), and the last completed
// breakdown at that moment IS tick N — the tick that overran.
//
// Sum contract (pinned by test): attributed phase times never exceed the tick
// total on the same clock; total − attributed = "other" (un-instrumented work +
// pacing), reported as its own quantity rather than smeared into a named phase.
//
// Single-task by contract, like the rest of diag/.

#include <array>
#include <cstddef>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// Measures where one tick's time went, phase by phase, on an INJECTED clock — the "who" that
/// LoopMonitor's "this tick blew its budget" cannot answer on its own. Only the LAST COMPLETED
/// tick's breakdown is kept, so a record stamped mid-tick necessarily carries the previous tick's
/// numbers; for the overrun path that lag is exactly right, because an overrun is detected on the
/// tick AFTER the one that caused it. Needs a clock that advances DURING a tick, which is why it
/// takes its own: the host sim clock only moves between ticks and would report every phase as
/// zero. Single-task by contract, like the rest of diag/.
class TickAttribution {
public:
    /// Per-phase durations for one tick, indexed by TickPhase. Sized by kTickPhaseSlots rather
    /// than by the phases that exist today — the spare slots are what make a new phase an append
    /// to the vocabulary instead of a reshape of the telemetry wire.
    using Phases = std::array<units::Time, static_cast<std::size_t>(kTickPhaseSlots)>;

    /// `clock` must outlive the instance (see header for WHICH clock).
    explicit TickAttribution(hal::IClock& clock) noexcept : clock_{clock} {}

    /// Open a tick: zero the working phases, mark the start instant.
    void beginTick() {
        SHULIB_PRECONDITION(!tickOpen_, "TickAttribution::beginTick: tick already open");
        tickOpen_ = true;
        current_.fill(units::Time{0.0});
        tickStart_ = clock_.now();
    }

    /// Time one phase, RAII-style: the duration is credited when the scope closes.
    ///   { auto scope = att.phase(TickPhase::Localization); localizer.update(); }
    /// Phases may repeat within a tick (durations accumulate); scopes must not
    /// overlap the same phase (the second-open would double-charge the overlap).
    class PhaseScope {
    public:
        /// Stamps the start instant on the attribution clock. Prefer TickAttribution::phase(),
        /// which additionally checks that a tick is actually open; constructing one directly
        /// skips that check and will credit its interval to whatever tick is open when it closes.
        PhaseScope(TickAttribution& att, TickPhase phase) noexcept
            : att_{att}, phase_{phase}, start_{att.clock_.now()} {}
        /// Credits (now − start) to the phase on scope exit, and only then: a scope still alive
        /// when endTick() runs contributes nothing to the tick it was opened in — its interval
        /// lands on whatever tick is open when it finally closes, or is discarded outright if the
        /// next beginTick() zeroes the working phases first. Repeated scopes on the same phase
        /// within one tick ACCUMULATE rather than replace.
        ~PhaseScope() {
            const std::size_t idx = static_cast<std::size_t>(phase_);
            att_.current_[idx] = att_.current_[idx] + (att_.clock_.now() - start_);
        }
        /// Non-copyable, and therefore non-movable: a scope charges exactly one interval, and a
        /// copy would charge it twice. phase() still returns one by value — that is guaranteed
        /// elision, not a move.
        PhaseScope(const PhaseScope&) = delete;
        PhaseScope& operator=(const PhaseScope&) = delete;

    private:
        TickAttribution& att_;
        TickPhase phase_;
        units::Time start_;
    };

    /// Open a scope that charges its own lifetime to `p`. Requires a tick to be open. The result
    /// MUST be bound to a named variable — an unnamed temporary dies at the semicolon and charges
    /// nothing, which is the whole reason this is [[nodiscard]].
    [[nodiscard]] PhaseScope phase(TickPhase p) {
        SHULIB_PRECONDITION(tickOpen_, "TickAttribution::phase: no tick open");
        return PhaseScope{*this, p};
    }

    /// Close the tick: snapshot the working phases + total as the LAST COMPLETED
    /// tick (what records and overrun lines read).
    void endTick() {
        SHULIB_PRECONDITION(tickOpen_, "TickAttribution::endTick: no tick open");
        tickOpen_ = false;
        last_ = current_;
        lastTotal_ = clock_.now() - tickStart_;
        hasCompleted_ = true;
    }

    /// Discard a half-measured tick (an exception unwound through the tick body):
    /// its numbers never completed, so they are dropped rather than reported, and
    /// the instrument re-arms. The last COMPLETED tick's story is untouched.
    void abandonTick() noexcept { tickOpen_ = false; }

    /// False until the first endTick(), and again after reset(). Worth asking first: before any
    /// tick completes every lastX() accessor reads zero, which is indistinguishable from a tick
    /// that genuinely cost nothing.
    [[nodiscard]] bool hasCompletedTick() const noexcept { return hasCompleted_; }
    /// The last completed tick's per-phase durations (zeros before any tick).
    [[nodiscard]] const Phases& lastPhases() const noexcept { return last_; }
    /// Seconds from beginTick() to endTick() of the last completed tick, on the attribution
    /// clock. It spans the whole tick, including work no phase scope wrapped — that remainder is
    /// what lastOther() reports rather than smearing it into a named phase.
    [[nodiscard]] units::Time lastTotal() const noexcept { return lastTotal_; }

    /// Sum of the attributed phases of the last completed tick.
    [[nodiscard]] units::Time lastAttributed() const noexcept {
        double sum = 0.0;
        for (const units::Time& t : last_) {
            sum += t.value();
        }
        return units::Time{sum};
    }

    /// total − attributed: un-instrumented work. Floored at 0 (a clock that jumped
    /// mid-phase can make phases overshoot the total; the floor keeps the report
    /// coherent rather than printing a negative time).
    [[nodiscard]] units::Time lastOther() const noexcept {
        const double other = lastTotal_.value() - lastAttributed().value();
        return units::Time{other > 0.0 ? other : 0.0};
    }

    /// The phase that consumed the most of the last completed tick — the NAME the
    /// overrun line prints. Ties resolve to the lower index (deterministic).
    [[nodiscard]] TickPhase lastWorstPhase() const noexcept {
        std::size_t worst = 0;
        for (std::size_t i = 1; i < last_.size(); ++i) {
            if (last_[i].value() > last_[worst].value()) {
                worst = i;
            }
        }
        return static_cast<TickPhase>(worst);
    }

    /// Forget everything (run boundary). The next tick starts a fresh story.
    void reset() noexcept {
        tickOpen_ = false;
        hasCompleted_ = false;
        current_.fill(units::Time{0.0});
        last_.fill(units::Time{0.0});
        lastTotal_ = units::Time{0.0};
    }

private:
    hal::IClock& clock_;
    Phases current_{};
    Phases last_{};
    units::Time tickStart_{};
    units::Time lastTotal_{};
    bool tickOpen_ = false;
    bool hasCompleted_ = false;
};

/// Short display token per phase for the overrun-attribution line ("loc"/"mot"/…).
[[nodiscard]] constexpr const char* tickPhaseName(TickPhase phase) noexcept {
    switch (phase) {
        case TickPhase::Localization: return "loc";
        case TickPhase::Motion: return "mot";
        case TickPhase::Health: return "hlt";
        case TickPhase::Telemetry: return "tel";
        case TickPhase::Scheduler: return "sch";
        case TickPhase::User: return "usr";
    }
    return "rsv";  // a spare slot (6..7) — reserved, unnamed until assigned
}

}  // namespace shulib::diag
