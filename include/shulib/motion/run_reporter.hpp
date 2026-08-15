#pragma once
//
// RunReporter — the glue that makes a run LEGIBLE end to end (WS13, chunk C5):
// session header (§18.5) → per-motion result lines (§18.3/§18.4) → run summary
// (§18.3). This is the class that closes M2's "the run is legible in real time
// on the terminal" clause.
//
//     TermSink term{clock, out};
//     Chassis chassis{deps, pacer};
//     RunReporter report{term, chassis.scheduler()};
//     report.sessionStart({.buildHash = diag::compiledBuildHash(),
//                          .routineId = "redLeftTall", .alliance = "red",
//                          .side = "left", .portMap = "L1,2,3 R4,5,6 IMU10"});
//     …the auton…
//     report.finishRun();
//
// ── Why it lives in motion/, not diag/ ──────────────────────────────────────────────
// A1 predicted C5's result/summary code would land in diag/. The dependency
// reality discovered here: the result line's DATA SOURCE is CompletedMotion and
// the scheduler's counters — motion-layer types diag/ must never name (diag/ is
// a dependency LEAF; debug_record.hpp's rule). So the split is: diag/ owns the
// VOCABULARY and FORMATTERS (MotionResult, emitResultLine, SessionInfo,
// emitSessionHeader, RunSummary — all diag-level, all golden-testable with hand
// data), and this class is the thin GLUE that feeds them from the scheduler.
// Formatting stays with the schema it formats; glue sits with the data it reads.
//
// ── Boundary results are STRUCTURAL, not remembered ─────────────────────────────────
// Construction ATTACHES the reporter as the scheduler's boundary observer, so
// every motion boundary — settle, timeout, cancel, fault abort, pre-empt — emits
// its result line with no per-verb calls to forget (the A1 emitRecord lesson;
// destruction detaches, if still attached). §18.4's boundary vocabulary is
// derived here: Cancelled + causal fault ⇒ FAULT_ABORT; Cancelled + preempted ⇒
// SUPERSEDED; bare Cancelled stays CANCELLED.
//
// ── The sink argument: give it the UNTHROTTLED head ─────────────────────────────────
// Header, result lines, and summary are LOW-RATE run landmarks — a handful per
// run. Wire the reporter directly to the formatter (TermSink), and put D-2's
// RateLimitedSink on the HIGH-RATE path (the deps' record/tick stream) instead:
// a result line eaten by a token bucket that per-tick chatter drained would be
// a landmark lost to noise control. (Info-level result lines through a shared
// throttled head DO get counted if dropped — nothing is ever silent — but the
// recommended wiring never puts them there.)
//
// ── What the summary reads, and one-run scope ───────────────────────────────────────
// Counters/latch/health/battery are read LIVE at finishRun() from the scheduler
// and its deps (battery END is a reading, not a memory). Scheduler counters are
// lifetime-cumulative and FaultLatch clears only at explicit run boundaries, so:
// ONE reporter + ONE scheduler per run — the normal auton shape. gatingRejects
// counts GPS_GATE_REJECT raises (HealthMonitor raises once per EPISODE, so this
// is episodes, not raw rejected fixes — honest label, E2 refines it).
//
// Single-task by contract, like everything it composes. Never throws into the
// scheduler (IMotionObserver contract): it only formats and logs.

#include <string_view>

#include "shulib/diag/motion_result.hpp"
#include "shulib/diag/rate_limit_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/triage.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace shulib::motion {

/// The glue that makes one run legible end to end: a session header first, a result line at every
/// motion boundary, a summary at the end. It formats nothing itself — diag/ owns the vocabulary
/// and the formatters — and it remembers almost nothing: apart from the provenance strings and
/// the starting battery voltage, everything the summary reports is read LIVE off the scheduler
/// and its deps at finishRun().
///
/// Result lines are STRUCTURAL rather than remembered: construction attaches the reporter as the
/// scheduler's boundary observer and destruction detaches it, so settle, timeout, cancel, fault
/// abort and pre-empt each emit their line with no per-verb call a routine could forget.
///
/// ONE reporter and ONE scheduler per run — the ordinary auton shape. The scheduler's counters
/// are lifetime-cumulative and the fault latch clears only at explicit run boundaries, so driving
/// a second run through the same pair reports the first run's totals over again. Single-task by
/// contract, and it never throws into the scheduler: an observer that threw would abort the very
/// motion it exists to describe.
class RunReporter final : public IMotionObserver {
public:
    /// `out` is where the report goes (see header: the UNTHROTTLED head);
    /// `sched` is the run's scheduler — the reporter self-attaches as its
    /// boundary observer. `limiter`, when given, contributes the D-2 drop
    /// totals to the summary (nullptr = no limiter in the chain = zeros);
    /// `blackbox`, when given, contributes the E1 blackbox's own drop count so
    /// a file with gaps in it says so on the terminal too (nullptr = no
    /// blackbox = the summary stays silent about one, rather than claiming a
    /// healthy zero for something that never ran). All must outlive the reporter.
    RunReporter(hal::ITelemetrySink& out, MotionScheduler& sched,
                const diag::RateLimitedSink* limiter = nullptr,
                const diag::SdSink* blackbox = nullptr) noexcept
        : out_{&out}, sched_{&sched}, limiter_{limiter}, blackbox_{blackbox} {
        sched_->setBoundaryObserver(this);
    }

    /// Detaches from the scheduler, but only while the scheduler still points at THIS reporter:
    /// if something else took the observer slot in the meantime, that one is left attached rather
    /// than silently unhooked. The scheduler must outlive the reporter: this destructor reads it,
    /// so tearing the scheduler down first is a use-after-free rather than a quiet no-op.
    ~RunReporter() override {
        if (sched_->boundaryObserver() == this) {
            sched_->setBoundaryObserver(nullptr);
        }
    }

    /// Neither copyable nor movable: the scheduler holds a raw back-pointer to this exact object,
    /// installed by the constructor and by nothing else. A copy would therefore never register —
    /// the one observer slot would still hold the ORIGINAL, and the copy would be a silent second
    /// reporter that emits a header and a summary but never a single result line (its destructor's
    /// identity check correctly declines to unhook the original on the way out). A move is worse:
    /// the members are raw pointers, so the scheduler would be left aimed at the husk that was
    /// moved out of. Construct it where it will live.
    RunReporter(const RunReporter&) = delete;
    RunReporter(RunReporter&&) = delete;
    RunReporter& operator=(const RunReporter&) = delete;
    RunReporter& operator=(RunReporter&&) = delete;

    /// Emit the §18.5 session header — call FIRST, before any motion, so
    /// provenance is the first thing in every log (§18.5: "first record of
    /// every run"). Battery start is READ here (a live value, not caller
    /// homework) and remembered for the summary's start→end pair; the hash and
    /// routine id are re-copied into bounded storage for the summary (the
    /// caller's string_views are not retained).
    void sessionStart(const diag::SessionInfo& info) {
        batteryStart_ = sched_->deps().ctx->battery().voltage();
        summarySeed_.setBuildHash(info.buildHash);
        summarySeed_.setRoutineId(info.routineId);
        diag::emitSessionHeader(*out_, info, batteryStart_);
    }

    /// The scheduler's boundary callback: one §18.3 result line per finished
    /// motion, translated to §18.4's boundary vocabulary (header note).
    void onMotionComplete(const CompletedMotion& completed) override {
        diag::MotionResult r;
        r.id = completed.id;
        r.name = completed.name;
        r.outcome = outcomeOf(completed);
        r.abortFault = completed.abortFault;
        r.duration = completed.endTime - completed.startTime;
        r.hasPathData = completed.hasPathData;
        r.finalPose = completed.finalPose;
        r.overshoot = completed.overshoot;
        r.drift = completed.drift;
        diag::emitResultLine(*out_, r);
    }

    /// Assemble the §18.3 run summary from live state and hand it to the sink's
    /// summarize() channel (TermSink renders the block). Call once, at run end.
    void finishRun() {
        diag::RunSummary s = summarySeed_;  // carries buildHash/routineId copies
        s.motionsStarted = sched_->motionsStarted();
        s.motionsSettled = sched_->motionsSettled();
        s.motionsTimedOut = sched_->motionsTimedOut();
        s.motionsCancelled = sched_->motionsCancelled();
        s.motionsAborted = sched_->motionsAborted();
        s.hasHeadingData = sched_->runHasHeadingData();
        s.headingMax = sched_->runMaxHeadingDrift();
        s.headingFinal = sched_->runFinalHeadingDrift();
        const diag::FaultLatch& faults = *sched_->deps().faults;
        s.gatingRejects = faults.raiseCount(diag::FaultCode::GpsGateReject);
        s.brownout = sched_->deps().health->brownedOut();
        s.worstLoopDt = sched_->loopMonitor().worstDt();
        s.firstFault = faults.firstFault();
        s.firstFaultTime = faults.firstFaultTime();
        if (limiter_ != nullptr) {
            s.droppedRecords = limiter_->droppedRecords();
            s.droppedLines = limiter_->droppedLines();
        }
        if (blackbox_ != nullptr) {
            s.blackboxDropped = blackbox_->droppedFrames();
        }
        s.batteryStart = batteryStart_;
        s.batteryEnd = sched_->deps().ctx->battery().voltage();
        out_->summarize(s);
        // D-7's post-run auto-triage: when the blackbox actually dumped, the run has a
        // "why did it break" story as well as a "how did it go" one, and it prints
        // LAST — the final thing on the screen is the root cause. Structural, like the
        // result lines: a routine cannot forget to ask for it. Nothing is printed when
        // no fault fired, so a clean run gains no noise.
        if (blackbox_ != nullptr && blackbox_->dumped()) {
            diag::emitTriageBlock(*out_, blackbox_->triage(), blackbox_->triageTick());
        }
    }

private:
    /// §18.4's boundary vocabulary from the C2/C5 boundary record (header note).
    [[nodiscard]] static diag::MotionOutcome outcomeOf(const CompletedMotion& c) noexcept {
        switch (c.exit) {
            case control::ExitReason::Settled: return diag::MotionOutcome::Settled;
            case control::ExitReason::TimedOut: return diag::MotionOutcome::TimedOut;
            case control::ExitReason::Cancelled:
                if (c.abortFault != diag::FaultCode::None) {
                    return diag::MotionOutcome::FaultAbort;
                }
                return c.preempted ? diag::MotionOutcome::Superseded
                                   : diag::MotionOutcome::Cancelled;
            case control::ExitReason::Running: break;  // boundaries are exits only
        }
        return diag::MotionOutcome::Cancelled;  // unreachable; safe rendering
    }

    hal::ITelemetrySink* out_;
    MotionScheduler* sched_;
    const diag::RateLimitedSink* limiter_;
    const diag::SdSink* blackbox_;
    units::Voltage batteryStart_{};
    diag::RunSummary summarySeed_{};  ///< provenance copies (bounded, no views)
};

}  // namespace shulib::motion
