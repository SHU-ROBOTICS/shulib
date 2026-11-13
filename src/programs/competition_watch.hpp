#pragma once
//
// competition_watch.hpp -- "shulib Competition"'s WATCH (chunk COMP-b, 2026-09-30): the pacer between
// the run guard and the brain's tick pacer that ENDS a live auton the moment VEXos DISABLES the robot
// or the match LEAVES AUTONOMOUS -- through the run guard's abandon latch (sequence/run_guard.hpp,
// ABANDON), on the program's own auton task, so the routine unwinds on its own stack and nothing is
// left dangling (the team lead's ruling of 2026-09-24, "our own auton task" -- the COMP brief's C5).
//
// THE LAYERING (the COMP brief's B7):
//     Chassis --pace()--> RunGuard --inner--> CompetitionWatch --inner--> the provider's 10 ms pacer
// The guard checks its deadlines and its abandon latch FIRST, then calls this watch, which reads
// VEXos's competition status and -- once a cause is latched and a run is live -- calls
// guard.requestAbandon() (one atomic store: no re-entry), then lets the world advance. The guard
// cuts the motion at the NEXT pace, before the world advances: at most one more commanded tick
// (10 ms) after the watch saw the change (B22's one-tick latch). Its reads are the status byte only.
//
// THE COMPETITION CAUSES ONLY. The Auton Tester's watch aborts on any controller button or touch;
// in a match that would let a driver's bumped button end the auton, so this program must never use
// that class (the COMP-t brief's §4) -- it shares the layering and the latch-every-tick rule, and
// the boot's chain proof (src/common/guarded_run.hpp), never the causes. Two causes, in order:
//   * DISABLED        -- the status's DISABLED bit (a field, a switch or a Timed Run disabling the
//                        robot mid-auton);
//   * LEFT AUTONOMOUS -- the status's AUTONOMOUS bit clear (the period ended, or the match moved to
//                        driver control without a disable in between).
// The first cause seen is LATCHED until the next arm(); a second one never replaces it.
//
// WHY IT ASKS EVERY TICK, NOT ONCE (B30, from COMP-t): a request made while no run is live is DROPPED
// when the next run arms (run_guard.hpp's ABANDON banner). The watch is armed just before runAuton
// arms the guard, so a change seen in that gap must still end the run -- it requests on its LATCHED
// cause, every pace while the guard is running.
//
// AND ONE READ BEFORE THE RUN (COMP-b's review, 2026-10-01): the watch reads only inside pace(), and
// runAuton's live-wait returns WITHOUT a pace when the estimate is already live -- so a request
// served after its period had ENDED re-seated the pose and commanded one full tick before the
// watch's first read (0.6 in on the host test plant, the motors live under driver control).
// latchNow() reads the status once before anything moves, and the request is not run when the
// period is already over (competition_run.hpp's TOO LATE).
//
// PROS-free: the program hands it one PROS read (competition_get_status(), one line, pinned by
// test/competition_source_test.cpp); the host suite scripts the status byte
// (test/competition_watch_test.cpp). NOTHING HERE HAS RUN ON A ROBOT (worksheet Station K).

#include <atomic>
#include <cstdint>

#include "common/state_screens.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sequence/run_guard.hpp"

namespace shulib::bench {

/// Why the watch ended a run: the FIRST competition cause it saw, latched until re-armed.
enum class CompetitionCause : std::uint8_t {
    None = 0,            ///< nothing has ended the run
    Disabled = 1,        ///< VEXos reported the robot DISABLED
    LeftAutonomous = 2,  ///< the status's AUTONOMOUS bit went clear: the period is over
};

/// The cause as words for the log (at most 27 characters).
[[nodiscard]] constexpr const char* competitionCauseWord(CompetitionCause c) noexcept {
    switch (c) {
        case CompetitionCause::None: return "none";
        case CompetitionCause::Disabled: return "the robot was disabled";
        case CompetitionCause::LeftAutonomous: return "the match left autonomous";
    }
    return "unknown";
}

/// The cause in at most 15 characters, for the brain's SMALL lines.
[[nodiscard]] constexpr const char* competitionCauseShort(CompetitionCause c) noexcept {
    switch (c) {
        case CompetitionCause::None: return "none";
        case CompetitionCause::Disabled: return "disabled";
        case CompetitionCause::LeftAutonomous: return "left autonomous";
    }
    return "unknown";
}

/// The decision for ONE status read (the file banner's causes, in order). PURE.
[[nodiscard]] constexpr CompetitionCause competitionCauseOf(std::uint8_t status) noexcept {
    if ((status & kStatusDisabled) != 0) {
        return CompetitionCause::Disabled;
    }
    if ((status & kStatusAutonomous) == 0) {
        return CompetitionCause::LeftAutonomous;
    }
    return CompetitionCause::None;
}

/// Where the watch reads the status: competition_get_status() on the robot, a script on the host.
class ICompetitionStatus {
public:
    virtual ~ICompetitionStatus() = default;
    ICompetitionStatus() = default;
    ICompetitionStatus(const ICompetitionStatus&) = default;
    ICompetitionStatus(ICompetitionStatus&&) = default;
    ICompetitionStatus& operator=(const ICompetitionStatus&) = default;
    ICompetitionStatus& operator=(ICompetitionStatus&&) = default;
    /// This tick's status byte (the kStatus* bits of src/common/state_screens.hpp).
    [[nodiscard]] virtual std::uint8_t read() = 0;
};

/// THE WATCH (file banner): a pacer between the RunGuard and the real pacer. Disarmed, it only
/// paces (and counts). Armed, it latches the first competition cause it reads and, while the guard
/// is running a run, requests the abandon every tick. arm()/disarm()/pace() belong to the ONE task
/// that runs the Chassis (the program's auton task); cause(), armed() and statusAtLatch() may be
/// read from any task.
class CompetitionWatch final : public motion::ITickPacer {
public:
    /// `inner` is the real pacer, `status` the reader; both must outlive the watch.
    CompetitionWatch(motion::ITickPacer& inner, ICompetitionStatus& status) noexcept
        : inner_{&inner}, status_{&status} {}

    /// Pinned: the guard holds it by reference as its inner pacer.
    CompetitionWatch(const CompetitionWatch&) = delete;
    CompetitionWatch& operator=(const CompetitionWatch&) = delete;
    CompetitionWatch(CompetitionWatch&&) = delete;
    CompetitionWatch& operator=(CompetitionWatch&&) = delete;
    ~CompetitionWatch() override = default;

    /// The guard to abandon (B7: given after the guard is emplaced).
    void attachGuard(sequence::RunGuard& guard) noexcept { guard_ = &guard; }

    /// True when attachGuard() was given `guard` -- the chain's one link back, which pacing alone
    /// cannot show (a watch never attached latches and asks nobody). The boot's chain proof
    /// requires it (pacerChainProven, src/common/guarded_run.hpp).
    [[nodiscard]] bool attachedTo(const sequence::RunGuard& guard) const noexcept {
        return guard_ == &guard;
    }

    /// Start watching: clear the latch, and arm. The first read is the next pace's.
    void arm() noexcept {
        statusAtLatch_.store(0);
        cause_.store(CompetitionCause::None);
        armed_.store(true);
    }

    /// Read the status ONCE, now, armed or not, and latch what it says -- an older latch cleared
    /// first, so the cause() and statusAtLatch() read after it describe THIS read. Returns the
    /// cause (None: autonomous is still on). The request's check before anything moves
    /// (competition_run.hpp): a request the auton task serves after its period ended -- the
    /// status already DISABLED, or already out of AUTONOMOUS -- is never run. Not armed by it:
    /// arm() clears the latch again for the run. The auton task's only (like pace()).
    CompetitionCause latchNow() {
        statusAtLatch_.store(0);
        cause_.store(CompetitionCause::None);
        const std::uint8_t s = status_->read();
        const CompetitionCause c = competitionCauseOf(s);
        if (c != CompetitionCause::None) {
            statusAtLatch_.store(s);
            cause_.store(c);
        }
        return c;
    }

    /// Stop watching. The latched cause stays readable until the next arm().
    void disarm() noexcept { armed_.store(false); }

    /// The latched cause (None until something ends the run). Any task.
    [[nodiscard]] CompetitionCause cause() const noexcept { return cause_.load(); }
    /// The status byte the cause latched on (0 while None). Any task.
    [[nodiscard]] std::uint8_t statusAtLatch() const noexcept { return statusAtLatch_.load(); }
    /// True while armed. Any task.
    [[nodiscard]] bool armed() const noexcept { return armed_.load(); }
    /// Every pace() since construction, armed or not (the pacing task only): the boot's chain
    /// proof reads it (pacerChainProven, src/common/guarded_run.hpp).
    [[nodiscard]] std::uint64_t paces() const noexcept { return paces_; }

    /// The pacer seam (file banner).
    void pace() override {
        ++paces_;  // every pace, armed or not: the boot's chain proof
        if (armed_.load()) {
            CompetitionCause c = cause_.load();
            if (c == CompetitionCause::None) {
                const std::uint8_t s = status_->read();
                c = competitionCauseOf(s);
                if (c != CompetitionCause::None) {
                    statusAtLatch_.store(s);
                    cause_.store(c);
                }
            }
            if (c != CompetitionCause::None && guard_ != nullptr && guard_->running()) {
                guard_->requestAbandon();  // every tick while latched (file banner)
            }
        }
        inner_->pace();
    }

private:
    motion::ITickPacer* inner_;
    ICompetitionStatus* status_;
    sequence::RunGuard* guard_ = nullptr;
    std::uint64_t paces_ = 0;
    std::atomic<bool> armed_{false};
    std::atomic<CompetitionCause> cause_{CompetitionCause::None};
    std::atomic<std::uint8_t> statusAtLatch_{0};
};

}  // namespace shulib::bench
