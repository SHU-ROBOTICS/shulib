#pragma once
//
// competition_run.hpp -- ONE autonomous REQUEST of "shulib Competition", exactly as the program's
// auton task serves it, and the HANDSHAKE that keeps every other task off the robot until it is
// done (chunk COMP-b, 2026-09-30; the COMP brief's B2, B7, B9, B12, B19, B30 and its re-check's RC1,
// ND1, ND2 and ND5).
//
// ═══ WHY PROS-FREE ══════════════════════════════════════════════════════════════════════
// This is the sequence that decides whether the robot moves in a match and who owns it when it
// stops. The program's TU cannot be compiled on the host (pros/screen.h), so the sequence lives
// here, where the host suite runs it on the A2 plant with the REAL CompetitionWatch between the
// plant's pacer and the guard (test/competition_run_test.cpp). The program only calls
// runAutonRequest() from its auton task, once per request, and paints what it is told.
//
// ═══ THE TERMINAL WORD (B2, as the re-check's RC1 binds it) ═════════════════════════════════
// autonomous() makes a REQUEST (AutonHandshake::request) and the auton task serves it. Every path
// out of a request -- an auton that ran, NONE chosen, a refusal, an abandon, a THROW -- ends by
// publishing the request's terminal word, FINISHED (AutonHandshake::finish), after the menu is
// unlocked. Nothing idles for ever without publishing it. opcontrol() waits for it with NO upper
// bound (waitForAutonTask below), so the only thing that can hold driver control back is a
// routine frozen outside the guard's waits -- never a caught error.
//
// ═══ ONE TRY FROM THE LOCK TO THE RUN'S END (RC1; the team lead's ND1 of 2026-09-28) ═════════════
// Whatever throws while a request is served -- the status painter, runAuton's own refusal log, the
// guard's rethrow of a routine's precondition, a student's std::out_of_range -- is caught HERE,
// and the handler: (1) runs the PROGRAM-ERROR STOP (src/common/guarded_run.hpp: the drive to
// 0 V + Brake, every present mechanism to its safe state and its mechanism-service slot cancelled,
// each step alone, none throwing); (2)
// logs one Error line naming what threw; (3) latches the program error, which paints the fault
// screen and REFUSES every later auton on this boot (ND1 (a), the team lead's answer: "yes, after
// a safe stop" -- driver control still runs, because the Chassis is idle and callable after the
// stop; later autons are refused because the program's own state is unproven after an unknown
// throw). Then FINISHED, like every other path.
//
// ═══ NOTHING TOUCHES THE ROBOT AFTER FINISHED (ND5) ═══════════════════════════════════════
// FINISHED hands the robot to opcontrol(). Everything the record needs from the robot -- the END
// estimate, this run's faults -- is read BEFORE it. AFTER it, the auton task only formats and
// writes: the run's lines to serial and the SD run log (ND5, the team lead's "yes": appended to the
// Auton Tester's file, after FINISHED so the card never delays driver control), and a throw there
// latches the program error and says so -- it never runs the program-error stop, because the
// driver may be driving by then.
//
// ═══ THE REFUSALS, BEFORE ANYTHING MOVES ═════════════════════════════════════════════════
// After the choice is LOCKED (competition::AutonMenu::lockChoice -- one atomic operation, so a touch
// racing the start cannot change the auton that runs): a program error earlier on this boot (ND1),
// a boot from the TYPED TABLE (ND2, the team lead's "refuse the auton, not the boot": the table's
// front may be the OTHER end from the calibration record's, so an auton could drive the wrong way;
// driver control still runs), NONE chosen (NONE is the menu's default: no auton runs), or -- read
// ONCE before anything moves -- a status that says autonomous is already OVER (TOO LATE: a request
// served after its period, behind a run held past its end; COMP-b's review). Each moves nothing,
// and each still publishes FINISHED.
//
// ═══ THE DRIVER LOOP LEAVES FIRST (deviation 7, made to hold at COMP-b's review) ═══════════════════
// The driver loop ends on a disable, on the AUTONOMOUS bit and on a waiting request, so that it
// never shares the robot with the auton task even if PROS did NOT delete opcontrol() at a direct
// change from driver control to autonomous (A4 register HA-161 (a) is the belief that it does).
// But the loop checks that only at the top of each tick, and its exit runs chassis.cancel() and
// every binding's makeSafe(): an auton task that started at once could be cancelled by it. So the
// opcontrol task CLAIMS the robot before its loop (DriverLoopPresence::enter -- refused while a
// request waits), beats once per tick, and leaves after the loop; and the auton task does not
// serve a request while a driver loop is in, until it has LEFT -- or until its heartbeat has
// stopped for kDriverLoopStaleSeconds (a loop PROS deleted mid-tick never leaves). The wait is
// host-tested (waitForDriverLoop); the request's lines say when it happened.
//
// ═══ THE MECHANISM SERVICE AND THE ROUTE REGISTRY (the mechanism core's MECH-A5, 2026-10-29) ═══════
// The run goes through competition::runAuton's MECHANISM-SERVICE overload: the service's
// enterAutonomous() first (whatever a deleted driver task left running stops; a pneumatic keeps its
// level), the guard safes the service's built mechanisms, and the routine's AutonContext carries the
// service, so `Routine r{ctx, "name"}` reaches every mechanism by its declaration. The program's
// route REGISTRY carries the robot's derived mechanism ROUTE IDS (runner::MechanismRoutes, registered
// at boot after the service binds): a ROUTE auton's markers fire them — runAuton refuses a route entry
// only when no registry is handed over.
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station K is its first run).

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <span>
#include <string_view>

#include "common/auton_run_log.hpp"
#include "common/guarded_run.hpp"
#include "common/state_screens.hpp"
#include "programs/competition_watch.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// Every serial line of the competition program carries this prefix. The library's own lines keep
/// theirs: runAuton's are [COMP], the run guard's [SEQ].
inline constexpr const char* kCompetitionTag = "CMP";

/// The most bytes one log line may take: TermSink cuts a message at 200 (diag/term_sink.hpp).
inline constexpr std::size_t kCompetitionLineBytes = 200;

/// B2's PROGRAM CONSTANT: how long opcontrol() waits for the auton task's terminal word before it
/// paints `AUTON ENDING -- driver control waits` and logs why. REASONED, not measured: an abandon
/// hands the robot back within a few ticks -- the watch latches on the next pace, the guard cuts
/// the motion on the one after, the routine records the stop and skips its remaining steps, and
/// run()'s cancel-all safes the mechanisms (~30 ms on the host test plant) -- so 0.25 s (25 ticks,
/// about eight times that) never shows the words for an ordinary hand-back, and is still inside a
/// person's reaction time when a routine is frozen in a wait the guard CANNOT cut (Chassis::wait,
/// Routine::pause: they run to their own end -- the guard's waits, guard.pause() and
/// guard.waitFor(), are the ones an abandon cuts). It decides only WHEN the program says so; the
/// wait itself has no upper bound (B2).
inline constexpr double kAutonEndingAfterSeconds = 0.25;

// ═══ THE HANDSHAKE ═══════════════════════════════════════════════════════════════════════

/// autonomous()'s request and the auton task's terminal word (file banner). Any task may call any
/// member. A request made while an earlier one is being served COLLAPSES into the next serve: the
/// auton task serves the NEWEST request, once (two autonomous() calls during one run -- a mode
/// change and back -- run one auton, the choice current at that serve). The request number and
/// busy()'s reads are SEQUENTIALLY CONSISTENT: with DriverLoopPresence (below) they are the two
/// halves of one store-then-load handshake, so the opcontrol task and the auton task can never
/// both decide the robot is theirs. No single-threaded host test can observe that order:
/// test/competition_source_test.cpp's (P16) pins every member of the pair whole.
class AutonHandshake {
public:
    /// autonomous(): a new request, made at robot-clock time `atSeconds`; returns its number (1, 2,
    /// ...). The time is stored first, so the auton task never reads a number with an older time.
    /// Kept in whole MILLISECONDS in a 32-bit atomic (a 64-bit atomic double may not be lock-free
    /// on the V5's Cortex-A9); it wraps after 49 days of uptime.
    std::uint32_t request(double atSeconds) noexcept {
        const double ms = atSeconds > 0.0 ? atSeconds * 1000.0 + 0.5 : 0.0;
        atMs_.store(static_cast<std::uint32_t>(ms), std::memory_order_relaxed);
        return requested_.fetch_add(1) + 1;  // seq_cst (the class doc)
    }
    /// The auton task: true when a request newer than `served` is waiting; it then gives the
    /// newest request's number and the time autonomous() made it.
    [[nodiscard]] bool pending(std::uint32_t served, std::uint32_t& request,
                               double& atSeconds) const noexcept {
        const std::uint32_t r = requested_.load();
        if (r == served) {
            return false;
        }
        request = r;
        atSeconds = static_cast<double>(atMs_.load(std::memory_order_relaxed)) / 1000.0;
        return true;
    }
    /// The auton task's TERMINAL WORD for request `n`: FINISHED. Every path out of a request.
    void finish(std::uint32_t n) noexcept { finished_.store(n); }
    /// True while a request has no terminal word yet: opcontrol() waits on it, and the driver loop
    /// ends on it.
    [[nodiscard]] bool busy() const noexcept { return finished_.load() != requested_.load(); }
    /// True once request `n` -- or a newer one it collapsed into -- has its terminal word.
    [[nodiscard]] bool isFinished(std::uint32_t n) const noexcept {
        return finished_.load(std::memory_order_acquire) >= n;
    }
    /// The newest request's number (0 before the first).
    [[nodiscard]] std::uint32_t requested() const noexcept {
        return requested_.load(std::memory_order_acquire);
    }
    /// The last request FINISHED (0 before the first).
    [[nodiscard]] std::uint32_t finished() const noexcept {
        return finished_.load(std::memory_order_acquire);
    }

private:
    std::atomic<std::uint32_t> requested_{0};
    std::atomic<std::uint32_t> finished_{0};
    std::atomic<std::uint32_t> atMs_{0};
};

/// opcontrol()'s wait for the terminal word (B2): checks every tick, with NO upper bound; once
/// kAutonEndingAfterSeconds have passed it calls `onSlow(waitedSeconds)` ONCE (the program paints
/// AUTON ENDING and logs why). `now()` is the robot clock in seconds, `tick()` sleeps one tick.
/// Returns how long it waited, seconds. It commands nothing: the robot is the auton task's until
/// the word is published.
template <class Now, class Tick, class OnSlow>
double waitForAutonTask(const AutonHandshake& handshake, Now&& now, Tick&& tick, OnSlow&& onSlow) {
    const double t0 = now();
    bool said = false;
    while (handshake.busy()) {
        const double waited = now() - t0;
        if (!said && waited >= kAutonEndingAfterSeconds) {
            onSlow(waited);
            said = true;
        }
        tick();
    }
    return now() - t0;
}

/// The driver loop's end (the driver file's `disabled` predicate, as this program gives it): VEXos
/// DISABLED the robot, OR the status says AUTONOMOUS, OR an auton request is waiting. The last two
/// end driver control even if PROS did not delete the opcontrol task on a direct change to
/// autonomous (A4 register HA-161 is the belief that it does): the auton task must never share the
/// robot with the driver loop -- and it does not start until the loop has LEFT (DriverLoopPresence
/// below). PURE.
[[nodiscard]] constexpr bool driverControlMustEnd(std::uint8_t status,
                                                  bool autonRequestWaiting) noexcept {
    return (status & (kStatusDisabled | kStatusAutonomous)) != 0 || autonRequestWaiting;
}

/// How long a driver loop's heartbeat may stand still before the auton task takes it for gone (a
/// loop PROS deleted mid-tick never leaves). REASONED, not measured: the loop beats once per 10 ms
/// tick, so ten ticks without a beat is a loop that is no longer running -- and far longer than any
/// tick of it (a whole page painted, a controller line written) should take. It delays an auton
/// only in that case: a request that finds a deleted loop waits out what is left of this.
inline constexpr double kDriverLoopStaleSeconds = 0.1;

/// THE DRIVER LOOP'S PRESENCE (file banner, "the driver loop leaves first"): the opcontrol task's
/// claim on the robot, and its heartbeat. Any task may read; only the opcontrol task writes.
class DriverLoopPresence {
public:
    /// opcontrol(), after its wait for the auton task and BEFORE the driver file's loop: claim the
    /// robot for driver control. The claim is STORED first and the request READ after it, and the
    /// auton task reads the two in the other order (both sequentially consistent), so the two tasks
    /// cannot both go ahead. False, and nothing claimed, when a request is already waiting: wait
    /// for the auton task again. It BEATS first, so a loop that has only just claimed the robot is
    /// not taken for a stale one before its first tick. (P16 pins this body whole.)
    [[nodiscard]] bool enter(const AutonHandshake& handshake) noexcept {
        beats_.fetch_add(1);
        active_.store(true);
        if (handshake.busy()) {
            active_.store(false);
            return false;
        }
        return true;
    }
    /// Once per tick of the driver loop (the program beats from the loop's end predicate).
    void beat() noexcept { beats_.fetch_add(1); }
    /// opcontrol(), after the driver file's loop returned: the robot is no longer driver control's.
    void leave() noexcept { active_.store(false); }
    /// True while a driver loop holds the robot (or was deleted holding it).
    [[nodiscard]] bool active() const noexcept { return active_.load(); }
    /// How many times the loop has beaten since the boot.
    [[nodiscard]] std::uint32_t beats() const noexcept { return beats_.load(); }

private:
    std::atomic<bool> active_{false};
    std::atomic<std::uint32_t> beats_{0};
};

/// The auton task's view of the driver loop's heartbeat: when it last saw it move. The auton task
/// alone calls these -- every idle tick, and every tick of a wait -- so a loop that died long ago
/// is already known to be gone when a request comes.
class DriverLoopWatcher {
public:
    /// Note the heartbeat at robot-clock time `now` (seconds).
    void observe(const DriverLoopPresence& presence, double now) noexcept {
        const std::uint32_t b = presence.beats();
        if (!seen_ || b != beats_) {
            seen_ = true;
            beats_ = b;
            movedAt_ = now;
        }
    }
    /// True when the auton task may take the robot now: no driver loop holds it, or the loop that
    /// does has not beaten for kDriverLoopStaleSeconds.
    [[nodiscard]] bool mayServe(const DriverLoopPresence& presence, double now) const noexcept {
        return !presence.active() || (seen_ && now - movedAt_ >= kDriverLoopStaleSeconds);
    }

private:
    bool seen_ = false;
    std::uint32_t beats_ = 0;
    double movedAt_ = 0.0;
};

/// What the auton task's wait for the driver loop found (the request's lines say it when `waited`).
struct DriverLoopWait {
    bool waited = false;   ///< a driver loop held the robot when the request was served
    bool left = false;     ///< ... and it LEFT (false: its heartbeat stopped -- deleted, or stalled)
    double seconds = 0.0;  ///< how long the auton task waited
};

/// The auton task, before it serves a request: wait, a tick at a time, until no driver loop holds
/// the robot -- it LEFT, or its heartbeat stopped (DriverLoopWatcher::mayServe). Commands nothing.
/// `now()` is the robot clock in seconds, `tick()` sleeps one tick. No upper bound of its own: a
/// live loop leaves within a tick of the request (it ends on a waiting request), and a dead one is
/// let go after kDriverLoopStaleSeconds.
template <class Now, class Tick>
DriverLoopWait waitForDriverLoop(const DriverLoopPresence& presence, DriverLoopWatcher& watcher,
                                 Now&& now, Tick&& tick) {
    DriverLoopWait w{};
    const double t0 = now();
    watcher.observe(presence, t0);
    while (!watcher.mayServe(presence, now())) {
        w.waited = true;
        tick();
        watcher.observe(presence, now());
    }
    if (w.waited) {
        w.left = !presence.active();
        w.seconds = now() - t0;
    }
    return w;
}

// ═══ THE PROGRAM'S FLAGS, THE RECORD, THE PARTS ═════════════════════════════════════════════

/// What the tasks share besides the handshake. `bootFromRecord` is set once, at boot, before the
/// auton task exists; `programError` is latched by the auton task (a throw it caught) and read by
/// every task (the screens and the refusal of later autons).
struct CompetitionFlags {
    /// The boot chose the calibration RECORD (ND2: a TYPED TABLE boot refuses every auton).
    bool bootFromRecord = false;
    /// A program error was caught on the auton task: the fault screen, and every later auton on
    /// this boot REFUSED (ND1). Never cleared: a restart clears it.
    std::atomic<bool> programError{false};
};

/// True when every auton on this boot is REFUSED (ND1's program error, or ND2's typed-table boot):
/// the menu phase then shows why instead of the selector, and a request runs nothing. PURE.
[[nodiscard]] inline bool autonsRefused(const CompetitionFlags& flags) noexcept {
    return flags.programError.load() || !flags.bootFromRecord;
}

/// What a program error says, for the autons-refused page (the auton task's; the program keeps one).
inline constexpr std::size_t kCompetitionErrorBytes = 160;

/// Why a request ran no auton, before anything armed.
enum class CompetitionSkip : std::uint8_t {
    None,               ///< nothing skipped: the chosen auton went to runAuton
    NoneChosen,         ///< NONE was the choice (the menu's default): no auton runs
    BootFromTable,      ///< this boot used the TYPED TABLE: autons refused (ND2)
    AfterProgramError,  ///< a program error earlier on this boot: autons refused until a restart
    TooLate,            ///< the status said autonomous was already OVER when the request was served
};

/// One request, as the auton task served it -- the lines and the result page are built from it.
struct CompetitionRunRecord {
    std::uint32_t request = 0;                     ///< the handshake's request number (0: none yet)
    int number = 0;                                ///< the auton task's serve count, 1-based
    int entry = -1;                                ///< the locked choice (-1 = NONE)
    CompetitionSkip skip = CompetitionSkip::None;  ///< why no auton ran, if one did not
    bool threw = false;                            ///< a program error was caught
    char threwWhat[160]{};                         ///< what it said
    competition::AutonRunResult run{};             ///< valid when skip == None and !threw
    CompetitionCause cause = CompetitionCause::None;  ///< the watch's latch at the end: a run's, or
                                                      ///< (a skip's) what the serve's read said
    std::uint8_t statusAtLatch = 0;                ///< the status byte it latched on
    double requestedAt = 0.0;                      ///< the robot clock when autonomous() asked (s)
    double servedAt = 0.0;                         ///< ... and when the auton task served it (s)
    DriverLoopWait loopWait{};                     ///< the auton task's wait for the driver loop
    math::Pose2d endEstimate{};                    ///< the estimate when the request ended
    bool endLive = false;                          ///< ... and whether it was live
    int faultsBefore = 0;                          ///< the fault latch's count at the request
    int faultsAfter = 0;                           ///< ... and when it ended
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< the last fault DURING it, or None
};

/// What the auton task works on. The references must outlive every request (the program's static
/// state). `chassis` MUST be the Chassis built with `guard` as its pacer, the guard's inner pacer
/// MUST be `mechanisms` and the service's inner pacer `watch` (B7, MECH-A5; pacerChainMissing()
/// checks it at boot). `mechanisms` is the robot's mechanism service, bound at boot: the guard safes
/// its built mechanisms — the Robot's PRESENT ones (B9). `commands` is the route registry the
/// robot's mechanism route ids were registered into (nullptr: a ROUTE auton is refused by name).
/// `telemetry` gets runAuton's lines and the program-error line.
struct CompetitionParts {
    chassis::Chassis& chassis;                   ///< the robot's Chassis (paced by `guard`)
    sequence::RunGuard& guard;                   ///< the guaranteed end
    CompetitionWatch& watch;                     ///< under the service: disable / mode change
    competition::AutonMenu& menu;                ///< the choice (locked for the run)
    const competition::AutonRegistry& registry;  ///< the team's autons (src/team/autons.cpp)
    hal::ITelemetrySink& telemetry;              ///< serial
    mechanism::MechanismService& mechanisms;     ///< the robot's mechanisms (under the guard)
    const runner::CommandRegistry* commands;     ///< the route registry (the mechanism route ids)
    AutonHandshake& handshake;                   ///< the terminal word
    CompetitionFlags& flags;                     ///< the typed-table boot and the program error
};

/// What the screen is told while a request is served (the program paints; a host test records).
class ICompetitionRunObserver {
public:
    virtual ~ICompetitionRunObserver() = default;
    ICompetitionRunObserver() = default;
    ICompetitionRunObserver(const ICompetitionRunObserver&) = default;
    ICompetitionRunObserver(ICompetitionRunObserver&&) = default;
    ICompetitionRunObserver& operator=(const ICompetitionRunObserver&) = default;
    ICompetitionRunObserver& operator=(ICompetitionRunObserver&&) = default;
    /// The choice is locked and the decision made (`rec.entry`, `rec.skip`): the status page,
    /// painted BEFORE runAuton (RC3). A throw here is a program error like any other.
    virtual void starting(const CompetitionRunRecord& rec) = 0;
};

namespace detail {
/// The auton's name for the lines and pages: its registered name, or "NONE".
[[nodiscard]] inline const char* competitionEntryName(const competition::AutonRegistry& registry,
                                                      int entry) noexcept {
    return entry >= 0 && static_cast<std::size_t>(entry) < registry.size()
               ? registry.at(static_cast<std::size_t>(entry)).name
               : "NONE";
}
}  // namespace detail

// ═══ ONE REQUEST ═════════════════════════════════════════════════════════════════════════

/// Serve ONE request (file banner): lock the choice, read the status once, refuse or run it through
/// competition::runAuton under the guard with the watch armed, all inside ONE try; on a throw the
/// program-error stop, one Error line and the latched program error; then -- on EVERY path -- the
/// watch disarmed, the END estimate and this request's faults read, the menu unlocked, and FINISHED
/// published (in that order, and FINISHED last: test/competition_source_test.cpp pins it, and
/// test/competition_run_test.cpp's (R18) watches the stop). Fills and returns `rec` (the caller's
/// storage -- never a task stack). `loopWait` is what the auton task's wait for the driver loop
/// found (waitForDriverLoop), recorded for the lines. Only from the auton task.
inline CompetitionRunRecord& serveAutonRequest(CompetitionParts& p, CompetitionRunRecord& rec,
                                               std::uint32_t request, double requestedAt,
                                               int number, ICompetitionRunObserver& observer,
                                               const DriverLoopWait& loopWait = {}) {
    rec = CompetitionRunRecord{};
    rec.request = request;
    rec.number = number;
    rec.requestedAt = requestedAt;
    rec.loopWait = loopWait;
    try {
        rec.servedAt = p.chassis.deps().ctx->clock().now().value();
        rec.faultsBefore = p.chassis.deps().faults->faultCount();
        rec.entry = p.menu.lockChoice();  // the auton that runs is the choice at THIS instant
        // ONE status read before anything moves: a request served after its period is not run.
        const CompetitionCause over = p.watch.latchNow();
        if (p.flags.programError.load()) {
            rec.skip = CompetitionSkip::AfterProgramError;
        } else if (!p.flags.bootFromRecord) {
            rec.skip = CompetitionSkip::BootFromTable;
        } else if (rec.entry < 0) {
            rec.skip = CompetitionSkip::NoneChosen;
        } else if (over != CompetitionCause::None) {
            rec.skip = CompetitionSkip::TooLate;
            rec.cause = over;
            rec.statusAtLatch = p.watch.statusAtLatch();
        }
        observer.starting(rec);
        if (rec.skip == CompetitionSkip::None) {
            const competition::AutonEntry& entry =
                p.registry.at(static_cast<std::size_t>(rec.entry));
            p.watch.arm();
            // THE RUN: the same call the Auton Tester makes from its RUN, on the same entry,
            // schedule, re-seat and guaranteed end — runAuton's MECHANISM-SERVICE overload (the
            // service handed over from driver control, the guard safing its mechanisms, the
            // routine's context carrying it), with the route registry the robot's mechanism route
            // ids are in (MECH-A5; until then no registry, and a ROUTE auton was refused by name —
            // the COMP brief's B30).
            rec.run = competition::runAuton(p.chassis, p.guard, entry, p.commands, p.telemetry,
                                            p.mechanisms, p.registry.periodFor(entry));
        }
    } catch (const PreconditionError& e) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%.159s", e.what());
    } catch (const std::exception& e) {
        rec.threw = true;  // a student's std::out_of_range: its OWN words, never "unknown"
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%.159s", e.what());
    } catch (...) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%s", "an unknown exception");
    }
    if (rec.threw) {
        stopAfterProgramError(p.chassis, p.mechanisms.mechanisms());  // 0 V + Brake, mechanisms safe
        p.flags.programError.store(true);             // the fault screen; later autons refused
        try {
            char line[kCompetitionLineBytes];
            std::snprintf(line, sizeof line,
                          "auton task: PROGRAM ERROR on request %u: %.110s -- the robot is stopped",
                          static_cast<unsigned>(request), rec.threwWhat);
            p.telemetry.log(hal::LogLevel::Error, kCompetitionTag, line);
        } catch (...) {  // NOLINT(bugprone-empty-catch): the terminal word must still be published
        }
    }
    p.watch.disarm();
    // The watch's latch describes THIS request: latchNow() cleared it at the serve (its read is a
    // skip's), and arm() cleared it again for a run (its latch is the run's).
    rec.cause = p.watch.cause();
    rec.statusAtLatch = p.watch.statusAtLatch();
    try {
        // Read BEFORE FINISHED: after it the robot is opcontrol()'s (file banner).
        const localization::Localizer& localizer = *p.chassis.deps().localizer;
        const diag::FaultLatch& faults = *p.chassis.deps().faults;
        rec.endEstimate = localizer.pose();
        rec.endLive =
            localizer.qualityClass() != localization::Localizer::Quality::Uninitialized;
        rec.faultsAfter = faults.faultCount();
        rec.lastFault =
            rec.faultsAfter > rec.faultsBefore ? faults.lastFault() : diag::FaultCode::None;
    } catch (...) {  // NOLINT(bugprone-empty-catch): reads only; the word must still be published
    }
    p.menu.unlock();               // RC1: every path (auton_menu.hpp: only after FINISHED is near)
    p.handshake.finish(request);   // FINISHED -- the terminal word, on every path (RC1)
    return rec;
}

// ═══ THE WORDS ═══════════════════════════════════════════════════════════════════════════

/// True when the watch latched a cause during a run the guard did NOT abandon: the watch asks for
/// the abandon on every pace while latched and the guard runs, so this is never a late change -- it
/// is the request not reaching the guard (a wiring fault the boot's chain proof exists to refuse),
/// and it must never read as a pass.
[[nodiscard]] inline bool abandonIgnored(const CompetitionRunRecord& r) noexcept {
    return r.cause != CompetitionCause::None && r.skip == CompetitionSkip::None && !r.threw
           && r.run.outcome != competition::AutonRunOutcome::Refused && !r.run.guard.abandoned;
}

/// STEP1-A (D5 (a)): the word for a routine that FAILED steps and carried on to its end -- "DONE, 2
/// FAILED", and "DONE, 9+ FAILED" for ten or more (the tester's words, restated here so this header
/// does not include the tester's -- the COMP-t brief's §4). Every word is at most 15 characters: it
/// fits the driver page's "auton %.15s" slot unchanged (competition_screen.hpp). `failed` below 1
/// reads as 1 (never reached: the caller asks only when failedOn > 0).
[[nodiscard]] inline const char* competitionDoneWord(int failed) noexcept {
    static constexpr const char* kWords[10] = {"DONE, 1 FAILED", "DONE, 2 FAILED", "DONE, 3 FAILED",
                                               "DONE, 4 FAILED", "DONE, 5 FAILED", "DONE, 6 FAILED",
                                               "DONE, 7 FAILED", "DONE, 8 FAILED", "DONE, 9 FAILED",
                                               "DONE, 9+ FAILED"};
    return kWords[failed < 1 ? 0 : (failed > 9 ? 9 : failed - 1)];
}

/// The word a served request earns -- the result page's big line and the first log line say the
/// same (one source). First match wins: a throw, NONE, TOO LATE (the period was over at the serve),
/// a refusal (ours or runAuton's), an abandon, an abandon the guard never acted on (red), never
/// started, the hard floor, the deadline cut, then (STEP1-A) DONE, n FAILED (a routine that carried
/// on past failed steps to its end -- no stop, no skip still looking for its step -- and the end
/// action succeeded; amber, logged at Warn), then FINISHED (the routine -- or route -- OK and the
/// end action succeeded) or ROUTINE STOPPED. At most ui::kLargeCols characters (test (R13)).
[[nodiscard]] inline const char* competitionResultWord(const CompetitionRunRecord& r) noexcept {
    if (r.threw) {
        return "PROGRAM ERROR";
    }
    if (r.skip == CompetitionSkip::NoneChosen) {
        return "NONE";
    }
    if (r.skip == CompetitionSkip::TooLate) {
        return "TOO LATE";
    }
    if (r.skip != CompetitionSkip::None
        || r.run.outcome == competition::AutonRunOutcome::Refused) {
        return "REFUSED";
    }
    if (r.run.guard.abandoned) {
        return "ABANDONED";
    }
    if (abandonIgnored(r)) {
        return "ABANDON IGNORED";
    }
    if (r.run.outcome == competition::AutonRunOutcome::NotStarted) {
        return "NOT STARTED";
    }
    if (r.run.guard.floorFired) {
        return "HARD STOP";
    }
    if (r.run.guard.scoringCut) {
        return "CUT BY DEADLINE";
    }
    if (r.run.outcome == competition::AutonRunOutcome::RoutineRan) {
        const chassis::RoutineResult& rr = r.run.routine;
        if (rr.stoppedAt == 0 && rr.skipPendingTo[0] == '\0' && rr.failedOn > 0
            && r.run.guard.endActionSucceeded) {
            return competitionDoneWord(rr.failedOn);
        }
    }
    const bool ranOk = r.run.outcome == competition::AutonRunOutcome::RouteRan
                           ? r.run.route.ok()
                           : r.run.routine.ok;
    return ranOk && r.run.guard.endActionSucceeded ? "FINISHED" : "ROUTINE STOPPED";
}

/// The level a served request's lines are logged at: Info for FINISHED and NONE, Error for a
/// program error or an ignored abandon, Warn for the rest.
[[nodiscard]] inline hal::LogLevel competitionRunLevel(const CompetitionRunRecord& r) noexcept {
    const std::string_view w = competitionResultWord(r);
    if (w == "FINISHED" || w == "NONE") {
        return hal::LogLevel::Info;
    }
    if (w == "PROGRAM ERROR" || w == "ABANDON IGNORED") {
        return hal::LogLevel::Error;
    }
    return hal::LogLevel::Warn;
}

/// The routine's stop, in the words its own stop line uses (the tester's words, restated here so
/// this header does not include the tester's -- the COMP-t brief's §4).
[[nodiscard]] constexpr const char* competitionStopWord(chassis::RoutineStopCause cause,
                                                        control::ExitReason exit) noexcept {
    switch (cause) {
        case chassis::RoutineStopCause::None: return "no stop";
        case chassis::RoutineStopCause::MotionFailed:
            return exit == control::ExitReason::TimedOut    ? "motion TIMEOUT"
                   : exit == control::ExitReason::Cancelled ? "motion CANCELLED"
                                                            : "motion failed";
        case chassis::RoutineStopCause::WaitTimedOut: return "wait TIMEOUT";
        case chassis::RoutineStopCause::ActionFailed: return "action FAILED";
        case chassis::RoutineStopCause::MechanismFailed: return "mechanism FAILED";
        case chassis::RoutineStopCause::MechanismStepFailed: return "mechanism step FAILED";
        case chassis::RoutineStopCause::MechanismNotStarted: return "mechanism NOT STARTED";
    }
    return "stopped";
}

/// The lines one served request leaves in the log -- serial and the SD card carry the same words.
struct CompetitionRunLines {
    /// each <= kCompetitionLineBytes - 1 characters. Six since STEP1-A: a routine that carried on past
    /// a failure (or whose skip never landed) adds line 3b, the first failed step.
    char line[6][kCompetitionLineBytes]{};
    int count = 0;                          ///< how many are real
    bool cut = false;  ///< a line did not fit and was cut (a test failure, never expected)
};

namespace detail {
/// Record one snprintf's verdict: a line that did not fit sets `cut`.
inline void noteCompetitionLine(CompetitionRunLines& out, int written) noexcept {
    if (written < 0 || static_cast<std::size_t>(written) >= kCompetitionLineBytes) {
        out.cut = true;
    }
}
/// A duration for a line, kept to [0, 99999.999] s so its width is bounded (a wait is never
/// negative, and nothing on the robot waits a day).
[[nodiscard]] inline double boundedSeconds(double s) noexcept {
    return s < 0.0 ? 0.0 : (s > 99999.999 ? 99999.999 : s);
}
}  // namespace detail

namespace detail {
/// competitionRunLines()' lines but the driver-loop line (below).
inline void competitionRequestLines(CompetitionRunLines& out, const CompetitionRunRecord& r,
                                    const competition::AutonRegistry& registry) {
    const unsigned req = static_cast<unsigned>(r.request);
    const char* name = detail::competitionEntryName(registry, r.entry);
    const char* word = competitionResultWord(r);
    const auto add = [&out](int written) {
        detail::noteCompetitionLine(out, written);
        ++out.count;
    };
    if (r.threw) {
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: '%.29s' %s -- it threw: %.100s", req, name, word,
                          r.threwWhat));
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: the drive is at 0 V + Brake and every present mechanism "
                          "safe; driver control still runs; later autons are REFUSED until the "
                          "program restarts",
                          req));
        return;
    }
    switch (r.skip) {
        case CompetitionSkip::NoneChosen:
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: NONE -- no auton was chosen (NONE is the default); "
                              "nothing moved",
                              req));
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: choose an auton on the brain's menu before the match "
                              "starts (disabled, or before the field connects)",
                              req));
            return;
        case CompetitionSkip::BootFromTable:
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: '%.29s' REFUSED -- this boot used the TYPED TABLE, whose "
                              "front may be the OTHER end; nothing moved",
                              req, name));
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: run shulib Calibrate so the robot boots from its "
                              "calibration record, then restart; driver control still runs",
                              req));
            return;
        case CompetitionSkip::AfterProgramError:
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: '%.29s' REFUSED -- a program error earlier on this "
                              "boot (its own Error line says where); nothing moved",
                              req, name));
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: restart the program to run autons again; driver "
                              "control still runs",
                              req));
            return;
        case CompetitionSkip::TooLate:
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: '%.29s' TOO LATE -- %.27s (status 0x%02x) when the auton "
                              "task served it, %.3fs after autonomous() asked; nothing moved",
                              req, name, competitionCauseWord(r.cause),
                              static_cast<unsigned>(r.statusAtLatch),
                              boundedSeconds(r.servedAt - r.requestedAt)));
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: an auton runs only while autonomous is on, and this "
                              "request was served after its period; driver control still runs",
                              req));
            return;
        case CompetitionSkip::None:
            break;
    }
    const sequence::RunGuardReport& g = r.run.guard;
    const bool refused = r.run.outcome == competition::AutonRunOutcome::Refused;
    const bool ignored = abandonIgnored(r);
    const bool named = g.abandoned || ignored;

    // 1. the word, the outcome, how it ended (and what ended it), the end action.
    if (refused) {
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: '%.29s' %s -- runAuton refused it before the guard armed; "
                          "nothing moved",
                          req, name, word));
    } else {
        char status[20] = "";  // the status byte the watch latched on, when it ended the run
        if (named) {
            std::snprintf(status, sizeof status, " (status 0x%02x)",
                          static_cast<unsigned>(r.statusAtLatch));
        }
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: '%.29s' %s -- %s; %s%.27s%.16s, end action %s%s", req, name,
                          word, competition::autonRunOutcomeWord(r.run.outcome),
                          g.abandoned ? "ABANDONED: "
                          : ignored   ? "IGNORED the watch: "
                                      : (g.scoringCut ? "cut at the deadline" : "in time"),
                          named ? competitionCauseWord(r.cause) : "", status,
                          !g.endActionRan       ? "skipped"
                          : g.abandoned         ? "CUT by the abandon"
                          : g.endActionSucceeded ? "succeeded"
                                                 : "FAILED",
                          g.floorFired ? ", HARD STOP fired" : ""));
    }

    // 2. the guard's clock and B12's hand-off: armedAt against autonomous()'s own reading.
    if (refused) {
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: %.150s", req, r.run.why));
        return;
    }
    char endedAt[24] = "--";  // "--" when the end action did not run
    if (g.endActionRan) {
        std::snprintf(endedAt, sizeof endedAt, "%.2fs", g.endActionEnded.value());
    }
    add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                      "request %u: scoring ended T+%.2fs, end action ended T+%.12s, %d paces, %d "
                      "cancels after the cut; hand-off %.3fs after autonomous(), live after %.2fs",
                      req, g.scoringEnded.value(), endedAt, g.pacesSeen, g.postExpiryCancels,
                      r.run.armedAt.value() - r.requestedAt, r.run.liveAfter.value()));

    // 3. the routine's verdict (or runAuton's sentence for NOT STARTED), and THIS request's faults.
    const int faultsThisRun = r.faultsAfter - r.faultsBefore;
    char faults[80] = "faults this run: none";
    if (faultsThisRun != 0) {
        std::snprintf(faults, sizeof faults, "faults this run: %d, last %.20s -- see the [ERROR] line",
                      faultsThisRun, diag::faultCodeName(r.lastFault));
    }
    if (r.run.outcome == competition::AutonRunOutcome::RoutineRan) {
        const chassis::RoutineResult& rr = r.run.routine;
        const bool skipPending = rr.skipPendingTo[0] != '\0';
        if (rr.stoppedAt > 0) {
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: routine STOPPED, %d of %d steps done, stopped at %.24s "
                              "(%s); %.79s",
                              req, rr.completed, rr.steps, rr.stoppedName,
                              competitionStopWord(rr.cause, rr.exit), faults));
        } else if (rr.failedOn > 0 || skipPending) {
            // STEP1-A: the routine went on past a failure (or a skip is still looking for its step).
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: routine %s, %d of %d steps done, %d failed, %d jumped; "
                              "%.79s",
                              req, skipPending ? "STOPPED" : "DONE", rr.completed, rr.steps,
                              rr.failedOn, rr.jumped, faults));
        } else {
            add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                              "request %u: routine %s, %d of %d steps done; %.79s", req,
                              rr.ok ? "OK" : "STOPPED", rr.completed, rr.steps, faults));
        }
        // 3b (STEP1-A). The first failed step, ONLY when the routine went on past one or a skip is
        // still pending -- so a request under the default rule keeps its lines, word for word.
        if (rr.failedOn > 0 || skipPending) {
            if (skipPending) {
                add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                                  "request %u: first failed: step %d %.24s (%s); a skip to '%.24s' "
                                  "never landed",
                                  req, rr.firstFailedAt, rr.firstFailedName,
                                  competitionStopWord(rr.firstFailedCause, control::ExitReason::Running),
                                  rr.skipPendingTo));
            } else {
                add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                                  "request %u: first failed: step %d %.24s (%s); the routine went on%s",
                                  req, rr.firstFailedAt, rr.firstFailedName,
                                  competitionStopWord(rr.firstFailedCause, control::ExitReason::Running),
                                  rr.stoppedAt > 0 ? ", then stopped" : ""));
            }
        }
    } else if (r.run.outcome == competition::AutonRunOutcome::RouteRan) {
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                          "request %u: route %s, %d legs run, %d skipped; %.79s", req,
                          r.run.route.ok() ? "OK" : "STOPPED", r.run.route.legsRun,
                          r.run.route.legsSkipped, faults));
    } else {
        add(std::snprintf(out.line[out.count], kCompetitionLineBytes, "request %u: %.100s; %.79s",
                          req, r.run.why, faults));
    }

    // 4. where it was seated, and the END estimate -- the numbers to put against the field.
    add(std::snprintf(out.line[out.count], kCompetitionLineBytes,
                      "request %u: seated (%.2f, %.2f) facing %.1f deg (the IMU read %.1f); END "
                      "estimate (%.2f, %.2f) facing %.1f deg %s",
                      req, r.run.seated.x().value(), r.run.seated.y().value(),
                      r.run.seated.heading().degrees(), r.run.imuAtSeat.degrees(),
                      r.endEstimate.x().value(), r.endEstimate.y().value(),
                      r.endEstimate.heading().degrees(), r.endLive ? "LIVE" : "NOT LIVE"));
}
}  // namespace detail

/// The request's lines (golden-tested): what happened, the guard's clock and B12's hand-off, the
/// routine's verdict with THIS request's faults, and the seat against the END estimate. A skip and a
/// throw are TWO lines. When the auton task had to wait for the driver loop to leave first, one more
/// line says so (A4 register HA-161 (a)'s evidence: a loop that LEFT was not deleted by PROS at the
/// change to autonomous). Every variable string carries a precision, so -Wformat-truncation can see
/// each line's real bound.
[[nodiscard]] inline CompetitionRunLines competitionRunLines(
    const CompetitionRunRecord& r, const competition::AutonRegistry& registry) {
    CompetitionRunLines out;
    detail::competitionRequestLines(out, r, registry);
    if (r.loopWait.waited) {
        const int written = std::snprintf(
            out.line[out.count], kCompetitionLineBytes,
            "request %u: a driver loop still held the robot when it was served -- %s; the auton "
            "waited %.3fs for it (A4 register HA-161 (a))",
            static_cast<unsigned>(r.request),
            r.loopWait.left ? "it LEFT" : "its heartbeat STOPPED (deleted by PROS, or stalled)",
            detail::boundedSeconds(r.loopWait.seconds));
        detail::noteCompetitionLine(out, written);
        ++out.count;
    }
    return out;
}

/// AFTER FINISHED (ND5): the request's lines to serial, then to the SD run log when it is open.
/// Never touches the robot. A throw here latches the program error (ND1), says so on serial, and
/// writes what threw into `errorWhat` (the autons-refused page's reason) -- it never runs the
/// program-error stop: the driver may be driving by now (file banner). True when it threw.
inline bool reportAutonRun(const CompetitionRunRecord& rec,
                           const competition::AutonRegistry& registry,
                           hal::ITelemetrySink& serial, hal::ITelemetrySink* sd,
                           CompetitionFlags& flags,
                           char (&errorWhat)[kCompetitionErrorBytes]) noexcept {
    const char* what = nullptr;
    try {
        const CompetitionRunLines lines = competitionRunLines(rec, registry);
        const hal::LogLevel level = competitionRunLevel(rec);
        for (int i = 0; i < lines.count; ++i) {
            serial.log(level, kCompetitionTag, lines.line[i]);
        }
        if (sd != nullptr) {
            for (int i = 0; i < lines.count; ++i) {
                sd->log(level, kCompetitionTag, lines.line[i]);
            }
        }
        return false;
    } catch (const std::exception& e) {
        what = e.what();  // its OWN words, never "unknown"
    } catch (...) {
        what = "an unknown exception";
    }
    flags.programError.store(true);
    std::snprintf(errorWhat, sizeof errorWhat,
                  "the report of request %u threw: %.88s -- the robot was not touched",
                  static_cast<unsigned>(rec.request), what != nullptr ? what : "");
    try {
        char line[kCompetitionLineBytes];
        std::snprintf(line, sizeof line,
                      "auton task: PROGRAM ERROR reporting request %u: %.60s -- the robot was not "
                      "touched; later autons REFUSED until a restart",
                      static_cast<unsigned>(rec.request), what != nullptr ? what : "");
        serial.log(hal::LogLevel::Error, kCompetitionTag, line);
    } catch (...) {  // NOLINT(bugprone-empty-catch): nothing may leave the auton task
    }
    return true;
}

/// The auton task's whole request, in THE ORDER THE PROGRAM RUNS IT: serve it (FINISHED is its
/// last act), THEN report it -- so the SD card's write can never delay driver control (ND5). When
/// the request threw, or its report did, `errorWhat` gets what was said (the autons-refused page's
/// reason: ND1's "the screen says why"). The program calls this and nothing else for a request;
/// the host suite calls the same function.
inline CompetitionRunRecord& runAutonRequest(CompetitionParts& p, CompetitionRunRecord& rec,
                                             std::uint32_t request, double requestedAt,
                                             int number, ICompetitionRunObserver& observer,
                                             hal::ITelemetrySink* sd,
                                             char (&errorWhat)[kCompetitionErrorBytes],
                                             const DriverLoopWait& loopWait = {}) {
    serveAutonRequest(p, rec, request, requestedAt, number, observer, loopWait);  // ... FINISHED
    if (rec.threw) {
        std::snprintf(errorWhat, sizeof errorWhat,
                      "request %u threw: %.100s -- the robot was stopped",
                      static_cast<unsigned>(rec.request), rec.threwWhat);
    }
    (void)reportAutonRun(rec, p.registry, p.telemetry, sd, p.flags, errorWhat);  // only after it
    return rec;
}

// ═══ THE BOOT'S LINES ════════════════════════════════════════════════════════════════════

/// The boot's log lines (serial and the SD card), THREE of them, every field bounded (the
/// tester's lesson: one line passed TermSink's 199 bytes at a real build hash): what booted and
/// which build; how it booted, with what, and whether autons are allowed (ND2); the SD run log.
struct CompetitionBootLines {
    char line[3][kCompetitionLineBytes]{};  ///< each <= kCompetitionLineBytes - 1 characters
    bool cut = false;                       ///< a line did not fit (a test failure)
};

/// The boot lines. `buildHash` empty reads MISSING; longer than diag::kMaxHashBytes it is cut there.
/// `sdPath` is printed only when the log is on.
[[nodiscard]] inline CompetitionBootLines competitionBootLines(const char* variantBeacon,
                                                               std::string_view buildHash,
                                                               const char* bootSource,
                                                               bool bootFromRecord, int autons,
                                                               int present, int mechanisms,
                                                               AutonRunLog sd, const char* sdPath) {
    CompetitionBootLines out;
    const auto note = [&out](int written) {
        if (written < 0 || static_cast<std::size_t>(written) >= kCompetitionLineBytes) {
            out.cut = true;
        }
    };
    const std::string_view hash = buildHash.empty() ? std::string_view{"MISSING"} : buildHash;
    const int hashLen = static_cast<int>(hash.size() < diag::kMaxHashBytes ? hash.size()
                                                                            : diag::kMaxHashBytes);
    note(std::snprintf(out.line[0], kCompetitionLineBytes,
                       "boot: shulib COMPETITION [%.40s] hash %.*s",
                       variantBeacon != nullptr ? variantBeacon : "", hashLen, hash.data()));
    note(std::snprintf(out.line[1], kCompetitionLineBytes,
                       "boot source %.11s, %d autons, %d of %d mechanisms present; autons %s",
                       bootSource != nullptr ? bootSource : "", autons, present, mechanisms,
                       bootFromRecord ? "ALLOWED"
                                      : "REFUSED (a typed-table boot: run shulib Calibrate)"));
    note(std::snprintf(out.line[2], kCompetitionLineBytes, "SD run log %s%.60s",
                       autonRunLogWords(sd),
                       sd != AutonRunLog::Off && sdPath != nullptr ? sdPath : ""));
    return out;
}

/// One line on entry to, or return from, a PROS phase function (B5(1)): `what` ("autonomous()"),
/// `when` ("entry", "normal return"), the brain's millis and the decoded status bits (`bits`, from
/// competitionBits()), and an optional note. Station K reads these in serial order: the NEXT phase
/// function's entry line marks a mode change (A4 register HA-161's settle).
[[nodiscard]] inline int competitionPhaseLine(char* out, std::size_t cap, const char* what,
                                              const char* when, std::uint32_t millis,
                                              const char* bits, const char* note) noexcept {
    return std::snprintf(out, cap, "%.24s %.16s at %lu ms, status %.44s%s%.80s",
                         what != nullptr ? what : "", when != nullptr ? when : "",
                         static_cast<unsigned long>(millis), bits != nullptr ? bits : "",
                         note != nullptr && note[0] != '\0' ? " -- " : "",
                         note != nullptr ? note : "");
}

}  // namespace shulib::bench
