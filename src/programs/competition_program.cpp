// competition_program.cpp -- "shulib Competition" (chunk COMP-b, 2026-09-30): the competition
// program. Pick an auton on the brain's menu before the match; the field (or the controller's Timed
// Run) starts autonomous and the program's own task runs it; then driver control runs the team's
// driver file.
//
// ═══ ONE TASK ON THE ROBOT AT A TIME (the COMP brief's B2, C5; the team lead's ruling of 2026-09-24,
// "our own auton task") ═══════════════════════════════════════════════════════════════════
// initialize() builds everything and starts THE AUTON TASK -- a task this program created, which
// PROS never deletes on a mode change (A4 register HA-161). autonomous() only REQUESTS a run and
// waits; the auton task serves it (competition_run.hpp's runAutonRequest) and publishes its
// TERMINAL WORD, FINISHED, on every path -- a finished auton, NONE, a refusal, an abandon, a throw.
// Until then no other task touches the Robot, the Chassis, the guard, the watch or the mechanisms:
// opcontrol() waits for the word with NO upper bound, checking every tick, and after 0.25 s says
// AUTON ENDING on the screen and in the log. A disable or the end of autonomous ends a live auton
// at once: the COMPETITION WATCH (competition_watch.hpp) sits under the run guard in the pacer
// chain and requests the abandon every tick a cause is latched, and the routine unwinds on the
// auton task's own stack -- nothing is left dangling on a task PROS deleted. The driver loop ends
// on a disable, on the autonomous bit and on a waiting request, and the auton task does not serve a
// request until that loop has LEFT (or its heartbeat stopped -- competition_run.hpp's
// DriverLoopPresence), so the two never share the robot even if PROS did not delete opcontrol on a
// direct change to autonomous (A4 register HA-161 (a)).
//
// ═══ THE PACER CHAIN (B7), PROVED AT BOOT ═════════════════════════════════════════════════
//     Chassis -> RunGuard -> CompetitionWatch -> the provider's 10 ms ProsTickPacer
// The boot PROVES it (pacerChainProven, src/common/guarded_run.hpp -- shared with the Auton Tester,
// B30): one idle tick must pace the guard AND the watch, and the watch must be attached to the
// guard, or the boot is REFUSED -- a Chassis built without the guard, or a watch that asks nobody,
// could never be ended by a disable. Driver control's tick is the provider's pacer itself (the
// driver file's run() takes it): nothing in driver control is the guard's to end.
//
// ═══ WHO PAINTS (RC3, ND4) ═══════════════════════════════════════════════════════════════
// The auton task paints the MENU (after the boot, in competition_initialize() and disabled()) and
// the auton's STATUS and RESULT (autonomous()); the opcontrol task paints the DRIVER page (after
// the terminal word, or AUTON ENDING); PROS's phase tasks paint nothing -- they log and hand the
// screen over. One atomic owner decides (competition_screen.hpp's ScreenOwnerCell), and a painter
// checks it before every paint. The auton task NEVER ticks the Chassis while it idles: its idle is
// a one-tick notify_take in a loop that logs every competition-status change, polls and paints the
// menu, and serves a request when one comes.
//
// ═══ REFUSED, NOT RUN (ND1, ND2; C7) ═════════════════════════════════════════════════════
// A boot neither the calibration record nor the typed table can describe is REFUSED: the refusal
// screen, no task, nothing powered, and every phase says so on serial. A boot from the TYPED TABLE
// refuses every AUTON (its front may be the other end from the record's) and still drives. A
// program error caught on the auton task stops the robot safely and refuses every later auton
// until the program restarts; driver control still runs.
//
// ═══ THE LOGS (B5(1), ND5; the T4 brief's N5 (a) with Decisions A and B) ═════════════════════════
// Serial: the banner, the boot's decision lines, the session header, the three boot lines, every
// auton the registry DROPPED, every absent mechanism, where the log copy is, the claw lines, every
// competition-status CHANGE (millis and the decoded bits, from the auton task's idle loop -- one
// tick of latency), one line on entry to and on normal return from each of
// competition_initialize(), disabled(), autonomous() and opcontrol(), and each request's lines;
// runAuton's own [COMP] lines, the guard's [SEQ] lines and the driver loop's protection lines ride
// the provider's sink.
// The SD card holds TWO files. (1) THE RUN LOG (src/common/auton_run_log.hpp's file, APPENDED --
// the Auton Tester's too): the boot lines and each request's lines, written by this program only
// AFTER the request's terminal word. (2) THE LOG COPY, /usd/comp_log.txt, APPENDED across boots
// (T4 Part 3; the team lead's B (a): a reboot mid-match must not erase that match's log): the
// provider's own copy of every log() line and summary that reaches its sink from the provider's
// construction on -- never a per-tick record (A (a)), and never a line per slow tick either: a
// loop that overruns logs LOOP_OVERRUN once per episode (diag/loop_monitor.hpp, the T4-3 review)
// -- so a practice match with no laptop still leaves the dead ports, every cut, the peak currents,
// and a boot refused after the provider was built (refuseBuilt). That copy is written WHEN each
// line is logged: during an auton (runAuton's and the guard's event lines, the scheduler's fault
// lines) and during driver control (the protections' events: a port dropped, a cut, a peak) a
// line is one card write and one flush in the tick it is logged (hal::BlockCharSink). What that
// costs on the brain is UNMEASURED (A4 register HA-60; the T4 brief's re-check item 2): Station K
// reads it.
// MORE THAN ONE TASK writes the provider's sink: the auton task (a status change, a request's
// lines), the PROS phase tasks (each phase function's entry and return lines) and the opcontrol
// task's driver loop (the protections' events) -- so around a mode change two can log at once.
// The CARD COPY has one writer at a time (diag/log_copy_sink.hpp's guard, the T4-3 review): a line
// that finds another task mid-copy stays on serial, is left off the card and counted, and the
// card's next line says how many it missed there -- no task waits on another's card write. A task
// PROS deletes while it is mid-copy (HA-161 (a), reasoned) never releases the guard, so the card's
// copy ENDS there for the rest of the boot, every later line counted and on serial -- the chosen
// failure (log_copy_sink.hpp, "A writer that never comes back"); this program's opcontrol() logs
// two lines right after its loop sees a disable, the moment PROS deletes the task.
// SERIAL has had the same exposure since COMP-b, unguarded:
// whether PROS's C library serializes two tasks' writes to stdout is measured nowhere here and
// stated in no A4 register entry (left for the coordinator: an entry, or a lock).
//
// ═══ ON THE BENCH: THE CONTROLLER'S TIMED RUN ═════════════════════════════════════════════
// The team owns no competition switch, so the bench's pretend match is the controller's Timed Run.
// A program started that way comes up DISABLED (A4 register HA-128, measured); what the Timed Run
// does next -- a disabled window, how long autonomous is enabled, what follows -- is UNMEASURED
// (A4 register HA-162), and worksheet Station K measures it from this program's own phase and
// status lines before anything depends on it. Nothing here depends on its numbers: every phase is
// driven by what VEXos reports, and the periods are the game manual's (src/team/autons.hpp).
//
// Compiled in every build -- seven since T4 Part 3 (every .cpp under src/ is); nothing here reads a
// build switch, no beacon literal lives here, and the state is one function-local static of EMPTY
// optionals, so --gc-sections drops all of its CODE from every build but tank-competition. Not
// always its TEXT: a file's string literals share one section, which the linker keeps whole when it
// keeps this file's copy of a shared inline function -- measured at COMP-b's review, 48 of these
// strings rode in the tank-library (Teleop) package and 26 in the X-drive's, as Teleop's rode in
// the X-drive's before (Teleop retired at T4 Part 3; not re-measured since). Words only, never
// code that runs; no beacon is among them. ASCII only in every runtime
// string; every screen string goes through the shared painters (printText); only a literal "%s"
// reaches a PROS print. No slot number appears in any string here (the documents name the slot).
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station K is its first run).

#include "programs/competition_program.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <optional>
#include <span>
#include <string_view>

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "common/auton_run_log.hpp"
#include "common/boot_source.hpp"
#include "common/driver_panel.hpp"
#include "common/dropped_autons.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/guarded_run.hpp"
#include "common/log_copy_line.hpp"
#include "common/screen_paint.hpp"
#include "common/screen_text.hpp"
#include "common/screen_touch.hpp"
#include "common/state_screens.hpp"
#include "programs/competition_run.hpp"
#include "programs/competition_screen.hpp"
#include "programs/competition_watch.hpp"
#include "robots/mechanism_table.hpp"
#include "team/autons.hpp"
#include "team/robot_two_driver.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/block_char_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

/// Every serial line of this program carries this prefix (competition_run.hpp's).
constexpr const char* kTag = kCompetitionTag;
/// The upload check: the time this file was compiled, on the driver page and the refusal bar.
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;
/// The auton task's idle wait and the phase functions' wait tick: one brain tick (RC3).
constexpr std::uint32_t kTickMs = 10;
/// THE LOG COPY's file on the card (header, THE LOGS): BARE -- the provider's ProsBlockSink owns
/// the /usd/ prefix -- and APPENDED across boots (the team lead's B (a)).
constexpr const char* kLogCopyFile = "comp_log.txt";
/// The controller's facts are read on the driver loop's first tick and then every 5 ticks (50 ms);
/// a changed line is written through src/common/driver_panel.hpp's ControllerLineWriter -- one line
/// per write, at most one write per 0.1 s, line 2 first.
constexpr int kLcdEveryTicks = 5;

static_assert(kGuardedTickSeconds * 1000.0 == static_cast<double>(hal::pros::ProsTickPacer::kTickMs)
                  && kTickMs == hal::pros::ProsTickPacer::kTickMs,
              "the chain proof's idle tick and the tasks' waits are one motion tick");
static_assert(calibration::kSmallCols == ui::kSmallCols && calibration::kMediumCols == ui::kMediumCols
                  && calibration::kLargeCols == ui::kLargeCols,
              "calibration:: and ui:: text budgets must be the same numbers (the shared painter "
              "maps one onto the other)");

// ═══ THE REFUSAL SCREENS' fixed words (checked here, where they are written) ═══════════════════
constexpr std::string_view kNothingPowered = "Nothing is powered. The full text is on the serial log.";
constexpr std::string_view kDriverStillRuns =
    "Driver control still runs. Restart the program to run autons.";
constexpr std::string_view kTableFooter = "Driver control still runs.";
static_assert(refusalBarFits(kCompetitionName)
                  && calibration::screenText(kBuildTimeText, calibration::kSmallCols),
              "a refusal-screen literal does not fit its budget");
static_assert(refusalFooterFits(kNothingPowered) && refusalFooterFits(kDriverStillRuns)
                  && refusalFooterFits(kTableFooter),
              "a refusal footer must fit its two SMALL rows");

// ═══ PROS's READS: one line each, the meaning in host-tested code ═════════════════════════════
/// competition_get_status() -- the watch's input, the status log's, and the driver loop's end.
std::uint8_t readStatus() { return pros::c::competition_get_status(); }
/// One read of the touch screen, reduced by src/common/screen_touch.hpp's screenTouchFrom().
ScreenTouch readTouch() {
    return screenTouchFrom(pros::c::screen_touch_status(), static_cast<int>(pros::E_TOUCH_ERROR));
}

/// The watch's status source on the robot.
class ProsCompetitionStatus final : public ICompetitionStatus {
public:
    [[nodiscard]] std::uint8_t read() override { return readStatus(); }
};

// ═══ PROGRAM STATE: one function-local static of EMPTY optionals ═════════════════════════════
// ("shulib Drive"'s rule, and "shulib Teleop"'s before it: nothing non-movable at namespace scope,
// nothing whose constructor can throw in the static itself.) Declaration order is the borrow order: the provider, the status
// source, the pacer chain from the inside out (watch, guard), the Robot -- whose Chassis keeps the
// guard as its pacer for the Robot's whole life -- the driver's bindings over it, the registry and
// the menu. The driver file lives HERE for the program's life (its loop holds the bindings by
// address; PROS deletes the opcontrol task on a disable), and so does the last request's record.
struct CompetitionState {
    BootOutcome boot{};
    CompetitionFlags flags{};
    AutonHandshake handshake{};
    ScreenOwnerCell owner{};
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<ProsCompetitionStatus> status{};
    std::optional<CompetitionWatch> watch{};
    std::optional<sequence::RunGuard> guard{};
    std::optional<robot::Robot> robot{};
    RobotTwoDriver driver{};
    std::optional<competition::AutonRegistry> registry{};
    std::optional<competition::AutonMenu> menu{};
    std::optional<hal::pros::ProsLineDisplay> lcd{};
    std::array<hal::IMechanism*, config::kMaxMechanisms> present{};
    std::size_t presentCount = 0;
    std::optional<hal::pros::ProsBlockSink> sdBlock{};
    std::optional<hal::BlockCharSink> sdChars{};
    std::optional<diag::TermSink> sdLog{};
    CompetitionRunRecord last{};  // the last request: here, never on a task stack
    std::optional<pros::Task> task{};
    DriverLoopPresence presence{};  // the opcontrol task's claim on the robot, and its heartbeat
    // The opcontrol task's own: what the controller shows, and when it was last written.
    ControllerLineWriter lcdWriter{};
    // The auton task's own (only it reads or writes these):
    std::uint32_t served = 0;         // the last request it served
    int serves = 0;                   // how many it has served
    std::uint8_t lastStatus = 0xFF;   // the last competition status it logged
    bool idleErrorSaid = false;       // one Error line for a throw outside a request
    char errorWhat[kCompetitionErrorBytes]{};  // what the last program error said (the fault page)
    char portMap[diag::kMaxPortMapBytes]{};
    char refusalHeadline[64]{};
    char refusalWhy[192]{};
};

CompetitionState& state() {
    static CompetitionState s;
    return s;
}

// ═══ THE LOG ═════════════════════════════════════════════════════════════════════════════
/// One line to serial -- through the provider's sink once the boot built it, else printf -- and,
/// when `alsoSd` and the run log is open, to the card. Only initialize() and the auton task pass
/// `alsoSd` (one writer on the card at a time).
void logLine(hal::LogLevel level, const char* text, bool alsoSd = false) {
    CompetitionState& st = state();
    if (st.devices.has_value()) {
        st.devices->telemetry().log(level, kTag, text);
    } else {
        std::printf("[%s] %s\n", kTag, text);
        std::fflush(stdout);
    }
    if (alsoSd && st.sdLog.has_value()) {
        st.sdLog->log(level, kTag, text);
    }
}

/// B5(1): one line on entry to, or return from, a PROS phase function -- the millis and the
/// decoded status bits (Station K reads them in serial order: the NEXT phase function's entry line
/// marks a mode change -- A4 register HA-161's settle).
void logPhase(const char* what, const char* when, const char* note) {
    char bits[64];
    competitionBits(readStatus(), bits, sizeof bits);
    char line[kCompetitionLineBytes];
    (void)competitionPhaseLine(line, sizeof line, what, when, pros::millis(), bits, note);
    logLine(hal::LogLevel::Info, line);
}

// ═══ PAINTING (every string through the shared painters, src/common/screen_paint.hpp) ════════════
/// One of this program's pages, WHOLE: the bar, its two texts, the big word, the lines.
void paintViewWhole(const CompetitionView& v) {
    clearPanel();
    paintBar(v.bar, v.header);
    paintLine(v.barRight, toneRgb(v.bar));
    if (v.hasBig) {
        paintLine(v.big, kPanelBg);
    }
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        paintLine(v.lines[i], kPanelBg);
    }
}

/// Only the lines that changed between `was` and `now` (the same page; a new bar colour is a
/// whole repaint).
void paintViewChanges(const CompetitionView& was, const CompetitionView& now) {
    if (was.bar != now.bar || was.hasBig != now.hasBig) {
        paintViewWhole(now);
        return;
    }
    if (lineDiffers(was.header, now.header)) {
        paintLine(now.header, toneRgb(now.bar));
    }
    if (lineDiffers(was.barRight, now.barRight)) {
        paintLine(now.barRight, toneRgb(now.bar));
    }
    if (now.hasBig && lineDiffers(was.big, now.big)) {
        paintLine(now.big, kPanelBg);
    }
    for (std::size_t i = 0; i < now.lineCount; ++i) {
        if (i >= was.lineCount || lineDiffers(was.lines[i], now.lines[i])) {
            paintLine(now.lines[i], kPanelBg);
        }
    }
}

/// The boot's refusal: the shared refusal screen and, unless `serialLine` is false (a refusal
/// after the provider was built is said through its sink first, so the log copy holds it), its
/// serial line (nothing is powered).
void refuse(const char* headline, const char* why, bool serialLine = true) {
    CompetitionState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%.63s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%.191s", why);
    paintRefusalScreen(RefusalScreen{.tag = kTag,
                                     .barWord = kCompetitionName.data(),
                                     .buildTime = kBuildTimeText.data(),
                                     .headline = st.refusalHeadline,
                                     .why = st.refusalWhy,
                                     .footer = kNothingPowered,
                                     .logToSerial = serialLine});
}

/// The menu phase's page when autons are REFUSED (ND1's program error, or ND2's typed-table
/// boot): the shared refusal screen, with no serial line -- the refusal was logged where it was
/// decided, and this page is repainted at every take of the screen. A program error's reason is
/// what the auton task recorded where it caught it: a request that threw (the robot stopped), its
/// report, or the idle half (the robot not touched) -- runAutonRequest() and idleProgramError().
void paintAutonsRefused() {
    CompetitionState& st = state();
    const bool error = st.flags.programError.load();
    paintRefusalScreen(RefusalScreen{
        .tag = kTag,
        .barWord = kCompetitionName.data(),
        .buildTime = kBuildTimeText.data(),
        .headline = error ? "AUTONS REFUSED: a program error on the auton task"
                          : "AUTONS REFUSED: this boot used the TYPED TABLE",
        .why = error ? st.errorWhat
                     : "The typed chassis table's front may be the OTHER end from the calibration "
                       "record's, so an auton could drive the wrong way. Run shulib Calibrate, then "
                       "restart.",
        .footer = error ? kDriverStillRuns : kTableFooter,
        .logToSerial = false});
}

// ═══ THE AUTON TASK'S SCREEN: the menu phase, the status, the result ═══════════════════════════
/// The status page before runAuton (RC3) -- painted only while the auton task holds the screen.
class StatusObserver final : public ICompetitionRunObserver {
public:
    void starting(const CompetitionRunRecord& rec) override {
        CompetitionState& st = state();
        if (st.owner.holds(ScreenOwner::AutonStatus)) {
            paintViewWhole(competitionStatusView(rec, *st.registry, kBuildTimeText));
        }
    }
};

/// The result page after the terminal word -- skipped when another owner took the screen (then the
/// result is only logged: RC3).
void paintResult() {
    CompetitionState& st = state();
    if (st.owner.holds(ScreenOwner::AutonStatus)) {
        paintViewWhole(competitionResultView(st.last, *st.registry, kBuildTimeText));
    }
}

/// What the menu phase remembers between the auton task's ticks.
struct MenuPainter {
    OwnerTracker tracker{};
    competition::MenuView drawn{};  // the selector frame ON THE SCREEN (when `menuUp`)
    bool menuUp = false;            // the selector -- not a refusal page -- was painted
    double paintedAt = 0.0;
};

/// The menu phase, once per idle tick (RC3 (ii)); what each read may do is competition_screen.hpp's
/// menuPhaseStep() (host-tested). Every release is read every tick, so the menu's edge sees each
/// exactly once: a release made while the selector is NOT up (another owner, a refusal page, or the
/// moment the screen is taken back) is consumed against an EMPTY frame and acts on nothing -- a tap
/// made during driver control can never become a choice later.
void paintMenuPhase(MenuPainter& p) {
    CompetitionState& st = state();
    const double now = st.devices->clock().now().value();
    const ScreenTouch touch = readTouch();
    const PaintAction action = p.tracker.next(st.owner.snapshot(), ScreenOwner::Menu, now);
    const bool refused = autonsRefused(st.flags);
    const MenuPhaseStep step = menuPhaseStep(action, refused, p.menuUp);
    p.menuUp = step.menuUpAfter;
    if (step.consumeBlind) {
        (void)st.menu->touch(touch.sample, competition::MenuView{});  // consumed, never acted on
    }
    if (step.paintRefused) {
        paintAutonsRefused();
    }
    if (step.paintMenuWhole) {
        p.drawn = st.menu->view(now);
        paintMenuFrame(p.drawn);
        p.paintedAt = now;
    }
    if (!step.actOnDrawn) {
        return;
    }
    // The selector is up and still ours: this read against the frame ON THE SCREEN.
    const int before = st.menu->choice();
    (void)st.menu->touch(touch.sample, p.drawn);
    if (st.menu->choice() != before) {
        char line[kCompetitionLineBytes];
        const competition::AutonEntry* e = st.menu->chosenEntry();
        std::snprintf(line, sizeof line, "menu: the auton is now '%.29s'",
                      e != nullptr ? e->name : "NONE");
        logLine(hal::LogLevel::Info, line);
    }
    const competition::MenuView next = st.menu->view(now);
    if (!sameMenuButtons(next, p.drawn)) {
        paintMenuFrame(next);
        p.drawn = next;
        p.paintedAt = now;
    } else if (now - p.paintedAt >= kLiveRepaintSeconds) {
        paintMenuText(p.drawn, next);
        p.drawn = next;
        p.paintedAt = now;
    }
}

/// B5(1): every competition-status CHANGE, from the auton task's idle loop (one tick of latency).
void logStatusChange() {
    CompetitionState& st = state();
    const std::uint8_t status = readStatus();
    if (status == st.lastStatus) {
        return;
    }
    char bits[64];
    char line[kCompetitionLineBytes];
    std::snprintf(line, sizeof line, "competition status %.44s at %lu ms",
                  competitionBits(status, bits, sizeof bits),
                  static_cast<unsigned long>(pros::millis()));
    logLine(hal::LogLevel::Info, line);
    st.lastStatus = status;
}

/// A throw caught on the auton task OUTSIDE a request (the idle half: the status log, the menu,
/// the wait for the driver loop): the robot was not the auton task's, so nothing is commanded --
/// the program error is latched (ND1: later autons refused; the menu phase shows why) and said once.
void idleProgramError(const char* what) noexcept {
    CompetitionState& st = state();
    st.flags.programError.store(true);
    std::snprintf(st.errorWhat, sizeof st.errorWhat,
                  "the auton task threw outside a request: %.80s -- the robot was not touched",
                  what != nullptr ? what : "");
    if (st.idleErrorSaid) {
        return;
    }
    st.idleErrorSaid = true;
    try {
        char line[kCompetitionLineBytes];
        std::snprintf(line, sizeof line,
                      "auton task: PROGRAM ERROR outside a request: %.80s -- the robot was not "
                      "touched; later autons REFUSED until a restart",
                      what != nullptr ? what : "");
        logLine(hal::LogLevel::Error, line);
    } catch (...) {  // NOLINT(bugprone-empty-catch): nothing may leave the auton task
    }
}

// ═══ THE AUTON TASK: every request, the status log and the menu, for the program's whole life ════
void autonTask(void* /*unused*/) {
    CompetitionState& st = state();
    CompetitionParts parts{st.robot->chassis(),
                           *st.guard,
                           *st.watch,
                           *st.menu,
                           *st.registry,
                           st.devices->telemetry(),
                           std::span<hal::IMechanism* const>{st.present.data(), st.presentCount},
                           st.handshake,
                           st.flags};
    StatusObserver observer;
    MenuPainter menu;
    DriverLoopWatcher loop;
    const auto now = [&st] { return st.devices->clock().now().value(); };
    for (;;) {
        try {
            // ONE tick of idle, or at once when autonomous() notifies. Never a Chassis tick (RC3).
            (void)pros::c::task_notify_take(true, kTickMs);
            loop.observe(st.presence, now());
            std::uint32_t request = 0;
            double requestedAt = 0.0;
            if (st.handshake.pending(st.served, request, requestedAt)) {
                // The driver loop LEAVES first (competition_run.hpp's banner); then the newest request.
                const DriverLoopWait waited =
                    waitForDriverLoop(st.presence, loop, now, [] { pros::delay(kTickMs); });
                (void)st.handshake.pending(st.served, request, requestedAt);
                st.served = request;
                runAutonRequest(parts, st.last, request, requestedAt, ++st.serves, observer,
                                st.sdLog.has_value() ? &*st.sdLog : nullptr, st.errorWhat, waited);
                paintResult();
            }
            logStatusChange();
            paintMenuPhase(menu);
        } catch (const std::exception& e) {
            idleProgramError(e.what());  // its own words, never "unknown"
        } catch (...) {
            idleProgramError("an unknown exception");
        }
    }
}

// ═══ THE DRIVER PAGE and the controller's lines (the opcontrol task, after the handover) ═══════════
/// The cut's length, from the config the driver file's loop runs with (its cutTicks times its
/// tick) -- what the PROTECTION row says a cut lasts; never retyped.
constexpr double kCutSeconds =
    RobotTwoDriver::kLoop.protection.cutTicks * RobotTwoDriver::kLoop.tickPeriod.value();

class DriverPanel {
public:
    DriverPanel(const robot::Robot& robot, const CompetitionState& st) noexcept
        : robot_{robot}, st_{st} {}

    /// runDriverControl's per-tick hook (the driver file's run()), with the FOURTH argument: the
    /// library's drive protections' state this tick (teleop/drive_protection.hpp) -- the
    /// PROTECTION row and the controller's REFUSED / CUT / dead words (T4 Part 2b).
    void operator()(const math::ChassisSpeeds& /*command*/, bool connected,
                    const teleop::DriveShaperState& /*shaper*/,
                    const teleop::DriveProtectionState& protection) {
        ++tick_;
        const double now = robot_.context().clock().now().value();
        const PaintAction action = tracker_.next(st_.owner.snapshot(), ScreenOwner::Driver, now);
        if (action == PaintAction::Whole) {
            painted_ = view(connected, protection);
            paintViewWhole(painted_);
            paintedAt_ = now;
        } else if (action == PaintAction::Changes && now - paintedAt_ >= kLiveRepaintSeconds) {
            const CompetitionView next = view(connected, protection);
            paintViewChanges(painted_, next);
            painted_ = next;
            paintedAt_ = now;
        }
        if (tick_ % kLcdEveryTicks == 1) {  // the FIRST tick too: AUTON ENDING must not linger
            refreshLcd(connected, protection, now);
        }
    }

private:
    [[nodiscard]] CompetitionView view(bool connected,
                                       const teleop::DriveProtectionState& protection) {
        const diag::FaultLatch& f = robot_.faults();
        faultRow_ = faultRowText(f.hasFault() ? f.faultCount() : 0, f.firstFault(), f.lastFault());
        clawRow_ = clawRowText(kMechanisms, robot_,
                               st_.driver.bindings().anyReleasedLeftoverClaim());
        CompetitionDriverFacts facts{};
        facts.protection = protectionRowText(protection, kCutSeconds);
        facts.auton = competitionAutonRow(st_.last, *st_.registry, st_.flags.programError.load(),
                                          st_.flags.bootFromRecord);
        facts.controller = controllerRowText(connected);
        facts.faultRow = faultRow_.text;
        facts.faultRed = f.hasFault();
        facts.clawRow = clawRow_.text;
        facts.clawAmber =
            anyClawAbsent(robot_) || st_.driver.bindings().anyReleasedLeftoverClaim();
        facts.buildStamp = kBuildTimeText;
        return competitionDriverView(facts);
    }

    void refreshLcd(bool connected, const teleop::DriveProtectionState& protection, double now) {
        ControllerFacts f{};
        f.batteryVolts = robot_.context().battery().voltage().value();
        f.headingDegrees = robot_.localizer().pose().heading().radians() * 180.0 / math::Angle::kPi;
        f.hasFault = robot_.faults().hasFault();
        f.lastFault = robot_.faults().lastFault();
        f.connected = connected;
        f.degraded = st_.boot.verdict.decision == calibration::BootDecision::Degraded;
        f.clawAbsent = anyClawAbsent(robot_);
        addProtectionFacts(f, protection);  // REFUSED, CUT, DEGRADED mid-run, the dead ports
        const ControllerScreen want = controllerScreenLines(f);
        const int row = state().lcdWriter.next(want, now);  // ONE line, line 2 first, >= 0.1 s apart
        if (row >= 0) {
            state().lcd->setLine(row, want.line[row]);
        }
    }

    const robot::Robot& robot_;
    const CompetitionState& st_;
    OwnerTracker tracker_{};
    CompetitionView painted_{};
    double paintedAt_ = 0.0;
    int tick_ = 0;
    ClawRow faultRow_{};
    ClawRow clawRow_{};
};

/// B2's constant passed before the terminal word: the screen to the driver page's owner, AUTON
/// ENDING painted, the controller told, and ONE log line naming the cause.
void sayAutonEnding(double waitedSeconds) {
    CompetitionState& st = state();
    st.owner.handTo(ScreenOwner::Driver);
    const competition::AutonEntry* running = st.menu->chosenEntry();
    const char* name = running != nullptr ? running->name : "NONE";
    paintViewWhole(competitionEndingView(name, waitedSeconds, kBuildTimeText));
    ControllerFacts f{};
    f.autonEnding = true;  // line 2 only: the robot is still the auton task's -- nothing is read
    const ControllerScreen ending = controllerScreenLines(f);
    st.lcd->setLine(2, ending.line[2]);
    st.lcdWriter.wrote(2, ending.line[2], st.devices->clock().now().value());  // the gap from it
    char line[kCompetitionLineBytes];
    std::snprintf(line, sizeof line,
                  "opcontrol(): AUTON ENDING -- '%.29s' has not handed the robot back %.2f s after "
                  "driver control started (a wait the guard cannot cut?); driver control waits",
                  name, waitedSeconds);
    logLine(hal::LogLevel::Warn, line);
}

}  // namespace

// ═══ THE ENTRY POINTS (src/main.cpp forwards to these) ═══════════════════════════════════════════

void competitionInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    CompetitionState& st = state();
    std::printf("\n[%s] booted: shulib COMPETITION build [%s] -- the auton chosen on the brain's\n"
                "[%s] menu runs in autonomous() on the program's own task (competition::runAuton,\n"
                "[%s] the Auton Tester's call); a disable or the end of autonomous ends it at once;\n"
                "[%s] driver control runs the team's driver file. NOT RUN ON A ROBOT YET.\n"
                "[%s] competition status at boot: %s\n",
                kTag, variantBeacon, kTag, kTag, kTag, kTag, competitionStatusBits);
    std::fflush(stdout);

    st.boot = decideBootSource(kTag);
    if (!st.boot.ok) {
        refuse(st.boot.headline, st.boot.why);  // no task: nothing paints over it (RC3, C7)
        return;
    }
    // Everything is built at the graph boundary, inside ONE try (B8); any refusal un-builds it all,
    // the driver's bindings first -- none may outlive the mechanisms it points at (B11).
    const auto unbuild = [&st] {
        st.driver.unbind();
        st.menu.reset();
        st.registry.reset();
        st.robot.reset();
        st.guard.reset();
        st.watch.reset();
        st.status.reset();
        st.devices.reset();
    };
    // A refusal decided once the provider exists is said through its sink FIRST -- serial and the
    // log copy, so the card's comp_log.txt holds why this boot refused (log_copy_line.hpp's
    // logBootRefusal(); the T4-3 review) -- then everything is un-built and the screen painted
    // with no serial line of its own. With NO provider (a throw inside its own construction)
    // nothing was said, so the painter says it on serial (the T4-3 verification).
    const auto refuseBuilt = [&st, &unbuild](const char* headline, const char* why) {
        const bool logged = st.devices.has_value();
        if (logged) {
            logBootRefusal(st.devices->telemetry(), kTag, headline, why);
        }
        unbuild();
        refuse(headline, why, !logged);
    };
    try {
        st.devices.emplace(kLogCopyFile, hal::pros::ProsFileMode::Append);  // + the log copy
        st.status.emplace();
        st.watch.emplace(st.devices->pacer(), *st.status);
        st.guard.emplace(*st.watch);
        st.watch->attachGuard(*st.guard);
        robot::BuildOptions opts = st.boot.built.buildOptions();
        opts.pacer = &*st.guard;  // BORROWED for the Robot's whole life (robot.hpp)
        const robot::BuildVerdict verdict = robot::validate(st.boot.built.config, *st.devices, opts);
        if (!verdict.ok) {
            refuseBuilt("the library graph REFUSED to build", verdict.why);
            return;
        }
        st.robot.emplace(st.boot.built.config, *st.devices, opts);
        // The team's driver file (B11): its buttons bound over this Robot, inside the try.
        char why[192] = "";
        if (!st.driver.bind(*st.robot, why, sizeof why)) {
            refuseBuilt("the driver's controls REFUSED", why);
            return;
        }
        // The team's autons. An entry the registry refuses is DROPPED -- never on the menu, named
        // there and in the log -- and the boot goes on: the team lead's ruling of 2026-10-02 ("drop
        // just that auton"; an auton bug must not cost the match's driver control).
        st.registry.emplace(kAutonPeriods, competition::OnRefusal::DropEntry);
        registerAutons(*st.registry);
        // The guard safes the Robot's PRESENT mechanisms (B9).
        for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
            hal::IMechanism* m = st.robot->builtMechanismAt(i);
            if (m != nullptr) {
                st.present[st.presentCount++] = m;
            }
        }
        st.menu.emplace(*st.registry, readTouch().sample.releaseCount);
        // THE CHAIN PROOF (B30): one idle tick must pace the guard AND the watch, and the watch
        // must be attached to the guard, or no disable could end an auton. Refused, not warned.
        if (!pacerChainProven(st.robot->chassis(), *st.guard, *st.watch)) {
            refuseBuilt("the competition program's pacer chain is WRONG",
                   "one idle tick did not pace the run guard and the competition watch, or the "
                   "watch is not attached to the guard -- a disable could not end an auton; this is "
                   "a build error, not a robot fault");
            return;
        }
    } catch (const std::exception& e) {
        refuseBuilt("the competition program REFUSED to start", e.what());
        return;
    }
    st.flags.bootFromRecord = st.boot.source == calibration::BootSource::UseRecord;  // ND2
    attachFaultLatch(&st.robot->faults());  // BEFORE the auton task exists (B8, B19)
    st.lcd.emplace();

    // The SD run log (src/common/auton_run_log.hpp's file, APPENDED -- the Auton Tester's): no
    // card, or a log that will not open, is serial only, never a refusal.
    st.sdBlock.emplace(kAutonRunLogFile, hal::pros::ProsFileMode::Append);
    if (st.sdBlock->isOpen()) {
        st.sdChars.emplace(*st.sdBlock);
        st.sdLog.emplace(st.devices->clock(), *st.sdChars);
    }
    // The session header (the S18.5 provenance block, serial): the hash LOUD when missing, the
    // port map generated from the config.
    st.robot->portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(competition: the menu's choice at autonomous())";
    session.alliance = "";
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(st.devices->telemetry(), session,
                            st.robot->context().battery().voltage());
    // The boot's three lines (host-tested at every field's worst case): the build hash is a
    // string_view, bounded there -- NEVER handed to %s.
    const AutonRunLog sd = !st.sdLog.has_value() ? AutonRunLog::Off
                           : st.sdBlock->appendOpen() == hal::pros::AppendOpen::Created
                               ? AutonRunLog::Created
                               : AutonRunLog::Appended;
    const CompetitionBootLines boot = competitionBootLines(
        variantBeacon, diag::compiledBuildHash(), bootSourceWord(st.boot.source),
        st.flags.bootFromRecord, static_cast<int>(st.registry->size()),
        static_cast<int>(st.presentCount), static_cast<int>(st.robot->mechanismCount()), sd,
        st.sdLog.has_value() ? st.sdBlock->path() : "");
    for (const char* bootLine : boot.line) {
        logLine(hal::LogLevel::Info, bootLine, true);
    }
    char line[kCompetitionLineBytes];
    for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
        if (st.robot->builtMechanismAt(i) == nullptr) {
            std::snprintf(line, sizeof line,
                          "mechanism '%.23s' is ABSENT -- the run guard cannot safe it: %.120s",
                          st.robot->config().mechanisms[i].name, st.robot->mechanismAbsence(i));
            logLine(hal::LogLevel::Warn, line, true);
        }
    }
    // Every auton the registry DROPPED (the team lead's ruling of 2026-10-02), by name, on serial
    // and the SD run log -- in the words src/common/dropped_autons.hpp makes for both programs that
    // register the team's autons; the menu names each too, in red, while the choice is open.
    sayDroppedAutons(*st.registry, true, [](const char* dropped) {
        logLine(hal::LogLevel::Warn, dropped, true);
    });
    // THE LOG COPY (header, THE LOGS): where it is, or that it did not open and why (no card, or
    // a card that would not take the file) -- ONCE.
    {
        const LogCopyOpen how = logCopyOpenFrom(
            st.devices->serialCopyOpen(), pros::c::usd_is_installed() != 0, true,
            st.devices->serialCopyAppendOpen() == hal::pros::AppendOpen::Created);
        const LogCopyLine copy = logCopyBootLine(how, st.devices->serialCopyPath(),
                                                 st.devices->serialCopyDroppedLines());
        logLine(copy.warn ? hal::LogLevel::Warn : hal::LogLevel::Info, copy.text);
    }
    // The claws, one line each (src/common/driver_panel.hpp's words, from the table AND the driver
    // file's controls -- the same lines "shulib Drive" prints; worksheet J.3.1 and K quote them),
    // printed whole: an ABSENT line can pass TermSink's 200 bytes.
    for (std::size_t i = 0; i < motorEntryCount(kMechanisms); ++i) {
        std::printf("[%s] %s\n", kTag,
                    motorClawBootLine(kMechanisms, RobotTwoDriver::kHolds, *st.robot, i).text);
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(kMechanisms); ++j) {
        std::printf("[%s] %s\n", kTag,
                    pneumaticClawBootLine(kMechanisms, RobotTwoDriver::kToggles, *st.robot, j).text);
    }
    std::fflush(stdout);

    st.owner.handTo(ScreenOwner::Menu);  // the auton task paints the menu from its first tick
    st.task.emplace(&autonTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                    "auton task");
}

void competitionDisabled() {
    CompetitionState& st = state();
    logPhase("disabled()", "entry", "");
    st.owner.handTo(ScreenOwner::Menu);  // RC3: the auton task paints the menu; this paints nothing
    logPhase("disabled()", "normal return", "the screen to the menu");
}

void competitionConnected() {
    CompetitionState& st = state();
    logPhase("competition_initialize()", "entry", "");
    st.owner.handTo(ScreenOwner::Menu);
    logPhase("competition_initialize()", "normal return", "the screen to the menu");
}

void competitionAutonomous() {
    CompetitionState& st = state();
    logPhase("autonomous()", "entry", "");
    if (!st.task.has_value()) {
        logPhase("autonomous()", "normal return", "the boot was REFUSED: no auton runs");
        return;
    }
    st.owner.handTo(ScreenOwner::AutonStatus);  // RC3: the auton task paints the status
    const std::uint32_t request = st.handshake.request(st.devices->clock().now().value());
    st.task->notify();
    while (!st.handshake.isFinished(request)) {
        pros::delay(kTickMs);  // an atomic checked every tick, never task_join (B19)
    }
    // HA-161 (a): if PROS deletes this task at a mode change, this line never appears.
    logPhase("autonomous()", "normal return", "its request FINISHED");
}

void runCompetition() {
    CompetitionState& st = state();
    logPhase("opcontrol()", "entry", "");
    if (!st.robot.has_value()) {
        logPhase("opcontrol()", "normal return", "the boot was REFUSED: nothing to drive");
        return;
    }
    st.lcdWriter.forget();  // this period writes every line again (one at a time, line 2 first)
    // B2: no task but the auton task touches the robot until its terminal word -- waited for with
    // NO upper bound, checking every tick; AUTON ENDING after kAutonEndingAfterSeconds. Then the
    // robot is CLAIMED for driver control -- refused while a request waits, and then waited again.
    double waited = 0.0;
    do {
        waited += waitForAutonTask(
            st.handshake, [&st] { return st.devices->clock().now().value(); },
            [] { pros::delay(kTickMs); }, [](double w) { sayAutonEnding(w); });
    } while (!st.presence.enter(st.handshake));
    st.owner.handTo(ScreenOwner::Driver);  // the driver page, whole, from the first tick
    char line[kCompetitionLineBytes];
    std::snprintf(line, sizeof line,
                  "opcontrol(): driver control starts -- the driver file, after %.3f s waiting for "
                  "the auton task",
                  waited);
    logLine(hal::LogLevel::Info, line);
    robot::Robot& r = *st.robot;
    DriverPanel panel{r, st};
    // The team's driver file (B11): its loop until a disable, the autonomous bit, or a waiting
    // auton request -- beating once per tick for the auton task. Its tick is the provider's pacer
    // (nothing in driver control is the guard's).
    st.driver.run(r, st.devices->pacer(),
                  [&st] {
                      st.presence.beat();
                      return driverControlMustEnd(readStatus(), st.handshake.busy());
                  },
                  panel);
    st.presence.leave();  // the robot is no longer driver control's: the auton task may take it
    st.owner.handOver(ScreenOwner::Driver, ScreenOwner::Menu);  // unless a phase took it first
    std::snprintf(line, sizeof line,
                  "opcontrol(): driver control ended -- 0 V + Brake, each claw pivot safe, the "
                  "pneumatic claw kept; %lu screen prints so far",
                  static_cast<unsigned long>(g_screenPrints));
    logLine(hal::LogLevel::Info, line);
    logPhase("opcontrol()", "normal return", "the screen back to the menu");
}

}  // namespace shulib::bench
