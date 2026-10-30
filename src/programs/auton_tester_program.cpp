// auton_tester_program.cpp -- "shulib Auton Tester" (COMP-t, 2026-09-28): pick one of the team's
// autons on the brain's screen, touch RUN, step back during a 3 s countdown, and watch it run --
// with no competition switch. ANY controller button or a touch of the screen aborts at once.
//
// ═══ THE ONE PROMISE ════════════════════════════════════════════════════════════════
// The auton runs through competition::runAuton over the team's registry (src/team/autons.cpp), on
// a Chassis whose pacer is a sequence::RunGuard -- the call the competition program's autonomous()
// makes, on the same entry, with the same schedule, the same start-pose re-seat and the same
// guaranteed end action. The sequence around it (arm the abort, the countdown, the quiet tick, the
// run, the record) is src/programs/auton_tester_run.hpp's runTesterOnce(), PROS-free and run on the
// host test plant; this file only builds the objects, paints, and calls it.
//
// ═══ ONE TASK OWNS THE ROBOT ════════════════════════════════════════════════════════
// initialize() builds everything and starts the tester's task; from then on ONLY that task touches
// the Robot, the Chassis, the guard, the watch, the screen and the SD log. src/main.cpp's disabled(),
// competition_initialize(), autonomous() and opcontrol() do nothing for this program: PROS deletes
// those tasks on a mode change (A4 register HA-161), and a run on a deleted task would leave the
// scheduler's motion on a freed stack -- the failure the team lead's "our own auton task" ruling
// exists to prevent. One task also means one painter (the COMP brief's B20) and one writer per
// sink. The task's whole body is inside ONE try: a throw anywhere after boot stops the drive
// (0 V + Brake), safes every present mechanism, disarms the watch, says so on both logs and the
// screen, and the task then idles for ever without touching the Robot again (the brief's R14).
//
// ═══ THE PACER CHAIN (the COMP brief's B7, plus the painter; the mechanism service since MECH-A5) ═══
//     Chassis -> RunGuard -> MechanismService -> AbortWatch -> ScreenPacer -> the provider's ProsTickPacer
// The guard checks its deadlines and its abandon latch first; the MECHANISM SERVICE ticks every
// mechanism (an auton's claw step runs while the robot drives); the watch reads the abort inputs; the
// painter paints; the tick pacer waits for the next 10 ms boundary. The service is constructed in the
// chain BEFORE the Robot and BOUND after it, over the Robot's own built mechanisms (the MECH brief's
// landmines 1 and 20). The BOOT PROVES the chain (pacerChainMissing, the brief's R13 --
// src/common/guarded_run.hpp's since COMP-b's first step, shared with the competition program): one
// idle tick must pace the guard, tick the service and pace the watch, and the watch must be attached
// to the guard, or the boot is REFUSED naming the layer -- a Chassis built without the guard, or on
// the watch directly, could not be aborted, a service outside the chain (or never bound) would run no
// mechanism, and no pin can see every spelling of those mistakes.
//
// ═══ THE ROUTE REGISTRY (the mechanism core's MECH-A5; the team lead's ruling of 2026-10-08) ═══════
// ONE runner::CommandRegistry and ONE runner::MechanismRoutes live in the state: every route id the
// robot's list derives is registered at boot, after the service binds and inside the boot's try, and
// the registry goes to runAuton -- so a ROUTE auton's markers fire the same steps an auton's lines do,
// on the bench exactly as in a match. The boot says how many it published.
//
// ═══ RUN IS REFUSED UNLESS ... (the brief's R4; D5, D9) ═════════════════════════════
// the estimate is live, a controller is connected (it carries the abort), VEXos reports no field,
// switch or Timed Run, and the boot chose the calibration RECORD -- whose forward is the end that
// led the Calibrate push. The typed table's front is the OTHER end, so a boot from it could drive
// the 40 in toward the operator.
//
// ═══ THE LOGS ═══════════════════════════════════════════════════════════════════════
// Serial carries the banner, the session header, the three boot lines (testerBootLines), every
// absent mechanism, the mechanism lines (one block per mechanism, tag MECH: what it is, one line per
// role, its state -- src/common/driver_panel.hpp's words; this program has no driver controls) and
// the route ids published, the competition status on change, and each run's RUN line and
// testerRunLines(); runAuton's own [COMP] lines and the guard's [SEQ] lines ride the provider's
// sink as always. The SD card gets the boot lines, the status changes and every run's lines,
// APPENDED to ONE file across boots
// (shulib_auton_runs.txt at the card's root, A4 register HA-177) through ProsBlockSink's Append
// mode and hal::BlockCharSink -- the adapter owns the /usd/ prefix and every open mode, so this
// file names neither. Never written during a run: the lines go out after the guard hands the
// robot back. No card = serial only, never a refusal.
//
// Compiled in all seven builds (every .cpp under src/ is); nothing here reads a build switch, no
// beacon literal lives here, and the state is one function-local static of EMPTY optionals, so
// --gc-sections drops all of its CODE from every build but tank-autontest (its string literals can
// still ride along where the linker keeps this file's copy of a shared inline function -- words
// only; COMP-b's review measured that for the competition program's). ASCII only in every runtime
// string; every screen string goes through printText(); only a literal "%s" reaches a PROS print.
// No slot number appears in any string here (the slot is the documents' to name).
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station M).

#include "programs/auton_tester_program.hpp"

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
#include "common/screen_paint.hpp"
#include "common/state_screens.hpp"
#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_panel.hpp"
#include "programs/auton_tester_run.hpp"
#include "team/autons.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/block_char_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/mechanism_routes.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

/// Every serial line of this program carries this prefix.
constexpr const char* kTag = "ATS";
/// The upload check: the time this file was compiled, on the READY page and the refusal bar.
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;
/// The SD run log: ONE file across boots, appended (A4 register HA-177) -- and the competition
/// program's too, so its name is src/common/auton_run_log.hpp's since COMP-b. A BARE name: the
/// adapter owns the /usd/ prefix and every open mode (hal/pros/block_sink.hpp; HA-122).
constexpr const char* kSdRunLogFile = kAutonRunLogFile;
/// Live numbers repaint at most 5 Hz (PROS leaks every print).
constexpr double kLiveRepaintSeconds = 0.2;

static_assert(kTesterTickSeconds * 1000.0 == static_cast<double>(hal::pros::ProsTickPacer::kTickMs),
              "the countdown's wait is one motion tick");
// (The status bits the panel reads -- kStatusDisabled, kStatusAutonomous, kStatusConnected -- are
// src/common/state_screens.hpp's since COMP-b's first step, and are checked against PROS there.)
// The screen's two size enums, mapped one to one by the shared painter (printText takes
// calibration::TextSize, the pages ui::TextSize): the budgets must be equal, or a line checked
// against one is cut by the other.
static_assert(calibration::kSmallCols == ui::kSmallCols && calibration::kMediumCols == ui::kMediumCols
                  && calibration::kLargeCols == ui::kLargeCols,
              "calibration:: and ui:: text budgets must be the same numbers");

// ═══ THE REFUSAL SCREEN's fixed words (checked here, where they are written) ═══════════════
constexpr std::string_view kBarWord = "shulib AUTON TESTER";
constexpr std::string_view kNothingPowered =
    "Nothing is powered. The full text is on the serial log.";
constexpr std::string_view kStoppedFooter =
    "The drive is stopped (0 V + Brake). Restart the program.";
// Checked against the shared refusal painter's own budgets (src/common/screen_paint.hpp, since
// COMP-b's first step; its "(the full text is on the serial log)" pointer is checked there).
static_assert(refusalBarFits(kBarWord)
                  && calibration::screenText(kBuildTimeText, calibration::kSmallCols),
              "a refusal-screen literal does not fit its budget");
static_assert(refusalFooterFits(kNothingPowered) && refusalFooterFits(kStoppedFooter),
              "a refusal footer must fit its two SMALL rows");

/// One read of the brain's touch screen: the ONE place PROS's touch struct is read, reduced by
/// the host-tested testerTouchFrom() (programs/auton_abort_watch.hpp).
TesterTouch readTouch() {
    return testerTouchFrom(pros::c::screen_touch_status(), static_cast<int>(pros::E_TOUCH_ERROR));
}

// ═══ THE ABORT'S INPUTS, read from PROS every tick the watch is armed ═══════════════════
class ProsAbortInputs final : public IAbortInputs {
public:
    explicit ProsAbortInputs(const hal::IController& master) noexcept : master_{&master} {}
    [[nodiscard]] AbortInputs read() override {
        return abortInputsFrom(readTouch(), *master_, pros::c::competition_is_disabled() != 0);
    }

private:
    const hal::IController* master_;
};

struct TesterState;
TesterState& state();
TesterFacts readFacts();
void paintFrame(const TesterView& v);
void paintText(const TesterView& was, const TesterView& now);

/// The painter, a layer of the pacer chain (file banner): each tick it asks the screen model for
/// the frame; a frame with different buttons is painted WHOLE at once (so a touch is always tested
/// against the frame on the screen), a text-only change at most every kLiveRepaintSeconds, and only
/// the lines that changed. Disabled until the tester's task enables it (the boot's chain proof
/// ticks once before that, painting nothing).
class ScreenPacer final : public motion::ITickPacer {
public:
    explicit ScreenPacer(motion::ITickPacer& inner) noexcept : inner_{&inner} {}
    void enable() noexcept { enabled_ = true; }
    void pace() override;
    [[nodiscard]] const TesterView& drawn() const noexcept { return painted_; }

private:
    motion::ITickPacer* inner_;
    bool enabled_ = false;
    bool paintedOnce_ = false;
    TesterView painted_{};
    double paintedAt_ = 0.0;
};

// ═══ PROGRAM STATE: one function-local static of EMPTY optionals ═════════════════════════
// ("shulib Drive"'s rule -- "shulib Teleop"'s until T4 Part 3 retired it: nothing non-movable at
// namespace scope, nothing whose constructor can throw in the static itself). Declaration order
// is the borrow order: the provider, the controller and the inputs, then the pacer chain from the
// inside out (painter, watch, mechanism service, guard), then the Robot, whose Chassis keeps the
// guard as its pacer for the Robot's whole life, then the route registry and the mechanism route
// ids over the Robot's Chassis and the service.
struct TesterState {
    BootOutcome boot{};
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<hal::pros::ProsController> master{};
    std::optional<ProsAbortInputs> inputs{};
    std::optional<ScreenPacer> painter{};
    std::optional<AbortWatch> watch{};
    std::optional<mechanism::MechanismService> mechanisms{};  // between the guard and the watch
    std::optional<sequence::RunGuard> guard{};
    std::optional<robot::Robot> robot{};
    std::optional<runner::CommandRegistry> commands{};     // the route registry ...
    std::optional<runner::MechanismRoutes> routes{};       // ... and the robot's mechanism route ids
    std::optional<competition::AutonRegistry> registry{};
    std::optional<TesterScreen> screen{};
    std::optional<hal::pros::ProsBlockSink> sdBlock{};
    std::optional<hal::BlockCharSink> sdChars{};
    std::optional<diag::TermSink> sdLog{};
    TesterRunRecord last{};  // the last run: here, never on the task's stack
    int runs = 0;
    std::uint8_t lastStatus = 0xFF;
    std::optional<pros::Task> task{};
    char portMap[diag::kMaxPortMapBytes]{};
    char refusalHeadline[64]{};
    char refusalWhy[192]{};
};

TesterState& state() {
    static TesterState s;
    return s;
}

// ═══ PAINTING (every string through common/screen_text.hpp's printText, in the shared painters) ═══
// The painters -- the tone colours, a line, a button, the bar, the auton selector's frame -- are
// src/common/screen_paint.{hpp,cpp}'s since COMP-b's first step (2026-09-30): the competition
// program paints the same selector. The calls below are the ones this file made before, in the
// same order, so the pages look exactly as they did.

void paintFrame(const TesterView& v) {
    if (v.page == TesterPage::Pick) {
        paintMenuFrame(v.menu);  // clear, the bar, info, status, the buttons
        return;
    }
    clearPanel();
    paintBar(v.bar, v.header);
    for (std::size_t i = 0; i < v.bigCount; ++i) {
        paintLine(v.big[i], kPanelBg);
    }
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        paintLine(v.lines[i], kPanelBg);
    }
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        const TesterButton& b = v.buttons[i];
        paintButton(b.rect, b.label, b.armed,
                    b.id == TesterButtonId::Run ? ui::Tone::Green : ui::Tone::Blue);
    }
}

/// Repaint only the text lines that changed (same page, same buttons).
void paintText(const TesterView& was, const TesterView& now) {
    if (now.page == TesterPage::Pick) {
        paintMenuText(was.menu, now.menu);
        return;
    }
    for (std::size_t i = 0; i < now.bigCount; ++i) {
        if (i >= was.bigCount || lineDiffers(was.big[i], now.big[i])) {
            paintLine(now.big[i], kPanelBg);
        }
    }
    for (std::size_t i = 0; i < now.lineCount; ++i) {
        if (i >= was.lineCount || lineDiffers(was.lines[i], now.lines[i])) {
            paintLine(now.lines[i], kPanelBg);
        }
    }
}

TesterFacts readFacts() {
    TesterState& st = state();
    const robot::Robot& r = *st.robot;
    TesterFacts f;
    f.now = r.context().clock().now().value();
    f.estimate = r.localizer().pose();
    f.estimateLive =
        r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized;
    f.controllerConnected = st.master->isConnected();
    f.competitionStatus = pros::c::competition_get_status();
    f.abortCause = st.watch->cause();
    f.sdLogging = st.sdLog.has_value();
    f.bootFromRecord = st.boot.source == calibration::BootSource::UseRecord;
    f.faultCount = r.faults().faultCount();
    f.lastFault = r.faults().lastFault();
    f.buildStamp = kBuildTimeText.data();
    return f;
}

void ScreenPacer::pace() {
    if (enabled_) {
        const TesterFacts f = readFacts();
        const TesterView next = state().screen->view(f);
        if (!paintedOnce_ || !sameButtons(next, painted_)) {
            paintFrame(next);
            painted_ = next;
            paintedAt_ = f.now;
            paintedOnce_ = true;
        } else if (f.now - paintedAt_ >= kLiveRepaintSeconds) {
            paintText(painted_, next);
            painted_ = next;
            paintedAt_ = f.now;
        }
    }
    inner_->pace();
}

// ═══ THE LOG: serial always; the SD card when the run log opened ═══════════════════════════
void logBoth(hal::LogLevel level, const char* text) {
    TesterState& st = state();
    st.devices->telemetry().log(level, kTag, text);
    if (st.sdLog.has_value()) {
        st.sdLog->log(level, kTag, text);
    }
}

// ═══ THE REFUSAL SCREEN ═══════════════════════════════════════════════════════════════
// Painted by src/common/screen_paint.cpp's paintRefusalScreen() since COMP-b's first step
// (2026-09-30) -- the painter this file carried until then ("teleop_program.cpp's, wrapped the
// same way"), moved there unchanged for the competition program: the red bar with this program's
// name and BUILD time, the headline over two MEDIUM rows, the reason over four SMALL rows (and a
// pointer to the serial log when longer), the footer over two SMALL rows, and the serial line.
void refuse(const char* headline, const char* why, std::string_view footer = kNothingPowered) {
    TesterState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%.63s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%.191s", why);
    paintRefusalScreen(RefusalScreen{.tag = kTag,
                                     .barWord = kBarWord.data(),
                                     .buildTime = kBuildTimeText.data(),
                                     .headline = st.refusalHeadline,
                                     .why = st.refusalWhy,
                                     .footer = footer});
}

// ═══ ONE RUN ═══════════════════════════════════════════════════════════════════════════
/// Tells the screen which page the run is on (runTesterOnce calls it).
class ScreenRunObserver final : public ITesterRunObserver {
public:
    void countdownStarted(double now) override { state().screen->showCountdown(now); }
    void runStarted(double now) override { state().screen->showRunning(now); }
};

/// `runPressCount`: the press count of the touch read whose release touched RUN -- the abort's
/// touch baseline (runTesterOnce).
void runOne(int index, std::int32_t runPressCount) {
    TesterState& st = state();
    const competition::AutonEntry& entry = st.registry->at(static_cast<std::size_t>(index));
    char line[kTesterLineBytes];
    std::snprintf(line, sizeof line,
                  "run %d: '%.29s' (%s, park T+%.2fs, stop T+%.2fs, start %s) -- RUN touched; 3 s "
                  "countdown, the abort armed",
                  st.runs + 1, entry.name, competition::autonKindWord(entry.kind),
                  entry.endActionAt.value(), entry.hardStopAt.value(),
                  competition::startModeWord(entry.effectiveStart().mode()));
    logBoth(hal::LogLevel::Info, line);
    ScreenRunObserver observer;
    const TesterRunRecord& rec = runTesterOnce(
        st.last, ++st.runs, st.robot->chassis(), *st.guard, *st.watch, *st.registry, index,
        st.devices->telemetry(), *st.mechanisms, &*st.commands, observer, runPressCount);
    const TesterRunLines lines = testerRunLines(rec, entry);
    const hal::LogLevel level = rec.cause != AbortCause::None || rec.threw
                                    ? hal::LogLevel::Warn
                                    : hal::LogLevel::Info;
    for (int i = 0; i < lines.count; ++i) {
        logBoth(level, lines.line[i]);
    }
    const TesterTouch t = readTouch();
    st.screen->showResult(rec, t.sample.releaseCount, t.fingerDown);
}

/// A throw reached the tester's task (a programming error): stop everything safely, say so on
/// both sinks and the screen, and never touch the Robot again until the program restarts.
void stopTheTester(const char* what) noexcept {
    TesterState& st = state();
    stopAfterProgramError(st.robot->chassis(), st.mechanisms->mechanisms(), *st.watch);
    try {
        char line[kTesterLineBytes];
        std::snprintf(line, sizeof line,
                      "the tester STOPPED on a program error: %.100s -- the drive is at 0 V + "
                      "Brake; restart the program",
                      what);
        logBoth(hal::LogLevel::Error, line);
        refuse("the tester STOPPED (a program error)", what,
               "The drive is stopped (0 V + Brake). Restart the program.");
    } catch (...) {
    }
}

// ═══ THE TESTER'S TASK: everything after boot, for the program's whole life ═════════════════
void testerTask(void* /*unused*/) {
    TesterState& st = state();
    try {
        chassis::Chassis& chassis = st.robot->chassis();
        st.screen->showPick(readTouch().sample.releaseCount);
        st.painter->enable();
        for (;;) {
            // ONE tick: the estimate advances, the health checks run, the frame is painted. The
            // drive is never commanded here (Chassis::wait) -- nothing moves until RUN and the
            // countdown.
            chassis.wait(units::Time{kTesterTickSeconds});
            const std::uint8_t status = pros::c::competition_get_status();
            if (status != st.lastStatus) {
                char bits[64];
                char line[kTesterLineBytes];
                const bool allowed =
                    (status & (kStatusDisabled | kStatusAutonomous | kStatusConnected)) == 0;
                std::snprintf(line, sizeof line, "competition status %s -- RUN is %s",
                              competitionBits(status, bits, sizeof bits),
                              allowed ? "allowed"
                                      : "OFF (start the tester from the brain, nothing connected)");
                logBoth(hal::LogLevel::Info, line);
                st.lastStatus = status;
            }
            // The read RUN's release came in on is also the abort's touch baseline: a finger that
            // lands after it aborts, however soon (runTesterOnce).
            const TesterTouch t = readTouch();
            const TesterAction a = st.screen->touch(t.sample, st.painter->drawn(), readFacts());
            if (a.kind == TesterActionKind::Run) {
                runOne(a.entry, t.pressCount);
            }
        }
    } catch (const PreconditionError& e) {
        stopTheTester(e.what());
    } catch (const std::exception& e) {
        stopTheTester(e.what());  // its own words on the logs and the screen, never "unknown"
    } catch (...) {
        stopTheTester("an unknown exception");
    }
    for (;;) {
        pros::delay(100);  // STOPPED: this task never touches the Robot again
    }
}

}  // namespace

void autonTesterInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    TesterState& st = state();
    // ONE write() per line (SER1, HA-189); the first carries the blank line before it.
    hal::pros::serialLine("\n[%s] booted: shulib AUTON TESTER build [%s] -- runs ONE of the "
                          "team's autons", kTag, variantBeacon);
    hal::pros::serialLine("[%s] (src/team/autons.cpp) through competition::runAuton, the "
                          "competition", kTag);
    hal::pros::serialLine("[%s] program's call, when RUN is touched: a 3 s countdown, then the "
                          "run. ANY", kTag);
    hal::pros::serialLine("[%s] controller button or a touch of the screen aborts. NOT RUN ON A "
                          "ROBOT YET.", kTag);
    hal::pros::serialLine("[%s] competition status at boot: %s", kTag, competitionStatusBits);

    st.boot = decideBootSource(kTag);
    if (!st.boot.ok) {
        refuse(st.boot.headline, st.boot.why);
        return;
    }
    // Everything is built at the graph boundary, inside ONE try; any refusal un-builds it all, each
    // object before what its DESTRUCTOR reads: the route ids first; then the SERVICE, before the Robot
    // (its destructor reads every slot's mechanism, and those, the fault latch and the clock it was
    // bound with are the Robot's -- service.hpp: they "must outlive the service"; MECH-A5's review);
    // then the Robot, the guard and the watch, whose destructors read nothing (nothing paces during
    // an unbuild). Nothing holds a mechanism's claim at boot (bind() and the proof's idle tick command
    // nothing), so the service's own end commands no device.
    const auto unbuild = [&st] {
        st.screen.reset();
        st.registry.reset();
        st.routes.reset();
        st.commands.reset();
        st.mechanisms.reset();
        st.robot.reset();
        st.guard.reset();
        st.watch.reset();
        st.painter.reset();
        st.inputs.reset();
        st.master.reset();
        st.devices.reset();
    };
    try {
        st.devices.emplace();
        st.master.emplace(hal::pros::ControllerId::Master);
        st.inputs.emplace(*st.master);
        st.painter.emplace(st.devices->pacer());
        st.watch.emplace(*st.painter, *st.inputs);
        st.mechanisms.emplace(*st.watch);  // in the chain BEFORE the Robot; bound AFTER it
        st.guard.emplace(*st.mechanisms);
        st.watch->attachGuard(*st.guard);
        robot::BuildOptions opts = st.boot.built.buildOptions();
        opts.pacer = &*st.guard;  // BORROWED for the Robot's whole life (robot.hpp)
        const robot::BuildVerdict verdict = robot::validate(st.boot.built.config, *st.devices, opts);
        if (!verdict.ok) {
            unbuild();
            refuse("the library graph REFUSED to build", verdict.why);
            return;
        }
        st.robot.emplace(st.boot.built.config, *st.devices, opts);
        // THE MECHANISM SERVICE: bound over the Robot's own built mechanisms -- one source of devices
        // (the MECH brief's landmine 20) -- before any auton can run.
        st.mechanisms->bind(kMechanisms, st.robot->mechanismParts().devices(), st.robot->context().clock(),
                            st.robot->faults(), st.robot->context().telemetry());
        // THE ROUTE REGISTRY (header): every mechanism route id the list derives, registered once.
        st.commands.emplace();
        st.routes.emplace(st.robot->chassis(), *st.mechanisms, kMechanisms, st.devices->telemetry());
        st.routes->registerInto(*st.commands);
        // The team's autons. An entry the registry refuses is DROPPED -- never on PICK, named there
        // and in the log -- and the boot goes on: the team lead's ruling of 2026-10-02 for the
        // competition program ("drop just that auton"), applied here too by the coordinator
        // (T4 Part 3's late step; the team lead is asked to confirm it), because this program
        // registers through the same function (src/team/autons.cpp) and a student fixing one
        // auton must still be able to test the others.
        st.registry.emplace(kAutonPeriods, competition::OnRefusal::DropEntry);
        registerAutons(*st.registry);
        // (The guard safes the Robot's PRESENT mechanisms -- the COMP brief's B9 -- through the
        // service: runAuton's service overload hands the guard the service's built ones.)
        st.screen.emplace(*st.registry);
        // THE CHAIN PROOF: one idle tick must pace the guard, tick the mechanism service and pace the
        // watch, or no abort could stop the robot (or no mechanism would run). Refused, not warned,
        // naming the gap: a build that cannot abort must not run an auton.
        if (const ChainGap gap = pacerChainMissing(st.robot->chassis(), *st.guard, *st.mechanisms, *st.watch);
            gap != ChainGap::None) {
            char chain[192];
            std::snprintf(chain, sizeof chain, "%.150s -- a build error, not a robot fault", chainGapWords(gap));
            unbuild();
            refuse("the tester's pacer chain is WRONG", chain);
            return;
        }
    } catch (const PreconditionError& e) {
        unbuild();
        refuse("the tester REFUSED to start", e.what());
        return;
    }
    attachFaultLatch(&st.robot->faults());

    // The SD run log (A4 register HA-177): ONE file, appended across boots, through the adapter
    // that owns /usd/ and every mode; no card, or a log that will not open = serial only.
    st.sdBlock.emplace(kSdRunLogFile, hal::pros::ProsFileMode::Append);
    if (st.sdBlock->isOpen()) {
        st.sdChars.emplace(*st.sdBlock);
        st.sdLog.emplace(st.devices->clock(), *st.sdChars);
    }
    // The session header (the S18.5 provenance block, serial): the hash LOUD when missing, the
    // port map generated from the config -- the block "shulib Drive" prints too ("shulib Teleop"'s
    // until T4 Part 3).
    st.robot->portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(auton tester: one per RUN)";
    session.alliance = "";
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(st.devices->telemetry(), session,
                            st.robot->context().battery().voltage());
    // The boot's three lines (testerBootLines, host-tested at every field's worst case): the build
    // hash is a string_view, bounded there -- NEVER handed to %s (the 2026-08-18 data abort).
    const TesterSdLog sd = !st.sdLog.has_value() ? TesterSdLog::Off
                           : st.sdBlock->appendOpen() == hal::pros::AppendOpen::Created
                               ? TesterSdLog::Created
                               : TesterSdLog::Appended;
    const TesterBootLines boot = testerBootLines(
        variantBeacon, diag::compiledBuildHash(), bootSourceWord(st.boot.source),
        static_cast<int>(st.registry->size()), static_cast<int>(st.mechanisms->mechanisms().size()),
        static_cast<int>(st.robot->mechanismCount()), sd,
        st.sdLog.has_value() ? st.sdBlock->path() : "");
    for (const char* bootLine : boot.line) {
        logBoth(hal::LogLevel::Info, bootLine);
    }
    char line[kTesterLineBytes];
    for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
        if (st.robot->builtMechanismAt(i) == nullptr) {
            std::snprintf(line, sizeof line,
                          "mechanism '%.23s' is ABSENT -- the run guard cannot safe it: %.120s",
                          st.robot->config().mechanisms[i].name, st.robot->mechanismAbsence(i));
            logBoth(hal::LogLevel::Warn, line);
        }
    }
    // THE MECHANISMS (MECH-A5): each one's block -- what it is, one line per role, its state -- in
    // src/common/driver_panel.hpp's words (the lines "shulib Drive" and the competition program print,
    // without the control lines: this program has no driver control), tag MECH, through the
    // provider's sink; then the route ids they publish, on serial and the SD run log.
    mechanismBootLines(kMechanisms, *st.robot, *st.mechanisms, nullptr,
                       [&st](const MechanismLine& m) { st.devices->telemetry().log(m.level, "MECH", m.text); });
    logBoth(hal::LogLevel::Info, routesBootLine(st.routes->ids()).text);
    // Every auton the registry DROPPED, by name, on serial and the SD run log (the words shared
    // with the competition program: src/common/dropped_autons.hpp). `false`: this program's
    // driver control does nothing, so its line never says driver control still runs.
    sayDroppedAutons(*st.registry, false,
                     [](const char* dropped) { logBoth(hal::LogLevel::Warn, dropped); });

    st.task.emplace(&testerTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                    "auton tester");
}

}  // namespace shulib::bench
