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
// ═══ THE PACER CHAIN (the COMP brief's B7, plus the painter) ════════════════════════
//     Chassis -> RunGuard -> AbortWatch -> ScreenPacer -> the provider's 10 ms ProsTickPacer
// The guard checks its deadlines and its abandon latch first; the watch reads the abort inputs;
// the painter paints; the tick pacer waits for the next 10 ms boundary. The BOOT PROVES the chain
// (pacerChainProven, the brief's R13): one idle tick must pace both the guard and the watch, or the
// boot is REFUSED -- a Chassis built without the guard, or on the watch directly, could not be
// aborted, and no pin can see every spelling of that mistake.
//
// ═══ RUN IS REFUSED UNLESS ... (the brief's R4; D5, D9) ═════════════════════════════
// the estimate is live, a controller is connected (it carries the abort), VEXos reports no field,
// switch or Timed Run, and the boot chose the calibration RECORD -- whose forward is the end that
// led the Calibrate push. The typed table's front is the OTHER end, so a boot from it could drive
// the 40 in toward the operator.
//
// ═══ THE LOGS ═══════════════════════════════════════════════════════════════════════
// Serial carries the banner, the session header, the three boot lines (testerBootLines), every
// absent mechanism, the competition status on change, and each run's RUN line and
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
// --gc-sections drops all of it from every build but tank-autontest. ASCII only in every runtime
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

#include "common/boot_source.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/screen_text.hpp"
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
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

using calibration::TextSize;

/// Every serial line of this program carries this prefix.
constexpr const char* kTag = "ATS";
/// The upload check: the time this file was compiled, on the READY page and the refusal bar.
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;
/// The SD run log: ONE file across boots, appended (A4 register HA-177). A BARE name: the adapter
/// owns the /usd/ prefix and every open mode (hal/pros/block_sink.hpp; HA-122).
constexpr const char* kSdRunLogFile = "shulib_auton_runs.txt";
/// Live numbers repaint at most 5 Hz (PROS leaks every print).
constexpr double kLiveRepaintSeconds = 0.2;

static_assert(kTesterTickSeconds * 1000.0 == static_cast<double>(hal::pros::ProsTickPacer::kTickMs),
              "the countdown's wait is one motion tick");
static_assert(kStatusDisabled == static_cast<std::uint8_t>(COMPETITION_DISABLED)
                  && kStatusAutonomous == static_cast<std::uint8_t>(COMPETITION_AUTONOMOUS)
                  && kStatusConnected == static_cast<std::uint8_t>(COMPETITION_CONNECTED),
              "the panel's status bits are PROS's");
// The screen's two size enums, mapped one to one (printText takes calibration::TextSize, the pages
// ui::TextSize): the budgets must be equal, or a line checked against one is cut by the other.
static_assert(calibration::kSmallCols == ui::kSmallCols && calibration::kMediumCols == ui::kMediumCols
                  && calibration::kLargeCols == ui::kLargeCols,
              "calibration:: and ui:: text budgets must be the same numbers");

// ═══ THE REFUSAL SCREEN's fixed words (checked here, where they are written) ═══════════════
constexpr std::string_view kBarWord = "shulib AUTON TESTER";
constexpr std::string_view kNothingPowered =
    "Nothing is powered. The full text is on the serial log.";
constexpr std::string_view kStoppedFooter =
    "The drive is stopped (0 V + Brake). Restart the program.";
constexpr std::string_view kMoreOnSerial = "(the full text is on the serial log)";
static_assert(calibration::screenText(kBarWord, calibration::kHeaderCols)
                  && calibration::screenText(kMoreOnSerial, calibration::kSmallCols)
                  && calibration::screenText(kBuildTimeText, calibration::kSmallCols),
              "a refusal-screen literal does not fit its budget");
static_assert(calibration::wrapWords<calibration::kSmallCols, 2>(kNothingPowered).complete
                  && calibration::wrapWords<calibration::kSmallCols, 2>(kStoppedFooter).complete,
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
// (teleop_program.cpp's rule: nothing non-movable at namespace scope, nothing whose constructor
// can throw in the static itself). Declaration order is the borrow order: the provider, the
// controller and the inputs, then the pacer chain from the inside out (painter, watch, guard),
// then the Robot, whose Chassis keeps the guard as its pacer for the Robot's whole life.
struct TesterState {
    BootOutcome boot{};
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<hal::pros::ProsController> master{};
    std::optional<ProsAbortInputs> inputs{};
    std::optional<ScreenPacer> painter{};
    std::optional<AbortWatch> watch{};
    std::optional<sequence::RunGuard> guard{};
    std::optional<robot::Robot> robot{};
    std::optional<competition::AutonRegistry> registry{};
    std::optional<TesterScreen> screen{};
    std::array<hal::IMechanism*, config::kMaxMechanisms> present{};
    std::size_t presentCount = 0;
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

// ═══ PAINTING (every string through common/screen_text.hpp's printText) ════════════════════
constexpr std::uint32_t kBg = 0x101418;

std::uint32_t rgb(ui::Tone t) {
    switch (t) {
        case ui::Tone::Blue: return 0x1E5AA8;
        case ui::Tone::Amber: return 0xE0A000;
        case ui::Tone::Green: return 0x2E8B3A;
        case ui::Tone::Red: return 0xC0392B;
        case ui::Tone::Grey: return 0x707070;
        case ui::Tone::White: return 0xF0F0F0;
    }
    return 0xF0F0F0;
}

TextSize printSize(ui::TextSize s) {
    switch (s) {
        case ui::TextSize::Small: return TextSize::Small;
        case ui::TextSize::Medium: return TextSize::Medium;
        case ui::TextSize::Large: return TextSize::Large;
    }
    return TextSize::Small;
}

void paintLine(const competition::MenuLine& l, std::uint32_t eraser) {
    printText(printSize(l.size), l.x, l.y, rgb(l.tone), eraser, l.text, ui::budgetFor(l.size));
}

void paintButton(ui::Rect r, const char* label, bool armed, ui::Tone tone) {
    const std::uint32_t fill = armed ? rgb(tone) : rgb(ui::Tone::Grey);
    pros::c::screen_set_pen(fill);
    pros::c::screen_fill_rect(r.x0, r.y0, r.x1, r.y1);
    printText(TextSize::Medium, static_cast<std::int16_t>(r.x0 + 8),
              static_cast<std::int16_t>(r.y0 + 16), rgb(ui::Tone::White), fill, label);
}

void paintFrame(const TesterView& v) {
    pros::c::screen_set_eraser(kBg);
    pros::c::screen_erase();
    if (v.page == TesterPage::Pick) {
        const competition::MenuView& m = v.menu;
        pros::c::screen_set_pen(rgb(m.header.tone));
        pros::c::screen_fill_rect(0, 0, 479, 26);
        paintLine(m.header, rgb(m.header.tone));
        paintLine(m.info, kBg);
        paintLine(m.status, kBg);
        for (std::size_t i = 0; i < m.buttonCount; ++i) {
            const competition::MenuButton& b = m.buttons[i];
            paintButton(b.rect, b.label, b.armed, b.chosen ? ui::Tone::Green : ui::Tone::Blue);
        }
        return;
    }
    pros::c::screen_set_pen(rgb(v.bar));
    pros::c::screen_fill_rect(0, 0, 479, 26);
    paintLine(v.header, rgb(v.bar));
    for (std::size_t i = 0; i < v.bigCount; ++i) {
        paintLine(v.big[i], kBg);
    }
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        paintLine(v.lines[i], kBg);
    }
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        const TesterButton& b = v.buttons[i];
        paintButton(b.rect, b.label, b.armed,
                    b.id == TesterButtonId::Run ? ui::Tone::Green : ui::Tone::Blue);
    }
}

/// Repaint only the text lines that changed (same page, same buttons).
void paintText(const TesterView& was, const TesterView& now) {
    const auto differs = [](const competition::MenuLine& a, const competition::MenuLine& b) {
        return a.tone != b.tone || std::string_view{a.text} != std::string_view{b.text};
    };
    if (now.page == TesterPage::Pick) {
        if (differs(was.menu.header, now.menu.header)) {
            paintLine(now.menu.header, rgb(now.menu.header.tone));
        }
        if (differs(was.menu.info, now.menu.info)) {
            paintLine(now.menu.info, kBg);
        }
        if (differs(was.menu.status, now.menu.status)) {
            paintLine(now.menu.status, kBg);
        }
        return;
    }
    for (std::size_t i = 0; i < now.bigCount; ++i) {
        if (i >= was.bigCount || differs(was.big[i], now.big[i])) {
            paintLine(now.big[i], kBg);
        }
    }
    for (std::size_t i = 0; i < now.lineCount; ++i) {
        if (i >= was.lineCount || differs(was.lines[i], now.lines[i])) {
            paintLine(now.lines[i], kBg);
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

// ═══ THE REFUSAL SCREEN (teleop_program.cpp's painter, wrapped the same way) ═══════════════════
/// A full-screen refusal, every string through printText(): the bar, the headline wrapped over two
/// MEDIUM rows, the reason over four SMALL rows (and a pointer to the serial log when it is
/// longer), and the footer over two SMALL rows. Also the serial line.
void paintRefusal(const char* headline, const char* why, std::string_view footer) {
    pros::c::screen_set_eraser(kBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(rgb(ui::Tone::Red));
    pros::c::screen_fill_rect(0, 0, 479, 26);
    printText(TextSize::Medium, 8, 4, rgb(ui::Tone::White), rgb(ui::Tone::Red), kBarWord.data());
    printText(TextSize::Small, 330, 8, rgb(ui::Tone::White), rgb(ui::Tone::Red),
              kBuildTimeText.data());
    const auto h =
        calibration::wrapWords<calibration::kMediumCols, 2>(headline != nullptr ? headline : "");
    for (std::size_t k = 0; k < h.count; ++k) {
        printText(TextSize::Medium, 8, static_cast<std::int16_t>(34 + 22 * static_cast<int>(k)),
                  rgb(ui::Tone::Red), kBg, h.line[k]);
    }
    const auto w = calibration::wrapWords<calibration::kSmallCols, 4>(why != nullptr ? why : "");
    for (std::size_t k = 0; k < w.count; ++k) {
        printText(TextSize::Small, 8, static_cast<std::int16_t>(82 + 16 * static_cast<int>(k)),
                  rgb(ui::Tone::White), kBg, w.line[k]);
    }
    if (!w.complete || !h.complete) {
        printText(TextSize::Small, 8, 146, rgb(ui::Tone::Grey), kBg, kMoreOnSerial.data());
    }
    const auto foot = calibration::wrapWords<calibration::kSmallCols, 2>(footer);
    for (std::size_t k = 0; k < foot.count; ++k) {
        printText(TextSize::Small, 8, static_cast<std::int16_t>(182 + 16 * static_cast<int>(k)),
                  rgb(ui::Tone::Grey), kBg, foot.line[k]);
    }
    std::printf("[%s] REFUSED: %s -- %s\n", kTag, headline != nullptr ? headline : "",
                why != nullptr ? why : "");
    std::fflush(stdout);
}

void refuse(const char* headline, const char* why, std::string_view footer = kNothingPowered) {
    TesterState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%.63s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%.191s", why);
    paintRefusal(st.refusalHeadline, st.refusalWhy, footer);
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
        st.devices->telemetry(),
        std::span<hal::IMechanism* const>{st.present.data(), st.presentCount}, observer,
        runPressCount);
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
    stopAfterProgramError(st.robot->chassis(),
                          std::span<hal::IMechanism* const>{st.present.data(), st.presentCount},
                          *st.watch);
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
    std::printf("\n[%s] booted: shulib AUTON TESTER build [%s] -- runs ONE of the team's autons\n"
                "[%s] (src/team/autons.cpp) through competition::runAuton, the competition\n"
                "[%s] program's call, when RUN is touched: a 3 s countdown, then the run. ANY\n"
                "[%s] controller button or a touch of the screen aborts. NOT RUN ON A ROBOT YET.\n"
                "[%s] competition status at boot: %s\n",
                kTag, variantBeacon, kTag, kTag, kTag, kTag, competitionStatusBits);
    std::fflush(stdout);

    st.boot = decideBootSource(kTag);
    if (!st.boot.ok) {
        refuse(st.boot.headline, st.boot.why);
        return;
    }
    // Everything is built at the graph boundary, inside ONE try; any refusal un-builds it all.
    const auto unbuild = [&st] {
        st.screen.reset();
        st.registry.reset();
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
        st.guard.emplace(*st.watch);
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
        st.registry.emplace(kAutonPeriods);
        registerAutons(*st.registry);  // an entry the registry refuses throws, naming the auton
        // The guard safes the Robot's PRESENT mechanisms (the COMP brief's B9).
        for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
            hal::IMechanism* m = st.robot->builtMechanismAt(i);
            if (m != nullptr) {
                st.present[st.presentCount++] = m;
            }
        }
        st.screen.emplace(*st.registry);
        // THE CHAIN PROOF: one idle tick must pace the guard AND the watch, or no abort could
        // stop the robot. Refused, not warned: a build that cannot abort must not run an auton.
        if (!pacerChainProven(st.robot->chassis(), *st.guard, *st.watch)) {
            unbuild();
            refuse("the tester's pacer chain is WRONG",
                   "one idle tick did not pace the run guard and the abort watch, or the watch is "
                   "not attached to the guard -- the abort could not stop the robot; this is a "
                   "build error, not a robot fault");
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
    // port map generated from the config -- teleop_program.cpp's block.
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
        static_cast<int>(st.registry->size()), static_cast<int>(st.presentCount),
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

    st.task.emplace(&testerTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                    "auton tester");
}

}  // namespace shulib::bench
