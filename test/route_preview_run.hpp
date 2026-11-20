#pragma once
//
// route_preview_run.hpp — THE ROUTE PREVIEW's logic (chunk RPV1-A — A0 and B1 together, 2026-11-12; the
// RPV1 brief's §1.1 and §1.2): one named auton from the team's REAL autons file, run through the REAL
// library on the host test plant exactly as "shulib Auton Tester" runs it, recorded, folded into a Model
// (test/route_preview_model.hpp) and written as ONE offline page plus the same table on stdout
// (test/route_preview_render.hpp). Say "host test plant" (the project's one name for it). Host only (allocates).
// NOTHING HERE RUNS ON A ROBOT, and nothing here changes anything a robot program compiles.
//
// ALL the program's logic is here, as runPreview(args, out, err) — argument parsing, the lookup, the
// run, the page, the exit code — and runProgram (runPreview with a failure of the preview itself turned
// into words and kExitToolFailed), so the suite calls both (test/route_preview_test.cpp);
// test/route_preview_main.cpp only forwards argv. tools/route_preview.py builds and runs that program
// under the suite's own bounds.
//
// ── THE RUN (reuse, not a fork — the brief's landmine 8) ──────────────────────────────────────────
// The Auton Tester's rig, test/auton_tester_rig.hpp's TesterStack, unchanged in behaviour: its 50 idle
// ticks, the 3 s countdown, the quiet tick, then competition::runAuton — the competition program's call
// — on the same entry, guard, schedule, mechanism service and route registry. The plant is PLACED at
// the entry's FIELD start (or at --place for a RELATIVE one; the field's centre facing 0 degrees
// without it, said in red): a plant left at the rig's origin would draw a wrong path that looks right
// (the brief's P5b). Three things are attached, none of which changes the run:
//   * the RECORDER — a pacer layer at the probe's place, above the plant: on every pace it notes what
//     the code above it left (each mechanism slot's step count, the guard), forwards, then stores the
//     plant's TRUE pose (SimHarness::sample() — never the estimate, which is stored beside it) and how
//     many lines the sink holds, which dates every log line to its pace;
//   * the scheduler's BOUNDARY OBSERVER — every motion's start, end, exit and target;
//   * the fake sink's LINE HOOK — the NO-PACE STOP (below).
// Every T+ is measured from the instant the guard armed (AutonRunResult::armedAt), never from the
// clock's zero: the tester's idle ticks and countdown come first (the brief's landmine 1).
//
// ── THE NO-PACE STOP (the brief's R3) ────────────────────────────────────────────────────────────
// A route that keeps going after a stop or a cut (`for (;;) { r.moveTo(A); r.moveTo(B); }`) never paces
// but logs one "skipped" line per step; the fake sink keeps every line, so it would grow to 16.7 million
// lines and die of std::bad_alloc. When more than kNoPaceLineLimit lines arrive with no pace between,
// the hook folds what was recorded, writes the page and the table with "after step N the auton keeps
// looping without letting time pass ...", and ends the process with std::_Exit — 1 (kExitProblem), or
// 3 (kExitToolFailed) when the page could not be written — it runs inside library code that must not
// throw, so nothing is unwound. Since API 2.11 (RMP1-K D4) a step that SUCCEEDS logs its "done in" line, so a loop of
// them is caught the same way; only code that never ends one step logs nothing: the launcher's 300 s timeout catches
// that one, and it says so.
//
// ── THE END ACTION, by ORDER (RPV1-A's review, 2026-11-13) ───────────────────────────────────────
// A step is the end action's (drawn "park", unnumbered) when it began AFTER the guard logged
// kEndActionLine — never by its time: a step that fails at once (a mechanism not on this robot, one
// not homed) stops the routine with no time passing, so it begins at the very instant scoring ends,
// and a time window called it the park. The sink's line hook notes, as that line is stored, every
// mechanism step begun so far and every motion boundary reported so far (the guard cancels every
// motion before it logs the line, sequence/run_guard.hpp); everything after is the end action's —
// an entry's own end action that commands a mechanism included.
//
// ── WHERE A RUN THREW ────────────────────────────────────────────────────────────────────────────
// A step that is running when anything throws — the world below the recorder (a plant, a
// degradation), a pacer layer above it, the library's own code — is ended BY THE UNWIND: the
// scheduler's WaitUnwindGuard and the guard's RunUnwindGuard are destructors, so a motion's boundary,
// and a mechanism step's CANCELLED line (the guard's cancel-all), are recorded while the exception is
// still in flight (std::uncaught_exceptions() > 0 — noted by the boundary observer for a motion, by
// the sink's line hook for a MECH line). That step is the one the run threw INSIDE: a motion, or a
// mechanism step the robot did not drive during (a blocking one — `r.run(...)`, `r.move(...)`). A
// mechanism the robot drove during was started in the background and the code had moved on past it,
// so its CANCELLED line does not place the throw. (From below, a background start followed by a pause
// or a waitUntilDone with no drive between looks the same as a blocking step — the code waiting while
// that step runs — and reads as inside it.) A step the deadline cut, or one that ended before the
// auton's own code threw, was recorded with nothing in flight: the run threw AFTER it. With no step
// ended by the unwind, a throw from inside a pace (the recorder raises a flag for the length of each
// one: a pause's plant, the plant's pace cap) is "during a wait"; any other is after the last step
// that began.
//
// ── MECHANISM STEPS ARE ATTRIBUTED BY NAME (the brief's R4) ──────────────────────────────────────
// A step begins when its slot's step count rises; its verdict is the MECH line that QUOTES the
// mechanism's name, whenever it arrives — never the step that happened to be open then. A mechanism
// started in the background (startRun, startMove, startSpinUp, startHome) is drawn as its own stretch of
// path, from where it began to where its verdict arrived; a failed one is red on ITS span, and the drive
// that was open stays the drive's colour.
//
// ── ANY ROBOT'S MECHANISMS (RPV1-B1, the brief's §1.2) ───────────────────────────────────────────
// `--robot two` (the default) previews robot two's real list on the rig's default fakes, exactly the
// suite's run. Another robot runs its own list on test/fakes_by_kind.hpp (fakes by kind, a lift or arm
// on a plant built from its declaration), still on the stand-in drive. Robot one's OWED facts are PLACEHOLDERS.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "auton_tester_rig.hpp"
#include "fakes_by_kind.hpp"
#include "motion_test_rig.hpp"
#include "programs/auton_tester_run.hpp"
#include "robots/mechanism_table.hpp"
#include "robots/robot_one_facts.hpp"
#include "robots/robot_two_facts.hpp"
#include "route_preview_model.hpp"
#include "route_preview_render.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/scenario.hpp"
#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"

namespace route_preview {

/// One number a robot file marks PROVISIONAL, by the mechanism it belongs to (the brief's R11): the
/// page hatches every step of that mechanism whose time comes from it. The suite reads the robot
/// file's `// PROVISIONAL (HA-…)` comments and holds this table equal to them (RP33).
struct Provisional {
    const char* mechanism;  ///< the declaration's name
    const char* number;     ///< the number, as the page says it ("0.5 s")
    const char* ha;         ///< its Hardware Assumptions row ("HA-185")
    const char* who;        ///< what measures it ("Station J times the real one")
};
/// The CLAW ROBOT's (the old robot two, retired at ROB1-BT — test/mechanism_fixtures.hpp): the claw's move time.
/// The preview's fixture robots read it.
inline constexpr Provisional kRobotTwoProvisional[] = {
    {"claw", "0.5 s", "HA-185", "Station J times the real one"},
};

/// The function a robot's autons file registers its autons with.
using RegisterFn = void (*)(shulib::competition::AutonRegistry&);
/// A robot's facts ledger, every fact typed or OWED (robot one's src/robots/robot_one_facts.hpp).
using Ledger = std::span<const shulib::bench::robot_one_facts::FactSummary>;

/// A robot the preview can preview (header: ANY ROBOT'S MECHANISMS).
struct PreviewRobot {
    const char* key = "";                                       ///< `--robot` word ("two")
    const char* title = "";                                     ///< "robot two"
    const shulib::mechanism::MechanismTable* table = nullptr;   ///< its mechanism list
    RegisterFn registerAutons = nullptr;                        ///< its autons file's register function
    const char* autonsFile = "";                                ///< where its autons are written
    bool defaultRig = false;                                    ///< the rig's own default fakes (robot two)
    std::span<const Provisional> provisional{};                 ///< its PROVISIONAL numbers
    Ledger ledger{};                                            ///< its facts ledger: each OWED fact is a PLACEHOLDER
    const char* ledgerFile = "";                                ///< where that ledger is
};

/// Robot two — robot one's twin since ROB1-BT (src/team/autons.cpp, `using namespace robot_two;`), its
/// mechanisms on fakes built from their declarations (its numbers are its facts file's placeholders until
/// typed; since 2026-11-20 robot one's values, the same build: its pistons' times are robot one's chosen ones,
/// unhatched as robot one's are — RP33b pins this to the ledger).
inline constexpr PreviewRobot kRobotTwo{
    .key = "two",
    .title = "robot two",
    .table = &shulib::bench::robot_two::kRobotTwoMechanisms,
    .registerAutons = &shulib::bench::registerAutons,
    .autonsFile = "src/team/autons.cpp",
    .defaultRig = false,
    .ledger = Ledger{shulib::bench::robot_two_facts::kRobotTwoFacts},
    .ledgerFile = "src/robots/robot_two_facts.hpp",
};
/// Robot one — its own list on fakes by kind; its ledger's OWED facts are the PLACEHOLDERS (header).
inline constexpr PreviewRobot kRobotOne{
    .key = "one",
    .title = "robot one",
    .table = &shulib::bench::robot_one::kRobotOneMechanisms,
    .registerAutons = &shulib::bench::registerRobotOneAutons,
    .autonsFile = "src/team/robot_one_autons.cpp",
    .defaultRig = false,
    .ledger = Ledger{shulib::bench::robot_one_facts::kRobotOneFacts},
    .ledgerFile = "src/robots/robot_one_facts.hpp",
};
/// The robots `--robot` knows, in order (the first is the default).
inline constexpr PreviewRobot kRobots[] = {kRobotTwo, kRobotOne};

/// The exit codes (the program's; tools/route_preview.py passes them through, and adds 1 for a run its
/// bounds cut and 3 for a program that did not build).
inline constexpr int kExitDrawn = 0;      ///< drawn, nothing wrong (TOUCH notes allowed)
inline constexpr int kExitProblem = 1;    ///< drawn: refused, threw, went THROUGH, or looped with no time passing
inline constexpr int kExitUsage = 2;      ///< usage: an unknown name, robot or flag — nothing written
inline constexpr int kExitToolFailed = 3; ///< the preview itself failed (the page could not be written, or it threw)

/// What a test (never the command line) may add to a run.
struct PreviewOptions {
    bool place = false;   ///< --place given
    double placeX = 0.0;  ///< its x, in
    double placeY = 0.0;  ///< its y, in
    double placeDeg = 0.0;  ///< its heading, degrees
    std::string stamp;       ///< --stamp
    std::string autonsHash;  ///< --autons-hash
    /// A degradation for the plant (a test's OffsetImu); nullptr = the ideal plant.
    shulib::sim::DegradationModel* degradation = nullptr;
    /// Called once everything is attached, before the run (a test's independent tap goes below the
    /// recorder through it): the stack and the recorder's `inner` slot.
    std::function<void(tester_rig::TesterStack&, shulib::motion::ITickPacer*& recorderInner)> onBuilt;
    /// The no-pace stop's writer (runPreview's: the page and the table); it returns whether the page
    /// was written. The process then ends with std::_Exit(kExitProblem), or std::_Exit(kExitToolFailed)
    /// when it was not (header: THE NO-PACE STOP).
    std::function<bool(const Model&)> onLoop;
};

/// The stand-in drive every preview runs on: the suite's 12 in tank.
[[nodiscard]] inline const shulib::kinematics::TankKinematics& standInDrive() {
    static const shulib::kinematics::TankKinematics k{shulib::units::Length{12.0}};
    return k;
}

/// Collects every motion boundary (header), and whether each was recorded while an exception was in
/// flight — the motion the run threw INSIDE (header: WHERE A RUN THREW).
struct MotionLog final : shulib::motion::IMotionObserver {
    std::vector<shulib::motion::CompletedMotion> motions;
    std::vector<char> duringThrow;  ///< per motion: 1 when its boundary came during an unwind
    void onMotionComplete(const shulib::motion::CompletedMotion& c) override {
        motions.push_back(c);
        duringThrow.push_back(std::uncaught_exceptions() > 0 ? 1 : 0);
    }
};

/// The guard's line that opens the end action (sequence/run_guard.hpp's logActStart): every step that
/// began before it is the routine's, every one after it the end action's (header: THE END ACTION).
inline constexpr std::string_view kEndActionLine = "end-of-run action starting";

/// THE RECORDER (header).
struct Recorder final : shulib::motion::ITickPacer {
    /// One recorded pace.
    struct Raw {
        shulib::sim::TruthSample truth{};  ///< the plant's truth after the pace
        double ex = 0.0;                   ///< the estimate's x then
        double ey = 0.0;                   ///< the estimate's y then
        std::size_t lines = 0;             ///< the sink's line count then
    };
    /// A mechanism step seen beginning: its slot, the sample current then, the sink's count then.
    struct Begin {
        std::size_t slot = 0;    ///< the slot's index
        std::size_t sample = 0;  ///< the raw sample current when it was seen
    };

    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::sim::SimHarness* h = nullptr;
    const shulib::localization::Localizer* loc = nullptr;
    const shulib::mechanism::MechanismService* svc = nullptr;
    const shulib::sequence::RunGuard* guard = nullptr;
    const MotionLog* log = nullptr;
    shulib::hal::fake::FakeTelemetrySink* sink = nullptr;
    std::vector<Raw> raw;
    std::vector<Begin> begins;
    std::vector<std::uint32_t> begun;   ///< each slot's step count as last seen
    long armSample = -1;                ///< the raw sample current when the guard was first seen live
    std::size_t linesAtLastPace = 0;    ///< the sink's count at the last pace (the no-pace stop)
    std::size_t firstLine = 0;          ///< the sink's count when the recorder was attached
    std::function<void()> onNoPace;     ///< the no-pace stop (header)
    bool stopped = false;
    /// THE END ACTION (header): the sink index of the guard's kEndActionLine (-1: it never started),
    /// and how many mechanism begins and motion boundaries had been seen when it was logged.
    long endActionLine = -1;
    std::size_t beginsBeforeEndAction = 0;
    std::size_t motionsBeforeEndAction = 0;
    /// True from the start of a pace until the world below it returns: still true after the run, the
    /// run's throw came from INSIDE a pace (header: WHERE A RUN THREW).
    bool inPace = false;
    /// The sink index of every MECH line logged while an exception was in flight, in order (header:
    /// WHERE A RUN THREW — a mechanism step the unwind ended).
    std::vector<std::size_t> mechLinesInUnwind;

    /// Attach: the first sample is the world as it is now.
    void attach() {
        begun.assign(svc->size(), 0);
        for (std::size_t i = 0; i < svc->size(); ++i) {
            begun[i] = svc->slotPtr(i)->stepsBegun();
        }
        firstLine = static_cast<std::size_t>(sink->size());
        linesAtLastPace = firstLine;
        record();
    }
    void record() {
        const shulib::math::Pose2d e = loc->pose();
        raw.push_back(Raw{h->sample(), e.x().value(), e.y().value(), static_cast<std::size_t>(sink->size())});
    }
    /// What the code above left, before the world moves (header).
    void observe() {
        if (armSample < 0 && guard->running()) {
            armSample = static_cast<long>(raw.size() - 1);
        }
        noteBegins();
    }
    /// Every mechanism step begun since the last look, dated to the sample current now.
    void noteBegins() {
        const std::size_t latest = raw.size() - 1;
        for (std::size_t i = 0; i < svc->size() && i < begun.size(); ++i) {
            const std::uint32_t now = svc->slotPtr(i)->stepsBegun();
            for (std::uint32_t k = begun[i]; k < now; ++k) {
                begins.push_back(Begin{i, latest});
            }
            begun[i] = now;
        }
    }
    void pace() override {
        linesAtLastPace = static_cast<std::size_t>(sink->size());
        inPace = true;
        observe();
        inner->pace();
        record();
        inPace = false;
    }
    /// The sink's hook (header: THE NO-PACE STOP, THE END ACTION, WHERE A RUN THREW).
    static void onLine(void* self, std::size_t lines) {
        auto* r = static_cast<Recorder*>(self);
        if (std::uncaught_exceptions() > 0 && lines > 0 &&
            r->sink->at(static_cast<int>(lines - 1)).subsystem == "MECH") {
            r->mechLinesInUnwind.push_back(lines - 1);
        }
        if (r->endActionLine < 0 && lines > 0) {
            const auto& e = r->sink->at(static_cast<int>(lines - 1));
            if (e.subsystem == "SEQ" && std::string_view{e.message}.substr(0, kEndActionLine.size()) == kEndActionLine) {
                r->noteBegins();  // every step begun so far is the routine's
                r->endActionLine = static_cast<long>(lines - 1);
                r->beginsBeforeEndAction = r->begins.size();
                r->motionsBeforeEndAction = r->log != nullptr ? r->log->motions.size() : 0;
            }
        }
        if (!r->stopped && lines > r->linesAtLastPace + kNoPaceLineLimit && r->onNoPace) {
            r->stopped = true;
            r->onNoPace();
        }
    }
};

/// A motion's kind by its name.
[[nodiscard]] inline StepKind kindOfMotion(std::string_view name) noexcept {
    if (name == "TurnTo") {
        return StepKind::Turn;
    }
    if (name == "CurveTo" || name == "ReverseCurveTo") {
        return StepKind::Curve;
    }
    if (name == "DriveBrake") {
        return StepKind::Brake;
    }
    if (name == "HoldPose") {
        return StepKind::Hold;
    }
    if (name == "StrafeTo") {
        return StepKind::Strafe;
    }
    return StepKind::Drive;
}

/// A mechanism outcome word's mark (UNCONFIRMED in its own colour, never green — RP13).
[[nodiscard]] inline Mark markOfOutcome(std::string_view w) noexcept {
    if (w == "SUCCEEDED") {
        return Mark::Ok;
    }
    if (w == "UNCONFIRMED") {
        return Mark::Unconfirmed;
    }
    if (w == "CANCELLED") {
        return Mark::Cut;
    }
    return Mark::Failed;
}

/// A MECH line's mechanism name and, when it is a step's verdict, its outcome word ("" otherwise).
/// `stepDidNothing` is set for "'x' is not on this robot ... the step did nothing" and its two
/// siblings (no slot to attribute to).
inline bool parseMechLine(std::string_view line, std::string& name, std::string& outcome, bool& stepDidNothing) {
    name.clear();
    outcome.clear();
    stepDidNothing = false;
    if (line.size() < 3 || line[0] != '\'') {
        return false;
    }
    const std::size_t close = line.find('\'', 1);
    if (close == std::string_view::npos) {
        return false;
    }
    name = std::string{line.substr(1, close - 1)};
    if (line.find("the step did nothing") != std::string_view::npos) {
        stepDidNothing = true;
        outcome = "ABSENT";
        return true;
    }
    if (line.substr(close, 3) != "': ") {
        return false;
    }
    const std::string_view rest = line.substr(close + 3);
    const std::size_t sp = rest.find(' ');
    const std::string_view word = rest.substr(0, sp);
    static constexpr std::string_view kOutcomes[] = {"SUCCEEDED", "UNCONFIRMED", "TIMED_OUT", "CANCELLED",
                                                     "STALLED",   "ABSENT",      "NOT_HOMED", "REFUSED"};
    for (const std::string_view o : kOutcomes) {
        if (word == o) {
            outcome = std::string{o};
            return true;
        }
    }
    return false;
}

/// An RTN line's mark.
[[nodiscard]] inline Mark markOfRoutineLine(std::string_view l) noexcept {
    if (l.find("' STOPPED at step ") != std::string_view::npos || l.find(") FAILED: ") != std::string_view::npos) {
        return Mark::Failed;
    }
    if (l.find("landed at step") != std::string_view::npos || l.find(") done in ") != std::string_view::npos ||
        l.find(") started in ") != std::string_view::npos) {  // RMP1-K D4: a step that ended well
        return Mark::Ok;
    }
    if (l.find(": its skip target '") != std::string_view::npos) {
        return Mark::Amber;
    }
    return Mark::Grey;
}

/// A WALL line's mark: a refusal is AMBER whatever its step's count says (the brief's landmine 4).
[[nodiscard]] inline Mark markOfWallLine(std::string_view l) noexcept {
    if (l.find("REFUSED") != std::string_view::npos) {
        return Mark::Amber;
    }
    if (l.find("SEATED") != std::string_view::npos) {
        return Mark::Ok;
    }
    return Mark::Grey;
}

/// The PLACEHOLDER note for a step of `mechanism` on `robot`, or "".
[[nodiscard]] inline std::string placeholderFor(const PreviewRobot& robot, const std::string& mechanism) {
    for (const Provisional& p : robot.provisional) {
        if (mechanism == p.mechanism) {
            return std::string{"PLACEHOLDER number \xe2\x80\x94 this time is the declaration's "} + p.number + " (" + p.ha +
                   "); " + p.who;
        }
    }
    if (robot.table == nullptr) {
        return "";
    }
    namespace mech = shulib::mechanism;
    for (std::size_t i = 0; i < robot.table->count && i < mech::kMaxMechanisms; ++i) {
        const mech::MechanismSpec& s = robot.table->specs[i];
        if (mechanism != std::string{mech::nameOf(s)}) {
            continue;
        }
        // ANY of its numbers still the library's starting value (HA-182) makes its time a placeholder; a
        // flywheel on fakes by kind also spins up on the host test plant's ASSUMED inertia
        // (test/fakes_by_kind.hpp's banner, INERTIA), which sets most of its time — a Flywheel's words
        // cannot state a kA (SpeedTuning has none), so only a spec built by hand escapes it.
        bool starting = false;
        bool assumedWheel = false;
        if (s.kind == mech::Kind::Position) {
            const mech::PositionSpec lib = mech::positionStartingValues(s);
            starting = s.position.kP == lib.kP || s.position.kV == lib.kV ||
                       s.position.maxVelocity == lib.maxVelocity || s.position.tolerance == lib.tolerance;
        } else if (s.kind == mech::Kind::Speed) {
            const mech::SpeedSpec lib = mech::speedStartingValues(s);
            starting = s.speed.kP == lib.kP || s.speed.kV == lib.kV;
            assumedWheel = !robot.defaultRig && !(s.speed.kA > 0.0);
        }
        if (starting || assumedWheel) {
            std::string why;
            if (starting) {
                why = "on the library's starting values (HA-182)";
            }
            if (assumedWheel) {
                why += std::string{starting ? ", " : ""} + "with an ASSUMED " + trimmed(tester_rig::kAssumedWheelTauS) +
                       " s wheel time constant (a flywheel's declaration has no number for it)";
            }
            return "PLACEHOLDER numbers \xe2\x80\x94 this time is what its declaration predicts, " + why +
                   "; 'shulib Tune' sets the real ones on the robot";
        }
    }
    return "";
}

/// Every fact `robot`'s ledger still OWES, in the ledger's order: each a PLACEHOLDER on the host test plant.
[[nodiscard]] inline std::vector<std::string> owedFacts(const PreviewRobot& robot) {
    std::vector<std::string> owed;
    for (const shulib::bench::robot_one_facts::FactSummary& f : robot.ledger) {
        if (!f.typed) {
            owed.emplace_back(f.name);
        }
    }
    return owed;
}

/// The robot line (the page's first line: which robot, which drive, what is pretend).
[[nodiscard]] inline std::string robotLineFor(const PreviewRobot& robot) {
    std::string names;
    if (robot.table != nullptr) {
        for (std::size_t i = 0; i < robot.table->count && i < shulib::mechanism::kMaxMechanisms; ++i) {
            names += (i == 0 ? "" : ", ") + std::string{shulib::mechanism::nameOf(robot.table->specs[i])};
        }
    }
    std::string title = robot.title;
    for (char& c : title) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    std::string s = "Previewing " + title + "'s mechanisms (" + (names.empty() ? std::string{"none"} : names) + ")";
    s += robot.defaultRig ? " on the Auton Tester rig's fakes"
                          : " on fakes built from their declarations' own numbers (an assumed time constant where "
                            "a declaration states no kA)";
    s += ", on the host test plant's STAND-IN drive \xe2\x80\x94 a 12 in tank on tracking wheels with made-up gains, "
         "not this robot's drive (its track width is not in the code yet) \xe2\x80\x94 with NO wall sensors: every "
         "wall reset is refused 'no sensor', as the robot's program does today. A step on a mechanism this "
         "robot does not have stops the route there, as it would on the robot.";
    return s;
}

/// The Model before the run: what was asked.
[[nodiscard]] inline Model askedModel(const PreviewRobot& robot, const shulib::competition::AutonEntry& e,
                                      double period, const PreviewOptions& opt) {
    Model m;
    m.auton = e.name != nullptr ? e.name : "";
    m.robotKey = robot.key;
    m.robotLine = robotLineFor(robot);
    m.owed = owedFacts(robot);
    m.ledgerFile = robot.ledgerFile;
    m.kindWord = shulib::competition::autonKindWord(e.kind);
    m.period = period;
    m.park = e.endActionAt.value();
    m.stop = e.hardStopAt.value();
    m.stamp = opt.stamp;
    m.autonsHash = opt.autonsHash;
    const shulib::competition::StartPose start = e.effectiveStart();
    if (start.mode() == shulib::competition::StartPose::Mode::Field) {
        const shulib::math::Pose2d p = start.pose();
        m.startLine = "at its FIELD start " + poseText(p.x().value(), p.y().value(), p.heading().radians());
        if (opt.place) {
            m.notices.push_back("--place is ignored: this auton declares a FIELD start, and it is drawn there.");
        }
    } else if (opt.place) {
        m.startLine = "RELATIVE, placed with --place at " +
                      poseText(opt.placeX, opt.placeY, opt.placeDeg * 3.14159265358979323846 / 180.0);
    } else {
        m.placedAtCentre = true;
        m.startLine = "RELATIVE, placed at the field's centre facing 0\xc2\xb0";
        m.notices.push_back("This auton has a RELATIVE start and no --place was given: the robot is drawn starting at "
                            "the field's CENTRE facing 0\xc2\xb0. Give its real place with --place X,Y,DEG.");
    }
    return m;
}

/// The plant's start pose for `e` (the brief's P5: a FIELD start is placed there).
[[nodiscard]] inline shulib::math::Pose2d plantStartFor(const shulib::competition::AutonEntry& e, const PreviewOptions& opt) {
    const shulib::competition::StartPose start = e.effectiveStart();
    if (start.mode() == shulib::competition::StartPose::Mode::Field) {
        return start.pose();
    }
    if (opt.place) {
        return shulib::math::Pose2d{shulib::units::Length{opt.placeX}, shulib::units::Length{opt.placeY},
                                    shulib::math::Angle::degrees(opt.placeDeg)};
    }
    return shulib::math::Pose2d{};
}

/// The exit word of a motion boundary.
[[nodiscard]] inline std::string motionVerdict(const shulib::motion::CompletedMotion& c) {
    switch (c.exit) {
        case shulib::control::ExitReason::Settled: return "SETTLED";
        case shulib::control::ExitReason::TimedOut: return "TIMED_OUT";
        case shulib::control::ExitReason::Cancelled:
            if (c.abortFault != shulib::diag::FaultCode::None) {
                return "ABORTED";
            }
            return c.preempted ? "SUPERSEDED" : "CANCELLED";
        case shulib::control::ExitReason::Running: return "RUNNING";
    }
    return "?";
}

/// FOLD what was recorded into `m` (header). `r` is the tester's record, or nullptr when the no-pace
/// stop fired inside the run (runAuton never returned).
inline void foldRun(Model& m, const PreviewRobot& robot, const Recorder& rec, const MotionLog& log,
                    const shulib::hal::fake::FakeTelemetrySink& sink, const shulib::bench::TesterRunRecord* r) {
    const bool started = rec.armSample >= 0;  // the guard went live: something ran, so there is a path
    if (r != nullptr) {
        m.word = shulib::bench::testerResultWord(*r);
        if (r->threw) {
            m.threw = true;
            m.why = r->threwWhat;
        } else if (r->run.outcome == shulib::competition::AutonRunOutcome::Refused ||
                   r->run.outcome == shulib::competition::AutonRunOutcome::NotStarted) {
            m.why = r->run.why;
            m.refused = r->run.outcome == shulib::competition::AutonRunOutcome::Refused;
        }
        m.cut = r->run.guard.scoringCut;
        m.scoringEnded = r->run.guard.scoringEnded.value();
        m.endActionEnded = r->run.guard.endActionRan ? r->run.guard.endActionEnded.value() : 0.0;
    } else {
        m.word = "LOOPS WITHOUT LETTING TIME PASS";
    }
    if (!started || m.refused || rec.raw.empty()) {
        m.refused = true;  // nothing ran: no path, no table
        m.exitCode = 1;
        return;
    }
    // The arming instant: the result's own; else (the no-pace stop, a throw) the sample the guard went live at.
    const double armedAt = r != nullptr && !r->threw && r->run.started()
                               ? r->run.armedAt.value()
                               : rec.raw[static_cast<std::size_t>(std::max(0L, rec.armSample))].truth.t;
    m.armedAt = armedAt;
    constexpr double kHalfTick = 0.005;
    std::size_t base = 0;
    while (base + 1 < rec.raw.size() && rec.raw[base].truth.t < armedAt - kHalfTick) {
        ++base;
    }
    for (std::size_t i = base; i < rec.raw.size(); ++i) {
        const Recorder::Raw& w = rec.raw[i];
        m.samples.push_back(Sample{w.truth.t - armedAt, w.truth.x, w.truth.y, w.truth.theta, w.ex, w.ey});
    }
    const auto sampleAtClock = [&](double t) {
        std::size_t i = 0;
        while (i + 1 < m.samples.size() && m.samples[i].t + armedAt < t - kHalfTick) {
            ++i;
        }
        return i;
    };
    // A line logged after raw sample k was recorded (and before k+1) happened while the world was at k.
    const auto sampleOfLine = [&](std::size_t line) {
        const auto after = std::upper_bound(rec.raw.begin(), rec.raw.end(), line,
                                            [](std::size_t v, const Recorder::Raw& w) { return v < w.lines; });
        const auto k = static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, (after - rec.raw.begin()) - 1));
        return k >= base ? k - base : std::size_t{0};
    };
    // THE END ACTION (header) is decided by ORDER — what began after the guard's kEndActionLine —
    // never by time: a step that fails at once leaves no time between it and the end of scoring.
    const bool endActionSeen = rec.endActionLine >= 0;

    // ── the motions ──
    std::vector<Step> steps;
    for (std::size_t k = 0; k < log.motions.size(); ++k) {
        const shulib::motion::CompletedMotion& c = log.motions[k];
        if (c.startTime.value() < armedAt - kHalfTick) {
            continue;
        }
        Step s;
        s.kind = kindOfMotion(c.name);
        s.what = c.name;
        s.t0 = c.startTime.value() - armedAt;
        s.t1 = c.endTime.value() - armedAt;
        s.s0 = sampleAtClock(c.startTime.value());
        s.s1 = sampleAtClock(c.endTime.value());
        s.hasTarget = c.hasPathData;
        s.tx = c.targetPose.x().value();
        s.ty = c.targetPose.y().value();
        s.tth = c.targetPose.heading().radians();
        s.verdict = motionVerdict(c);
        s.mark = s.verdict == "SETTLED" ? Mark::Ok
                 : (s.verdict == "CANCELLED" || s.verdict == "SUPERSEDED") ? Mark::Cut
                                                                            : Mark::Failed;
        s.endAction = endActionSeen && k >= rec.motionsBeforeEndAction;
        s.openAtThrow = k < log.duringThrow.size() && log.duringThrow[k] != 0;
        steps.push_back(s);
    }
    // ── the mechanism steps, attributed by NAME (header) ──
    std::vector<Step> mechs;
    for (std::size_t j = 0; j < rec.begins.size(); ++j) {
        const Recorder::Begin& b = rec.begins[j];
        if (b.sample < base) {
            continue;
        }
        Step s;
        s.kind = StepKind::Mechanism;
        s.what = rec.svc->slotPtr(b.slot)->name();
        s.s0 = b.sample - base;
        s.t0 = m.samples[s.s0].t;
        s.s1 = m.samples.size() - 1;
        s.t1 = m.samples.back().t;
        s.verdict = "no verdict";
        s.mark = Mark::Grey;
        s.placeholder = placeholderFor(robot, s.what);
        s.endAction = endActionSeen && j >= rec.beginsBeforeEndAction;
        mechs.push_back(s);
    }
    std::vector<char> closed(mechs.size(), 0);
    std::vector<char> endedByUnwind(mechs.size(), 0);  // its verdict line came during an unwind (header)
    // The no-pace stop's flood (every line since the last pace) is the page's loop section, not its log.
    const std::size_t lineEnd = r != nullptr ? static_cast<std::size_t>(sink.size()) : rec.linesAtLastPace;
    for (std::size_t i = rec.firstLine; i < lineEnd; ++i) {
        const auto& e = sink.at(static_cast<int>(i));
        const std::size_t si = sampleOfLine(i);
        LogLine l{m.samples[si].t, si, e.subsystem, e.message, Mark::Grey};
        if (e.subsystem == "RTN") {
            l.mark = markOfRoutineLine(e.message);
            m.routineLines.push_back(l);
        } else if (e.subsystem == "WALL") {
            l.mark = markOfWallLine(e.message);
            m.wallLines.push_back(l);
        } else if (e.subsystem == "MECH") {
            std::string name;
            std::string outcome;
            bool nothing = false;
            if (parseMechLine(e.message, name, outcome, nothing)) {
                std::size_t hit = mechs.size();
                if (!nothing) {
                    for (std::size_t k = 0; k < mechs.size(); ++k) {
                        if (closed[k] == 0 && mechs[k].what == name && mechs[k].s0 <= si) {
                            hit = k;
                            break;
                        }
                    }
                }
                if (hit == mechs.size()) {  // a step with no slot (not on this robot), or a stop step's refusal
                    Step s;
                    s.kind = StepKind::Mechanism;
                    s.what = name;
                    s.s0 = si;
                    s.t0 = m.samples[si].t;
                    s.endAction = endActionSeen && static_cast<long>(i) > rec.endActionLine;
                    mechs.push_back(s);
                    closed.push_back(0);
                    endedByUnwind.push_back(0);
                }
                Step& s = mechs[hit];
                closed[hit] = 1;
                endedByUnwind[hit] = std::binary_search(rec.mechLinesInUnwind.begin(), rec.mechLinesInUnwind.end(), i)
                                         ? 1
                                         : 0;
                s.s1 = si;
                s.t1 = m.samples[si].t;
                s.verdict = outcome;
                s.mark = markOfOutcome(outcome);
                s.sentence = e.message;
                s.placeholder = placeholderFor(robot, s.what);
            }
        }
        m.allLines.push_back(l);
    }
    for (std::size_t k = 0; k < mechs.size(); ++k) {
        Step& s = mechs[k];
        for (const Step& d : steps) {
            // Only a MOTION is driving: `steps` holds the mechanism steps already placed too, and another
            // mechanism running beside this one is not the robot driving (the verification round).
            if (d.kind != StepKind::Mechanism && d.s0 < s.s1 && d.s1 > s.s0) {
                s.background = true;  // the robot drove while it ran: its own span
            }
        }
        // Ended by the unwind and not driven during: the step the run threw inside (header).
        s.openAtThrow = endedByUnwind[k] != 0 && !s.background;
        steps.push_back(s);
    }
    // In the order they began. Steps begun on ONE pace (nothing paced between them) are ordered as well as
    // can be seen from below: a mechanism's START before a motion (a start form, then the drive it runs
    // during), and two mechanisms in their robot's list order (noteBegins reads the slots in order) —
    // which may not be the code's (`startRun(b)` then `startRun(a)`: a is numbered first; guide 16 says
    // so). The library's own lines carry the code's numbers; A2's listener ends the guess.
    std::stable_sort(steps.begin(), steps.end(), [](const Step& a, const Step& b) {
        if (a.s0 != b.s0) {
            return a.s0 < b.s0;
        }
        return a.kind == StepKind::Mechanism && b.kind != StepKind::Mechanism;
    });
    int n = 0;
    for (Step& s : steps) {
        s.number = s.endAction ? 0 : ++n;
    }
    m.steps = std::move(steps);

    // ── what the page says on top ──
    for (const Step& s : m.steps) {
        if (s.sentence.find("is not on this robot") != std::string::npos) {
            std::string title = robot.title;
            for (char& c : title) {
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            }
            m.notices.push_back("Previewing " + title + "'s mechanisms: a step on any other mechanism ('" + s.what +
                                "' here) stops the route, as it would on " + robot.title + ".");
            break;
        }
    }
    if (m.threw) {
        // WHERE it threw (header): inside the motion the unwind cut; else, from inside a pace, during a
        // wait; else after the last step that began.
        int last = 0;
        const Step* inside = nullptr;
        for (const Step& s : m.steps) {
            last = std::max(last, s.number);
            if (s.openAtThrow) {
                inside = &s;
            }
        }
        const std::string afterLast = "after step " + std::to_string(last) + " (counted from below)";
        m.notices.push_back(inside != nullptr
                                ? "PROGRAM ERROR inside " +
                                      (inside->endAction ? std::string{"the park (the end action)"}
                                                         : "step " + std::to_string(inside->number) + " (counted from below)")
                            : rec.inPace ? "PROGRAM ERROR during a wait, " + afterLast
                                         : "PROGRAM ERROR " + afterLast);
    }
    fieldCheck(m);
    m.exitCode = (m.threw || m.through || r == nullptr) ? 1 : 0;
}

/// The no-pace stop's sentence (header): "after step N the auton keeps looping ...".
inline void foldLoop(Model& m, const Recorder& rec) {
    m.looped = true;
    m.loopLines = static_cast<std::size_t>(rec.sink->size()) - rec.linesAtLastPace;
    for (std::size_t i = rec.linesAtLastPace;
         i < static_cast<std::size_t>(rec.sink->size()) && m.loopHead.size() < kLoopLinesKept; ++i) {
        const auto& e = rec.sink->at(static_cast<int>(i));
        m.loopHead.push_back("[" + e.subsystem + "] " + e.message);
    }
    // The routine's own STOPPED line gives the code's step number — the last one before the loop, which
    // is usually the first line of the flood itself (the stop is logged after the last pace); else the
    // last step counted from below.
    std::string after;
    const auto stoppedAt = [&after](const std::string& text) {
        const std::size_t at = text.find("' STOPPED at step ");
        if (at != std::string::npos) {
            const std::size_t from = at + std::strlen("' STOPPED at step ");
            const std::size_t to = text.find(' ', from);
            after = "step " + text.substr(from, to - from);
        }
    };
    for (const LogLine& l : m.routineLines) {
        stoppedAt(l.text);
    }
    for (const std::string& l : m.loopHead) {
        if (l.rfind("[RTN] ", 0) == 0) {
            stoppedAt(l);
        }
    }
    if (after.empty()) {
        int last = 0;
        for (const Step& s : m.steps) {
            last = std::max(last, s.number);
        }
        after = "step " + std::to_string(last) + " (counted from below)";
    }
    m.loopSentence = "after " + after +
                     " the auton keeps looping without letting time pass; on the robot it would sit there until "
                     "the field disables it, and its park would never run (" +
                     std::to_string(m.loopLines) + " lines logged with no time passing).";
    m.exitCode = 1;
}

/// RUN ONE ENTRY and fold it (header). `index` < reg.size().
[[nodiscard]] inline Model previewEntry(const PreviewRobot& robot, const shulib::competition::AutonRegistry& reg,
                                        std::size_t index, const PreviewOptions& opt) {
    const shulib::competition::AutonEntry& e = reg.at(index);
    Model m = askedModel(robot, e, reg.periodFor(e).value(), opt);
    shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
    cfg.plant.initialPose = plantStartFor(e, opt);
    std::unique_ptr<tester_rig::FakesByKind> fakes;
    tester_rig::RigMechanisms binding{};
    if (!robot.defaultRig) {
        SHULIB_PRECONDITION(robot.table != nullptr, "route preview: a robot with no mechanism list");
        fakes = std::make_unique<tester_rig::FakesByKind>(*robot.table);
        binding = fakes->binding();
    }
    auto stack = std::make_unique<tester_rig::TesterStack>(standInDrive(), 1, 1.0, cfg, opt.degradation, 50, false,
                                                           binding);
    Recorder rec;
    rec.inner = stack->probe.inner;
    rec.h = &stack->rig.h;
    rec.loc = &stack->rig.loc;
    rec.svc = &stack->mechanisms;
    rec.guard = &stack->guard;
    rec.sink = &stack->sink;
    stack->probe.inner = &rec;
    MotionLog log;
    rec.log = &log;
    SHULIB_PRECONDITION(stack->chassis.scheduler().boundaryObserver() == nullptr,
                        "route preview: the scheduler's boundary observer is taken");
    stack->chassis.scheduler().setBoundaryObserver(&log);
    rec.onNoPace = [&] {
        Model lm = m;
        foldRun(lm, robot, rec, log, stack->sink, nullptr);
        foldLoop(lm, rec);
        const bool written = opt.onLoop ? opt.onLoop(lm) : true;  // no writer: nothing was asked for
        std::fflush(nullptr);
        // Inside library code that must not throw: nothing is unwound (header). A page that could not be
        // written is the tool's failure (3), never the drawn loop's 1.
        std::_Exit(written ? kExitProblem : kExitToolFailed);
    };
    stack->sink.setOnLine(&Recorder::onLine, &rec);
    rec.attach();
    if (opt.onBuilt) {
        opt.onBuilt(*stack, rec.inner);
    }
    const shulib::bench::TesterRunRecord& r = stack->run(reg, static_cast<int>(index));
    stack->sink.setOnLine(nullptr, nullptr);
    stack->chassis.scheduler().setBoundaryObserver(nullptr);
    foldRun(m, robot, rec, log, stack->sink, &r);
    return m;
}

/// The Model of an auton the REGISTRY refused (it was never registered): its sentence, nothing run.
[[nodiscard]] inline Model refusedModel(const PreviewRobot& robot, const std::string& name, const std::string& sentence,
                                        const PreviewOptions& opt) {
    Model m;
    m.auton = name;
    m.robotKey = robot.key;
    m.robotLine = robotLineFor(robot);
    m.owed = owedFacts(robot);
    m.ledgerFile = robot.ledgerFile;
    m.kindWord = "refused";
    m.stamp = opt.stamp;
    m.autonsHash = opt.autonsHash;
    m.startLine = "nowhere: the program refuses this auton, so it is not on the brain's menu";
    m.refused = true;
    m.word = "REFUSED";
    m.why = sentence;
    m.exitCode = 1;
    return m;
}

/// A file name for `auton`: letters, digits, space, '-', '_' and '.' kept, anything else '_'.
[[nodiscard]] inline std::string pageFileName(const std::string& auton) {
    std::string f;
    for (const char c : auton) {
        const bool keep = std::isalnum(static_cast<unsigned char>(c)) != 0 || c == ' ' || c == '-' || c == '_' || c == '.';
        f += keep ? c : '_';
    }
    if (f.empty() || f[0] == '.') {
        f = "_" + f;
    }
    return f + ".html";
}

/// The page's path with no --out; a robot but the first goes under its key (the robots share auton names).
[[nodiscard]] inline std::string defaultPagePath(std::span<const PreviewRobot> robots, const PreviewRobot& robot,
                                                 const std::string& auton) {
    const bool first = !robots.empty() && std::string_view{robot.key} == robots[0].key;
    return std::string{"build/route_preview/"} + (first ? "" : std::string{robot.key} + "/") + pageFileName(auton);
}

/// Write `m`'s page to `path` and its table to `out`; false (said on `err`) if the page cannot be written.
/// The page goes to `path`.part first and is renamed into place: it is whole, or it is not there.
inline bool writeOutputs(const Model& m, const std::string& path, std::ostream& out, std::ostream& err) {
    out << renderText(m);
    std::error_code ec;
    const std::filesystem::path p{path};
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
    const std::string part = path + ".part";
    std::ofstream f{part, std::ios::binary | std::ios::trunc};
    if (!f) {
        err << "route preview: could not write the page to " << path << "\n";
        return false;
    }
    f << renderHtml(m);
    f.close();
    if (!f) {
        err << "route preview: could not finish writing the page to " << path << "\n";
        std::filesystem::remove(part, ec);
        return false;
    }
    std::filesystem::rename(part, p, ec);
    if (ec) {
        err << "route preview: could not put the page in place at " << path << " (" << ec.message() << ")\n";
        std::filesystem::remove(part, ec);
        return false;
    }
    out << "page: " << std::filesystem::absolute(p, ec).string() << "\n";
    out.flush();
    return true;
}

/// The usage line.
inline constexpr const char* kUsage =
    "usage: shulib_route_preview \"<auton name>\" [--robot two|one] [--place X,Y,DEG] [--out FILE] "
    "[--stamp TEXT] [--autons-hash HEX] | --list   (each flag's value after a space or an '=')";

/// THE PROGRAM (header). `robots` defaults to kRobots; a test passes its own (a fixture robot).
/// Exit codes: kExitDrawn, kExitProblem, kExitUsage, kExitToolFailed (above). The page of a previous run
/// of the same auton is REMOVED before the run, and the new one is written whole or not at all (a
/// temporary file renamed into place): a run that is killed leaves no page that looks like its own.
[[nodiscard]] inline int runPreview(std::span<const std::string_view> args, std::ostream& out, std::ostream& err,
                                    std::span<const PreviewRobot> robots = std::span<const PreviewRobot>{kRobots}) {
    std::string name;
    bool haveName = false;
    bool list = false;
    std::string robotKey = robots.empty() ? std::string{} : std::string{robots[0].key};
    std::string outPath;
    PreviewOptions opt;
    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string_view a = args[i];
        // `--flag=value` is `--flag value` (the guide writes `--place=-48,-48,90`: a value that starts
        // with '-' must be glued on for the launcher's argument parser).
        bool glued = false;
        std::string gluedValue;
        if (a.size() > 2 && a.substr(0, 2) == "--" && a.find('=') != std::string_view::npos) {
            glued = true;
            gluedValue = std::string{a.substr(a.find('=') + 1)};
            a = a.substr(0, a.find('='));
        }
        const auto value = [&](std::string& into) {
            if (glued) {
                into = gluedValue;
                return true;
            }
            if (i + 1 >= args.size()) {
                err << "route preview: " << a << " needs a value\n" << kUsage << "\n";
                return false;
            }
            into = std::string{args[++i]};
            return true;
        };
        if (a == "--list") {
            if (glued) {
                err << "route preview: --list takes no value\n" << kUsage << "\n";
                return 2;
            }
            list = true;
        } else if (a == "--robot") {
            if (!value(robotKey)) {
                return 2;
            }
        } else if (a == "--out") {
            if (!value(outPath)) {
                return 2;
            }
        } else if (a == "--stamp") {
            if (!value(opt.stamp)) {
                return 2;
            }
        } else if (a == "--autons-hash") {
            if (!value(opt.autonsHash)) {
                return 2;
            }
        } else if (a == "--place") {
            std::string v;
            if (!value(v)) {
                return 2;
            }
            double x = 0.0;
            double y = 0.0;
            double d = 0.0;
            char tail = 0;
            if (std::sscanf(v.c_str(), "%lf,%lf,%lf%c", &x, &y, &d, &tail) != 3 || !std::isfinite(x) ||
                !std::isfinite(y) || !std::isfinite(d)) {
                err << "route preview: --place takes X,Y,DEG (inches, inches, degrees), e.g. --place -48,-48,90\n";
                return 2;
            }
            opt.place = true;
            opt.placeX = x;
            opt.placeY = y;
            opt.placeDeg = d;
        } else if (a.size() >= 2 && a.substr(0, 2) == "--") {
            err << "route preview: unknown flag " << a << "\n" << kUsage << "\n";
            return 2;
        } else if (!haveName) {
            name = std::string{a};
            haveName = true;
        } else {
            err << "route preview: one auton name at a time (put a name with spaces in quotes)\n" << kUsage << "\n";
            return 2;
        }
    }
    const PreviewRobot* robot = nullptr;
    for (const PreviewRobot& r : robots) {
        if (robotKey == r.key) {
            robot = &r;
        }
    }
    if (robot == nullptr) {
        err << "route preview: no robot '" << robotKey << "' — the robots are:";
        for (const PreviewRobot& r : robots) {
            err << " " << r.key;
        }
        err << "\n";
        return 2;
    }
    SHULIB_PRECONDITION(robot->registerAutons != nullptr, "route preview: a robot with no autons file");
    shulib::competition::AutonRegistry reg{shulib::bench::kAutonPeriods, shulib::competition::OnRefusal::DropEntry};
    robot->registerAutons(reg);
    const auto listAll = [&](std::ostream& to) {
        to << robot->title << "'s autons (" << robot->autonsFile << "):\n";
        for (std::size_t i = 0; i < reg.size(); ++i) {
            to << "  '" << reg.at(i).name << "'\n";
        }
        for (std::size_t i = 0; i < reg.refusedCount() && i < shulib::competition::kMaxAutonRefusals; ++i) {
            to << "  REFUSED: " << reg.refusal(i) << "\n";
        }
        if (reg.refusedCount() > shulib::competition::kMaxAutonRefusals) {
            to << "  ... and " << reg.refusedCount() - shulib::competition::kMaxAutonRefusals
               << " more refused than the registry keeps the names of\n";
        }
    };
    if (list || !haveName) {
        listAll(list ? out : err);
        if (!list) {
            err << kUsage << "\n";
        }
        return list ? 0 : 2;
    }
    if (outPath.empty()) {
        outPath = defaultPagePath(robots, *robot, name);
    }
    // The last run's page of this name goes first: a run killed from outside (the launcher's 300 s
    // bound, a signal) must not leave it looking like this one's.
    const auto removeOldPage = [&outPath] {
        std::error_code ec;
        std::filesystem::remove(outPath, ec);
    };
    // EXACT name only: never a nearest match, never case-folded (the brief's D7, RM12).
    for (std::size_t i = 0; i < reg.size(); ++i) {
        if (std::string_view{reg.at(i).name} == name) {
            removeOldPage();
            opt.onLoop = [&](const Model& lm) { return writeOutputs(lm, outPath, out, err); };
            const Model m = previewEntry(*robot, reg, i, opt);
            if (!writeOutputs(m, outPath, out, err)) {
                return kExitToolFailed;
            }
            return m.exitCode;
        }
    }
    // A REFUSED entry is not registered: its name is only inside its sentence (the brief's R12b).
    const std::string prefix = "auton '" + name + "': ";
    for (std::size_t i = 0; i < reg.refusedCount() && i < shulib::competition::kMaxAutonRefusals; ++i) {
        const std::string sentence = reg.refusal(i);
        if (sentence.rfind(prefix, 0) == 0) {
            removeOldPage();
            const Model m = refusedModel(*robot, name, sentence, opt);
            if (!writeOutputs(m, outPath, out, err)) {
                return kExitToolFailed;
            }
            return kExitProblem;
        }
    }
    if (reg.refusedCount() > shulib::competition::kMaxAutonRefusals) {
        err << "route preview: no auton named '" << name << "' \xe2\x80\x94 and "
            << reg.refusedCount() - shulib::competition::kMaxAutonRefusals
            << " more autons were refused than the registry keeps the names of; fix the refusals listed first\n";
        listAll(err);
        return 2;
    }
    err << "route preview: no auton named '" << name << "' in " << robot->autonsFile << " \xe2\x80\x94 registered:";
    for (std::size_t i = 0; i < reg.size(); ++i) {
        err << (i == 0 ? " '" : ", '") << reg.at(i).name << "'";
    }
    err << "\n";
    for (std::size_t i = 0; i < reg.refusedCount() && i < shulib::competition::kMaxAutonRefusals; ++i) {
        err << "  REFUSED: " << reg.refusal(i) << "\n";
    }
    return 2;
}

/// THE PROGRAM's main, minus argv (test/route_preview_main.cpp calls only this): runPreview, with a
/// failure of the preview ITSELF — an exception escaping it — said in words and exited kExitToolFailed,
/// never an abort and never mistaken for a usage error. (The auton's own throws never get here: the run
/// catches them, as the tester does, and draws a PROGRAM ERROR.)
[[nodiscard]] inline int runProgram(std::span<const std::string_view> args, std::ostream& out, std::ostream& err,
                                    std::span<const PreviewRobot> robots = std::span<const PreviewRobot>{kRobots}) {
    try {
        return runPreview(args, out, err, robots);
    } catch (const std::exception& e) {
        err << "route preview: the preview itself failed: " << e.what() << "\n";
        return kExitToolFailed;
    }
}

}  // namespace route_preview
