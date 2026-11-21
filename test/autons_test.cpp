// autons_test.cpp — the team's autons (src/team/autons.cpp, linked into this suite by
// test/CMakeLists.txt) registered and RUN on the host test plant through the Auton Tester's own
// run sequence, runTesterOnce(): the real AbortWatch between the plant's pacer and the RunGuard,
// 50 idle ticks, the 3 s countdown, the quiet tick, then competition::runAuton — exactly what
// "shulib Auton Tester" does when RUN is touched (chunk COMP-t, 2026-09-28; the brief's §5 tests
// (1)–(5), (3b), (3b′), (3c) and (4b); and (1b), added by the chunk's review on 2026-09-29: EVERY
// registered auton, not only the two named here). Since the mechanism core's MECH-A5 (2026-10-29)
// the rig is the program's whole chain: a mechanism list bound to a live
// MechanismService between the guard and the watch, and its six route ids registered, so a
// mechanism step or a mechanism route in any auton here runs, and is judged, as on the bench
// (test/auton_tester_rig.hpp; the claws auton and route: test/robot_two_claws_example_test.cpp).
//
// Graded on PLANT GROUND TRUTH (the A2 truth integrator) AND on the record the tester builds (its
// END estimate, its fault counts, its word), because the record is what the students read.
// Host test plant numbers — instant motors, a tank that never skids; none of this has run on a
// robot (worksheet Station M). The weak-side cases are the brief's re-prediction after TANK1 Part
// A (2026-09-28): a 5 % weak side now FINISHES (the library steers the tank back onto its line),
// and the honest stop is shown at a 35 % weak side, where it still cannot arrive.
//
// SINCE ROB1-BT (2026-11-16) robot two is robot one's TWIN: its file registers robot one's list — the team's
// 40 in, the acceptance run and the skills route MIRRORED (src/team/skills_route.cpp), every one a SKILLS
// entry on the 57 / 59 s schedule (test/robot_twin_test.cpp holds the two lists equal). The cases below
// find each entry BY NAME.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "auton_tester_rig.hpp"
#include "mirrored_pair_check.hpp"
#include "source_pins.hpp"
#include "motion_test_rig.hpp"
#include "programs/auton_tester_panel.hpp"
#include "programs/auton_tester_run.hpp"
#include "team/autons.hpp"
#include "team/skills_route.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::kAutonPeriods;
using shulib::bench::registerAutons;
using shulib::bench::TesterRunRecord;
using shulib::bench::testerResultWord;
using shulib::competition::AutonKind;
using shulib::competition::AutonPeriods;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::competition::StartPose;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using tester_rig::TesterStack;

namespace {

constexpr double kDeg = Angle::kPi / 180.0;

/// Robot two's drive as the plant sees it: a 12 in tank (the rig's stand-in).
const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

AutonRegistry teamRegistry() {
    AutonRegistry reg{kAutonPeriods};
    registerAutons(reg);
    return reg;
}

/// (1d)'s rule (RMP1-C D6) over `reg`'s MIRRORED entries, each found in `routes` by name and checked as its route's
/// answer says: Yes the gap, No skipped by name; not answered with a field start: `unanswered` (the caller fails it).
struct RouteVerdict {
    std::string name;
    bool unanswered = false;
    mirror_pair::PairVerdict v;
};
std::vector<RouteVerdict> checkTeamRoutes(const AutonRegistry& reg, std::span<const shulib::bench::TeamRoute> routes) {
    using shulib::bench::Together;
    std::vector<RouteVerdict> out;
    for (std::size_t i = 0; i < reg.size(); ++i) {
        const shulib::competition::AutonEntry& e = reg.at(i);
        if (e.mirror == nullptr) {
            continue;
        }
        const shulib::bench::TeamRoute* route = nullptr;
        for (const shulib::bench::TeamRoute& r : routes) {
            route = std::string_view{r.entry().name} == e.name ? &r : route;
        }
        REQUIRE_MESSAGE(route != nullptr, "auton '", e.name, "' is mirrored but is not one of the route builder's routes");
        mirror_pair::PairOptions o;
        o.kin = &tank();
        o.period = reg.periodFor(e);
        o.together = route->together == Together::Yes;
        o.answered = route->together != Together::NotAnswered;
        const bool field = e.effectiveStart().mode() == StartPose::Mode::Field;
        out.push_back({e.name, field && !o.answered, mirror_pair::checkMirroredPair(e, o)});
    }
    return out;
}

/// A TEST route (G4) on the team lead's first real start (-36, 54.076) facing -90, standing still: mirrored, its
/// square and robot two's are 4.35 in apart.
shulib::chassis::RoutineResult standStill(shulib::competition::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "planted"};
    return r.result();
}
shulib::competition::AutonEntry plantedNearTheDiagonal() {
    shulib::competition::AutonEntry e{};
    e.name = "planted";
    e.kind = AutonKind::Skills;
    e.routine = &standStill;
    e.start = StartPose::field(Length{-36.0}, Length{54.076}, Angle::degrees(-90.0));
    e.endActionAt = Time{57.0};
    e.hardStopAt = Time{59.0};
    return e;
}

/// The index of the entry named `name` (FAILS the case when there is none).
int indexOf(const AutonRegistry& reg, std::string_view name) {
    for (std::size_t i = 0; i < reg.size(); ++i) {
        if (std::string_view{reg.at(i).name} == name) {
            return static_cast<int>(i);
        }
    }
    FAIL("no auton named ", std::string{name});
    return -1;
}

/// The 40 in's index, and the acceptance run's.
int forty(const AutonRegistry& reg) { return indexOf(reg, "40in forward"); }
int acceptance(const AutonRegistry& reg) { return indexOf(reg, shulib::competition::kAcceptanceRunName); }

bool says(const std::string& text, std::string_view needle) {
    return text.find(needle) != std::string::npos;
}

std::string joined(const shulib::bench::TesterRunLines& lines) {
    std::string all;
    for (int i = 0; i < lines.count; ++i) {
        all += lines.line[i];
        all += '\n';
    }
    return all;
}

/// The IMU reads truth + `offsetDeg` — it booted where the robot was placed and then drifted, or
/// the robot was placed facing somewhere else (competition_runner_test.cpp's DisagreeingImu shape).
struct OffsetImu final : shulib::sim::DegradationModel {
    explicit OffsetImu(double offset) : offsetDeg{offset} {}
    [[nodiscard]] Angle imuHeading(Angle truth, Time /*now*/, shulib::sim::Rng& /*rng*/) override {
        return truth + Angle::degrees(offsetDeg);
    }
    double offsetDeg;
};

}  // namespace

// (1) Bug caught: the students' list drifting from what the programs expect — an entry dropped or
// reordered (the menu lists them in this order), a schedule edited to something that no longer
// leaves the end action its runway, a kind or start left unset, an end action given where the
// file says the brake runs, or the periods changed away from the game manual's.
TEST_CASE("autons (1): registerAutons registers robot one's list -- the 40 in, the acceptance run, the skills route MIRRORED -- every one SKILLS") {
    CHECK(kAutonPeriods.match.value() == 30.0);
    CHECK(kAutonPeriods.skills.value() == 60.0);
    static_assert(kAutonPeriods.match.value() == 30.0 && kAutonPeriods.skills.value() == 60.0,
                  "kAutonPeriods is a constant expression with the manual's 30 s and 60 s");

    const AutonRegistry reg = teamRegistry();
    const std::span<const shulib::bench::TeamRoute> routes = shulib::bench::teamRoutes();
    REQUIRE_MESSAGE(reg.size() == 2 + routes.size(),
                    "src/team/autons.cpp registers ", reg.size(), " autons and this test pins the 40 in, the acceptance run "
                    "and the route builder's ", routes.size(), " routes. Added one by hand? Pin its name, kind and schedule "
                    "here too (and in robot one's file: the twins register the same list; the route builder counts on two).");
    int skills = 0;
    for (std::size_t i = 0; i < routes.size(); ++i) {  // RMP1-C D2: every route, in order, mirrored only where marked
        const auto& e = reg.at(2 + i);
        CAPTURE(e.name);
        CHECK(std::string_view{e.name} == routes[i].entry().name);
        CHECK(e.kind == AutonKind::Skills);
        CHECK(e.mirror == (routes[i].mirror == shulib::bench::Mirror::ForRobotTwo ? &shulib::competition::override2026::kSkillsMirror
                                                                                   : nullptr));
        CHECK(e.endActionAt.value() == 57.0);
        CHECK(e.hardStopAt.value() == 59.0);
        skills += std::string_view{e.name} == "skills" ? 1 : 0;
    }
    CHECK(skills == 1);  // the competition's skills route is always there (the route builder keeps it)

    const auto& fortyIn = reg.at(0);
    CHECK(std::string_view{fortyIn.name} == "40in forward");
    CHECK(fortyIn.kind == AutonKind::Skills);
    CHECK(fortyIn.mirror == nullptr);
    CHECK(fortyIn.routine != nullptr);
    CHECK(fortyIn.route == nullptr);
    CHECK(fortyIn.start.mode() == StartPose::Mode::Relative);
    CHECK(fortyIn.endActionAt.value() == 57.0);
    CHECK(fortyIn.hardStopAt.value() == 59.0);
    CHECK(fortyIn.endAction == nullptr);  // the brake: "endAction is left out"

    const auto& accept = reg.at(1);
    CHECK(std::string_view{accept.name} == shulib::competition::kAcceptanceRunName);
    CHECK(accept.kind == AutonKind::Skills);
    CHECK(accept.mirror == nullptr);
    CHECK(accept.routine != nullptr);
    CHECK(accept.endActionAt.value() == 57.0);
    CHECK(accept.hardStopAt.value() == 59.0);
    CHECK(accept.effectiveStart().mode() == StartPose::Mode::Relative);
    CHECK(reg.periodFor(fortyIn).value() == 60.0);
}

// (1b) Bug caught (COMP-t's review, 2026-09-29): the claim — in the students' file, the guide and the
// README — that the host suite runs EVERY auton in the file, when it ran only the two it names. A
// student's next auton would never have run on the host test plant. EVERY registered entry, whatever
// the file grows to, runs here through the tester's own sequence on the clean plant, and must end the
// way the bench needs: not refused, no throw, never aborted, started, no hard stop, the end action run
// and succeeded, the drive at 0 V + Brake, robot two's mechanisms at their safe states (MECH-A5: the
// service is live, so a mechanism step that can never be met — a refusal at the STEP — fails here,
// before any upload), the guard and the watch let go. The word is printed for each (FINISHED is not
// required: an auton cut at its own endActionAt is legal); a run that FAILS here would fail on the
// bench.
TEST_CASE("autons (1b): EVERY registered auton runs through runTesterOnce and ends safe") {
    const AutonRegistry reg = teamRegistry();
    REQUIRE(reg.size() >= 1);
    for (std::size_t i = 0; i < reg.size(); ++i) {
        const std::string name{reg.at(i).name};
        CAPTURE(name);
        TesterStack s{tank()};
        const TesterRunRecord& r = s.run(reg, static_cast<int>(i));
        const std::string word{testerResultWord(r)};
        MESSAGE("auton '", name, "' on the host test plant: ", word);
        CHECK(word != "REFUSED");
        CHECK(word != "PROGRAM ERROR");
        CHECK(word != "NOT STARTED");
        CHECK(word != "HARD STOP");
        CHECK(word != "ABORTED");
        CHECK(word != "ABORT IGNORED");
        CHECK_FALSE(r.threw);
        CHECK(r.run.started());
        CHECK(r.run.guard.endActionRan);
        CHECK(r.run.guard.endActionSucceeded);
        CHECK_FALSE(r.run.guard.floorFired);
        CHECK(s.driveSafe());
        CHECK(s.mechanismsSafe());
        CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
        CHECK_FALSE(s.guard.running());
        CHECK_FALSE(s.watch.armed());
    }
}

// (1d) MIRROR1-A T9 (the MIRROR1 brief's §2.2, D11): every MIRRORED entry robot two registers — robot
// one's route, `registry.add(competition::mirrored(...))` — also runs TOGETHER with its unmirrored self
// on the host test plant (test/mirrored_pair_check.hpp: robot one from its start, robot two from the
// reflection, pace by pace), and their 15 in squares must never come within 6 in. Bug caught: a shared
// route that crosses the field's diagonal (or sends both robots to the centre goal at once) uploaded to
// both robots — a crash at the event, found here first, naming the moment and the motions. Also (MIRROR1-B,
// the team lead's D6 ruling: "Block it") a tank step that drives AND turns around, which each robot ends
// beside its target. A RELATIVE mirrored entry's gap is skipped by name (its field positions are wherever
// the drivers place it); its half-turn legs are still refused. Today's file (ROB1-BT) registers ONE mirrored
// entry, the skills route — RELATIVE until the route maker writes robot one's start, so its gap is skipped
// by name today; (1b) above runs every entry, mirrored ones included, through the tester's own sequence.
//
// RMP1-C D6: the gap is checked only for a route the team said BOTH robots run at the same time (Together::Yes); one
// they run one at a time is reported as not checked, by name (its half-turn legs still refused); a mirrored route
// with a field start and no answer FAILS here (the route builder asks before it exports one).
TEST_CASE("autons (1d): every MIRRORED entry passes the pair check — 6 in apart when run together, no tank half-turn leg") {
    const auto verdicts = checkTeamRoutes(teamRegistry(), shulib::bench::teamRoutes());
    for (const RouteVerdict& r : verdicts) {
        MESSAGE(r.v.sentence);
        CHECK_MESSAGE(!r.unanswered, "auton '", r.name, "': will both robots run it at the SAME time? Answer it in the route builder");
        CHECK((r.v.ok || !r.v.checked));
    }
    // none is fine: the skills route may be unmirrored (the team, 2026-11-21)
    MESSAGE("src/team/autons.cpp registers ", verdicts.size(), " mirrored auton(s); each checked as its route says");
}

// (1d') RMP1-C D6, the plumbing (1d) runs on: the route's answer read from the list. Bug caught: every mirrored route
// gap-checked (one the robots run one at a time failing for a meeting that never happens), none (two robots that
// would collide, uploaded to both), or a route nobody answered passing in silence. The team lead's real start, three
// ways: Yes fails (4.35 in apart), No is skipped by name, not answered is reported.
TEST_CASE("autons (1d'): the pair check follows each route's answer -- Yes checked, No skipped by name, unanswered reported") {
    using shulib::bench::Together;
    const struct {
        Together answer;
        const char* says;
    } cases[] = {{Together::Yes, "come within 6.0 in (4.35 in apart)"},
                 {Together::No, "do NOT run it at the same time (the route builder's answer)"},
                 {Together::NotAnswered, "nobody has said yet whether both robots run it at the same time"}};
    for (const auto& c : cases) {
        CAPTURE(static_cast<int>(c.answer));
        const shulib::bench::TeamRoute routes[] = {{&plantedNearTheDiagonal, shulib::bench::Mirror::ForRobotTwo, c.answer}};
        AutonRegistry reg{kAutonPeriods};
        shulib::bench::registerRoutes(reg, routes, shulib::bench::TeamRobot::Two);
        const auto v = checkTeamRoutes(reg, routes);
        REQUIRE(v.size() == 1);
        CAPTURE(v[0].v.sentence);
        CHECK(v[0].name == "planted");
        CHECK(says(v[0].v.sentence, c.says));
        CHECK(v[0].unanswered == (c.answer == Together::NotAnswered));
        CHECK(v[0].v.checked == (c.answer == Together::Yes));
        CHECK(v[0].v.ok == (c.answer != Together::Yes));
    }
}

// (1c) Bug caught (the MECH brief's U3, 2026-10-29): an auton built as `Routine r{ctx.chassis, ...}`
// — the form every auton here used until MECH-A5 — has NO mechanisms: its first `r.set(claw[...])`
// or `r.run(...)` is refused at the step ("Routine: a mechanism step needs the robot's mechanism
// service -- construct it as Routine r{ctx, "name"} in an auton ..."), so a student who copies it and
// adds a claw step gets an auton that refuses on the bench. The file the students
// copy from must build every Routine on the CONTEXT — the only spelling the guide teaches — and name
// robot two's mechanisms through its namespace, the one place the src build gate allows outside the
// table file (TEAM_FILE_ROBOT).
TEST_CASE("autons (1c): src/team/autons.cpp builds every Routine on the context, never ctx.chassis") {
    const std::string code = source_pins::stripComments(source_pins::readFile("src/team/autons.cpp"));
    const std::size_t routines = source_pins::count(code, "chassis::Routine r{");
    CHECK(routines >= 1);
    CHECK(source_pins::count(code, "chassis::Routine r{ctx, \"") == routines);
    CHECK(source_pins::count(code, "Routine r{ctx.chassis") == 0);
    CHECK(source_pins::count(code, "using namespace robot_two;") == 1);
    CHECK(source_pins::count(code, "#include \"robots/mechanism_table.hpp\"") == 1);
}

// (2) Bug caught: a schedule that is never really checked against the period it runs in. The same
// file registered into a registry whose SKILLS period is 58 s must THROW (a registry's default), and
// the sentence must name the auton — the first, the 40 in. (The two programs build their
// registries to DROP such an entry instead -- (2b) below; the team lead's ruling of 2026-10-02.)
TEST_CASE("autons (2): a period the 59 s hard stop does not fit is refused, naming the auton") {
    AutonRegistry reg{AutonPeriods{.match = Time{30.0}, .skills = Time{58.0}}};
    bool threw = false;
    std::string what;
    try {
        registerAutons(reg);
    } catch (const shulib::PreconditionError& e) {
        threw = true;
        what = e.what();
    }
    CHECK(threw);
    CHECK(says(what, "auton '40in forward'"));
    CHECK(reg.size() == 0);  // the first entry is refused: nothing half-registered
}

// (2b) Bug caught (T4 Part 3; the team lead's ruling of 2026-10-02 on COMP-b's C5, "drop just that
// auton"): the REAL students' file, registered the way both programs now register it (a registry
// that DROPS a refused entry), throwing anyway -- the boot, and with it driver control, would be
// lost to one bad schedule -- or stopping at the first refused entry, or losing an entry's name.
// With a 58 s skills period all three of the file's entries (each with its 59 s hard stop) are
// refused: all DROPPED, in the file's order, each named, nothing thrown, nothing registered.
TEST_CASE("autons (2b): the real file into a DROPPING registry -- each refused entry dropped by name") {
    AutonRegistry reg{AutonPeriods{.match = Time{30.0}, .skills = Time{58.0}},
                      shulib::competition::OnRefusal::DropEntry};
    CHECK_NOTHROW(registerAutons(reg));
    CHECK(reg.size() == 0);
    REQUIRE(reg.refusedCount() == 3);
    CHECK(says(reg.refusal(0), "auton '40in forward': hardStopAt 59.00 s is past its SKILLS period"));
    CHECK(says(reg.refusal(1), std::string{"auton '"} + shulib::competition::kAcceptanceRunName));
    CHECK(says(reg.refusal(2), "auton 'skills'"));
    // ...and with the game manual's periods, the same registry drops nothing.
    AutonRegistry real{kAutonPeriods, shulib::competition::OnRefusal::DropEntry};
    registerAutons(real);
    CHECK(real.refusedCount() == 0);
    CHECK(real.size() == teamRegistry().size());
}

// (3) Bug caught: the 40 in auton not running through the tester's path to the end — a schedule,
// a start or a guard wired wrong; an estimate and a truth that disagree; the result word or the
// fault count lying about a clean run. Also the baseline for (3b): on an EVEN drive the library
// does not steer (TSTR on no motion record).
TEST_CASE("autons (3): the 40 in auton through runTesterOnce FINISHES within 0.5 in of 40") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const int recordsBefore = s.sink.recordCount();
    const TesterRunRecord& r = s.run(reg, forty(reg));

    CHECK_FALSE(r.countdownAborted);
    CHECK_FALSE(r.threw);
    CHECK(r.cause == shulib::bench::AbortCause::None);
    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.run.routine.ok);
    CHECK(r.run.routine.completed == 1);
    CHECK(r.run.routine.steps == 1);
    CHECK(r.run.guard.endActionRan);
    CHECK(r.run.guard.endActionSucceeded);
    CHECK_FALSE(r.run.guard.scoringCut);
    CHECK_FALSE(r.run.guard.abandoned);
    CHECK(r.run.guard.pacesSeen > 0);
    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    CHECK(r.faultsAfter == r.faultsBefore);
    CHECK(r.lastFault == FaultCode::None);

    const Pose2d truth = s.rig.h.truePose();
    CHECK(std::abs(truth.x().value() - 40.0) < 0.5);
    CHECK(std::abs(truth.y().value()) < 0.5);
    CHECK(std::abs(truth.heading().degrees()) < 1.2);
    CHECK(r.endLive);
    CHECK(std::abs(r.endEstimate.x().value() - 40.0) < 0.5);
    CHECK(std::abs(r.endEstimate.y().value()) < 0.5);
    CHECK(std::abs(r.endEstimate.heading().degrees()) < 1.2);
    CHECK(s.driveSafe());

    int motionRecords = 0;
    int tstr = 0;
    s.tstrCount(recordsBefore, motionRecords, tstr);
    CHECK(motionRecords > 0);
    CHECK(tstr == 0);  // an even drive on a plant with no noise: nothing to steer out
}

// (3b) Bug caught (the re-prediction after TANK1 Part A, 2026-09-28): the PREDICTED first floor
// result on a drift like robot two's. A RIGHT side 5 % weak turns the tank right; TANK1 steers it
// back onto its line, so the 40 in FINISHES — word FINISHED, no fault this run, the END inside the
// arrival rule by truth AND by the record, the brake run. The TSTR clause is what makes this a test
// of the weak side at all: without it, a (3b) whose weak side was never wired would pass on a clean
// plant (the clean run, (3), has TSTR on no record). Measured when written: END (39.70, 0.01,
// 0.06 deg), T+1.85 s, TSTR on 81 of the run's 199 motion records.
TEST_CASE("autons (3b): the 40 in with a 5 % weak RIGHT side FINISHES, steered (TSTR)") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank(), /*weakWheel=*/1, /*weakScale=*/0.95};
    const int recordsBefore = s.sink.recordCount();
    const TesterRunRecord& r = s.run(reg, forty(reg));

    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    CHECK(r.run.routine.ok);
    CHECK(r.run.routine.completed == 1);
    CHECK(r.run.routine.steps == 1);
    CHECK(r.faultsAfter == r.faultsBefore);
    CHECK(r.lastFault == FaultCode::None);
    CHECK(says(joined(shulib::bench::testerRunLines(r, reg.at(static_cast<std::size_t>(forty(reg))))), "faults this run: none"));
    CHECK(r.run.guard.endActionRan);
    CHECK(r.run.guard.endActionSucceeded);

    const Pose2d truth = s.rig.h.truePose();
    CHECK(std::abs(truth.x().value() - 40.0) < 0.5);
    CHECK(std::abs(truth.y().value()) < 0.5);
    CHECK(std::abs(truth.heading().degrees()) < 1.2);
    CHECK(std::abs(r.endEstimate.x().value() - 40.0) < 0.5);
    CHECK(std::abs(r.endEstimate.y().value()) < 0.5);
    CHECK(std::abs(r.endEstimate.heading().degrees()) < 1.2);
    CHECK(s.driveSafe());

    int motionRecords = 0;
    int tstr = 0;
    s.tstrCount(recordsBefore, motionRecords, tstr);
    CHECK(tstr > 0);  // the library WAS steering: the weak side is real
    CHECK(tstr < motionRecords);
}

// (3b′) Bug caught: after TANK1 the tester must STILL report a real failure. A RIGHT side 35 %
// weak (TANK1's own honest-stop case, 7.3 points past the weakest side that finishes on the plant,
// x0.723) cannot arrive: ROUTINE STOPPED at step 1 (moveTo), exit TimedOut, ONE fault this run
// (MOTION_TIMEOUT), the END OUTSIDE the arrival rule — its distance to (40, 0) over 0.5 in — and
// the brake still run and the drive safe after. The END is bounded by the arrival rule, not by
// END |y|: an END-|y| bound would pin TANK1's miss shape (x0.70 ends at |y| 0.41). The left side
// is the mirror image. Measured when written: END (39.38, -0.93, 0.56 deg), 1.12 in from (40, 0),
// scoring T+5.01 s.
TEST_CASE("autons (3b'): the 40 in with a 35 % weak side is the honest ROUTINE STOPPED") {
    const AutonRegistry reg = teamRegistry();
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        TesterStack s{tank(), wheel, 0.65};
        const TesterRunRecord& r = s.run(reg, forty(reg));

        REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
        CHECK(std::string_view{testerResultWord(r)} == "ROUTINE STOPPED");
        CHECK_FALSE(r.run.routine.ok);
        CHECK(r.run.routine.stoppedAt == 1);
        CHECK(std::string_view{r.run.routine.stoppedName} == "moveTo");
        CHECK(r.run.routine.cause == shulib::chassis::RoutineStopCause::MotionFailed);
        CHECK(r.run.routine.exit == ExitReason::TimedOut);
        CHECK(r.faultsAfter - r.faultsBefore == 1);
        CHECK(r.lastFault == FaultCode::MotionTimeout);
        const double miss = std::hypot(r.endEstimate.x().value() - 40.0, r.endEstimate.y().value());
        CHECK(miss > 0.5);  // outside the arrival rule: the stop is honest, not a tolerance
        const double truthMiss =
            std::hypot(s.rig.h.truePose().x().value() - 40.0, s.rig.h.truePose().y().value());
        CHECK(truthMiss > 0.5);
        // The side it ends on is the weak side's: right weak -> y < 0; left weak -> y > 0.
        CHECK((wheel == 1 ? r.endEstimate.y().value() < 0.0 : r.endEstimate.y().value() > 0.0));
        CHECK(r.run.guard.endActionRan);
        CHECK(r.run.guard.endActionSucceeded);
        CHECK_FALSE(r.run.guard.scoringCut);
        CHECK(s.driveSafe());
        const std::string lines = joined(shulib::bench::testerRunLines(r, reg.at(static_cast<std::size_t>(forty(reg)))));
        CHECK(says(lines, "ROUTINE STOPPED -- ROUTINE_RAN; in time, end action succeeded"));
        CHECK(says(lines, "stopped at moveTo (motion TIMEOUT)"));
        CHECK(says(lines, "faults this run: 1, last MOTION_TIMEOUT -- see the [ERROR] line"));
    }
}

// (3c) Bug caught: the whole-boot fault latch leaking into a run's record — a fault latched BEFORE
// RUN reported as this run's (the RESULT page and the log would blame the auton for a fault it did
// not cause). A fault raised on the latch, then a clean run: none this run, and no last fault.
TEST_CASE("autons (3c): a fault latched before RUN is not this run's") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    s.rig.latch.raise(FaultCode::GpsGateReject, "TST", "latched before RUN");
    REQUIRE(s.rig.latch.faultCount() == 1);
    const TesterRunRecord& r = s.run(reg, forty(reg));

    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    CHECK(r.faultsBefore == 1);
    CHECK(r.faultsAfter == 1);
    CHECK(r.lastFault == FaultCode::None);
    CHECK(says(joined(shulib::bench::testerRunLines(r, reg.at(static_cast<std::size_t>(forty(reg))))), "faults this run: none"));
    CHECK_FALSE(says(joined(shulib::bench::testerRunLines(r, reg.at(static_cast<std::size_t>(forty(reg))))), "GPS_GATE_REJECT"));

    shulib::bench::TesterScreen screen{reg};
    shulib::bench::TesterFacts f;
    f.now = 100.0;
    screen.showResult(r, 0, false);
    const shulib::bench::TesterView v = screen.view(f);
    bool noneLine = false;
    for (int i = 0; i < v.lineCount; ++i) {
        noneLine = noneLine || std::string_view{v.lines[i].text} == "faults this run: none";
    }
    CHECK(noneLine);
}

// (4) Bug caught: the library's own acceptance run not surviving the tester's path (the second
// entry, same schedule): it must drive 24 in, turn, face the start and come back — ending within
// tolerance of (0, 0) facing 180 deg by truth. Measured after TANK1: (0.23, 0.04, -179.97 deg),
// T+5.44 s (pre-TANK1: (0.24, 0.12, 180.00 deg)).
TEST_CASE("autons (4): the acceptance run through runTesterOnce ends back at the start, reversed") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const TesterRunRecord& r = s.run(reg, acceptance(reg));
    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    CHECK(r.run.routine.completed == 4);
    CHECK(r.run.routine.steps == 4);
    const Pose2d truth = s.rig.h.truePose();
    CHECK(std::hypot(truth.x().value(), truth.y().value()) < 0.6);
    CHECK(std::abs(std::abs(truth.heading().degrees()) - 180.0) < 3.0);
    CHECK(r.faultsAfter == r.faultsBefore);
    CHECK(s.driveSafe());
}

// (4b) Bug caught (re-predicted after TANK1 Part A): the acceptance run on the 5 % weak plant now
// FINISHES (4 of 4, no fault); and the honest stop moves to 35 % weak, where step 1 (moveTo) times
// out, steps 2-4 are skipped, ONE MOTION_TIMEOUT is this run's, and the END is outside step 1's
// arrival rule (distance to (24, 0) over 0.5 in; measured 0.80 in). NO END-y bound: that END's |y|
// is 0.49, inside 0.5 — an END-|y| clause copied from (3b') would fail by 0.01 in.
TEST_CASE("autons (4b): the acceptance run finishes on a 5 % weak side, stops honestly on 35 %") {
    const AutonRegistry reg = teamRegistry();
    {
        TesterStack s{tank(), 1, 0.95};
        const TesterRunRecord& r = s.run(reg, acceptance(reg));
        REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
        CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
        CHECK(r.run.routine.ok);
        CHECK(r.run.routine.completed == 4);
        CHECK(r.faultsAfter == r.faultsBefore);
        const Pose2d truth = s.rig.h.truePose();
        CHECK(std::hypot(truth.x().value(), truth.y().value()) < 0.6);
        CHECK(std::abs(std::abs(truth.heading().degrees()) - 180.0) < 3.0);
    }
    for (const int wheel : {1, 0}) {
        CAPTURE(wheel);
        TesterStack s{tank(), wheel, 0.65};
        const TesterRunRecord& r = s.run(reg, acceptance(reg));
        REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
        CHECK(std::string_view{testerResultWord(r)} == "ROUTINE STOPPED");
        CHECK(r.run.routine.stoppedAt == 1);
        CHECK(std::string_view{r.run.routine.stoppedName} == "moveTo");
        CHECK(r.run.routine.exit == ExitReason::TimedOut);
        CHECK(r.run.routine.completed == 0);
        CHECK(r.run.routine.skipped == 3);
        CHECK(r.faultsAfter - r.faultsBefore == 1);
        CHECK(r.lastFault == FaultCode::MotionTimeout);
        CHECK(std::hypot(r.endEstimate.x().value() - 24.0, r.endEstimate.y().value()) > 0.5);
        CHECK(r.run.guard.endActionSucceeded);
        CHECK(s.driveSafe());
    }
}

// (5) Bug caught: a RELATIVE start that does not zero the pre-run drift. The robot is placed at
// (10, -5) facing 30 deg; its IMU reads 17 deg there (it booted elsewhere, or drifted while READY
// was up). RELATIVE says: wherever it stands is (0, 0) facing 0 — so "40 in ahead" is 40 in along
// the robot's OWN 30 deg line by truth, while the END estimate reads about (40, 0, 0) in the start
// frame. Without runAuton's re-seat the estimate would start at 17 deg and aim the leg 17 deg off.
TEST_CASE("autons (5): a RELATIVE start after a pre-run drift drives 40 in along the robot's line") {
    const AutonRegistry reg = teamRegistry();
    OffsetImu imu{17.0 - 30.0};
    shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
    cfg.plant.initialPose = Pose2d{Length{10.0}, Length{-5.0}, Angle::degrees(30.0)};
    TesterStack s{tank(), 1, 1.0, cfg, &imu};
    const TesterRunRecord& r = s.run(reg, forty(reg));

    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.run.imuAtSeat.approxEqual(Angle::degrees(17.0), 1e-6));  // the drift, stated
    CHECK(r.run.seated.x().value() == 0.0);
    CHECK(r.run.seated.y().value() == 0.0);
    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    CHECK(std::abs(r.endEstimate.x().value() - 40.0) < 0.5);
    CHECK(std::abs(r.endEstimate.y().value()) < 0.5);
    CHECK(std::abs(r.endEstimate.heading().degrees()) < 1.2);
    const Pose2d truth = s.rig.h.truePose();
    const Pose2d expected{Length{10.0 + 40.0 * std::cos(30.0 * kDeg)},
                          Length{-5.0 + 40.0 * std::sin(30.0 * kDeg)}, Angle::degrees(30.0)};
    CHECK(motion_rig::posErr(truth, expected) < 0.5);
    CHECK(motion_rig::headErr(truth, expected) < 1.2 * kDeg);
}
