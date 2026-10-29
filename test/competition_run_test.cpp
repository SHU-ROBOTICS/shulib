// competition_run_test.cpp — ONE autonomous request of "shulib Competition" on the host test plant
// (src/programs/competition_run.hpp: serveAutonRequest, reportAutonRun, runAutonRequest, the
// handshake, the words; chunk COMP-b, 2026-09-30; the COMP brief's B2, B9, B12, B19, B22, and its
// re-check's RC1 (a)-(c), ND1, ND2 and ND5). Every case names the bug it would catch.
//
// The stack is the program's pacer chain with the plant in the provider's place
// (test/competition_rig.hpp): Chassis -> RunGuard -> the REAL CompetitionWatch -> a probe layer ->
// the plant, the team's registry (src/team/autons.cpp), a real AutonMenu, the handshake and the
// flags. Graded on plant GROUND TRUTH and on the drive motors' commanded state. Host test plant
// numbers; nothing here has run on a robot (worksheet Station K).

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "competition_rig.hpp"
#include "motion_test_rig.hpp"
#include "common/guarded_run.hpp"
#include "programs/competition_run.hpp"
#include "programs/competition_screen.hpp"
#include "programs/competition_watch.hpp"
#include "team/autons.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/block_char_sink.hpp"
#include "shulib/hal/block_sink.hpp"
#include "shulib/hal/fake/fake_mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using comp_rig::CompetitionStack;
using shulib::bench::AutonHandshake;
using shulib::bench::CompetitionCause;
using shulib::bench::CompetitionRunLines;
using shulib::bench::CompetitionRunRecord;
using shulib::bench::CompetitionSkip;
using shulib::bench::competitionResultWord;
using shulib::bench::competitionRunLines;
using shulib::bench::kCompetitionLineBytes;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::competition::StartPose;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

namespace {

const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

bool says(std::string_view text, std::string_view needle) {
    return text.find(needle) != std::string_view::npos;
}

/// The team's two autons' indices in a CompetitionStack with no extra autons.
constexpr int kForty = 0;
constexpr int kAcceptance = 1;

RoutineResult driveThenThrow(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    SHULIB_PRECONDITION(false, "test: a routine tripped a precondition");
    return r.result();
}
RoutineResult driveThenThrowStd(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw-std"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    if (r.result().steps >= 0) {
        throw std::out_of_range("test: a student's std::out_of_range");
    }
    return r.result();
}
RoutineResult driveThenThrowInt(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw-int"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    if (r.result().steps >= 0) {
        throw 7;
    }
    return r.result();
}
AutonEntry routineEntry(const char* name, shulib::competition::AutonRoutine fn) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Match;
    e.routine = fn;
    e.start = StartPose::relative();
    e.endActionAt = Time{27.0};
    e.hardStopAt = Time{29.0};
    return e;
}
/// Three throwing autons, registered BEFORE the team's (indices 0, 1, 2; the team's then 3 and 4).
void addThrowers(AutonRegistry& r) {
    r.add(routineEntry("throws precondition", &driveThenThrow));
    r.add(routineEntry("throws std", &driveThenThrowStd));
    r.add(routineEntry("throws int", &driveThenThrowInt));
}

/// Drive every drive motor at `volts` under Coast — a live drive nothing but a stop can end.
void energize(CompetitionStack& s, double volts) {
    for (shulib::hal::IMotor* m : s.rig.h.context().driveMotors()) {
        m->setBrakeMode(shulib::hal::BrakeMode::Coast);
        m->setVoltage(shulib::units::Voltage{volts});
    }
}

/// A block sink that records, for every write, the handshake's FINISHED at that instant (ND5).
struct OrderedBlockSink final : shulib::hal::IBlockSink {
    const AutonHandshake* handshake = nullptr;
    std::vector<std::uint32_t> finishedAtWrite;
    std::string text;
    [[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override {
        try {
            finishedAtWrite.push_back(handshake->finished());
            text.append(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        } catch (...) {
            return false;
        }
        return true;
    }
};

}  // namespace

// ═══ a request that runs ════════════════════════════════════════════════════════════════════

// (R1) Bug caught: the program's request path not reaching runAuton with the chosen entry, the
// terminal word not published after a clean run, the menu left LOCKED (a second autonomous() could
// never take a new choice), the watch left armed, B12's hand-off latency measured against the
// wrong reading, the status page told AFTER the run (RC3: it is painted BEFORE runAuton -- told
// later, the menu would stay up during the auton and AUTON RUNNING flash after it), or log line 4
// printing anything but the END estimate (the seat again, or LIVE for an estimate that is not).
TEST_CASE("request (R1): the 40 in through serveAutonRequest — FINISHED, the word published") {
    CompetitionStack s{tank()};
    s.choose(kForty);
    const double before = s.plant.distance;
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.request == 1);
    CHECK(r.entry == kForty);
    CHECK(r.skip == CompetitionSkip::None);
    CHECK_FALSE(r.threw);
    CHECK(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
    CHECK(s.plant.distance - before > 39.0);
    CHECK(s.handshake.isFinished(1));
    CHECK_FALSE(s.handshake.busy());
    CHECK_FALSE(s.menu->locked());
    CHECK_FALSE(s.watch.armed());
    CHECK(r.cause == CompetitionCause::None);
    CHECK(s.obs.calls == 1);
    CHECK(s.obs.entry == kForty);
    CHECK(s.obs.lockedAtStart);  // LOCKED for the run (a touch racing the start changes nothing)
    CHECK(s.obs.distanceAtStart == before);  // RC3: told BEFORE anything moved ...
    CHECK_FALSE(s.obs.guardRunningAtStart);  // ... before the guard armed ...
    CHECK_FALSE(s.obs.watchArmedAtStart);    // ... and before the watch armed
    // B12: the guard armed within a tick of autonomous()'s own reading (the host serves at once).
    const double handOff = r.run.armedAt.value() - r.requestedAt;
    CHECK(handOff >= -0.0005);  // the request time is kept in whole milliseconds
    CHECK(handOff <= 0.0105);
    CHECK(s.driveSafe());
    CHECK(s.probe.energizedWhileIdle == 0);
    const CompetitionRunLines lines = competitionRunLines(r, s.registry);
    REQUIRE(lines.count == 4);
    CHECK_FALSE(lines.cut);
    CHECK(std::string_view{lines.line[0]}
          == "request 1: '40in forward' FINISHED -- ROUTINE_RAN; in time, end action succeeded");
    CHECK(says(lines.line[1], "hand-off 0.00"));
    CHECK(says(lines.line[2], "routine OK, 1 of 1 steps done; faults this run: none"));
    // Line 4: the seat, then the END estimate -- the estimate's own numbers, which track the plant's
    // 40 in (never the seat's (0, 0)), and LIVE.
    CHECK(r.endLive);
    CHECK(r.endEstimate.x().value() == doctest::Approx(s.rig.h.truePose().x().value()).epsilon(0.02));
    CHECK(r.endEstimate.x().value() > 39.0);
    char want[kCompetitionLineBytes];
    std::snprintf(want, sizeof want,
                  "request 1: seated (0.00, 0.00) facing 0.0 deg (the IMU read %.1f); END estimate "
                  "(%.2f, %.2f) facing %.1f deg LIVE",
                  r.run.imuAtSeat.degrees(), r.endEstimate.x().value(), r.endEstimate.y().value(),
                  r.endEstimate.heading().degrees());
    CHECK(std::string_view{lines.line[3]} == want);
}

// (R2) Bug caught: an auton the team registers that the COMPETITION program's request cannot run
// to a clean end — the tester's (1b) proves every auton through the tester's sequence; this proves
// it through the match's. Each: FINISHED, the drive at 0 V + Brake after, nothing commanded idle.
TEST_CASE("request (R2): EVERY registered auton runs through the program's request and ends safe") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    REQUIRE(reg.size() >= 2);
    for (std::size_t i = 0; i < reg.size(); ++i) {
        CAPTURE(reg.at(i).name);
        CompetitionStack s{tank()};
        s.choose(static_cast<int>(i));
        const CompetitionRunRecord& r = s.serve();
        CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
        CHECK(s.handshake.isFinished(r.request));
        CHECK(s.driveSafe());
        CHECK(s.probe.energizedWhileIdle == 0);
    }
}

// (R3) Bug caught (B7, C5): a field DISABLE mid-auton not ending the run — the watch unarmed for
// the program's request, the latch not reaching the guard — or ending it late, or with the end
// action run into a disabled robot; and the latch or the lock surviving into the NEXT request (a
// second autonomous() on the same boot would be abandoned at T+0, or run the old choice).
TEST_CASE("request (R3): a disable mid-auton — ABANDONED within a tick, safe; the next one is clean") {
    CompetitionStack s{tank()};
    s.choose(kAcceptance);
    const double start = s.now();
    s.status.onRead = [&](double now) {
        s.status.status = now - start >= 1.0 ? comp_rig::kDisabledOnField : comp_rig::kAutonEnabled;
    };
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.cause == CompetitionCause::Disabled);
    CHECK(r.statusAtLatch == comp_rig::kDisabledOnField);
    CHECK(r.run.guard.abandoned);
    CHECK_FALSE(r.run.guard.endActionRan);
    CHECK(std::string_view{competitionResultWord(r)} == "ABANDONED");
    CHECK(r.run.guard.scoringEnded.value() <= 1.0 + 0.03);  // within ~2 ticks of the disable
    CHECK(s.plant.distance - s.probe.travelAtLatch <= 60.0 * 0.01 + 1e-6);
    CHECK(s.driveSafe());
    CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
    CHECK(s.handshake.isFinished(r.request));
    const CompetitionRunLines lines = competitionRunLines(r, s.registry);
    CHECK(says(lines.line[0], "ABANDONED -- ROUTINE_RAN; ABANDONED: the robot was disabled "
                              "(status 0x05), end action skipped"));
    // The next autonomous() on this boot: the field enables again; the SAME choice runs, clean.
    s.status.onRead = nullptr;
    s.status.status = comp_rig::kAutonEnabled;
    s.probe.travelAtLatch = -1.0;
    const CompetitionRunRecord& again = s.serve();
    CHECK(again.request == 2);
    CHECK(again.cause == CompetitionCause::None);
    CHECK_FALSE(again.run.guard.abandoned);
    CHECK(std::string_view{competitionResultWord(again)} == "FINISHED");
}

// (R4) Bug caught: the end of the autonomous period (the field moving to driver control with no
// disable in between) not ending the run — the watch checking only the disabled bit.
TEST_CASE("request (R4): the period ends mid-auton (driver control) — ABANDONED, left autonomous") {
    CompetitionStack s{tank()};
    s.choose(kAcceptance);
    const double start = s.now();
    s.status.onRead = [&](double now) {
        s.status.status = now - start >= 0.8 ? comp_rig::kDriverOnField : comp_rig::kAutonEnabled;
    };
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.cause == CompetitionCause::LeftAutonomous);
    CHECK(r.run.guard.abandoned);
    CHECK(std::string_view{competitionResultWord(r)} == "ABANDONED");
    CHECK(s.driveSafe());
    CHECK(s.handshake.isFinished(r.request));
}

// (R17) Bug caught (RC3 (iii): the auton task never ticks the Chassis while it idles, so on the
// robot the estimate's ONLY tick before a match is the boot's chain proof, and its next one is
// runAuton's live-wait, minutes later): a first tick after a long gap that the estimate or the
// request mishandles — a fault or a velocity spike from the gap's huge dt and the moved robot's
// encoder jump, a seat taken from a stale estimate, or the auton driving from where the robot WAS
// at boot rather than where it now stands. On the plant: the chain proof's one tick, then the robot
// is moved by hand (driven through its motors with NOTHING ticking the Chassis: a turn, a push) and
// left for a minute; the 40 in must then run 40 in straight ahead of where it stands, no fault.
TEST_CASE("request (R17): the first tick after a long idle — the 40 in from where the robot stands") {
    CompetitionStack s{tank(), 1, 1.0, 0};
    REQUIRE(shulib::bench::pacerChainProven(s.chassis, s.guard, s.watch));  // the boot's ONE tick
    s.choose(kForty);
    const auto byHand = [&s](double left, double right, int ticks) {
        const auto motors = s.rig.h.context().driveMotors();
        REQUIRE(motors.size() == 2);  // 0 = left, 1 = right (TankKinematics' canonical order)
        motors[0]->setVoltage(shulib::units::Voltage{left});
        motors[1]->setVoltage(shulib::units::Voltage{right});
        for (int i = 0; i < ticks; ++i) {
            s.rig.h.plant().step(Time{0.01});  // the world moves; the Chassis is never ticked
        }
        motors[0]->setVoltage(shulib::units::Voltage{0.0});
        motors[1]->setVoltage(shulib::units::Voltage{0.0});
    };
    byHand(-6.0, 6.0, 30);  // turned left, about 80 deg
    byHand(6.0, 6.0, 60);   // pushed about 17 in
    byHand(0.0, 0.0, 6000);  // a minute on the field, disabled
    const Pose2d placed = s.rig.h.truePose();
    REQUIRE(std::abs(placed.heading().degrees()) > 20.0);                     // it WAS turned
    REQUIRE(std::hypot(placed.x().value(), placed.y().value()) > 5.0);         // and moved
    const double chainTickAt = 0.01;
    REQUIRE(s.now() - chainTickAt > 60.0);  // the live-wait's first tick is a minute after the last
    const CompetitionRunRecord& r = s.serve();
    CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
    CHECK(r.faultsAfter == r.faultsBefore);  // the gap raised no fault
    CHECK(r.run.seated.x().value() == 0.0);  // RELATIVE: seated at (0, 0) facing 0, where it stands
    CHECK(r.run.seated.y().value() == 0.0);
    CHECK(r.run.seated.heading().radians() == 0.0);
    CHECK(r.endLive);
    const Pose2d end = s.rig.h.truePose();
    const double dx = (end.x() - placed.x()).value();
    const double dy = (end.y() - placed.y()).value();
    const double c = std::cos(placed.heading().radians());
    const double sn = std::sin(placed.heading().radians());
    CAPTURE(dx);
    CAPTURE(dy);
    CHECK(dx * c + dy * sn == doctest::Approx(40.0).epsilon(0.02));  // 40 in straight ahead of it
    CHECK(std::abs(-dx * sn + dy * c) < 1.0);                         // and not off to one side
    CHECK(s.driveSafe());
}

// ═══ a request that runs nothing ═══════════════════════════════════════════════════════════

// (R5) Bug caught: NONE — the menu's default — driving anything, runAuton called for it, or no
// terminal word (opcontrol() would wait for ever on a robot nobody chose an auton for).
TEST_CASE("request (R5): NONE chosen — nothing moves, runAuton is never called, FINISHED") {
    CompetitionStack s{tank()};
    const double before = s.plant.distance;
    const int logBefore = s.sink.size();
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.entry == -1);
    CHECK(r.skip == CompetitionSkip::NoneChosen);
    CHECK(std::string_view{competitionResultWord(r)} == "NONE");
    CHECK(s.plant.distance == before);
    for (int i = logBefore; i < s.sink.size(); ++i) {
        CHECK(s.sink.at(i).subsystem != "COMP");  // runAuton never ran
    }
    CHECK(s.handshake.isFinished(1));
    CHECK(s.obs.skip == CompetitionSkip::NoneChosen);
    const CompetitionRunLines lines = competitionRunLines(r, s.registry);
    REQUIRE(lines.count == 2);
    CHECK(std::string_view{lines.line[0]}
          == "request 1: NONE -- no auton was chosen (NONE is the default); nothing moved");
}

// (R6) Bug caught (ND2, the team lead's "refuse the auton, not the boot"): a boot from the TYPED
// TABLE running an auton (its front may be the other end: the 40 in could drive at the operator).
TEST_CASE("request (R6): a typed-table boot REFUSES the auton — nothing moves, FINISHED") {
    CompetitionStack s{tank()};
    s.flags.bootFromRecord = false;
    s.choose(kForty);
    const double before = s.plant.distance;
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.skip == CompetitionSkip::BootFromTable);
    CHECK(std::string_view{competitionResultWord(r)} == "REFUSED");
    CHECK(s.plant.distance == before);
    CHECK(s.handshake.isFinished(1));
    const CompetitionRunLines lines = competitionRunLines(r, s.registry);
    CHECK(says(lines.line[0], "'40in forward' REFUSED -- this boot used the TYPED TABLE"));
    CHECK_FALSE(s.flags.programError.load());  // a refusal is not a program error
}

// (R6b) Bug caught (ND1, ND2 -- the REACHABLE states): (R6) chooses an auton on a typed-table boot,
// which the program never allows -- with autons refused the menu phase paints the refusal and only
// CONSUMES touches (competition_screen.hpp's menuPhaseStep), so the choice there is always NONE. A
// refusal checked AFTER "NONE chosen" (or only when an entry is chosen) would then log "NONE -- no
// auton was chosen ... choose an auton on the brain's menu" while the brain says the autons are
// REFUSED. Both refusals, with nothing chosen: the refusal's word and lines, never NONE's.
TEST_CASE("request (R6b): NONE on a typed-table boot, or after a program error, reads the REFUSAL") {
    {
        CompetitionStack s{tank()};
        s.flags.bootFromRecord = false;  // no choose(): the selector is never up on this boot
        REQUIRE(s.menu->choice() < 0);
        const double before = s.plant.distance;
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.skip == CompetitionSkip::BootFromTable);
        CHECK(std::string_view{competitionResultWord(r)} == "REFUSED");
        CHECK(s.plant.distance == before);
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        REQUIRE(lines.count == 2);
        CHECK(std::string_view{lines.line[0]}
              == "request 1: 'NONE' REFUSED -- this boot used the TYPED TABLE, whose front may be "
                 "the OTHER end; nothing moved");
        CHECK_FALSE(says(lines.line[1], "choose an auton"));
    }
    {
        CompetitionStack s{tank()};
        s.flags.programError.store(true);
        REQUIRE(s.menu->choice() < 0);
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.skip == CompetitionSkip::AfterProgramError);
        CHECK(std::string_view{competitionResultWord(r)} == "REFUSED");
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        CHECK(says(lines.line[0], "REFUSED -- a program error earlier on this boot"));
    }
    // The menu phase's refusal and the request's agree, for every pair of flags.
    for (const bool fromRecord : {false, true}) {
        for (const bool error : {false, true}) {
            CAPTURE(fromRecord);
            CAPTURE(error);
            shulib::bench::CompetitionFlags f;
            f.bootFromRecord = fromRecord;
            f.programError.store(error);
            CHECK(shulib::bench::autonsRefused(f) == (error || !fromRecord));
        }
    }
}

// ═══ RC1: a throw on the request path never costs driver control ════════════════════════════════

// (R7) Bug caught (RC1 (a)): a throw OUTSIDE runAuton — the status painter, or a broken log sink on
// runAuton's own first line — escaping (the task dies: std::terminate), leaving the drive LIVE
// (only the handler's stop can save it: the throw is outside every scheduler wait), leaving a
// present mechanism unsafe, or NOT publishing the terminal word (opcontrol() would paint AUTON
// ENDING for the rest of the match). Each of the three handlers.
TEST_CASE("request (R7): a throw OUTSIDE runAuton — stopped, safed, latched, and FINISHED") {
    for (const tester_rig::ProbeThrow kind : {tester_rig::ProbeThrow::Precondition,
                                              tester_rig::ProbeThrow::Standard,
                                              tester_rig::ProbeThrow::Unknown}) {
        CAPTURE(static_cast<int>(kind));
        SUBCASE("(i) the status painter throws") {
            CompetitionStack s{tank()};
            shulib::hal::fake::FakeMechanism pivot{"claw pivot 6"};
            shulib::hal::fake::FakeMechanism claw{"claw"};
            s.present = {&pivot, &claw};
            s.choose(kForty);
            s.obs.throwOnStart = true;
            s.obs.throwKind = kind;
            energize(s, 6.0);
            REQUIRE_FALSE(s.driveSafe());
            const CompetitionRunRecord* r = nullptr;
            CHECK_NOTHROW(r = &s.serve());
            REQUIRE(r != nullptr);
            CHECK(r->threw);
            CHECK(std::string_view{r->threwWhat}
                  == (kind == tester_rig::ProbeThrow::Precondition
                          ? "probe: the status painter threw"
                          : kind == tester_rig::ProbeThrow::Standard
                                ? "probe: the status painter threw a runtime_error"
                                : "an unknown exception"));
            CHECK(std::string_view{competitionResultWord(*r)} == "PROGRAM ERROR");
            CHECK(s.driveSafe());  // the handler's program-error stop
            CHECK(pivot.safeStateApplications() >= 1);
            CHECK(claw.safeStateApplications() >= 1);
            CHECK(s.flags.programError.load());
            CHECK(s.handshake.isFinished(1));
            CHECK_FALSE(s.handshake.busy());
            CHECK_FALSE(s.menu->locked());
            CHECK_FALSE(s.watch.armed());
            bool errorLine = false;
            for (int i = 0; i < s.sink.size(); ++i) {
                errorLine = errorLine || (s.sink.at(i).level == shulib::hal::LogLevel::Error
                                          && says(s.sink.at(i).message,
                                                  "PROGRAM ERROR on request 1"));
            }
            CHECK(errorLine);
        }
        SUBCASE("(ii) the log sink throws on runAuton's first line (and on the Error line)") {
            CompetitionStack s{tank()};
            comp_rig::ThrowingSink broken;
            broken.throwAt = 0;
            s.telemetryOverride = &broken;
            shulib::hal::fake::FakeMechanism pivot{"claw pivot 7"};
            s.present = {&pivot};
            s.choose(kForty);
            energize(s, 6.0);
            const CompetitionRunRecord* r = nullptr;
            CHECK_NOTHROW(r = &s.serve());
            REQUIRE(r != nullptr);
            CHECK(r->threw);
            CHECK(std::string_view{r->threwWhat} == "probe: the log sink threw");
            CHECK(s.driveSafe());
            CHECK(pivot.safeStateApplications() >= 1);
            CHECK(s.handshake.isFinished(1));  // the Error line threw too: the word still came
            CHECK(broken.calls >= 2);          // runAuton's line, then the Error line
        }
    }
}

// (R8) Bug caught (RC1 (b)): a throw INSIDE a routine (the guard's rethrow after its own safing) —
// a precondition, a student's std::out_of_range, a non-standard throw — escaping, losing its words,
// or skipping the stop / the mechanisms / the terminal word.
TEST_CASE("request (R8): a routine that throws (the guard rethrows) — PROGRAM ERROR, FINISHED") {
    const char* want[3] = {"test: a routine tripped a precondition",
                           "test: a student's std::out_of_range", "an unknown exception"};
    for (int index = 0; index < 3; ++index) {
        CAPTURE(index);
        CompetitionStack s{tank(), 1, 1.0, 50, &addThrowers};
        shulib::hal::fake::FakeMechanism pivot{"claw pivot 6"};
        s.present = {&pivot};
        s.choose(index);
        const double before = s.plant.distance;
        const CompetitionRunRecord* r = nullptr;
        CHECK_NOTHROW(r = &s.serve());
        REQUIRE(r != nullptr);
        CHECK(r->threw);
        CHECK(std::string_view{r->threwWhat} == want[index]);
        CHECK(std::string_view{competitionResultWord(*r)} == "PROGRAM ERROR");
        CHECK(s.plant.distance - before > 9.0);  // it drove before it threw
        CHECK(s.driveSafe());
        CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
        CHECK_FALSE(s.guard.running());
        CHECK(pivot.safeStateApplications() >= 1);
        CHECK(s.flags.programError.load());
        CHECK(s.handshake.isFinished(1));
        const CompetitionRunLines lines = competitionRunLines(*r, s.registry);
        REQUIRE(lines.count == 2);
        CHECK(says(lines.line[0], "PROGRAM ERROR -- it threw: "));
        CHECK(says(lines.line[1], "driver control still runs; later autons are REFUSED"));
    }
}

// (R9) Bug caught (RC1 (c); B2): opcontrol()'s wait never ending after a caught throw, or ending
// before the terminal word; AUTON ENDING said early, twice, or never. The waiter is the program's
// own (waitForAutonTask), on a scripted clock: it returns at once after a throw was caught; it
// waits — commanding nothing — while a request has no word, says AUTON ENDING exactly ONCE after
// 0.25 s, and returns on the word, with NO upper bound of its own.
TEST_CASE("request (R9): opcontrol()'s wait — proceeds after a caught throw; AUTON ENDING once") {
    // RC1 (c) for both of RC1's throws: (a) OUTSIDE runAuton (the status painter), (b) INSIDE a
    // routine (the guard's rethrow).
    for (const bool insideRoutine : {false, true}) {
        CAPTURE(insideRoutine);
        CompetitionStack s{tank(), 1, 1.0, 50, &addThrowers};
        s.choose(insideRoutine ? 0 : 3);  // a throwing routine, or the team's 40 in ...
        s.obs.throwOnStart = !insideRoutine;  // ... with a painter that throws before it runs
        (void)s.serve();
        REQUIRE(s.flags.programError.load());
        REQUIRE(s.rec.threw);
        int ticks = 0;
        int slow = 0;
        const double waited = shulib::bench::waitForAutonTask(
            s.handshake, [] { return 0.0; },
            [&ticks] {
                if (++ticks > 1000) {
                    throw std::runtime_error("the waiter never ended: no terminal word");
                }
            },
            [&slow](double) { ++slow; });
        CHECK(ticks == 0);
        CHECK(slow == 0);
        CHECK(waited == 0.0);
    }
    // A request with no word yet: the waiter ticks, says AUTON ENDING once at 0.25 s, and returns
    // the tick the word is published (here, 1.0 s in: a frozen routine's remainder).
    AutonHandshake h;
    const std::uint32_t n = h.request(0.0);
    double t = 0.0;
    int ticks = 0;
    int slow = 0;
    double slowAt = -1.0;
    const double waited = shulib::bench::waitForAutonTask(
        h, [&t] { return t; },
        [&] {
            ++ticks;
            t += 0.01;
            if (t >= 1.0 - 1e-9) {
                h.finish(n);
            }
            if (ticks > 100000) {
                throw std::runtime_error("the waiter never ended");
            }
        },
        [&](double w) {
            ++slow;
            slowAt = w;
        });
    CHECK(slow == 1);
    CHECK(slowAt >= shulib::bench::kAutonEndingAfterSeconds);
    CHECK(slowAt < shulib::bench::kAutonEndingAfterSeconds + 0.011);
    CHECK(waited == doctest::Approx(1.0).epsilon(0.02));
    CHECK_FALSE(h.busy());
}

// (R10) Bug caught (ND1, the team lead's answer "yes, after a safe stop"): after a program error,
// a later autonomous() on the same boot running an auton (the program's state is unproven) — or
// being refused without its terminal word.
TEST_CASE("request (R10): after a program error, a later auton is REFUSED — nothing moves") {
    CompetitionStack s{tank(), 1, 1.0, 50, &addThrowers};
    s.choose(1);
    (void)s.serve();
    REQUIRE(s.flags.programError.load());
    s.choose(3);  // the team's 40 in: a good auton
    const double before = s.plant.distance;
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.skip == CompetitionSkip::AfterProgramError);
    CHECK(std::string_view{competitionResultWord(r)} == "REFUSED");
    CHECK(s.plant.distance == before);
    CHECK(s.handshake.isFinished(2));
    const CompetitionRunLines lines = competitionRunLines(r, s.registry);
    CHECK(says(lines.line[0], "REFUSED -- a program error earlier on this boot (its own Error line "
                              "says where); nothing moved"));
    CHECK(says(lines.line[1], "restart the program to run autons again"));
}

// ═══ ND5: the card never delays driver control ════════════════════════════════════════════

// (R11) Bug caught (ND5): the SD run log written BEFORE the terminal word (a slow card would delay
// driver control), or not written at all, or saying something serial does not. Through the
// program's own composition, runAutonRequest: every block write sees FINISHED already published,
// and the card carries the serial lines' words.
TEST_CASE("request (R11): the SD run log is written only AFTER FINISHED, with serial's words") {
    CompetitionStack s{tank()};
    OrderedBlockSink card;
    card.handshake = &s.handshake;
    shulib::hal::BlockCharSink chars{card};
    shulib::diag::TermSink sd{s.rig.h.clock(), chars};
    s.choose(kForty);
    const std::uint32_t n = s.handshake.request(s.now());
    shulib::bench::CompetitionParts p = s.parts();
    const int serialBefore = s.sink.size();
    char errorWhat[shulib::bench::kCompetitionErrorBytes] = "";
    (void)shulib::bench::runAutonRequest(p, s.rec, n, s.now(), 1, s.obs, &sd, errorWhat);
    CHECK(std::string_view{errorWhat}.empty());  // nothing threw: no reason recorded
    REQUIRE(card.finishedAtWrite.size() == 4);  // the four lines of a run
    for (const std::uint32_t f : card.finishedAtWrite) {
        CHECK(f == n);  // FINISHED was already published at every write
    }
    const CompetitionRunLines lines = competitionRunLines(s.rec, s.registry);
    for (int i = 0; i < lines.count; ++i) {
        CHECK(says(card.text, lines.line[i]));
    }
    int cmpLines = 0;
    for (int i = serialBefore; i < s.sink.size(); ++i) {
        cmpLines += s.sink.at(i).subsystem == "CMP" ? 1 : 0;
    }
    CHECK(cmpLines == 4);
}

// (R12) Bug caught (B9): the Robot's PRESENT mechanisms not reaching the guard through the program's
// request (the collection empty, or the span not handed on) — nothing would be safed at the cut or
// the end; an ABSENT one is simply not in the span. At a clean end AND at a disable.
TEST_CASE("request (R12): the present mechanisms are safed at a clean end and at a disable") {
    for (const bool disable : {false, true}) {
        CAPTURE(disable);
        CompetitionStack s{tank()};
        shulib::hal::fake::FakeMechanism pivot6{"claw pivot 6"};
        shulib::hal::fake::FakeMechanism claw{"claw"};
        s.present = {&pivot6, &claw};  // claw pivot 7 ABSENT: not listed
        s.choose(kAcceptance);
        if (disable) {
            const double start = s.now();
            s.status.onRead = [&](double now) {
                s.status.status =
                    now - start >= 1.5 ? comp_rig::kDisabledOnField : comp_rig::kAutonEnabled;
            };
        }
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.run.guard.abandoned == disable);
        CHECK(pivot6.safeStateApplications() >= 1);
        CHECK(claw.safeStateApplications() >= 1);
    }
}

// (R16) Bug caught (the file banner: nothing touches the robot after FINISHED): the report — which
// runs AFTER the terminal word, while the driver may be driving — reaching for the robot when it
// throws (the program-error stop from the auton task would fight the driver), or letting the throw
// out of the auton task. A throwing serial sink during the report: the program error is latched,
// the drive the "driver" set is left exactly as it was, and nothing escapes.
TEST_CASE("request (R16): a throw in the report after FINISHED latches, and never touches the robot") {
    CompetitionStack s{tank()};
    s.choose(kForty);
    (void)s.serve();
    REQUIRE(s.handshake.isFinished(1));
    energize(s, 4.5);  // driver control has the robot now
    comp_rig::ThrowingSink broken;
    broken.throwAt = 0;
    char errorWhat[shulib::bench::kCompetitionErrorBytes] = "";
    bool threw = false;
    CHECK_NOTHROW(threw = shulib::bench::reportAutonRun(s.rec, s.registry, broken, nullptr, s.flags,
                                                        errorWhat));
    CHECK(threw);
    CHECK(s.flags.programError.load());
    // ...and the autons-refused page has its reason (COMP-b's review: it read an empty one).
    CHECK(std::string_view{errorWhat}
          == "the report of request 1 threw: probe: the log sink threw -- the robot was not touched");
    for (const shulib::hal::IMotor* m : s.rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 4.5);  // untouched
        CHECK(m->brakeMode() == shulib::hal::BrakeMode::Coast);
    }
}

// ═══ the handshake, the driver loop's end, the words ═════════════════════════════════════════

// (R14) Bug caught: two autonomous() calls during one serve running TWO autons (or the older
// choice), isFinished() false for a request that collapsed into a later one (autonomous() would
// wait for ever), or busy() wrong around a request.
TEST_CASE("request (R14): the handshake — requests collapse; isFinished and busy") {
    AutonHandshake h;
    CHECK_FALSE(h.busy());
    std::uint32_t req = 0;
    double at = 0.0;
    CHECK_FALSE(h.pending(0, req, at));
    const std::uint32_t a = h.request(1.2345);
    const std::uint32_t b = h.request(2.5);
    CHECK(a == 1);
    CHECK(b == 2);
    CHECK(h.busy());
    REQUIRE(h.pending(0, req, at));
    CHECK(req == 2);   // the NEWEST request is served
    CHECK(at == 2.5);  // with ITS time (whole milliseconds)
    h.finish(req);
    CHECK_FALSE(h.busy());
    CHECK(h.isFinished(1));  // the collapsed one is finished too
    CHECK(h.isFinished(2));
    CHECK_FALSE(h.pending(2, req, at));
    CHECK_FALSE(h.isFinished(3));
    (void)h.request(3.0);
    CHECK(h.busy());
}

// (R15) Bug caught: driver control running while the status says DISABLED or AUTONOMOUS, or while
// an auton request waits — the driver loop and the auton task on the robot at once (a direct change
// to autonomous PROS did not delete opcontrol for, HA-161) — or ending in plain driver control.
TEST_CASE("request (R15): driverControlMustEnd — every status byte, with and without a request") {
    for (int s = 0; s < 256; ++s) {
        CAPTURE(s);
        const auto b = static_cast<std::uint8_t>(s);
        const bool notDriver = (b & (shulib::bench::kStatusDisabled | shulib::bench::kStatusAutonomous))
                               != 0;
        CHECK(shulib::bench::driverControlMustEnd(b, false) == notDriver);
        CHECK(shulib::bench::driverControlMustEnd(b, true));
    }
    CHECK_FALSE(shulib::bench::driverControlMustEnd(0, false));  // the bench, nothing connected
    CHECK_FALSE(shulib::bench::driverControlMustEnd(comp_rig::kDriverOnField, false));
}

// (R13) Bug caught: a result word for the wrong outcome (a refusal reading FINISHED, an ignored
// abandon reading as a pass), a word past the LARGE budget, or a line cut by TermSink's 200 bytes
// at the worst case of every field (a 29-character name, huge numbers, a 159-character throw).
TEST_CASE("request (R13): the words — every outcome's word; no line cut at the worst case") {
    CompetitionRunRecord r{};
    r.request = 4294967295u;
    r.entry = 0;
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    AutonEntry longName = routineEntry("abcdefghijklmnopqrstuvwxyz012", &driveThenThrow);
    reg.add(longName);
    const auto word = [&r] { return std::string_view{competitionResultWord(r)}; };
    r.threw = true;
    std::memset(r.threwWhat, 'w', sizeof r.threwWhat - 1);
    CHECK(word() == "PROGRAM ERROR");
    CompetitionRunLines l = competitionRunLines(r, reg);
    CHECK_FALSE(l.cut);
    r.threw = false;
    for (const CompetitionSkip k : {CompetitionSkip::NoneChosen, CompetitionSkip::BootFromTable,
                                    CompetitionSkip::AfterProgramError}) {
        r.skip = k;
        CHECK(word() == (k == CompetitionSkip::NoneChosen ? "NONE" : "REFUSED"));
        l = competitionRunLines(r, reg);
        CHECK_FALSE(l.cut);
        CHECK(l.count == 2);
    }
    r.skip = CompetitionSkip::None;
    r.run.outcome = AutonRunOutcome::Refused;
    std::memset(r.run.why, 'y', sizeof r.run.why - 1);
    CHECK(word() == "REFUSED");
    CHECK_FALSE(competitionRunLines(r, reg).cut);
    r.run.outcome = AutonRunOutcome::RoutineRan;
    r.run.guard.abandoned = true;
    r.cause = CompetitionCause::LeftAutonomous;
    r.statusAtLatch = 0xFF;
    CHECK(word() == "ABANDONED");
    r.run.guard.abandoned = false;
    CHECK(word() == "ABANDON IGNORED");  // latched, never acted on: red, never a pass
    r.cause = CompetitionCause::None;
    r.run.outcome = AutonRunOutcome::NotStarted;
    CHECK(word() == "NOT STARTED");
    r.run.outcome = AutonRunOutcome::RoutineRan;
    r.run.guard.floorFired = true;
    CHECK(word() == "HARD STOP");
    r.run.guard.floorFired = false;
    r.run.guard.scoringCut = true;
    CHECK(word() == "CUT BY DEADLINE");
    r.run.guard.scoringCut = false;
    r.run.routine.ok = true;
    r.run.guard.endActionSucceeded = true;
    CHECK(word() == "FINISHED");
    r.run.guard.endActionSucceeded = false;
    CHECK(word() == "ROUTINE STOPPED");
    // The worst case of every numeric field, with an ignored abandon (the longest line 1).
    r.cause = CompetitionCause::LeftAutonomous;
    r.run.guard.endActionRan = true;
    r.run.guard.floorFired = true;
    r.run.guard.pacesSeen = 2147483647;
    r.run.guard.postExpiryCancels = 2147483647;
    r.run.guard.scoringEnded = Time{9999.99};
    r.run.guard.endActionEnded = Time{9999.99};
    r.run.armedAt = Time{99999.999};
    r.requestedAt = 0.0;
    r.run.liveAfter = Time{9999.99};
    r.run.routine.ok = false;
    r.run.routine.stoppedAt = 1;
    r.run.routine.completed = 2147483647;
    r.run.routine.steps = 2147483647;
    r.faultsBefore = 0;  // a fault COUNT is never negative: -2147483647 here (the stopped run's
    r.faultsAfter = 2147483647;  // first form) overflowed the difference -- undefined behaviour
    r.run.seated = Pose2d{Length{-9999.99}, Length{-9999.99}, Angle{}};
    r.endEstimate = Pose2d{Length{-9999.99}, Length{-9999.99}, Angle{}};
    l = competitionRunLines(r, reg);
    CHECK(l.count == 4);
    CHECK_FALSE(l.cut);
    for (int i = 0; i < l.count; ++i) {
        CAPTURE(l.line[i]);
        CHECK(std::strlen(l.line[i]) <= kCompetitionLineBytes - 1);
        CHECK(std::string_view{competitionResultWord(r)}.size() <= shulib::ui::kLargeCols);
    }
    // ...and the driver loop's line on top (COMP-b's review), its wait absurdly long: five lines.
    r.loopWait = shulib::bench::DriverLoopWait{.waited = true, .left = false, .seconds = 1e12};
    l = competitionRunLines(r, reg);
    CHECK(l.count == 5);
    CHECK_FALSE(l.cut);
    // TOO LATE at its worst: the longest cause, a huge serve delay.
    CompetitionRunRecord late{};
    late.request = 4294967295u;
    late.entry = 0;
    late.skip = CompetitionSkip::TooLate;
    late.cause = CompetitionCause::LeftAutonomous;
    late.statusAtLatch = 0xFF;
    late.requestedAt = -1e9;
    late.servedAt = 1e12;
    late.loopWait = shulib::bench::DriverLoopWait{.waited = true, .left = true, .seconds = -5.0};
    CHECK(std::string_view{competitionResultWord(late)} == "TOO LATE");
    l = competitionRunLines(late, reg);
    CHECK(l.count == 3);
    CHECK_FALSE(l.cut);
    CHECK(says(l.line[0], "99999.999s after autonomous() asked"));  // bounded, never a huge number
    CHECK(says(l.line[2], "the auton waited 0.000s"));             // never a negative wait
    // The boot lines at every field's worst case, and a phase line.
    const std::string hash(200, 'h');
    const shulib::bench::CompetitionBootLines boot = shulib::bench::competitionBootLines(
        "shulib-robot-variant=tank-competition-and-then-some", hash, "TYPED TABLE and more", false,
        2147483647, 2147483647, 2147483647, shulib::bench::AutonRunLog::Appended,
        "/usd/a-very-long-path-that-keeps-going-and-going-and-going-past-sixty-characters.txt");
    CHECK_FALSE(boot.cut);
    CHECK(says(boot.line[1], "autons REFUSED (a typed-table boot: run shulib Calibrate)"));
    char phase[kCompetitionLineBytes];
    const int written = shulib::bench::competitionPhaseLine(
        phase, sizeof phase, "competition_initialize()", "normal return", 4294967295u,
        "0xff field=yes disabled=yes auton=yes", "the screen to the menu, and a note that goes on");
    CHECK(written >= 0);
    CHECK(static_cast<std::size_t>(written) < sizeof phase);
}

// ═══ COMP-b's review (2026-10-01): the cases the review found missing ═══════════════════════════

namespace {

/// A mechanism that records, at every safe-state application, the handshake's FINISHED at that
/// instant -- so a test can see whether a stop came before or after the terminal word.
struct FinishedAtSafeMechanism final : shulib::hal::IMechanism {
    const AutonHandshake* handshake = nullptr;
    std::vector<std::uint32_t> finishedAtSafe;
    void applySafeState() override { finishedAtSafe.push_back(handshake->finished()); }
    [[nodiscard]] const char* name() const noexcept override { return "probe pivot"; }
};

/// The opcontrol task's driver loop, modelled where it meets the auton task: one tick every 10 ms of
/// robot clock, from `firstAt` -- it beats, then checks its end (teleop_loop.hpp's step 1: the
/// program's driverControlMustEnd), and on the end runs its exit (chassis.cancel(); the bindings'
/// makeSafe has no mechanism here) and LEAVES. `deleted` models PROS deleting the task: no tick
/// runs at all, and it never leaves.
struct DriverTaskModel {
    CompetitionStack* s = nullptr;
    shulib::bench::DriverLoopPresence* presence = nullptr;
    std::uint8_t status = comp_rig::kAutonEnabled;  ///< what the loop's end check reads
    double nextAt = 0.0;
    bool alive = true;
    bool deleted = false;
    int ticks = 0;
    double exitAt = -1.0;           ///< the robot clock when its exit ran
    double plantAtExit = -1.0;      ///< plant travel when its exit ran
    /// Run every driver tick due by `now`.
    void advance(double now) {
        while (alive && !deleted && now + 1e-9 >= nextAt) {
            ++ticks;
            presence->beat();
            if (shulib::bench::driverControlMustEnd(status, s->handshake.busy())) {
                s->chassis.cancel();  // the loop's exit path, on the opcontrol task
                presence->leave();
                alive = false;
                exitAt = now;
                plantAtExit = s->plant.distance;
            }
            nextAt += 0.01;
        }
    }
};

}  // namespace

// (R18) Bug caught (RC1, the file banner's order -- the review's RV3): the program-error stop moved
// AFTER the terminal word. FINISHED hands the robot to opcontrol(); a stop after it would put the
// drive to 0 V + Brake and force the claws safe while the driver drives. A probe mechanism records
// FINISHED at every safe-state application: on both of RC1's throws, and at a clean run's end,
// every one comes BEFORE the word. (The END-estimate and fault reads cannot be watched from a
// single-threaded host: test/competition_source_test.cpp's (P15) pins their place.)
TEST_CASE("request (R18): the program-error stop and the guard's safing come BEFORE FINISHED") {
    for (const int path : {0, 1, 2}) {  // 0: the status painter throws; 1: a routine throws; 2: clean
        CAPTURE(path);
        CompetitionStack s{tank(), 1, 1.0, 50, &addThrowers};
        FinishedAtSafeMechanism probe;
        probe.handshake = &s.handshake;
        s.present = {&probe};
        s.choose(path == 1 ? 0 : 3);  // a throwing routine, or the team's 40 in
        s.obs.throwOnStart = path == 0;
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.threw == (path != 2));
        REQUIRE_FALSE(probe.finishedAtSafe.empty());  // safed on every path
        for (const std::uint32_t f : probe.finishedAtSafe) {
            CHECK(f < r.request);  // ...and always before the terminal word
        }
        CHECK(s.handshake.isFinished(r.request));
    }
}

// (R19) Bug caught (chapter 17: "this run's faults, never faults from before the run" -- the review's
// RV02/RV11/RV12/RV15, and the Auton Tester's own (3c), which this request lacked): a fault latched
// at boot blamed on the auton -- the count before the request never read, the latch's last fault
// reported though nothing came during the run, the count since boot printed, or the result page
// saying a fault when only the boot had one. Then a fault DURING the run: counted once, by name.
TEST_CASE("request (R19): a fault latched before the request is not this run's; one during it is") {
    {
        CompetitionStack s{tank()};
        s.rig.latch.raise(shulib::diag::FaultCode::GpsGateReject, "TST", "latched before the request");
        REQUIRE(s.rig.latch.faultCount() == 1);
        s.choose(kForty);
        const CompetitionRunRecord& r = s.serve();
        CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
        CHECK(r.faultsBefore == 1);
        CHECK(r.faultsAfter == 1);
        CHECK(r.lastFault == shulib::diag::FaultCode::None);
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        CHECK(says(lines.line[2], "; faults this run: none"));
        for (int i = 0; i < lines.count; ++i) {
            CHECK_FALSE(says(lines.line[i], "GPS_GATE_REJECT"));
        }
        const shulib::bench::CompetitionView v =
            shulib::bench::competitionResultView(r, s.registry, "BUILD 00:00:00");
        bool noneLine = false;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            noneLine = noneLine || std::string_view{v.lines[i].text} == "faults this run: none";
        }
        CHECK(noneLine);
    }
    {
        CompetitionStack s{tank()};
        s.rig.latch.raise(shulib::diag::FaultCode::GpsGateReject, "TST", "latched before the request");
        s.choose(kForty);
        const double start = s.now();
        bool raised = false;
        s.status.onRead = [&](double now) {
            if (!raised && now - start >= 0.5) {
                raised = true;
                s.rig.latch.raise(shulib::diag::FaultCode::MotionTimeout, "TST", "during the run");
            }
        };
        const CompetitionRunRecord& r = s.serve();
        REQUIRE(raised);
        CHECK(r.faultsBefore == 1);
        CHECK(r.faultsAfter == 2);
        CHECK(r.lastFault == shulib::diag::FaultCode::MotionTimeout);
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        CHECK(says(lines.line[2], "; faults this run: 1, last MOTION_TIMEOUT -- see the [ERROR] line"));
        const shulib::bench::CompetitionView v =
            shulib::bench::competitionResultView(r, s.registry, "BUILD 00:00:00");
        bool oneLine = false;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            oneLine = oneLine
                      || std::string_view{v.lines[i].text} == "faults this run: 1, last MOTION_TIMEOUT";
        }
        CHECK(oneLine);
    }
}

// (R20) Bug caught (the review's PB1): a request the auton task serves after its period has ENDED --
// behind a run held past its end -- re-seating the pose and commanding a tick before the watch's
// first read (runAuton's live-wait returns without a pace when the estimate is live: 0.6 in on the
// plant, the motors live under driver control, and a FIELD start re-seated where the robot is
// not). Now the status is read ONCE before anything moves: TOO LATE, nothing moved, no re-seat, the
// guard never armed, FINISHED -- for a disable and for driver control. And NONE served late is
// still NONE (nothing would have run).
TEST_CASE("request (R20): served after the period ended -- TOO LATE: no re-seat, no command") {
    for (const std::uint8_t late : {comp_rig::kDisabledOnField, comp_rig::kDriverOnField}) {
        CAPTURE(static_cast<int>(late));
        CompetitionStack s{tank()};
        s.choose(kForty);
        s.status.status = late;
        const double before = s.plant.distance;
        const Pose2d estimateBefore = s.rig.loc.pose();
        const int readsBefore = s.status.reads;
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.skip == CompetitionSkip::TooLate);
        CHECK(std::string_view{competitionResultWord(r)} == "TOO LATE");
        CHECK(s.status.reads == readsBefore + 1);  // ONE read, and no pace after it
        CHECK(s.plant.distance == before);        // nothing commanded
        CHECK(s.rig.loc.pose().x().value() == estimateBefore.x().value());  // no re-seat
        CHECK(s.rig.loc.pose().heading().radians() == estimateBefore.heading().radians());
        CHECK(r.run.armedAt.value() == 0.0);  // runAuton never ran: the guard never armed
        CHECK_FALSE(s.watch.armed());
        CHECK(r.cause == (late == comp_rig::kDisabledOnField ? CompetitionCause::Disabled
                                                             : CompetitionCause::LeftAutonomous));
        CHECK(r.statusAtLatch == late);
        CHECK(s.obs.skip == CompetitionSkip::TooLate);  // the status page says so too
        CHECK(s.handshake.isFinished(r.request));
        for (const shulib::hal::IMotor* m : s.rig.h.context().driveMotors()) {
            CHECK(m->commandedVoltage().value() == 0.0);  // nothing was ever commanded
        }
        CHECK(shulib::bench::competitionRunLevel(r) == shulib::hal::LogLevel::Warn);
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        REQUIRE(lines.count == 2);
        CHECK_FALSE(lines.cut);
        char want[kCompetitionLineBytes];
        std::snprintf(want, sizeof want,
                      "request 1: '40in forward' TOO LATE -- %s (status 0x%02x) when the auton task "
                      "served it, 0.000s after autonomous() asked; nothing moved",
                      late == comp_rig::kDisabledOnField ? "the robot was disabled"
                                                         : "the match left autonomous",
                      static_cast<unsigned>(late));
        CHECK(std::string_view{lines.line[0]} == want);
        CHECK(std::string_view{lines.line[1]}
              == "request 1: an auton runs only while autonomous is on, and this request was served "
                 "after its period; driver control still runs");
    }
    // A REFUSED boot served late is still REFUSED, with its own reason (precedence: the refusal is
    // what a person must act on -- a restart, or shulib Calibrate -- and it would have refused in
    // time too). The verification round's V11 (TOO LATE first) survived without this.
    for (const bool programError : {true, false}) {
        CAPTURE(programError);
        CompetitionStack late{tank()};
        late.choose(kForty);
        if (programError) {
            late.flags.programError.store(true);
        } else {
            late.flags.bootFromRecord = false;  // a typed-table boot (ND2)
        }
        late.status.status = comp_rig::kDisabledOnField;
        const CompetitionRunRecord& refused = late.serve();
        CHECK(refused.skip
              == (programError ? CompetitionSkip::AfterProgramError : CompetitionSkip::BootFromTable));
        CHECK(std::string_view{competitionResultWord(refused)} == "REFUSED");
        const CompetitionRunLines lines = competitionRunLines(refused, late.registry);
        CHECK(says(lines.line[0], "REFUSED"));
        CHECK_FALSE(says(lines.line[0], "TOO LATE"));
    }
    // NONE served late: NONE (precedence -- nothing would have run anyway).
    CompetitionStack s{tank()};
    s.status.status = comp_rig::kDisabledOnField;
    const CompetitionRunRecord& r = s.serve();
    CHECK(r.skip == CompetitionSkip::NoneChosen);
    // And a request served while autonomous is ON still runs (the read is not a refusal).
    CompetitionStack on{tank()};
    on.choose(kForty);
    CHECK(std::string_view{competitionResultWord(on.serve())} == "FINISHED");
}

// (R21) Bug caught (deviation 7 -- the review's PC: with HA-161 (a) FALSE, PROS does not delete
// opcontrol() at a direct change from driver control to autonomous; the driver loop sees the
// change only at its next tick, and its exit runs chassis.cancel() -- which, landing inside the
// auton's run, ended the 40 in "ROUTINE STOPPED ... (motion CANCELLED)" at every tick the review
// tried). Through the program's own composition (the auton task: its watcher observes the heartbeat
// on EVERY idle tick from boot, then waitForDriverLoop, then the request --
// test/competition_source_test.cpp's (P4) pins it): the watcher the request finds has watched the
// heartbeat since boot -- idle ticks with no loop, then a second of driver control, the loop beating
// every 10 ms of robot clock and the auton task observing every tick, half a tick out of step --
// and the 40 in is served only after the loop has LEFT: FINISHED, its exit before anything moved.
// A loop PROS DELETED at the change never leaves: the auton is served kDriverLoopStaleSeconds after
// the LAST beat the watcher saw. A loop that died long before the request costs nothing. A request
// in the tick between enter() and the loop's first beat still waits (enter() beats). The watcher
// counts from the last beat it saw, never the first. And enter() never claims the robot while a
// request waits. (The verification round, 2026-10-01: this case first seeded a FRESH watcher 5 ms
// before the request -- a state the program is never in at a request -- so a watcher that never
// saw a later beat (V01), and an enter() that did not beat (V05), both survived the suite.)
TEST_CASE("request (R21): the driver loop LEAVES before the auton task serves (HA-161 (a) false)") {
    using shulib::bench::DriverLoopPresence;
    using shulib::bench::DriverLoopWait;
    using shulib::bench::DriverLoopWatcher;
    using shulib::bench::kDriverLoopStaleSeconds;
    // The REASONED margin (register HA-161): ten of the loop's 10 ms ticks. A lower value takes a
    // slow driver tick (a whole page painted) for a deleted loop -- the review's C2 failure, back --
    // so lowering it is a decision for the register, not an edit.
    CHECK(kDriverLoopStaleSeconds >= 10 * 0.01 - 1e-12);
    for (const bool deleted : {false, true}) {
        CAPTURE(deleted);
        CompetitionStack s{tank()};
        s.choose(kForty);
        DriverLoopPresence presence;
        DriverLoopWatcher watcher;
        DriverTaskModel driver;
        driver.s = &s;
        driver.presence = &presence;
        driver.status = comp_rig::kDriverOnField;  // driver control, until the change
        // The auton task's idle ticks from boot: half a second with no driver loop ...
        for (int i = 0; i < 50; ++i) {
            s.plant.pace();
            watcher.observe(presence, s.now());
        }
        // ... then opcontrol() claims the robot, and a second of driver control: the loop beats
        // every 10 ms, the auton task observes every tick, half a tick out of step.
        REQUIRE(presence.enter(s.handshake));
        driver.nextAt = s.now() + 0.005;
        for (int i = 0; i < 100; ++i) {
            s.plant.pace();
            driver.advance(s.now());
            watcher.observe(presence, s.now());
        }
        REQUIRE(driver.ticks == 100);
        REQUIRE(driver.alive);
        // The change to autonomous: PROS deletes the loop (deleted), or does not (HA-161 (a) false:
        // it reads the autonomous bit at its next tick).
        driver.status = comp_rig::kAutonEnabled;
        driver.deleted = deleted;
        const int ticksBefore = driver.ticks;
        const double t0 = s.now();  // the auton task's last idle tick saw the 100th beat here
        s.status.onRead = [&](double now) { driver.advance(now); };  // it runs on, during the run
        const std::uint32_t n = s.handshake.request(t0);  // autonomous() asks
        const DriverLoopWait w = shulib::bench::waitForDriverLoop(
            presence, watcher, [&s] { return s.now(); },
            [&] {
                s.plant.pace();  // a tick of the auton task's sleep: the world moves on
                driver.advance(s.now());
            });
        CHECK(w.waited);
        CHECK(w.left == !deleted);
        if (deleted) {
            // Stale from the LAST beat the watcher saw (at t0), never from its first sight at boot.
            CHECK(w.seconds >= kDriverLoopStaleSeconds - 1e-9);
            CHECK(w.seconds <= kDriverLoopStaleSeconds + 0.0101);
            CHECK(driver.ticks == ticksBefore);
        } else {
            CHECK(w.seconds <= 0.0101);  // the loop leaves at its next tick
            CHECK(driver.ticks == ticksBefore + 1);
            CHECK(driver.exitAt >= t0);
        }
        const double before = s.plant.distance;
        std::uint32_t request = 0;
        double at = 0.0;
        REQUIRE(s.handshake.pending(0, request, at));
        REQUIRE(request == n);
        shulib::bench::CompetitionParts p = s.parts();
        const CompetitionRunRecord& r =
            shulib::bench::serveAutonRequest(p, s.rec, request, at, 1, s.obs, w);
        CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
        CHECK(s.plant.distance - before > 39.0);
        if (!deleted) {
            CHECK(driver.plantAtExit >= 0.0);     // its exit ran ...
            CHECK(driver.plantAtExit <= before);  // ... before the auton moved
        }
        // The request's lines say what the wait found (HA-161 (a)'s evidence).
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        REQUIRE(lines.count == 5);
        CHECK_FALSE(lines.cut);
        CHECK(says(lines.line[4], deleted ? "its heartbeat STOPPED (deleted by PROS, or stalled)"
                                          : "a driver loop still held the robot when it was served -- "
                                            "it LEFT; the auton waited 0.010s"));
    }
    // A loop that died LONG before the request (PROS deleted it at a disable): no wait at all.
    {
        DriverLoopPresence presence;
        AutonHandshake h;
        REQUIRE(presence.enter(h));  // ...and never left
        DriverLoopWatcher watcher;
        watcher.observe(presence, 10.0);  // the auton task's idle ticks since: no beat
        watcher.observe(presence, 11.0);
        double t = 11.0;
        int ticks = 0;
        const DriverLoopWait w = shulib::bench::waitForDriverLoop(
            presence, watcher, [&t] { return t; }, [&] {
                ++ticks;
                t += 0.01;
            });
        CHECK_FALSE(w.waited);
        CHECK(ticks == 0);
    }
    // No loop at all: no wait.
    {
        DriverLoopPresence presence;
        DriverLoopWatcher watcher;
        int ticks = 0;
        const DriverLoopWait w = shulib::bench::waitForDriverLoop(
            presence, watcher, [] { return 3.0; }, [&ticks] { ++ticks; });
        CHECK_FALSE(w.waited);
        CHECK(ticks == 0);
    }
    // The watcher itself: a loop watched beating since boot holds the robot until
    // kDriverLoopStaleSeconds after the LAST beat seen -- one more beat restarts the count from it --
    // and a loop that LEFT is let go at once. (The verification round's V01: a watcher that kept its
    // FIRST sighting took a live loop, watched since boot, for a deleted one at every request.)
    {
        AutonHandshake h;
        DriverLoopPresence presence;
        DriverLoopWatcher watcher;
        watcher.observe(presence, 0.0);  // boot: the auton task's first idle tick, no loop
        REQUIRE(presence.enter(h));
        double last = 0.0;
        for (int i = 0; i < 200; ++i) {  // two seconds of driver ticks, every beat seen
            presence.beat();
            last = 1.0 + 0.01 * i;
            watcher.observe(presence, last);
        }
        watcher.observe(presence, last + 0.05);  // an idle tick with no beat since: no change
        CHECK_FALSE(watcher.mayServe(presence, last + 0.05));
        CHECK_FALSE(watcher.mayServe(presence, last + kDriverLoopStaleSeconds - 0.001));
        CHECK(watcher.mayServe(presence, last + kDriverLoopStaleSeconds + 1e-9));
        presence.beat();  // a late beat (a slow tick), seen half a second on
        watcher.observe(presence, last + 0.5);
        CHECK_FALSE(watcher.mayServe(presence, last + 0.5 + kDriverLoopStaleSeconds - 0.001));
        CHECK(watcher.mayServe(presence, last + 0.5 + kDriverLoopStaleSeconds + 1e-9));
        presence.leave();
        CHECK(watcher.mayServe(presence, last + 0.5));  // LEFT: at once, whatever the heartbeat
    }
    // A request in the tick between enter() and the loop's first beat (the verification round's
    // V05): enter() itself beats, so a loop that has only just claimed the robot is PRESENT although
    // the watcher last saw the heartbeat move long before (an earlier driver period's loop, which
    // left at the disable). The auton task waits for the loop's first tick, which leaves.
    {
        AutonHandshake h;
        DriverLoopPresence presence;
        DriverLoopWatcher watcher;
        REQUIRE(presence.enter(h));  // an earlier driver period ...
        presence.beat();
        presence.leave();  // ... that left at the disable
        watcher.observe(presence, 1.0);
        watcher.observe(presence, 20.0);  // idle ticks since: no beat
        REQUIRE(presence.enter(h));       // driver control again: claimed, not yet ticked
        double t = 20.005;
        (void)h.request(t);  // autonomous() asks before the loop's first tick
        bool alive = true;
        int ticks = 0;
        const DriverLoopWait w = shulib::bench::waitForDriverLoop(
            presence, watcher, [&t] { return t; }, [&] {
                t += 0.01;
                ++ticks;
                if (alive) {  // the loop's first tick: it beats, sees the request, and leaves
                    presence.beat();
                    if (shulib::bench::driverControlMustEnd(comp_rig::kDriverOnField, h.busy())) {
                        presence.leave();
                        alive = false;
                    }
                }
            });
        CHECK(w.waited);
        CHECK(w.left);
        CHECK(ticks == 1);
    }
    // enter(): refused (and nothing claimed) while a request waits; claimed once it is FINISHED.
    {
        AutonHandshake h;
        DriverLoopPresence presence;
        const std::uint32_t n = h.request(1.0);
        CHECK_FALSE(presence.enter(h));
        CHECK_FALSE(presence.active());
        h.finish(n);
        CHECK(presence.enter(h));
        CHECK(presence.active());
        presence.leave();
        CHECK_FALSE(presence.active());
    }
}

// (R22) Bug caught (the review's PD): a program error whose page has no reason -- the
// autons-refused page read an empty (or an older) reason after a throw in the REPORT, and its
// headline said the auton task stopped when it had not. Each source records its own words: a
// request that threw (the robot stopped), and its report (the robot not touched).
TEST_CASE("request (R22): every program error records its own reason for the autons-refused page") {
    {
        CompetitionStack s{tank()};
        s.choose(kForty);
        s.obs.throwOnStart = true;  // a runtime_error from the status painter
        char errorWhat[shulib::bench::kCompetitionErrorBytes] = "an older reason";
        const std::uint32_t n = s.handshake.request(s.now());
        shulib::bench::CompetitionParts p = s.parts();
        (void)shulib::bench::runAutonRequest(p, s.rec, n, s.now(), 1, s.obs, nullptr, errorWhat);
        CHECK(s.flags.programError.load());
        CHECK(std::string_view{errorWhat}
              == "request 1 threw: probe: the status painter threw a runtime_error -- the robot "
                 "was stopped");
    }
    {
        CompetitionStack s{tank()};
        s.choose(kForty);
        comp_rig::ThrowingSink broken;  // the report's first serial line throws
        s.telemetryOverride = &broken;
        char errorWhat[shulib::bench::kCompetitionErrorBytes] = "";
        const std::uint32_t n = s.handshake.request(s.now());
        shulib::bench::CompetitionParts p = s.parts();
        broken.throwAt = 1000;  // the run's own lines pass ...
        (void)shulib::bench::serveAutonRequest(p, s.rec, n, s.now(), 1, s.obs);
        REQUIRE(std::string_view{competitionResultWord(s.rec)} == "FINISHED");
        broken.throwAt = 0;  // ... and the report's do not
        CHECK(shulib::bench::reportAutonRun(s.rec, s.registry, broken, nullptr, s.flags, errorWhat));
        CHECK(s.flags.programError.load());
        CHECK_FALSE(s.rec.threw);
        CHECK(std::string_view{errorWhat}
              == "the report of request 1 threw: probe: the log sink threw -- the robot was not "
                 "touched");
    }
}

// (R23) Bug caught (the review's RV03, RV05-RV10): the words Station K reads, unpinned -- a NONE
// request's throw reading NONE, every request's lines logged at one level, TIMEOUT and CANCELLED
// swapped, boot line 0 without the build hash, the phase lines' millis lost, an estimate never
// ticked reading LIVE.
TEST_CASE("request (R23): the levels, the stop words, the boot and phase lines, NOT LIVE") {
    // A throw on a NONE request is a PROGRAM ERROR, in the word as in the lines.
    {
        CompetitionStack s{tank()};
        s.obs.throwOnStart = true;
        const CompetitionRunRecord& r = s.serve();
        CHECK(r.threw);
        CHECK(std::string_view{competitionResultWord(r)} == "PROGRAM ERROR");
    }
    // An estimate never ticked reads NOT LIVE (a NONE request on a robot the boot never ticked).
    {
        CompetitionStack s{tank(), 1, 1.0, 0};
        const CompetitionRunRecord& r = s.serve();
        CHECK_FALSE(r.endLive);
        CompetitionRunRecord run{};
        run.request = 3;
        run.entry = -1;
        run.run.outcome = AutonRunOutcome::RoutineRan;
        run.endLive = false;
        AutonRegistry reg{shulib::bench::kAutonPeriods};
        const CompetitionRunLines lines = competitionRunLines(run, reg);
        REQUIRE(lines.count == 4);
        CHECK(says(lines.line[3], "facing 0.0 deg NOT LIVE"));
    }
    // The level each word is logged at.
    {
        const auto levelOf = [](CompetitionRunRecord r) { return shulib::bench::competitionRunLevel(r); };
        CompetitionRunRecord r{};
        r.threw = true;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Error);  // PROGRAM ERROR
        r = CompetitionRunRecord{};
        r.skip = CompetitionSkip::NoneChosen;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Info);  // NONE
        r.skip = CompetitionSkip::BootFromTable;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Warn);  // REFUSED
        r.skip = CompetitionSkip::TooLate;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Warn);  // TOO LATE
        r = CompetitionRunRecord{};
        r.run.outcome = AutonRunOutcome::RoutineRan;
        r.run.routine.ok = true;
        r.run.guard.endActionSucceeded = true;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Info);  // FINISHED
        r.run.guard.endActionSucceeded = false;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Warn);  // ROUTINE STOPPED
        r.run.guard.abandoned = true;
        r.cause = CompetitionCause::Disabled;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Warn);  // ABANDONED
        r.run.guard.abandoned = false;
        CHECK(levelOf(r) == shulib::hal::LogLevel::Error);  // ABANDON IGNORED
    }
    // The routine's stop, in its own words.
    {
        using shulib::chassis::RoutineStopCause;
        using shulib::control::ExitReason;
        const auto word = [](RoutineStopCause c, ExitReason e) {
            return std::string_view{shulib::bench::competitionStopWord(c, e)};
        };
        CHECK(word(RoutineStopCause::None, ExitReason::Settled) == "no stop");
        CHECK(word(RoutineStopCause::MotionFailed, ExitReason::TimedOut) == "motion TIMEOUT");
        CHECK(word(RoutineStopCause::MotionFailed, ExitReason::Cancelled) == "motion CANCELLED");
        CHECK(word(RoutineStopCause::MotionFailed, ExitReason::Settled) == "motion failed");
        CHECK(word(RoutineStopCause::WaitTimedOut, ExitReason::Settled) == "wait TIMEOUT");
        CHECK(word(RoutineStopCause::ActionFailed, ExitReason::Settled) == "action FAILED");
        CHECK(word(RoutineStopCause::MechanismFailed, ExitReason::Settled) == "mechanism FAILED");
        CHECK(word(RoutineStopCause::MechanismStepFailed, ExitReason::Settled) == "mechanism step FAILED");
    }
    // The boot's three lines, whole, at a real build hash.
    {
        const shulib::bench::CompetitionBootLines boot = shulib::bench::competitionBootLines(
            "shulib-robot-variant=tank-competition", "v0.1.1-356-g0f56301-dirty", "RECORD", true, 2,
            3, 3, shulib::bench::AutonRunLog::Appended, "/usd/shulib_auton_runs.txt");
        CHECK_FALSE(boot.cut);
        CHECK(std::string_view{boot.line[0]}
              == "boot: shulib COMPETITION [shulib-robot-variant=tank-competition] hash "
                 "v0.1.1-356-g0f56301-dirty");
        CHECK(std::string_view{boot.line[1]}
              == "boot source RECORD, 2 autons, 3 of 3 mechanisms present; autons ALLOWED");
        CHECK(std::string_view{boot.line[2]}
              == "SD run log ON, APPENDED to /usd/shulib_auton_runs.txt");
        const shulib::bench::CompetitionBootLines missing = shulib::bench::competitionBootLines(
            "v", "", "RECORD", true, 2, 3, 3, shulib::bench::AutonRunLog::Off, "/usd/x");
        CHECK(std::string_view{missing.line[0]} == "boot: shulib COMPETITION [v] hash MISSING");
        CHECK(std::string_view{missing.line[2]} == "SD run log OFF (no card, or the log would not open)");
    }
    // A phase line (B5(1)), whole: Station K reads its millis against the other lines.
    {
        char line[kCompetitionLineBytes];
        (void)shulib::bench::competitionPhaseLine(line, sizeof line, "autonomous()", "entry", 61234u,
                                                  "0x06 field=yes disabled=no auton=yes", "");
        CHECK(std::string_view{line}
              == "autonomous() entry at 61234 ms, status 0x06 field=yes disabled=no auton=yes");
        (void)shulib::bench::competitionPhaseLine(line, sizeof line, "disabled()", "normal return",
                                                  7u, "0x05", "the screen to the menu");
        CHECK(std::string_view{line}
              == "disabled() normal return at 7 ms, status 0x05 -- the screen to the menu");
    }
}

// (R24) Bug caught (RC9, B17 -- the review found Station K quoting only part of COMP-t's re-predicted
// table): Station K's expected results ARE Station M's, because the competition request makes the
// tester's call on the same entry -- but only if the request path really reproduces them. Through
// THIS program's request on the host test plant: a 9 % weak right side gives the 40 in FINISHED at
// T+1.89 s, END (39.64, 0.02, 0.10 deg), and the acceptance run FINISHED at T+6.14 s, END (0.37,
// 0.06, 177.28 deg) -- COMP-t's table; the honest-stop thresholds are COMP-t's too: the 40 in
// finishes down to x0.723 and stops at x0.722, the acceptance run finishes to x0.724 and stops at
// x0.723 -- and a stop is the request's line 3, "stopped at <step> (<cause>)" with the fault by
// name (the K.5.3 row Station K records). Host test plant numbers; nothing here ran on a robot.
TEST_CASE("request (R24): Station M's predictions, through the competition request (x0.91; the thresholds)") {
    const auto runAt = [](int entry, double weakScale) {
        CompetitionStack s{tank(), /*weakWheel=*/1, weakScale};
        s.choose(entry);
        const CompetitionRunRecord r = s.serve();
        const CompetitionRunLines lines = competitionRunLines(r, s.registry);
        return std::make_pair(r, std::string{lines.line[2]});
    };
    {
        const auto [r, line3] = runAt(kForty, 0.91);
        CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
        CHECK(r.run.guard.scoringEnded.value() == doctest::Approx(1.89).epsilon(0.005));
        CHECK(r.endEstimate.x().value() == doctest::Approx(39.64).epsilon(0.0005));
        CHECK(std::abs(r.endEstimate.y().value() - 0.02) < 0.01);
        CHECK(std::abs(r.endEstimate.heading().degrees() - 0.10) < 0.01);
    }
    {
        const auto [r, line3] = runAt(kAcceptance, 0.91);
        CHECK(std::string_view{competitionResultWord(r)} == "FINISHED");
        CHECK(r.run.guard.scoringEnded.value() == doctest::Approx(6.14).epsilon(0.005));
        CHECK(std::abs(r.endEstimate.x().value() - 0.37) < 0.01);
        CHECK(std::abs(r.endEstimate.y().value() - 0.06) < 0.01);
        CHECK(std::abs(r.endEstimate.heading().degrees() - 177.28) < 0.01);
    }
    CHECK(std::string_view{competitionResultWord(runAt(kForty, 0.723).first)} == "FINISHED");
    CHECK(std::string_view{competitionResultWord(runAt(kAcceptance, 0.724).first)} == "FINISHED");
    for (const auto& [entry, scale] : {std::pair{kForty, 0.722}, std::pair{kAcceptance, 0.723}}) {
        CAPTURE(entry);
        const auto [r, line3] = runAt(entry, scale);
        CHECK(std::string_view{competitionResultWord(r)} == "ROUTINE STOPPED");
        CHECK(says(line3, "routine STOPPED, "));
        CHECK(says(line3, " (motion TIMEOUT); faults this run: 1, last MOTION_TIMEOUT -- see the "
                          "[ERROR] line"));
    }
}
