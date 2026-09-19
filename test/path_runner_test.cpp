// runner::PathRunner — a Route, executed (chunk G2, brief §4 tests 3–10).
//
// ── The keystone is test 3, and it is an ACCEPTANCE test, not a smoke test ──────────
// G2's whole claim is that the runner adds A WAY TO AUTHOR and not a different physics. The
// only test that can show that is a differential one: the same five legs written once as data
// and once as a `Routine` chain, driven across two identically-seeded A2 plants, required to
// agree BIT FOR BIT — per-tick ground truth by `memcmp`, every commanded volt as an exact
// double, the exit reason and the tick count equal. It is robot_builder_test.cpp's move (G1's
// "the same robot, wired twice") applied one tier up, and for the same reason: a tolerance
// would hide exactly the defect this exists to catch — a bearing computed with the `atan2`
// arguments swapped, a heading defaulted instead of read, an option not plumbed.
//
// The `Routine` arm is written out longhand rather than through any helper the runner also
// uses. A shared helper would make the two sides agree by construction and prove nothing.
//
// ── The rest is the error policy, which is the part that costs a match ──────────────
// Markers that fire at the wrong place, a route that keeps driving after a leg it never
// reached, an unknown id that ends an autonomous period instead of logging a line — each of
// those is a real failure mode with a rule in the header, and each has a case here that
// breaks it deliberately.
//
// NO HARDWARE CLAIM. Everything here runs on the A2 host plant behind the F4 fakes. No Route
// has ever executed on a physical robot, and the library's motion stack has never driven one.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/path_runner.hpp"
#include "shulib/sim/scenario.hpp"

namespace cfgns = shulib::config;
using shulib::chassis::Routine;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::manipulation::MechanismOutcome;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::runner::CommandRegistry;
using shulib::runner::PathRunner;
using shulib::runner::RunReport;
using shulib::runner::RunStopCause;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Everything a run produces that must match bit for bit (robot_builder_test.cpp's Trace, the
/// same idea one tier up).
struct Trace {
    std::vector<TruthSample> truth;  ///< ground truth at each tick, memcmp-compared
    std::vector<double> volts;       ///< every plant wheel's commanded volts, per tick
    int ticks = 0;
};

/// motion_test_rig.hpp's PlantPacer with a recorder in front of the step: it samples AFTER the
/// tick and BEFORE the plant advances, so a sample is exactly the state the tick was rendered on.
/// Hard-capped, so a wait that never ends turns a test RED instead of hanging the suite.
struct RecordingPacer final : shulib::motion::ITickPacer {
    explicit RecordingPacer(SimHarness& harness, Time dt = Time{0.01}, int cap = 200000)
        : h{&harness}, tickDt{dt}, capPaces{cap} {}

    void pace() override {
        if (trace.ticks >= capPaces) {
            throw std::runtime_error("RecordingPacer: pace cap exceeded — a wait failed to end");
        }
        ++trace.ticks;
        trace.truth.push_back(h->sample());
        for (int i = 0; i < h->motorCount(); ++i) {
            trace.volts.push_back(h->motor(i).commandedVoltage().value());
        }
        h->plant().step(tickDt);
    }

    SimHarness* h;
    Time tickDt;
    int capPaces;
    Trace trace;
};

/// MotionRig + a recording pacer + a Chassis + a sink for the runner's own lines.
struct RunnerRig {
    motion_rig::MotionRig rig;
    RecordingPacer pacer;
    shulib::chassis::Chassis chassis;
    FakeTelemetrySink telemetry;

    explicit RunnerRig(const shulib::kinematics::IKinematics& kin,
                       const SimHarnessConfig& cfg = motion_rig::plantConfig())
        : rig{kin, cfg}, pacer{rig.h}, chassis{rig.deps, pacer, motion_rig::chassisConfig()} {}
};

/// The 2026 tank stand-in every tank case here uses (C3's geometry).
shulib::kinematics::TankKinematics tankKin() {
    return shulib::kinematics::TankKinematics{Length{12.0}};
}

bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}

/// Bit identity, spelled out: never Approx.
void requireIdentical(const Trace& a, const Trace& b) {
    REQUIRE(a.ticks == b.ticks);
    REQUIRE(sameTruth(a.truth, b.truth));
    REQUIRE(a.volts.size() == b.volts.size());
    for (std::size_t i = 0; i < a.volts.size(); ++i) {
        REQUIRE(a.volts[i] == b.volts[i]);  // bits, not Approx
    }
}

/// A plant whose truth starts where the route's Start says it does, so seeding the estimate is
/// not also a teleport.
SimHarnessConfig plantAt(const Pose2d& pose) {
    SimHarnessConfig cfg = motion_rig::plantConfig();
    cfg.plant.initialPose = pose;
    return cfg;
}

constexpr double kStartX = -24.0;
constexpr double kStartY = -12.0;
constexpr double kStartHeadingDeg = 90.0;

Pose2d startPose() {
    return Pose2d{Length{kStartX}, Length{kStartY}, Angle::degrees(kStartHeadingDeg)};
}

/// The five-leg acceptance route: one of every verb that moves, plus the Start.
/// `legTimeout` is applied to the three driving legs so both drivetrains get an honest budget.
cfgns::Route acceptanceRoute(double legTimeout) {
    const cfgns::Constraints budget{.timeout = Time{legTimeout}};
    cfgns::Route r{.name = "acceptance"};
    r.waypoints[0] = {.x = Length{kStartX},
                      .y = Length{kStartY},
                      .heading = Angle::degrees(kStartHeadingDeg),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::Start};
    r.waypoints[1] = {.x = Length{0.0},
                      .y = Length{24.0},
                      .headingMode = cfgns::HeadingMode::Free,
                      .motion = cfgns::WaypointMotion::DriveTo,
                      .constraints = budget};
    r.waypoints[2] = {.heading = Angle::degrees(0.0),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::TurnTo,
                      .constraints = budget};
    r.waypoints[3] = {.x = Length{12.0},
                      .y = Length{36.0},
                      .heading = Angle::degrees(45.0),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::MoveToPose,
                      .constraints = budget};
    r.waypoints[4] = {.headingMode = cfgns::HeadingMode::Free,
                      .motion = cfgns::WaypointMotion::Hold,
                      .constraints = {.timeout = Time{0.3}}};
    r.count = 5;
    return r;
}

/// The SAME route, AUTHORED FOR A TANK: the two heading changes are waypoints of their own.
///
/// This is not a workaround, it is the drivetrain contract showing through the data layer. The
/// facade never sequences a turn it was not asked for (D12 / C1's honesty: `moveTo` on a tank
/// translates along the heading it is servoing to, and a target off that line is refused by
/// TIMING OUT rather than by quietly arcing there). `chassis_routine_test.cpp`'s tank cases
/// carry exactly this shape in the recipe layer — "the AUTHOR plans the turn" — and the runner
/// inherits it unchanged, because the runner adds a way to author and not a physics.
///
/// The turns are computed HERE, at authoring time, from the nominal path — which is what a path
/// editor has too. That is the honest version of the test: a route is written before the match
/// and cannot read the live pose, so the residual is the drift between nominal and actual, and
/// the arrival bounds below are the ones that have to absorb it.
cfgns::Route tankAcceptanceRoute(double legTimeout) {
    const cfgns::Constraints budget{.timeout = Time{legTimeout}};
    // Bearing from the Start to the DriveTo's point, and from there to the MoveToPose's.
    const double leg1 = std::atan2(24.0 - kStartY, 0.0 - kStartX);
    const double leg2 = std::atan2(36.0 - 24.0, 12.0 - 0.0);

    cfgns::Route r{.name = "acceptance_tank"};
    r.waypoints[0] = {.x = Length{kStartX},
                      .y = Length{kStartY},
                      .heading = Angle::degrees(kStartHeadingDeg),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::Start};
    r.waypoints[1] = {.heading = Angle::radians(leg1),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::TurnTo,
                      .constraints = budget};
    r.waypoints[2] = {.x = Length{0.0},
                      .y = Length{24.0},
                      .headingMode = cfgns::HeadingMode::Free,
                      .motion = cfgns::WaypointMotion::DriveTo,
                      .constraints = budget};
    r.waypoints[3] = {.heading = Angle::radians(leg2),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::TurnTo,
                      .constraints = budget};
    r.waypoints[4] = {.x = Length{12.0},
                      .y = Length{36.0},
                      .heading = Angle::radians(leg2),
                      .headingMode = cfgns::HeadingMode::Fixed,
                      .motion = cfgns::WaypointMotion::MoveToPose,
                      .constraints = budget};
    r.waypoints[5] = {.headingMode = cfgns::HeadingMode::Free,
                      .motion = cfgns::WaypointMotion::Hold,
                      .constraints = {.timeout = Time{0.3}}};
    r.count = 6;
    return r;
}

/// The SAME five legs, written longhand as a recipe chain (file banner: deliberately not
/// factored through anything the runner uses).
void acceptanceRoutine(shulib::chassis::Chassis& chassis, double legTimeout) {
    const shulib::chassis::MotionOptions budget{.timeout = Time{legTimeout}};
    Routine r{chassis, "acceptance"};
    r.startAt(startPose())
        .driveTo(Length{0.0}, Length{24.0}, budget)
        .turnTo(Angle::degrees(0.0), budget)
        .moveTo(Pose2d{Length{12.0}, Length{36.0}, Angle::degrees(45.0)}, budget)
        .hold(Time{0.3});
}

/// How many entries of `sink` are Warn lines containing `needle`.
int warnCount(const FakeTelemetrySink& sink, std::string_view needle) {
    int n = 0;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).level == LogLevel::Warn
            && sink.at(i).message.find(needle) != std::string::npos) {
            ++n;
        }
    }
    return n;
}

/// The first entry at `level` containing `needle`, or "" when there is none.
std::string lineWith(const FakeTelemetrySink& sink, LogLevel level, std::string_view needle) {
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).level == level && sink.at(i).message.find(needle) != std::string::npos) {
            return sink.at(i).message;
        }
    }
    return {};
}

/// Every plant motor at 0 V under Brake — what `Chassis::cancel()` leaves behind.
bool driveIsSafed(SimHarness& h) {
    for (int i = 0; i < h.motorCount(); ++i) {
        if (h.motor(i).commandedVoltage().value() != 0.0
            || h.motor(i).brakeMode() != BrakeMode::Brake) {
            return false;
        }
    }
    return true;
}

}  // namespace

// ═══ 3. THE ACCEPTANCE TEST: a route and a recipe are bit-identical ══════════════════
// Bug caught: the runner computing ANYTHING differently from the recipe layer for the same
// intent — above all the DriveTo bearing, which is the one piece of arithmetic that exists in
// both files (ruling R1). `atan2(dx, dy)` instead of `atan2(dy, dx)` is a plausible typo that
// no accuracy bound would catch on a symmetric field, and it is precisely what this memcmp does.
TEST_CASE("G2 runner 3: the same five legs as DATA and as a RECIPE drive the plant bit-identically") {
    constexpr double kLegTimeout = 12.0;
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});

    // Arm A — the route, through PathRunner.
    RunnerRig a{kin, plantAt(startPose())};
    CommandRegistry registry;  // no markers on this route: the physics is the claim here
    PathRunner runner{a.chassis, registry, a.telemetry};
    const RunReport report = runner.run(acceptanceRoute(kLegTimeout));

    // Arm B — the same five legs, hand-written as a Routine.
    RunnerRig b{kin, plantAt(startPose())};
    acceptanceRoutine(b.chassis, kLegTimeout);

    requireIdentical(a.pacer.trace, b.pacer.trace);
    CHECK(a.chassis.lastExitReason() == b.chassis.lastExitReason());
    CHECK(a.pacer.trace.ticks > 100);  // it really ran; an empty trace is trivially identical

    // …and the report says what a reader would expect of it.
    CHECK(report.ok());
    CHECK(report.legsRun == 5);
    CHECK(report.legsSkipped == 0);
    CHECK(report.markersFired == 0);
    CHECK(report.unknownIds == 0);
    CHECK(report.stoppedAt == -1);
    CHECK(report.reason == RunStopCause::None);
    CHECK(report.lastExit == ExitReason::Settled);
}

// ═══ 4. Both drivetrains actually arrive — each authored for what it can do ══════════
// Bug caught: a vocabulary that only works on the holonomic drive. Graded on GROUND TRUTH, never
// on the estimate the motions themselves read.
//
// MEASURED HERE, and it is the finding of this test rather than a footnote to it: the X-drive
// route does NOT run on a tank, and it should not. Its first leg is a `DriveTo` to a point 34°
// off the start heading, and on a tank `moveTo` translates along the heading it servos to — so
// the leg exits TimedOut, the error policy stops the route at waypoint 1 and safes the drive.
// That is the facade's D12 honesty ("the library never sequences a turn you did not ask for")
// arriving intact at the data layer, which is exactly what R1 claims the runner is for. A tank
// route is AUTHORED with its turns, the same way a tank recipe is written with them
// (`tankAcceptanceRoute`), and the second subcase pins both halves: the authored one arrives,
// and the un-authored one fails in the defined way rather than in some new way.
TEST_CASE("G2 runner 4: the acceptance route settles on ground truth — X-drive and tank") {
    constexpr double kLegTimeout = 14.0;

    SUBCASE("X-drive") {
        const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(acceptanceRoute(kLegTimeout));
        REQUIRE(report.ok());
        const Pose2d truth = r.rig.h.truePose();
        CHECK(motion_rig::posErr(truth, Pose2d{Length{12.0}, Length{36.0}, Angle{}}) < 2.0);
        CHECK(std::abs(truth.heading().errorTo(Angle::degrees(45.0))) < 0.10);
    }

    SUBCASE("tank, authored for a tank: the turns are waypoints") {
        const TankKinematics kin = tankKin();
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const cfgns::Route route = tankAcceptanceRoute(kLegTimeout);
        REQUIRE(route.valid());
        const RunReport report = runner.run(route);
        REQUIRE(report.ok());
        CHECK(report.legsRun == 6);
        const Pose2d truth = r.rig.h.truePose();
        const double bearing = std::atan2(36.0 - 24.0, 12.0 - 0.0);
        CHECK(motion_rig::posErr(truth, Pose2d{Length{12.0}, Length{36.0}, Angle{}}) < 2.0);
        CHECK(std::abs(truth.heading().errorTo(Angle::radians(bearing))) < 0.10);
    }

    SUBCASE("tank, given the X-drive route: it fails HONESTLY, not silently") {
        const TankKinematics kin = tankKin();
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(acceptanceRoute(kLegTimeout));
        CHECK_FALSE(report.ok());
        CHECK(report.reason == RunStopCause::MotionFailed);
        CHECK(report.stoppedAt == 1);            // the DriveTo whose approach is off the heading
        CHECK(report.lastExit == ExitReason::TimedOut);
        CHECK(report.legsSkipped == 3);
        CHECK(driveIsSafed(r.rig.h));            // and it parked, rather than chasing the field
    }
}

// ═══ 5. Markers fire IN ORDER, and only after Settled (ruling R2) ════════════════════
// Bug caught, twice over: a marker fired before its leg settles (a clamp closing where the robot
// isn't yet), and a route that keeps going after a leg it never finished (every later leg then
// drives from a position the robot is not at — D1's whole argument).
TEST_CASE("G2 runner 5: markers fire in order after Settled; a timed-out leg fires none and stops") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});

    SUBCASE("in order, and after the leg — never during it") {
        RunnerRig r{kin, plantAt(startPose())};
        std::string order;
        double firedAtY = -999.0;
        auto first = [&] { order += "1"; firedAtY = r.rig.h.truePose().y().value(); };
        auto second = [&] { order += "2"; };
        auto third = [&] { order += "3"; };

        CommandRegistry registry;
        registry.on("first", first);
        registry.on("second", second);
        registry.on("third", third);

        cfgns::Route route = acceptanceRoute(12.0);
        route.waypoints[1].markers[0] = cfgns::Marker{.id = "first"};
        route.waypoints[1].markers[1] = cfgns::Marker{.id = "second"};
        route.waypoints[1].markerCount = 2;
        route.waypoints[3].markers[0] = cfgns::Marker{.id = "third"};
        route.waypoints[3].markerCount = 1;

        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);

        REQUIRE(report.ok());
        CHECK(order == "123");        // in order, and across waypoints in waypoint order
        CHECK(report.markersFired == 3);
        CHECK(report.legsRun == 5);
        CHECK(report.legsSkipped == 0);
        // The leg the first two hang off ends at y = 24: firing DURING the leg would have caught
        // the robot short of it. (The tolerance is the settle tolerance, not a fudge factor.)
        CHECK(firedAtY > 22.0);
    }

    SUBCASE("a TimedOut leg fires NO markers and stops the route, counting what never ran") {
        RunnerRig r{kin, plantAt(startPose())};
        int fired = 0;
        auto count = [&] { ++fired; };
        CommandRegistry registry;
        registry.on("count_me", count);

        cfgns::Route route = acceptanceRoute(12.0);
        route.waypoints[1].constraints.timeout = Time{0.05};  // far target, no time at all
        route.waypoints[1].markers[0] = cfgns::Marker{.id = "count_me"};
        route.waypoints[1].markerCount = 1;
        route.waypoints[3].markers[0] = cfgns::Marker{.id = "count_me"};
        route.waypoints[3].markerCount = 1;

        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);

        CHECK_FALSE(report.ok());
        CHECK(fired == 0);                                  // NOT at the wrong place
        CHECK(report.markersFired == 0);
        CHECK(report.reason == RunStopCause::MotionFailed);
        CHECK(report.stoppedAt == 1);
        CHECK(report.lastExit == ExitReason::TimedOut);
        CHECK(report.legsRun == 2);      // the Start and the leg that failed
        CHECK(report.legsSkipped == 3);  // waypoints 2, 3, 4 never attempted
        CHECK(driveIsSafed(r.rig.h));
        CHECK(warnCount(r.telemetry, "STOPPED") == 1);
        CHECK(lineWith(r.telemetry, LogLevel::Warn, "STOPPED").find("waypoint 1")
              != std::string::npos);
    }
}

// ═══ 6. An unknown id is data: one Warn, counted, and the route CONTINUES ════════════
// Bug caught: breaking the guarantee shulib made VexBuilder in writing (vexbuilder-integration.md
// §4.2) — "an unknown id logs a warning and is skipped; it never crashes". A typo in a picker
// must not end an autonomous period, and it must not be silent either: a route that quietly did
// not score is the hardest failure on this list to diagnose from the stands.
TEST_CASE("G2 runner 6: an unknown id warns once, is counted, and the route runs to the end") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});
    RunnerRig r{kin, plantAt(startPose())};
    int known = 0;
    auto knownFn = [&] { ++known; };
    CommandRegistry registry;
    registry.on("known", knownFn);

    cfgns::Route route = acceptanceRoute(12.0);
    route.waypoints[1].markers[0] = cfgns::Marker{.id = "nobody_registered_this"};
    route.waypoints[1].markers[1] = cfgns::Marker{.id = "known"};
    route.waypoints[1].markerCount = 2;
    route.waypoints[3].markers[0] = cfgns::Marker{.id = "also_unknown"};
    route.waypoints[3].markerCount = 1;

    PathRunner runner{r.chassis, registry, r.telemetry};
    const RunReport report = runner.run(route);

    CHECK(report.ok());                 // the route CONTINUED — this is the guarantee
    CHECK(report.legsRun == 5);
    CHECK(report.legsSkipped == 0);
    CHECK(report.unknownIds == 2);
    CHECK(report.markersFired == 1);    // only the one that existed
    CHECK(known == 1);
    CHECK(report.reason == RunStopCause::None);

    // Exactly one line per occurrence, each naming the id AND the waypoint — never silent.
    CHECK(warnCount(r.telemetry, "no handler") == 2);
    const std::string first = lineWith(r.telemetry, LogLevel::Warn, "nobody_registered_this");
    CHECK(first.find("waypoint 1") != std::string::npos);
    CHECK(lineWith(r.telemetry, LogLevel::Warn, "also_unknown").find("waypoint 3")
          != std::string::npos);
    // And no fault was latched: an id this robot was not given is strategy, not a pathology.
    CHECK_FALSE(r.rig.latch.hasFault());
}

// ═══ 7. A handler that fails stops the route and SAFES the drive ═════════════════════
// Bug caught: D1's error policy silently not applying through the runner — the route driving on
// after the intake reported it never got the ring, or stopping without parking the robot.
// `cancel()` is the safe state, not `brake()`: `brake()` is a motion with a watchdog, and a safe
// state that can time out is not one.
TEST_CASE("G2 runner 7: a failing handler stops the route, safes the drive, and counts the rest") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});

    SUBCASE("bool false") {
        RunnerRig r{kin, plantAt(startPose())};
        int later = 0;
        auto fails = [] { return false; };
        auto never = [&] { ++later; };
        CommandRegistry registry;
        registry.on("fails", fails);
        registry.on("never", never);

        cfgns::Route route = acceptanceRoute(12.0);
        route.waypoints[1].markers[0] = cfgns::Marker{.id = "fails"};
        route.waypoints[1].markers[1] = cfgns::Marker{.id = "never"};
        route.waypoints[1].markerCount = 2;

        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);

        CHECK_FALSE(report.ok());
        CHECK(report.reason == RunStopCause::MarkerFailed);
        CHECK(report.stoppedAt == 1);
        CHECK(report.legsRun == 2);
        CHECK(report.legsSkipped == 3);
        CHECK(report.markersFired == 1);  // the one that failed ran; its sibling never did
        CHECK(later == 0);
        CHECK(driveIsSafed(r.rig.h));
        CHECK(lineWith(r.telemetry, LogLevel::Warn, "STOPPED").find("fails")
              != std::string::npos);
        CHECK(lineWith(r.telemetry, LogLevel::Warn, "STOPPED").find("UNCONFIRMED")
              != std::string::npos);
    }

    SUBCASE("every MechanismOutcome other than Succeeded stops it, and names itself") {
        const MechanismOutcome bad[] = {MechanismOutcome::Running, MechanismOutcome::Unconfirmed,
                                        MechanismOutcome::TimedOut, MechanismOutcome::Cancelled,
                                        MechanismOutcome::Stalled};
        for (const MechanismOutcome o : bad) {
            CAPTURE(std::string{shulib::manipulation::mechanismOutcomeName(o)});
            RunnerRig r{kin, plantAt(startPose())};
            auto fn = [o] { return o; };
            CommandRegistry registry;
            registry.on("grab", fn);
            cfgns::Route route = acceptanceRoute(12.0);
            route.waypoints[1].markers[0] = cfgns::Marker{.id = "grab"};
            route.waypoints[1].markerCount = 1;

            PathRunner runner{r.chassis, registry, r.telemetry};
            const RunReport report = runner.run(route);
            CHECK(report.reason == RunStopCause::MarkerFailed);
            CHECK(report.stoppedAt == 1);
            CHECK(driveIsSafed(r.rig.h));
            CHECK(lineWith(r.telemetry, LogLevel::Warn, "STOPPED")
                      .find(shulib::manipulation::mechanismOutcomeName(o))
                  != std::string::npos);
        }
    }

    SUBCASE("a Succeeded handler does NOT stop it — the sibling really was reachable") {
        RunnerRig r{kin, plantAt(startPose())};
        int later = 0;
        auto okFn = [] { return MechanismOutcome::Succeeded; };
        auto after = [&] { ++later; };
        CommandRegistry registry;
        registry.on("fails", okFn);
        registry.on("never", after);
        cfgns::Route route = acceptanceRoute(12.0);
        route.waypoints[1].markers[0] = cfgns::Marker{.id = "fails"};
        route.waypoints[1].markers[1] = cfgns::Marker{.id = "never"};
        route.waypoints[1].markerCount = 2;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);
        CHECK(report.ok());
        CHECK(later == 1);
        CHECK(report.markersFired == 2);
    }
}

// ═══ 8. The four built-in ids, which no team registers ═══════════════════════════════
// Bug caught: a built-in that does not do what its name and its manifest entry say. These are
// the ids a non-coder reaches for FIRST (wait, hold, log), so they are the ones whose meaning
// must be exactly what the picker's label claims.
TEST_CASE("G2 runner 8: wait_ms, hold_ms, brake and log are performed by the runner itself") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});

    SUBCASE("wait_ms advances the clock by its argument, and does not energize the drive") {
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        cfgns::Route route{.name = "waiter"};
        route.waypoints[0] = {.x = Length{kStartX},
                              .y = Length{kStartY},
                              .heading = Angle::degrees(kStartHeadingDeg),
                              .motion = cfgns::WaypointMotion::Start,
                              .markers = {{.id = "wait_ms", .args = {250}, .argCount = 1}},
                              .markerCount = 1};
        route.count = 1;
        const double before = r.rig.h.clock().now().value();
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);
        const double elapsed = r.rig.h.clock().now().value() - before;

        CHECK(report.ok());
        CHECK(report.markersFired == 1);
        // The wait ends on the first tick at or past the deadline, so it is [250 ms, 250 ms + dt).
        CHECK(elapsed >= 0.250);
        CHECK(elapsed < 0.250 + 0.011);
        CHECK(driveIsSafed(r.rig.h) == false);  // wait() never energizes AND never brakes
        CHECK(r.rig.h.motor(0).commandedVoltage().value() == 0.0);
    }

    SUBCASE("hold_ms holds for its argument and reports Succeeded") {
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        cfgns::Route route{.name = "holder"};
        route.waypoints[0] = {.x = Length{kStartX},
                              .y = Length{kStartY},
                              .heading = Angle::degrees(kStartHeadingDeg),
                              .motion = cfgns::WaypointMotion::Start,
                              .markers = {{.id = "hold_ms", .args = {300}, .argCount = 1}},
                              .markerCount = 1};
        route.count = 1;
        const double before = r.rig.h.clock().now().value();
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);
        const double elapsed = r.rig.h.clock().now().value() - before;
        CHECK(report.ok());
        CHECK(elapsed >= 0.300);
        CHECK(elapsed < 0.400);
    }

    SUBCASE("brake safes the drive") {
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        cfgns::Route route{.name = "braker"};
        route.waypoints[0] = {.x = Length{kStartX},
                              .y = Length{kStartY},
                              .heading = Angle::degrees(kStartHeadingDeg),
                              .motion = cfgns::WaypointMotion::Start,
                              .markers = {{.id = "brake"}},
                              .markerCount = 1};
        route.count = 1;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);
        CHECK(report.ok());
        CHECK(driveIsSafed(r.rig.h));
    }

    SUBCASE("log writes one Info line naming the marker and its args") {
        RunnerRig r{kin, plantAt(startPose())};
        CommandRegistry registry;
        cfgns::Route route{.name = "talker"};
        route.waypoints[0] = {.x = Length{kStartX},
                              .y = Length{kStartY},
                              .heading = Angle::degrees(kStartHeadingDeg),
                              .motion = cfgns::WaypointMotion::Start,
                              .markers = {{.id = "log", .args = {7, -3}, .argCount = 2}},
                              .markerCount = 1};
        route.count = 1;
        PathRunner runner{r.chassis, registry, r.telemetry};
        const RunReport report = runner.run(route);
        CHECK(report.ok());
        const std::string line = lineWith(r.telemetry, LogLevel::Info, ": log");
        CHECK(line.find("route 'talker'") != std::string::npos);
        CHECK(line.find("waypoint 0") != std::string::npos);
        CHECK(line.find("7") != std::string::npos);
        CHECK(line.find("-3") != std::string::npos);
    }

    SUBCASE("a built-in cannot be shadowed by a registration, so a route's meaning is portable") {
        CommandRegistry registry;
        auto fn = [] {};
        CHECK_THROWS_AS(registry.on("brake", fn), shulib::PreconditionError);
    }
}

// ═══ 9. HeadingMode::Free on a MoveToPose = the estimate's heading at leg start (R6) ═
// Bug caught: "Free" quietly meaning zero. moveTo ALWAYS commands a heading — there is no
// don't-care to pass down — so the runner must supply one, and the only honest one is the
// heading the robot already has. Zero would spin the robot to the field's +X axis on a leg the
// author asked to translate only, and on a symmetric test rig starting at 0 that mistake is
// invisible. So the rig starts at 90°, and the pin is bit-identity against the Fixed spelling
// of the same heading.
TEST_CASE("G2 runner 9: Free on a MoveToPose commands the ESTIMATE's heading at leg start") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});
    const Pose2d start = startPose();  // heading 90°, deliberately not 0

    cfgns::Route freeRoute{.name = "free"};
    freeRoute.waypoints[0] = {.x = Length{-6.0},
                              .y = Length{6.0},
                              .heading = Angle::degrees(137.0),  // present, and NOT read
                              .headingMode = cfgns::HeadingMode::Free,
                              .motion = cfgns::WaypointMotion::MoveToPose,
                              .constraints = {.timeout = Time{8.0}}};
    freeRoute.count = 1;

    RunnerRig a{kin, plantAt(start)};
    CommandRegistry registry;
    const Angle estimateAtStart = a.chassis.pose().heading();
    PathRunner runner{a.chassis, registry, a.telemetry};
    const RunReport freeReport = runner.run(freeRoute);

    // The same waypoint, written Fixed at the heading the estimate had. Bit-identical.
    cfgns::Route fixedRoute = freeRoute;
    fixedRoute.waypoints[0].headingMode = cfgns::HeadingMode::Fixed;
    fixedRoute.waypoints[0].heading = estimateAtStart;
    RunnerRig b{kin, plantAt(start)};
    PathRunner keptRunner{b.chassis, registry, b.telemetry};
    const RunReport fixedReport = keptRunner.run(fixedRoute);

    requireIdentical(a.pacer.trace, b.pacer.trace);
    CHECK(freeReport.lastExit == fixedReport.lastExit);
    CHECK(a.pacer.trace.ticks > 50);

    // And it is NOT zero, and NOT the ignored 137° — the two mistakes this case exists for.
    const Angle truth = a.rig.h.truePose().heading();
    CHECK(std::abs(truth.errorTo(Angle::degrees(kStartHeadingDeg))) < 0.10);
    CHECK(std::abs(truth.errorTo(Angle::degrees(0.0))) > 1.0);
    CHECK(std::abs(truth.errorTo(Angle::degrees(137.0))) > 0.5);
}

// ═══ 10. reverse on a DriveTo arrives BACKWARDS onto the point ═══════════════════════
// Bug caught: `reverse` being carried in the data and read by nobody (route.hpp says it is
// carried "because a route that loses it on import has silently changed what a person drew" —
// so the runner has to actually mean it), or the π being dropped somewhere in the wrap.
TEST_CASE("G2 runner 10: reverse on a DriveTo turns the approach around; without it, forwards") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});
    const Pose2d origin{Length{0.0}, Length{0.0}, Angle::degrees(0.0)};

    cfgns::Route route{.name = "backwards"};
    route.waypoints[0] = {.x = Length{24.0},
                          .y = Length{0.0},
                          .headingMode = cfgns::HeadingMode::Free,
                          .motion = cfgns::WaypointMotion::DriveTo,
                          .reverse = true,
                          .constraints = {.timeout = Time{10.0}}};
    route.count = 1;

    RunnerRig rev{kin, plantAt(origin)};
    CommandRegistry registry;
    PathRunner revRunner{rev.chassis, registry, rev.telemetry};
    REQUIRE(revRunner.run(route).ok());
    const Pose2d revTruth = rev.rig.h.truePose();
    // Bearing to (24, 0) from the origin is 0; reversed, the robot faces π.
    CHECK(std::abs(revTruth.heading().errorTo(Angle::radians(Angle::kPi))) < 0.10);
    CHECK(motion_rig::posErr(revTruth, Pose2d{Length{24.0}, Length{0.0}, Angle{}}) < 2.0);

    cfgns::Route forward = route;
    forward.waypoints[0].reverse = false;
    RunnerRig fwd{kin, plantAt(origin)};
    PathRunner fwdRunner{fwd.chassis, registry, fwd.telemetry};
    REQUIRE(fwdRunner.run(forward).ok());
    const Pose2d fwdTruth = fwd.rig.h.truePose();
    CHECK(std::abs(fwdTruth.heading().errorTo(Angle::degrees(0.0))) < 0.10);

    // The two arrivals are a half turn apart — the whole content of the flag.
    CHECK(std::abs(std::abs(revTruth.heading().errorTo(fwdTruth.heading())) - Angle::kPi) < 0.20);
}

// ═══ 10b. The runner's preconditions throw THROUGH, exactly as the facade's do ═══════
// Bug caught: the error policy quietly swallowing a programming error. A nonsense route must be
// a loud throw at the door, not a polite report saying the route "stopped".
TEST_CASE("G2 runner 10b: an invalid route is refused at the door, naming the waypoint") {
    const shulib::kinematics::MatrixKinematics kin = xDrive(Length{7.0});
    RunnerRig r{kin, plantAt(startPose())};
    CommandRegistry registry;
    PathRunner runner{r.chassis, registry, r.telemetry};

    cfgns::Route bad = acceptanceRoute(12.0);
    bad.waypoints[2].motion = cfgns::WaypointMotion::Start;  // a Start in the middle
    CHECK_THROWS_AS((void)runner.run(bad), shulib::PreconditionError);
    CHECK(r.pacer.trace.ticks == 0);  // and nothing drove first

    try {
        (void)runner.run(bad);
    } catch (const shulib::PreconditionError& e) {
        const std::string what = e.what();
        CHECK(what.find("PathRunner::run") != std::string::npos);
        CHECK(what.find("waypoint 2") != std::string::npos);
    }
}
