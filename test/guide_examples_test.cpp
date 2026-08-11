// GUIDE EXAMPLES (chunk C8) — every code example in docs/guide/ compiles and
// runs HERE, so the guide can never quietly rot: an example that stops
// compiling or stops behaving as the prose claims turns this file red.
//
// Mapping (keep in sync with the guide — see docs/guide/README.md):
//   guide-08a/b/c  -> docs/guide/08-your-first-routine.md   (the tutorial)
//   guide-09a/b/c  -> docs/guide/09-the-recipe-api.md       (the Tier-2 chain)
//   guide-10a..e   -> docs/guide/10-the-api.md              (API idioms)
//
// The guide quotes these bodies VERBATIM. If you change code here, change the
// matching listing in the chapter (and vice versa) — that rule is the whole
// anti-rot mechanism, so treat a mismatch as a bug even when both sides work.
//
// Run just these, with the diagnostic transcript printed to your terminal:
//   SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-*'

#include "doctest.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/motion/run_reporter.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::TrajectoryResult;
using shulib::control::ExitReason;
using shulib::hal::fake::FakeCharSink;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::motion::WaitResult;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

namespace {

/// Occurrences of `needle` in `haystack` (for asserting on transcript text).
int countOf(const std::string& haystack, const std::string& needle) {
    int n = 0;
    for (std::size_t at = haystack.find(needle); at != std::string::npos;
         at = haystack.find(needle, at + needle.size())) {
        ++n;
    }
    return n;
}

/// Print a captured transcript when the reader asks for it (the guide's
/// SHULIB_GUIDE_PRINT=1 instruction). Quiet in CI.
void printIfRequested(const std::string& text) {
    if (std::getenv("SHULIB_GUIDE_PRINT") != nullptr) {
        std::fputs(text.c_str(), stdout);
    }
}

// ═══ Chapter 8, listing 1 — the pacer (quoted in "Step 5") ═════════════════════════

// While a motion verb blocks, SOMETHING has to advance the world. That seam is
// the pacer. In simulation it steps the physics forward one 10 ms tick; on the
// real robot it will simply wait for the next control tick (that adapter is
// R1's work).
struct SimPacer final : shulib::motion::ITickPacer {
    explicit SimPacer(shulib::sim::SimHarness& h) : harness{&h} {}
    void pace() override { harness->plant().step(0.01_s); }
    shulib::sim::SimHarness* harness;
};

// ═══ Chapter 8, listing 2 — the routine (quoted in "Step 7") ═══════════════════════

// The first routine: leave the start tile, make a slow precise approach, slide
// sideways along the goal wall, and turn to face the corner. Every call blocks
// until the motion settles or honestly gives up — the return value says which.
ExitReason firstRoutine(Chassis& chassis) {
    // Tell the localizer where the robot starts. Position comes from you;
    // heading is owned by the IMU (chapter 3).
    chassis.setPose(Pose2d{-48_in, -24_in, 90_deg});

    // Drive to a field position AND rotate to a heading, at the same time.
    ExitReason leg1 = chassis.moveTo(Pose2d{-24_in, 0_in, 45_deg},
                                     {.timeout = 5_s});

    // A slow, precise approach: this leg's speed is capped at 20 in/s.
    ExitReason leg2 = chassis.moveTo(Pose2d{-12_in, 12_in, 45_deg},
                                     {.timeout = 4_s,
                                      .maxLinearSpeed = Velocity{20.0}});

    // Slide sideways while actively holding the current heading.
    ExitReason leg3 = chassis.strafeTo(-12_in, 24_in, {.timeout = 3_s});

    // Face the corner — always the short way around.
    ExitReason leg4 = chassis.turnTo(135_deg, {.timeout = 2_s});

    // Real routines branch on these; here we just report the worst one.
    if (leg1 != ExitReason::Settled) { return leg1; }
    if (leg2 != ExitReason::Settled) { return leg2; }
    if (leg3 != ExitReason::Settled) { return leg3; }
    return leg4;
}

// ═══ Chapter 8, listing 4 — the tick-stream tap (quoted in "Step 9") ═══════════════

// The simulated hardware wants its diagnostics sink at construction time, but
// the terminal formatter needs the simulation's clock — which doesn't exist
// yet. This tiny forwarder breaks the circle: hand it to the harness now, point
// it at the real sink after. (On the robot this wart disappears: the clock is
// just the V5's clock, no simulation involved.)
struct RecordTap final : shulib::hal::ITelemetrySink {
    shulib::hal::ITelemetrySink* target = nullptr;
    void log(shulib::hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        if (target != nullptr) { target->log(level, subsystem, message); }
    }
    [[nodiscard]] bool wantsRecord() const noexcept override {
        return target != nullptr && target->wantsRecord();
    }
    void emit(const shulib::diag::DebugRecord& record) override {
        if (target != nullptr) { target->emit(record); }
    }
    void summarize(const shulib::diag::RunSummary& summary) override {
        if (target != nullptr) { target->summarize(summary); }
    }
};

}  // namespace

// ═══ guide-08a: the tutorial's core wiring + routine, exactly as the chapter ═══════
// shows it (chapter 8, listing 3 = this case body through finishRun()).

TEST_CASE("guide-08a: your first routine — wiring, running, and the report") {
    namespace k = shulib::kinematics;
    namespace loc = shulib::localization;

    // Step 1 — the drivetrain, described as data (an X-drive, wheels 7 in
    // from the center).
    const k::MatrixKinematics kin = k::xDrive(7_in);

    // Step 2 — the robot. On a real V5 this is where hardware adapters will
    // go (phase R1); today it is the simulated robot the whole library is
    // tested against. The feedforward constants describe the robot being
    // driven — they MUST match between plant and controller (chapter 5).
    shulib::sim::SimHarnessConfig simCfg;
    simCfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 90_deg};  // where it's placed
    shulib::sim::SimHarness robot{kin, simCfg};

    // Step 3 — localization: odometry (two tracking wheels + IMU) fused into
    // the one official pose estimate.
    loc::PilonsOdometry odom{robot.imu(), robot.makeForwardTrackingWheel(),
                             robot.makeLateralTrackingWheel()};
    loc::ComplementaryFusion fusion{};
    loc::Localizer localizer{robot.clock(), robot.imu(), odom, fusion};

    // Step 4 — diagnostics: a terminal formatter, the fault latch that
    // records what goes wrong, and the health monitor that watches for it.
    FakeCharSink capture;  // on the robot: the USB serial port; here: a string
    shulib::diag::TermSink term{robot.clock(), capture};
    shulib::diag::FaultLatch faults{term, robot.clock()};
    shulib::diag::HealthMonitor health{faults};

    // Step 5 — one dependencies bundle, one pacer, one Chassis.
    const shulib::motion::MotionDeps deps{.ctx = &robot.context(),
                                          .localizer = &localizer,
                                          .kinematics = &kin,
                                          .faults = &faults,
                                          .health = &health};
    SimPacer pacer{robot};
    shulib::chassis::ChassisConfig cfg;
    cfg.motion.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};  // matches step 2
    Chassis chassis{deps, pacer, cfg};

    // Step 6 — the run reporter: session header now, one result line per
    // motion as it ends, a summary when we say the run is over.
    shulib::motion::RunReporter report{term, chassis.scheduler()};
    report.sessionStart({.buildHash = shulib::diag::compiledBuildHash(),
                         .routineId = "first-auton",
                         .alliance = "red",
                         .side = "left",
                         .portMap = "sim"});

    // Step 7 — run the routine.
    const ExitReason outcome = firstRoutine(chassis);

    // Step 8 — end the run.
    report.finishRun();

    // ── What the chapter claims about this run, held as assertions ──────────────
    CHECK(outcome == ExitReason::Settled);

    // The robot is genuinely at the last target — graded on the simulator's
    // ground truth, which the estimator cannot see.
    const Pose2d goal{-12_in, 24_in, 135_deg};
    CHECK(motion_rig::posErr(robot.truePose(), goal) < 1.0);          // within 1 in
    CHECK(motion_rig::headErr(robot.truePose(), goal) < 0.035);       // within ~2°

    const std::string& text = capture.text();
    // The session header opens the transcript.
    CHECK(text.find("[SES] run start") != std::string::npos);
    // Four settled motions -> four ✓SETTLED result lines, in routine order.
    CHECK(countOf(text, "✓SETTLED") == 4);
    CHECK(countOf(text, "MoveToPose#1 ✓SETTLED") == 1);
    CHECK(countOf(text, "MoveToPose#2 ✓SETTLED") == 1);
    CHECK(countOf(text, "StrafeTo#3 ✓SETTLED") == 1);
    CHECK(countOf(text, "TurnTo#4 ✓SETTLED") == 1);
    // Without the tick stream connected, over/drift honestly read n/a —
    // the library reports "no data" rather than inventing a number.
    CHECK(text.find("over   n/a") != std::string::npos);
    // The summary closes the transcript, with a clean ledger.
    CHECK(text.find("RUN SUMMARY") != std::string::npos);
    CHECK(text.find("motions 4 · settled 4") != std::string::npos);
    CHECK(text.find("first fault none") != std::string::npos);
    CHECK_FALSE(faults.hasFault());

    printIfRequested(text);
}

// ═══ guide-08b: the same run with the 100 Hz tick stream connected ═════════════════
// (chapter 8, "Step 9 — turn on the full stream").

TEST_CASE("guide-08b: the tutorial with the per-tick stream connected") {
    namespace k = shulib::kinematics;
    namespace loc = shulib::localization;

    const k::MatrixKinematics kin = k::xDrive(7_in);

    RecordTap tap;  // handed to the harness now, aimed at the sink below
    shulib::sim::SimHarnessConfig simCfg;
    simCfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 90_deg};
    shulib::sim::SimHarness robot{kin, simCfg, &tap};

    loc::PilonsOdometry odom{robot.imu(), robot.makeForwardTrackingWheel(),
                             robot.makeLateralTrackingWheel()};
    loc::ComplementaryFusion fusion{};
    loc::Localizer localizer{robot.clock(), robot.imu(), odom, fusion};

    FakeCharSink capture;
    shulib::diag::TermSink term{robot.clock(), capture};
    tap.target = &term;  // close the loop: records now reach the terminal
    shulib::diag::FaultLatch faults{term, robot.clock()};
    shulib::diag::HealthMonitor health{faults};

    const shulib::motion::MotionDeps deps{.ctx = &robot.context(),
                                          .localizer = &localizer,
                                          .kinematics = &kin,
                                          .faults = &faults,
                                          .health = &health};
    SimPacer pacer{robot};
    shulib::chassis::ChassisConfig cfg;
    cfg.motion.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    Chassis chassis{deps, pacer, cfg};

    shulib::motion::RunReporter report{term, chassis.scheduler()};
    report.sessionStart({.buildHash = shulib::diag::compiledBuildHash(),
                         .routineId = "first-auton",
                         .alliance = "red",
                         .side = "left",
                         .portMap = "sim"});
    const ExitReason outcome = firstRoutine(chassis);
    report.finishRun();

    CHECK(outcome == ExitReason::Settled);
    const std::string& text = capture.text();
    // Now the transcript carries per-tick [MOT] lines between the landmarks…
    CHECK(countOf(text, "cmd#1▸") > 50);            // motion 1 ticked, visibly
    CHECK(text.find("tgt( -24.0,   0.0,  45.0°)") != std::string::npos);
    // …and the result lines carry real measurements instead of n/a.
    CHECK(countOf(text, "✓SETTLED") == 4);
    CHECK(text.find("over   n/a") == std::string::npos);

    printIfRequested(text);
}

// ═══ guide-08c: change something, and see what changes ═════════════════════════════
// (chapter 8, "Step 10": starve a leg's time budget and read the honest report.)

TEST_CASE("guide-08c: a starved timeout exits TimedOut — and the log says so") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    FakeCharSink capture;
    motion_rig::ChassisRig c{kin};  // the test suite's standard pre-wired stack
    shulib::diag::TermSink term{c.rig.h.clock(), capture};
    shulib::motion::RunReporter report{term, c.chassis.scheduler()};

    // 0.5 s is not enough to drive 30 inches and settle. The verb returns at
    // the budget (it does NOT hang), the motors stop, and the result line
    // reads ✗TIMEOUT.
    const ExitReason r = c.chassis.moveTo(Pose2d{30_in, 0_in, 0_deg},
                                          {.timeout = 0.5_s});
    CHECK(r == ExitReason::TimedOut);
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
    // The fault latch names it, too (MOTION_TIMEOUT is informational — it
    // never aborts anything; the motion already ended).
    CHECK(c.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 1);
    CHECK(capture.text().find("✗TIMEOUT") != std::string::npos);

    printIfRequested(capture.text());
}

// ═══ guide-09a: chapter 8's auton, one tier up — the recipe chain ══════════════════

// The same routine as chapter 8's firstRoutine, written as a recipe: each
// step runs (and blocks) the moment it is chained, so the routine reads in
// exactly the order the robot acts. If any step fails, the chain stops, parks
// the robot, and skips the rest — r.ok() tells you which world you are in.
RoutineResult firstRecipe(Chassis& chassis) {
    Routine r{chassis, "first-recipe"};
    r.startAt(Pose2d{-48_in, -24_in, 90_deg})
        .moveTo(Pose2d{-24_in, 0_in, 45_deg}, {.timeout = 5_s})
        .moveTo(Pose2d{-12_in, 12_in, 45_deg},
                {.timeout = 4_s, .maxLinearSpeed = Velocity{20.0}})
        .strafeTo(-12_in, 24_in, {.timeout = 3_s})
        .turnTo(135_deg, {.timeout = 2_s})
        .hold(300_ms)  // stand your ground for 0.3 s…
        .brake();    // …then park, braked
    return r.result();
}

TEST_CASE("guide-09a: the first recipe — chapter 8's routine in one readable chain") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 90_deg};
    motion_rig::ChassisRig c{kin, simCfg};  // the suite's standard pre-wired stack

    const RoutineResult res = firstRecipe(c.chassis);

    // What the chapter claims, held as assertions: the whole chain succeeded…
    CHECK(res.ok);
    CHECK(res.steps == 7);
    CHECK(res.completed == 7);
    CHECK(res.skipped == 0);
    // …six of the seven steps were motions (startAt only seeds the pose)…
    CHECK(c.chassis.scheduler().motionsStarted() == 6);
    // …and the robot is genuinely at the last target, on ground truth.
    const Pose2d goal{-12_in, 24_in, 135_deg};
    CHECK(motion_rig::posErr(c.rig.h.truePose(), goal) < 1.0);
    CHECK(motion_rig::headErr(c.rig.h.truePose(), goal) < 0.035);
}

// ═══ guide-09b: what a failed step looks like — the chain's error policy ═══════════

TEST_CASE("guide-09b: when a step fails, the chain stops — and says so") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    shulib::hal::fake::FakeTelemetrySink log;  // on the robot: the terminal
    motion_rig::ChassisRig c{kin, motion_rig::plantConfig(), &log};

    // 0.5 s is not enough to cross half the field, so step 2 times out. The
    // chain then STOPS: the drive is put in the safe state (0 V + brake) and
    // step 3 is skipped — a routine that kept driving from a position it is
    // not at would compound the miss blindly.
    Routine r{c.chassis, "starved"};
    r.moveTo(Pose2d{12_in, 0_in, 0_deg}, {.timeout = 5_s})
        .moveTo(Pose2d{60_in, 40_in, 0_deg}, {.timeout = 0.5_s})
        .turnTo(90_deg, {.timeout = 2_s});

    // The result says WHERE it stopped and WHY — a strategy branch, not a mystery.
    CHECK_FALSE(r.ok());
    const RoutineResult res = r.result();
    CHECK(res.stoppedAt == 2);                // which step failed…
    CHECK(res.exit == ExitReason::TimedOut);  // …and the motion's honest verdict
    CHECK(res.completed == 1);
    CHECK(res.skipped == 1);                  // the turn never ran

    // The transcript names it too (one Warn from the routine layer).
    bool sawStop = false;
    for (int i = 0; i < log.size(); ++i) {
        if (log.at(i).message.find("'starved' STOPPED at step 2 (moveTo)")
            != std::string::npos) {
            sawStop = true;
        }
    }
    CHECK(sawStop);
}

// ═══ guide-09c: tank recipes and the no-cliff rule ═════════════════════════════════

TEST_CASE("guide-09c: tank recipes — face the point, drive to it; the full API stays "
          "one line away") {
    const shulib::kinematics::TankKinematics kin{12_in};
    motion_rig::ChassisRig c{kin};

    // A tank drive cannot slide sideways, and shulib never pretends it can
    // (chapter 4). In a recipe YOU still write the turn — in field words:
    // face the point, then drive to it.
    Routine r{c.chassis, "tank-recipe"};
    r.face(0_in, 24_in, {.timeout = 3_s})
        .driveTo(0_in, 24_in, {.timeout = 8_s});
    CHECK(r.ok());
    CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{0_in, 24_in, 90_deg}) < 1.0);

    // No cliff between tiers: the full API is the same chassis, mid-routine.
    // Here the direct turnTo IS this leg's "face", done one tier down…
    REQUIRE(c.chassis.turnTo(0_deg, {.timeout = 3_s}) == ExitReason::Settled);
    // …and the same chain object carries on afterwards, unconfused.
    r.driveTo(24_in, 24_in, {.timeout = 8_s}).brake();
    CHECK(r.ok());
    CHECK(motion_rig::posErr(c.rig.h.truePose(),
                             Pose2d{24_in, 24_in, c.rig.h.truePose().heading()})
          < 1.0);
}

// ═══ guide-10a: per-call options override the config for ONE motion ════════════════

TEST_CASE("guide-10a: options — a slow precise approach leg") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    // Config defaults drive this leg…
    CHECK(c.chassis.moveTo(Pose2d{20_in, 0_in, 0_deg}, {.timeout = 8_s})
          == ExitReason::Settled);
    // …and per-call options slow just this one down (0 = keep the default).
    CHECK(c.chassis.moveTo(Pose2d{28_in, 6_in, 0_deg},
                           {.timeout = 8_s,
                            .maxLinearSpeed = Velocity{15.0},
                            .maxAngularSpeed = AngularVelocity{2.0}})
          == ExitReason::Settled);
    // The chassis-wide config is untouched afterwards.
    CHECK(c.chassis.motionConfig().maxLinearSpeed.value() == 60.0);
}

// ═══ guide-10b: tank honesty + the turn-then-drive idiom ═══════════════════════════

// On a tank drive, a sideways target is physically unreachable — the library
// says TimedOut rather than pretending. The idiom: turn to the bearing of the
// target, then drive to it WITH that bearing as the target heading, so the
// approach is a straight line the drivetrain can actually follow.
ExitReason tankGoTo(Chassis& chassis, shulib::units::Length x, shulib::units::Length y) {
    const Pose2d here = chassis.pose();
    const Angle bearing = Angle::radians(
        std::atan2((y - here.y()).value(), (x - here.x()).value()));
    const ExitReason turn = chassis.turnTo(bearing, {.timeout = 3_s});
    if (turn != ExitReason::Settled) { return turn; }
    return chassis.moveTo(Pose2d{x, y, bearing}, {.timeout = 8_s});
}

TEST_CASE("guide-10b: tank cannot strafe — TimedOut, honestly; turn-then-drive works") {
    const shulib::kinematics::TankKinematics kin{12_in};
    motion_rig::ChassisRig c{kin};

    // Straight sideways: impossible on tank. Honest TimedOut at OUR budget.
    CHECK(c.chassis.strafeTo(0_in, 24_in, {.timeout = 1_s})
          == ExitReason::TimedOut);

    // The idiom reaches the same point.
    CHECK(tankGoTo(c.chassis, 0_in, 24_in) == ExitReason::Settled);
    CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{0_in, 24_in, 90_deg}) < 1.0);
}

// ═══ guide-10c: followTrajectory tells you WHERE a chain broke ═════════════════════

TEST_CASE("guide-10c: followTrajectory — success counts legs; failure names the leg") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    const TrajectoryResult ok = c.chassis.followTrajectory(
        {Pose2d{12_in, 0_in, 0_deg}, Pose2d{24_in, 12_in, 45_deg},
         Pose2d{24_in, 24_in, 90_deg}},
        {.timeout = 8_s});
    CHECK(ok.succeeded());
    CHECK(ok.completedLegs == 3);

    // Starve the per-LEG budget: the chain stops at the first failed leg and
    // reports how far it got, instead of chasing later waypoints blind.
    const TrajectoryResult broke = c.chassis.followTrajectory(
        {Pose2d{36_in, 24_in, 0_deg}, Pose2d{-48_in, -24_in, 0_deg}},
        {.timeout = 0.6_s});
    CHECK_FALSE(broke.succeeded());
    CHECK(broke.exit == ExitReason::TimedOut);
    CHECK(broke.completedLegs < broke.totalLegs);
}

// ═══ guide-10d: drive() — the manual verb; the frame is always explicit ════════════

TEST_CASE("guide-10d: drive() moves the robot the way the FRAME says") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    // A teleop-shaped loop: command, then let the world advance one tick.
    // Frame::Body: "+x" means the robot's OWN forward, wherever it faces.
    for (int i = 0; i < 100; ++i) {
        c.chassis.drive(ChassisSpeeds{Velocity{20.0}, Velocity{0.0},
                                      AngularVelocity{0.0}},
                        Frame::Body);
        c.pacer.pace();
    }
    // Facing 0°, body-forward IS field +x: the robot moved right, not up.
    CHECK(c.rig.h.truePose().x().value() > 10.0);
    CHECK(std::abs(c.rig.h.truePose().y().value()) < 2.0);

    // There is no default frame — `drive(speeds)` does not compile. That is
    // deliberate: a silently-assumed frame is the classic field bug.
    c.chassis.cancel();  // done driving: the panic stop safes the drivetrain
}

// ═══ guide-10e: waitUntil is a bounded wait, and timing out is not a fault ═════════

TEST_CASE("guide-10e: waitUntil — an honest, bounded strategy branch") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    // Wait up to 0.5 s for a condition that never comes true (say, a game
    // piece a sensor never sees). The result is a value you must look at —
    // and no fault is raised: a timed-out wait is a strategy branch, not an
    // emergency.
    const WaitResult seen = c.chassis.waitUntil([] { return false; }, 0.5_s);
    CHECK(seen == WaitResult::TimedOut);
    CHECK_FALSE(c.rig.latch.hasFault());
}
