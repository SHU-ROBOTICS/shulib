// C5 END-TO-END — the run is legible in real time on the terminal (M2's clause).
// What this file targets:
//  * THE TRANSCRIPT: header → tick records → result line → summary through ONE
//    TermSink, byte-exact — the §18.3 output as one artifact, not four fragments.
//  * NUMBERS ARE TRUE: the result line's fields cross-checked against GROUND TRUTH
//    across swept routines on all three drivetrains (the C1 settled-vs-truth
//    discipline applied to the REPORT) — a line claiming 0.1 in while the robot is
//    6 in off is worse than no line.
//  * SUMMARY CORRECTNESS under injected faults: first fault really is FIRST, worst
//    loop dt really is the WORST, drops really are the drops.
//  * COST: the NullSink chain never builds a record, attribution-off never exists.
//  * HOSTILITY: a full A3-composed hostile run still produces a coherent report.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/motion_result.hpp"
#include "shulib/diag/rate_limit_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/run_reporter.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::PreconditionError;
using shulib::chassis::Chassis;
using shulib::control::ExitReason;
using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::MotionResult;
using shulib::diag::RateLimitConfig;
using shulib::diag::RateLimitedSink;
using shulib::diag::RunSummary;
using shulib::diag::SessionInfo;
using shulib::diag::TermSink;
using shulib::diag::TickPhase;
using shulib::hal::ITelemetrySink;
using shulib::hal::LogLevel;
using shulib::hal::emitRecord;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::CompletedMotion;
using shulib::motion::IMotionObserver;
using shulib::motion::MotionScheduler;
using shulib::motion::MotionSchedulerConfig;
using shulib::motion::MotionState;
using shulib::motion::MotionStatsSink;
using shulib::motion::MoveToPose;
using shulib::motion::RunReporter;
using shulib::motion::WaitResult;
using shulib::sim::FullHostility;
using shulib::sim::Rng;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
namespace units = shulib::units;

namespace {
constexpr double kDegToRad = Angle::kPi / 180.0;

/// Routes the harness's construction-time sink pointer to a target chosen AFTER
/// construction — SimHarness owns the clock TermSink needs, so the chain has to
/// be closed late. Drops quietly until a target is set.
class ForwardingSink final : public ITelemetrySink {
public:
    ITelemetrySink* target = nullptr;
    void log(LogLevel level, std::string_view subsystem, std::string_view message) override {
        if (target != nullptr) {
            target->log(level, subsystem, message);
        }
    }
    [[nodiscard]] bool wantsRecord() const noexcept override {
        return target != nullptr && target->wantsRecord();
    }
    void emit(const DebugRecord& record) override {
        if (target != nullptr) {
            target->emit(record);
        }
    }
    void summarize(const RunSummary& summary) override {
        if (target != nullptr) {
            target->summarize(summary);
        }
    }
};

/// PlantPacer's shape + a truth trace: truth pose after every pace, so a test can
/// compute GROUND-TRUTH overshoot with the same formula the stats sink applies to
/// the estimate — the reported-vs-true cross-check's raw material.
class TruthProbePacer final : public shulib::motion::ITickPacer {
public:
    explicit TruthProbePacer(shulib::sim::SimHarness& harness, Time dt = Time{0.01})
        : h{&harness}, tickDt{dt} {}
    void pace() override {
        if (++paces > 200000) {
            throw std::runtime_error("TruthProbePacer: pace cap exceeded");
        }
        h->plant().step(tickDt);
        trace.push_back(h->truePose());
    }
    shulib::sim::SimHarness* h;
    Time tickDt;
    int paces = 0;
    std::vector<Pose2d> trace;
};

/// The stats sink's overshoot definition, applied to a TRUTH window: max
/// projection past `target` along start→target (or worst wander for a
/// stationary target).
double truthOvershoot(const std::vector<Pose2d>& trace, std::size_t from,
                      const Pose2d& start, const Pose2d& target) {
    const double axx = (target.x() - start.x()).value();
    const double axy = (target.y() - start.y()).value();
    const double alen = std::hypot(axx, axy);
    double maxProj = 0.0;
    double maxDist = 0.0;
    for (std::size_t i = from; i < trace.size(); ++i) {
        const double dx = (trace[i].x() - target.x()).value();
        const double dy = (trace[i].y() - target.y()).value();
        maxDist = std::max(maxDist, std::hypot(dx, dy));
        if (alen >= 0.1) {
            maxProj = std::max(maxProj, (dx * axx + dy * axy) / alen);
        }
    }
    return alen >= 0.1 ? std::max(0.0, maxProj) : maxDist;
}

int countOf(const std::string& text, const std::string& needle) {
    int n = 0;
    for (std::size_t at = text.find(needle); at != std::string::npos;
         at = text.find(needle, at + needle.size())) {
        ++n;
    }
    return n;
}
}  // namespace

// ═══ The synthetic transcript — the M2-closure golden ══════════════════════════════

// Bug caught: the four §18.3 surfaces (header, tick stream, result line, summary)
// drifting apart in shape or losing co-residence on one sink — each is pinned
// alone elsewhere; THIS pins the whole terminal artifact, byte for byte.
TEST_CASE("C5 transcript: header → ticks → result line → summary through one "
          "TermSink, byte-exact (§18.3 end to end)") {
    FakeClock clock;
    FakeCharSink out;
    TermSink term{clock, out};

    shulib::diag::emitSessionHeader(
        term,
        SessionInfo{.buildHash = "a1b2c3d", .routineId = "redLeftTall",
                    .alliance = "red", .side = "left", .portMap = "L1,2 R3,4"},
        Voltage{12.4});

    DebugRecord running;
    running.t = units::Time{12.34};
    running.activeCommandId = 7;
    running.activeCommandState = 2;
    running.targetPose = Pose2d{Length{24.0}, Length{36.0}, Angle::degrees(90.0)};
    running.errorX = units::Length{0.4};
    running.errorY = units::Length{0.2};
    running.errorHeading = units::AngleDim{0.3 * kDegToRad};
    running.commanded = shulib::math::ChassisSpeeds{units::Velocity{18.0},
                                                    units::Velocity{4.0},
                                                    units::AngularVelocity{0.1}};
    running.quality = 0.91;
    term.emit(running);

    DebugRecord settled = running;
    settled.t = units::Time{12.50};
    settled.activeCommandState = 3;
    settled.errorX = units::Length{0.1};
    settled.errorY = units::Length{0.0};
    settled.errorHeading = units::AngleDim{0.1 * kDegToRad};
    settled.commanded = shulib::math::ChassisSpeeds{};
    settled.quality = 0.95;
    term.emit(settled);

    clock.set(units::Time{12.50});
    MotionResult r;
    r.id = 7;
    r.name = "MoveToPose";
    r.outcome = shulib::diag::MotionOutcome::Settled;
    r.duration = units::Time{1.16};
    r.hasPathData = true;
    r.finalPose = Pose2d{Length{24.1}, Length{36.0}, Angle::degrees(90.1)};
    r.overshoot = units::Length{0.2};
    r.drift = units::AngleDim{0.1 * kDegToRad};
    shulib::diag::emitResultLine(term, r);

    RunSummary s;
    s.motionsStarted = 1;
    s.motionsSettled = 1;
    s.hasHeadingData = true;
    s.headingMax = units::AngleDim{0.1 * kDegToRad};
    s.headingFinal = units::AngleDim{0.1 * kDegToRad};
    s.worstLoopDt = units::Time{0.010};
    s.setBuildHash("a1b2c3d");
    s.setRoutineId("redLeftTall");
    s.batteryStart = Voltage{12.4};
    s.batteryEnd = Voltage{12.2};
    term.summarize(s);

    CHECK(out.text() ==
          "[t=   0.00] [SES] run start · build a1b2c3d · routine \"redLeftTall\"\n"
          "[t=   0.00] [SES] alliance red · side left · batt 12.40V\n"
          "[t=   0.00] [SES] ports L1,2 R3,4\n"
          "[t=  12.34] [MOT] cmd#7▸2 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.40\",  0.20\",  0.3°) v(  18.0,   4.0, 0.10) q=0.91\n"
          "[t=  12.50] [MOT] cmd#7▸3 tgt(  24.0,  36.0,  90.0°) "
          "err(  0.10\",  0.00\",  0.1°) v(   0.0,   0.0, 0.00) q=0.95\n"
          "[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) "
          "over  0.20\" drift  0.1°   1.16s\n"
          "── RUN SUMMARY ───────────────────────────────────────────\n"
          " motions 1 · settled 1 · timeout 0 · cancelled 0 · aborted 0\n"
          " heading max  0.1° final  0.1° · gating rejects 0 · brownout no\n"
          " worst loop dt   10.0ms · first fault none · dropped 0 rec 0 ln\n"
          " build a1b2c3d · routine \"redLeftTall\" · batt 12.4→12.2V\n"
          "──────────────────────────────────────────────────────────\n");
}

// ═══ The stats formula, pinned by hand-fed streams ═════════════════════════════════

// Bug caught: the overshoot/drift derivation wrong at the source — the rig-level
// cross-checks compare small-vs-small, so THIS is where the formula itself is
// pinned against hand-computable numbers (including the 0.5 in overshoot a real
// clean run never produces).
TEST_CASE("C5 stats: hand-fed record stream — overshoot is the excursion PAST the "
          "target; waiting/idle/boot records never count") {
    FakeTelemetrySink inner;
    MotionStatsSink stats{inner};
    auto rec = [](std::uint32_t id, MotionState st, double x, double y, double tx,
                  double ty, double errH) {
        DebugRecord r;
        r.activeCommandId = id;
        r.activeCommandState = static_cast<std::uint8_t>(st);
        r.measuredPose = Pose2d{Length{x}, Length{y}, Angle{}};
        r.targetPose = Pose2d{Length{tx}, Length{ty}, Angle{}};
        r.errorHeading = units::AngleDim{errH};
        return r;
    };

    stats.beginMotion();
    stats.emit(rec(1, MotionState::WaitingForEstimate, 500.0, 0.0, 0.0, 0.0, 9.9));
    CHECK_FALSE(stats.hasData());  // waiting records: deliberately-zero fabrications
    stats.emit(rec(0, MotionState::Running, 400.0, 0.0, 10.0, 0.0, 9.9));
    CHECK_FALSE(stats.hasData());  // id 0: an idle/teleop record, not this motion
    stats.emit(rec(1, MotionState::Running, 0.0, 0.0, 10.0, 0.0, 0.05));   // start
    stats.emit(rec(1, MotionState::Running, 6.0, 0.0, 10.0, 0.0, 0.03));   // en route
    stats.emit(rec(1, MotionState::Running, 10.5, 0.0, 10.0, 0.0, 0.02));  // past it!
    stats.emit(rec(1, MotionState::Settled, 10.1, 0.0, 10.0, 0.0, 0.01));  // exit
    REQUIRE(stats.hasData());
    CHECK(stats.overshoot().value() == doctest::Approx(0.5));  // 10.5 − 10.0
    CHECK(stats.drift().value() == doctest::Approx(0.01));     // the EXIT record's error
    CHECK(inner.recordCount() == 6);                           // pure observer: all pass

    // Stationary target (a turn): overshoot degrades to worst wander from the point.
    stats.beginMotion();
    CHECK_FALSE(stats.hasData());  // beginMotion really resets
    stats.emit(rec(2, MotionState::Running, 5.0, 5.0, 5.0, 5.0, 1.5));
    stats.emit(rec(2, MotionState::Running, 5.3, 5.0, 5.0, 5.0, 0.7));
    stats.emit(rec(2, MotionState::Settled, 5.1, 5.0, 5.0, 5.0, 0.02));
    CHECK(stats.overshoot().value() == doctest::Approx(0.3));
    CHECK(stats.drift().value() == doctest::Approx(0.02));

    // A boot-window cancel (exit with NO prior Running tick): no data, no lies.
    stats.beginMotion();
    stats.emit(rec(3, MotionState::Cancelled, 0.0, 0.0, 0.0, 0.0, 0.0));
    CHECK_FALSE(stats.hasData());
}

// ═══ Numbers are TRUE — the ground-truth cross-checks (constraint 3) ═══════════════

// Bug caught: a result line that LIES — reported final pose / drift / overshoot
// diverging from what the robot physically did. Swept over a multi-leg routine on
// X and H; every motion's reported numbers graded against plant truth (the C1
// settled-vs-truth method applied to the REPORT).
TEST_CASE("C5 truth: reported result-line numbers track ground truth — X and H "
          "drives, swept routine") {
    struct Bounds {
        double pos, headRad, over;
    };
    // Observed on the clean plant: ~1e-13 / 0 / 0 (the estimate coincides with
    // truth when sensors are perfect). Pinned near-machine-tight: on a clean
    // plant the report must EQUAL the physical truth — any looseness here would
    // be the reporting path itself.
    const Bounds clean{1e-9, 1e-9, 1e-9};
    const auto xkin = xDrive(Length{7.0});
    const auto hkin = hBotKinematics();
    const shulib::kinematics::IKinematics* kins[] = {&xkin, &hkin};
    const double timeouts[] = {8.0, 14.0};

    for (int d = 0; d < 2; ++d) {
        CAPTURE(d);
        auto pcfg = plantConfig();
        pcfg.plant.seed = 4242;
        FakeTelemetrySink sink;  // records must flow: the stats need the stream
        MotionRig rig{*kins[d], pcfg, &sink};
        TruthProbePacer pacer{rig.h};
        Chassis chassis{rig.deps, pacer, chassisConfig()};

        Rng wp{991};
        Pose2d target{};
        double worstPos = 0.0;
        double worstHead = 0.0;
        double worstOver = 0.0;
        for (int k = 0; k < 6; ++k) {
            target = Pose2d{Length{wp.uniform(-40.0, 40.0)}, Length{wp.uniform(-40.0, 40.0)},
                            Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi))};
            const std::size_t mark = pacer.trace.size();
            const Pose2d truthStart = rig.h.truePose();
            REQUIRE(chassis.moveTo(target, {.timeoutSeconds = timeouts[d]})
                    == ExitReason::Settled);
            const CompletedMotion& c = chassis.lastCompleted();
            REQUIRE(c.hasPathData);
            const Pose2d truth = rig.h.truePose();
            // Reported final pose vs physical truth:
            worstPos = std::max(worstPos, posErr(c.finalPose, truth));
            // Reported drift vs the TRUE final heading error:
            const double trueHeadErr =
                std::abs(truth.heading().errorTo(c.targetPose.heading()));
            worstHead = std::max(worstHead, std::abs(trueHeadErr - c.drift.value()));
            // Reported overshoot vs truth-trajectory overshoot:
            const double trueOver =
                truthOvershoot(pacer.trace, mark, truthStart, c.targetPose);
            worstOver = std::max(worstOver, std::abs(c.overshoot.value() - trueOver));
        }
        MESSAGE("drivetrain " << d << ": worst |reported-true| pos=" << worstPos
                              << "in head=" << worstHead << "rad over=" << worstOver
                              << "in");
        CHECK(worstPos < clean.pos);
        CHECK(worstHead < clean.headRad);
        CHECK(worstOver < clean.over);
    }
}

// Bug caught: the overshoot report VACUOUSLY true — the memoryless (kA = 0)
// accuracy plant cannot physically overshoot, so the swept case above compares
// 0 with 0. This case gives the plant INERTIA (kA = 0.05 ⇒ wheel time-constant
// ~0.3 s ⇒ an underdamped approach) while the motion's feedforward knows only
// kS/kV — the robot genuinely sails past the target, and the REPORTED overshoot
// must both be nonzero and match the truth-trajectory overshoot. This is the
// non-vacuity twin the hand-fed formula test cannot provide alone.
TEST_CASE("C5 truth: with an inertial plant the robot REALLY overshoots — and the "
          "reported overshoot matches the truth trajectory") {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.wheelFf.kA = 0.05;  // inertia: the plant lags its commands
    FakeTelemetrySink sink;
    MotionRig rig{kin, pcfg, &sink};
    TruthProbePacer pacer{rig.h};
    Chassis chassis{rig.deps, pacer, chassisConfig()};

    const Pose2d target{Length{30.0}, Length{0.0}, Angle{}};
    const Pose2d truthStart = rig.h.truePose();
    REQUIRE(chassis.moveTo(target, {.timeoutSeconds = 10.0}) == ExitReason::Settled);
    const CompletedMotion& c = chassis.lastCompleted();
    REQUIRE(c.hasPathData);
    const double trueOver = truthOvershoot(pacer.trace, 0, truthStart, c.targetPose);
    MESSAGE("inertial overshoot: reported " << c.overshoot.value() << "in, true "
                                            << trueOver << "in");
    CHECK(c.overshoot.value() > 0.05);  // the comparison is NOT 0-vs-0
    CHECK(std::abs(c.overshoot.value() - trueOver) < 0.05);
    CHECK(posErr(c.finalPose, rig.h.truePose()) < 1e-9);  // clean sensors: exact
}

// Bug caught: the report lying on TANK, whose author-planned turn+drive idiom
// exercises TurnTo's stationary-target path (the overshoot hold-branch) — a
// drivetrain-shaped hole in the truth net.
TEST_CASE("C5 truth: reported numbers track ground truth on tank (turn-then-drive "
          "idiom)") {
    const TankKinematics kin{Length{12.0}};
    auto pcfg = plantConfig();
    pcfg.plant.seed = 7;
    FakeTelemetrySink sink;
    MotionRig rig{kin, pcfg, &sink};
    TruthProbePacer pacer{rig.h};
    Chassis chassis{rig.deps, pacer, chassisConfig()};

    Rng wp{55};
    double worstPos = 0.0;
    double worstHead = 0.0;
    for (int k = 0; k < 4; ++k) {
        const Pose2d here = chassis.pose();
        const double tx = wp.uniform(-30.0, 30.0);
        const double ty = wp.uniform(-30.0, 30.0);
        const Angle bearing = Angle::radians(
            std::atan2(ty - here.y().value(), tx - here.x().value()));
        REQUIRE(chassis.turnTo(bearing, {.timeoutSeconds = 8.0}) == ExitReason::Settled);
        {
            const CompletedMotion& turn = chassis.lastCompleted();
            REQUIRE(turn.hasPathData);
            const double trueHeadErr =
                std::abs(rig.h.truePose().heading().errorTo(bearing));
            worstHead = std::max(worstHead, std::abs(trueHeadErr - turn.drift.value()));
        }
        REQUIRE(chassis.moveTo(Pose2d{Length{tx}, Length{ty}, bearing},
                               {.timeoutSeconds = 10.0})
                == ExitReason::Settled);
        const CompletedMotion& c = chassis.lastCompleted();
        REQUIRE(c.hasPathData);
        worstPos = std::max(worstPos, posErr(c.finalPose, rig.h.truePose()));
    }
    MESSAGE("tank: worst |reported-true| pos=" << worstPos << "in head=" << worstHead
                                               << "rad");
    CHECK(worstPos < 0.75);
    CHECK(worstHead < 0.02);
}

// ═══ The real rig, end to end ══════════════════════════════════════════════════════

// Bug caught: the assembled pipeline losing a piece in vivo — the header not
// first, a boundary without a result line, result lines out of order with the
// tick stream (the observer must fire AFTER the exit record), or the summary
// missing/duplicated/not-last.
TEST_CASE("C5 e2e: a real two-motion run renders header, ordered result lines, and "
          "one final summary") {
    const auto kin = xDrive(Length{7.0});
    ForwardingSink fwd;
    ChassisRig cr{kin, plantConfig(), &fwd};
    FakeCharSink out;
    TermSink term{cr.rig.h.clock(), out};
    fwd.target = &term;
    RunReporter reporter{term, cr.chassis.scheduler()};

    reporter.sessionStart(SessionInfo{.buildHash = shulib::diag::compiledBuildHash(),
                                      .routineId = "e2e", .alliance = "red",
                                      .side = "left", .portMap = "sim"});
    REQUIRE(cr.chassis.moveTo(Pose2d{Length{24.0}, Length{18.0}, Angle::degrees(45.0)},
                              {.timeoutSeconds = 8.0})
            == ExitReason::Settled);
    REQUIRE(cr.chassis.turnTo(Angle::degrees(-90.0), {.timeoutSeconds = 8.0})
            == ExitReason::Settled);
    reporter.finishRun();

    const std::string& text = out.text();
    // Header first — provenance opens every run (§18.5).
    CHECK(text.find("[SES] run start · build ") == text.find("[SES]"));
    CHECK(text.rfind("[SES] run start", 20) != std::string::npos);
    // One ✓SETTLED result line per settled motion, none spurious.
    CHECK(countOf(text, "✓SETTLED") == 2);
    CHECK(countOf(text, "MoveToPose#1 ✓SETTLED") == 1);
    CHECK(countOf(text, "TurnTo#2 ✓SETTLED") == 1);
    // Ordering: motion 1's result line sits AFTER its exit record and BEFORE any
    // motion-2 tick (the boundary fires where the motion ended).
    const std::size_t exit1 = text.find("cmd#1▸3");
    const std::size_t line1 = text.find("MoveToPose#1 ✓SETTLED");
    const std::size_t first2 = text.find("cmd#2▸");
    REQUIRE(exit1 != std::string::npos);
    REQUIRE(line1 != std::string::npos);
    REQUIRE(first2 != std::string::npos);
    CHECK(exit1 < line1);
    CHECK(line1 < first2);
    // Exactly one summary, at the end, carrying the true ledger.
    CHECK(countOf(text, "── RUN SUMMARY ") == 1);
    CHECK(text.find(" motions 2 · settled 2 · timeout 0 · cancelled 0 · aborted 0\n")
          != std::string::npos);
    const std::size_t block = text.find("── RUN SUMMARY ");
    CHECK(block > line1);
    // The transcript ENDS with the block's bottom rule: the summary is last.
    const std::string bottomRule =
        "──────────────────────────────────────────────────────────\n";
    REQUIRE(text.size() > bottomRule.size());
    CHECK(text.substr(text.size() - bottomRule.size()) == bottomRule);
    // The real build hash rode through (this build has one — pinned elsewhere).
    CHECK(text.find(std::string{"build "}
                    + std::string{shulib::diag::compiledBuildHash()})
          != std::string::npos);
}

// Bug caught: boundary vocabulary wrong in vivo — a pre-empt reading as a user
// cancel (blaming the author for last-command-wins) or a fault abort losing its
// causal code on the line.
TEST_CASE("C5 e2e: SUPERSEDED and FAULT_ABORT reach the terminal with their causes") {
    const auto kin = xDrive(Length{7.0});
    ForwardingSink fwd;
    ChassisRig cr{kin, plantConfig(), &fwd};
    FakeCharSink out;
    TermSink term{cr.rig.h.clock(), out};
    fwd.target = &term;
    RunReporter reporter{term, cr.chassis.scheduler()};
    MotionScheduler& sched = cr.chassis.scheduler();

    // Pre-empt: m1 superseded by m2 (Tier-3 async, the C2 semantics).
    MoveToPose m1{sched.deps(), Pose2d{Length{30.0}, Length{0.0}, Angle{}},
                  motionConfig(), 8.0};
    MoveToPose m2{sched.deps(), Pose2d{Length{0.0}, Length{30.0}, Angle{}},
                  motionConfig(), 8.0};
    sched.async(m1);
    (void)sched.waitUntil([&] { return false; }, 0.3);  // let m1 genuinely run
    sched.async(m2);  // supersedes
    CHECK(sched.lastCompleted().preempted);
    CHECK(out.text().find("MoveToPose#1 ✗SUPERSEDED") != std::string::npos);
    REQUIRE(sched.waitUntilSettled() == ExitReason::Settled);

    // Fault abort: IMU_LOST added to the mask, raised mid-motion → the line
    // carries the causal code.
    MotionSchedulerConfig abortCfg;
    abortCfg.abortFaultMask = shulib::motion::faultBit(FaultCode::OdoStuck)
                              | shulib::motion::faultBit(FaultCode::ImuLost);
    ForwardingSink fwd2;
    SchedulerRig s2{kin, plantConfig(), &fwd2, nullptr, abortCfg};
    FakeCharSink out2;
    TermSink term2{s2.rig.h.clock(), out2};
    fwd2.target = &term2;
    RunReporter reporter2{term2, s2.sched};
    MoveToPose m3{s2.sched.deps(), Pose2d{Length{30.0}, Length{0.0}, Angle{}},
                  motionConfig(), 8.0};
    s2.sched.async(m3);
    const double t0 = s2.rig.h.clock().now().value();
    (void)s2.sched.waitUntil(
        [&] {
            if (s2.rig.h.clock().now().value() > t0 + 0.2) {
                s2.rig.latch.raise(FaultCode::ImuLost, "IMU", "injected");
            }
            return !s2.sched.hasActiveMotion();
        },
        5.0);
    CHECK(s2.sched.lastCompleted().abortFault == FaultCode::ImuLost);
    CHECK_FALSE(s2.sched.lastCompleted().preempted);
    CHECK(out2.text().find("MoveToPose#1 ✗FAULT_ABORT=IMU_LOST") != std::string::npos);
}

// Bug caught: summary quantities that don't survive contact with a messy run —
// a later fault usurping FIRST, a smaller gap overwriting the WORST dt, the
// brownout latch not reaching the block, gating episodes uncounted.
TEST_CASE("C5 e2e: summary correctness under injected faults — first is first, worst "
          "is worst") {
    const auto kin = xDrive(Length{7.0});
    ForwardingSink fwd;
    ChassisRig cr{kin, plantConfig(), &fwd};
    FakeTelemetrySink capture;  // struct-level capture: exact field checks
    fwd.target = &capture;
    RunReporter reporter{capture, cr.chassis.scheduler()};
    reporter.sessionStart(SessionInfo{.buildHash = "abc1234", .routineId = "faulty"});

    // The ROOT CAUSE, first, at a known time:
    cr.rig.h.clock().advance(Time{1.0});
    cr.rig.latch.raise(FaultCode::ImuLost, "IMU", "injected first");
    // A cascade AND health episodes after it:
    cr.rig.latch.raise(FaultCode::GpsGateReject, "LOC", "");
    cr.rig.latch.raise(FaultCode::GpsGateReject, "LOC", "");
    cr.rig.health.tick({.batteryVolts = Voltage{9.0}});  // brownout latches
    // Loop gaps: 30 ms then 22 ms — the worst must be the WORST, not the last.
    MotionScheduler& sched = cr.chassis.scheduler();
    (void)sched.tick();  // baseline
    cr.rig.h.clock().advance(Time{0.030});
    (void)sched.tick();
    cr.rig.h.clock().advance(Time{0.022});
    (void)sched.tick();
    REQUIRE(cr.chassis.moveTo(Pose2d{Length{12.0}, Length{0.0}, Angle{}},
                              {.timeoutSeconds = 8.0})
            == ExitReason::Settled);
    reporter.finishRun();

    REQUIRE(capture.summaryCount() == 1);
    const RunSummary& s = capture.lastSummary();
    CHECK(s.firstFault == FaultCode::ImuLost);  // NOT Brownout, NOT LoopOverrun
    CHECK(s.firstFaultTime.value() == doctest::Approx(1.0));
    CHECK(s.worstLoopDt.value() == doctest::Approx(0.030));  // the 30, not the 22
    CHECK(s.brownout);
    CHECK(s.gatingRejects == 2);
    CHECK(s.motionsStarted == 1);
    CHECK(s.motionsSettled == 1);
    CHECK(s.hasHeadingData);
    CHECK(s.buildHash() == "abc1234");
    CHECK(s.routineId() == "faulty");
    CHECK(s.batteryEnd.value() == doctest::Approx(s.batteryStart.value()).epsilon(0.2));
}

// Bug caught: D-2 dark in vivo — the full stack flooding a tight record budget
// with drops that are not counted, not stamped, and not in the summary.
TEST_CASE("C5 e2e: a flooded record channel drops visibly — counted, stamped on the "
          "wire, reported in the summary") {
    const auto kin = xDrive(Length{7.0});
    ForwardingSink fwd;
    ChassisRig cr{kin, plantConfig(), &fwd};
    FakeTelemetrySink inner;
    RateLimitedSink limiter{inner, cr.rig.h.clock(),
                            RateLimitConfig{.recordsPerSecond = 20.0}};  // vs ~100 Hz stream
    fwd.target = &limiter;
    RunReporter reporter{inner, cr.chassis.scheduler(), &limiter};

    REQUIRE(cr.chassis.moveTo(Pose2d{Length{24.0}, Length{0.0}, Angle{}},
                              {.timeoutSeconds = 8.0})
            == ExitReason::Settled);
    reporter.finishRun();

    CHECK(limiter.droppedRecords() > 0);  // the flood really dropped
    REQUIRE(inner.recordCount() > 0);
    CHECK(inner.lastRecord().droppedRecords > 0);  // the wire explains its own gap
    REQUIRE(inner.summaryCount() == 1);
    CHECK(inner.lastSummary().droppedRecords == limiter.droppedRecords());
    CHECK(inner.lastSummary().droppedLines == limiter.droppedLines());
}

// ═══ Cost (constraint 6) ═══════════════════════════════════════════════════════════

// Bug caught: the C5 chain acquiring a per-tick cost in the competition build —
// a stats/stamp/attribution path that populates records over NullSink, or
// attribution existing when no clock was given. Also pins the honest consequence:
// no records ⇒ hasPathData=false ⇒ the result line says n/a, never a lie — while
// finalPose stays REAL (boundary-read).
TEST_CASE("C5 cost: the NullSink chain never builds a record; attribution absent by "
          "default; the n/a path is honest") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};  // harness sink defaulted → NullSink
    // The A1 mechanism THROUGH the C5 chain (stamper → stats → NullSink):
    CHECK_FALSE(s.sched.deps().ctx->telemetry().wantsRecord());
    int builds = 0;
    emitRecord(s.sched.deps().ctx->telemetry(), [&] {
        ++builds;
        return DebugRecord{};
    });
    CHECK(builds == 0);
    CHECK(s.sched.attribution() == nullptr);  // off unless a clock is injected

    const Pose2d target{Length{18.0}, Length{6.0}, Angle::degrees(30.0)};
    FakeClock reportClock;
    FakeCharSink out;
    TermSink term{reportClock, out};
    RunReporter reporter{term, s.sched};
    MoveToPose m{s.sched.deps(), target, motionConfig(), 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);

    const CompletedMotion& c = s.sched.lastCompleted();
    CHECK_FALSE(c.hasPathData);                      // no stream, no derived numbers
    CHECK(posErr(c.finalPose, target) < 1.0);        // …but the boundary pose is REAL
    CHECK(out.text().find("over   n/a  drift  n/a") != std::string::npos);
    CHECK_FALSE(s.sched.runHasHeadingData());        // and the summary will say n/a
}

// ═══ D-3 through the scheduler ═════════════════════════════════════════════════════

namespace {
/// A motion that burns ATTRIBUTION-clock time inside its tick (each now() call
/// advances the injected auto-advance clock) — the deliberately slow subsystem.
/// Emits one record per tick through the STAMPED chain (deps.ctx->telemetry()),
/// because the stamp-visibility assertions need records that actually rode the
/// scheduler's decorators — the harness's own plant records deliberately do not
/// (the C2 §4.3 two-producers lesson).
class BusyMotion final : public shulib::motion::IMotion {
public:
    BusyMotion(const shulib::motion::MotionDeps& deps, shulib::hal::IClock& busyClock,
               int ticks) noexcept
        : deps_{&deps}, busy_{&busyClock}, total_{ticks} {}
    void start() override {
        n_ = 0;
        reason_ = ExitReason::Running;
        state_ = MotionState::Running;
    }
    [[nodiscard]] ExitReason tick() override {
        if (reason_ != ExitReason::Running) {
            return reason_;
        }
        for (int i = 0; i < 10; ++i) {
            (void)busy_->now();  // 10 quanta of "work"
        }
        if (++n_ >= total_) {
            reason_ = ExitReason::Settled;
            state_ = MotionState::Settled;
        }
        emitRecord(deps_->ctx->telemetry(), [&] {
            DebugRecord r;
            r.t = deps_->ctx->clock().now();
            r.activeCommandState = static_cast<std::uint8_t>(state_);
            return r;
        });
        return reason_;
    }
    void cancel() override {
        if (reason_ == ExitReason::Running && state_ != MotionState::Idle) {
            reason_ = ExitReason::Cancelled;
            state_ = MotionState::Cancelled;
        }
    }
    [[nodiscard]] ExitReason exitReason() const noexcept override { return reason_; }
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    [[nodiscard]] const char* name() const noexcept override { return "BusyMotion"; }

private:
    const shulib::motion::MotionDeps* deps_;
    shulib::hal::IClock* busy_;
    int total_;
    int n_ = 0;
    ExitReason reason_ = ExitReason::Running;
    MotionState state_ = MotionState::Idle;
};

/// An IClock that advances a fixed quantum per now() call — deterministic
/// intra-tick time for attribution tests (the sim clock only moves between ticks).
class AutoAdvanceClock final : public shulib::hal::IClock {
public:
    explicit AutoAdvanceClock(double quantum) noexcept : quantum_{quantum} {}
    [[nodiscard]] Time now() const override {
        const double t = t_;
        t_ += quantum_;
        return Time{t};
    }

private:
    double quantum_;
    mutable double t_ = 0.0;
};
}  // namespace

// Bug caught: the scheduler wiring of D-3 broken — phases not measured around the
// real localization/motion work, not stamped into records (with the documented
// one-tick lag), spare slots polluted, or the overrun line failing to NAME the
// deliberately slow subsystem in vivo.
TEST_CASE("C5 e2e: scheduler attribution measures phases, stamps records, and the "
          "overrun line names the slow subsystem") {
    const auto kin = xDrive(Length{7.0});
    AutoAdvanceClock attClock{0.0005};  // 0.5 ms per now()
    MotionSchedulerConfig cfg;
    cfg.attributionClock = &attClock;
    FakeTelemetrySink sink;
    SchedulerRig s{kin, plantConfig(), &sink, nullptr, cfg};
    REQUIRE(s.sched.attribution() != nullptr);

    BusyMotion busy{s.sched.deps(), attClock, 8};
    s.sched.async(busy);
    // Force ONE mid-run loop gap so the overrun path fires while BusyMotion's
    // tick is the last completed attribution story. The gap must come AFTER the
    // monitor's post-reset baseline tick (predicate call 1 precedes tick 1,
    // which only baselines) — fire it before tick 3.
    int predCalls = 0;
    const WaitResult wr = s.sched.waitUntil(
        [&] {
            if (++predCalls == 3) {
                s.rig.h.clock().advance(Time{0.030});  // > the 15 ms budget
            }
            return !s.sched.hasActiveMotion();
        },
        5.0);
    CHECK(wr == WaitResult::Satisfied);

    // Phase math (the auto-advance quanta are exact): a phase scope brackets with
    // one now() each side; BusyMotion burns 10 more → loc = 1q, mot = 11q.
    const auto* att = s.sched.attribution();
    REQUIRE(att->hasCompletedTick());
    CHECK(att->lastPhases()[0].value() == doctest::Approx(0.0005));
    CHECK(att->lastPhases()[1].value() == doctest::Approx(0.0055));
    CHECK(att->lastWorstPhase() == TickPhase::Motion);

    // Stamped onto the wire (one-tick lag: a STAMPED record after the first
    // tick — the plant's own records deliberately bypass the stamper, C2 §4.3,
    // so pick the last record that rode the scheduler chain: nonzero id).
    REQUIRE(sink.recordCount() >= 2);
    const DebugRecord* lastStamped = nullptr;
    for (int i = 0; i < sink.recordCount(); ++i) {
        if (sink.recordAt(i).activeCommandId != 0) {
            lastStamped = &sink.recordAt(i);
        }
    }
    REQUIRE(lastStamped != nullptr);
    CHECK(lastStamped->tickPhase[0].value() == doctest::Approx(0.0005));
    CHECK(lastStamped->tickPhase[1].value() == doctest::Approx(0.0055));
    for (std::size_t i = 2; i < lastStamped->tickPhase.size(); ++i) {
        CHECK(lastStamped->tickPhase[i].value() == 0.0);  // reserved slots stay quiet
    }

    // The overrun named its consumer, in vivo:
    CHECK(s.rig.latch.raiseCount(FaultCode::LoopOverrun) >= 1);
    bool named = false;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).level == LogLevel::Warn && sink.at(i).subsystem == "SCH"
            && sink.at(i).message.find("overrun attribution:") != std::string::npos
            && sink.at(i).message.find("(worst mot)") != std::string::npos) {
            named = true;
        }
    }
    CHECK(named);
}

// ═══ The observer seam's guard rails ═══════════════════════════════════════════════

namespace {
class VerbCallingObserver final : public IMotionObserver {
public:
    explicit VerbCallingObserver(MotionScheduler& sched) noexcept : sched_{&sched} {}
    void onMotionComplete(const CompletedMotion& /*completed*/) override {
        sched_->cancel();  // forbidden: re-planning from inside a boundary
    }

private:
    MotionScheduler* sched_;
};
}  // namespace

// Bug caught: a boundary observer re-entering the scheduler (canceling/starting
// motions from inside finalize) — half-consistent state mutated under the
// boundary's feet. Must be a loud precondition, not undefined behavior.
TEST_CASE("C5 e2e: a boundary observer calling scheduler verbs trips a loud "
          "precondition") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    VerbCallingObserver evil{s.sched};
    s.sched.setBoundaryObserver(&evil);
    MoveToPose m{s.sched.deps(), Pose2d{Length{6.0}, Length{0.0}, Angle{}},
                 motionConfig(), 8.0};
    s.sched.async(m);
    CHECK_THROWS_AS((void)s.sched.waitUntilSettled(), PreconditionError);
    s.sched.setBoundaryObserver(nullptr);  // detach so teardown stays clean
}

// ═══ Hostility (the A3 composed world) ═════════════════════════════════════════════

// Bug caught: the report DEGRADING under hostility exactly when it matters — a
// NaN leaking into the text, result lines lost, the summary incoherent with the
// counters, or believed-vs-true divergence beyond the C2/C3 hostile bounds
// without the report saying anything.
TEST_CASE("C5 hostile: a full composed-hostility run produces a coherent, complete "
          "report") {
    const auto kin = xDrive(Length{7.0});
    FullHostility world{};
    auto pcfg = plantConfig();
    pcfg.plant.seed = 11;
    ForwardingSink fwd;
    ChassisRig cr{kin, pcfg, &fwd, &world.model()};
    FakeCharSink out;
    TermSink term{cr.rig.h.clock(), out};
    fwd.target = &term;
    RunReporter reporter{term, cr.chassis.scheduler()};
    reporter.sessionStart(SessionInfo{.buildHash = shulib::diag::compiledBuildHash(),
                                      .routineId = "hostile", .alliance = "blue",
                                      .side = "right", .portMap = "sim"});

    Rng wp{321};
    Pose2d target{};
    int settled = 0;
    double worstBelievedVsTrue = 0.0;
    for (int k = 0; k < 5; ++k) {
        target = Pose2d{Length{wp.uniform(-35.0, 35.0)}, Length{wp.uniform(-35.0, 35.0)},
                        Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi))};
        const ExitReason r = cr.chassis.moveTo(target, {.timeoutSeconds = 8.0});
        if (r == ExitReason::Settled) {
            ++settled;
            const CompletedMotion& c = cr.chassis.lastCompleted();
            worstBelievedVsTrue = std::max(
                worstBelievedVsTrue, posErr(c.finalPose, cr.rig.h.truePose()));
        }
    }
    reporter.finishRun();
    MESSAGE("hostile: settled " << settled << "/5, worst believed-vs-true "
                                << worstBelievedVsTrue << "in");

    const std::string& text = out.text();
    // Complete: every boundary reported, one summary, ledger agrees with reality.
    CHECK(countOf(text, "✓SETTLED") == settled);
    CHECK(countOf(text, "MoveToPose#") == 5);
    CHECK(countOf(text, "── RUN SUMMARY ") == 1);
    char ledger[64];
    std::snprintf(ledger, sizeof ledger, " motions 5 · settled %d ·", settled);
    CHECK(text.find(ledger) != std::string::npos);
    // Coherent: the §18.3 tokens only — no libc NaN spellings ANYWHERE in a run
    // whose sensors actively lie.
    CHECK(text.find("nan") == std::string::npos);
    CHECK(text.find("inf") == std::string::npos);
    // The believed-vs-true gap stays inside the C2/C3 hostile class — the report
    // is honest to within the estimator's documented hostile envelope.
    CHECK(worstBelievedVsTrue < 5.0);
    CHECK(settled >= 3);  // the hostile world is survivable (C2/C3 pinned harder)
}
