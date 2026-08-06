// C2 MotionScheduler — the semantics tests. Every case names the bug it would
// catch. The scheduler is graded the way the motions were at C1: against plant
// GROUND TRUTH (h.truePose()), with the estimate reserved for what the code
// under test actually read.
//
// The bounded-loop shape matters: every blocking wait here runs against a
// HARD-CAPPED pacer (motion_test_rig.hpp PlantPacer), so a mutation that
// defeats a wait bound turns the suite RED instead of hanging it — the same
// design that let C1's defeated-watchdog mutation read as red.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"

using namespace motion_rig;
using shulib::PreconditionError;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::faultBit;
using shulib::motion::IMotion;
using shulib::motion::MotionScheduler;
using shulib::motion::MotionSchedulerConfig;
using shulib::motion::MotionState;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::motion::WaitResult;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::FullHostility;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::JitterSchedule;
using shulib::sim::PowerHostileConfig;
using shulib::sim::PowerHostileModel;
using shulib::units::Voltage;

namespace {

/// Every drive motor commands exactly 0 V under BrakeMode::Brake — the defined
/// cancel safe state, checked motor by motor.
void checkSafeState(MotionRig& rig) {
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
}

/// A hostile IMotion whose tick energizes the drive and then breaches a
/// precondition — the nastiest task-boundary shape: the throw leaves motors
/// HOT unless the scheduler's catch safes them.
class ThrowingMotion final : public IMotion {
public:
    explicit ThrowingMotion(const shulib::motion::MotionDeps& deps) : deps_{deps} {}
    void start() override {
        reason_ = ExitReason::Running;
        state_ = MotionState::Running;
    }
    [[nodiscard]] ExitReason tick() override {
        for (shulib::hal::IMotor* m : deps_.ctx->driveMotors()) {
            m->setVoltage(Voltage{5.0});
        }
        throw PreconditionError{"deliberate mid-tick contract breach"};
    }
    void cancel() override {
        if (state_ == MotionState::Idle) {
            return;
        }
        shulib::motion::applyCancelSafeState(*deps_.ctx);
        if (reason_ != ExitReason::Running) {
            return;
        }
        reason_ = ExitReason::Cancelled;
        state_ = MotionState::Cancelled;
    }
    [[nodiscard]] ExitReason exitReason() const noexcept override { return reason_; }
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    [[nodiscard]] const char* name() const noexcept override { return "ThrowingMotion"; }

private:
    shulib::motion::MotionDeps deps_;
    ExitReason reason_ = ExitReason::Running;
    MotionState state_ = MotionState::Idle;
};

/// A motion that tries to command its own scheduler from inside tick() — the
/// re-entrancy breach the preconditions exist to reject.
class SchedulerCallingMotion final : public IMotion {
public:
    SchedulerCallingMotion(const shulib::motion::MotionDeps& deps, MotionScheduler& sched)
        : deps_{deps}, sched_{&sched} {}
    void start() override {
        reason_ = ExitReason::Running;
        state_ = MotionState::Running;
    }
    [[nodiscard]] ExitReason tick() override {
        sched_->cancel();  // must precondition-throw: the slot is mid-tick
        return ExitReason::Running;
    }
    void cancel() override {
        if (state_ == MotionState::Idle) {
            return;
        }
        shulib::motion::applyCancelSafeState(*deps_.ctx);
        if (reason_ != ExitReason::Running) {
            return;
        }
        reason_ = ExitReason::Cancelled;
        state_ = MotionState::Cancelled;
    }
    [[nodiscard]] ExitReason exitReason() const noexcept override { return reason_; }
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    [[nodiscard]] const char* name() const noexcept override { return "SchedCaller"; }

private:
    shulib::motion::MotionDeps deps_;
    MotionScheduler* sched_;
    ExitReason reason_ = ExitReason::Running;
    MotionState state_ = MotionState::Idle;
};

/// A pacer that never advances the clock — the broken world-advancer the
/// stalled-pace guard exists for. Its own call counter is the external bound
/// that keeps the GUARD-REMOVED mutation red instead of hanging the suite.
struct FrozenPacer final : shulib::motion::ITickPacer {
    int calls = 0;
    int cap = 100000;
    void pace() override {
        if (calls >= cap) {
            throw std::runtime_error("FrozenPacer: cap exceeded — the stall guard is dead");
        }
        ++calls;
    }
};

/// A records-off sink that flags if emit() is EVER invoked: the probe for the
/// A1 cost contract through the id stamp (wantsRecord must forward as false).
class RecordsOffProbeSink final : public shulib::hal::ITelemetrySink {
public:
    void log(shulib::hal::LogLevel, std::string_view, std::string_view) override {}
    // wantsRecord() deliberately NOT overridden: default false (records-off).
    void emit(const shulib::diag::DebugRecord&) override { emitCalled = true; }
    bool emitCalled = false;
};

const Pose2d kTargetA{Length{40.0}, Length{0.0}, Angle{}};
const Pose2d kTargetB{Length{10.0}, Length{-20.0}, Angle::degrees(90.0)};

}  // namespace

// ═══ Wire-stable vocabulary pins ═══════════════════════════════════════════════════

// Bug caught: a reorder/renumber of the appended cancel vocabulary — these
// values ride DebugRecord.activeCommandState onto the F9 wire.
TEST_CASE("C2 vocabulary: MotionState::Cancelled == 5, appended after TimedOut") {
    CHECK(static_cast<int>(MotionState::Cancelled) == 5);
    CHECK(static_cast<int>(MotionState::TimedOut) == 4);  // unchanged by the append
}

// Bug caught: faultBit disagreeing with the FaultCode values the mask is
// documented against (the config would silently watch the wrong codes).
TEST_CASE("C2 vocabulary: faultBit maps the wire values") {
    CHECK(faultBit(FaultCode::OdoStuck) == (1U << 4));
    CHECK(faultBit(FaultCode::Brownout) == (1U << 7));
    CHECK(MotionSchedulerConfig{}.abortFaultMask == faultBit(FaultCode::OdoStuck));
}

// ═══ async() ═══════════════════════════════════════════════════════════════════════

// Bug caught: async blocking (ticking the world before returning) or failing
// to arm the motion at all.
TEST_CASE("C2 async: returns without advancing the world; the motion runs on scheduler ticks") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};

    const double t0 = s.rig.h.clock().now().value();
    s.sched.async(m);
    CHECK(s.rig.h.clock().now().value() == t0);  // no tick happened
    CHECK(s.pacer.paces == 0);                   // no world advance happened
    CHECK(s.sched.hasActiveMotion());
    CHECK(s.sched.activeCommandId() == 1);
    CHECK(m.state() != MotionState::Idle);  // armed

    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetA) < 0.6);  // graded against TRUTH
    CHECK_FALSE(s.sched.hasActiveMotion());
    CHECK(s.sched.motionsStarted() == 1);
    CHECK(s.sched.motionsSettled() == 1);
}

// Bug caught: scheduler state leaking across motions (stale controllers, stale
// ids, a boundary that never clears) — the chained shape every routine uses.
TEST_CASE("C2 sequencing: move -> turn -> strafe chained through the scheduler, each graded on truth") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};

    MoveToPose m1{s.sched.deps(), Pose2d{Length{24.0}, Length{18.0}, Angle::degrees(45.0)},
                  motionConfig(), 8.0};
    REQUIRE(s.run(m1) == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(),
                 Pose2d{Length{24.0}, Length{18.0}, Angle::degrees(45.0)}) < 0.6);

    TurnTo m2{s.sched.deps(), Angle::degrees(-120.0), motionConfig(), 8.0};
    REQUIRE(s.run(m2) == ExitReason::Settled);
    CHECK(std::abs(s.rig.h.truePose().heading().errorTo(Angle::degrees(-120.0))) < 0.035);

    StrafeTo m3{s.sched.deps(), Length{-10.0}, Length{6.0}, motionConfig(), 8.0};
    REQUIRE(s.run(m3) == ExitReason::Settled);
    CHECK(std::hypot(s.rig.h.truePose().x().value() + 10.0,
                     s.rig.h.truePose().y().value() - 6.0) < 0.6);

    CHECK(s.sched.motionsStarted() == 3);
    CHECK(s.sched.motionsSettled() == 3);
    CHECK(s.sched.completedCount() == 3);
    CHECK(s.sched.lastCompleted().id == 3);  // ids are 1-based and monotonic
    CHECK_FALSE(s.rig.latch.hasFault());
}

// ═══ One-at-a-time: pre-emption ════════════════════════════════════════════════════

// Bug caught: THE two-motions bug — async-while-active leaving the old motion
// live (able to command) or leaving its last command on the motors across the
// swap. This is the load-bearing structural claim of the chunk.
TEST_CASE("C2 pre-empt: async while active cancels the old motion into the safe state first") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose a{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    MoveToPose b{s.sched.deps(), kTargetB, motionConfig(), 8.0};

    s.sched.async(a);
    REQUIRE(s.sched.waitUntil([&] { return s.rig.loc.pose().x().value() > 15.0; }, 10.0)
            == WaitResult::Satisfied);  // a is mid-flight at speed

    s.sched.async(b);  // pre-empt: BEFORE any tick of b —
    CHECK(a.state() == MotionState::Cancelled);      // the old object is inert
    CHECK(a.exitReason() == ExitReason::Cancelled);
    checkSafeState(s.rig);                           // and the drive is braked NOW
    CHECK(s.sched.activeCommandId() == 2);
    CHECK(s.sched.lastCompleted().name == a.name());
    CHECK(s.sched.lastCompleted().exit == ExitReason::Cancelled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::None);

    // b drives; a stray tick of the CANCELLED a must not touch the motors.
    // (The predicate demands genuine b-progress — y well below the swap point —
    // so b is provably mid-flight and commanding when the audit runs.)
    REQUIRE(s.sched.waitUntil([&] { return s.rig.loc.pose().y().value() < -8.0; }, 10.0)
            == WaitResult::Satisfied);
    std::vector<double> before;
    bool anyNonzero = false;
    for (int w = 0; w < s.rig.h.motorCount(); ++w) {
        before.push_back(s.rig.h.motor(w).commandedVoltage().value());
        anyNonzero = anyNonzero || before.back() != 0.0;
    }
    REQUIRE(anyNonzero);  // b is genuinely commanding
    (void)a.tick();       // the stray tick of the CANCELLED motion
    CHECK(a.exitReason() == ExitReason::Cancelled);
    for (int w = 0; w < s.rig.h.motorCount(); ++w) {
        CHECK(s.rig.h.motor(w).commandedVoltage().value()
              == before[static_cast<std::size_t>(w)]);  // untouched by the stray tick
    }

    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetB) < 0.6);  // b's ABSOLUTE target
    CHECK(s.sched.motionsStarted() == 2);
    CHECK(s.sched.motionsCancelled() == 1);
    CHECK(s.sched.motionsSettled() == 1);
}

// Bug caught: async(active motion) corrupting the slot instead of behaving as
// the well-defined restart (cancel + full re-arm, fresh watchdog, new id).
TEST_CASE("C2 pre-empt: async of the ACTIVE motion is a restart") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};

    s.sched.async(m);
    REQUIRE(s.sched.waitUntil([&] { return s.rig.loc.pose().x().value() > 15.0; }, 10.0)
            == WaitResult::Satisfied);
    s.sched.async(m);  // restart mid-flight
    CHECK(s.sched.activeCommandId() == 2);
    CHECK(m.state() == MotionState::WaitingForEstimate);  // fully re-armed
    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetA) < 0.6);
    CHECK(s.sched.motionsStarted() == 2);
    CHECK(s.sched.motionsCancelled() == 1);
    CHECK(s.sched.motionsSettled() == 1);
}

// ═══ cancel(): the safe state, in every case ═══════════════════════════════════════

// Bug caught: THE robot-into-wall bug — a cancel that leaves the last
// commanded voltage on the motors, or that needs further ticks to become safe.
TEST_CASE("C2 cancel mid-motion: motors braked by the call itself; the drivetrain reaches rest") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    // A LAGGED plant (kA > 0, tau = kA/kV ~ 0.18 s) on purpose: the default
    // memoryless rig stops instantly at 0 V, which would make "the drivetrain
    // reaches rest" vacuous. With real wheel inertia the robot must be SHOWN
    // to decay to rest on the braked command, and the coast distance is a
    // meaningful number.
    auto pcfg = plantConfig();
    pcfg.plant.wheelFf.kA = 0.03;
    SchedulerRig s{kin, pcfg, &sink};
    MoveToPose m{s.sched.deps(), Pose2d{Length{60.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};

    s.sched.async(m);
    REQUIRE(s.sched.waitUntil([&] { return s.rig.loc.pose().x().value() > 20.0; }, 10.0)
            == WaitResult::Satisfied);
    const double speedAtCancel = std::hypot(s.rig.h.trueBodyTwist().vx().value(),
                                            s.rig.h.trueBodyTwist().vy().value());
    REQUIRE(speedAtCancel > 20.0);  // genuinely at speed — the dangerous case

    s.sched.cancel();
    checkSafeState(s.rig);  // ONE call: zero volts + Brake, no tick needed
    CHECK(m.state() == MotionState::Cancelled);
    CHECK_FALSE(s.sched.hasActiveMotion());
    CHECK(s.sched.lastExitReason() == ExitReason::Cancelled);
    CHECK(s.sched.motionsCancelled() == 1);
    // The exit record is honest: state Cancelled, commanded zero, still id 1.
    const auto& r = sink.lastRecord();
    CHECK(r.activeCommandState == static_cast<std::uint8_t>(MotionState::Cancelled));
    CHECK(r.activeCommandId == 1);
    CHECK(r.commanded.vx().value() == 0.0);
    CHECK(r.commanded.vy().value() == 0.0);

    // Physics: with NO further scheduler involvement the plant must come to
    // rest on the zero-volt command (the plant does not model brake torque —
    // motion.hpp HA-53 note — so this is the CONSERVATIVE stopping estimate).
    const Pose2d atCancel = s.rig.h.truePose();
    for (int i = 0; i < 150; ++i) {
        s.rig.h.plant().step(Time{0.01});
    }
    const double residual = std::hypot(s.rig.h.trueBodyTwist().vx().value(),
                                       s.rig.h.trueBodyTwist().vy().value());
    CHECK(residual < 0.5);  // at rest
    const double coast = posErr(s.rig.h.truePose(), atCancel);
    MESSAGE("cancel from ", speedAtCancel, " in/s: coast-to-rest distance ", coast,
            " in over 1.5 s (0 V only; real Brake mode can only be shorter — HA-53)");
    CHECK(coast > 1.0);   // the lagged plant genuinely coasted (else this test is vacuous)
    CHECK(coast < 20.0);  // bounded even without modeled brake torque
}

// Bug caught: a first-tick / zero-tick cancel path that dereferences state the
// motion has not built yet, or that skips the safe state because "nothing ran".
TEST_CASE("C2 cancel at the very first opportunity: before any tick") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    s.sched.async(m);
    s.sched.cancel();  // zero ticks have happened
    CHECK(m.state() == MotionState::Cancelled);
    checkSafeState(s.rig);
    CHECK_FALSE(s.sched.hasActiveMotion());
    CHECK(s.sched.motionsCancelled() == 1);
    CHECK(posErr(s.rig.h.truePose(), Pose2d{}) < 1e-9);  // never moved
}

// Bug caught: a boot-window cancel that reads the (nonexistent) estimate or a
// never-captured target — and a scheduler left unusable by a boot cancel.
TEST_CASE("C2 cancel during the boot window: safe, honest, and the scheduler stays usable") {
    ImuHostileModel imu{ImuHostileConfig{}};  // 2 s calibration window
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    StrafeTo m{s.sched.deps(), Length{20.0}, Length{0.0}, motionConfig(), 8.0};

    s.sched.async(m);
    REQUIRE(s.sched.waitUntil([] { return false; }, 0.5) == WaitResult::TimedOut);
    REQUIRE(m.state() == MotionState::WaitingForEstimate);  // still in the boot window
    s.sched.cancel();
    CHECK(m.state() == MotionState::Cancelled);
    checkSafeState(s.rig);
    CHECK(posErr(s.rig.h.truePose(), Pose2d{}) < 1e-9);  // zero volts throughout: never moved

    // The scheduler (and the field) recover: a fresh motion waits out the rest
    // of calibration and lands.
    MoveToPose m2{s.sched.deps(), Pose2d{Length{24.0}, Length{0.0}, Angle{}}, motionConfig(),
                  8.0};
    REQUIRE(s.run(m2) == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), Pose2d{Length{24.0}, Length{0.0}, Angle{}}) < 1.5);
}

// Bug caught: cancel rewriting history (a settled/timed-out verdict must
// survive), double-cancel double-counting, or a panic stop that stops working
// when nothing is active.
TEST_CASE("C2 cancel after exit: verdict preserved, idempotent, panic stop always available") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), Pose2d{Length{20.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);

    // Direct motion-level cancel after settle: verdict preserved.
    m.cancel();
    CHECK(m.state() == MotionState::Settled);
    CHECK(m.exitReason() == ExitReason::Settled);
    checkSafeState(s.rig);  // but the safe state WAS applied (brake vs settle's coast)

    // Scheduler-level cancel with no active motion: the panic stop.
    s.sched.cancel();
    checkSafeState(s.rig);
    CHECK(s.sched.lastExitReason() == ExitReason::Settled);  // no boundary invented
    CHECK(s.sched.motionsCancelled() == 0);

    // Back-to-back cancels: no state change, no faults, no extra boundaries.
    s.sched.cancel();
    s.sched.cancel();
    m.cancel();
    CHECK(m.state() == MotionState::Settled);
    CHECK(s.sched.completedCount() == 1);
    CHECK_FALSE(s.rig.latch.hasFault());
}

// Bug caught: cancelling an already-timed-out motion overwriting TimedOut (the
// C5 result line would lie about why the motion ended).
TEST_CASE("C2 cancel an already-timed-out motion: TimedOut is preserved") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{1e17};  // never live -> the motion must time out
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 1.5};

    s.sched.async(m);
    REQUIRE(s.sched.waitUntilSettled() == ExitReason::TimedOut);
    CHECK(s.rig.latch.firstFault() == FaultCode::MotionTimeout);

    s.sched.cancel();  // no active motion left: panic path
    m.cancel();        // direct: must not rewrite the verdict
    CHECK(m.state() == MotionState::TimedOut);
    CHECK(m.exitReason() == ExitReason::TimedOut);
    CHECK(s.sched.lastExitReason() == ExitReason::TimedOut);
    CHECK(s.sched.lastCompleted().exit == ExitReason::TimedOut);
    CHECK(s.sched.motionsCancelled() == 0);
    checkSafeState(s.rig);
    CHECK(posErr(s.rig.h.truePose(), Pose2d{}) < 1e-9);  // waited at zero volts throughout
}

// ═══ waitUntilSettled() ════════════════════════════════════════════════════════════

// Bug caught: an unbounded wait — the motion's watchdog must surface THROUGH
// the scheduler as a TimedOut return, not a hang (pacer cap = the red guard).
TEST_CASE("C2 waitUntilSettled: returns TimedOut via the motion watchdog, never hangs") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{1e17};  // never live
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 1.5};
    s.sched.async(m);
    CHECK(s.sched.waitUntilSettled() == ExitReason::TimedOut);
    CHECK(s.pacer.paces < 200);  // ~150 ticks of 10 ms: bounded by the 1.5 s watchdog
    CHECK(s.sched.motionsTimedOut() == 1);
}

// Bug caught: a zero-motion scheduler that ticks, waits, or crashes its way
// into inventing state (the vacuous-wait contract).
TEST_CASE("C2 zero-motion routine: waits are vacuous, ticks are idle, nothing is invented") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), &sink};

    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);  // vacuously over
    CHECK(s.pacer.paces == 0);
    CHECK(s.sched.completedCount() == 0);  // …and distinguishable from a real settle

    // Idle ticks: health + a quiet record, no target invented, id 0/state 0.
    for (int i = 0; i < 5; ++i) {
        CHECK_FALSE(s.sched.tick());
        s.pacer.pace();
    }
    REQUIRE(sink.recordCount() >= 5);
    const auto& r = sink.lastRecord();
    CHECK(r.activeCommandId == 0);
    CHECK(r.activeCommandState == 0);
    CHECK(r.targetPose.x().value() == 0.0);
    CHECK(r.commanded.vx().value() == 0.0);

    CHECK(s.sched.waitUntil([] { return true; }, 0.0) == WaitResult::Satisfied);
    s.sched.cancel();  // panic stop with nothing ever started
    checkSafeState(s.rig);
    CHECK(s.sched.motionsStarted() == 0);
    CHECK_FALSE(s.rig.latch.hasFault());
}

// ═══ waitUntil(pred, timeout) ══════════════════════════════════════════════════════

// Bug caught: the marker primitive failing to tick the active motion (a wait
// during which the robot freezes), or missing the mid-flight condition.
TEST_CASE("C2 waitUntil: satisfied mid-motion while the motion keeps driving") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    s.sched.async(m);
    const WaitResult r =
        s.sched.waitUntil([&] { return s.rig.loc.pose().x().value() > 15.0; }, 10.0);
    CHECK(r == WaitResult::Satisfied);
    CHECK(s.sched.hasActiveMotion());                  // the motion is NOT disturbed
    CHECK(s.rig.h.truePose().x().value() > 10.0);      // it genuinely drove meanwhile
    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetA) < 0.6);
}

// Bug caught: a never-true predicate hanging, the timeout being reported as
// success, a spurious fault for a legitimate strategy branch, or the deadline
// drifting from the requested bound.
TEST_CASE("C2 waitUntil: never-true predicate times out at the deadline, distinguishably, no fault") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), &sink};
    const double t0 = s.rig.h.clock().now().value();
    const WaitResult r = s.sched.waitUntil([] { return false; }, 0.75);
    CHECK(r == WaitResult::TimedOut);
    const double elapsed = s.rig.h.clock().now().value() - t0;
    CHECK(elapsed >= 0.75);
    CHECK(elapsed < 0.78);  // deadline honored to within one tick
    CHECK(s.rig.latch.faultCount() == 0);  // a timed-out wait is NOT a pathology
    // …but it is visible: exactly one Warn line from the scheduler.
    bool sawWarn = false;
    for (int i = 0; i < sink.size(); ++i) {
        const auto& e = sink.at(i);
        if (e.level == shulib::hal::LogLevel::Warn && e.subsystem == "SCH") {
            sawWarn = true;
            CHECK(e.message.find("timed out") != std::string::npos);
        }
    }
    CHECK(sawWarn);
}

// Bug caught: the entry check missing (a true-on-entry predicate paying a tick
// / advancing the world) and the zero-timeout poll misbehaving either way.
TEST_CASE("C2 waitUntil: true on entry returns instantly; timeout 0 is an honest poll") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    const double t0 = s.rig.h.clock().now().value();
    CHECK(s.sched.waitUntil([] { return true; }, 5.0) == WaitResult::Satisfied);
    CHECK(s.rig.h.clock().now().value() == t0);  // zero ticks
    CHECK(s.pacer.paces == 0);                   // zero world advance
    CHECK(s.sched.waitUntil([] { return true; }, 0.0) == WaitResult::Satisfied);
    CHECK(s.sched.waitUntil([] { return false; }, 0.0) == WaitResult::TimedOut);
    CHECK(s.pacer.paces == 0);  // polls never advance the world either
}

// Bug caught: a wait during a motion timing out but reporting the MOTION's
// fate — the two verdicts must stay independent.
TEST_CASE("C2 waitUntil: times out while a motion is active without disturbing it") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    s.sched.async(m);
    CHECK(s.sched.waitUntil([] { return false; }, 0.3) == WaitResult::TimedOut);
    CHECK(s.sched.hasActiveMotion());  // the motion rides on
    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetA) < 0.6);
}

// ═══ Re-entrancy (decided and pinned) ══════════════════════════════════════════════

// Bug caught: async-from-predicate corrupting the active slot — the pinned
// semantics is clean pre-emption from inside the wait loop.
TEST_CASE("C2 re-entrancy: async from inside a waitUntil predicate pre-empts cleanly") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose a{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    MoveToPose b{s.sched.deps(), kTargetB, motionConfig(), 8.0};

    s.sched.async(a);
    bool fired = false;
    const WaitResult r = s.sched.waitUntil(
        [&] {
            if (!fired && s.rig.loc.pose().x().value() > 12.0) {
                fired = true;
                s.sched.async(b);  // start a motion from INSIDE the predicate
            }
            return fired && !s.sched.hasActiveMotion();
        },
        20.0);
    CHECK(r == WaitResult::Satisfied);
    REQUIRE(fired);
    CHECK(a.state() == MotionState::Cancelled);
    CHECK(b.state() == MotionState::Settled);
    CHECK(posErr(s.rig.h.truePose(), kTargetB) < 0.6);  // b's absolute target won
    CHECK(s.sched.motionsStarted() == 2);
    CHECK(s.sched.motionsCancelled() == 1);
    CHECK(s.sched.motionsSettled() == 1);
}

// Bug caught: cancel-from-predicate being rejected (it is explicitly allowed —
// the "abort the routine on a sensor cue" idiom).
TEST_CASE("C2 re-entrancy: cancel from inside a predicate is allowed and safes the drive") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    MoveToPose m{s.sched.deps(), kTargetA, motionConfig(), 8.0};
    s.sched.async(m);
    REQUIRE(s.sched.waitUntil([&] { return s.rig.loc.pose().x().value() > 10.0; }, 10.0)
            == WaitResult::Satisfied);
    const WaitResult r = s.sched.waitUntil(
        [&] {
            s.sched.cancel();
            return !s.sched.hasActiveMotion();
        },
        5.0);
    CHECK(r == WaitResult::Satisfied);
    CHECK(m.state() == MotionState::Cancelled);
    checkSafeState(s.rig);
}

// Bug caught: nested blocking waits recursing instead of failing loudly — and
// a rejection that leaves the scheduler wedged (the flags must unwind).
TEST_CASE("C2 re-entrancy: blocking verbs from inside a predicate are rejected, and recoverably") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    CHECK_THROWS_AS((void)s.sched.waitUntil(
                        [&] {
                            (void)s.sched.waitUntilSettled();
                            return true;
                        },
                        1.0),
                    PreconditionError);
    CHECK_THROWS_AS((void)s.sched.waitUntil(
                        [&] {
                            (void)s.sched.waitUntil([] { return true; }, 0.0);
                            return true;
                        },
                        1.0),
                    PreconditionError);
    CHECK_THROWS_AS((void)s.sched.waitUntil(
                        [&] {
                            (void)s.sched.tick();
                            return true;
                        },
                        1.0),
                    PreconditionError);
    // The guards unwound (FlagScope): the scheduler still works.
    MoveToPose m{s.sched.deps(), Pose2d{Length{15.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    CHECK(s.run(m) == ExitReason::Settled);
}

// ═══ The task-boundary catch (check.hpp's promised conversion) ═════════════════════

// Bug caught: a mid-tick PreconditionError crashing the run OR leaving the
// mid-tick motor command energized — check.hpp: "caught by the motion
// scheduler at the task boundary and converted to a FAULT_ABORT exit + a safe
// drivetrain state. One bad reading degrades one motion; it never aborts the
// auton."
TEST_CASE("C2 task boundary: a PreconditionError mid-tick becomes a Precondition fault-abort") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    ThrowingMotion tm{s.sched.deps()};
    s.sched.async(tm);
    CHECK_FALSE(s.sched.tick());  // caught, converted, motion gone — no throw out
    checkSafeState(s.rig);        // the 5 V left by the throwing tick was safed
    CHECK(s.rig.latch.raiseCount(FaultCode::Precondition) == 1);  // raised exactly once
    CHECK(tm.state() == MotionState::Cancelled);
    CHECK(s.sched.lastCompleted().exit == ExitReason::Cancelled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::Precondition);
    CHECK(s.sched.motionsAborted() == 1);
    // The auton continues: a real motion still runs to settle.
    MoveToPose m{s.sched.deps(), Pose2d{Length{15.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    CHECK(s.run(m) == ExitReason::Settled);
}

// Bug caught: a motion commanding its own scheduler mid-tick being silently
// honored (slot mutation under the tick) instead of rejected-and-contained.
TEST_CASE("C2 task boundary: a motion calling the scheduler from tick() is rejected and contained") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    SchedulerCallingMotion sm{s.sched.deps(), s.sched};
    s.sched.async(sm);
    CHECK_FALSE(s.sched.tick());  // the precondition throw is converted at the boundary
    CHECK(sm.state() == MotionState::Cancelled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::Precondition);
    checkSafeState(s.rig);
}

// ═══ The fault policy ══════════════════════════════════════════════════════════════

// Bug caught: the headline policy failure — the scheduler continuing to servo
// against a LYING estimate for the full watchdog budget. The A/B twin (policy
// off) doubles as proof the abort is what bounds the damage.
TEST_CASE("C2 fault policy: ODO_STUCK aborts promptly; the policy-off twin rides to timeout") {
    const auto kin = xDrive(Length{7.0});
    const Pose2d target{Length{40.0}, Length{0.0}, Angle{}};

    // A: default policy (abort on ODO_STUCK).
    EncoderHostileConfig encA;
    encA.trackingFreezeAt = Time{1.0};
    encA.trackingFreezeIndex = -1;  // both tracking channels die
    EncoderHostileModel modelA{encA};
    SchedulerRig a{kin, plantConfig(), nullptr, &modelA};
    MoveToPose ma{a.sched.deps(), target, motionConfig(), 6.0};
    a.sched.async(ma);
    REQUIRE(a.sched.waitUntilSettled() == ExitReason::Cancelled);
    const double tA = a.rig.h.clock().now().value();
    const double damageA = posErr(a.rig.h.truePose(), a.rig.loc.pose());
    CHECK(a.sched.lastCompleted().abortFault == FaultCode::OdoStuck);
    CHECK(a.sched.motionsAborted() == 1);
    CHECK(tA < 2.0);  // freeze at 1.0 s + ~0.3 s window + margin — NOT the 6 s watchdog
    checkSafeState(a.rig);
    CHECK(ma.state() == MotionState::Cancelled);

    // B: policy disabled — C1's behaviour: full-speed runaway until the watchdog.
    EncoderHostileConfig encB = encA;
    EncoderHostileModel modelB{encB};
    MotionSchedulerConfig off;
    off.abortFaultMask = 0;
    SchedulerRig b{kin, plantConfig(), nullptr, &modelB, off};
    MoveToPose mb{b.sched.deps(), target, motionConfig(), 6.0};
    b.sched.async(mb);
    REQUIRE(b.sched.waitUntilSettled() == ExitReason::TimedOut);
    const double tB = b.rig.h.clock().now().value();
    const double damageB = posErr(b.rig.h.truePose(), b.rig.loc.pose());
    CHECK(tB > 5.9);  // rode the whole watchdog
    MESSAGE("ODO_STUCK: abort at ", tA, "s with ", damageA, "in truth-vs-estimate gap; ",
            "policy-off twin: ", tB, "s and ", damageB, "in");
    CHECK(damageA < damageB);        // the abort bounded the damage…
    CHECK(damageB > damageA + 3.0);  // …by a real margin, not noise
}

// Bug caught: an over-aggressive policy aborting on survivable faults — C1
// pinned that a mid-run IMU loss keeps driving (Degraded does not gate).
TEST_CASE("C2 fault policy: IMU_LOST mid-motion does NOT abort — the motion still settles") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};  // live from the start; isolate the dropout
    cfg.dropoutAt = Time{1.0};
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    MoveToPose m{s.sched.deps(), Pose2d{Length{30.0}, Length{8.0}, Angle::degrees(30.0)},
                 motionConfig(), 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);
    CHECK(s.rig.latch.raiseCount(FaultCode::ImuLost) == 1);  // seen, once
    CHECK(s.sched.motionsAborted() == 0);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::None);
}

// Bug caught: aborting on BROWNOUT — a power collapse is not a lying estimate;
// the motion's own watchdog is the bound, and the RUN must continue.
TEST_CASE("C2 fault policy: BROWNOUT does NOT abort — watchdog bounds it, the run continues") {
    PowerHostileModel model{PowerHostileConfig{}};
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.batteryVoltage = Voltage{10.8};  // nearly-dead pack (the C1 shape)
    SchedulerRig s{kin, pcfg, nullptr, &model};
    MoveToPose m{s.sched.deps(), Pose2d{Length{60.0}, Length{0.0}, Angle{}}, motionConfig(),
                 2.0};
    s.sched.async(m);
    CHECK(s.sched.waitUntilSettled() == ExitReason::TimedOut);  // not Cancelled
    CHECK(s.rig.latch.firstFault() == FaultCode::Brownout);
    CHECK(s.rig.health.brownedOut());
    CHECK(s.sched.motionsAborted() == 0);
    CHECK(s.sched.motionsTimedOut() == 1);
    CHECK(s.rig.h.clock().now().value() > 1.9);  // the watchdog, not the policy, ended it
}

// Bug caught: the snapshot semantics — a fault latched by an EARLIER motion
// (or pre-run) must not abort the next motion; a RE-raise during it must.
// This is exactly the hole a since-clear bitmask would have (found at design
// time; raiseCount exists because of it).
TEST_CASE("C2 fault policy: stale ODO_STUCK does not abort; a re-raise during the motion does") {
    const auto kin = xDrive(Length{7.0});
    EncoderHostileConfig enc;
    enc.trackingFreezeAt = Time{2.0};  // fires mid-CRUISE of motion 2 (motion 1 is a
    enc.trackingFreezeIndex = -1;      // short hop that settles well before 2.0 s)
    EncoderHostileModel model{enc};
    SchedulerRig s{kin, plantConfig(), nullptr, &model};

    // A stale latch entry from "before" (an earlier motion's episode):
    s.rig.latch.raise(FaultCode::OdoStuck, "TEST", "stale pre-existing episode");
    REQUIRE(s.rig.latch.raiseCount(FaultCode::OdoStuck) == 1);

    MoveToPose m1{s.sched.deps(), Pose2d{Length{6.0}, Length{0.0}, Angle{}}, motionConfig(),
                  8.0};
    REQUIRE(s.run(m1) == ExitReason::Settled);  // NOT insta-aborted by the stale latch
    CHECK(s.sched.motionsAborted() == 0);
    REQUIRE(s.rig.h.clock().now().value() < 2.0);  // the freeze has not happened yet

    // Motion 2 runs into the real freeze at full cruise: the RE-raise must abort.
    MoveToPose m2{s.sched.deps(), Pose2d{Length{-40.0}, Length{10.0}, Angle::degrees(60.0)},
                  motionConfig(), 8.0};
    s.sched.async(m2);
    REQUIRE(s.sched.waitUntilSettled() == ExitReason::Cancelled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::OdoStuck);
    CHECK(s.rig.latch.raiseCount(FaultCode::OdoStuck) == 2);
    checkSafeState(s.rig);
}

// Bug caught: the mask not actually being configurable (policy hardwired to
// ODO_STUCK) — a team choosing abort-on-brownout must get it.
TEST_CASE("C2 fault policy: the abort mask is honored — abort-on-BROWNOUT when configured") {
    PowerHostileModel model{PowerHostileConfig{}};
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.batteryVoltage = Voltage{10.8};
    MotionSchedulerConfig cfg;
    cfg.abortFaultMask = faultBit(FaultCode::OdoStuck) | faultBit(FaultCode::Brownout);
    SchedulerRig s{kin, pcfg, nullptr, &model, cfg};
    MoveToPose m{s.sched.deps(), Pose2d{Length{60.0}, Length{0.0}, Angle{}}, motionConfig(),
                 4.0};
    s.sched.async(m);
    REQUIRE(s.sched.waitUntilSettled() == ExitReason::Cancelled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::Brownout);
    CHECK(s.rig.h.clock().now().value() < 1.0);  // aborted promptly, not at the watchdog
    checkSafeState(s.rig);
}

// ═══ Monitor ownership between motions (C1 handoff #3) ═════════════════════════════

// Bug caught: the idle gap — a pathology arising BETWEEN motions (here a
// mid-idle IMU loss) going unreported because nothing ticks the HealthMonitor.
TEST_CASE("C2 idle gap: an IMU dropout between motions still raises IMU_LOST") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};
    cfg.dropoutAt = Time{2.5};  // AFTER motion 1 settles, DURING the idle wait
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    MoveToPose m{s.sched.deps(), Pose2d{Length{20.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);
    REQUIRE(s.rig.h.clock().now().value() < 2.4);  // the dropout has NOT happened yet
    REQUIRE(s.rig.latch.raiseCount(FaultCode::ImuLost) == 0);

    // Idle across the dropout: no active motion, only scheduler ticks.
    CHECK(s.sched.waitUntil([] { return false; }, 1.5) == WaitResult::TimedOut);
    CHECK_FALSE(s.sched.hasActiveMotion());
    CHECK(s.rig.latch.raiseCount(FaultCode::ImuLost) == 1);  // the gap is owned
}

// Bug caught: the scheduler-owned LoopMonitor being dead — hostile tick timing
// must surface as LOOP_OVERRUN with the exact spike count.
TEST_CASE("C2 loop monitor: jittered pacing is measured — overruns == slow ticks exactly") {
    struct JitterPacer final : shulib::motion::ITickPacer {
        JitterPacer(shulib::sim::SimHarness& harness, JitterSchedule& schedule)
            : h{&harness}, sched{&schedule} {}
        void pace() override {
            if (paces >= 5000) {
                throw std::runtime_error("JitterPacer: cap exceeded");
            }
            ++paces;
            const Time dt = (*sched)(paces);
            if (dt.value() >= 0.015) {  // the LoopMonitor default budget, inclusive
                ++slowTicks;
            }
            h->plant().step(dt);
        }
        shulib::sim::SimHarness* h;
        JitterSchedule* sched;
        int paces = 0;
        int slowTicks = 0;
    };

    FullHostility world{};
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.seed = 99;
    MotionRig rig{kin, pcfg, nullptr, &world.model()};
    JitterSchedule schedule{99};
    JitterPacer pacer{rig.h, schedule};
    MotionScheduler sched{rig.deps, pacer};
    MoveToPose m{sched.deps(), Pose2d{Length{24.0}, Length{10.0}, Angle::degrees(45.0)},
                 motionConfig(), 8.0};
    sched.async(m);
    REQUIRE(sched.waitUntilSettled() == ExitReason::Settled);
    REQUIRE(pacer.slowTicks > 0);  // the schedule genuinely spiked (else vacuous)
    CHECK(sched.loopMonitor().overrunCount() == pacer.slowTicks);
    CHECK(rig.latch.raiseCount(FaultCode::LoopOverrun) == pacer.slowTicks);
}

// ═══ Ids and records (C1 handoff #2 + A1 cost contract) ════════════════════════════

// Bug caught: activeCommandId never assigned (the cmd#0-forever defect the C1
// TermSink fix worked around), ids bleeding across motions, or idle records
// carrying a stale id.
TEST_CASE("C2 ids: every record of motion k carries id k; idle records carry id 0") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), &sink};

    // The record stream interleaves TWO producers: the motion's records
    // (state != 0, routed through the stamper) and the A2 plant's per-step
    // truth records (state 0, emitted UPSTREAM of the stamper — deliberately
    // never command-attributed). The audit discriminates on state.
    MoveToPose m1{s.sched.deps(), Pose2d{Length{15.0}, Length{0.0}, Angle{}}, motionConfig(),
                  8.0};
    REQUIRE(s.run(m1) == ExitReason::Settled);
    const int n1 = sink.recordCount();
    int motionRecords = 0;
    for (int i = 0; i < n1; ++i) {
        const auto& r = sink.recordAt(i);
        if (r.activeCommandState != 0) {
            ++motionRecords;
            CHECK(r.activeCommandId == 1);
        } else {
            CHECK(r.activeCommandId == 0);  // plant truth records stay unstamped
        }
    }
    CHECK(motionRecords > 10);
    CHECK(sink.lastRecord().activeCommandState
          == static_cast<std::uint8_t>(MotionState::Settled));  // the exit record ends
                                                                // the motion's segment
    for (int i = 0; i < 4; ++i) {  // idle ticks between motions
        (void)s.sched.tick();
        s.pacer.pace();
    }
    const int n2 = sink.recordCount();
    REQUIRE(n2 > n1);
    for (int i = n1; i < n2; ++i) {
        CHECK(sink.recordAt(i).activeCommandId == 0);  // idle: nothing to attribute
        CHECK(sink.recordAt(i).activeCommandState == 0);
    }

    TurnTo m2{s.sched.deps(), Angle::degrees(90.0), motionConfig(), 8.0};
    REQUIRE(s.run(m2) == ExitReason::Settled);
    int motion2Records = 0;
    for (int i = n2; i < sink.recordCount(); ++i) {
        const auto& r = sink.recordAt(i);
        if (r.activeCommandState != 0) {
            ++motion2Records;
            CHECK(r.activeCommandId == 2);  // ids advance per motion, never bleed
        } else {
            CHECK(r.activeCommandId == 0);
        }
    }
    CHECK(motion2Records > 10);
}

// Bug caught: the id never reaching the WIRE format — the §18.3 line must
// finally print a real cmd# (TermSink needed no change; the id had to arrive).
TEST_CASE("C2 ids: TermSink renders cmd#1 with the running state") {
    shulib::hal::fake::FakeClock termClock;
    shulib::hal::fake::FakeCharSink out;
    shulib::diag::TermSink term{termClock, out};
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), &term};
    MoveToPose m{s.sched.deps(), Pose2d{Length{15.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);
    CHECK(out.text().find("[MOT] cmd#1▸2") != std::string::npos);  // ▸ running ticks
    CHECK(out.text().find("[MOT] cmd#1▸3") != std::string::npos);  // ▸ the settle record
}

// Bug caught: the stamp breaking the A1 cost contract — with a records-off
// sink the record builder must never run and emit() must never be called
// (wantsRecord()/emit() overridden as a PAIR, forwarding).
TEST_CASE("C2 cost contract: a records-off sink stays records-off through the id stamp") {
    RecordsOffProbeSink probe;
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), &probe};
    MoveToPose m{s.sched.deps(), Pose2d{Length{15.0}, Length{0.0}, Angle{}}, motionConfig(),
                 8.0};
    REQUIRE(s.run(m) == ExitReason::Settled);
    for (int i = 0; i < 3; ++i) {  // idle path too
        (void)s.sched.tick();
        s.pacer.pace();
    }
    CHECK_FALSE(probe.emitCalled);
}

// ═══ Boot window through the scheduler ═════════════════════════════════════════════

// Bug caught: the scheduler loop violating the C1 wait-for-live contract — a
// scheduled motion must wait out calibration at zero volts, THEN drive.
TEST_CASE("C2 boot: a scheduled motion waits out IMU calibration, motionless, then lands") {
    ImuHostileModel imu{ImuHostileConfig{}};  // 2 s calibration + drift + noise
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin, plantConfig(), nullptr, &imu};
    const Pose2d target{Length{24.0}, Length{0.0}, Angle{}};
    MoveToPose m{s.sched.deps(), target, motionConfig(), 8.0};
    s.sched.async(m);
    // waitUntil as the boot marker: block until the motion goes live, checking
    // motionlessness DURING the wait (the pred observes Running only AFTER the
    // first live tick has legitimately commanded — so the origin pin must be
    // sampled while still WaitingForEstimate, not after).
    bool movedWhileWaiting = false;
    REQUIRE(s.sched.waitUntil(
                [&] {
                    if (m.state() == MotionState::WaitingForEstimate
                        && posErr(s.rig.h.truePose(), Pose2d{}) > 1e-9) {
                        movedWhileWaiting = true;
                    }
                    return m.state() == MotionState::Running;
                },
                6.0)
            == WaitResult::Satisfied);
    CHECK(s.rig.h.clock().now().value() > 1.9);  // it genuinely waited out calibration
    CHECK_FALSE(movedWhileWaiting);              // zero volts on every waiting tick
    CHECK(posErr(s.rig.h.truePose(), Pose2d{}) < 1.5);  // at most ~2 live ticks of travel
    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(posErr(s.rig.h.truePose(), target) < 1.5);
    CHECK_FALSE(s.rig.latch.hasFault());  // boot is normal, not a fault
}

// ═══ Determinism + loop-shape equivalence ══════════════════════════════════════════

// Bug caught: the formalized loop deviating from the C1 hand loop (an extra,
// dropped, or reordered localizer update / plant step) — the outcome must be
// BIT-IDENTICAL, which is the strongest available statement that the scheduler
// added engine, not physics.
TEST_CASE("C2 equivalence: a scheduled motion is bit-identical to the C1 hand loop") {
    const auto kin = xDrive(Length{7.0});
    const Pose2d target{Length{26.0}, Length{-14.0}, Angle::degrees(75.0)};
    auto pcfg = plantConfig();
    pcfg.plant.seed = 11;

    FullHostility worldA{};
    MotionRig a{kin, pcfg, nullptr, &worldA.model()};
    MoveToPose ma{a.deps, target, motionConfig(), 8.0};
    REQUIRE(a.run(ma, 1600) == ExitReason::Settled);

    FullHostility worldB{};
    SchedulerRig b{kin, pcfg, nullptr, &worldB.model()};
    MoveToPose mb{b.sched.deps(), target, motionConfig(), 8.0};
    REQUIRE(b.run(mb) == ExitReason::Settled);

    CHECK(a.h.clock().now().value() == b.rig.h.clock().now().value());
    CHECK(a.h.truePose().x().value() == b.rig.h.truePose().x().value());
    CHECK(a.h.truePose().y().value() == b.rig.h.truePose().y().value());
    CHECK(a.h.truePose().heading().radians() == b.rig.h.truePose().heading().radians());
    CHECK(a.loc.pose().x().value() == b.rig.loc.pose().x().value());
    CHECK(a.loc.pose().y().value() == b.rig.loc.pose().y().value());
}

// ═══ The stalled-pacer guard ═══════════════════════════════════════════════════════

// Bug caught: the hang no watchdog can catch — a pacer that never advances the
// clock freezes every deadline; the scheduler must fail LOUDLY instead of
// spinning (the FrozenPacer's own cap keeps the guard-removed mutation red).
TEST_CASE("C2 stalled pacer: a clock that never advances trips the precondition, never hangs") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    FrozenPacer frozen;
    MotionScheduler sched{rig.deps, frozen};
    CHECK_THROWS_AS((void)sched.waitUntil([] { return false; }, 5.0), PreconditionError);
    CHECK(frozen.calls <= MotionScheduler::kMaxStalledPaces);
    CHECK(frozen.calls >= MotionScheduler::kMaxStalledPaces - 1);
}
