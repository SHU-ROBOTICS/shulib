// F2 SCHEDULER SUITE — the two C2-layer properties this chunk added or pinned
// (motion_scheduler.hpp banner: "Re-entrancy" + "Unwind safety"). Every case
// names the bug it would catch.
//
// The trap, in this suite's form: the safety claims are asserted at the
// DEVICE (motor voltage AND brake mode) and on the scheduler's own state —
// never against what the pacer or the predicate believes it did.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "motion_test_rig.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/units/quantity.hpp"

using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::StallingPacer;
using motion_rig::motionConfig;
using shulib::PreconditionError;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::IMotor;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::ITickPacer;
using shulib::motion::MotionScheduler;
using shulib::motion::MoveToPose;
using shulib::motion::WaitResult;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Drive motors safe at the DEVICE: 0 V AND Brake, on every motor.
void checkDriveSafe(MotionRig& rig) {
    for (const IMotor* m : rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 0.0);
        CHECK(m->brakeMode() == BrakeMode::Brake);
    }
}

/// A pacer that cancels the scheduler at a chosen pace — the F2 RunGuard's
/// exact position (cancel from inside pace(), between ticks), reduced to its
/// minimal form so the pin is about C2's precondition set, not about F2.
struct CancellingPacer final : ITickPacer {
    CancellingPacer(PlantPacer& inner, MotionScheduler*& sched, int cancelAt)
        : inner_{&inner}, sched_{&sched}, at_{cancelAt} {}

    void pace() override {
        ++calls;
        if (calls == at_ && *sched_ != nullptr) {
            (*sched_)->cancel();  // the pinned re-entrancy position
        }
        inner_->pace();
    }

    PlantPacer* inner_;
    MotionScheduler** sched_;
    int at_;
    int calls = 0;
};

}  // namespace

// ── the pinned pacer position (C2 re-entrancy list, F2 addition) ────────────────────

// Bug caught: a later chunk "tightening" cancel()'s preconditions to forbid
// the pacer position (e.g. adding !inWait_) — which would break the F2
// end-of-run guard's deadline cut. This pin makes the reliance a red build,
// not an archaeology exercise. Also pins the PAYOFF: the cancel unwinds
// waitUntilSettled on the SAME iteration (zero further ticks), returning the
// motion's honest Cancelled verdict.
TEST_CASE("C2/F2 pin: cancel() from inside pace() is legal and unwinds "
          "waitUntilSettled immediately") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};
    MotionScheduler* schedPtr = nullptr;
    CancellingPacer pacer{plant, schedPtr, /*cancelAt=*/25};
    MotionScheduler sched{rig.deps, pacer};
    schedPtr = &sched;

    MoveToPose far{sched.deps(), Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                   motionConfig(), /*timeout=*/60.0};
    sched.async(far);
    const ExitReason reason = sched.waitUntilSettled();  // must NOT throw

    CHECK(reason == ExitReason::Cancelled);   // the honest verdict, not Settled
    CHECK_FALSE(sched.hasActiveMotion());
    CHECK(pacer.calls == 25);                 // unwound on the SAME iteration:
                                              // not one further pace happened
    checkDriveSafe(rig);
}

// ── unwind safety of the blocking waits (the C4 gap, closed at its root) ────────────

// Bug caught: MEASUREMENT 15's state, the worst found in the campaign —
// a throw through a wait leaving the active motion armed and the motors at
// their last command (11.4 V under Coast) with the slot pointing at a dying
// stack object. C4's DetachGuard patched the facade verbs only; direct
// Tier-3 waits had the hole until F2. waitUntil is the acute case: the
// mechanism idiom puts user predicates inside it, and user code throws.
TEST_CASE("F2 unwind: a throwing predicate inside waitUntil cancels the active "
          "motion before propagating") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};
    MotionScheduler sched{rig.deps, plant};

    MoveToPose far{sched.deps(), Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                   motionConfig(), /*timeout=*/60.0};
    sched.async(far);
    int ticks = 0;
    CHECK_THROWS_AS((void)sched.waitUntil(
                        [&] {
                            if (++ticks == 30) {
                                throw std::runtime_error("user predicate bug");
                            }
                            return false;
                        },
                        60.0),
                    std::runtime_error);

    CHECK_FALSE(sched.hasActiveMotion());          // slot cleared, nothing dangles
    CHECK(sched.lastExitReason() == ExitReason::Cancelled);
    checkDriveSafe(rig);                            // NOT 11.4 V under Coast
    // The boundary was RECORDED, not skipped: accounting stays truthful.
    CHECK(sched.motionsCancelled() == 1);
}

// Bug caught: the same hole in waitUntilSettled for a DIRECT Tier-3 caller —
// the facade's DetachGuard cannot cover a wait it never wraps. A stalled
// pacer mid-motion must leave the scheduler detached and the drive safe.
TEST_CASE("F2 unwind: a stalled pacer mid-waitUntilSettled (no facade) detaches "
          "and safes") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    StallingPacer pacer{rig.h, /*healthy=*/50};
    MotionScheduler sched{rig.deps, pacer};

    MoveToPose far{sched.deps(), Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                   motionConfig(), /*timeout=*/60.0};
    sched.async(far);
    CHECK_THROWS_AS((void)sched.waitUntilSettled(), PreconditionError);

    CHECK_FALSE(sched.hasActiveMotion());
    CHECK(sched.lastExitReason() == ExitReason::Cancelled);
    checkDriveSafe(rig);
}

// Bug caught: the unwind guard OVERREACHING — cancelling on the normal return
// path. waitUntil legitimately returns with a motion still active (that is
// its whole point: the marker primitive ticks the motion through the wait);
// a guard that cancelled on every exit would kill the motion it protects.
TEST_CASE("F2 unwind: normal waitUntil exits leave the active motion running") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};
    MotionScheduler sched{rig.deps, plant};

    MoveToPose far{sched.deps(), Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                   motionConfig(), /*timeout=*/60.0};
    sched.async(far);
    int ticks = 0;
    const WaitResult w = sched.waitUntil([&] { return ++ticks >= 20; }, 60.0);
    CHECK(w == WaitResult::Satisfied);
    CHECK(sched.hasActiveMotion());  // still alive: the wait did not murder it
    sched.cancel();                  // cleanup (the stack motion dies at scope end)
}

// ── the remaining unwind path: the DESTRUCTOR (DEFECTS1, item A28) ──────────────────

// Bug caught: `~MotionScheduler() = default`. F2 closed the throw-through-a-wait hole
// with WaitUnwindGuard, but the destructor was the SAME hole reached by a path that
// guard cannot see — `sched.async(m);` and then simply returning, or a throw out of a
// hand-rolled non-blocking loop. The scheduler died with active_ != nullptr, the safe
// state was never commanded, and the drive held its last voltage with nothing logged,
// no boundary recorded, and completedCount() never accounting for the motion.
//
// The negative control is what makes the zero mean anything: this case FIRST measures
// that the motors really are energized at the moment of destruction, so a passing
// assertion cannot come from a scenario that never commanded anything.
TEST_CASE("A28: destroying a scheduler with a motion ARMED leaves the drive safe") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};

    double energizedVolts = 0.0;
    {
        MotionScheduler sched{rig.deps, plant};
        MoveToPose far{sched.deps(), Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                       motionConfig(), /*timeout=*/60.0};
        sched.async(far);
        for (int i = 0; i < 20; ++i) {  // let the pipeline actually command volts
            (void)sched.tick();
            plant.pace();
        }

        // NEGATIVE CONTROL: the drive must be genuinely energized right now, or the
        // post-destruction check below proves nothing.
        energizedVolts = std::abs(rig.h.context().driveMotors()[0]->commandedVoltage().value());
        REQUIRE(energizedVolts > 1.0);
        REQUIRE(sched.hasActiveMotion());
        CHECK(sched.completedCount() == 0);
    }  // <-- destroyed with the motion still armed

    checkDriveSafe(rig);  // 0 V AND Brake, on every motor
}

// Bug caught: a destructor that cancels UNCONDITIONALLY. cancel() with no active motion
// is the documented PANIC STOP — it commands the safe state anyway — which is right for
// an explicit call and wrong for destruction: tearing down an idle scheduler must not
// reach out and brake a drivetrain the caller may still be driving through another
// object. This pins that an idle scheduler's destructor touches no motor at all.
TEST_CASE("A28: destroying an IDLE scheduler commands nothing") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};

    // Park the drive somewhere recognisable that the safe state would overwrite.
    for (IMotor* m : rig.h.context().driveMotors()) {
        m->setBrakeMode(BrakeMode::Coast);
        m->setVoltage(shulib::units::Voltage{3.0});
    }
    {
        MotionScheduler sched{rig.deps, plant};
        CHECK_FALSE(sched.hasActiveMotion());
    }  // <-- destroyed idle

    for (const IMotor* m : rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 3.0);   // untouched
        CHECK(m->brakeMode() == BrakeMode::Coast);     // untouched
    }
}

// Bug caught (DEFECTS1 item A27): MotionStatsSink::beginMotion() reset the scalar
// aggregates but not target_/startPose_, so the PUBLIC targetPose() served the previous
// motion's target between motions — not a default Pose2d, and not detectable as stale
// from the value. Only the scheduler's own hasData() guard hid it from the run summary;
// a direct reader of the sink — which the generated reference documents and invites —
// got motion 1's target presented as motion 2's. Driven against the public class rather
// than through the scheduler, because the scheduler's guard is exactly what hides it.
TEST_CASE("A27: beginMotion() clears the target — a new motion cannot inherit the old one") {
    shulib::hal::fake::FakeTelemetrySink inner;
    shulib::motion::MotionStatsSink stats{inner};

    shulib::diag::DebugRecord r{};
    r.activeCommandId = 1;
    r.activeCommandState = static_cast<std::uint8_t>(shulib::motion::MotionState::Running);
    r.measuredPose = Pose2d{Length{0.0}, Length{0.0}, Angle{}};
    r.targetPose = Pose2d{Length{48.0}, Length{24.0}, Angle{}};

    stats.beginMotion();
    stats.emit(r);
    // NEGATIVE CONTROL: motion 1 really did publish a target, or the check below is vacuous.
    REQUIRE(stats.hasData());
    REQUIRE(stats.targetPose().x().value() == doctest::Approx(48.0));
    REQUIRE(stats.targetPose().y().value() == doctest::Approx(24.0));

    stats.beginMotion();  // motion 2 armed; no live tick yet

    CHECK_FALSE(stats.hasData());
    CHECK(stats.targetPose().x().value() == 0.0);  // was 48.0 — motion 1's target
    CHECK(stats.targetPose().y().value() == 0.0);  // was 24.0
    CHECK(stats.overshoot().value() == 0.0);       // these always reset; pinned as the
    CHECK(stats.drift().value() == 0.0);           // invariant the poses now share
}
