// F2 SCHEDULER SUITE — the two C2-layer properties this chunk added or pinned
// (motion_scheduler.hpp banner: "Re-entrancy" + "Unwind safety"). Every case
// names the bug it would catch.
//
// The trap, in this suite's form: the safety claims are asserted at the
// DEVICE (motor voltage AND brake mode) and on the scheduler's own state —
// never against what the pacer or the predicate believes it did.

#include "doctest.h"

#include <stdexcept>

#include "motion_test_rig.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
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
