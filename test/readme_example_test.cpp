// README EXAMPLE (chunk D3) — the code block on the project's front page,
// compiled and run.
//
// Bug this file catches: the root README's "what using it looks like" example
// silently going stale. Every other code example in this project's public
// documentation has been quoted verbatim from a compiled test since C8, but the
// README's was not — it was swept BY HAND at D2's retype, which is exactly the
// failure mode the verbatim rule exists to remove. The first thing a stranger
// reads about shulib was the one example nothing checked.
//
// The README quotes the block below VERBATIM, includes and all, which is why
// the `#include` and `using` lines appear mid-file: they are part of the
// example, and moving them would make the README's listing a paraphrase.
//
// (D3 also broadened the drift scan so this file is found automatically: the
// scan reads every test/*example*_test.cpp, so a future examples file is
// covered the moment it exists.)

#include "doctest.h"

#include "motion_test_rig.hpp"
#include "shulib/kinematics/x_drive.hpp"

// ═══ README, "What using it looks like" — quoted verbatim from here ════════════════

#include "shulib/chassis/chassis.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib;
using namespace shulib::units::literals;

control::ExitReason firstScore(chassis::Chassis& chassis) {
    // Tell the localizer where the robot starts (heading comes from the IMU).
    chassis.setPose(math::Pose2d{-48_in, -24_in, 90_deg});

    // Drive to a field position AND rotate to a heading, simultaneously —
    // translation and rotation are independent on a holonomic drive.
    chassis.moveTo(math::Pose2d{-24_in, 0_in, 45_deg}, {.timeout = 3_s});

    // A slow, precise approach: per-call options cap this leg's speed.
    chassis.moveTo(math::Pose2d{-12_in, 12_in, 45_deg},
                   {.timeout = 2_s, .maxLinearSpeed = units::Velocity{20.0}});

    // Sideways to the goal, actively holding the current heading.
    chassis.strafeTo(-12_in, 24_in, {.timeout = 2_s});

    // Face the corner — always the short way around.
    const control::ExitReason last = chassis.turnTo(135_deg, {.timeout = 1.5_s});
    return last;  // Settled, TimedOut, or Cancelled — never a lie, never a hang
}

// ═══ end of the quoted block ══════════════════════════════════════════════════════

// Bug caught: the README example compiling but not WORKING — timeouts too tight
// for the legs they cover, or a verb that no longer reaches its target. The
// README claims "each verb blocks until the robot settles"; this grades that
// claim on the simulator's ground truth, which the estimator cannot see.
TEST_CASE("readme-a: the front-page example runs, and ends where it says it does") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = math::Pose2d{-48_in, -24_in, 90_deg};
    motion_rig::ChassisRig c{kin, simCfg};

    const control::ExitReason last = firstScore(c.chassis);

    CHECK(last == control::ExitReason::Settled);
    const math::Pose2d goal{-12_in, 24_in, 135_deg};
    CHECK(motion_rig::posErr(c.rig.h.truePose(), goal) < 1.0);
    CHECK(motion_rig::headErr(c.rig.h.truePose(), goal) < 0.035);
    // Four motions, no more: the example is four verbs and nothing hidden.
    CHECK(c.chassis.scheduler().motionsStarted() == 4);
}
