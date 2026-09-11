// Adversarial tests for the drivetrain degradation policy (chunk R3b Part 0b, brief §4 test 1).
//
// The policy decides whether a drive program with dead motors drives, drives degraded, or
// refuses. Every expected row below is the brief's own table plus the exact boundary rows
// on either side of each limit — a policy whose boundary was never pinned is a policy that
// silently moves. The two mutations the brief requires (`< 3` → `< 2`; total `> 2` → `> 3`)
// were run against these cases and the observed reds are in the R3b log.

#include "doctest.h"

#include "shulib/teleop/drivetrain_degradation.hpp"

using shulib::teleop::DegradationVerdict;
using shulib::teleop::DriveVerdict;
using shulib::teleop::evaluateDegradation;
using shulib::teleop::kMaxDeadTotal;
using shulib::teleop::kMinAnsweringPerSide;
using shulib::teleop::SideCount;

namespace {
[[nodiscard]] DegradationVerdict eval(int lExp, int lAns, int rExp, int rAns) {
    return evaluateDegradation(SideCount{lExp, lAns}, SideCount{rExp, rAns});
}
}  // namespace

// The policy is constexpr: a verdict evaluable at compile time has no hidden state. The
// limits are pinned as literals (the brief's decided numbers), not as "whatever the header
// says".
static_assert(kMinAnsweringPerSide == 3);
static_assert(kMaxDeadTotal == 2);
static_assert(evaluateDegradation(SideCount{5, 5}, SideCount{5, 5}).verdict == DriveVerdict::Drive);

TEST_CASE("degradation: the brief's rows — 5/5+5/5 DRIVE; 4/5+5/5, 4/5+4/5, 3/5+5/5 DEGRADED; "
          "2/5+5/5 and 3-dead REFUSE") {
    // BUG CAUGHT: a side with two motors dragged into service (the robot curves at speed),
    // or a healthy robot refused (a match lost to a policy that was too strict).
    const DegradationVerdict all = eval(5, 5, 5, 5);
    CHECK(all.verdict == DriveVerdict::Drive);
    CHECK(all.totalDead == 0);
    CHECK(all.leftDead == 0);
    CHECK(all.rightDead == 0);

    const DegradationVerdict oneLeft = eval(5, 4, 5, 5);
    CHECK(oneLeft.verdict == DriveVerdict::DriveDegraded);
    CHECK(oneLeft.leftDead == 1);
    CHECK(oneLeft.rightDead == 0);
    CHECK(oneLeft.totalDead == 1);

    const DegradationVerdict oneEach = eval(5, 4, 5, 4);
    CHECK(oneEach.verdict == DriveVerdict::DriveDegraded);
    CHECK(oneEach.totalDead == 2);

    const DegradationVerdict twoLeft = eval(5, 3, 5, 5);
    CHECK(twoLeft.verdict == DriveVerdict::DriveDegraded);
    CHECK(twoLeft.leftDead == 2);
    CHECK(twoLeft.totalDead == 2);

    const DegradationVerdict threeLeftDead = eval(5, 2, 5, 5);
    CHECK(threeLeftDead.verdict == DriveVerdict::Refuse);
    CHECK(threeLeftDead.leftDead == 3);

    // 4/5 + 4/5 + one more dead = 3 total: the cap, not the floor, refuses this one
    // (both sides are still at or above three answering).
    const DegradationVerdict threeTotal = eval(5, 3, 5, 4);
    CHECK(threeTotal.verdict == DriveVerdict::Refuse);
    CHECK(threeTotal.totalDead == 3);
    CHECK(threeTotal.leftDead == 2);
    CHECK(threeTotal.rightDead == 1);
    // ... and the mirror, so the cap is not a left-side-only check.
    const DegradationVerdict threeTotalMirror = eval(5, 4, 5, 3);
    CHECK(threeTotalMirror.verdict == DriveVerdict::Refuse);
    CHECK(threeTotalMirror.totalDead == 3);
}

TEST_CASE("degradation: the EXACT boundary rows on the per-side floor, both sides") {
    // BUG CAUGHT: an off-by-one on the floor (`< 3` written as `<= 3` or `< 2`). Three
    // answering is the last row that drives; two is the first that refuses — on EITHER side.
    CHECK(eval(5, 3, 5, 5).verdict == DriveVerdict::DriveDegraded);
    CHECK(eval(5, 2, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, 5, 5, 3).verdict == DriveVerdict::DriveDegraded);
    CHECK(eval(5, 5, 5, 2).verdict == DriveVerdict::Refuse);
    // The bench bot's four-per-side: 3/4 drives, 2/4 refuses.
    CHECK(eval(4, 3, 4, 4).verdict == DriveVerdict::DriveDegraded);
    CHECK(eval(4, 2, 4, 4).verdict == DriveVerdict::Refuse);
    CHECK(eval(4, 4, 4, 3).verdict == DriveVerdict::DriveDegraded);
    CHECK(eval(4, 4, 4, 2).verdict == DriveVerdict::Refuse);
    // Zero answering on a side is the extreme of the same rule.
    CHECK(eval(5, 0, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, 5, 5, 0).verdict == DriveVerdict::Refuse);
}

TEST_CASE("degradation: the EXACT boundary rows on the total cap") {
    // BUG CAUGHT: an off-by-one on the cap (`> 2` written as `>= 2` or `> 3`). Two dead in
    // total drives (however split); three refuses even when every side is at the floor or
    // above — which only a total check can see.
    CHECK(eval(5, 4, 5, 4).verdict == DriveVerdict::DriveDegraded);  // 1 + 1 = 2
    CHECK(eval(5, 3, 5, 5).verdict == DriveVerdict::DriveDegraded);  // 2 + 0 = 2
    CHECK(eval(5, 5, 5, 3).verdict == DriveVerdict::DriveDegraded);  // 0 + 2 = 2
    CHECK(eval(5, 3, 5, 4).verdict == DriveVerdict::Refuse);         // 2 + 1 = 3, sides fine
    CHECK(eval(5, 4, 5, 3).verdict == DriveVerdict::Refuse);         // 1 + 2 = 3, sides fine
    // A bigger side makes the cap the ONLY thing refusing: 6-per-side, 3 dead on one side
    // leaves 3 answering (floor passes) — the cap must still refuse.
    CHECK(eval(6, 3, 6, 6).verdict == DriveVerdict::Refuse);
    CHECK(eval(6, 4, 6, 6).verdict == DriveVerdict::DriveDegraded);  // 2 dead: drives
}

TEST_CASE("degradation: the verdict carries a reason, and Drive carries none") {
    // BUG CAUGHT: a refusal painted on the panel with an empty reason — the driver would
    // see 0 V and nothing else, which is the silent failure this policy exists to prevent.
    CHECK(eval(5, 5, 5, 5).reason[0] == '\0');
    CHECK(eval(5, 4, 5, 5).reason[0] != '\0');
    CHECK(eval(5, 2, 5, 5).reason[0] != '\0');
    CHECK(eval(5, 3, 5, 4).reason[0] != '\0');
}

TEST_CASE("degradation: nonsense counts REFUSE rather than drive") {
    // BUG CAUGHT: a caller bug (answering > expected, an empty side, a negative) turning into
    // "drive anyway". 0 V is the only safe answer to a census that cannot be right.
    CHECK(eval(0, 0, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, 5, 0, 0).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, 6, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, 5, 5, 6).verdict == DriveVerdict::Refuse);
    CHECK(eval(5, -1, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(-5, -5, 5, 5).verdict == DriveVerdict::Refuse);
    CHECK(eval(0, 0, 5, 5).reason[0] != '\0');
}
