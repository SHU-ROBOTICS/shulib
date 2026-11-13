// Adversarial tests for the drivetrain degradation policy (chunk R3b Part 0b, brief §4 test 1).
//
// The policy decides whether a drive program with dead motors drives, drives degraded, or
// refuses. Every expected row below is the brief's own table plus the exact boundary rows
// on either side of each limit — a policy whose boundary was never pinned is a policy that
// silently moves. The two mutations the brief requires (`< 3` → `< 2`; total `> 2` → `> 3`)
// were run against these cases and the observed reds are in the R3b log.

#include "doctest.h"

#include <string>

#include "shulib/teleop/drivetrain_degradation.hpp"

using shulib::teleop::DegradationVerdict;
using shulib::teleop::DriveVerdict;
using shulib::teleop::evaluateDegradation;
using shulib::teleop::evaluateRuntimeDegradation;
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

// ═══ T4 Part 2 — the MID-RUN rule (the team lead's N7 (c), 2026-09-27: "Only when 3+ are dead in
// total") ════════════════════════════════════════════════════════════════════════════════
namespace {
[[nodiscard]] DegradationVerdict runtime(int lExp, int lAns, int rExp, int rAns) {
    return evaluateRuntimeDegradation(SideCount{lExp, lAns}, SideCount{rExp, rAns});
}
}  // namespace

static_assert(evaluateRuntimeDegradation(SideCount{4, 4}, SideCount{4, 4}).verdict
              == DriveVerdict::Drive);

TEST_CASE("T4 runtime degradation: robot two's four a side — 2/4 on ONE side DRIVES (degraded) "
          "mid-run, where the boot rule refuses it; the third dead anywhere REFUSES") {
    // BUG CAUGHT (the brief's mutation "the per-side floor applied mid-run under N7 (c)"): the
    // second death on one side of robot two stopping the robot for the rest of the match — against
    // the team lead's words ("1-2 dead ports shouldn't stop driving"). And the cap off by one.
    CHECK(runtime(4, 4, 4, 4).verdict == DriveVerdict::Drive);
    CHECK(runtime(4, 3, 4, 4).verdict == DriveVerdict::DriveDegraded);
    CHECK(runtime(4, 2, 4, 4).verdict == DriveVerdict::DriveDegraded);  // the boot rule's REFUSE
    CHECK(eval(4, 2, 4, 4).verdict == DriveVerdict::Refuse);             // …unchanged at boot
    CHECK(runtime(4, 4, 4, 2).verdict == DriveVerdict::DriveDegraded);
    CHECK(runtime(4, 3, 4, 3).verdict == DriveVerdict::DriveDegraded);  // 1 + 1
    CHECK(runtime(4, 2, 4, 3).verdict == DriveVerdict::Refuse);         // 2 + 1 = 3
    CHECK(runtime(4, 3, 4, 2).verdict == DriveVerdict::Refuse);         // 1 + 2 = 3
    CHECK(runtime(4, 1, 4, 4).verdict == DriveVerdict::Refuse);         // 3 on one side
    const DegradationVerdict two = runtime(4, 2, 4, 4);
    CHECK(two.leftDead == 2);
    CHECK(two.rightDead == 0);
    CHECK(two.totalDead == 2);
    CHECK(two.reason[0] != '\0');
    CHECK(runtime(4, 4, 4, 4).reason[0] == '\0');
    const DegradationVerdict three = runtime(4, 3, 4, 2);
    CHECK(three.totalDead == 3);
    CHECK(std::string{three.reason} == "REFUSE: more than 2 motors dead in total");
    // Five a side (the typed table's shape): the same cap.
    CHECK(runtime(5, 3, 5, 5).verdict == DriveVerdict::DriveDegraded);
    CHECK(runtime(5, 2, 5, 5).verdict == DriveVerdict::Refuse);
}

TEST_CASE("T4 runtime degradation: an EMPTY side is legal mid-run (only the total is judged); a "
          "negative count or more answering than listed still REFUSES") {
    // BUG CAUGHT: a one-group chassis refused at every entry (the boot rule's "empty side" check
    // copied here), or a census that cannot be right driving anyway.
    CHECK(runtime(4, 4, 0, 0).verdict == DriveVerdict::Drive);
    CHECK(runtime(4, 2, 0, 0).verdict == DriveVerdict::DriveDegraded);
    CHECK(runtime(4, 1, 0, 0).verdict == DriveVerdict::Refuse);
    CHECK(runtime(0, 0, 0, 0).verdict == DriveVerdict::Drive);
    CHECK(runtime(4, 5, 4, 4).verdict == DriveVerdict::Refuse);
    CHECK(runtime(4, -1, 4, 4).verdict == DriveVerdict::Refuse);
    CHECK(runtime(-1, 0, 4, 4).verdict == DriveVerdict::Refuse);
    CHECK(runtime(4, 4, 4, 5).reason[0] != '\0');
    // The RIGHT side's clauses, each by its VERDICT and its REASON (the review of 2026-09-30, D2 /
    // D3: a non-empty reason alone is also what a DEGRADED verdict gives, and a nonsense census that
    // happens to exceed the cap refuses for the wrong reason — so neither check could fail).
    const std::string nonsense = "REFUSE: motor counts make no sense";
    for (const DegradationVerdict v : {runtime(4, 4, 4, 5), runtime(4, 4, 4, -1), runtime(4, 4, -1, -1),
                                       runtime(4, 4, -1, 0), runtime(4, 5, 4, 4), runtime(-1, -1, 4, 4)}) {
        CAPTURE(v.reason);
        CHECK(v.verdict == DriveVerdict::Refuse);
        CHECK(std::string{v.reason}.rfind(nonsense, 0) == 0);
    }
}
