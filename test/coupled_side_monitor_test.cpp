// Adversarial tests for the coupled-side monitor, the shared per-side arcade arithmetic and
// the runtime dead-port detector (chunk R3b Part 0b, brief §4 tests 2 and 3).
//
// The monitor is the tester's gate 5 EXTRACTED (it ran on hardware, R3b-PROGRESS Session 2
// §10.4, though no motor ever turned under it). Each case names the bug it catches. The
// mutations the brief requires — drop the persistence; drop the present-flag check; flip the
// sign test; swap ∓ in the side arithmetic — were run against these cases and the observed
// reds are in the R3b log.

#include "doctest.h"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <span>

#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/stick_mapping.hpp"

using shulib::teleop::CoupledSideMonitor;
using shulib::teleop::DriveRequest;
using shulib::teleop::kMaxSideMembers;
using shulib::teleop::MemberAbsenceDetector;
using shulib::teleop::MemberSample;
using shulib::teleop::SideMonitorConfig;
using shulib::teleop::SideVerdict;
using shulib::teleop::SideVolts;
using shulib::teleop::tankSideVolts;

namespace {

constexpr std::size_t kN = 5;  // robot two: five coupled members per side

/// Five members all turning at `v` rad/s, present.
[[nodiscard]] std::array<MemberSample, kN> allAt(double v) {
    std::array<MemberSample, kN> m{};
    for (MemberSample& s : m) s = MemberSample{v, true};
    return m;
}

/// Run `ticks` updates of the same sample; return the last verdict.
[[nodiscard]] SideVerdict run(CoupledSideMonitor& mon, double cmd,
                              const std::array<MemberSample, kN>& m, int ticks) {
    SideVerdict v{};
    for (int i = 0; i < ticks; ++i) v = mon.update(cmd, std::span<const MemberSample>{m});
    return v;
}

}  // namespace

// The defaults are the station's INVENTED thresholds, pinned as literals so a drift in the
// header is a failing test with the number in it, not a silent re-tune.
static_assert(SideMonitorConfig{}.commandFloorV == 1.0);
static_assert(SideMonitorConfig{}.movingFloorRadS == 1.0);
static_assert(SideMonitorConfig{}.oppositeFloorRadS == 0.5);
static_assert(SideMonitorConfig{}.nearZeroFraction == 0.25);
static_assert(SideMonitorConfig{}.persistTicks == 25);
static_assert(kMaxSideMembers == 21);

TEST_CASE("side monitor: all members agree with the command -> nothing flagged, ever") {
    // BUG CAUGHT: a false cut on a healthy side — the drive would cut every second on a
    // perfectly good robot.
    CoupledSideMonitor mon;
    const auto m = allAt(8.0);
    const SideVerdict v = run(mon, 6.0, m, 500);
    CHECK(v.evaluated);
    CHECK(v.presentCount == 5);
    CHECK(v.fastestRadS == 8.0);
    CHECK(v.disagreeingMask == 0);
    CHECK(v.persistedMask == 0);
    CHECK(v.persistedCount == 0);
    for (std::size_t i = 0; i < kN; ++i) CHECK(mon.disagreeTicks(i) == 0);
    // Same the other way: a negative command with negative velocities.
    CoupledSideMonitor back;
    const SideVerdict b = run(back, -6.0, allAt(-8.0), 500);
    CHECK(b.evaluated);
    CHECK(b.persistedMask == 0);
    CHECK(b.disagreeingMask == 0);
}

TEST_CASE("side monitor: ONE member opposite in sign -> THAT member, on exactly the 25th tick, "
          "not the 24th") {
    // BUG CAUGHT: the persistence window dropped (cuts on the first transient tick) or
    // off by one; or the wrong member named (bit position drift).
    CoupledSideMonitor mon;
    auto m = allAt(8.0);
    m[2].velocityRadS = -8.0;  // member 2 driven backwards against its mates
    const SideVerdict at24 = run(mon, 6.0, m, 24);
    CHECK(at24.evaluated);
    CHECK(at24.disagreeingMask == (std::uint32_t{1} << 2));
    CHECK(at24.persistedMask == 0);
    CHECK(at24.persistedCount == 0);
    CHECK(mon.disagreeTicks(2) == 24);
    const SideVerdict at25 = run(mon, 6.0, m, 1);
    CHECK(at25.persistedMask == (std::uint32_t{1} << 2));
    CHECK(at25.persistedCount == 1);
    CHECK(mon.disagreeTicks(2) == 25);
    // The others stayed clean throughout.
    for (std::size_t i : {std::size_t{0}, std::size_t{1}, std::size_t{3}, std::size_t{4}}) {
        CHECK(mon.disagreeTicks(i) == 0);
    }
    // And it stays persisted while the fight continues (the caller's cut policy decides
    // what happens next; the monitor keeps reporting).
    const SideVerdict at60 = run(mon, 6.0, m, 35);
    CHECK(at60.persistedMask == (std::uint32_t{1} << 2));
    // Mirror: a negative command with one member turning positive.
    CoupledSideMonitor back;
    auto mb = allAt(-8.0);
    mb[4].velocityRadS = +8.0;
    CHECK(run(back, -6.0, mb, 24).persistedMask == 0);
    CHECK(run(back, -6.0, mb, 1).persistedMask == (std::uint32_t{1} << 4));
}

TEST_CASE("side monitor: an opposite-sign member BELOW the opposite floor is not a fight") {
    // BUG CAUGHT: the 0.5 rad/s floor dropped — a member jittering at −0.1 rad/s around zero
    // on a 20 rad/s side is noise, and would be flagged as backwards... but it IS under 25 %
    // of the fastest, so the near-zero rule flags it instead. Separate the two rules: a
    // member at −0.4 rad/s on a side whose fastest is 1.2 rad/s is above 25 % (0.3) and
    // below the 0.5 opposite floor — ONLY the opposite rule could flag it, and it must not.
    CoupledSideMonitor mon;
    auto m = allAt(1.2);
    m[1].velocityRadS = -0.4;
    const SideVerdict v = run(mon, 6.0, m, 100);
    CHECK(v.evaluated);
    CHECK(v.disagreeingMask == 0);
    CHECK(v.persistedMask == 0);
    // At −0.6 it clears the opposite floor and IS flagged.
    CoupledSideMonitor mon2;
    m[1].velocityRadS = -0.6;
    CHECK(run(mon2, 6.0, m, 25).persistedMask == (std::uint32_t{1} << 1));
}

TEST_CASE("side monitor: ONE member near zero while its mates move -> that member (stalled or "
          "frozen)") {
    // BUG CAUGHT: the near-zero rule dropped — a member whose encoder is frozen (ProsMotor's
    // hold-last-good on a sentinel reads a constant position and 0 velocity) or a member
    // stalled against the train would go unnoticed; or the 25 % boundary moved.
    CoupledSideMonitor mon;
    auto m = allAt(8.0);
    m[0].velocityRadS = 0.0;
    CHECK(run(mon, 6.0, m, 24).persistedMask == 0);
    CHECK(run(mon, 6.0, m, 1).persistedMask == (std::uint32_t{1} << 0));
    // The boundary: 25 % of 8.0 is 2.0. 1.99 is flagged; 2.0 is not (strict <).
    CoupledSideMonitor under;
    m[0].velocityRadS = 1.99;
    CHECK(run(under, 6.0, m, 25).persistedMask == (std::uint32_t{1} << 0));
    CoupledSideMonitor at;
    m[0].velocityRadS = 2.0;
    CHECK(run(at, 6.0, m, 25).persistedMask == 0);
    CHECK(run(at, 6.0, m, 1).disagreeingMask == 0);
}

TEST_CASE("side monitor: NOT evaluated under 1 V of command, or under 1 rad/s of motion") {
    // BUG CAUGHT: a side at rest, or barely nudged, judged as fighting — five motors reading
    // 0 rad/s would all be "near zero" and "cut" a robot that is standing still; a 0.8 V
    // creep command would do the same.
    CoupledSideMonitor mon;
    auto m = allAt(8.0);
    m[3].velocityRadS = -8.0;  // a real fight, but the command is below the floor
    SideVerdict v = run(mon, 1.0, m, 100);  // exactly the floor: NOT above it
    CHECK_FALSE(v.evaluated);
    CHECK(v.disagreeingMask == 0);
    CHECK(v.persistedMask == 0);
    CHECK(mon.disagreeTicks(3) == 0);
    v = run(mon, 0.5, m, 100);
    CHECK_FALSE(v.evaluated);
    CHECK(v.persistedMask == 0);
    // Just above the floor it IS evaluated (so the case above tested the floor, not a
    // broken monitor).
    v = run(mon, 1.01, m, 25);
    CHECK(v.evaluated);
    CHECK(v.persistedMask == (std::uint32_t{1} << 3));

    // Motion floor: everything under 1 rad/s, one member opposite -> not evaluated.
    CoupledSideMonitor slow;
    auto s = allAt(0.9);
    s[3].velocityRadS = -0.9;
    v = run(slow, 6.0, s, 100);
    CHECK_FALSE(v.evaluated);
    CHECK(v.persistedMask == 0);
    // The floor is on the FASTEST member: one member at 1.5 makes the side evaluated.
    s[0].velocityRadS = 1.5;
    v = run(slow, 6.0, s, 25);
    CHECK(v.evaluated);
    CHECK(v.fastestRadS == 1.5);
    CHECK(v.persistedMask == (std::uint32_t{1} << 3));  // −0.9: opposite AND above 0.5
}

TEST_CASE("side monitor: a tick below the floor RESETS every streak (the station's rule)") {
    // BUG CAUGHT: streaks surviving a pause — a member that disagreed for 20 ticks, then the
    // driver let go of the stick, then pushed again: the count must start over, or a cut
    // arrives 5 ticks into the next push for a fight that may be over.
    CoupledSideMonitor mon;
    auto m = allAt(8.0);
    m[2].velocityRadS = -8.0;
    (void)run(mon, 6.0, m, 20);
    CHECK(mon.disagreeTicks(2) == 20);
    (void)run(mon, 0.0, allAt(0.0), 1);  // stick released
    CHECK(mon.disagreeTicks(2) == 0);
    CHECK(run(mon, 6.0, m, 24).persistedMask == 0);
    CHECK(run(mon, 6.0, m, 1).persistedMask == (std::uint32_t{1} << 2));
    // reset() does the same explicitly (the drive program's re-arm after a cut).
    mon.reset();
    CHECK(mon.disagreeTicks(2) == 0);
    CHECK(run(mon, 6.0, m, 24).persistedMask == 0);
}

TEST_CASE("side monitor: an ABSENT member is NEVER flagged, and is never the side's fastest") {
    // BUG CAUGHT: a dead port flagged forever — its velocity reads 0 (or a frozen last-good),
    // so without the present flag it is "near zero" on every tick and the drive cuts every
    // second for the rest of the match. That is the exact failure the team lead's "1–2 dead
    // ports shouldn't stop driving" rules out.
    CoupledSideMonitor mon;
    auto m = allAt(8.0);
    m[1] = MemberSample{0.0, false};   // absent, reads 0
    m[4] = MemberSample{-8.0, false};  // absent, reads a garbage opposite value
    const SideVerdict v = run(mon, 6.0, m, 200);
    CHECK(v.evaluated);
    CHECK(v.presentCount == 3);
    CHECK(v.disagreeingMask == 0);
    CHECK(v.persistedMask == 0);
    CHECK(mon.disagreeTicks(1) == 0);
    CHECK(mon.disagreeTicks(4) == 0);
    // An absent member reading 50 rad/s must not become the "fastest" and make every real
    // member look near-zero.
    CoupledSideMonitor ghost;
    auto g = allAt(2.0);
    g[0] = MemberSample{50.0, false};
    const SideVerdict gv = run(ghost, 6.0, g, 50);
    CHECK(gv.fastestRadS == 2.0);
    CHECK(gv.persistedMask == 0);
    // Every member absent: nothing to evaluate.
    CoupledSideMonitor none;
    std::array<MemberSample, kN> all{};
    for (MemberSample& s : all) s = MemberSample{8.0, false};
    const SideVerdict nv = run(none, 6.0, all, 50);
    CHECK_FALSE(nv.evaluated);
    CHECK(nv.presentCount == 0);
    CHECK(nv.persistedMask == 0);
}

TEST_CASE("side monitor: a STICK REVERSAL — all members briefly opposite — clears before 25 "
          "ticks and never cuts") {
    // BUG CAUGHT: a false cut on every reversal. When the driver flips the stick, the command
    // sign changes at once and the train coasts the OLD way for a few ticks (every member
    // reads opposite the new command). That is a transient, and the 250 ms window exists to
    // ride through it.
    CoupledSideMonitor mon;
    (void)run(mon, 6.0, allAt(8.0), 100);  // driving forward, settled
    // Reversal: command −6 V; members decelerate from +8 through zero to −8 over 15 ticks.
    std::uint32_t worstPersisted = 0;
    for (int t = 0; t < 15; ++t) {
        const double v = 8.0 - (16.0 * (t + 1)) / 15.0;  // +8 → −8 (ends exactly at −8.0)
        const SideVerdict s = mon.update(-6.0, std::span<const MemberSample>{allAt(v)});
        worstPersisted |= s.persistedMask;
    }
    CHECK(worstPersisted == 0);
    const SideVerdict settled = run(mon, -6.0, allAt(-8.0), 50);
    CHECK(settled.disagreeingMask == 0);
    CHECK(settled.persistedMask == 0);
    for (std::size_t i = 0; i < kN; ++i) CHECK(mon.disagreeTicks(i) == 0);
    // Whereas a reversal that never completes — the train stuck going the old way for 25
    // ticks under the new command — IS reported, on every member (the "whole side opposite"
    // case the tester names as "back-first push, or the front is wrong").
    CoupledSideMonitor stuck;
    (void)run(stuck, 6.0, allAt(8.0), 10);
    const SideVerdict w = run(stuck, -6.0, allAt(8.0), 25);
    CHECK(w.persistedCount == 5);
    CHECK(w.persistedMask == 0x1Fu);
}

TEST_CASE("side monitor: members beyond kMaxSideMembers are ignored, never written past the "
          "counters") {
    // BUG CAUGHT: a 21-member span indexing a 21-slot counter array off its end (the V5 has
    // 21 ports, so 21 is the honest maximum and the last slot must be usable).
    CoupledSideMonitor mon;
    std::array<MemberSample, kMaxSideMembers + 3> big{};
    for (MemberSample& s : big) s = MemberSample{8.0, true};
    big[kMaxSideMembers - 1].velocityRadS = -8.0;  // the LAST counted slot
    big[kMaxSideMembers].velocityRadS = -8.0;      // beyond: ignored
    SideVerdict v{};
    for (int i = 0; i < 25; ++i) v = mon.update(6.0, std::span<const MemberSample>{big});
    CHECK(v.presentCount == static_cast<int>(kMaxSideMembers));
    CHECK(v.persistedMask == (std::uint32_t{1} << (kMaxSideMembers - 1)));
    CHECK(v.persistedCount == 1);
    CHECK(mon.disagreeTicks(kMaxSideMembers) == 0);
}

TEST_CASE("side volts: 12 x (forward -/+ yawCcw), clamped, both signs — the ONE arithmetic") {
    // BUG CAUGHT: the driver's stick drives the wrong way at 12 V — ∓ swapped (a CCW request
    // turns the robot clockwise), a side unclamped past 12 V, or the request scaled by the
    // wrong axis. Every expected value is a hand-computed literal.
    const SideVolts fwd = tankSideVolts(DriveRequest{1.0, 0.0, 0.0}, 12.0);
    CHECK(fwd.left == 12.0);
    CHECK(fwd.right == 12.0);
    const SideVolts back = tankSideVolts(DriveRequest{-0.5, 0.0, 0.0}, 12.0);
    CHECK(back.left == -6.0);
    CHECK(back.right == -6.0);
    // A CCW (left) turn: the LEFT side slows, the RIGHT side speeds up.
    const SideVolts ccw = tankSideVolts(DriveRequest{0.5, 0.0, 0.25}, 12.0);
    CHECK(ccw.left == 12.0 * (0.5 - 0.25));   // 3.0
    CHECK(ccw.right == 12.0 * (0.5 + 0.25));  // 9.0
    CHECK(ccw.left == 3.0);
    CHECK(ccw.right == 9.0);
    // A CW turn (right stick pushed RIGHT gives a NEGATIVE yawCcw): mirror.
    const SideVolts cw = tankSideVolts(DriveRequest{0.5, 0.0, -0.25}, 12.0);
    CHECK(cw.left == 9.0);
    CHECK(cw.right == 3.0);
    // Spin in place: opposite signs, equal magnitude.
    const SideVolts spin = tankSideVolts(DriveRequest{0.0, 0.0, 1.0}, 12.0);
    CHECK(spin.left == -12.0);
    CHECK(spin.right == 12.0);
    // Clamp: full forward plus full turn would be 24 V on one side; it is 12.
    const SideVolts sat = tankSideVolts(DriveRequest{1.0, 0.0, 1.0}, 12.0);
    CHECK(sat.left == 0.0);
    CHECK(sat.right == 12.0);
    const SideVolts satNeg = tankSideVolts(DriveRequest{-1.0, 0.0, 1.0}, 12.0);
    CHECK(satNeg.left == -12.0);
    CHECK(satNeg.right == 0.0);
    // The ceiling is the caller's: the tester's 3 V ceiling scales the same way.
    const SideVolts three = tankSideVolts(DriveRequest{1.0, 0.0, 0.5}, 3.0);
    CHECK(three.left == 1.5);
    CHECK(three.right == 3.0);
    // The `left` (strafe) component is IGNORED by a tank: it cannot strafe.
    const SideVolts strafe = tankSideVolts(DriveRequest{0.0, 1.0, 0.0}, 12.0);
    CHECK(strafe.left == 0.0);
    CHECK(strafe.right == 0.0);
}

TEST_CASE("absence detector: faulted reads advancing for 25 consecutive ticks latch ABSENT; "
          "a healthy or intermittent port does not") {
    // BUG CAUGHT: a port that stopped answering mid-match staying in the fight detector's
    // agreement set (its frozen 0 velocity would then cut the drive every second); or a
    // single screened read — one bad sample — marking a good port dead.
    MemberAbsenceDetector d;
    int faulted = 0;
    // Healthy: faultedReads never advances.
    for (int i = 0; i < 100; ++i) CHECK_FALSE(d.update(faulted, 8.0, 6.0, 8.0));
    // Dead: advances every tick. The first call after the change is tick 1 of the streak.
    for (int i = 0; i < 24; ++i) CHECK_FALSE(d.update(++faulted, 8.0, 6.0, 8.0));
    CHECK(d.update(++faulted, 8.0, 6.0, 8.0));  // the 25th
    CHECK(d.absent());
    CHECK(d.reason()[0] != '\0');
    // Latched: a port that answers again stays absent for this run.
    for (int i = 0; i < 50; ++i) CHECK(d.update(faulted, 8.0, 6.0, 8.0));
    // Intermittent: 24 advancing, one clean, 24 advancing — never latches.
    MemberAbsenceDetector flaky;
    int f = 0;
    for (int i = 0; i < 24; ++i) CHECK_FALSE(flaky.update(++f, 8.0, 6.0, 8.0));
    CHECK_FALSE(flaky.update(f, 8.0, 6.0, 8.0));  // one good read resets the streak
    for (int i = 0; i < 24; ++i) CHECK_FALSE(flaky.update(++f, 8.0, 6.0, 8.0));
    CHECK_FALSE(flaky.absent());
}

TEST_CASE("absence detector: velocity EXACTLY 0 while the train turns for 25 ticks latches; "
          "a stalled-but-reporting or resting member does not") {
    // BUG CAUGHT: the zero-while-mates-move signature missing (a port whose encoder reports
    // nothing while the train turns is a dead port), or firing at rest (every member reads 0
    // when the robot is still — that must never mark anything absent).
    MemberAbsenceDetector d;
    // At rest: 0 everywhere, no command -> never absent, however long.
    for (int i = 0; i < 500; ++i) CHECK_FALSE(d.update(0, 0.0, 0.0, 0.0));
    // Commanded but the train is not turning (all stalled, mates 0): not this signature.
    for (int i = 0; i < 500; ++i) CHECK_FALSE(d.update(0, 0.0, 6.0, 0.0));
    // Turning train, this member exactly 0: 24 ticks no, 25th yes.
    for (int i = 0; i < 24; ++i) CHECK_FALSE(d.update(0, 0.0, 6.0, 8.0));
    CHECK(d.update(0, 0.0, 6.0, 8.0));
    CHECK(d.absent());
    // A member reading a tiny nonzero velocity (stalled, dragging) is the MONITOR's case, not
    // absence: never latches here.
    MemberAbsenceDetector stalled;
    for (int i = 0; i < 500; ++i) CHECK_FALSE(stalled.update(0, 0.05, 6.0, 8.0));
    // Command below the floor: the train "turning" from a push, not a command — not absence.
    MemberAbsenceDetector pushed;
    for (int i = 0; i < 500; ++i) CHECK_FALSE(pushed.update(0, 0.0, 0.5, 8.0));
}
