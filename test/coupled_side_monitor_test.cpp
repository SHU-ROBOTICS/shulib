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
#include <limits>
#include <span>

#include "shulib/core/check.hpp"
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

// ═══ T4 Part 2 — THE FROZEN-READING RULE (FrozenReadingDetector), alone (P2-10b) ═════════
// The rule the library's driver loop runs to see a drive port die through the IMotor seam
// (drive_protection.hpp). Every rule is pinned at its boundary, on FOUR members (robot two's side
// since its record of 2026-09-25), with the team lead's decision D (a) of 2026-09-28: a repeat needs
// position AND velocity unchanged, and FROZEN needs at least as many of the side answering as not.
// Each case names the bug it catches; the mutations are in T4-PROGRESS "Part 2a".
using shulib::teleop::FrozenReadingDetector;
using shulib::teleop::FrozenVerdict;
using shulib::teleop::MemberReading;

namespace {

/// Four members, each either HELD (the same reading every tick, as ProsMotor's screen holds a dead
/// port) or TURNING (its position advancing by velocity × 10 ms each tick, as a live encoder does).
struct Side4 {
    std::array<double, 4> pos{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> vel{5.0, 5.0, 5.0, 5.0};
    std::array<bool, 4> held{false, false, false, false};
    std::array<bool, 4> present{true, true, true, true};
    FrozenReadingDetector det{SideMonitorConfig{}, 5};

    /// One tick: the readings as they are now, then the turning members advance.
    FrozenVerdict tick() {
        std::array<MemberReading, 4> r{};
        for (std::size_t i = 0; i < 4; ++i) {
            r[i] = MemberReading{pos[i], vel[i], present[i]};
        }
        const FrozenVerdict v = det.update(std::span<const MemberReading>{r});
        for (std::size_t i = 0; i < 4; ++i) {
            if (!held[i]) pos[i] += vel[i] * 0.01;
        }
        return v;
    }
    FrozenVerdict ticks(int n) {
        FrozenVerdict v{};
        for (int k = 0; k < n; ++k) v = tick();
        return v;
    }
};

constexpr std::uint32_t bit(std::size_t i) { return std::uint32_t{1} << i; }

}  // namespace

TEST_CASE("T4 P2-10b: SUSPECT at the 5th REPEATED read, not the 4th — a dead member held at +v "
          "beside turning mates") {
    // BUG CAUGHT: the read count off by one either way; SUSPECT on the first repeat (a live
    // encoder between counts would leave the monitor every other tick).
    Side4 s;
    s.held[0] = true;
    s.vel[0] = 3.0;  // its stale report says it turns
    // Read 1 is the first (repeat 0); reads 2..5 are repeats 1..4.
    const FrozenVerdict at4 = s.ticks(5);
    CHECK(s.det.repeatReads(0) == 4);
    CHECK((at4.suspectMask & bit(0)) == 0u);
    CHECK((at4.frozenMask & bit(0)) == 0u);
    const FrozenVerdict at5 = s.tick();
    CHECK(s.det.repeatReads(0) == 5);
    CHECK(at5.suspectMask == bit(0));
    CHECK(at5.frozenMask == bit(0));  // one suspect against three answering
    CHECK(at5.latchedMask == 0u);
}

TEST_CASE("T4 P2-10b: the OWN-speed branch at its floor — a held member reporting 0.50 rad/s is "
          "not SUSPECT, 0.51 is (mates answering, but slower than the 1 rad/s moving floor)") {
    // BUG CAUGHT: `>=` for `>` on the opposite floor; the own-speed branch dropped (a member that
    // dies turning slowly, beside slow mates, is then never seen).
    for (const double own : {0.50, 0.51}) {
        Side4 s;
        s.vel = {own, 0.9, 0.9, 0.9};  // the mates turn (their encoders change) below 1 rad/s
        s.held[0] = true;
        const FrozenVerdict v = s.ticks(10);
        CAPTURE(own);
        CHECK(((v.suspectMask & bit(0)) != 0) == (own > 0.5));
        CHECK(((v.frozenMask & bit(0)) != 0) == (own > 0.5));
    }
    // …and the same boundary BACKWARD (the review of 2026-09-30, F8): the floor is on the speed's
    // magnitude, so a member that died turning in reverse is caught by its own stale speed too.
    for (const double own : {-0.50, -0.51}) {
        Side4 s;
        s.vel = {own, -0.9, -0.9, -0.9};
        s.held[0] = true;
        const FrozenVerdict v = s.ticks(10);
        CAPTURE(own);
        CHECK(((v.suspectMask & bit(0)) != 0) == (own < -0.5));
        CHECK(((v.frozenMask & bit(0)) != 0) == (own < -0.5));
    }
}

TEST_CASE("T4 P2-10b: the MATES branch at its floor — a held member reading 0 beside mates at "
          "1.00 rad/s is not SUSPECT, at 1.01 it is") {
    // BUG CAUGHT: `>=` for `>` on the moving floor; the mates branch dropped (a port that died AT
    // REST, reading 0, is then never seen when the train drives away — P2-5).
    for (const double mates : {1.00, 1.01}) {
        Side4 s;
        s.vel = {0.0, mates, mates, mates};
        s.held[0] = true;
        const FrozenVerdict v = s.ticks(10);
        CAPTURE(mates);
        CHECK(((v.suspectMask & bit(0)) != 0) == (mates > 1.0));
    }
    // …and the same boundary with the mates turning BACKWARD (the review of 2026-09-30, F7): their
    // speed is judged by its magnitude, so a port that died at rest is seen when the robot backs away.
    for (const double mates : {-1.00, -1.01}) {
        Side4 s;
        s.vel = {0.0, mates, mates, mates};
        s.held[0] = true;
        const FrozenVerdict v = s.ticks(10);
        CAPTURE(mates);
        CHECK(((v.suspectMask & bit(0)) != 0) == (mates < -1.0));
    }
}

TEST_CASE("T4 P2-10b: a mate whose OWN readings repeat is NOT evidence that the train turns") {
    // BUG CAUGHT (the brief's mutation "the mates' fastest counting a member whose position
    // repeats"): one dead member's stale +v making every standing mate look stalled — during a cut
    // the train stands, the dead member's report still says 5 rad/s, and the three live members
    // would all turn SUSPECT, outvote it, and it would never be marked (P2-6c on the loop).
    Side4 s;
    s.vel = {0.0, 5.0, 0.5, 0.5};  // member 1 is the dead one, frozen at +5; 2 and 3 creep
    s.held[0] = true;               // member 0 genuinely stands (0 rad/s, position unchanged)
    s.held[1] = true;
    const FrozenVerdict v = s.ticks(10);
    CHECK((v.suspectMask & bit(1)) != 0);  // its own stale speed makes it SUSPECT
    CHECK((v.suspectMask & bit(0)) == 0);  // member 1's 5 rad/s is not a mate's evidence
    CHECK(v.frozenMask == bit(1));
}

TEST_CASE("T4 P2-10b: a mate stops being evidence at EXACTLY suspectReads repeats — at its 4th repeat it "
          "still is, at its 5th it is not") {
    // BUG CAUGHT (the review of 2026-09-30, F2): the mates' fastest counting a mate whose repeat has
    // just reached suspectReads (`<=` for `<`) — the case above asserts only long after, so the
    // boundary itself was unpinned. Two members only (the others not present), both held from the
    // first read: member 1 at a stale 2 rad/s, member 0 at rest. Their repeats rise together.
    Side4 s;
    s.present = {true, true, false, false};
    s.held = {true, true, false, false};
    s.vel = {0.0, 2.0, 0.0, 0.0};
    const FrozenVerdict at4 = s.ticks(5);  // repeat 4 each
    REQUIRE(s.det.repeatReads(0) == 4);
    REQUIRE(s.det.repeatReads(1) == 4);
    CHECK(at4.suspectMask == 0u);  // nobody has 5 repeats yet
    const FrozenVerdict at5 = s.tick();  // repeat 5 each
    REQUIRE(s.det.repeatReads(1) == 5);
    // Member 1 is SUSPECT by its own stale speed; member 0 is NOT — its only mate has now repeated
    // suspectReads times and is no evidence that the train turns. One against one: a tie, FROZEN.
    CHECK(at5.suspectMask == bit(1));
    CHECK(at5.frozenMask == bit(1));
}

TEST_CASE("T4 P2-10b: THE ENCODER CHECK (the team lead's ruling of 2026-10-01, \"D, again\") — a mate "
          "whose ENCODER has held for suspectReads reads is no evidence that the train turns, however "
          "its report still changes: at its 4th encoder repeat it still is, at its 5th it is not") {
    // BUG CAUGHT: the mates' evidence on a changing REPORT over a stopped encoder (the rule until
    // 2026-10-01): at a wall every encoder stops while the lagging reports decay, and a member whose
    // report settled first was SUSPECT, FROZEN and marked — a live motor (P2-19e on the loop). And the
    // encoder window off by one either way. Member 0 stands from its first read (reading 0); its
    // mates turn for 10 ticks, then their encoders hold while their reports keep decaying above
    // 1 rad/s — so their position-AND-velocity repeat stays 0 throughout, and only the ENCODER's
    // repeat can take them out of the evidence.
    Side4 s;
    s.held[0] = true;
    s.vel = {0.0, 3.0, 3.0, 3.0};
    const FrozenVerdict turning = s.ticks(10);
    CHECK(turning.suspectMask == bit(0));  // the mates' encoders move: member 0 is SUSPECT, FROZEN
    CHECK(turning.frozenMask == bit(0));
    s.held = {true, true, true, true};  // the wall: every encoder stops from this read on
    for (int k = 0; k <= 4; ++k) {      // the mates' encoder repeats 0..4 (their first held read is 0)
        for (std::size_t j = 1; j < 4; ++j) s.vel[j] -= 0.05;  // their reports decay, still > 1 rad/s
        const FrozenVerdict v = s.tick();
        CAPTURE(k);
        CHECK(s.det.repeatReads(1) == 0);  // the report changes: no position-AND-velocity repeat
        CHECK(v.suspectMask == bit(0));     // an encoder that moved within 5 reads is evidence
    }
    for (int k = 5; k < 30; ++k) {  // the mates' 5th encoder repeat on: no longer evidence
        for (std::size_t j = 1; j < 4; ++j) s.vel[j] -= 0.05;
        const FrozenVerdict v = s.tick();
        CAPTURE(k);
        REQUIRE(s.vel[1] > 1.0);  // their reports still say "turning", above the moving floor
        CHECK(v.suspectMask == 0u);
        CHECK(v.frozenMask == 0u);
    }
    CHECK(s.det.latchedMask() == 0u);
}

TEST_CASE("T4 P2-10b: THE ENCODER CHECK at a wall whose reports settle UNEVENLY — every encoder stops "
          "together, one report reaches 0 first, the others decay for 60 reads: NOBODY is suspect, "
          "nobody FROZEN, nobody latched") {
    // BUG CAUGHT: the any-wall gap of the verification of 2026-10-01 at the detector itself — the
    // member whose report settles first repeats, and the mates' still-decaying reports (with every
    // encoder stopped) kept it SUSPECT and FROZEN for 25 ticks: latched, a live motor marked. Under
    // the encoder check its mates are no evidence from their 5th encoder repeat, before its own
    // position-AND-velocity repeat can reach 5.
    Side4 s;
    s.vel = {3.0, 3.0, 3.0, 3.0};
    (void)s.ticks(10);
    s.held = {true, true, true, true};
    for (int k = 0; k < 60; ++k) {
        s.vel[0] = k < 2 ? 1.5 - 0.75 * k : 0.0;  // member 0's report: 1.5, 0.75, then 0 for good
        for (std::size_t j = 1; j < 4; ++j) s.vel[j] = 3.0 - 0.03 * k;  // theirs: still > 1 at 60
        const FrozenVerdict v = s.tick();
        CAPTURE(k);
        CHECK(v.suspectMask == 0u);
        CHECK(v.frozenMask == 0u);
    }
    CHECK(s.det.repeatReads(0) >= 5);  // member 0 HAS repeated: only its mates' evidence is gone
    CHECK(s.det.latchedMask() == 0u);
}

TEST_CASE("T4 P2-10b: a repeat is BIT FOR BIT — a report alternating 0.0 and -0.0 is no repeat; a NaN "
          "held bit for bit is one") {
    // BUG CAUGHT (the review of 2026-09-30, F1): the repeat compared with `==`, which calls 0.0 and
    // -0.0 the same reading (a live report whose sign bit flickers at zero would read as held) and
    // never calls a NaN a repeat (a port whose screen held a NaN would never be seen) — the two
    // reasons the header gives for comparing bits.
    Side4 flicker;
    flicker.held[0] = true;
    for (int k = 0; k < 20; ++k) {
        flicker.vel[0] = (k % 2 == 0) ? 0.0 : -0.0;
        const FrozenVerdict v = flicker.tick();
        CHECK(flicker.det.repeatReads(0) == 0);
        CHECK((v.suspectMask & bit(0)) == 0u);
    }
    Side4 nan;
    nan.held[0] = true;
    nan.vel[0] = std::numeric_limits<double>::quiet_NaN();
    (void)nan.ticks(5);
    CHECK(nan.det.repeatReads(0) == 4);
    const FrozenVerdict v = nan.tick();
    CHECK(nan.det.repeatReads(0) == 5);
    CHECK(v.suspectMask == bit(0));  // by its mates (a NaN is above no floor)
    CHECK(v.frozenMask == bit(0));
}

TEST_CASE("T4 P2-10b: D (a) — a read whose position repeats but whose VELOCITY changed is not a "
          "repeat") {
    // BUG CAUGHT (the brief's mutation "the repeat on position alone"): a live member whose
    // encoder stops between counts while its lagging report is still decaying — a slipping member
    // coasting in a cut (the re-check's S1c), or a coast-down (P2-19c) — counted as frozen.
    Side4 s;
    s.held[0] = true;
    for (int k = 0; k < 40; ++k) {
        s.vel[0] = (k % 2 == 0) ? 3.0 : 2.9;  // the report changes, the position does not
        const FrozenVerdict v = s.tick();
        CHECK(s.det.repeatReads(0) == 0);
        CHECK((v.suspectMask & bit(0)) == 0u);
    }
    CHECK(s.det.latchedMask() == 0u);
}

TEST_CASE("T4 P2-10b: D (a)'s guard at its boundary on four members — 1 suspect vs 3 FROZEN, 2 vs 2 "
          "(a tie) both FROZEN, 3 vs 1 NOBODY, 4 vs 0 nobody") {
    // BUG CAUGHT (the brief's mutation "FROZEN guard 'some other member not SUSPECT'", item 4 as
    // first written): three stalled LIVE members beside one slipping mate all FROZEN and marked —
    // three dead, the drive REFUSED (the re-check's finding 14, port 18). And a strict majority,
    // which would never mark a 2-of-4 side's next death (S8).
    for (int nHeld = 1; nHeld <= 4; ++nHeld) {
        Side4 s;
        for (std::size_t i = 0; i < 4; ++i) {
            s.held[i] = static_cast<int>(i) < nHeld;
            s.vel[i] = 3.0;  // every one says it turns: the held ones are SUSPECT by their own speed
        }
        const FrozenVerdict v = s.ticks(8);
        CAPTURE(nHeld);
        std::uint32_t heldMask = 0;
        for (int i = 0; i < nHeld; ++i) heldMask |= bit(static_cast<std::size_t>(i));
        CHECK(v.suspectMask == heldMask);
        CHECK(v.frozenMask == (nHeld <= 2 ? heldMask : 0u));
    }
}

TEST_CASE("T4 P2-10b: on a side with two members already gone (not present), 1 suspect vs 1 "
          "answering is a tie — FROZEN") {
    // BUG CAUGHT: the census counting absent members (a 2-of-4 side's next death never marked, and
    // it cuts every 1.25 s instead — the re-check's S8), or a strict majority.
    Side4 s;
    s.present[0] = false;
    s.present[1] = false;
    s.held[2] = true;
    s.vel[2] = 3.0;
    const FrozenVerdict v = s.ticks(8);
    CHECK(v.suspectMask == bit(2));
    CHECK(v.frozenMask == bit(2));
    CHECK(s.det.repeatReads(0) == 0);  // a member not present is reset, never counted
    // …and the two that are left stopping TOGETHER (a wall) is two suspect against none answering:
    // nobody. A census that counted the gone members as answering would freeze both — the robot's
    // last two on that side marked at a wall.
    Side4 both;
    both.present[0] = false;
    both.present[1] = false;
    both.held[2] = true;
    both.held[3] = true;
    both.vel = {0.0, 0.0, 3.0, 3.0};
    const FrozenVerdict w = both.ticks(8);
    CHECK(w.suspectMask == (bit(2) | bit(3)));
    CHECK(w.frozenMask == 0u);
}

TEST_CASE("T4 P2-10b: a member NOT PRESENT for a tick starts over — its repeat count and its FROZEN "
          "streak reset, and its first read back is a first read") {
    // BUG CAUGHT: a presence that flickers (a caller's present flag) carrying a stale repeat count
    // across the gap — a member that came back reading what it read before would be SUSPECT on its
    // first read back.
    Side4 s;
    s.held[0] = true;
    s.vel[0] = 3.0;
    (void)s.ticks(12);  // repeat 11, FROZEN for 7
    REQUIRE(s.det.repeatReads(0) == 11);
    REQUIRE(s.det.frozenTicks(0) == 7);
    s.present[0] = false;
    const FrozenVerdict gone = s.tick();
    CHECK(s.det.repeatReads(0) == 0);
    CHECK(s.det.frozenTicks(0) == 0);
    CHECK((gone.suspectMask & bit(0)) == 0u);
    s.present[0] = true;  // back, reading exactly what it read before the gap
    const FrozenVerdict back = s.tick();
    CHECK(s.det.repeatReads(0) == 0);  // a first read, not the 12th repeat
    CHECK((back.suspectMask & bit(0)) == 0u);
    (void)s.ticks(4);
    CHECK(s.det.repeatReads(0) == 4);
    CHECK((s.tick().frozenMask & bit(0)) != 0u);  // SUSPECT and FROZEN again at its 5th repeat
}

TEST_CASE("T4 P2-10b: LATCHED on the 25th consecutive FROZEN tick, not the 24th — and a latched "
          "member stays latched, whatever it reads after") {
    // BUG CAUGHT: the latch window off by one; a latch that forgets (a port that answered again for
    // a moment is not one to hand the fight monitor back to — the old detector's rule).
    Side4 s;
    s.held[0] = true;
    s.vel[0] = 3.0;
    (void)s.ticks(5);  // repeats 0..4: not yet FROZEN
    for (int k = 1; k <= 24; ++k) {
        const FrozenVerdict v = s.tick();
        REQUIRE(v.frozenMask == bit(0));
        CHECK(v.latchedMask == 0u);
        CHECK(s.det.frozenTicks(0) == k);
    }
    const FrozenVerdict at25 = s.tick();
    CHECK(at25.latchedMask == bit(0));
    CHECK(at25.frozenMask == bit(0));
    CHECK(s.det.latchedMask() == bit(0));
    s.held[0] = false;  // it "answers" again
    for (int k = 0; k < 30; ++k) {
        const FrozenVerdict v = s.tick();
        CHECK(v.latchedMask == 0u);  // latched ONCE — never again, never un-latched
        CHECK((v.suspectMask & bit(0)) == 0u);
        CHECK((v.frozenMask & bit(0)) == 0u);
    }
    CHECK(s.det.latchedMask() == bit(0));
    s.det.reset();
    CHECK(s.det.latchedMask() == 0u);
}

TEST_CASE("T4 P2-10b: a whole side that stops together is NOBODY frozen — the wall, a stall, a "
          "report that lags a stop") {
    // BUG CAUGHT: a live train at a wall with its reports lagging marking a live member dead (the
    // 2026-09-27 re-check's fourth failure). Every member alike is never one death.
    Side4 s;
    s.held = {true, true, true, true};
    s.vel = {4.0, 3.9, 4.1, 4.0};  // lagging reports, held (the probe's worst case: they do not decay)
    for (int k = 0; k < 60; ++k) {
        const FrozenVerdict v = s.tick();
        CHECK(v.frozenMask == 0u);
    }
    CHECK(s.det.latchedMask() == 0u);
}

TEST_CASE("T4 P2-10b: the precondition — 0 < suspectReads < persistTicks") {
    // BUG CAUGHT: a read count as long as the monitor's window (the first proposal's): a dead
    // member's stale velocity then starts a fight streak before it is excluded — the race P2-6 pins.
    CHECK_THROWS_AS(FrozenReadingDetector(SideMonitorConfig{}, 0), shulib::PreconditionError);
    CHECK_THROWS_AS(FrozenReadingDetector(SideMonitorConfig{}, -1), shulib::PreconditionError);
    CHECK_THROWS_AS(FrozenReadingDetector(SideMonitorConfig{}, 25), shulib::PreconditionError);
    CHECK_NOTHROW(FrozenReadingDetector(SideMonitorConfig{}, 24));
    SideMonitorConfig shortWindow{};
    shortWindow.persistTicks = 5;
    CHECK_THROWS_AS(FrozenReadingDetector(shortWindow, 5), shulib::PreconditionError);
}
