#pragma once
//
// MotorGroup — N physically COUPLED motors behind ONE IMotor (chunk R3b Part 1, 2026-09-14).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// A real VEX drivetrain has several motors per side on one gear train. The kinematics
// layer sees ONE wheel per side (kinematics/tank.hpp:80: "how many physical motors sit
// on each side is the HAL's business"), and the command pipeline maps kinematic wheel →
// motor 1:1 (motion/command_pipeline.hpp step 7). Until this class existed, no HAL
// facility answered the kinematics' delegation: a context built with N motors per side
// was ACCEPTED by MotionDeps::validate()'s old `>=` guard, two motors were commanded and
// the rest were never given a voltage or a brake mode, silently (roadmap: "What R3b must
// BUILD", item 1; measured on the bench bot's then-7-motor drive). This class is that
// facility, and MotionDeps::validate() now demands EQUALITY so the defect cannot recur:
// with `driveMotors = {&leftGroup, &rightGroup}` the 1:1 mapping is CORRECT by
// construction, which is the whole point of the group.
//
// ── Command path: fan-out, nothing else ─────────────────────────────────────────────
// setVoltage(v) and setBrakeMode(m) go to every member, the SAME value. Clamping is each
// member's own; commandedVoltage() is the group's local mirror of the last applied value
// (the ProsMotor convention). THE GROUP NEVER NEGATES: a member that must turn the other
// way is constructed with a NEGATIVE port, and PROS applies that reversal exactly once,
// in the adapter (hal/pros/motor.hpp header). One place for a sign, or the sign gets
// applied twice (R3b Session 2 §2.3, landmine 4).
//
// ── Read path: MEDIAN, not mean — and why ───────────────────────────────────────────
// position() and velocity() are the MEDIAN across members. Members are coupled, so they
// agree to within noise when healthy; the question is what happens when one is not. A
// member whose port stopped answering reports its LAST GOOD position forever (ProsMotor's
// sentinel screen — the IMotor class doc in hal/motor.hpp, ~:60-67). A MEAN would then drift
// by 1/N of all further travel — silently wrong odometry, exactly the failure the odometry
// cross-checks cannot see because the number keeps moving. A median is untouched until a
// MAJORITY fails.
// Rejected: mean (above); "first member" (one dead port kills the side with no signal at
// all). For an even N the median is the mean of the two middle values, which one dead
// member still cannot reach when N >= 4; for N == 2 no aggregate can tell the live member
// from the dead one, and the median degrades to a half-drag — stated, not hidden.
// (Robot two is N = 4 per side on its calibration record of 2026-09-25 — its typed table
// listed five, and this header was written for five; the bench bot is N = 4 too.)
//
// current() is the MEAN per member, so the tree's per-motor stall/capture thresholds keep
// their meaning (a member at 2.4 A reads as a member at 2.4 A); the SUM is exposed as the
// separate, non-IMotor totalCurrent(). temperature() is the MAX: the hottest member
// throttles first, and the thermal monitor must see the worst one. brakeMode() is the
// FIRST member's read-back (a device answer, not the library's memory — the IMotor
// contract), with a member that disagrees with it counted by the observable below.
//
// ── A READING THAT IS NOT A NUMBER (DIAG1 Part B, 2026-09-29) ───────────────────────
// A member whose reading is not finite — NaN of either sign, +Inf or -Inf — is LEFT OUT of that
// read, the way an ABSENT member (below) is left out of every read: position() and velocity()
// are the median of the present members whose reading is finite, current() the mean and
// totalCurrent() the sum over them, temperature() their max. The rule lives in ONE place,
// isUsableReading(): medianInPlace() excludes with it and nonFiniteReads() counts with it.
//  * PER READ, NEVER A LATCH. The same member's next finite reading is counted again;
//    presentCount() and absentMask() never change for it. A member that is ALWAYS unreadable is
//    a dead port, and that is the absent member's business, decided at boot.
//  * NaN WHEN NONE IS FINITE. When no present member gives a finite reading the read returns a
//    fresh quiet NaN (std::numeric_limits<double>::quiet_NaN()), never one of its inputs, so the
//    answer does not depend on which non-finite values arrived ({+Inf, -Inf} used to give a
//    NEGATIVE NaN). The drive-encoder odometry already handles a NaN side: that tick freezes
//    position and the next finite tick integrates across it. motion::OdoStallCheck, which reads
//    the drive motors when a window closes, gets a NaN spin travel from such a side and gives NO
//    verdict for that window, nor for the next (its new baseline is the NaN) — where a side all
//    +Inf or all -Inf used to read as an infinite travel and, on a drivetrain with an independent
//    motion source, a stall (ODO_STUCK). What the check should do with a travel it cannot read
//    is a follow-up's decision.
//  * WHY ±Inf TOO, NOT ONLY NaN. +Inf is PROS's own failure sentinel (PROS_ERR_F, A4 register
//    HA-08): it means "no reading", not a position. Kept in, one infinite member of three or
//    more still moves the median by half a neighbour's gap, and on a side of one or two members
//    it makes the whole side non-finite.
//  * WHY NOT A LAST-GOOD CACHE HERE. The adapter already holds last-good (ProsMotor's screen);
//    a second cache in the group would be a frozen member feeding the median — the drag this
//    median exists to survive — and a latch the absent-member rule does not have.
//  * WHY NOT A MAJORITY RULE (NaN once half or more are non-finite — the DIAG1 research's other
//    option). The recommendation the team lead approved (2026-09-26) chose exclusion: one rule
//    for "no reading", whether the port died at boot or one read failed.
//  * THE MEMBERSHIP STEP (A4 register HA-165). Leaving members out changes WHICH values the
//    median is taken over, so the side's reading can step on the tick a member drops out, and
//    step back on the tick it returns, with nothing moving. For ONE member left out: of an odd
//    count (three or more), the median moves to the midpoint of the two values then in the
//    middle — the middle value and a neighbour, or the middle value's two neighbours when the
//    member left out IS the middle one — by at most HALF the larger of the two gaps beside the
//    middle value; of an even count, it moves from the midpoint to one of the two middle
//    values — by half the gap between them (for a two-member side, half the gap between the
//    two). With SEVERAL left out, the median of the rest can land anywhere between the lowest
//    and the highest member position, so the step is bounded only by the members' SPREAD
//    (highest minus lowest); the worst case is the one member farthest from the full median,
//    read alone. Coupled members sit close together, so the step is small — unless their
//    CUMULATIVE positions have drifted apart: robot two's port 18 travelled ~20 % short on a
//    hand push (HA-130; unmeasured under power), so its offset would grow with the distance
//    driven. The odometry reads the step as travel; its trust gate bounds one tick at
//    maxTickTravel (HA-123), and the return tick carries the step back. A side whose infinite
//    members used to reach the middle — two +Inf of three, say — read non-finite and froze its
//    tick as NONFINITE+TRAVEL; it now reads the rest's median and integrates, so a step under
//    maxTickTravel (36 in by default) moves the pose SILENTLY (DIAG1 N7 pins that, and TRAVEL
//    past the bound).
//  * THE UNDEFINED BEHAVIOUR IT REMOVES. The median used to std::sort every value, and a NaN
//    breaks the strict weak ordering std::sort requires. Probed against this header before the
//    change (host x86-64, g++ 13.3.0 libstdc++, -O2; the robot's library was not probed):
//    {3, NaN, 1, 2} gave NaN; {NaN, 1, 2, 3, 4} gave 2 where the finite median is 2.5;
//    {1, 2, NaN, 3, 4} gave NaN while {1, 2, 3, 4, NaN} gave 3; 21 members with 1-3 NaN gave a
//    WRONG FINITE value on 18,030 of 20,000 draws and NaN on 1,350 — a plausible wrong number,
//    decided by which slot was bad.
//  * NOT A ROBOT PATH TODAY. ProsMotor screens a non-finite read to the last good one before any
//    group sees it (HA-08), so on a V5 this exclusion is expected never to fire. It defines what
//    the library does when a reading does get through — a future adapter, a nested group, a host
//    test. Nothing here has run on a robot.
//
// ── The disagreement observable: ONE evaluator, shared with the drive program ───────
// The fighting-motor / frozen-member detector is teleop::CoupledSideMonitor — the pure,
// host-tested evaluator "shulib Drive" and the tester's DRIVE station already run (R3b
// Part 0b). This class REUSES it rather than writing a second one: same thresholds, same
// persistence window, same meaning of "disagreeing" (a member opposite in sign to the
// command above 0.5 rad/s, or under 25 % of the group's fastest, while the group is
// commanded above 1 V and turning above 1 rad/s, for 25 consecutive ticks). Because the
// monitor COUNTS PERSISTENCE, it must be evaluated exactly ONCE per control tick, not on
// every read: evaluateDisagreement() is that tick, and motion::tickHealthObservables()
// calls it for every group in MotionDeps::motorGroups — so it runs in Chassis::drive()
// and in every motion with no extra caller wiring. (The Session 2 brief said "evaluated
// on each read"; a stateful persistence counter cannot be, and a read-side evaluation
// would advance the window once per position()/velocity()/temperature() call. Recorded
// in the Parts 1–3 log.)
//
// The group RAISES NO FAULT — raising is the loop layer's policy, exactly as
// hal/pros/motor.hpp says of the adapter. diag::HealthMonitor turns a persisted
// disagreement into FaultCode::MotorGroupDisagree (appended, wire-stable), and the
// scheduler's default abort mask leaves it on the CONTINUE side: the drive still works
// with one bad member, and the driver must be TOLD, not stopped. The alternative — abort
// on a fight — was rejected because a match must not end on a member that is merely
// slow (robot two's port 18 travels ~20 % short of its side-mates on every push and is
// NOT a fight — A4 register HA-130; test 16 pins that it is tolerated and that an opposing
// member is not — the disagreement FLOOR, A4 register HA-133, invented with the rest of the
// thresholds, HA-132). Configure the mask to abort if a season's data says otherwise.
// In DRIVER CONTROL, since T4 Part 2, the library's drive protections act on the same verdict
// with a third trade — the old "shulib Drive"'s, ruled in by the team lead (R4): a 1 s CUT of
// the whole drive that re-arms by itself (teleop/drive_protection.hpp) — a pause, never the end
// of a match; the fault is still raised as above.
//
// ── THE ABSENT MEMBER (chunk R3d Part 2, ruling P4) ─────────────────────────────────
// A `nullptr` in the members span is an ABSENT member: a drive port the composition root
// found DEAD at boot. The group's contract was "every member is PRESENT"; it is now "AT
// LEAST ONE member is present", and every read below says which members it is over.
//
// WHY A HOLE IN THE SPAN, AND NOT A NullMotor OBJECT: a fake in a competition binary is
// test scaffolding (R3b §6's rule), and a NullMotor would have to answer position() and
// velocity() with SOMETHING — and every answer is a lie the median would then average in.
// A hole cannot be read by accident.
//
// WHY A HOLE AT BOOT: a dead port is known before the graph exists. The graphs that use this
// class — robot::Robot (`BuildOptions::absentDrivePorts`, since G1 Part 2a), which "shulib
// Teleop" is built on since G1 Part 2b — take the MISSING set calibration::boot_check read from
// the brain's own registry, never construct those ports, and hand this class a span with a hole
// where each one would be. (The old "shulib Drive", src/programs/drive_program.cpp, never used
// this class: it caught hal/pros/motor.hpp's constructor throw per port and drove the rest itself.)
//
// WHAT CHANGES, AND WHAT DOES NOT: setVoltage/setBrakeMode SKIP the holes; position(),
// velocity(), current() and temperature() are over the PRESENT members only; the monitor is
// told `present = false` for a hole and never flags it; memberCount() still reports how many
// SLOTS the group has (so the panel's port list keeps its indices) and presentCount() how
// many answer. ALL-PRESENT GROUPS ARE BIT-IDENTICAL — not "equivalent": the whole
// pre-existing suite is the pin, and its assertion count was recorded before and after this
// change, with `MotorGroup 8b` and the plant's case 7 as the local half of it. A PRESENT
// member whose reading is not finite (above) is left out of that one read and moves neither
// count: it is still a slot, still present, still commanded.
//
// ── A MEMBER THAT DIES MID-RUN (chunk T4 Part 2, 2026-09-29) ─────────────────────────
// A port that answered at boot can still stop answering in a match. Through the IMotor seam it
// does not say so — ProsMotor holds its last good readings (hal/pros/motor.hpp) and IMotor has no
// validity channel (F4, locked) — so the group cannot see it by itself. The driver loop's
// protections can (teleop/drive_protection.hpp: a member whose readings FREEZE while its side
// answers, teleop::FrozenReadingDetector), and they act through two calls here:
//   * markAbsent(i) LATCHES member i ABSENT for the group's life. From then on it is out of EVERY
//     read, exactly like a boot hole, and the monitor is told `present = false` — but it is STILL
//     COMMANDED: setVoltage and setBrakeMode still reach it (the old "shulib Drive"'s rule, "still
//     commanded (harmless)": a dead port ignores a command, and a member whose READS failed while
//     its motor still turns keeps pulling). It REFUSES — returns false, changes nothing, never
//     throws — out of range, a hole, a second mark, and the LAST present member: a throw mid-match
//     would end driver control, and a group must always have a reading to report.
//   * setMonitorExcluded(mask) keeps the members whose bit is set out of the fight monitor (fed
//     `present = false`) until the mask changes, and changes NOTHING else: they stay in every read
//     and are still commanded. It is how a member whose readings are FROZEN this tick is kept from
//     starting a fight streak with its stale velocity before it is marked. Only the protections set
//     it, and they clear it whenever driver control ends; motion code never touches it, so
//     MOTOR_GROUP_DISAGREE in an autonomous motion is unchanged.
// absentMask() stays the BOOT holes; runtimeAbsentMask() is the runtime marks; presentCount() and
// isPresent() count both. A group with no marks and a zero mask is BIT-IDENTICAL to one before
// T4 Part 2 (the whole suite is the pin, its assertion count recorded before and after).
//
// LABELS: an optional span giving each slot's PORT NUMBER, so a log line can name the port a pit
// crew can find. robot::Robot passes the description's port magnitudes; a group with no labels
// names a slot "member i". Labels are only printed — nothing reads them to decide anything.
//
// Non-owning: the pointer ARRAY the span views, and the motors it points to, must
// outlive the group — the tree's ownership convention (the IMotor destructor's doc in
// hal/motor.hpp, ~:70-78).
// PROS-free; host-tested against FakeMotors and through the A2 plant's coupled members.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

// ── A BUILD THAT ASSUMES EVERY NUMBER IS FINITE IS REFUSED HERE ─────────────────────
// isUsableReading() below is std::isfinite, and under -ffinite-math-only (which -ffast-math and
// -Ofast imply) the compiler may assume no value is ever NaN or infinite and fold that test to
// "true". Measured (DIAG1 Part B review round, 2026-09-30): arm-none-eabi-g++ 13.2.1 at
// -Os -ffinite-math-only compiles std::isfinite(x) to `mov r0, #1; bx lr`, and host g++ 13.3.0
// at -O2 -ffinite-math-only gives isUsableReading(NaN) == true and medianInPlace({1, 2, NaN, 3,
// 4}) == NaN — the undefined behaviour this header removes, back, with every gate green. No
// build sets either flag today (the Makefile, common.mk and the host CMake were grepped); this
// makes one that does stop at compile time, in every build that includes this header (the ARM
// header gate and every robot build the src build gate links). GCC and Clang define
// __FINITE_MATH_ONLY__ as 1 under those flags and 0 otherwise.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "shulib needs NaN and infinity: build without -ffinite-math-only, -ffast-math or -Ofast"
#endif

namespace shulib::hal {

/// True when `v` is a READING the group's aggregates count: a finite number. NaN (of either
/// sign), +Inf and -Inf are not readings — PROS's failure sentinel is +Inf (A4 register HA-08) —
/// and every aggregate in this header leaves them out, the way it leaves out an ABSENT member
/// (the header's "A READING THAT IS NOT A NUMBER"). This is the ONE test: medianInPlace()
/// excludes with it, MotorGroup::nonFiniteReads() counts with it, and the drivetrain
/// calibrator's "moved on the push" rule is built on it, so the three cannot disagree about what
/// a reading is (DIAG1 Part B). It needs a build that keeps NaN and infinity: a build with
/// -ffinite-math-only or -ffast-math is refused by an #error just above it.
[[nodiscard]] inline bool isUsableReading(double v) noexcept { return std::isfinite(v); }

/// THE median of the FINITE values among the `n` in `v` — the one definition in the tree, so
/// nothing that has to agree with a hal::MotorGroup's reading can drift from it. A value that is
/// not finite (isUsableReading() false: NaN of either sign, ±Inf) is LEFT OUT, the way the group
/// leaves out an absent member; when none is finite the result is a fresh quiet NaN, never one of
/// the inputs. Over the k finite values: odd k, the middle value, no arithmetic at all (k == 1 is
/// the bare value, bit for bit); even k, std::midpoint of the two middle values — exact when they
/// agree, and it cannot overflow where 0.5 * (a + b) could — which one dead member still cannot
/// reach when k >= 4 (the header's "why not the mean"). REORDERS `v`: its contents afterwards are
/// unspecified, which is why the caller passes a scratch array it owns and never reads it again.
/// Precondition: v non-null, n >= 1.
///
/// Extracted from MotorGroup at chunk R3d Part 1 because the drivetrain calibrator measures each
/// side's distance scale through the SAME statistic the group will later report to the odometry:
/// a slow member (robot two's port 18, A4 register HA-130) then cancels exactly instead of
/// approximately. A second, privately written median would have made that cancellation an
/// accident rather than a guarantee — and two medians that treated a NaN differently would break
/// it exactly when a reading is bad, which is why the exclusion lives HERE and not in the group.
///
/// DIAG1 Part B (2026-09-29) — how, and what it pins. The finite values are copied FORWARD in
/// their input order before the sort, never compacted with std::partition or swap-with-last:
/// std::sort is not stable, so among EQUAL values (+0.0 and -0.0) which one lands in the middle
/// depends on the order it is handed, and a compaction that reordered the finite values would let
/// {+0.0, NaN, -0.0, +0.0} answer differently from the same group with that member absent,
/// {+0.0, -0.0, +0.0}. With every value finite the answer is bit-identical to the version that
/// sorted everything and halved the sum — except where the two middle values' SUM overflowed
/// (possible only when at least one of them is past DBL_MAX/2 in size, so reachable only in host
/// tests), which gave +Inf or -Inf and now gives the finite midpoint. So the median is non-finite
/// exactly when no value is finite.
[[nodiscard]] inline double medianInPlace(double* v, std::size_t n) {
    SHULIB_PRECONDITION(v != nullptr, "medianInPlace: values must be non-null");
    SHULIB_PRECONDITION(n >= 1, "medianInPlace: needs at least one value");
    std::size_t k = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (isUsableReading(v[i])) {
            v[k++] = v[i];  // copy forward: the finite values keep their input order
        }
    }
    if (k == 0) {
        return std::numeric_limits<double>::quiet_NaN();  // no reading at all: a FRESH NaN
    }
    std::sort(v, v + k);
    return (k % 2 == 1) ? v[k / 2] : std::midpoint(v[k / 2 - 1], v[k / 2]);
}

/// N coupled motors presented as ONE IMotor: commands fan out to every member unchanged;
/// position and velocity read back as the MEDIAN (a dead port's frozen reading cannot drag
/// them until a majority is dead), current as the per-member mean (the sum is
/// totalCurrent()), temperature as the maximum, brake mode as the first member's device
/// read-back. Every aggregate is over the PRESENT members whose reading is FINITE: a member
/// reading NaN or ±Inf is left out of that read (never latched), counted by nonFiniteReads(),
/// and a read with no finite member returns NaN (the header's "A READING THAT IS NOT A NUMBER").
/// The coupled-side monitor rides inside as the disagreement observable — one
/// evaluation per tick through evaluateDisagreement(), which motion::tickHealthObservables
/// performs for every group it is handed. Raises nothing itself; never negates a member.
/// Non-owning: members and the array the span views must outlive the group.
class MotorGroup final : public IMotor {
public:
    /// The most members one group may carry — a V5 brain has 21 smart ports, and the
    /// monitor's bit masks are sized to the same bound.
    static constexpr std::size_t kMaxMembers = teleop::kMaxSideMembers;

    /// `members`: N >= 1 slots in a FIXED order (the bit positions of disagreeingMask() and
    /// absentMask() index this span). A `nullptr` slot is an ABSENT member — a drive port found
    /// dead at boot (header). `thresholds`: the disagreement detector's knobs — the defaults are
    /// the old "shulib Drive"'s (INVENTED for a first run; R4 measures). `labels`: empty, or ONE
    /// port number per slot in the same order, for log lines (header, LABELS) — non-owning, so the
    /// array it views must outlive the group.
    ///
    /// Precondition: at least ONE member is PRESENT. A group of nothing but holes has no
    /// position, no velocity and no temperature to report, and every read below would have to
    /// invent one — so it is refused at construction, where a composition root can still paint
    /// a refusal, rather than at the first read, mid-match. And `labels` is empty or exactly
    /// as long as `members`.
    explicit MotorGroup(std::span<IMotor* const> members,
                        teleop::SideMonitorConfig thresholds = {},
                        std::span<const int> labels = {})
        : members_{members}, labels_{labels}, monitor_{thresholds} {
        SHULIB_PRECONDITION(!members_.empty(), "MotorGroup: members is empty (N >= 1 required)");
        SHULIB_PRECONDITION(members_.size() <= kMaxMembers, "MotorGroup: too many members");
        SHULIB_PRECONDITION(labels_.empty() || labels_.size() == members_.size(),
                            "MotorGroup: labels must be empty or one per member slot");
        for (std::size_t i = 0; i < members_.size(); ++i) {
            if (members_[i] != nullptr) {
                ++present_;
            } else {
                absent_ |= std::uint32_t{1} << i;
            }
        }
        SHULIB_PRECONDITION(present_ >= 1,
                            "MotorGroup: every member is ABSENT (a nullptr slot) — a group with "
                            "nothing present has no reading to report");
    }

    /// Fan the SAME command to every member (each clamps for itself); the group mirrors the
    /// clamped value for commandedVoltage(). Non-finite is rejected here, before any member
    /// sees it, so a bad command cannot half-apply. A boot hole is skipped; a member marked
    /// ABSENT at runtime is still commanded (header).
    void setVoltage(units::Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "MotorGroup::setVoltage: voltage must be finite");
        commanded_ = units::Voltage{
            std::clamp(volts.value(), -kMaxMotorVoltage.value(), kMaxMotorVoltage.value())};
        for (IMotor* m : members_) {
            if (m != nullptr) {  // a boot HOLE is skipped, not commanded (header)
                m->setVoltage(volts);
            }
        }
    }

    /// The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror
    /// (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().
    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    /// Fan the mode to every member that is not a boot hole — a runtime-ABSENT member included
    /// (header: it is still commanded).
    void setBrakeMode(BrakeMode mode) override {
        for (IMotor* m : members_) {
            if (m != nullptr) {
                m->setBrakeMode(mode);
            }
        }
    }

    /// The FIRST PRESENT member's read-back (the device's answer, per the IMotor contract).
    [[nodiscard]] BrakeMode brakeMode() const override { return first().brakeMode(); }

    /// MEDIAN of the PRESENT members' cumulative shaft rotation (header: why not the mean), over
    /// the members whose reading is FINITE; NaN if none is finite. A member left out is counted
    /// by nonFiniteReads().
    [[nodiscard]] units::AngleDim position() const override {
        return units::AngleDim{medianOf([](const IMotor& m) { return m.position().value(); })};
    }

    /// MEDIAN of the PRESENT members' measured angular velocity, over the members whose reading
    /// is FINITE; NaN if none is finite.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        return units::AngularVelocity{
            medianOf([](const IMotor& m) { return m.velocity().value(); })};
    }

    /// MEAN current PER PRESENT MEMBER, so per-motor thresholds keep their meaning. An ABSENT
    /// member is not a member drawing 0 A: dividing by the SLOT count would make a side with a
    /// dead port read cooler than it is. Neither is a member whose reading is not finite: it is
    /// left out of the sum AND of the count it is divided by (header: "A READING THAT IS NOT A
    /// NUMBER"). NaN if no present member's reading is finite. The sum is totalCurrent().
    [[nodiscard]] units::Current current() const override {
        const CurrentSum s = sumCurrent();
        return units::Current{s.counted > 0 ? s.amps / static_cast<double>(s.counted)
                                            : std::numeric_limits<double>::quiet_NaN()};
    }

    /// MAX temperature (°C) over the PRESENT members whose reading is FINITE: the hottest member
    /// throttles first. A member reading +Inf °C is left out, not taken as the hottest — so on its
    /// own it no longer raises MOTOR_OVER_TEMP — and counted by nonFiniteReads(); a NaN member can
    /// no longer win by standing first (std::max keeps its first argument against a NaN, so the
    /// old answer depended on member order). NaN if no present member's reading is finite. Each
    /// member is read ONCE (until DIAG1 Part B the first present member was read twice, once as
    /// the seed).
    [[nodiscard]] double temperature() const override {
        double worst = std::numeric_limits<double>::quiet_NaN();
        bool any = false;
        for (std::size_t i = 0; i < members_.size(); ++i) {
            if (!readable(i)) {
                continue;  // a boot hole or a member marked ABSENT at runtime: not read
            }
            const double t = members_[i]->temperature();
            if (!isUsableReading(t)) {
                ++nonFiniteReads_;
                continue;
            }
            worst = any ? std::max(worst, t) : t;
            any = true;
        }
        return worst;
    }

    /// The SUM of the current draw (amperes) of every PRESENT member whose reading is FINITE — the
    /// side's total, exposed outside the IMotor surface so current() can stay per-member. NaN if
    /// no present member's reading is finite: an empty sum would read 0 A, a side drawing nothing,
    /// which is a claim no member made.
    [[nodiscard]] units::Current totalCurrent() const {
        const CurrentSum s = sumCurrent();
        return units::Current{s.counted > 0 ? s.amps : std::numeric_limits<double>::quiet_NaN()};
    }

    /// How many member READINGS this group has left out for not being finite (NaN of either
    /// sign, +Inf, -Inf) — one per member per read, over position(), velocity(), current(),
    /// totalCurrent() and temperature() alike; cumulative for the group's life, never reset.
    /// It makes the exclusion OBSERVABLE, as absentMask() does a hole: a member left out in
    /// silence would be a reading nobody knows is missing. Counted with the exclusion's own test
    /// (isUsableReading()), so the two cannot drift. Mirrors hal::pros::ProsMotor::faultedReads(),
    /// including its `mutable` counter bumped by a const read. evaluateDisagreement()'s velocity
    /// samples are NOT counted: what the monitor does with a non-finite member velocity is its own,
    /// later change. The group raises nothing on this number (raising is the loop layer's policy,
    /// header) and nothing in the library reads it yet. On a V5 it is expected to stay 0 —
    /// ProsMotor screens a non-finite read to the last good one first (A4 register HA-08).
    [[nodiscard]] int nonFiniteReads() const noexcept { return nonFiniteReads_; }

    /// One tick of the disagreement observable: samples every member's velocity() against
    /// the group's commanded voltage through the shared coupled-side monitor, advancing its
    /// persistence counters ONCE. Call exactly once per control tick — the health tick
    /// (motion::tickHealthObservables) does. Returns the verdict it also stores. A member that
    /// is ABSENT (a boot hole or a runtime mark) or excluded by setMonitorExcluded() is fed as
    /// `present = false`, with no reading taken.
    const teleop::SideVerdict& evaluateDisagreement() {
        teleop::MemberSample samples[kMaxMembers];
        const std::size_t n = members_.size();
        for (std::size_t i = 0; i < n; ++i) {
            // An ABSENT member is reported present = FALSE, with no reading taken (there is
            // nothing to read, or nothing worth reading). The monitor excludes it from the
            // fastest-member comparison and never flags it — a dead port must not read as a
            // fighting one. An EXCLUDED member (a frozen reading, this tick) the same way.
            const bool fed = readable(i) && (excluded_ & bit(i)) == 0;
            samples[i] = fed ? teleop::MemberSample{members_[i]->velocity().value(), true}
                             : teleop::MemberSample{0.0, false};
        }
        verdict_ = monitor_.update(commanded_.value(),
                                   std::span<const teleop::MemberSample>{samples, n});
        return verdict_;
    }

    /// Members that have disagreed with the group for the full persistence window, as of
    /// the last evaluateDisagreement() (0 = healthy, or never evaluated).
    [[nodiscard]] int disagreeingMembers() const noexcept { return verdict_.persistedCount; }

    /// Bit i set = member i (this span's index) is PERSISTENTLY disagreeing.
    [[nodiscard]] std::uint32_t disagreeingMask() const noexcept { return verdict_.persistedMask; }

    /// The whole verdict of the last evaluateDisagreement() — for a panel that wants the
    /// this-tick mask, the fastest member and the present count as well.
    [[nodiscard]] const teleop::SideVerdict& lastVerdict() const noexcept { return verdict_; }

    /// Clear the monitor's streaks and the stored verdict (a re-arm after a cut, or a
    /// new-run boundary).
    void resetDisagreement() noexcept {
        monitor_.reset();
        verdict_ = teleop::SideVerdict{};
    }

    /// Latch member `i` ABSENT for the rest of the group's life — a port that stopped answering
    /// mid-run (header, A MEMBER THAT DIES MID-RUN). From then on it is out of every read and fed
    /// to the monitor as `present = false`, and it is STILL COMMANDED. Returns true when it marked
    /// the member; REFUSES — false, nothing changed, never a throw — when `i` is out of range, is
    /// a boot hole, is already marked, or is the LAST present member (a group must always have a
    /// reading to report). A mark also clears the member's setMonitorExcluded() bit.
    bool markAbsent(std::size_t i) noexcept {
        if (i >= members_.size() || members_[i] == nullptr || (runtimeAbsent_ & bit(i)) != 0
            || present_ <= 1) {
            return false;
        }
        runtimeAbsent_ |= bit(i);
        excluded_ &= ~bit(i);
        --present_;
        return true;
    }

    /// Bit i set = member i was marked ABSENT at runtime (markAbsent()). 0 = no runtime mark.
    /// absentMask() is the BOOT holes; the two never overlap.
    [[nodiscard]] std::uint32_t runtimeAbsentMask() const noexcept { return runtimeAbsent_; }

    /// Keep the members whose bit is set OUT of the fight monitor — fed `present = false` on
    /// every evaluateDisagreement() until the mask changes — and change nothing else: they stay
    /// in every read and are still commanded (header). Bits past memberCount(), and bits on a
    /// boot hole or a runtime-ABSENT member, are dropped. 0 (the default) is bit-identical to a
    /// group that never had a mask. The driver loop's protections set it every tick and clear it
    /// whenever driver control ends; nothing else should.
    void setMonitorExcluded(std::uint32_t mask) noexcept {
        excluded_ = mask & rangeMask() & ~absent_ & ~runtimeAbsent_;
    }

    /// The mask setMonitorExcluded() left in force, with its dropped bits already gone.
    [[nodiscard]] std::uint32_t monitorExcludedMask() const noexcept { return excluded_; }

    /// How many SLOTS the group has — present and absent together, so a panel's port list keeps
    /// its indices when a member goes missing. presentCount() is how many answer.
    [[nodiscard]] std::size_t memberCount() const noexcept { return members_.size(); }

    /// How many members are PRESENT — neither a boot hole nor marked ABSENT at runtime. Always
    /// >= 1 (the constructor's precondition, and markAbsent()'s refusal of the last one).
    [[nodiscard]] std::size_t presentCount() const noexcept { return present_; }

    /// Bit i set = slot i is a BOOT hole (a nullptr slot). 0 = a complete group — the state every
    /// graph built from a healthy robot is in, and the one the whole pre-existing suite pins.
    /// A member marked ABSENT at runtime is runtimeAbsentMask()'s, not this.
    [[nodiscard]] std::uint32_t absentMask() const noexcept { return absent_; }

    /// True when slot `i` is PRESENT: neither a boot hole nor marked ABSENT at runtime. Out of
    /// range is false, so a panel walking a fixed-size array cannot fall off the end.
    [[nodiscard]] bool isPresent(std::size_t i) const noexcept {
        return i < members_.size() && readable(i);
    }

    /// Member `i` (0-based, the span's order). Preconditions: i < memberCount() AND slot i is not
    /// a BOOT hole (absentMask()). Asking a hole for its motor is a caller bug — there is no
    /// object to return and no sentinel that would not be a lie — so it is a precondition
    /// violation rather than a reference to nothing. A member marked ABSENT at runtime IS
    /// returned — it exists and is still commanded — and its readings are its last good ones,
    /// which is why nothing in the library reads it (ask isPresent(i) first).
    [[nodiscard]] IMotor& member(std::size_t i) const {
        SHULIB_PRECONDITION(i < members_.size(), "MotorGroup::member: index out of range");
        SHULIB_PRECONDITION(members_[i] != nullptr,
                            "MotorGroup::member: that member is ABSENT (a dead port at boot) — "
                            "ask isPresent(i) first");
        return *members_[i];
    }

    /// Slot `i`'s label — the port number the composition root gave it — or 0 when this group
    /// was built without labels or `i` is out of range. Only printed, never decided on.
    [[nodiscard]] int label(std::size_t i) const noexcept {
        return i < labels_.size() ? labels_[i] : 0;
    }

    /// The thresholds the disagreement observable runs under.
    [[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept {
        return monitor_.config();
    }

private:
    /// Bit `i` of a member mask.
    [[nodiscard]] static constexpr std::uint32_t bit(std::size_t i) noexcept {
        return std::uint32_t{1} << i;
    }

    /// Every slot's bit.
    [[nodiscard]] std::uint32_t rangeMask() const noexcept {
        return members_.size() >= 32 ? ~std::uint32_t{0}
                                     : (std::uint32_t{1} << members_.size()) - 1U;
    }

    /// Slot `i` holds a member that is READ: not a boot hole, not marked ABSENT at runtime.
    [[nodiscard]] bool readable(std::size_t i) const noexcept {
        return members_[i] != nullptr && (runtimeAbsent_ & bit(i)) == 0;
    }

    /// The first PRESENT member. The constructor's precondition and markAbsent()'s refusal of the
    /// last member guarantee there is one, so this never falls off the end; the loop is what makes
    /// brakeMode() read a DEVICE rather than a hole. (temperature() used it as its seed until DIAG1
    /// Part B; it no longer does.)
    [[nodiscard]] IMotor& first() const {
        for (std::size_t i = 0; i < members_.size(); ++i) {
            if (readable(i)) {
                return *members_[i];
            }
        }
        return *members_[0];  // unreachable: present_ >= 1 always
    }

    template <typename Read>
    [[nodiscard]] double medianOf(Read read) const {
        double v[kMaxMembers];
        std::size_t k = 0;
        for (std::size_t i = 0; i < members_.size(); ++i) {
            if (readable(i)) {
                const double x = read(*members_[i]);
                if (!isUsableReading(x)) {
                    ++nonFiniteReads_;  // the exclusion's own test (medianInPlace leaves it out)
                }
                v[k++] = x;
            }
        }
        // Over the PRESENT members only: an absent member counted as a reading (0, or a
        // neighbour's) would drag the median exactly the way a dead port's frozen reading
        // would, which is the failure this median exists to survive. A present member whose
        // reading is not finite is handed in and left out BY the median itself, so the
        // calibrator's sites, which call it directly, cannot treat it differently.
        return medianInPlace(v, k);  // the ONE median (above), shared with the calibrator
    }

    /// The current draw summed over the present members whose reading is finite, and how many
    /// that was — current() divides by the SAME count it summed over. "Present" is readable():
    /// neither a boot hole nor a member marked ABSENT at runtime (T4 Part 2a).
    struct CurrentSum {
        double amps = 0.0;
        std::size_t counted = 0;
    };

    [[nodiscard]] CurrentSum sumCurrent() const {
        CurrentSum s;
        for (std::size_t i = 0; i < members_.size(); ++i) {
            if (readable(i)) {
                const double a = members_[i]->current().value();
                if (!isUsableReading(a)) {
                    ++nonFiniteReads_;
                    continue;
                }
                s.amps += a;
                ++s.counted;
            }
        }
        return s;
    }

    std::span<IMotor* const> members_;
    std::span<const int> labels_;
    std::size_t present_ = 0;
    std::uint32_t absent_ = 0;
    std::uint32_t runtimeAbsent_ = 0;
    std::uint32_t excluded_ = 0;
    teleop::CoupledSideMonitor monitor_;
    teleop::SideVerdict verdict_{};
    units::Voltage commanded_{0.0};
    /// nonFiniteReads(): `mutable` because the reads that bump it are const (the IMotor
    /// contract), exactly as hal::pros::ProsMotor's faultedReads_ is.
    mutable int nonFiniteReads_ = 0;
};

}  // namespace shulib::hal
