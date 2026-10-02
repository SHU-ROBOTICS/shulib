#pragma once
//
// Coupled-side monitor — the fighting-motor detector for a drivetrain side whose
// motors are mechanically coupled through one gear train (chunk R3b Part 0b,
// 2026-09-10), plus the small pure pieces the drive loops need: the per-side arcade
// arithmetic, and the two runtime dead-port detectors (the faulted-read one of 2026-09-10,
// and T4 Part 2's frozen-reading one, which is the one the library's driver loop runs).
//
// THE HAZARD, so the thresholds have a reason: robot two has several motors per
// side on ONE gear train (five in its typed table of 2026-09-10, four on its
// calibration record of 2026-09-25 — the reasoning holds for either). A wrong sign
// on one of them stalls it against the others — every motor sits at stall current,
// the train hums and twitches, nothing moves, and gears strip. The bench tester's
// DRIVE station carries this logic as its gate 5, and that station has run on robot
// two's brain (2026-09-10, ten adapters constructed; no motor has yet turned under
// it); this header is that logic EXTRACTED into a pure, host-tested evaluator so the
// old "shulib Drive", the station, hal::MotorGroup and — since T4 Part 2 — the
// library's driver-loop protections (teleop/drive_protection.hpp) share one detector
// and one set of numbers.
//
// WHAT COUNTS AS DISAGREEING (the station's gate 5, verbatim in meaning):
//   * evaluated only while the side is COMMANDED above `commandFloorV` and its
//     fastest PRESENT member turns above `movingFloorRadS` — below either, a
//     side at rest or a reversing stick would flag everything;
//   * a present member whose velocity has the OPPOSITE sign to the command and
//     magnitude above `oppositeFloorRadS` — it is being driven backwards
//     against its mates;
//   * or a present member whose |velocity| is under `nearZeroFraction` of the
//     side's fastest — it is stalled (or its encoder is not turning with the
//     train: a frozen reading, which ProsMotor holds on a sentinel).
//   * an ABSENT member is never flagged and never counts as the side's fastest.
//     A dead port is a WARNING elsewhere (drivetrain_degradation.hpp); if it
//     were also a disagreement it would cut the drive on every tick forever.
//
// PERSISTENCE IS COUNTED INSIDE: a member must disagree for `persistTicks`
// CONSECUTIVE ticks (250 ms at 10 ms) before it is reported as persisted. That
// window survives a stick reversal — all members coast the old way for a few
// ticks after the command flips, which is a transient, not a fight — and it is
// the same window as the over-current cut. The 25th consecutive tick flags;
// the 24th does not (the tests pin the boundary exactly).
//
// The monitor does not command anything. What to DO about a persisted
// disagreement is the caller's policy: the tester cuts permanently (diagnosis);
// the driver loop's protections (teleop/drive_protection.hpp, since T4 Part 2 —
// the old "shulib Drive" did the same) cut for one second, log, count and re-arm
// (a match must not end on a transient — brief §3); an autonomous motion raises
// MOTOR_GROUP_DISAGREE and continues. Every threshold is INVENTED for a first
// run (R3b Session 2's own words) and is a config field, not a magic number.
//
// PURE and PROS-free: numbers in, a verdict out; the only state is the per-member
// tick counters, and reset() clears them. Host-tested in isolation.

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/teleop/stick_mapping.hpp"

namespace shulib::teleop {

/// |x| as a constexpr-safe helper (the standard's abs(double) is only guaranteed constexpr
/// from C++23, and this tree builds as gnu++20 on both the host and the V5).
[[nodiscard]] constexpr double magnitudeOf(double x) noexcept { return x < 0.0 ? -x : x; }

/// The most members one side may carry. A V5 brain has 21 smart ports, so no side can have
/// more; the counters are sized to this so the monitor never allocates.
inline constexpr std::size_t kMaxSideMembers = 21;

/// The thresholds, every one of them INVENTED for a first hardware run (R3b Session 2 gate 5)
/// and carried here unchanged so the old "shulib Drive", the tester, hal::MotorGroup and the
/// driver loop's protections agree; R3d/R4 measure. PROVISIONAL (A4: HA-132); the near-zero
/// fraction is also HA-133. The protections' FROZEN rule reuses two of them as its floors
/// (FrozenReadingDetector), so it has no threshold of its own but its read count.
struct SideMonitorConfig {
    double commandFloorV = 1.0;        ///< Evaluate only while |side command| exceeds this.
    double movingFloorRadS = 1.0;      ///< ... and only while the side's fastest present member exceeds this.
    double oppositeFloorRadS = 0.5;    ///< An opposite-sign member counts only above this |velocity|.
    double nearZeroFraction = 0.25;    ///< Under this fraction of the side's fastest = stalled/frozen.
    int persistTicks = 25;             ///< Consecutive disagreeing ticks before a member is PERSISTED (250 ms at 10 ms).
};

/// One member's reading this tick, as the monitor needs it.
struct MemberSample {
    double velocityRadS = 0.0;  ///< The adapter's velocity(), canonical rad/s (sign already reversed by PROS for a negative port).
    bool present = true;        ///< false = ABSENT (adapter never constructed, or marked dead at runtime): never flagged, never the fastest.
};

/// What one update() concluded about the side.
struct SideVerdict {
    bool evaluated = false;            ///< The side was above both floors this tick, so members were judged.
    double fastestRadS = 0.0;          ///< The largest |velocity| among PRESENT members this tick.
    int presentCount = 0;              ///< Members with present == true this tick.
    std::uint32_t disagreeingMask = 0; ///< Bit i set = member i disagreed THIS tick (any streak length).
    std::uint32_t persistedMask = 0;   ///< Bit i set = member i has disagreed for persistTicks consecutive ticks or more.
    int persistedCount = 0;            ///< Number of bits set in persistedMask.
};

/// The per-side evaluator. Construct one per side, call update() every tick with that side's
/// command and its members in a FIXED order (the bit positions in the verdict are indices
/// into that span), and act on persistedMask. Members beyond kMaxSideMembers are ignored
/// (the caller's precondition, not an allocation).
class CoupledSideMonitor {
public:
    /// Default thresholds are the station's INVENTED ones (SideMonitorConfig).
    constexpr CoupledSideMonitor() noexcept = default;

    /// Thresholds stated by the caller.
    constexpr explicit CoupledSideMonitor(SideMonitorConfig thresholds) noexcept : cfg_{thresholds} {}

    /// One tick: `commandVolts` is the volts this side was commanded (its SIGN is the expected
    /// velocity sign); `members` are this side's readings in the fixed order. Counters advance
    /// for members disagreeing this tick, reset for members that agree, and reset for EVERY
    /// member on a tick the side is not evaluated (below a floor) — so a side that stops being
    /// driven forgets its streaks, exactly as the station did.
    [[nodiscard]] constexpr SideVerdict update(double commandVolts,
                                               std::span<const MemberSample> members) noexcept {
        SideVerdict v{};
        const std::size_t n = std::min(members.size(), kMaxSideMembers);
        for (std::size_t i = 0; i < n; ++i) {
            if (!members[i].present) continue;
            ++v.presentCount;
            v.fastestRadS = std::max(v.fastestRadS, magnitudeOf(members[i].velocityRadS));
        }
        v.evaluated = magnitudeOf(commandVolts) > cfg_.commandFloorV
                      && v.fastestRadS > cfg_.movingFloorRadS;
        const double expected = commandVolts > 0.0 ? 1.0 : -1.0;
        for (std::size_t i = 0; i < n; ++i) {
            const MemberSample& m = members[i];
            if (!v.evaluated || !m.present) {
                ticks_[i] = 0;
                continue;
            }
            const double speed = magnitudeOf(m.velocityRadS);
            const bool oppositeSign = (m.velocityRadS * expected) < 0.0
                                      && speed > cfg_.oppositeFloorRadS;
            const bool nearZero = speed < cfg_.nearZeroFraction * v.fastestRadS;
            if (oppositeSign || nearZero) {
                ticks_[i] = ticks_[i] + 1;
                v.disagreeingMask |= (std::uint32_t{1} << i);
                if (ticks_[i] >= cfg_.persistTicks) {
                    v.persistedMask |= (std::uint32_t{1} << i);
                    ++v.persistedCount;
                }
            } else {
                ticks_[i] = 0;
            }
        }
        for (std::size_t i = n; i < kMaxSideMembers; ++i) ticks_[i] = 0;
        return v;
    }

    /// Consecutive disagreeing ticks for member `i` right now (0 = agreeing or not evaluated).
    /// The panel colours a member amber on any streak and red on a persisted one.
    [[nodiscard]] constexpr int disagreeTicks(std::size_t i) const noexcept {
        return i < kMaxSideMembers ? ticks_[i] : 0;
    }

    /// Clear every streak — what a re-arm after a cut does, so the same fight has to persist
    /// again for the full window before it cuts again.
    constexpr void reset() noexcept {
        for (int& t : ticks_) t = 0;
    }

    /// The thresholds in force.
    [[nodiscard]] constexpr const SideMonitorConfig& config() const noexcept { return cfg_; }

private:
    SideMonitorConfig cfg_{};
    int ticks_[kMaxSideMembers] = {};
};

/// The two side commands of a tank drive, in volts.
struct SideVolts {
    double left = 0.0;   ///< Volts for every member of the LEFT side.
    double right = 0.0;  ///< Volts for every member of the RIGHT side.
};

/// The arcade arithmetic both the drive program and the tester's DRIVE station use — ONE
/// definition, so the driver feels one robot: left = maxV × (forward − yawCcw), right =
/// maxV × (forward + yawCcw), each clamped to ±maxV. A CCW (left) turn is a positive yawCcw,
/// which slows the left side and speeds the right — the sign convention of the locked body
/// frame (stick_mapping.hpp). Every member of a side gets the same volts (a group is a
/// voltage fan-out).
[[nodiscard]] constexpr SideVolts tankSideVolts(const DriveRequest& request,
                                               double maxVolts) noexcept {
    return SideVolts{
        std::clamp(maxVolts * (request.forward - request.yawCcw), -maxVolts, maxVolts),
        std::clamp(maxVolts * (request.forward + request.yawCcw), -maxVolts, maxVolts)};
}

/// Runtime dead-port detection for ONE member, from the ADAPTER's faulted-read count (R3b
/// Part 0b brief §3 item 4): a port whose adapter constructed at boot can still die in a
/// match. Two signatures, either one for
/// `persistTicks` consecutive ticks marks the member ABSENT — LATCHED for the rest of the
/// run, because a port that answered again for a moment is not one to hand the fight
/// detector back to:
///   * its adapter's faultedReads() counter ADVANCED on every one of those ticks (every read
///     screened to last-good — the port is not answering), or
///   * its velocity read EXACTLY 0.0 while the side was commanded above the command floor
///     and its side-mates' fastest exceeded the moving floor (the train is turning, this
///     encoder is not: a port that reports nothing, not a stalled motor — a stall reads a
///     small nonzero velocity and a large current, and is the monitor's business).
/// Its caller — the old "shulib Drive" (src/programs/drive_program.cpp), which holds each
/// port as a hal::pros::ProsMotor and so HAS the count — keeps commanding the absent member's
/// voltage (harmless) and drops it from the monitor's agreement set by passing present = false.
/// The library's driver loop does NOT run this class: it holds its drive motors as hal::IMotor,
/// which has no faulted-read count (F4, locked), and it does not run the velocity-exactly-0
/// signature either (the team lead's ruling of 2026-09-28, decision C: through IMotor a dead
/// port reads exactly 0 only when it died at rest, where FrozenReadingDetector below latches on
/// the same tick, and alone the signature marks live members — a stalled side beside a slipping
/// mate). It stays for a caller that has the count; after T4 Part 3 retires the old program,
/// no src/ caller does.
class MemberAbsenceDetector {
public:
    /// Default thresholds are the monitor's.
    constexpr MemberAbsenceDetector() noexcept = default;

    /// Thresholds stated by the caller.
    constexpr explicit MemberAbsenceDetector(SideMonitorConfig thresholds) noexcept
        : cfg_{thresholds} {}

    /// One tick. `faultedReadsNow` is the adapter's cumulative faultedReads(); `velocityRadS`
    /// its velocity(); `sideCommandVolts` the side's command; `matesFastestRadS` the largest
    /// |velocity| among the OTHER present members of the side. Returns true once the member
    /// is absent (and stays true: latched).
    [[nodiscard]] constexpr bool update(int faultedReadsNow, double velocityRadS,
                                        double sideCommandVolts,
                                        double matesFastestRadS) noexcept {
        if (absent_) return true;
        if (seen_ && faultedReadsNow > lastFaulted_) {
            ++faultStreak_;
        } else {
            faultStreak_ = 0;
        }
        seen_ = true;
        lastFaulted_ = faultedReadsNow;
        const bool trainTurning = magnitudeOf(sideCommandVolts) > cfg_.commandFloorV
                                  && matesFastestRadS > cfg_.movingFloorRadS;
        if (trainTurning && velocityRadS == 0.0) {
            ++zeroStreak_;
        } else {
            zeroStreak_ = 0;
        }
        if (faultStreak_ >= cfg_.persistTicks) {
            absent_ = true;
            why_ = "reads faulted for 250 ms";
        } else if (zeroStreak_ >= cfg_.persistTicks) {
            absent_ = true;
            why_ = "velocity exactly 0 while the train turned for 250 ms";
        }
        return absent_;
    }

    /// True once update() has latched the member absent.
    [[nodiscard]] constexpr bool absent() const noexcept { return absent_; }

    /// Which signature latched it ("" while present) — for the one-time log line.
    [[nodiscard]] constexpr const char* reason() const noexcept { return why_; }

private:
    SideMonitorConfig cfg_{};
    bool seen_ = false;
    bool absent_ = false;
    int lastFaulted_ = 0;
    int faultStreak_ = 0;
    int zeroStreak_ = 0;
    const char* why_ = "";
};

// ═══ THE FROZEN-READING RULE (chunk T4 Part 2, 2026-09-29) ══════════════════════════════
//
// WHY IT EXISTS. The library holds its drive motors as hal::IMotor, and IMotor has no
// faulted-read count (F4, locked; the team lead's N2 (a)). What a dead port DOES show through
// the seam is ProsMotor's last-good screen (hal/pros/motor.hpp): its position(), velocity() and
// current() all repeat their last good values, bit for bit, for as long as it is dead. So this
// rule looks for exactly that — a member whose readings stop changing while its side is plainly
// turning — and hands the verdict to a caller (the driver loop's protections,
// teleop/drive_protection.hpp), which keeps a FROZEN member out of the fight monitor and the
// over-current watch at once and marks it ABSENT once it has stayed FROZEN for the monitor's
// window.
//
// WHY IT MUST BE SHORTER THAN THE MONITOR. The fight monitor counts a present member's STALE
// velocity — which can be its side's fastest — so a dead member left in it can start a fight
// streak and CUT the drive. The rule decides in `suspectReads` reads (5), well inside the
// monitor's 25 ticks, and it needs neither the command nor the mates when the stale velocity
// itself says "turning" (the 2026-09-27 re-check's probe: every rule as long as the monitor's,
// or that waited for the mates to turn, lost that race). The constructor refuses
// `suspectReads >= persistTicks`.
//
// THE RULES, per member i, once per tick, over the side's PRESENT members:
//   * REPEAT: `repeat[i]` counts the consecutive reads whose position AND velocity are both
//     bit-identical to the member's previous read. 0 on its first read, whenever either
//     changes, and whenever the member is not present (which resets it).
//   * THE MATES' FASTEST for i: the largest |velocity| among the OTHER present, unlatched
//     members whose `repeat` is below `suspectReads` — a member whose readings have ALL held (a
//     dead port's, or a stalled one's once its report has settled) is no evidence that the train
//     turns. A mate whose encoder has stopped while its lagging REPORT still decays DOES count:
//     its report keeps changing, so its `repeat` stays 0 (decision D (a)'s repeat, applied here
//     too). That is where the report-lag gap in WHAT IT CANNOT SEE comes from.
//   * SUSPECT(i): `repeat[i] >= suspectReads`, AND its own |velocity| is above the floors'
//     `oppositeFloorRadS` (the motor says it turns and its encoder does not, which a live motor
//     cannot do) OR the mates' fastest is above `movingFloorRadS` (the train turns and this
//     encoder does not).
//   * FROZEN(i): SUSPECT(i), AND the side's present, unlatched members that are NOT suspect are
//     AT LEAST AS MANY as those that are. A whole side stopping at once — a wall, a stall, a
//     coast-down whose reports lag — is every member alike, never one death, for as long as its
//     members' reports settle alike; and a side where ONE member slips (robot two's port 18, A4
//     register HA-130) and the other three stall at a wall is three suspect against one
//     answering, so nobody is frozen — for as long as the three stalled members read alike
//     (WHAT IT CANNOT SEE, below).
//   * LATCHED: FROZEN for the floors' `persistTicks` consecutive ticks (250 ms). Latched for the
//     detector's life; a latched member is left out of every rule above.
//
// WHY BOTH CHANGES OF 2026-09-28 (the team lead's decision D (a), "G4"). The first rule had
// "some OTHER member is not suspect" as its guard and counted a repeat on POSITION alone. The
// re-check's probe put one slipping member beside three coupled members stalled at a wall under
// a held stick: the slipping member's encoder kept turning, so all three live members were
// FROZEN and, 250 ms later, marked dead — three dead, and the drive refused for the rest of the
// run. The majority-or-tie guard fixes that; requiring the VELOCITY to repeat too closes a
// second path (a slipping member coasting down in a cut, its lagging report still above the
// floor while its position has stopped), because ProsMotor's screen holds every reading of a
// dead port while a live motor's lagging report changes as it decays.
//
// WHAT IT CANNOT SEE, stated. A whole side that stops answering at once looks like a side that
// stopped, so it is never marked (the robot shows it by not driving that side). And the guard is
// a COUNT, which at a wall is right only while every stalled member reads alike:
//   * AT ANY WALL — any sudden stop under a held stick, no slipping member needed: every encoder
//     stops at once, but the mates' lagging REPORTS still count as the train turning (THE MATES'
//     FASTEST). A member whose report settles to 0 enough sooner repeats and is SUSPECT alone —
//     one against three, so the guard holds — and stays FROZEN while a mate's report still reads
//     above `movingFloorRadS`. If that lasts 29 ticks after its own report first reads 0 (5
//     reads to SUSPECT, then 25 FROZEN), a LIVE motor is marked (two of four reading so: a tie,
//     both marked). In the host model: a 30 ms report beside 120 ms ones at full stick, 10 ms
//     beside 100 ms, 30 ms beside 150 ms at half stick (P2-19e); 30 beside 80 stays FROZEN 9 ticks,
//     50 beside 150 24, one short (P2-19d). Found by the verification of 2026-10-01.
//   * BESIDE A SLIPPING MEMBER, whose encoder keeps turning, so the evidence never stops: a side
//     ALREADY down to two of four, whose slipping member keeps turning while its last live mate
//     stalls, is one suspect against one answering — a tie — and the stalled live member is
//     marked (the cost decision D accepted; P2-21c);
//   * and on a FULL side there, ONE stalled member whose readings keep changing — a report
//     flickering by 1 RPM, an encoder toggling a count on a humming train — counts as answering:
//     a 2-2 tie, and two LIVE members are marked; one whose report settles sooner is suspect
//     alone, at a smaller spread than at a plain wall (30 ms beside 80), and is marked. After the
//     tie, a second push marks the side's last stalled member against the slipping one — the
//     third dead, REFUSED for the rest of the run (P2-21d; the review of 2026-09-30).
// The first and the third are wider than decision D weighed, and the team lead's to rule on (the
// second is the cost D accepted). Fixing port 18's slipping gear ends the ties and the smaller
// spread, NOT the report-lag case. Without a slipping member, the flicker and the dither marked
// nobody in the probes (P2-19d). A live member's report that holds one value while its encoder
// also holds for 5 reads beside moving mates would be suspect; the probe saw spells of at most 3
// ticks at a coast-down. All of it is A4 register HA-175 — reasoned and host-probed, NOT measured
// on a robot.
//
// PURE, allocation-free, PROS-free: readings in, a verdict out; the per-member counters are the
// only state and reset() clears them.

/// One member's readings this tick, in the side's fixed order, as FrozenReadingDetector needs them.
struct MemberReading {
    double positionRad = 0.0;   ///< The member's position(), radians — compared bit for bit.
    double velocityRadS = 0.0;  ///< Its velocity(), rad/s — compared bit for bit, and its magnitude against the floors.
    bool present = true;        ///< false = not read this tick (a boot hole, or a member already marked ABSENT): its counters reset and it takes part in no rule.
};

/// What one FrozenReadingDetector::update() concluded, as member bit masks (bit i = member i).
struct FrozenVerdict {
    std::uint32_t suspectMask = 0;  ///< SUSPECT this tick: repeated for suspectReads reads while it, or a mate whose readings still change, says the train turns.
    std::uint32_t frozenMask = 0;   ///< FROZEN this tick — the members a caller keeps out of the fight monitor and the over-current watch.
    std::uint32_t latchedMask = 0;  ///< NEWLY latched ABSENT this tick (each member latches once in the detector's life).
};

/// The frozen-reading rule for ONE side (the header section above): which members read FROZEN
/// this tick, and which have been FROZEN long enough to be marked ABSENT. Construct one per side
/// with that side's monitor thresholds (its floors) and the read count; call update() once per
/// tick with the side's members in a FIXED order. Members beyond kMaxSideMembers are ignored.
class FrozenReadingDetector {
public:
    /// `floors`: the side's own monitor thresholds (`oppositeFloorRadS`, `movingFloorRadS`,
    /// `persistTicks`). `suspectReads`: the consecutive repeated reads that make a member SUSPECT
    /// (DriveProtectionConfig::frozenSuspectReads, 5, A4 register HA-175). PRECONDITION:
    /// `0 < suspectReads < floors.persistTicks` — the rule must decide before the monitor can.
    FrozenReadingDetector(SideMonitorConfig floors, int suspectReads)
        : cfg_{floors}, suspectReads_{suspectReads} {
        SHULIB_PRECONDITION(suspectReads > 0,
                            "FrozenReadingDetector: suspectReads must be > 0");
        SHULIB_PRECONDITION(suspectReads < floors.persistTicks,
                            "FrozenReadingDetector: suspectReads must be below the monitor's "
                            "persistTicks -- a dead member must leave the monitor before the "
                            "monitor can cut on it");
    }

    /// One tick: the side's members' readings, in the fixed order. Returns this tick's verdict.
    [[nodiscard]] FrozenVerdict update(std::span<const MemberReading> members) noexcept {
        FrozenVerdict v{};
        const std::size_t n = std::min(members.size(), kMaxSideMembers);
        bool live[kMaxSideMembers] = {};
        // 1. The repeats.
        for (std::size_t i = 0; i < n; ++i) {
            const MemberReading& m = members[i];
            if (!m.present || (latched_ & bit(i)) != 0) {
                seen_[i] = false;
                repeat_[i] = 0;
                frozenTicks_[i] = 0;
                continue;
            }
            live[i] = true;
            const bool same = seen_[i] && sameBits(m.positionRad, prevPos_[i])
                              && sameBits(m.velocityRadS, prevVel_[i]);
            repeat_[i] = same ? std::min(repeat_[i] + 1, kCountCap) : 0;
            prevPos_[i] = m.positionRad;
            prevVel_[i] = m.velocityRadS;
            seen_[i] = true;
        }
        for (std::size_t i = n; i < kMaxSideMembers; ++i) {
            seen_[i] = false;
            repeat_[i] = 0;
            frozenTicks_[i] = 0;
        }
        // 2. SUSPECT, and the census the guard compares.
        int nSuspect = 0;
        int nAnswering = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (!live[i]) {
                continue;
            }
            double matesFastest = 0.0;
            for (std::size_t j = 0; j < n; ++j) {
                if (j != i && live[j] && repeat_[j] < suspectReads_) {
                    matesFastest = std::max(matesFastest, magnitudeOf(members[j].velocityRadS));
                }
            }
            const bool suspect =
                repeat_[i] >= suspectReads_
                && (magnitudeOf(members[i].velocityRadS) > cfg_.oppositeFloorRadS
                    || matesFastest > cfg_.movingFloorRadS);
            if (suspect) {
                v.suspectMask |= bit(i);
                ++nSuspect;
            } else {
                ++nAnswering;
            }
        }
        // 3. FROZEN (the majority-or-tie guard), and the latch.
        const bool guardHolds = nAnswering >= nSuspect;
        for (std::size_t i = 0; i < n; ++i) {
            if (!live[i]) {
                continue;
            }
            const bool frozen = guardHolds && (v.suspectMask & bit(i)) != 0;
            frozenTicks_[i] = frozen ? std::min(frozenTicks_[i] + 1, kCountCap) : 0;
            if (frozen) {
                v.frozenMask |= bit(i);
                if (frozenTicks_[i] >= cfg_.persistTicks) {
                    latched_ |= bit(i);
                    v.latchedMask |= bit(i);
                }
            }
        }
        return v;
    }

    /// Forget every counter and every latch — a new detector in all but its thresholds.
    void reset() noexcept {
        for (std::size_t i = 0; i < kMaxSideMembers; ++i) {
            seen_[i] = false;
            repeat_[i] = 0;
            frozenTicks_[i] = 0;
            prevPos_[i] = 0.0;
            prevVel_[i] = 0.0;
        }
        latched_ = 0;
    }

    /// Member `i`'s consecutive repeated reads right now (0 out of range) — for a panel or a test.
    [[nodiscard]] int repeatReads(std::size_t i) const noexcept {
        return i < kMaxSideMembers ? repeat_[i] : 0;
    }

    /// Member `i`'s consecutive FROZEN ticks right now (0 out of range, or not frozen).
    [[nodiscard]] int frozenTicks(std::size_t i) const noexcept {
        return i < kMaxSideMembers ? frozenTicks_[i] : 0;
    }

    /// Every member latched ABSENT so far, as a bit mask.
    [[nodiscard]] std::uint32_t latchedMask() const noexcept { return latched_; }

    /// The read count this detector was built with.
    [[nodiscard]] int suspectReads() const noexcept { return suspectReads_; }

    /// The side's thresholds this detector reads its floors and its window from.
    [[nodiscard]] const SideMonitorConfig& floors() const noexcept { return cfg_; }

private:
    /// A counter's ceiling, so a member frozen for a whole match cannot overflow an int.
    static constexpr int kCountCap = 0x40000000;  // 2^30

    /// Bit `i` of a member mask.
    [[nodiscard]] static constexpr std::uint32_t bit(std::size_t i) noexcept {
        return std::uint32_t{1} << i;
    }

    /// Bit for bit, as ProsMotor's screen holds a dead port's reading — not `==`, which would
    /// call 0.0 and -0.0 the same reading and never call a NaN a repeat.
    [[nodiscard]] static bool sameBits(double a, double b) noexcept {
        return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
    }

    SideMonitorConfig cfg_;
    int suspectReads_;
    double prevPos_[kMaxSideMembers] = {};
    double prevVel_[kMaxSideMembers] = {};
    bool seen_[kMaxSideMembers] = {};
    int repeat_[kMaxSideMembers] = {};
    int frozenTicks_[kMaxSideMembers] = {};
    std::uint32_t latched_ = 0;
};

}  // namespace shulib::teleop
