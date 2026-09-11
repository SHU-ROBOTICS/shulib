#pragma once
//
// Coupled-side monitor — the fighting-motor detector for a drivetrain side whose
// motors are mechanically coupled through one gear train (chunk R3b Part 0b,
// 2026-09-10), plus the two small pure pieces the same drive loop needs: the
// per-side arcade arithmetic, and the runtime dead-port detector.
//
// THE HAZARD, so the thresholds have a reason: robot two has five motors per
// side on ONE gear train. A wrong sign on one of them stalls it against the
// other four — every motor sits at stall current, the train hums and twitches,
// nothing moves, and gears strip. The bench tester's DRIVE station carries this
// logic as its gate 5, and that station has run on robot two's brain (2026-09-10,
// ten adapters constructed; no motor has yet turned under it); this header is that
// logic EXTRACTED into a pure, host-tested evaluator so the
// drive program and the station share one detector and one set of numbers.
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
// the drive program cuts for one second, logs, counts and re-arms (a match must
// not end on a transient — brief §3). Every threshold is INVENTED for a first
// run (R3b Session 2's own words) and is a config field, not a magic number.
//
// PURE and PROS-free: numbers in, a verdict out; the only state is the per-member
// tick counters, and reset() clears them. Host-tested in isolation.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/teleop/stick_mapping.hpp"

namespace shulib::teleop {

/// |x| as a constexpr-safe helper (the standard's abs(double) is only guaranteed constexpr
/// from C++23, and this tree builds as gnu++20 on both the host and the V5).
[[nodiscard]] constexpr double magnitudeOf(double x) noexcept { return x < 0.0 ? -x : x; }

/// The most members one side may carry. A V5 brain has 21 smart ports, so no side can have
/// more; the counters are sized to this so the monitor never allocates.
inline constexpr std::size_t kMaxSideMembers = 21;

/// The thresholds, every one of them INVENTED for a first hardware run (R3b Session 2 gate 5)
/// and carried here unchanged so the drive program and the tester agree; R4 measures.
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

/// Runtime dead-port detection for ONE member (brief §3 item 4): a port whose adapter
/// constructed at boot can still die in a match. Two signatures, either one for
/// `persistTicks` consecutive ticks marks the member ABSENT — LATCHED for the rest of the
/// run, because a port that answered again for a moment is not one to hand the fight
/// detector back to:
///   * its adapter's faultedReads() counter ADVANCED on every one of those ticks (every read
///     screened to last-good — the port is not answering), or
///   * its velocity read EXACTLY 0.0 while the side was commanded above the command floor
///     and its side-mates' fastest exceeded the moving floor (the train is turning, this
///     encoder is not: a port that reports nothing, not a stalled motor — a stall reads a
///     small nonzero velocity and a large current, and is the monitor's business).
/// The drive program keeps commanding the absent member's voltage (harmless) and drops it
/// from the monitor's agreement set by passing present = false.
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

}  // namespace shulib::teleop
