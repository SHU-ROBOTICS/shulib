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
// sentinel screen, hal/motor.hpp:47-54). A MEAN would then drift by 1/N of all further
// travel — silently wrong odometry, exactly the failure the odometry cross-checks cannot
// see because the number keeps moving. A median is untouched until a MAJORITY fails.
// Rejected: mean (above); "first member" (one dead port kills the side with no signal at
// all). For an even N the median is the mean of the two middle values, which one dead
// member still cannot reach when N >= 4; for N == 2 no aggregate can tell the live member
// from the dead one, and the median degrades to a half-drag — stated, not hidden.
// (Robot two is N = 5 per side; the bench bot N = 4.)
//
// current() is the MEAN per member, so the tree's per-motor stall/capture thresholds keep
// their meaning (a member at 2.4 A reads as a member at 2.4 A); the SUM is exposed as the
// separate, non-IMotor totalCurrent(). temperature() is the MAX: the hottest member
// throttles first, and the thermal monitor must see the worst one. brakeMode() is the
// FIRST member's read-back (a device answer, not the library's memory — the IMotor
// contract), with a member that disagrees with it counted by the observable below.
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
// WHY BOOT, AND NOT RUNTIME: hal/pros/motor.hpp's constructor already throws on a dead port
// (its read-back, HA-98), so the composition root catches per port, keeps
// std::optional<ProsMotor> (the drive program's `Member` shape) and hands this class a span
// with holes. Runtime death INSIDE the library graph — promoting the drive program's
// MemberAbsenceDetector in here — is a later chunk; today absence is decided once, at boot,
// by calibration::boot_check against the brain's own registry.
//
// WHAT CHANGES, AND WHAT DOES NOT: setVoltage/setBrakeMode SKIP the holes; position(),
// velocity(), current() and temperature() are over the PRESENT members only; the monitor is
// told `present = false` for a hole and never flags it; memberCount() still reports how many
// SLOTS the group has (so the panel's port list keeps its indices) and presentCount() how
// many answer. ALL-PRESENT GROUPS ARE BIT-IDENTICAL — not "equivalent": the whole
// pre-existing suite is the pin, and its assertion count was recorded before and after this
// change, with `MotorGroup 8b` and the plant's case 7 as the local half of it.
//
// Non-owning: the pointer ARRAY the span views, and the motors it points to, must
// outlive the group — the tree's ownership convention (hal/motor.hpp:57-65).
// PROS-free; host-tested against FakeMotors and through the A2 plant's coupled members.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// THE median of the `n` values in `v` — the one definition in the tree, so nothing that has to
/// agree with a hal::MotorGroup's reading can drift from it. Odd n: the middle value, no
/// arithmetic at all (n == 1 is the bare value, bit for bit). Even n: the mean of the two middle
/// values (exact when they agree), which one dead member still cannot reach when n >= 4 — see the
/// header's "why not the mean". SORTS `v` IN PLACE, which is why the caller passes a scratch
/// array it owns. Precondition: v non-null, n >= 1.
///
/// Extracted from MotorGroup at chunk R3d Part 1 because the drivetrain calibrator measures each
/// side's distance scale through the SAME statistic the group will later report to the odometry:
/// a slow member (robot two's port 18, A4 register HA-130) then cancels exactly instead of
/// approximately. A second, privately written median would have made that cancellation an
/// accident rather than a guarantee.
[[nodiscard]] inline double medianInPlace(double* v, std::size_t n) {
    SHULIB_PRECONDITION(v != nullptr, "medianInPlace: values must be non-null");
    SHULIB_PRECONDITION(n >= 1, "medianInPlace: needs at least one value");
    std::sort(v, v + n);
    return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/// N coupled motors presented as ONE IMotor: commands fan out to every member unchanged;
/// position and velocity read back as the MEDIAN (a dead port's frozen reading cannot drag
/// them until a majority is dead), current as the per-member mean (the sum is
/// totalCurrent()), temperature as the maximum, brake mode as the first member's device
/// read-back. The coupled-side monitor rides inside as the disagreement observable — one
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
    /// the drive program's (INVENTED for a first run; R4 measures).
    ///
    /// Precondition: at least ONE member is PRESENT. A group of nothing but holes has no
    /// position, no velocity and no temperature to report, and every read below would have to
    /// invent one — so it is refused at construction, where a composition root can still paint
    /// a refusal, rather than at the first read, mid-match.
    explicit MotorGroup(std::span<IMotor* const> members,
                        teleop::SideMonitorConfig thresholds = {})
        : members_{members}, monitor_{thresholds} {
        SHULIB_PRECONDITION(!members_.empty(), "MotorGroup: members is empty (N >= 1 required)");
        SHULIB_PRECONDITION(members_.size() <= kMaxMembers, "MotorGroup: too many members");
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
    /// sees it, so a bad command cannot half-apply.
    void setVoltage(units::Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "MotorGroup::setVoltage: voltage must be finite");
        commanded_ = units::Voltage{
            std::clamp(volts.value(), -kMaxMotorVoltage.value(), kMaxMotorVoltage.value())};
        for (IMotor* m : members_) {
            if (m != nullptr) {  // an ABSENT member is skipped, not commanded (header)
                m->setVoltage(volts);
            }
        }
    }

    /// The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror
    /// (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().
    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    /// Fan the mode to every PRESENT member.
    void setBrakeMode(BrakeMode mode) override {
        for (IMotor* m : members_) {
            if (m != nullptr) {
                m->setBrakeMode(mode);
            }
        }
    }

    /// The FIRST PRESENT member's read-back (the device's answer, per the IMotor contract).
    [[nodiscard]] BrakeMode brakeMode() const override { return first().brakeMode(); }

    /// MEDIAN of the PRESENT members' cumulative shaft rotation (header: why not the mean).
    [[nodiscard]] units::AngleDim position() const override {
        return units::AngleDim{medianOf([](const IMotor& m) { return m.position().value(); })};
    }

    /// MEDIAN of the PRESENT members' measured angular velocity.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        return units::AngularVelocity{
            medianOf([](const IMotor& m) { return m.velocity().value(); })};
    }

    /// MEAN current PER PRESENT MEMBER, so per-motor thresholds keep their meaning. An ABSENT
    /// member is not a member drawing 0 A: dividing by the SLOT count would make a side with a
    /// dead port read cooler than it is. The sum is totalCurrent().
    [[nodiscard]] units::Current current() const override {
        return units::Current{sumCurrent() / static_cast<double>(present_)};
    }

    /// MAX PRESENT member temperature (°C): the hottest member throttles first.
    [[nodiscard]] double temperature() const override {
        double worst = first().temperature();
        for (const IMotor* m : members_) {
            if (m != nullptr) {
                worst = std::max(worst, m->temperature());
            }
        }
        return worst;
    }

    /// The SUM of every PRESENT member's current draw (amperes) — the side's total, exposed
    /// outside the IMotor surface so current() can stay per-member.
    [[nodiscard]] units::Current totalCurrent() const { return units::Current{sumCurrent()}; }

    /// One tick of the disagreement observable: samples every member's velocity() against
    /// the group's commanded voltage through the shared coupled-side monitor, advancing its
    /// persistence counters ONCE. Call exactly once per control tick — the health tick
    /// (motion::tickHealthObservables) does. Returns the verdict it also stores.
    const teleop::SideVerdict& evaluateDisagreement() {
        teleop::MemberSample samples[kMaxMembers];
        const std::size_t n = members_.size();
        for (std::size_t i = 0; i < n; ++i) {
            // An ABSENT member is reported present = FALSE, with no reading taken (there is
            // nothing to read). The monitor excludes it from the fastest-member comparison and
            // never flags it — a dead port must not read as a fighting one.
            samples[i] = members_[i] != nullptr
                             ? teleop::MemberSample{members_[i]->velocity().value(), true}
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

    /// How many SLOTS the group has — present and absent together, so a panel's port list keeps
    /// its indices when a member goes missing. presentCount() is how many answer.
    [[nodiscard]] std::size_t memberCount() const noexcept { return members_.size(); }

    /// How many members are PRESENT (not a nullptr slot). Always >= 1.
    [[nodiscard]] std::size_t presentCount() const noexcept { return present_; }

    /// Bit i set = slot i is ABSENT. 0 = a complete group — the state every graph built from a
    /// healthy robot is in, and the one the whole pre-existing suite pins.
    [[nodiscard]] std::uint32_t absentMask() const noexcept { return absent_; }

    /// True when slot `i` is present. Out of range is false, so a panel walking a fixed-size
    /// array cannot fall off the end.
    [[nodiscard]] bool isPresent(std::size_t i) const noexcept {
        return i < members_.size() && members_[i] != nullptr;
    }

    /// Member `i` (0-based, the span's order). Preconditions: i < memberCount() AND slot i is
    /// PRESENT. Asking an absent slot for its motor is a caller bug — there is no object to
    /// return and no sentinel that would not be a lie — so it is a precondition violation
    /// rather than a reference to nothing.
    [[nodiscard]] IMotor& member(std::size_t i) const {
        SHULIB_PRECONDITION(i < members_.size(), "MotorGroup::member: index out of range");
        SHULIB_PRECONDITION(members_[i] != nullptr,
                            "MotorGroup::member: that member is ABSENT (a dead port at boot) — "
                            "ask isPresent(i) first");
        return *members_[i];
    }

    /// The thresholds the disagreement observable runs under.
    [[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept {
        return monitor_.config();
    }

private:
    /// The first PRESENT member. The constructor's precondition guarantees there is one, so
    /// this never falls off the end; the loop is what makes brakeMode() and temperature() read
    /// a DEVICE rather than a hole.
    [[nodiscard]] IMotor& first() const {
        for (IMotor* m : members_) {
            if (m != nullptr) {
                return *m;
            }
        }
        return *members_[0];  // unreachable: present_ >= 1 is a construction precondition
    }

    template <typename Read>
    [[nodiscard]] double medianOf(Read read) const {
        double v[kMaxMembers];
        std::size_t k = 0;
        for (const IMotor* m : members_) {
            if (m != nullptr) {
                v[k++] = read(*m);
            }
        }
        // Over the PRESENT members only: an absent member counted as a reading (0, or a
        // neighbour's) would drag the median exactly the way a dead port's frozen reading
        // would, which is the failure this median exists to survive.
        return medianInPlace(v, k);  // the ONE median (above), shared with the calibrator
    }

    [[nodiscard]] double sumCurrent() const {
        double sum = 0.0;
        for (const IMotor* m : members_) {
            if (m != nullptr) {
                sum += m->current().value();
            }
        }
        return sum;
    }

    std::span<IMotor* const> members_;
    std::size_t present_ = 0;
    std::uint32_t absent_ = 0;
    teleop::CoupledSideMonitor monitor_;
    teleop::SideVerdict verdict_{};
    units::Voltage commanded_{0.0};
};

}  // namespace shulib::hal
