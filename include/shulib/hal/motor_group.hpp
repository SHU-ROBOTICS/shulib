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
// Every member is PRESENT by contract: the library graph requires N >= 1 constructed
// adapters and has no dead-port tolerance (that is "shulib Drive"'s job until R3d), so
// the monitor's `present` flag is always true here.
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

    /// `members`: N >= 1 present adapters, all non-null, in a FIXED order (the bit positions
    /// of disagreeingMask() index this span). `thresholds`: the disagreement detector's
    /// knobs — the defaults are the drive program's (INVENTED for a first run; R4 measures).
    explicit MotorGroup(std::span<IMotor* const> members,
                        teleop::SideMonitorConfig thresholds = {})
        : members_{members}, monitor_{thresholds} {
        SHULIB_PRECONDITION(!members_.empty(), "MotorGroup: members is empty (N >= 1 required)");
        SHULIB_PRECONDITION(members_.size() <= kMaxMembers, "MotorGroup: too many members");
        for (const IMotor* m : members_) {
            SHULIB_PRECONDITION(m != nullptr, "MotorGroup: a member is null");
        }
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
            m->setVoltage(volts);
        }
    }

    /// The group's last applied command after the ±kMaxMotorVoltage clamp — a local mirror
    /// (the ProsMotor convention), never a member read-back. 0 V until the first setVoltage().
    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    /// Fan the mode to every member.
    void setBrakeMode(BrakeMode mode) override {
        for (IMotor* m : members_) {
            m->setBrakeMode(mode);
        }
    }

    /// The FIRST member's read-back (the device's answer, per the IMotor contract).
    [[nodiscard]] BrakeMode brakeMode() const override { return members_[0]->brakeMode(); }

    /// MEDIAN of the members' cumulative shaft rotation (header: why not the mean).
    [[nodiscard]] units::AngleDim position() const override {
        return units::AngleDim{medianOf([](const IMotor& m) { return m.position().value(); })};
    }

    /// MEDIAN of the members' measured angular velocity.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        return units::AngularVelocity{
            medianOf([](const IMotor& m) { return m.velocity().value(); })};
    }

    /// MEAN current PER MEMBER, so per-motor thresholds keep their meaning; the sum is
    /// totalCurrent().
    [[nodiscard]] units::Current current() const override {
        return units::Current{sumCurrent() / static_cast<double>(members_.size())};
    }

    /// MAX member temperature (°C): the hottest member throttles first.
    [[nodiscard]] double temperature() const override {
        double worst = members_[0]->temperature();
        for (const IMotor* m : members_) {
            worst = std::max(worst, m->temperature());
        }
        return worst;
    }

    /// The SUM of every member's current draw (amperes) — the side's total, exposed outside
    /// the IMotor surface so current() can stay per-member.
    [[nodiscard]] units::Current totalCurrent() const { return units::Current{sumCurrent()}; }

    /// One tick of the disagreement observable: samples every member's velocity() against
    /// the group's commanded voltage through the shared coupled-side monitor, advancing its
    /// persistence counters ONCE. Call exactly once per control tick — the health tick
    /// (motion::tickHealthObservables) does. Returns the verdict it also stores.
    const teleop::SideVerdict& evaluateDisagreement() {
        teleop::MemberSample samples[kMaxMembers];
        const std::size_t n = members_.size();
        for (std::size_t i = 0; i < n; ++i) {
            samples[i] = teleop::MemberSample{members_[i]->velocity().value(), true};
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

    /// How many members the group commands.
    [[nodiscard]] std::size_t memberCount() const noexcept { return members_.size(); }

    /// Member `i` (0-based, the span's order). Precondition: i < memberCount().
    [[nodiscard]] IMotor& member(std::size_t i) const {
        SHULIB_PRECONDITION(i < members_.size(), "MotorGroup::member: index out of range");
        return *members_[i];
    }

    /// The thresholds the disagreement observable runs under.
    [[nodiscard]] const teleop::SideMonitorConfig& thresholds() const noexcept {
        return monitor_.config();
    }

private:
    template <typename Read>
    [[nodiscard]] double medianOf(Read read) const {
        double v[kMaxMembers];
        const std::size_t n = members_.size();
        for (std::size_t i = 0; i < n; ++i) {
            v[i] = read(*members_[i]);
        }
        std::sort(v, v + n);
        // Odd N: the middle value, no arithmetic (N == 1 is the bare motor, bit for bit).
        // Even N: the mean of the two middle values (exact when they agree).
        return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }

    [[nodiscard]] double sumCurrent() const {
        double sum = 0.0;
        for (const IMotor* m : members_) {
            sum += m->current().value();
        }
        return sum;
    }

    std::span<IMotor* const> members_;
    teleop::CoupledSideMonitor monitor_;
    teleop::SideVerdict verdict_{};
    units::Voltage commanded_{0.0};
};

}  // namespace shulib::hal
