#pragma once
//
// The mechanism device seam (chunk F1, WS7/M4): IMechanism + the two concrete
// compositions every VEX mechanism reduces to at the device level — a group of
// motors on one shaft (MotorMechanism) and a set of digital lines switching one
// pneumatic circuit (PneumaticMechanism).
//
// ── What the interface is, and why it is this small ─────────────────────────────────
// IMechanism's virtual surface is exactly two members: applySafeState() and
// name(). That is deliberate, and it is the answer to "does the interface earn
// its existence": the ONE operation every mechanism supports uniformly —
// regardless of whether it is motors or air — is "put yourself in your declared
// safe state, NOW, synchronously". F2's guaranteed end-of-run park guard must be
// able to walk a heterogeneous list of mechanisms it knows nothing about and
// force every one of them safe at the buzzer; that requires a common base and
// nothing else does. Command-and-read surfaces are NOT unified here because the
// physics is not unified: a voltage command on a solenoid would be a lie, and a
// "command a double" abstraction over both would be a worse one. Code that
// commands a mechanism holds the concrete type; code that only needs "make it
// safe" (F2's park guard, a panic stop) holds IMechanism*.
//
// Rejected alternative — a fat IMechanism with setCommand(double)/read():
// it adds a vtable and a document without adding a capability (the anti-
// abstraction test in the F1 brief), and it bakes the motor shape into the seam
// the pneumatic clamp then has to fit through.
//
// ── The declared safe state (T4) ────────────────────────────────────────────────────
// The drivetrain's cancel safe state (0 V + Brake, motion.hpp) is defined once
// for ALL drive motors because a drivetrain is one thing. Mechanisms are not:
//   * a loaded lift at 0 V + Coast DROPS ITS STACK — its safe state is Hold;
//   * a jammed intake commanded Hold sits at stall current until the thermal
//     fault fires (~55 °C, motor.hpp) — its safe state is Coast or Brake.
// So there is NO library-wide default that is safe for both, and this header
// refuses to pick one: the safe state is DECLARED, per mechanism, at
// construction, and applySafeState() applies that declaration. Every stop path
// in the manipulation layer (operation exit, cancel, failure) and F2's park
// guard land here, so the declaration is applied by construction, not by
// convention. Whether BrakeMode::Hold actually holds a LOADED cascade lift is a
// hardware claim no host test can verify: PROVISIONAL (A4: HA-92).
//
// ── The operation claim (T3) ────────────────────────────────────────────────────────
// Two mechanisms running at once is required (intake while the lift settles).
// Two OPERATIONS driving ONE mechanism is a collision — the same argument that
// made one-active-motion structural at C2. The claim token below makes it
// structural here: an operation's start() takes the claim or trips a loud
// precondition; every operation exit releases it. It is a plain flag, not a
// mutex — single-task by contract like everything in this library. Pre-empt-
// then-replace (C2's policy) is deliberately NOT built in at this level: a
// sequencing layer that wants pre-emption cancels the old operation first
// (cancel releases the claim), which keeps the policy where the policy-owner
// lives (F2) and keeps a silent double-drive impossible everywhere.
//
// ── The claimant hook (chunk F2 — the gap its measurements exposed) ─────────────────
// F1 promised the end-of-run guard a span<IMechanism*> it could force safe.
// Building that guard found the promise short by one capability: the claim
// said THAT a mechanism was driven but not BY WHAT, so a stalled operation
// was unreachable from the guard — and applySafeState() alone lasts exactly
// until the live operation's next tick re-commands its voltage (measured:
// the re-command restores voltage but not brake mode, leaving the half-safe
// `brake=Hold, V=9.0` that passes any mode-only assertion). Worse, the
// unreleased claim makes the END ACTION's own operation throw at start().
// So the claim now carries an optional ICancellable: an operation that
// registers itself is reachable — the guard cancels it (inert + safe +
// claim released) instead of merely repainting the device state it will
// overwrite. tryClaim() without a claimant stays legal (F1 tests, third-party
// ops) but is INVISIBLE to the guard's cancel-all, which can then only
// force-release the claim and warn; register a claimant if an end-of-run
// guard must be able to stop your operation.
//
// ── Portability ─────────────────────────────────────────────────────────────────────
// The concrete compositions are written over the L0 seams (IMotor*/IDigitalOut*),
// so they run unchanged on hal/fake (host tests), hal/pros (R1 implements the
// devices, not the mechanisms) and hal/sim (H2). What a mechanism MEANS — which
// motors, which safe state, what confirms an action — stays with the team that
// built the robot; this file owns only the device grammar. No game semantics
// here, per the hal/vision.hpp house rule (classId is an opaque int for the
// same reason).

#include <algorithm>
#include <cmath>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/digital_out.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// The hal-level face of "whatever is currently driving a mechanism" — exactly
/// the ONE capability an end-of-run guard needs from a claimant it knows
/// nothing about: stop, synchronously, into the mechanism's declared safe
/// state, and become inert (further ticks are no-ops). Declared HERE, below
/// the manipulation layer, so IMechanism's claim token can carry it without
/// an upward include; manipulation::IMechanismOp implements it (its cancel()
/// contract is already exactly this). See the file banner's claimant-hook
/// section for the measured failure this closes.
class ICancellable {
public:
    /// Re-declared only because the virtual destructor suppresses the implicit copy/move
    /// members; this seam holds no state of its own. The virtual destructor is what makes
    /// deleting through a stored ICancellable* well-defined. Note that a claimant is
    /// registered BY ADDRESS (IMechanism::tryClaim below), so an implementer that holds a
    /// claim should DELETE its own copy/move instead of inheriting these defaults — a copy
    /// would leave the mechanism's registration aimed at the original. Manipulation's
    /// operations do exactly that.
    virtual ~ICancellable() = default;
    ICancellable() = default;
    ICancellable(const ICancellable&) = default;
    ICancellable(ICancellable&&) = default;
    ICancellable& operator=(const ICancellable&) = default;
    ICancellable& operator=(ICancellable&&) = default;

    /// Render the claimant inert and its mechanism safe, now. Idempotent;
    /// never raises (the IMechanismOp cancel contract).
    virtual void cancel() = 0;
};

/// The minimal common surface of every mechanism: a declared safe state that
/// can be forced from outside, a stable name for logs, and the one-operation
/// claim token. See the file banner for why nothing else is unified.
class IMechanism {
public:
    /// NON-COPYABLE AND NON-MOVABLE, and unlike the other HAL seams that is about state
    /// rather than style: this base HOLDS the claim token. While copy/move were defaulted, a
    /// copied mechanism arrived already claimed(), with claimant() aimed at an operation
    /// registered against the ORIGINAL — so a legitimate tryClaim(copy) failed for no reason
    /// the caller could see, and F2's end-of-run guard walking a span containing the copy
    /// reached claimant() and cancelled an operation driving the original, whose own claim
    /// was never released. That is exactly the unreleased-claim failure the claimant hook
    /// exists to close, reintroduced by a defaulted special member.
    ///
    /// manipulation/mechanism_op.hpp already deletes copy/move on both operations for the
    /// mirror-image reason ("the claim is a resource and the mechanism's registered claimant
    /// points at THIS object"); the mechanism side simply never got the same treatment.
    /// Construct a mechanism once where it lives and hand out IMechanism&/IMechanism*.
    virtual ~IMechanism() = default;
    IMechanism() = default;
    IMechanism(const IMechanism&) = delete;
    IMechanism(IMechanism&&) = delete;
    IMechanism& operator=(const IMechanism&) = delete;
    IMechanism& operator=(IMechanism&&) = delete;

    /// Command the DECLARED safe state, synchronously — safe when the call
    /// returns, no further tick required (the same synchronous rule as the
    /// scheduler's cancel path: a safe state that depends on someone continuing
    /// to tick can leave things energized). Idempotent; callable at any time,
    /// including while an operation is running (F2's park guard does exactly
    /// that — it does not ask permission at the buzzer).
    virtual void applySafeState() = 0;

    /// Stable short name for logs / fault details (a stable literal — stored,
    /// not copied, like Routine's name).
    [[nodiscard]] virtual const char* name() const noexcept = 0;

    // ── the operation claim (banner: one operation per mechanism, structural) ──────
    // Non-virtual on purpose: no implementation can get the token wrong.

    /// Take the claim ANONYMOUSLY. False if another operation already holds
    /// it. An anonymous claim is invisible to F2's end-of-run cancel-all
    /// (banner: the claimant hook) — prefer the registering overload.
    [[nodiscard]] bool tryClaim() noexcept {
        if (claimed_) {
            return false;
        }
        claimed_ = true;
        return true;
    }

    /// Take the claim AND register the claimant, so an end-of-run guard
    /// holding only IMechanism* can reach the operation and cancel it (chunk
    /// F2). `claimant` must stay valid until the claim is released — every
    /// operation exit path releases, and since F2 the library operations also
    /// cancel-on-destruction, so a registered pointer cannot dangle.
    [[nodiscard]] bool tryClaim(ICancellable& claimant) noexcept {
        if (!tryClaim()) {
            return false;
        }
        claimant_ = &claimant;
        return true;
    }

    /// Release the claim (no-op if not held — release is always safe).
    void releaseClaim() noexcept {
        claimed_ = false;
        claimant_ = nullptr;
    }

    /// True while an operation holds the claim.
    [[nodiscard]] bool claimed() const noexcept { return claimed_; }

    /// The registered claimant, or nullptr (unclaimed, or claimed anonymously
    /// via the parameterless tryClaim). The end-of-run guard's reach.
    [[nodiscard]] ICancellable* claimant() const noexcept { return claimant_; }

private:
    bool claimed_ = false;
    ICancellable* claimant_ = nullptr;
};

/// N motors on one mechanically coupled shaft (an intake's two motors, a lift's
/// pair), commanded as one. The SAME voltage goes to every motor: direction
/// reversal is a device-level fact (the pros adapter owns it, exactly as it
/// owns mA→A), so by the time a motor reaches this seam "+V" already means
/// "forward" for that motor.
class MotorMechanism : public IMechanism {
public:
    /// `motors` (non-empty, all non-null) must outlive the mechanism; `safe` is
    /// the DECLARED safe brake mode (banner: Hold for a loaded lift, Coast or
    /// Brake for an intake — there is no correct default, so there is no
    /// default). `mechName` must be a stable literal.
    MotorMechanism(std::span<IMotor* const> motors, BrakeMode safe, const char* mechName)
        : motors_{motors}, safe_{safe}, name_{mechName} {
        SHULIB_PRECONDITION(!motors_.empty(), "MotorMechanism: motors is empty");
        for (const IMotor* m : motors_) {
            SHULIB_PRECONDITION(m != nullptr, "MotorMechanism: a motor is null");
        }
        SHULIB_PRECONDITION(mechName != nullptr, "MotorMechanism: name is null");
    }

    /// Command every motor (clamped/validated by the IMotor contract).
    void setVoltage(units::Voltage volts) {
        for (IMotor* m : motors_) {
            m->setVoltage(volts);
        }
    }

    /// The voltage the DEVICE actually got, read back from the first motor —
    /// never this object's own record of what it thinks it commanded (the
    /// bottom-of-the-stack rule every F1 test also follows). All motors are
    /// commanded identically through this seam, so one readback speaks for the
    /// group.
    [[nodiscard]] units::Voltage commandedVoltage() const {
        return motors_.front()->commandedVoltage();
    }

    /// The declared safe state: safe brake mode on every motor, THEN zero
    /// volts, so the stop lands under the declared semantics and never a
    /// momentary coast — the same ordering applyCancelSafeState() documents.
    void applySafeState() override {
        for (IMotor* m : motors_) {
            m->setBrakeMode(safe_);
            m->setVoltage(units::Voltage{0.0});
        }
    }

    /// The `mechName` pointer given at construction, returned verbatim — this class
    /// BORROWS the string and never copies it, so the literal must outlive the mechanism.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

    /// The declared safe brake mode (construction-time fact, for tests/logs).
    [[nodiscard]] BrakeMode safeBrakeMode() const noexcept { return safe_; }

    /// Highest per-motor current draw — the jam/stall signal (motor.hpp calls
    /// current() "the PRIMARY capture/stall signal for manipulation
    /// sensor-confirm"). Max, not mean: a jam shows on the most loaded motor.
    [[nodiscard]] units::Current maxCurrent() const {
        double amps = 0.0;
        for (const IMotor* m : motors_) {
            amps = std::max(amps, std::abs(m->current().value()));
        }
        return units::Current{amps};
    }

    /// Mean output-shaft angular velocity across the group (one coupled shaft,
    /// so the mean IS the shaft; signed, so a direction fact survives).
    [[nodiscard]] units::AngularVelocity meanVelocity() const {
        double sum = 0.0;
        for (const IMotor* m : motors_) {
            sum += m->velocity().value();
        }
        return units::AngularVelocity{sum / static_cast<double>(motors_.size())};
    }

    /// The devices themselves — for readers this grammar does not cover
    /// (per-motor position for F3's liftToLevel homing, temperatures). Handing
    /// out the seam rather than wrapping every reader keeps this class honest
    /// about what it is: a command fan-out with a declared safe state.
    [[nodiscard]] std::span<IMotor* const> motors() const noexcept { return motors_; }

private:
    std::span<IMotor* const> motors_;
    BrakeMode safe_;
    const char* name_;
};

/// One pneumatic circuit behind N digital lines (a clamp's solenoid, a pair of
/// deploy cylinders fired together), commanded as one. The declared safe value
/// is per-mechanism for the same reason the brake mode is (T4): whether "safe
/// at the buzzer" means clamp-closed (keep the goal) or cylinder-retracted
/// (inside expansion limits) is a fact about the robot, not about the library.
class PneumaticMechanism : public IMechanism {
public:
    /// `lines` (non-empty, all non-null) must outlive the mechanism; `safe` is
    /// the DECLARED safe command. `mechName` must be a stable literal.
    PneumaticMechanism(std::span<IDigitalOut* const> lines, bool safe, const char* mechName)
        : lines_{lines}, safe_{safe}, name_{mechName} {
        SHULIB_PRECONDITION(!lines_.empty(), "PneumaticMechanism: lines is empty");
        for (const IDigitalOut* d : lines_) {
            SHULIB_PRECONDITION(d != nullptr, "PneumaticMechanism: a line is null");
        }
        SHULIB_PRECONDITION(mechName != nullptr, "PneumaticMechanism: name is null");
    }

    /// Command every line.
    void set(bool value) {
        for (IDigitalOut* d : lines_) {
            d->set(value);
        }
    }

    /// The command the DEVICE actually got (first line's readback — the same
    /// bottom-of-stack rule as MotorMechanism::commandedVoltage). Remember what
    /// this is NOT (digital_out.hpp): evidence that anything moved.
    [[nodiscard]] bool commanded() const { return lines_.front()->commanded(); }

    /// The declared safe state: every line driven to `safe`. ONE command, not the motor
    /// version's brake-then-zero two-step — a solenoid has no coast phase to slip through.
    /// Still only a command: nothing here is evidence the air actually moved.
    void applySafeState() override { set(safe_); }

    /// The `mechName` pointer given at construction, returned verbatim — BORROWED, never
    /// copied, so the literal must outlive the mechanism.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

    /// The declared safe command (construction-time fact, for tests/logs).
    [[nodiscard]] bool safeCommand() const noexcept { return safe_; }

    /// The lines themselves, in the construction order — for anything this fan-out
    /// grammar does not cover (driving one cylinder of a pair alone on a bench check).
    /// NON-OWNING, like the span it was built from: the caller still owns every line.
    [[nodiscard]] std::span<IDigitalOut* const> lines() const noexcept { return lines_; }

private:
    std::span<IDigitalOut* const> lines_;
    bool safe_;
    const char* name_;
};

}  // namespace shulib::hal
