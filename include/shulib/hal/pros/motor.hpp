#pragma once
//
// ProsMotor — IMotor over pros::Motor (chunk R1a).
//
// BINDS:
//  * move_voltage(mV)          ← setVoltage()   (motor_conversion.hpp, HA-94)
//  * brake()                   ← setVoltage() at 0 mV under a commanded Brake/Hold, and
//                                setBrakeMode() while stopped (DEFECTS2 — below)
//  * get_position() [degrees]  → position()     (motorPositionDegToCanonical, HA-95)
//  * get_actual_velocity() RPM → velocity()     (motorRpmToCanonical, HA-96)
//  * get_current_draw() mA     → current()      (motorMilliampsToCanonical, HA-97)
//  * get_temperature() °C      → temperature()  (identity — already canonical)
//  * set/get_brake_mode        ↔ setBrakeMode()/brakeMode() (mapped both ways below)
//
// THE STOP — HOW A BRAKE MODE REACHES THE MOTOR (chunk DEFECTS2, 2026-09-26;
// A4 register HA-142): PROS applies a brake mode ONLY through brake(). move_voltage()
// "will not respect brake modes, and simply sets the voltage" (vendored motors.h:253-254;
// the same note on motor_move, :52); brake() "stops the motor using the currently
// configured brake mode ... sets motor velocity to zero, which will cause it to act
// according to the set brake mode" (motors.hpp:260-284). Until DEFECTS2 this adapter only
// ever called move_voltage(), so every stop in the library — a mechanism's declared Hold, the
// drive's "0 V + Brake" cancel — was a 0 V command whose mode, by PROS's own documentation, the
// motor ignored: on a V5, Hold went limp and Brake coasted (reasoned from that text — never
// observed), while every host test passed (the shim had no brake(), the fakes echo the mode,
// and the A2 plant pins a wheel at 0 V). The rule now:
//   * setVoltage() whose MILLIVOLTS round to 0 (the value SENT — a 0.4 mV command is a stop,
//     never keyed on the raw double), under a mode THIS PROGRAM commanded through
//     setBrakeMode() that is Brake or Hold ⇒ brake(). Issued on the TRANSITION into that stop
//     only: a zero command repeated under the same mode (a live operation re-sends its volts
//     every tick; DriveBrake re-commands Brake + 0 V every tick) sends nothing more, because a
//     repeated brake() under Hold may re-capture the held position and let a loaded shaft creep
//     (A4 register HA-144, invented).
//   * Coast, or NO mode commanded yet ⇒ move_voltage(0) on every call, exactly as before
//     DEFECTS2. Calibrate, the old shulib Drive (until T4 Part 3) and the tester's DRIVE station
//     set Coast and their measured behaviour rests on this 0 V path (what a motor physically does
//     at 0 V is A4 register HA-143 — unknown).
//   * setBrakeMode() while the last command was 0 mV RE-ISSUES the stop under the new mode, so
//     the order of the two calls never matters: Hold-then-0 V and 0 V-then-Hold both hold, and
//     Hold-then-0 V-then-Coast leaves the motor at 0 V as Coast always meant.
//   * Any non-zero command leaves the stop (move_voltage replaces it — A4 register HA-146).
//   * A call the port REFUSES (PROS_ERR — a dropout) is never believed. brake() stops under
//     whatever the device's REGISTER holds, so a set_brake_mode() the port refused is re-sent
//     before the next brake(), and the stop is recorded only when BOTH were accepted; until
//     then every zero command retries. A caller that sends its zero ONCE (applySafeState() at
//     an operation's exit, T3's binding) gets no retry if the port is down at that moment —
//     the limit every refused command has had, move_voltage(0) included.
// KEYED ON THE COMMANDED MODE, NEVER ON brakeMode_: the ctor SEEDS brakeMode_ from the device
// (the T7 fallback, below), and a drive motor an earlier program left in Hold would otherwise
// brake() at centred sticks in a program that never asked for Hold. A motor this program never
// gave a mode stays on move_voltage(0) at zero, whatever the device remembers (HA-98).
// The stop record is per ADAPTER OBJECT: two ProsMotors on one port (IMotor's copy semantics)
// each believe only their own history, and nothing here can see a VEXos disable
// (A4 register HA-145): after a re-enable, a zero repeated under the same mode sends nothing new.
// The fix lives HERE, below every wrapper (MotorGroup, the hostile fakes, test wrappers), so
// each inherits it; IMotor's signature is unchanged (F4) — no stop verb was added above.
//
// WHY THE CTOR DEMANDS A GEARSET AND SETS UNITS EXPLICITLY (trap A, HA-98):
// pros::Motor's ctor defaults are MotorGears::invalid / MotorUnits::invalid,
// which mean "leave the device as it is" — get_position() then returns
// whatever encoder units that motor's firmware was last told, possibly by a
// different program on a different day. A motor left in rotations returns
// 1/360 of what this adapter believes, odometry is silently wrong by 360×,
// and nothing crashes. So this ctor:
//   1. passes an EXPLICIT gearset (no default parameter here — the cartridge
//      color is a physical fact of the built robot; velocity scaling depends
//      on it, so the adapter must not guess), and
//   2. explicitly sets MotorUnits::degrees, and
//   3. READS BOTH BACK and raises SHULIB_PRECONDITION if the device disagrees
//      — a motor that ignores configuration is a miswired robot, and finding
//      that at boot beats finding it mid-match.
//
// REVERSAL: a NEGATIVE port reverses the motor, applied by PROS itself —
// exactly once, there. This adapter never negates on top.
//
// SENTINELS (T7): IMotor has no validity channel, so PROS_ERR / PROS_ERR_F
// from a read is screened HERE: hold the last good value, never propagate
// (breaks the F4 finiteness contract A3's hostility models as a bug), never
// substitute zero (a zeroed encoder reads as "the robot stopped" — exactly
// the dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch,
// and a zero would make it plausible instead of visible). A frozen position
// is what the loop's wheels-spin-but-no-motion cross-check is DESIGNED to
// see (health_monitor.hpp odomStalled); faultedReads() is exposed for
// telemetry. Raising the fault itself stays with the loop layer — hal/ is
// below diag/ and raising is policy (health_monitor.hpp header).
//
// NON-FINITE COMMAND (L4): rejected via SHULIB_PRECONDITION, exactly like
// FakeMotor (fake_motor.hpp:18-24) — NOT coerced to 0. The measurement
// prototype for this brief made that mistake; it compiles, passes a clamp
// test, and turns a programming error into a robot that coasts.
//
// HA register: HA-94..HA-98, HA-142..HA-146 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/error.h"
#include "pros/motors.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>
#include <optional>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// The drive cartridge in the physical motor — a construction fact the caller
/// must state (see header: velocity scaling is gearset-dependent, so there is
/// deliberately no default).
enum class MotorGearset {
    Red,    ///< 36:1, 100 RPM
    Green,  ///< 18:1, 200 RPM
    Blue,   ///< 6:1, 600 RPM
};

/// IMotor over a real pros::Motor — the adapter that converts V5 units to canonical ones
/// exactly once, at the edge. Four contracts a caller should know: the constructor CONFIGURES
/// the device (degrees + the stated gearset) and reads both back, so a miswired port fails at
/// boot rather than mid-match; a NEGATIVE port reverses the motor inside PROS and this adapter
/// never negates on top of that; a ZERO command under a Brake or Hold mode that this program
/// commanded is sent as PROS brake() — the one call that applies a mode — while Coast, or no
/// commanded mode, stays a 0 V command (header: THE STOP; before DEFECTS2 every zero was 0 V and
/// no mode ever reached the motor); and the five screened READS — position(), velocity(),
/// current(), temperature() and brakeMode() — handle the PROS_ERR / PROS_ERR_F / `invalid`
/// sentinels by HOLDING THE LAST GOOD VALUE (for brakeMode(), the last mode this adapter
/// commanded or seeded) rather than propagating a sentinel or substituting zero, because a
/// frozen reading is what the loop's odometry cross-checks are built to notice while a zeroed
/// encoder would look exactly like a robot that stopped. faultedReads() counts every one of
/// those five screens. commandedVoltage() sits outside the scheme: a local mirror of the last
/// applied command that touches no device. This adapter does not raise faults; raising is the
/// loop layer's policy.
class ProsMotor final : public IMotor {
public:
    /// `port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once).
    /// `gearset`: the physical cartridge color. The ctor configures the device
    /// (degrees + gearset) and READS BOTH BACK — a disagreeing device raises
    /// SHULIB_PRECONDITION (header: why boot-loud beats match-silent).
    ProsMotor(std::int8_t port, MotorGearset gearset)
        : motor_{port, toProsGears(gearset), ::pros::v5::MotorUnits::degrees} {
        // Read-back (HA-98): the ctor above SET both; a healthy device echoes
        // them. `invalid` here means the port did not answer — also a failure.
        SHULIB_PRECONDITION(motor_.get_encoder_units() == ::pros::v5::MotorUnits::degrees,
                            "ProsMotor: encoder units read-back disagrees (device did not "
                            "accept degrees — wrong port, or not a motor?)");
        SHULIB_PRECONDITION(motor_.get_gearing() == toProsGears(gearset),
                            "ProsMotor: gearing read-back disagrees (device did not accept "
                            "the configured gearset — wrong port, or not a motor?)");
        // SEED THE T7 FALLBACK FROM THE DEVICE. The ctor deliberately does NOT command a
        // brake mode — unlike ProsDigitalOut, construction here is not a physical action, and
        // choosing a mode for a mechanism the adapter knows nothing about is not its call. But
        // brakeMode_ is also what brakeMode() serves when the device read comes back invalid,
        // and seeding it to a GUESS meant the fallback could contradict the device from boot:
        // a port left in Hold by the previous program, dying before any setBrakeMode(), used
        // to report Coast forever. The device's own answer is the only honest starting value;
        // if the port cannot answer even now, the guess is all there is and Coast stands.
        const auto bootMode = motor_.get_brake_mode();
        if (bootMode != ::pros::v5::MotorBrake::invalid) {
            brakeMode_ = fromProsBrake(bootMode);
        }
    }

    /// Clamp to ±kMaxMotorVoltage, REJECT non-finite (never coerce — L4), send as millivolts
    /// (HA-94). commandedVoltage() is the clamped volts, UNROUNDED, however the command was sent
    /// — so a sub-millivolt command reads back as itself (0.0004 V) although the wire sent a
    /// stop. When the millivolts SENT are 0 and this program's last setBrakeMode() was Brake
    /// or Hold, the stop goes out as PROS brake() instead of move_voltage(0) — once, on the way
    /// INTO the stop; a zero repeated under the same mode sends nothing more (header: THE STOP,
    /// HA-142/HA-144). Coast, or no mode commanded yet, sends move_voltage(0) every time, as
    /// before.
    void setVoltage(units::Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "ProsMotor::setVoltage: voltage must be finite");
        const std::int32_t millivolts = motorVoltageToMillivolts(volts);
        commanded_ = motorVoltageApplied(volts);
        atZero_ = millivolts == 0;  // keyed on what is SENT, never on the raw double
        if (atZero_) {
            stopAtZero(/*fromModeChange=*/false);
            return;
        }
        motor_.move_voltage(millivolts);
        stoppedUnder_.reset();  // a driving command replaces any stop (HA-146)
    }

    /// The voltage this adapter last APPLIED, after the ±kMaxMotorVoltage clamp. A locally
    /// mirrored value computed by the same clamp as the millivolt command, NOT a read-back
    /// from the device — so it can never disagree with what went on the wire, and it never
    /// faults. 0 V until the first setVoltage().
    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    /// Send the brake mode to the device AND remember it twice: as the value brakeMode() falls
    /// back to when the device's own read comes back invalid (T7), and as the mode THIS PROGRAM
    /// commanded, which is what decides how a zero command is sent (header: THE STOP). If the
    /// last command was a zero, the stop is RE-ISSUED under the new mode — brake() for Brake or
    /// Hold (unless already stopped under that very mode), move_voltage(0) for Coast when the
    /// motor was in a brake() stop — so mode-then-zero and zero-then-mode end the same way.
    /// Whether the port ACCEPTED the write is kept as well: a refused one is re-sent before the
    /// next brake(), which stops under the device's register (header: THE STOP).
    /// The ctor configures encoder units and gearing but never a brake mode, and brakeMode()
    /// reads the DEVICE first — so until this is called at least once, the value you observe is
    /// whatever mode the motor is still holding from an earlier program (the persistent-device-
    /// state trap the header describes for encoder units), and a zero command is sent as 0 V
    /// whatever that inherited mode is. Call this during setup if the mode has to be applied
    /// rather than inherited.
    void setBrakeMode(BrakeMode mode) override {
        brakeMode_ = mode;
        commandedMode_ = mode;
        modeOnDevice_ = motor_.set_brake_mode(toProsBrake(mode)) == 1;
        if (atZero_) {
            stopAtZero(/*fromModeChange=*/true);
        }
    }

    /// Reads the DEVICE back and maps to the canonical enum; an unreadable device
    /// (MotorBrake::invalid) holds the last commanded mode — or, before any command, the mode
    /// the ctor read from the device (T7) — and COUNTS a faulted read, like the other four
    /// screens. This reports the device's configured mode; it says nothing about whether the
    /// motor is currently stopped under it (header: THE STOP).
    [[nodiscard]] BrakeMode brakeMode() const override {
        const auto raw = motor_.get_brake_mode();
        if (raw == ::pros::v5::MotorBrake::invalid) {
            faultedReads_ += 1;  // COUNTED, like the other four screens
            return brakeMode_;   // screened: hold last commanded
        }
        return fromProsBrake(raw);
    }

    /// Cumulative output-shaft radians (never wrapped). Sentinel-screened:
    /// PROS_ERR_F holds the last good value (header, T7).
    [[nodiscard]] units::AngleDim position() const override {
        const double deg = motor_.get_position();
        if (!std::isfinite(deg)) {
            faultedReads_ += 1;
            return lastPosition_;
        }
        lastPosition_ = motorPositionDegToCanonical(deg);
        return lastPosition_;
    }

    /// Measured output-shaft angular velocity, canonical rad/s, converted from the device's
    /// RPM (HA-96). This is the DEVICE's reported velocity — this adapter never differentiates
    /// position() to get it. Sentinel-screened like position(): PROS_ERR_F holds the last good
    /// value and counts a faulted read; 0 rad/s before the first successful read.
    [[nodiscard]] units::AngularVelocity velocity() const override {
        const double rpm = motor_.get_actual_velocity();
        if (!std::isfinite(rpm)) {
            faultedReads_ += 1;
            return lastVelocity_;
        }
        lastVelocity_ = motorRpmToCanonical(rpm);
        return lastVelocity_;
    }

    /// Canonical amperes. PROS_ERR (INT32_MAX) is IN-BAND for the int32 mA
    /// read — only this adapter can screen it (motor_conversion.hpp note).
    [[nodiscard]] units::Current current() const override {
        const std::int32_t ma = motor_.get_current_draw();
        if (ma == PROS_ERR) {
            faultedReads_ += 1;
            return lastCurrent_;
        }
        lastCurrent_ = motorMilliampsToCanonical(static_cast<double>(ma));
        return lastCurrent_;
    }

    /// Motor temperature in degrees Celsius — already canonical, so this is the one reader
    /// that applies no conversion. Sentinel-screened like the others (non-finite holds the
    /// last good value and counts a faulted read), but note the seed: before ANY successful
    /// read this returns 20 °C, not 0, so a port that has never answered reports a
    /// room-temperature motor. Check faultedReads() before trusting it as a thermal signal.
    [[nodiscard]] double temperature() const override {
        const double c = motor_.get_temperature();
        if (!std::isfinite(c)) {
            faultedReads_ += 1;
            return lastTemperature_;
        }
        lastTemperature_ = c;
        return lastTemperature_;
    }

    /// How many of the five screened READS (position/velocity/current/temperature/brakeMode)
    /// were screened to last-good — cumulative for the life of the object, never reset. T7
    /// observability: telemetry and the loop's health policy can see a flaky port without
    /// this seam growing a validity channel F4 does not have. brakeMode() has been counted
    /// since DEFECTS1 (I9/E1); this comment said otherwise until DEFECTS2. COMMANDS are not
    /// counted: a move_voltage() or brake() the port refuses leaves this number unchanged.
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    /// Send the stop for a zero command under the mode THIS PROGRAM commanded (header: THE
    /// STOP). `fromModeChange` = called from setBrakeMode(), re-issuing a stop already in force.
    ///   * Brake/Hold commanded: brake(), unless the motor is already in a brake() stop under
    ///     that very mode — the transition rule (HA-144). brake() stops under whatever the
    ///     device's REGISTER holds, so a set_brake_mode() the port refused is re-sent first; and
    ///     the stop is recorded only when PROS answered 1 to BOTH. A refusal of either (PROS_ERR)
    ///     leaves no record, so the next zero command retries instead of believing a stop that
    ///     never happened — or one that happened under the register's stale mode. brake() is
    ///     sent even when the mode write was refused: a stop under the wrong mode, not believed,
    ///     beats leaving the motor on whatever it was last driven at.
    ///   * Coast, or no mode commanded: move_voltage(0). From setVoltage() on every call (the
    ///     pre-DEFECTS2 wire traffic, unchanged); from setBrakeMode() only when leaving a brake()
    ///     stop, since a 0 V command already in force needs no repeat. The register does not
    ///     matter here: move_voltage() ignores it (HA-142).
    /// Adds no throwing path: setBrakeMode()/setVoltage(0) are what the scheduler's implicitly
    /// noexcept destructor calls (motion_scheduler.hpp).
    void stopAtZero(bool fromModeChange) {
        const bool applyMode = commandedMode_.has_value() && *commandedMode_ != BrakeMode::Coast;
        if (applyMode) {
            if (stoppedUnder_ != commandedMode_) {
                if (!modeOnDevice_) {
                    modeOnDevice_ = motor_.set_brake_mode(toProsBrake(*commandedMode_)) == 1;
                }
                const bool stopped = motor_.brake() == 1;
                if (stopped && modeOnDevice_) {
                    stoppedUnder_ = commandedMode_;
                } else {
                    stoppedUnder_.reset();
                }
            }
            return;
        }
        if (fromModeChange && !stoppedUnder_.has_value()) {
            return;  // already at 0 V under voltage control: nothing to re-issue
        }
        motor_.move_voltage(0);
        stoppedUnder_.reset();
    }

    [[nodiscard]] static ::pros::v5::MotorGears toProsGears(MotorGearset g) {
        switch (g) {
            case MotorGearset::Red: return ::pros::v5::MotorGears::red;
            case MotorGearset::Green: return ::pros::v5::MotorGears::green;
            case MotorGearset::Blue: return ::pros::v5::MotorGears::blue;
        }
        return ::pros::v5::MotorGears::green;  // unreachable; switch is exhaustive
    }

    [[nodiscard]] static ::pros::v5::MotorBrake toProsBrake(BrakeMode m) {
        switch (m) {
            case BrakeMode::Coast: return ::pros::v5::MotorBrake::coast;
            case BrakeMode::Brake: return ::pros::v5::MotorBrake::brake;
            case BrakeMode::Hold: return ::pros::v5::MotorBrake::hold;
        }
        return ::pros::v5::MotorBrake::coast;  // unreachable; switch is exhaustive
    }

    [[nodiscard]] static BrakeMode fromProsBrake(::pros::v5::MotorBrake m) {
        switch (m) {
            case ::pros::v5::MotorBrake::coast: return BrakeMode::Coast;
            case ::pros::v5::MotorBrake::brake: return BrakeMode::Brake;
            case ::pros::v5::MotorBrake::hold: return BrakeMode::Hold;
            case ::pros::v5::MotorBrake::invalid: break;
        }
        return BrakeMode::Coast;  // screened by the caller before this is reached
    }

    ::pros::v5::Motor motor_;
    units::Voltage commanded_{0.0};
    /// The T7 FALLBACK for brakeMode() — seeded from the DEVICE by the ctor, then the last
    /// commanded mode. Never consulted for the stop (header: THE STOP — the seeded-mode trap).
    BrakeMode brakeMode_ = BrakeMode::Coast;
    /// The mode THIS PROGRAM last commanded through setBrakeMode(); empty until the first call.
    /// The ONLY input to how a zero command is sent.
    std::optional<BrakeMode> commandedMode_;
    /// The port ACCEPTED the last set_brake_mode() of commandedMode_ (answered 1). False while
    /// a refused write has not yet been re-sent successfully — the stop then re-sends it before
    /// its brake(), because brake() stops under the register, not under commandedMode_.
    bool modeOnDevice_ = false;
    /// The last setVoltage() SENT 0 mV (false until the first command) — what makes
    /// setBrakeMode() re-issue the stop under its new mode.
    bool atZero_ = false;
    /// The motor is in a brake() stop issued under this mode, not yet replaced by a
    /// move_voltage(); empty otherwise. What makes a repeated zero send nothing.
    std::optional<BrakeMode> stoppedUnder_;
    // `mutable` because IMotor's readers are const but screening must remember
    // the last good value (T7's hold-last-good) — logically these caches are
    // part of the READ, not of the object's commanded state.
    mutable units::AngleDim lastPosition_{0.0};
    mutable units::AngularVelocity lastVelocity_{0.0};
    mutable units::Current lastCurrent_{0.0};
    mutable double lastTemperature_ = 20.0;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
