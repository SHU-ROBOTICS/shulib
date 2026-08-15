#pragma once
//
// IDigitalOut — a single digital output line behind the HAL (chunk F1, WS7/M4):
// a pneumatic solenoid on the ADI ports, or any other two-state actuator. The
// first actuation seam in the tree that is not a motor — the H-drive's primary
// mechanism is a pneumatic clamp (master plan §14), so a mechanism layer shaped
// only around motors would have been shaped by half the hardware.
//
// Contract:
//  * set() commands the line; commanded() returns the value actually COMMANDED —
//    the same telemetry-reflects-the-command rule as IMotor::commandedVoltage().
//  * There is NO feedback channel, deliberately. A V5 solenoid cannot report
//    whether the cylinder moved: air can be exhausted, a linkage can bind, and
//    the line reads exactly what it was told either way. That physical fact is
//    why the manipulation layer's ActuateAndConfirm carries an Unconfirmed
//    verdict — confirmation must come from a SEPARATE sensor (current, distance,
//    optical), never from this seam pretending to know.
//  * Like the always-valid sensors (F4 note, master plan §7): no validity
//    signal. A dead ADI port is indistinguishable from a working one here;
//    detecting the difference is a confirmation sensor's job.
//
// NOT part of the F4 freeze (that register row locked 2026-06-19 with the ten
// runtime interfaces; this seam is F1's addition, outside it — register row F11
// records the non-freeze). The hal/pros implementation (ADI digital out) is R1's.

namespace shulib::hal {

/// A single two-state output line — a pneumatic solenoid on the ADI ports, or any other on/off
/// actuator. It is the first actuation seam here that is not a motor, because the H-drive's
/// primary mechanism is a pneumatic clamp. There is NO feedback channel and no validity signal,
/// both deliberate: a V5 solenoid cannot report whether the cylinder actually moved (air can be
/// exhausted, a linkage can bind) and a dead ADI port reads exactly like a working one.
/// Confirmation must come from a SEPARATE sensor — current, distance, optical — which is why the
/// manipulation layer carries an Unconfirmed verdict rather than letting this seam pretend to know.
class IDigitalOut {
public:
    /// Polymorphic-base boilerplate: the destructor is virtual so a concrete solenoid held as
    /// `IDigitalOut&`/`IDigitalOut*` destroys correctly, and DECLARING it is what suppresses the
    /// implicit copy/move, which are re-defaulted below. The base holds no state of its own.
    /// Ownership stays with the caller either way: PneumaticMechanism takes a NON-OWNING
    /// `span<IDigitalOut* const>` (non-empty, every line checked non-null at construction), so
    /// each line must outlive the mechanism that fans commands out to it.
    virtual ~IDigitalOut() = default;
    IDigitalOut() = default;
    IDigitalOut(const IDigitalOut&) = default;
    IDigitalOut(IDigitalOut&&) = default;
    IDigitalOut& operator=(const IDigitalOut&) = default;
    IDigitalOut& operator=(IDigitalOut&&) = default;

    /// Command the line high (true) or low (false). What "true" means physically
    /// (clamp closed? cylinder extended?) is plumbing-dependent and belongs to
    /// the mechanism that owns the line, not to this seam.
    virtual void set(bool value) = 0;

    /// The value actually commanded (readback of the command, NOT of the world —
    /// see the no-feedback note in the header).
    [[nodiscard]] virtual bool commanded() const = 0;
};

}  // namespace shulib::hal
