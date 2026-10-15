#pragma once
//
// IReadFaults — how many reads a SENSOR ADAPTER has SCREENED (chunk MECH-A1, 2026-10-08).
//
// ── An F4-ADDITIVE SIBLING, not a change to F4 ──────────────────────────────────────
// Exactly as IDigitalOut (F11), IController (F13) and IDigitalIn (F14) were added beside the
// locked F4 seams, this one is added beside them: IDistance, IOptical, IRotation and the rest are
// untouched, byte for byte. An adapter that screens its reads implements this interface AS WELL
// as its seam (hal::pros::ProsDistance, ProsOptical, ProsDigitalIn); a consumer that needs the
// count asks the device provider for it (IDeviceProvider::readFaults / lineReadFaults).
//
// ── WHY IT EXISTS (the MECH brief's re-check, its probe R) ──────────────────────────
// The locked sensor seams carry NO validity channel, by design: a PROS adapter that gets a
// PROS_ERR answers with its LAST GOOD value (hal/pros/distance.hpp's "T7" note; optical.hpp;
// digital_in.hpp) and counts the screen in its own faultedReads(). At the seam a held value is
// indistinguishable from a fresh one — and for a distance sensor a value held from a FAR reading
// (or from boot: the hold starts at the far no-object distance) is byte for byte the 9999
// "no object" reading. The mechanism core ends a step on "the piece is GONE"; acting on a held
// value there is a false success: the brief's probe R measured a distance sensor whose cable came
// out while the intake was empty, one dead from boot, and an optical that failed while empty —
// each ended an eject step SUCCEEDED "stopped seeing a piece" at its first tick with the motor
// never run. So the core reads this count BESIDE the value: a sample taken while the count moved
// is UNKNOWN, never evidence (the rule is the mechanism service's, MECH-A2).
//
// ── What it cannot see ──────────────────────────────────────────────────────────────
// Only what the adapter can: a read the device REFUSED. An UNPLUGGED 3-wire switch is not a
// refused read — the brain reads a level from an empty 3-wire port like any other (HA-158), and
// which level is HA-184's question. A count that never moves is therefore not proof a switch is
// connected.
//
// One member, no state of its own, no reset: the count only grows (each adapter's own rule).
// PROS-free, allocation-free.

namespace shulib::hal {

/// How many reads a sensor adapter has SCREENED — answered with its last good value because the
/// device failed (unplugged, a wrong port, a PROS_ERR). A consumer that must not act on a held value
/// remembers this count and compares it at its next read: a change means a value it was given in
/// between was held, not measured (header).
class IReadFaults {
public:
    /// Polymorphic base: the virtual destructor makes `delete` through an `IReadFaults*`
    /// well-defined. A consumer holds one by pointer and never owns it (the provider does).
    virtual ~IReadFaults() = default;
    /// Default-constructible: the base has no state.
    IReadFaults() = default;
    /// Copyable at the base, because the base holds no state; an adapter decides for itself.
    IReadFaults(const IReadFaults&) = default;
    /// Movable, same reasoning as the copy constructor.
    IReadFaults(IReadFaults&&) = default;
    /// Copy-assignable, same reasoning.
    IReadFaults& operator=(const IReadFaults&) = default;
    /// Move-assignable, same reasoning.
    IReadFaults& operator=(IReadFaults&&) = default;

    /// Reads screened since the adapter was constructed. MONOTONIC: it never decreases and is never
    /// reset, so "it moved" is the only question a consumer may ask of it.
    [[nodiscard]] virtual int faultedReads() const noexcept = 0;
};

}  // namespace shulib::hal
