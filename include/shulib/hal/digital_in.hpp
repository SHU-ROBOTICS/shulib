#pragma once
//
// IDigitalIn — a single digital input line behind the HAL (chunk R1b): a limit
// switch, a bumper, a jumper — any two-state sensor on an ADI port. The input
// sibling of IDigitalOut (digital_out.hpp), added by the same additive-sibling
// path (F1's IDigitalOut, R1a's IController).
//
// ── Built on an open question, and saying so ────────────────────────────────────────
// The lift-homing question (switch or stall?) was asked 2026-08-13 and is NOT
// answered. This seam exists on the "cheap now, expensive to discover at R3
// with the robot on the bench" ruling — it has NO consumer yet, which departs
// from F1's earned-interface standard deliberately and out loud. If the answer
// comes back "stall", this is a small unused sibling; that cost was accepted
// when the seam was ruled in. What keeps the shape honest: a digital input has
// exactly one degree of freedom, so there is nothing to guess at.
//
// Contract (each clause inherited from IDigitalOut's rulings, digital_out.hpp):
//  * state() is the RAW LEVEL, now. What "true" means physically (pressed?
//    released? beam broken?) is plumbing-dependent and belongs to the mechanism
//    that owns the line, never to this seam.
//  * NO debouncing, deliberately. A homing switch wants a different filter from
//    a collision bumper, and the adapter cannot know which it is serving —
//    a time constant baked in here would be an unmeasured constant in the one
//    layer with no idea what the switch is for. Debounce is the consumer's.
//  * NO edge detection, deliberately. "Was it JUST pressed" is per-consumer
//    state: reuse hal::ButtonEdge (controller.hpp) above the seam, exactly as
//    driver buttons do — never PROS's adi DigitalIn::get_new_press(), which
//    CONSUMES the event so a second consumer silently misses every press
//    (HA-121, the ADI sibling of HA-104).
//  * NO validity channel, like IDigitalOut: a dead ADI port is
//    indistinguishable from a working one at this seam. Detecting a dead
//    switch is a cross-check's job (e.g. homing travel limits), not this
//    seam's.
//
// NOT part of the F4 freeze (locked 2026-06-19 with the ten runtime
// interfaces). This is chunk R1b's addition, outside it — register row F14
// records the NON-freeze out loud (D2's lesson: silence in that register
// reads as "frozen"). Freeze trigger: a real consumer — F3's homing decision,
// if it comes back "switch".

namespace shulib::hal {

/// One digital input line behind the HAL — a limit switch, a bumper, a jumper: any two-state
/// sensor on an ADI port. The input sibling of IDigitalOut, and a seam with exactly one degree of
/// freedom, which is why three things are deliberately absent and belong to the CONSUMER instead:
///   * NO debouncing. A homing switch and a collision bumper want different filters, and this
///     layer cannot know which it is serving — a time constant here would be an unmeasured
///     constant chosen by the one layer with no idea what the switch is for.
///   * NO edge detection. "Was it JUST pressed" is per-consumer state; put a `hal::ButtonEdge`
///     above the seam, one per consumer, exactly as driver buttons do.
///   * NO validity channel. A dead ADI port is indistinguishable from a working one at this
///     level; catching a dead switch is a cross-check's job (homing travel limits), not this
///     seam's.
///
/// It is deliberately OUTSIDE the F4 runtime-interface freeze, and said out loud rather than left
/// to be inferred: it was added ahead of any consumer, while the lift-homing question (switch or
/// stall?) is still open, so it may still move. A real consumer is what would freeze it.
class IDigitalIn {
public:
    /// The rule-of-five set plus the default constructor — six `= default`s, none of which add
    /// behaviour, because this seam holds no state of its own. Spelled out in full because the
    /// suppression chain is easy to get wrong: a user-declared destructor suppresses only the
    /// implicit MOVE members (the copies survive it, their generation merely deprecated),
    /// re-declaring the moves is what would DELETE those copies, and declaring any constructor at
    /// all is what costs you the implicit default one. Writing only `virtual ~IFoo() = default;`
    /// therefore leaves a new interface still copyable, with every "move" of it quietly resolving
    /// to that copy. What copying a concrete IMPLEMENTATION means is that implementation's
    /// business — the interface makes no claim.
    virtual ~IDigitalIn() = default;
    IDigitalIn() = default;
    IDigitalIn(const IDigitalIn&) = default;
    IDigitalIn(IDigitalIn&&) = default;
    IDigitalIn& operator=(const IDigitalIn&) = default;
    IDigitalIn& operator=(IDigitalIn&&) = default;

    /// The line's current level, raw and unfiltered (a LEVEL, not an edge —
    /// per-consumer edge detection lives in ButtonEdge, above the seam; header
    /// note). What "true" means physically belongs to the mechanism that owns
    /// the line.
    [[nodiscard]] virtual bool state() const = 0;
};

}  // namespace shulib::hal
