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

class IDigitalIn {
public:
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
