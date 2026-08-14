#pragma once
//
// IController — the V5 game controller behind the HAL (chunk R1a, absorbing
// Phase T's T1): the INPUT half of driver control. The frozen F6 facade already
// has the OUTPUT half — drive(ChassisSpeeds, Frame) — so this seam is what
// makes the library a one-stop shop for teleop without a second library.
//
// Canonical at the seam, like every other HAL edge (§7):
//  * axis() is normalized to [-1, 1] — the V5's raw −127…127 never reaches the
//    core (controller_conversion.hpp owns the one scale; HA-103).
//  * pressed() is a plain bool LEVEL (is it down NOW). Edge detection ("was it
//    JUST pressed") deliberately lives ABOVE the seam, in ButtonEdge below —
//    PROS's get_digital_new_press() CONSUMES the event on read, so with two
//    consumers one silently loses. The adapter binds get_digital() and NEVER
//    get_digital_new_press() (HA-104); each consumer owns its own ButtonEdge.
//  * isConnected() is a POSITIVE validity signal (true = usable), matching
//    IImu::isReady() / IGps::hasFix() polarity (imu.hpp:31-33) — a controller
//    really does drop mid-match, and the core must be able to tell that from
//    "sticks centred" (PROS reports 0 on every channel when disconnected, so
//    without this signal the two are indistinguishable).
//
// PARTNER SUPPORT is structural, not a retrofit: VEX U runs two drivers, so
// "which controller" is a CONSTRUCTION question (make two instances — the
// hal/pros adapter takes ControllerId::Master or ControllerId::Partner), never
// an interface question. Retrofitting a second driver through a single-
// controller seam is the reshape a seam exists to prevent.
//
// NOT part of the F4 freeze (that register row locked 2026-06-19 with the ten
// runtime interfaces; this seam is R1a's addition, outside it — the same
// additive-sibling shape F1 used for IDigitalOut, digital_out.hpp:22-24).
// Register row F13 records the NON-freeze out loud (D2's lesson: silence in
// that register reads as "frozen"). Freeze trigger: a second real consumer —
// T2's driver-control layer is the first.

namespace shulib::hal {

/// The four analog stick channels, named by physical position.
enum class ControllerAxis {
    LeftX,   ///< left stick, horizontal (+ = pushed right)
    LeftY,   ///< left stick, vertical (+ = pushed up)
    RightX,  ///< right stick, horizontal (+ = pushed right)
    RightY,  ///< right stick, vertical (+ = pushed up)
};

/// The twelve driver-usable buttons. The power button is deliberately absent:
/// pressing it turns the controller off, so no routine may ever bind it.
enum class ControllerButton {
    L1, L2, R1, R2,             ///< shoulder triggers
    Up, Down, Left, Right,      ///< left arrow pad
    X, B, Y, A,                 ///< right button pad
};

class IController {
public:
    virtual ~IController() = default;
    IController() = default;
    IController(const IController&) = default;
    IController(IController&&) = default;
    IController& operator=(const IController&) = default;
    IController& operator=(IController&&) = default;

    /// Current deflection of `axis`, normalized to [-1, 1] (full deflection =
    /// ±1). Reads 0.0 while disconnected — check isConnected() to tell a drop
    /// from centred sticks.
    [[nodiscard]] virtual double axis(ControllerAxis axis) const = 0;

    /// True while `button` is held down (a LEVEL, not an edge — see header:
    /// per-consumer edge detection lives in ButtonEdge, above the seam).
    [[nodiscard]] virtual bool pressed(ControllerButton button) const = 0;

    /// True while this controller is connected and its readings are live.
    /// POSITIVE polarity by convention (cf. IImu::isReady, IGps::hasFix).
    [[nodiscard]] virtual bool isConnected() const = 0;
};

/// Per-consumer rising-edge detector over IController::pressed() levels — the
/// PROS-free replacement for get_digital_new_press(), which consumes the event
/// on read so a second consumer silently misses every press (HA-104). Each
/// consumer owns its own ButtonEdge, so N consumers all see the same press.
///
/// Stateful by nature (it must remember the previous level); call update()
/// exactly once per tick per consumer.
class ButtonEdge {
public:
    /// Feed this tick's level; returns true exactly on a false→true transition.
    /// The first call after construction reports a press only if the button is
    /// already down (prev starts false) — a button held across a mode switch
    /// registers once, deliberately, rather than being swallowed.
    [[nodiscard]] bool update(bool pressedNow) noexcept {
        const bool rising = pressedNow && !prev_;
        prev_ = pressedNow;
        return rising;
    }

private:
    bool prev_ = false;
};

}  // namespace shulib::hal
