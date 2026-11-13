#pragma once
//
// Field-centric driving, and the honest failure mode without which it must not
// ship (chunk T2).
//
// WHAT IT IS. In field-centric mode the driver's stick vector is read in the
// DRIVER's frame, not the robot's: "away from me" means away from the driver
// whatever way the robot happens to be facing. That is a rotation of the
// TRANSLATION by minus the robot's heading (measured from the driver's zero);
// yaw is frame-invariant and is never touched.
//
// WHY IT IS DANGEROUS, AND WHY THE FALLBACK IS THE POINT OF THE FILE. The
// rotation uses the ESTIMATED heading. A3 proved the heading drifts. So when
// heading quality degrades, the driver's controls silently rotate away from
// reality WHILE EVERYTHING STILL APPEARS TO WORK — the robot moves, the sticks
// respond, and the mapping between them is wrong by an angle nobody announced.
// Build-order T2: "do not ship field-centric without" a documented fallback to
// robot-centric, a driver-accessible re-zero, and a visible indication. All
// three are here.
//
// THE FALLBACK IS A LATCH, NOT A FLICKER. When the caller says the heading is
// not trusted, this returns the BODY request unrotated, reports `fellBack` with
// a reason a panel can show, and STAYS fallen back until the driver re-zeroes —
// even after trust returns. Controls that silently flip between two frames
// mid-match are worse than controls that stay in one: the driver can adapt to a
// robot that is robot-centric, and cannot adapt to a robot that changes its
// mind. Clearing the latch is therefore a DRIVER action, `rezero()`, and never
// an automatic one.
//
// NOTHING HERE TOUCHES THE IMU OR THE ESTIMATE (HA-05). The driver's zero is
// this object's own offset, subtracted from the heading it is handed. Re-zeroing
// by taring the IMU or resetting the localizer was the obvious alternative and
// is rejected outright: a tare under a live boot-heading offset silently
// corrupts the estimate, and a driver pressing a comfort button must not be able
// to break autonomous localization.
//
// THE HEADING IS AN ARGUMENT, NOT A READING. This class never sees a Chassis, a
// Localizer or an IImu — it takes a number and a bool. That keeps it testable
// without any of them, keeps the boot window out of its vocabulary (the caller
// already knows whether the estimate is live), and keeps this file out of the
// dependency graph of everything it would otherwise have to include.
//
// THE RESULT IS STILL A BODY-FRAME REQUEST, so the caller commands
// `Frame::Body`. Routing field-centric through `Chassis::drive(…, Frame::Field)`
// was the alternative and was rejected: the facade's field frame is correct, but
// going through it means the fallback needs the facade's cooperation, and the
// facade's boot-window rule silently commands zero volts. Rotating here and
// always commanding Body makes the fallback one branch and the boot window a
// non-event. (The facade's field verb stays frozen and untouched; autonomous
// callers may still use it.)
//
// PROS-free, allocation-free, no std::string, no std::function, total (no
// preconditions): a teleop tick must not be able to abort the program.

#include <cmath>
#include <cstdint>

#include "shulib/teleop/stick_mapping.hpp"

namespace shulib::teleop {

/// Why field-centric is or is not rotating the driver's sticks right now — an enum rather than
/// a string because this runs on the V5 and a teleop tick allocates nothing.
enum class FieldCentricReason : std::uint8_t {
    /// Rotating normally: the heading is trusted and the driver has a zero.
    Active,
    /// The caller reported the heading as not trustworthy THIS tick (the boot window before the
    /// estimate is live, a latched IMU_LOST fault, or a heading that is not even a finite
    /// number). The request was passed through unrotated.
    HeadingNotTrusted,
    /// The heading is trustworthy again, but the fallback has LATCHED: the controls stay
    /// robot-centric until the driver re-zeroes. This is the state a panel most needs to show,
    /// because it is the one where nothing looks wrong.
    LatchedUntilRezero,
    /// Field-centric is not selected — the profile's mode is a body-frame one. Reported so a
    /// panel can distinguish "off" from "fell back", which look identical at the drivetrain.
    NotSelected,
};

/// What a panel shows about field-centric — the third of the three things this mode may not
/// ship without (a documented fallback, a driver re-zero, and a VISIBLE INDICATION). Three
/// plain values, cheap to copy every tick.
struct FieldCentricState {
    /// True only when the translation was actually rotated this tick.
    bool active = false;
    /// True while the fallback latch is set — the controls are robot-centric and will stay that
    /// way until `rezero()`. Distinct from `!active`, which is also true when the mode simply
    /// is not selected.
    bool fellBack = false;
    /// Which of the four situations above produced this tick's behaviour.
    FieldCentricReason reason = FieldCentricReason::NotSelected;
};

/// The rotation and the latch: the driver's zero offset, and whether the fallback is engaged.
///
/// Two doubles and a bool of state. One instance per driver (VEX U runs two), held by the
/// DriveShaper that owns that driver's feel.
class FieldCentric {
public:
    /// A field-centric mapper with zero at heading 0 and the fallback latch CLEARED. The latch
    /// starts clear on purpose: it is a record of something that went wrong, and nothing has
    /// gone wrong yet. The first untrusted tick sets it.
    constexpr FieldCentric() noexcept = default;

    /// "The way I am facing now is away from me" — the driver's button. Records `headingRadians`
    /// as the new zero and CLEARS the fallback latch, which is the only thing that clears it.
    ///
    /// It does NOT check whether that heading is trustworthy, and it takes no trust argument:
    /// the driver is asserting a frame, and the very next `request()` re-latches the fallback if
    /// the heading is still untrusted. Refusing the press outright would leave the driver
    /// pressing a dead button with no way to tell why; re-latching one tick later says the same
    /// thing through the indication that is already on the panel.
    ///
    /// Nothing is tared and no estimate moves (HA-05) — the zero lives here.
    ///
    /// ONE EXCEPTION (G1 Part 2a, found while wiring this to a button): a heading that is NOT A
    /// FINITE NUMBER is not stored. It would become a NaN zero, and every later trusted tick
    /// would rotate by `heading − NaN` — a NaN request that `Chassis::drive` rejects by
    /// precondition, long after the press, with nothing on the panel to explain it. Instead the
    /// previous zero is kept and the fallback latch is SET, so the controls stay robot-centric
    /// and the panel says `LatchedUntilRezero` until the driver presses again on a real heading.
    /// (`request()` already treats a non-finite heading as untrusted; this closes the one path
    /// that let such a heading outlive its tick.)
    constexpr void rezero(double headingRadians) noexcept {
        // `x - x == 0` is false exactly for NaN and ±infinity — std::isfinite is not constexpr
        // in C++20, and this function is.
        if (!(headingRadians - headingRadians == 0.0)) {
            fellBack_ = true;
            return;
        }
        zero_ = headingRadians;
        fellBack_ = false;
    }

    /// The driver's zero, in radians, as last set by `rezero()` (0 until then).
    [[nodiscard]] constexpr double zeroRadians() const noexcept { return zero_; }

    /// True while the fallback latch is engaged. Cleared only by `rezero()`.
    [[nodiscard]] constexpr bool fellBack() const noexcept { return fellBack_; }

    /// Forget everything: zero back to 0 and the latch cleared. For a program handing control
    /// between drivers or restarting teleop, not for anything a match does.
    constexpr void reset() noexcept {
        zero_ = 0.0;
        fellBack_ = false;
    }

    /// What `request()` returns: the body-frame request to command, and what to show about it.
    struct Result {
        /// The request to hand to the drivetrain, always in the ROBOT's frame — rotated when
        /// field-centric is active, passed through untouched when it is not.
        DriveRequest request{};
        /// The indication (see FieldCentricState).
        FieldCentricState state{};
    };

    /// Rotate a SHAPED body-frame request out of the driver's frame into the robot's, or fall
    /// back honestly.
    ///
    /// `shapedBody` is the request as the mode mapping and the shaping produced it: its
    /// `forward`/`left` pair is read as the driver's stick vector in the DRIVER's frame, and
    /// `yawCcw` is frame-invariant and is copied through untouched.
    ///
    /// `headingRadians` is the robot's ESTIMATED heading, CCW-positive, in the same frame the
    /// driver's zero was captured in. `headingTrusted` is the caller's verdict on it — the
    /// estimate is live (past the boot window) and no IMU_LOST fault is latched.
    ///
    /// Rotates by MINUS `(heading - zero)`: a robot turned CCW from the driver's zero must move
    /// CW in its own frame to keep going the way the driver pushed. This is the same R(-θ) the
    /// frozen `math::fieldToRobot` applies, re-derived here on plain doubles so that this file
    /// stays free of Angle's non-finite precondition and of the velocity types.
    ///
    /// FALLS BACK when `headingTrusted` is false OR the heading is not a finite number: the
    /// request is returned unrotated, `fellBack` is set, and it LATCHES — subsequent trusted
    /// ticks keep falling back, reporting `LatchedUntilRezero`, until the driver re-zeroes. A
    /// non-finite heading is treated as untrusted rather than propagated because "trusted" is
    /// exactly the claim such a number fails; propagating it would turn the whole request into
    /// NaN and surface as a precondition abort inside `Chassis::drive` with nothing on the panel
    /// to explain it.
    [[nodiscard]] Result request(const DriveRequest& shapedBody, double headingRadians,
                                 bool headingTrusted) noexcept {
        if (!headingTrusted || !std::isfinite(headingRadians)) {
            fellBack_ = true;
            return Result{shapedBody, FieldCentricState{.active = false,
                                                        .fellBack = true,
                                                        .reason = FieldCentricReason::HeadingNotTrusted}};
        }
        if (fellBack_) {
            return Result{shapedBody, FieldCentricState{.active = false,
                                                        .fellBack = true,
                                                        .reason = FieldCentricReason::LatchedUntilRezero}};
        }
        // R(-phi) on the translation; phi is wrapped first so a caller that hands over an
        // unwrapped accumulated heading loses no precision in cos/sin.
        const double phi = std::remainder(headingRadians - zero_, 2.0 * kPi);
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        const double fx = shapedBody.forward;
        const double fy = shapedBody.left;
        return Result{DriveRequest{c * fx + s * fy, -s * fx + c * fy, shapedBody.yawCcw},
                      FieldCentricState{.active = true,
                                        .fellBack = false,
                                        .reason = FieldCentricReason::Active}};
    }

    /// The indication with no tick run — what a panel shows when the mode is not selected, or
    /// between ticks. `active` is never true here: it is a property of a tick, not of the object.
    [[nodiscard]] constexpr FieldCentricState state() const noexcept {
        return FieldCentricState{.active = false,
                                 .fellBack = fellBack_,
                                 .reason = fellBack_ ? FieldCentricReason::LatchedUntilRezero
                                                     : FieldCentricReason::NotSelected};
    }

private:
    /// Pi. The same value math::Angle::kPi carries, restated rather than included so this header
    /// depends on nothing but the request vocabulary; the wrap below is the only use.
    static constexpr double kPi = 3.14159265358979323846;

    double zero_ = 0.0;    ///< the driver's zero heading, radians
    bool fellBack_ = false;  ///< the fallback latch — cleared only by rezero()
};

}  // namespace shulib::teleop
