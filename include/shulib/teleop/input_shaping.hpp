#pragma once
//
// Input shaping — what a driver's raw stick deflection becomes before it is a
// command (chunk T2). Two pure functions, one composition, and the profile that
// names their settings.
//
// PROPERTY 1 — THE DEADBAND MUST BE CONTINUOUS, and that is the whole reason
// this file exists. The naive deadband (`|x| < d ? 0 : x`, stick_mapping.hpp's
// `deadbanded()`) JUMPS from 0 to d the instant the stick crosses the
// threshold: the robot is stopped, the driver eases the stick out, and 5% of
// full authority arrives at once. That discontinuity is precisely the "twitchy,
// I can't place it" complaint, and it is a defect rather than a taste. The fix
// is not a smoothing function but a RESCALE of the surviving range:
//
//     continuousDeadband(x, d) = sign(x) * (|x| - d) / (1 - d)
//
// which leaves zero at exactly 0 at the threshold, is monotonic everywhere, and
// still reaches EXACTLY ±1 at full deflection — a driver must be able to ask
// for everything. A cubic blend was the alternative and was rejected: more
// parameters for no felt gain, and harder to reason about at the ends, which is
// where the two properties that matter live.
//
// PROPERTY 2 — RESPONSE CURVES MUST BE MONOTONIC. A non-monotonic curve means
// pushing the stick further can slow the robot down, which is not a feel
// preference, it is a control system that lies. Exactly one curve shape is
// offered, `expo`:
//
//     expoCurve(x, e) = x * (e*x^2 + (1 - e))
//
// monotonic for every e in [0, 1] (derivative 3*e*x^2 + (1 - e) >= 0), identity
// at e = 0, through (0,0) and (±1,±1) for every e. It is what every RC
// transmitter calls "expo", so a driver's intuition transfers with the word. A
// free polynomial was the alternative and was rejected: a non-monotonic one is
// one typo away.
//
// OFF IS OFF, BIT FOR BIT. `d = 0` and `e = 0` return their argument itself,
// not a value that rounds to it — early returns, no arithmetic. That is what
// lets the shipped default be compared against today's mapping at the ends and
// come out identical rather than nearly identical (test/input_shaping_test.cpp
// compares bits).
//
// THE SIGN OF ZERO IS PART OF THE CONTRACT. Inside the deadband the result is
// POSITIVE zero — the literal `0.0`, never `-x*0` and never a negated zero —
// because stick_mapping.hpp pins negate-then-deadband so that a centred stick
// yields +0.0, and a later bit-identity pin that failed on the sign of a zero
// would fail for a reason nobody could see.
//
// EVERY NUMBER HERE IS INVENTED. PROVISIONAL (A4: HA-112). No driver has felt
// any of it; the defaults are chosen to reproduce today's mapping as closely as
// a continuous deadband allows, which is a compatibility argument, not a feel
// claim. `DriverProfile` exists so that tuning is a NAMED profile a program
// selects rather than a constant somebody edits.
//
// TOTAL, PURE, constexpr, PROS-free, allocation-free. No preconditions and no
// asserts: a teleop tick must not be able to abort the program, so a NaN axis
// shapes to a NaN and is rejected where it always was, at Chassis::drive
// (chassis.hpp) and at the motor adapter. A nonsensical SETTING (a deadband
// outside [0, 1), a rate below zero) disables its feature rather than producing
// nonsense — see continuousDeadband() — and `AxisShaping::valid()` /
// `DriverProfile::valid()` are how a caller finds out before a driver does.

#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/slew_limiter.hpp"
#include "shulib/teleop/stick_mapping.hpp"

namespace shulib::teleop {

/// The feel layer's default deadband, as a fraction of full deflection. Deliberately the SAME
/// number as `kStickDeadband`, the value the library has shipped since R1a, so that turning the
/// shaping on changes the SHAPE of the response and not the size of the dead zone. INVENTED —
/// PROVISIONAL (A4: HA-112).
inline constexpr double kDefaultDeadband = kStickDeadband;

/// The feel layer's default expo: 0, meaning NO curve — the identity, bit for bit. Chosen so
/// the shipped default is as close to today's mapping as a continuous deadband allows; a curve
/// nobody has driven is not a default, it is a guess with a number on it.
/// INVENTED — PROVISIONAL (A4: HA-112).
inline constexpr double kDefaultExpo = 0.0;

/// A deadband that rescales instead of cutting: the axis is 0 inside the dead zone and the
/// surviving range is stretched back over the full [-1, 1], so the output LEAVES ZERO SMOOTHLY
/// and still reaches ±1 at full deflection.
///
///     |axis| <= deadband          -> +0.0   (positive zero, always)
///     otherwise                   -> sign(axis) * (|axis| - deadband) / (1 - deadband)
///
/// Continuous at ±deadband (the value there is exactly zero), monotonic, odd, through (0,0) and
/// (±1, ±1) exactly. Compare `deadbanded()` (stick_mapping.hpp), which jumps from 0 to the
/// threshold value and is kept only for the R1a bit-identity pin.
///
/// The comparison is `<=`, so the threshold value ITSELF is inside the dead zone — where
/// `deadbanded()` lets it pass. Both spellings put exactly zero at ±deadband; `<=` is chosen
/// because the open form reaches the threshold through the rescale branch and returns NEGATIVE
/// zero at `-deadband` (a negated `0.0`), and this library has one rule about the sign of zero
/// at the stick mapping and keeps it.
///
/// `deadband` OUTSIDE (0, 1) — including the default-ish 0, a negative value, a NaN, and any
/// value at or above 1 — disables the deadband and returns `axis` itself, BIT FOR BIT (sign of
/// zero and NaN payload included). Three cases collapse into one deliberately: 0 must be a true
/// no-op so "off" is off; 1 or more would divide by zero or by a negative and hand the driver a
/// silently inverted axis; and a NaN setting has no sensible interpretation at all. A pure
/// constexpr function has nowhere to report an error to, so it does the least destructive thing
/// and `AxisShaping::valid()` is where the nonsense is meant to be caught.
[[nodiscard]] constexpr double continuousDeadband(double axis, double deadband) noexcept {
    if (!(deadband > 0.0 && deadband < 1.0)) {
        return axis;  // OFF (or nonsensical): pass through, bit for bit
    }
    const double magnitude = axis < 0.0 ? -axis : axis;  // not std::abs: that is not constexpr here
    if (magnitude <= deadband) {
        return 0.0;  // POSITIVE zero, literally — see the file header
    }
    const double scaled = (magnitude - deadband) / (1.0 - deadband);
    return axis < 0.0 ? -scaled : scaled;
}

/// The one response curve the library offers: `axis * (expo*axis^2 + (1 - expo))`.
///
/// Monotonic for every `expo` in [0, 1] (derivative `3*expo*axis^2 + (1 - expo) >= 0`), odd,
/// and through (0,0) and (±1, ±1) EXACTLY for every expo — so raising expo softens the middle
/// of the stick without ever costing the driver top speed. Larger expo = flatter near centre =
/// finer control there.
///
/// `expo == 0` returns `axis` itself, bit for bit — an early return rather than a multiply by
/// one, so that an infinite axis stays infinite instead of becoming a NaN through `0 * inf`,
/// and so that "no curve" is structurally a no-op.
///
/// `expo` outside [0, 1] is NOT rejected (this function is total and has nowhere to report to),
/// and outside that range the monotonicity guarantee is void: above 1 the curve dips below zero
/// near centre, below 0 it overshoots ±1. `AxisShaping::valid()` is the guard.
[[nodiscard]] constexpr double expoCurve(double axis, double expo) noexcept {
    if (expo == 0.0) {
        return axis;  // OFF: the identity, bit for bit
    }
    return axis * (expo * axis * axis + (1.0 - expo));
}

/// One axis's feel settings. Per axis rather than global because forward, strafe and yaw are
/// not the same job: yaw is where a driver most often wants a softer centre, and strafe on a
/// holonomic robot is the axis most often left raw.
///
/// Both values are INVENTED — PROVISIONAL (A4: HA-112). Nothing here has been driven.
struct AxisShaping {
    /// Dead zone as a fraction of full deflection, in [0, 1). 0 disables the deadband.
    /// Defaults to `kDefaultDeadband`.
    double deadband = kDefaultDeadband;
    /// Curve strength in [0, 1]. 0 is the identity (no curve); 1 is pure cubic.
    /// Defaults to `kDefaultExpo`.
    double expo = kDefaultExpo;

    /// True when both settings are in the ranges the shaping functions guarantee their
    /// properties over: `deadband` in [0, 1) and `expo` in [0, 1], both finite. A program that
    /// builds a profile from a file or a menu should check this and say so LOUDLY — the shaping
    /// functions themselves stay total and will quietly disable a setting they cannot honour,
    /// which is the right behaviour in a teleop tick and the wrong one at configuration time.
    [[nodiscard]] constexpr bool valid() const noexcept {
        return deadband >= 0.0 && deadband < 1.0 && expo >= 0.0 && expo <= 1.0;
    }
};

/// The whole shaping of one axis: deadband FIRST, then the curve —
/// `expoCurve(continuousDeadband(axis, s.deadband), s.expo)`.
///
/// The order is load-bearing and is not symmetric. Deadband-first means the curve is applied to
/// the RESCALED range, so the curve's own (0,0) and (±1,±1) anchors line up with "the stick just
/// left the dead zone" and "the stick is at the rail". Curve-first would compress the dead zone
/// itself and make the threshold's position depend on the expo setting, which is the sort of
/// coupling that makes tuning feel like guessing.
///
/// A centred stick yields POSITIVE zero: the deadband returns `0.0` and the curve passes zero
/// through with its sign intact.
[[nodiscard]] constexpr double shapeAxis(double axis, const AxisShaping& shaping) noexcept {
    return expoCurve(continuousDeadband(axis, shaping.deadband), shaping.expo);
}

/// One driver's whole feel, as a named value a program selects — "per-driver tuning" is a
/// profile, never a constant somebody edits between matches. Copyable, comparable by hand,
/// and cheap enough to keep several in a program.
///
/// THE DEFAULT-CONSTRUCTED PROFILE IS WHAT THE LIBRARY SHIPS, and it is chosen to surprise a
/// driver who drove yesterday as little as possible: deadband 0.05 (the number already in the
/// tree), expo 0 (no curve at all), slew OFF (no added lag), mode ArcadeBody (today's mapping).
/// Against `mapSticks()` it is identical at centre and at both rails, and inside that range it
/// asks for slightly LESS than the old mapping did — the rescale's cost, bounded by the
/// deadband itself and pinned in test/input_shaping_test.cpp.
///
/// EVERY FIELD IS INVENTED — PROVISIONAL (A4: HA-112). No driver has felt this.
struct DriverProfile {
    /// Shaping for the +X (forward) axis.
    AxisShaping forward{};
    /// Shaping for the +Y (strafe-left) axis. Applied on a tank drivetrain too, where the axis
    /// is simply always zero; the authority question a strafe raises belongs to the kinematics
    /// (C3's strafe clamp), not here.
    AxisShaping left{};
    /// Shaping for the CCW-yaw axis.
    AxisShaping yaw{};
    /// The rate limit, applied to all three axes alike. OFF by default. ONE rate rather than
    /// three because all three axes are dimensionless request fractions in [-1, 1], so one
    /// number is meaningful across them; SlewConfig is a struct so per-axis rates can be added
    /// later without changing a caller's spelling.
    SlewConfig slew{};
    /// Which way the sticks are wired (drive_modes.hpp). ArcadeBody is today's mapping.
    DriveMode mode = DriveMode::ArcadeBody;

    /// True when all three axes' settings are in range. Says nothing about whether the profile
    /// feels good — no code in this library can know that, and none of it claims to.
    [[nodiscard]] constexpr bool valid() const noexcept {
        return forward.valid() && left.valid() && yaw.valid();
    }
};

/// A whole DriveRequest through a profile's three per-axis shapings, axis by axis. The
/// step between the mode mapping and the slew limit in DriveShaper::shape()'s documented
/// order (driver_control.hpp, ruling S5), exposed on its own so a caller that wants the
/// shaping without the statefulness can have it.
[[nodiscard]] constexpr DriveRequest shapeRequest(const DriveRequest& request,
                                                  const DriverProfile& profile) noexcept {
    return DriveRequest{shapeAxis(request.forward, profile.forward),
                        shapeAxis(request.left, profile.left),
                        shapeAxis(request.yawCcw, profile.yaw)};
}

}  // namespace shulib::teleop
