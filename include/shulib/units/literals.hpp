#pragma once
//
// User-defined literals for shulib units. Each converts to CANONICAL units at
// the point of writing, so the rest of the code never sees a raw unit again:
//   distance = 24_in;  field = 1_tile;  dt = 20_ms;  v = 12_volt;
//
// `_ms` divides by 1000 -> seconds, enforcing the "internal seconds, always"
// decision at the literal boundary. `_deg`/`_rad` build an (auto-wrapped)
// shulib::math::Angle, NOT a bare Quantity (decision §13 #3).
//
// Each literal has two forms: a floating (long double) and an integer
// (unsigned long long), so both `24_in` and `24.0_in` compile. The narrowing
// to double is made explicit (the build is -Wconversion -Werror).

#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::units::literals {

// length
/// Inches — THE canonical length unit, so `24_in` stores exactly 24.0 and nothing converts.
constexpr Length operator""_in(long double v) noexcept { return Length{static_cast<double>(v)}; }
/// Integer spelling of `_in`, and NOT redundant with the overload above: C++ offers an integer
/// literal only to the `unsigned long long` form, so without this one `24_in` would not compile —
/// it is never quietly converted to the `long double` operator. That the parameter is unsigned is
/// also why there is no negative literal: `-24_in` is unary minus applied to `Length{24.0}`, which
/// works only because Length is a signed displacement.
constexpr Length operator""_in(unsigned long long v) noexcept { return Length{static_cast<double>(v)}; }
/// Foam field tiles, converted HERE: one tile is 24 inches, so `1_tile` is Length{24.0}. The
/// result is an ordinary Length — nothing downstream ever knows a tile was mentioned.
constexpr Length operator""_tile(long double v) noexcept { return Length{static_cast<double>(v) * 24.0}; }
/// Integer spelling of `_tile`; `2_tile` is 48 inches.
constexpr Length operator""_tile(unsigned long long v) noexcept { return Length{static_cast<double>(v) * 24.0}; }

// time (canonical = seconds)
/// Seconds — the canonical time unit, so `2_s` stores exactly 2.0.
constexpr Time operator""_s(long double v) noexcept { return Time{static_cast<double>(v)}; }
/// Integer spelling of `_s`, which C++ forces to be a SEPARATE overload — a literal written with
/// no decimal point is offered only to the `unsigned long long` form and never converted to the
/// `long double` one above. Nothing is computed on the way in, so `2_s` is exactly 2.0, and the
/// integer-to-double conversion stays exact far past any duration a match can run.
constexpr Time operator""_s(unsigned long long v) noexcept { return Time{static_cast<double>(v)}; }
/// Milliseconds, divided by 1000 AT THE LITERAL: `20_ms` is Time{0.02}. This is where the
/// "internal seconds, always" rule is enforced — no millisecond value reaches a gain or a dt.
constexpr Time operator""_ms(long double v) noexcept { return Time{static_cast<double>(v) / 1000.0}; }
/// Integer spelling of `_ms` — the form a loop period is normally written in (`10_ms`).
constexpr Time operator""_ms(unsigned long long v) noexcept { return Time{static_cast<double>(v) / 1000.0}; }

// voltage
/// Volts — canonical, so `12_volt` stores 12.0. It does NOT clamp: the ±12 V ceiling is
/// applied by IMotor::setVoltage (hal::kMaxMotorVoltage), never by writing the literal.
constexpr Voltage operator""_volt(long double v) noexcept { return Voltage{static_cast<double>(v)}; }
/// Integer spelling of `_volt` — the form the rail voltage is normally written in — and required
/// as its own overload, since an integer literal is never offered to the `long double` operator
/// above. It does NOT clamp either: `13_volt` builds Voltage{13.0} and only IMotor::setVoltage
/// applies the ±kMaxMotorVoltage ceiling. Reverse is `-12_volt`, which is unary minus on
/// Voltage{12.0} rather than a negative literal — the parameter here is unsigned.
constexpr Voltage operator""_volt(unsigned long long v) noexcept { return Voltage{static_cast<double>(v)}; }

// angle -> a wrapping Angle (decision §13 #3), so 90_deg is a heading, not a scalar
/// Degrees as a WRAPPING math::Angle rather than a bare Quantity, so `90_deg` is a heading and
/// not a scalar: it normalizes to (-π, π] on construction, which means `359_deg` stores -1°.
/// Degrees exist only at this boundary — the value inside is radians.
inline ::shulib::math::Angle operator""_deg(long double v) {
    return ::shulib::math::Angle::degrees(static_cast<double>(v));
}
/// Integer spelling of `_deg`. The angle literals are the only ones here that are NOT
/// constexpr — Angle's factories run a runtime finiteness check — so `90_deg` cannot
/// initialize a `constexpr` variable the way `24_in` can.
inline ::shulib::math::Angle operator""_deg(unsigned long long v) {
    return ::shulib::math::Angle::degrees(static_cast<double>(v));
}
/// Radians as a wrapping math::Angle, with no conversion (`3_rad` stores 3.0) but the SAME
/// normalization as `_deg`, so `7_rad` stores 7 − 2π ≈ 0.717. There is no literal here that
/// builds an un-wrapped heading; use units::AngleDim for a rate or an accumulated total.
inline ::shulib::math::Angle operator""_rad(long double v) {
    return ::shulib::math::Angle::radians(static_cast<double>(v));
}
/// Integer spelling of `_rad`. Same type as `_deg` produces, so the two are interchangeable
/// wherever an Angle is taken; also not constexpr (see `_deg`).
inline ::shulib::math::Angle operator""_rad(unsigned long long v) {
    return ::shulib::math::Angle::radians(static_cast<double>(v));
}

}  // namespace shulib::units::literals
